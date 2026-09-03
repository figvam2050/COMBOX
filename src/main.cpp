#ifdef __clang__
#undef __GNUC_MINOR__
#define __GNUC_MINOR__ 10
#endif
#include <Arduino.h>

/*
  Vision V-LFP48100 -> Felicity COM-Box hardware -> Deye CAN/Pylontech
  Target: GD32F305RCT6 on Felicity "BMS COM BOX" PCB
  Compiled with PlatformIO: board genericSTM32F103RC, framework arduino

  PIN ASSIGNMENT (Locked per PORT_MAPPING.md):
  - PA9:  BMS RS485 TX (USART1/USART0 TX, AF Push-Pull 50MHz)
  - PA10: BMS RS485 RX (USART1/USART0 RX, Floating Input)
  - PB8:  PCS CAN RX   (CAN1/CAN0 RX, Partial Remap, Floating Input)
  - PB9:  PCS CAN TX   (CAN1/CAN0 TX, Partial Remap, AF Push-Pull 50MHz)
  - PB0:  LED3 Running (Active High, Heartbeat)
  - PB1:  LED4 BMS-485 Connect (Active High, Activity Blink)
  - PC5:  LED6 PCS-CAN Connect (Active High, Activity Blink)
  - PC13: LED5 PCS-485 Connect (Active High, Inactive/LOW)
  - PC3:  Factory control line -> LOW (Confirmed)
  - PC4:  Factory control line -> HIGH (Confirmed)
  - PB12..PB15: DIP switches 1..4 (Active Low, Input Pullup)
  - PA1:  NOT USED (No manual DE/RE; board has automatic hardware direction)
  - PC10/PC11, PC12/PD2, PD0/PD1: NOT USED
*/

// -----------------------------------------------------------------------------
// HARDWARE PINS (CONFIRMED BY PORT_MAPPING.md)
// -----------------------------------------------------------------------------
#define PIN_LED_RUN         PB0   // LED3: Running
#define PIN_LED_BMS_485     PB1   // LED4: BMS-485 Connect
#define PIN_LED_PCS_CAN     PC5   // LED6: PCS-CAN Connect
#define PIN_LED_PCS_485     PC13  // LED5: PCS-485 Connect

#define PIN_FACTORY_CTRL_LOW   PC3  // Factory fixed level: LOW
#define PIN_FACTORY_CTRL_HIGH  PC4  // Factory fixed level: HIGH

// USART1 base in STM32 CMSIS (0x40013800 = USART0 in GD32 SPL)
#define COMBOX_BMS_USART ((USART_TypeDef *)0x40013800UL)

// CAN partial remap (PB8 RX, PB9 TX): AFIO_MAPR bit 14 (CAN_REMAP = 0b10)
#define COMBOX_CAN_REMAP 0x00004000UL

// -----------------------------------------------------------------------------
// PROTOCOL & BATTERY SETTINGS
// -----------------------------------------------------------------------------
#define MAX_BATTERIES 4

// Factory default / Vision single pack address confirmed by live BMS_TOOLS capture:
// "10 03 00 00 00 27 06 91" -> "10 03 4E 13 3F ..."
static constexpr uint8_t MASTER_ADDRESS = 0x10;
static const uint8_t moduleAddresses[MAX_BATTERIES] = {0x01, 0x02, 0x03, 0x04};

// Set to true to poll only MASTER_ADDRESS (0x10).
// Set to false to scan addresses 0x01..0x04 sequentially.
static constexpr bool USE_MASTER_ONLY = true;

// Vision register map request: 39 holding registers (0x0000 .. 0x0026)
static constexpr uint16_t BMS_START_REGISTER = 0x0000;
static constexpr uint16_t BMS_REGISTER_COUNT = 0x0027; // 39 registers
static constexpr uint16_t BMS_RESPONSE_LEN   = 83;     // 1(addr) + 1(func) + 1(len=78) + 78(data) + 2(crc)

// Timing parameters
static constexpr uint32_t BMS_POLL_INTERVAL_MS   = 300;   // Poll BMS every 300ms
static constexpr uint32_t BMS_RESPONSE_TIMEOUT_MS = 150;  // 150ms timeout waiting for response
static constexpr uint32_t BMS_STALE_MS           = 5000;  // Data considered stale after 5s
static constexpr uint32_t CAN_PERIOD_MS          = 1000;  // Deye heartbeat every 1s
static constexpr uint32_t LED_PULSE_DURATION_MS  = 40;    // 40ms pulse for clear visibility

// Conservative system safety limits for Deye low-voltage 48V/51.2V
static constexpr uint16_t SYSTEM_MAX_CHARGE_A    = 100;
static constexpr uint16_t SYSTEM_MAX_DISCHARGE_A = 120;

// Voltage limits in 0.1V units for Pylontech CAN frame 0x351
// 54.0V charge cutoff, 43.0V discharge cutoff (matches 15S/16S LFP safety curve)
static constexpr uint16_t CHARGE_VOLTAGE_LIMIT_01V    = 540; // 54.0 V
static constexpr uint16_t DISCHARGE_VOLTAGE_LIMIT_01V = 430; // 43.0 V

// -----------------------------------------------------------------------------
// VISION PROTECTION BIT MASKS
// -----------------------------------------------------------------------------
static constexpr uint16_t PROT_PACK_OV       = 0x0001;
static constexpr uint16_t PROT_CELL_OV       = 0x0002;
static constexpr uint16_t PROT_PACK_UV       = 0x0004;
static constexpr uint16_t PROT_CELL_UV       = 0x0008;
static constexpr uint16_t PROT_CHARGE_OC     = 0x0010;
static constexpr uint16_t PROT_DISCHARGE_OC  = 0x0020;
static constexpr uint16_t PROT_ENV_TEMP      = 0x0040;
static constexpr uint16_t PROT_MOS_TEMP      = 0x0080;
static constexpr uint16_t PROT_CHARGE_OT     = 0x0100;
static constexpr uint16_t PROT_DISCHARGE_OT  = 0x0200;
static constexpr uint16_t PROT_CHARGE_UT     = 0x0400;
static constexpr uint16_t PROT_DISCHARGE_UT  = 0x0800;
static constexpr uint16_t PROT_LOW_CAPACITY  = 0x1000;
static constexpr uint16_t PROT_DISCHARGE_SC  = 0x2000;

// -----------------------------------------------------------------------------
// TELEMETRY STRUCTURE
// -----------------------------------------------------------------------------
struct BatteryTelemetry {
  uint8_t address = 0;

  uint16_t totalVoltageRaw = 0; // 10 mV units (e.g. 4927 = 49.27 V)
  int16_t currentRaw = 0;       // 10 mA units (Discharge > 0, Charge < 0 per Vision doc)
  uint16_t cellVoltages[16] = {0};

  int16_t tempPCB = 0;          // °C
  int16_t tempAvg = 0;          // °C
  int16_t tempMax = 0;          // °C

  uint16_t remainCapRaw = 0;
  uint16_t maxChargeCurrentRaw = 0;
  uint16_t soh = 0;             // % (0..100)
  uint16_t soc = 0;             // % (0..100)

  uint16_t status = 0;
  uint16_t warning = 0;
  uint16_t protection = 0;

  bool valid = false;
  uint32_t lastSeen = 0;
};

BatteryTelemetry bmsData[MAX_BATTERIES];

// Non-blocking LED pulse timers
uint32_t ledBmsPulseUntil = 0;
uint32_t ledCanPulseUntil = 0;

// -----------------------------------------------------------------------------
// DIP SWITCH READER (PB12..PB15, Active Low)
// -----------------------------------------------------------------------------
uint8_t readFactoryDipAddress() {
  const uint32_t masks[] = {1UL << 15, 1UL << 14, 1UL << 13, 1UL << 12};
  const uint8_t weights[] = {8, 4, 2, 1};
  uint8_t value = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if ((GPIOB->IDR & masks[i]) == 0)
      value += weights[i];
  }
  return value;
}

// -----------------------------------------------------------------------------
// MODBUS CRC16 (Polynomial 0xA001)
// -----------------------------------------------------------------------------
uint16_t calculateCRC16(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static inline uint16_t be16(const uint8_t *p) {
  return ((uint16_t)p[0] << 8) | p[1];
}

static inline int16_t be16s(const uint8_t *p) {
  return (int16_t)be16(p);
}

static uint16_t clampU16(uint32_t v, uint16_t maxv) {
  return (v > maxv) ? maxv : (uint16_t)v;
}

// -----------------------------------------------------------------------------
// BMS RS485 UART (USART1 on APB2: PA9 TX / PA10 RX, 9600 8N1)
// -----------------------------------------------------------------------------
void initBmsUart() {
  RCC->APB2ENR |= (1UL << 14) | (1UL << 2); // USART1EN, GPIOAEN

  // PA9 TX: alternate-function push-pull, 50 MHz (MODE=11, CNF=10 -> 0xB)
  GPIOA->CRH &= ~(0xFUL << 4);
  GPIOA->CRH |= (0xBUL << 4);

  // PA10 RX: floating input (MODE=00, CNF=01 -> 0x4)
  GPIOA->CRH &= ~(0xFUL << 8);
  GPIOA->CRH |= (0x4UL << 8);

  COMBOX_BMS_USART->CR1 = 0;
  COMBOX_BMS_USART->CR2 = 0;
  COMBOX_BMS_USART->CR3 = 0;

  // Baudrate setup: APB2 clock / 9600
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  COMBOX_BMS_USART->BRR = (pclk2 + (9600U / 2U)) / 9600U;
  COMBOX_BMS_USART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void bmsUartWrite(uint8_t data) {
  while (!(COMBOX_BMS_USART->SR & USART_SR_TXE)) {
  }
  COMBOX_BMS_USART->DR = data;
}

void bmsUartWriteBuffer(const uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++)
    bmsUartWrite(data[i]);
}

bool bmsUartAvailable() {
  uint32_t sr = COMBOX_BMS_USART->SR;
  // If Overrun / Noise / Framing error occurred, clear it by reading DR
  if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
    (void)COMBOX_BMS_USART->DR;
  }
  return (sr & USART_SR_RXNE) != 0;
}

uint8_t bmsUartRead() {
  return (uint8_t)COMBOX_BMS_USART->DR;
}

void bmsUartFlush() {
  while (!(COMBOX_BMS_USART->SR & USART_SR_TC)) {
  }
}

// -----------------------------------------------------------------------------
// CAN1 (500 kbps, PB8 RX / PB9 TX partial remap)
// ISO1050 isolated CAN transceiver (no DE/RE)
// -----------------------------------------------------------------------------
bool initCAN_500k() {
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | (1UL << 3); // AFIOEN, GPIOBEN

  // CAN_RX PB8: floating input (0x4)
  GPIOB->CRH &= ~(0xFUL << 0);
  GPIOB->CRH |= (0x4UL << 0);

  // CAN_TX PB9: alternate-function push-pull, 50 MHz (0xB)
  GPIOB->CRH &= ~(0xFUL << 4);
  GPIOB->CRH |= (0xBUL << 4);

  // Remap CAN1 to PB8/PB9 (Partial remap: bits 14:13 = 10 -> 0x00004000)
  AFIO->MAPR = (AFIO->MAPR & ~0x00006000UL) | COMBOX_CAN_REMAP;

  CAN1->MCR &= ~CAN_MCR_SLEEP;
  CAN1->MCR |= CAN_MCR_INRQ;

  uint32_t timeout = millis() + 100;
  while ((CAN1->MSR & CAN_MSR_INAK) == 0) {
    if ((int32_t)(millis() - timeout) >= 0)
      return false;
  }

  // APB1 = 32 MHz with standard Arduino STM32F1 HSI PLL (64MHz SYSCLK / 2 = 32MHz APB1)
  // 32 MHz / (Prescaler 4 * 16 TQ) = 500 kbit/s
  // SJW = 2 TQ (1U << 24)
  // TS2 = 4 TQ (3U << 20)
  // TS1 = 11 TQ (10U << 16)
  // BRP = 4 (prescaler = 3U + 1)
  CAN1->BTR = (1U << 24) | (3U << 20) | (10U << 16) | 3U;

  // Auto bus-off recovery enabled, automatic retransmission enabled
  CAN1->MCR |= CAN_MCR_ABOM;
  CAN1->MCR &= ~CAN_MCR_NART;

  // Accept all filter setup
  CAN1->FMR |= CAN_FMR_FINIT;
  CAN1->FA1R &= ~CAN_FA1R_FACT0;
  CAN1->FM1R &= ~CAN_FM1R_FBM0;
  CAN1->FS1R |= CAN_FS1R_FSC0;
  CAN1->FFA1R &= ~CAN_FFA1R_FFA0;
  CAN1->sFilterRegister[0].FR1 = 0;
  CAN1->sFilterRegister[0].FR2 = 0;
  CAN1->FA1R |= CAN_FA1R_FACT0;
  CAN1->FMR &= ~CAN_FMR_FINIT;

  CAN1->MCR &= ~CAN_MCR_INRQ;
  timeout = millis() + 100;
  while ((CAN1->MSR & CAN_MSR_INAK) != 0) {
    if ((int32_t)(millis() - timeout) >= 0)
      return false;
  }

  return true;
}

// Transmit CAN frame with timeout and transmission abort to prevent mailbox lockup
bool sendCANMessage(uint16_t id, const uint8_t *data, uint8_t len,
                    uint32_t timeoutUs = 5000) {
  if (len > 8)
    return false;

  uint32_t start = micros();
  uint8_t mailbox = 0xFF;

  while ((micros() - start) < timeoutUs) {
    if (CAN1->TSR & CAN_TSR_TME0) {
      mailbox = 0;
      break;
    }
    if (CAN1->TSR & CAN_TSR_TME1) {
      mailbox = 1;
      break;
    }
    if (CAN1->TSR & CAN_TSR_TME2) {
      mailbox = 2;
      break;
    }
  }
  if (mailbox == 0xFF)
    return false;

  CAN_TxMailBox_TypeDef &mb = CAN1->sTxMailBox[mailbox];

  mb.TIR = ((uint32_t)(id & 0x7FF) << 21);
  mb.TDTR = (uint32_t)(len & 0x0F);

  uint32_t low = 0, high = 0;
  for (uint8_t i = 0; i < 4 && i < len; i++)
    low |= ((uint32_t)data[i] << (8U * i));
  for (uint8_t i = 4; i < 8 && i < len; i++)
    high |= ((uint32_t)data[i] << (8U * (i - 4U)));

  mb.TDLR = low;
  mb.TDHR = high;
  mb.TIR |= CAN_TI0R_TXRQ;

  // Wait for transmission completion or timeout
  start = micros();
  uint32_t tmeMask = (mailbox == 0)   ? CAN_TSR_TME0
                     : (mailbox == 1) ? CAN_TSR_TME1
                                      : CAN_TSR_TME2;

  while ((micros() - start) < timeoutUs) {
    if (CAN1->TSR & tmeMask)
      return true;
  }

  // Safety: If transmission timed out (e.g. no ACK from bus), abort the request
  // so the mailbox does not remain permanently blocked.
  if (mailbox == 0) {
    CAN1->TSR |= CAN_TSR_ABRQ0;
  } else if (mailbox == 1) {
    CAN1->TSR |= CAN_TSR_ABRQ1;
  } else {
    CAN1->TSR |= CAN_TSR_ABRQ2;
  }

  return false;
}

// -----------------------------------------------------------------------------
// RS485 MODBUS REQUEST (Function 0x03, 39 registers from 0x0000)
// -----------------------------------------------------------------------------
void sendBmsRequest(uint8_t address) {
  uint8_t req[8] = {address,
                    0x03,
                    (uint8_t)(BMS_START_REGISTER >> 8),
                    (uint8_t)(BMS_START_REGISTER & 0xFF),
                    (uint8_t)(BMS_REGISTER_COUNT >> 8),
                    (uint8_t)(BMS_REGISTER_COUNT & 0xFF),
                    0,
                    0};

  uint16_t crc = calculateCRC16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  // Drain stale RX bytes before a new transaction
  while (bmsUartAvailable())
    (void)bmsUartRead();

  // Send request (board auto-direction switches from PA9 TX line)
  bmsUartWriteBuffer(req, sizeof(req));
  bmsUartFlush();
}

// -----------------------------------------------------------------------------
// PARSE VISION RESPONSE (83 bytes total)
// -----------------------------------------------------------------------------
bool parseBmsResponse(const uint8_t *buf, uint16_t len, uint8_t expectedAddress,
                      uint8_t slot) {
  if (slot >= MAX_BATTERIES || len < BMS_RESPONSE_LEN)
    return false;

  // Validate address, function code 0x03, and byte count 0x4E (78 bytes data)
  if (buf[0] != expectedAddress || buf[1] != 0x03 || buf[2] != 0x4E)
    return false;

  uint16_t receivedCrc = (uint16_t)buf[BMS_RESPONSE_LEN - 2] | ((uint16_t)buf[BMS_RESPONSE_LEN - 1] << 8);
  if (calculateCRC16(buf, BMS_RESPONSE_LEN - 2) != receivedCrc)
    return false;

  BatteryTelemetry &bat = bmsData[slot];
  bat.address = expectedAddress;

  // Register 0: Total Voltage (10 mV = 0.01 V)
  bat.totalVoltageRaw = be16(&buf[3]);
  // Register 1: Current (10 mA, signed: >0 discharge, <0 charge)
  bat.currentRaw = be16s(&buf[5]);

  // Registers 2..17: Cell voltages 1..16 (mV)
  for (uint8_t i = 0; i < 16; i++)
    bat.cellVoltages[i] = be16(&buf[7 + i * 2]);

  // Registers 18..20: Temperatures (°C)
  bat.tempPCB = be16s(&buf[39]);
  bat.tempAvg = be16s(&buf[41]);
  bat.tempMax = be16s(&buf[43]);

  // Registers 21..24
  bat.remainCapRaw = be16(&buf[45]);
  bat.maxChargeCurrentRaw = be16(&buf[47]);
  bat.soh = be16(&buf[49]);
  bat.soc = be16(&buf[51]);

  // Registers 25..27
  bat.status = be16(&buf[53]);
  bat.warning = be16(&buf[55]);
  bat.protection = be16(&buf[57]);

  // Sanity checks
  if (bat.soc > 100 || bat.soh > 100)
    return false;
  if (bat.totalVoltageRaw < 3000 || bat.totalVoltageRaw > 6500)
    return false;

  bat.valid = true;
  bat.lastSeen = millis();
  return true;
}

// -----------------------------------------------------------------------------
// SAFETY LOGIC
// -----------------------------------------------------------------------------
bool chargeAllowedByProtection(uint16_t p) {
  const uint16_t deny = PROT_PACK_OV | PROT_CELL_OV | PROT_CHARGE_OC |
                        PROT_ENV_TEMP | PROT_MOS_TEMP | PROT_CHARGE_OT |
                        PROT_CHARGE_UT;
  return (p & deny) == 0;
}

bool dischargeAllowedByProtection(uint16_t p) {
  const uint16_t deny = PROT_PACK_UV | PROT_CELL_UV | PROT_DISCHARGE_OC |
                        PROT_ENV_TEMP | PROT_MOS_TEMP | PROT_DISCHARGE_OT |
                        PROT_DISCHARGE_UT | PROT_DISCHARGE_SC;
  return (p & deny) == 0;
}

void buildFrame359(uint16_t warning, uint16_t protection, uint8_t out[8]) {
  memset(out, 0, 8);
  if (protection != 0)
    out[0] = 0x01; // generic protection active
  if (warning != 0)
    out[2] = 0x01; // generic warning active
  out[4] = 0x01;   // module count
  out[6] = 'P';
  out[7] = 'N';
}

// -----------------------------------------------------------------------------
// AGGREGATE + SEND PYLONTECH CAN FRAMES TO DEYE
// -----------------------------------------------------------------------------
void sendDeyeCanFrames() {
  uint8_t validCount = 0;

  uint32_t sumSoc = 0;
  uint32_t sumSoh = 0;
  uint32_t sumVoltage = 0;
  int32_t totalCurrentRaw = 0;
  int32_t sumTemp = 0;

  uint32_t chargeLimitA = 0;
  uint32_t dischargeLimitA = 0;

  uint16_t warningOR = 0;
  uint16_t protectionOR = 0;

  bool chargeEnable = true;
  bool dischargeEnable = true;

  for (uint8_t i = 0; i < MAX_BATTERIES; i++) {
    const BatteryTelemetry &b = bmsData[i];
    if (!b.valid)
      continue;

    validCount++;
    sumSoc += b.soc;
    sumSoh += b.soh;
    sumVoltage += b.totalVoltageRaw;
    totalCurrentRaw += b.currentRaw;
    sumTemp += b.tempAvg;

    warningOR |= b.warning;
    protectionOR |= b.protection;

    chargeEnable &= chargeAllowedByProtection(b.protection);
    dischargeEnable &= dischargeAllowedByProtection(b.protection);

    uint16_t bmsChargeA = b.maxChargeCurrentRaw;
    if (bmsChargeA == 0 || bmsChargeA > SYSTEM_MAX_CHARGE_A)
      bmsChargeA = SYSTEM_MAX_CHARGE_A;

    if (USE_MASTER_ONLY) {
      chargeLimitA = bmsChargeA;
      dischargeLimitA = SYSTEM_MAX_DISCHARGE_A;
    } else {
      chargeLimitA += bmsChargeA;
      dischargeLimitA += SYSTEM_MAX_DISCHARGE_A;
    }
  }

  if (validCount == 0)
    return;

  uint16_t avgVoltage001V = (uint16_t)(sumVoltage / validCount);
  int16_t avgTemp01C = (int16_t)((sumTemp / (int32_t)validCount) * 10);
  uint16_t soc = (uint16_t)(sumSoc / validCount);
  uint16_t soh = (uint16_t)(sumSoh / validCount);

  // CURRENT SIGN CONVERSION:
  // Vision BMS: 10 mA, discharge > 0, charge < 0.
  // Pylontech CAN 0x356: 0.1 A, charge > 0, discharge < 0.
  // We divide by 10 and invert sign:
  int32_t current01A32 = - (totalCurrentRaw / 10);
  if (current01A32 > 32767)
    current01A32 = 32767;
  if (current01A32 < -32768)
    current01A32 = -32768;
  int16_t current01A = (int16_t)current01A32;

  // Dynamic high-SOC taper to protect battery top end
  if (soc >= 99) {
    if (chargeLimitA > 5) chargeLimitA = 5;
  } else if (soc >= 97) {
    if (chargeLimitA > 10) chargeLimitA = 10;
  } else if (soc >= 95) {
    if (chargeLimitA > 20) chargeLimitA = 20;
  }

  if (!chargeEnable)
    chargeLimitA = 0;
  if (!dischargeEnable)
    dischargeLimitA = 0;

  // Pylon 0x351 current units = 0.1 A
  uint16_t chargeLimit01A = clampU16(chargeLimitA * 10U, 65000);
  uint16_t dischargeLimit01A = clampU16(dischargeLimitA * 10U, 65000);

  // Frame 0x359: Alarms / Warnings
  uint8_t d359[8];
  buildFrame359(warningOR, protectionOR, d359);

  // Frame 0x351: CVL, CCL, DCL, DVL (0.1 V and 0.1 A)
  uint8_t d351[8] = {(uint8_t)(CHARGE_VOLTAGE_LIMIT_01V & 0xFF),
                     (uint8_t)(CHARGE_VOLTAGE_LIMIT_01V >> 8),
                     (uint8_t)(chargeLimit01A & 0xFF),
                     (uint8_t)(chargeLimit01A >> 8),
                     (uint8_t)(dischargeLimit01A & 0xFF),
                     (uint8_t)(dischargeLimit01A >> 8),
                     (uint8_t)(DISCHARGE_VOLTAGE_LIMIT_01V & 0xFF),
                     (uint8_t)(DISCHARGE_VOLTAGE_LIMIT_01V >> 8)};

  // Frame 0x355: SOC, SOH (uint16 LE)
  uint8_t d355[4] = {(uint8_t)(soc & 0xFF), (uint8_t)(soc >> 8),
                     (uint8_t)(soh & 0xFF), (uint8_t)(soh >> 8)};

  // Frame 0x356: Voltage (0.01V LE), Current (0.1A signed LE), Temp (0.1°C signed LE)
  uint8_t d356[6] = {(uint8_t)(avgVoltage001V & 0xFF),
                     (uint8_t)(avgVoltage001V >> 8),
                     (uint8_t)((uint16_t)current01A & 0xFF),
                     (uint8_t)(((uint16_t)current01A >> 8) & 0xFF),
                     (uint8_t)((uint16_t)avgTemp01C & 0xFF),
                     (uint8_t)(((uint16_t)avgTemp01C >> 8) & 0xFF)};

  // Frame 0x35C: Operation permissions (Byte 0: Bit 7=Charge enable, Bit 6=Discharge enable)
  uint8_t d35C[8] = {0};
  if (chargeEnable)
    d35C[0] |= 0x80;
  if (dischargeEnable)
    d35C[0] |= 0x40;

  // Frame 0x35E: Manufacturer ASCII
  uint8_t d35E[8] = {'P', 'Y', 'L', 'O', 'N', ' ', ' ', ' '};

  const struct {
    uint16_t id;
    const uint8_t *data;
    uint8_t len;
  } frames[] = {
      {0x351, d351, 8}, {0x355, d355, 4}, {0x356, d356, 6},
      {0x359, d359, 8}, {0x35C, d35C, 8}, {0x35E, d35E, 8},
  };

  bool allOk = true;
  for (const auto &f : frames) {
    if (!sendCANMessage(f.id, f.data, f.len)) {
      allOk = false;
      break;
    }
    delayMicroseconds(600);
  }

  if (allOk) {
    // Non-blocking trigger for PCS-CAN LED
    digitalWrite(PIN_LED_PCS_CAN, HIGH);
    ledCanPulseUntil = millis() + LED_PULSE_DURATION_MS;
  }
}

// -----------------------------------------------------------------------------
// POLLING STATE MACHINE
// -----------------------------------------------------------------------------
uint8_t rxBuffer[128];
uint16_t rxIndex = 0;
bool waitingForResponse = false;
uint32_t responseStartTime = 0;
uint32_t pollTimer = 0;
uint8_t currentBatteryIndex = 0;
uint8_t currentExpectedAddress = MASTER_ADDRESS;

uint8_t getAddressForSlot(uint8_t slot) {
  if (USE_MASTER_ONLY)
    return MASTER_ADDRESS;
  return moduleAddresses[slot];
}

uint8_t activeSlotCount() {
  return USE_MASTER_ONLY ? 1 : MAX_BATTERIES;
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
bool canReady = false;

void setup() {
  // DIP switch inputs (PB12..PB15, Active Low)
  pinMode(PB12, INPUT_PULLUP);
  pinMode(PB13, INPUT_PULLUP);
  pinMode(PB14, INPUT_PULLUP);
  pinMode(PB15, INPUT_PULLUP);

  // Indicators (PB0, PB1, PC5, PC13)
  pinMode(PIN_LED_RUN, OUTPUT);
  pinMode(PIN_LED_BMS_485, OUTPUT);
  pinMode(PIN_LED_PCS_CAN, OUTPUT);
  pinMode(PIN_LED_PCS_485, OUTPUT);

  // Factory control lines (PC3 LOW, PC4 HIGH)
  pinMode(PIN_FACTORY_CTRL_LOW, OUTPUT);
  pinMode(PIN_FACTORY_CTRL_HIGH, OUTPUT);

  // Initial output states
  digitalWrite(PIN_LED_RUN, HIGH);
  digitalWrite(PIN_LED_BMS_485, LOW);
  digitalWrite(PIN_LED_PCS_CAN, LOW);
  digitalWrite(PIN_LED_PCS_485, LOW);
  digitalWrite(PIN_FACTORY_CTRL_LOW, LOW);
  digitalWrite(PIN_FACTORY_CTRL_HIGH, HIGH);

  // Initialize BMS UART (USART1/USART0: PA9 TX / PA10 RX)
  initBmsUart();

  // Initialize CAN (CAN1/CAN0: PB8 RX / PB9 TX, 500k)
  canReady = initCAN_500k();

  for (uint8_t i = 0; i < MAX_BATTERIES; i++) {
    bmsData[i] = BatteryTelemetry();
  }
}

// -----------------------------------------------------------------------------
// MAIN LOOP
// -----------------------------------------------------------------------------
void loop() {
  uint32_t now = millis();

  // 1. Poll BMS via Modbus RTU
  if (!waitingForResponse) {
    if (now - pollTimer >= BMS_POLL_INTERVAL_MS) {
      pollTimer = now;
      rxIndex = 0;
      currentExpectedAddress = getAddressForSlot(currentBatteryIndex);
      sendBmsRequest(currentExpectedAddress);
      waitingForResponse = true;
      responseStartTime = now;
    }
  } else {
    while (bmsUartAvailable()) {
      uint8_t b = bmsUartRead();
      if (rxIndex < sizeof(rxBuffer))
        rxBuffer[rxIndex++] = b;

      if (rxIndex >= BMS_RESPONSE_LEN) {
        if (parseBmsResponse(rxBuffer, rxIndex, currentExpectedAddress,
                             currentBatteryIndex)) {
          // Successful telemetry parse: trigger visible non-blocking LED blink
          digitalWrite(PIN_LED_BMS_485, HIGH);
          ledBmsPulseUntil = now + LED_PULSE_DURATION_MS;
        }

        waitingForResponse = false;
        currentBatteryIndex++;
        if (currentBatteryIndex >= activeSlotCount())
          currentBatteryIndex = 0;
        break;
      }
    }

    if (waitingForResponse &&
        (now - responseStartTime > BMS_RESPONSE_TIMEOUT_MS)) {
      waitingForResponse = false;
      currentBatteryIndex++;
      if (currentBatteryIndex >= activeSlotCount())
        currentBatteryIndex = 0;
    }
  }

  // 2. Invalidate stale telemetry if silent for > 5s
  for (uint8_t i = 0; i < MAX_BATTERIES; i++) {
    if (bmsData[i].valid && (now - bmsData[i].lastSeen > BMS_STALE_MS))
      bmsData[i].valid = false;
  }

  // 3. Send Deye / Pylontech CAN periodically (1s heartbeat)
  static uint32_t lastCanTime = 0;
  if (canReady && now - lastCanTime >= CAN_PERIOD_MS) {
    lastCanTime = now;
    sendDeyeCanFrames();
  }

  // 4. Non-blocking LED Pulse Resets
  if (ledBmsPulseUntil != 0 && now >= ledBmsPulseUntil) {
    digitalWrite(PIN_LED_BMS_485, LOW);
    ledBmsPulseUntil = 0;
  }
  if (ledCanPulseUntil != 0 && now >= ledCanPulseUntil) {
    digitalWrite(PIN_LED_PCS_CAN, LOW);
    ledCanPulseUntil = 0;
  }

  // 5. RUN Heartbeat (PB0 toggles every 1 second)
  static uint32_t lastBlink = 0;
  if (now - lastBlink >= 1000) {
    lastBlink = now;
    digitalWrite(PIN_LED_RUN, !digitalRead(PIN_LED_RUN));
  }
}
