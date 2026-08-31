#include <Arduino.h> 
#include <algorithm>

/*
  Vision V-LFP48100 -> Felicity COM-Box hardware -> Deye CAN/Pylontech
  Revised main.cpp

  IMPORTANT:
  1) This file fixes application logic only.
  2) The stock Felicity bootloader/application offset (0x08004000) must be
  handled in the linker script / PlatformIO configuration, NOT in main.cpp. 3)
  Hardware pin mapping must still be verified on the actual COM-Box PCB.
*/

// -----------------------------------------------------------------------------
// HARDWARE PINS (VERIFY ON REAL PCB)
// -----------------------------------------------------------------------------
#define PIN_LED_RUN PB0
#define PIN_LED_485 PB1
#define PIN_LED_CAN PB2
#define PIN_RS485_DIR PA1 // DE/RE for battery RS485

// Deye uses CAN in this firmware. The second RS485 path from the old draft is
// intentionally disabled because PB12 was only a guess and is not needed.

// -----------------------------------------------------------------------------
// BATTERY / PROTOCOL SETTINGS
// -----------------------------------------------------------------------------
#define MAX_BATTERIES 4

// Recommended starting mode:
//   true  -> poll only Vision master address 0x10 (safe, no double counting)
//   false -> poll module addresses 0x01..0x04
//
// After live RS485 tests we can switch this if Vision exposes each module
// separately and 0x10 is NOT an aggregate master.
static constexpr bool USE_MASTER_ONLY = true;

static const uint8_t moduleAddresses[MAX_BATTERIES] = {0x01, 0x02, 0x03, 0x04};
static const uint8_t MASTER_ADDRESS = 0x10;

// Vision register map request: holding registers 0x0000 .. 0x0026 (39 regs).
static constexpr uint16_t BMS_START_REGISTER = 0x0000;
static constexpr uint16_t BMS_REGISTER_COUNT = 0x0027; // 39 registers
static constexpr uint16_t BMS_RESPONSE_LEN = 83;       // 1+1+1+78+2

// Timeouts / periods.
static constexpr uint32_t BMS_RESPONSE_TIMEOUT_MS = 150;
static constexpr uint32_t BMS_STALE_MS = 5000;
static constexpr uint32_t BMS_POLL_INTERVAL_MS = 300;
static constexpr uint32_t CAN_PERIOD_MS = 1000;

// Conservative system limits. Deye 6 kW low-voltage inverter can exceed these,
// but the BMS remains the primary source. These are hard safety ceilings.
static constexpr uint16_t SYSTEM_MAX_CHARGE_A = 100;
static constexpr uint16_t SYSTEM_MAX_DISCHARGE_A = 120;

// Voltage limits sent to Deye in 0.1 V units.
static constexpr uint16_t CHARGE_VOLTAGE_LIMIT_01V = 540;    // 54.0 V
static constexpr uint16_t DISCHARGE_VOLTAGE_LIMIT_01V = 430; // 43.0 V

// -----------------------------------------------------------------------------
// VISION PROTECTION BIT MASKS (from supplied Vision/GCE documentation)
// -----------------------------------------------------------------------------
static constexpr uint16_t PROT_PACK_OV = 0x0001;
static constexpr uint16_t PROT_CELL_OV = 0x0002;
static constexpr uint16_t PROT_PACK_UV = 0x0004;
static constexpr uint16_t PROT_CELL_UV = 0x0008;
static constexpr uint16_t PROT_CHARGE_OC = 0x0010;
static constexpr uint16_t PROT_DISCHARGE_OC = 0x0020;
static constexpr uint16_t PROT_ENV_TEMP = 0x0040;
static constexpr uint16_t PROT_MOS_TEMP = 0x0080;
static constexpr uint16_t PROT_CHARGE_OT = 0x0100;
static constexpr uint16_t PROT_DISCHARGE_OT = 0x0200;
static constexpr uint16_t PROT_CHARGE_UT = 0x0400;
static constexpr uint16_t PROT_DISCHARGE_UT = 0x0800;
static constexpr uint16_t PROT_LOW_CAPACITY = 0x1000;
static constexpr uint16_t PROT_DISCHARGE_SC = 0x2000;

// -----------------------------------------------------------------------------
// TELEMETRY
// -----------------------------------------------------------------------------
struct BatteryTelemetry {
  uint8_t address = 0;

  uint16_t totalVoltageRaw = 0; // 10 mV => already 0.01 V
  int16_t currentRaw = 0;       // 10 mA
  uint16_t cellVoltages[16] = {0};

  int16_t tempPCB = 0; // documentation shows deg C
  int16_t tempAvg = 0;
  int16_t tempMax = 0;

  uint16_t remainCapRaw = 0;
  uint16_t maxChargeCurrentRaw = 0;
  uint16_t soh = 0;
  uint16_t soc = 0;

  uint16_t status = 0;
  uint16_t warning = 0;
  uint16_t protection = 0;

  bool valid = false;
  uint32_t lastSeen = 0;
};

BatteryTelemetry bmsData[MAX_BATTERIES];

// -----------------------------------------------------------------------------
// HELPERS
// -----------------------------------------------------------------------------
static inline uint16_t be16(const uint8_t *p) {
  return ((uint16_t)p[0] << 8) | p[1];
}

static inline int16_t be16s(const uint8_t *p) { return (int16_t)be16(p); }

static uint16_t clampU16(uint32_t v, uint16_t maxv) {
  return (v > maxv) ? maxv : (uint16_t)v;
}

// -----------------------------------------------------------------------------
// MODBUS CRC16
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

// -----------------------------------------------------------------------------
// USART2: BMS RS485, 9600 8N1 (APB1 = 36 MHz assumption)
// -----------------------------------------------------------------------------
void initUSART2_BMS() {
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  // PA3 RX floating input
  GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
  GPIOA->CRL |= (0x04 << 12);

  // PA2 TX AF push-pull 50 MHz
  GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
  GPIOA->CRL |= (0x0A << 8);

  USART2->CR1 = 0;
  USART2->CR2 = 0;
  USART2->CR3 = 0;

  // 36 MHz / 9600 = 3750 => BRR 0xEA6 for oversampling x16.
  USART2->BRR = 0xEA6;
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void USART2_Write(uint8_t data) {
  while (!(USART2->SR & USART_SR_TXE)) {
  }
  USART2->DR = data;
}

void USART2_WriteBuffer(const uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++)
    USART2_Write(data[i]);
}

bool USART2_Available() { return (USART2->SR & USART_SR_RXNE) != 0; }

uint8_t USART2_Read() { return (uint8_t)USART2->DR; }

void USART2_Flush() {
  while (!(USART2->SR & USART_SR_TC)) {
  }
}

// -----------------------------------------------------------------------------
// CAN1 500 kbps, APB1 = 36 MHz, PA11 RX / PA12 TX
// 36 MHz / (BRP 4 * 18 tq) = 500 kbit/s
// TS1 = 12 tq, TS2 = 5 tq, SJW = 2 tq.
// -----------------------------------------------------------------------------
bool initCAN_500k() {
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;

  // CAN_RX PA11 floating input
  GPIOA->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
  GPIOA->CRH |= (0x04 << 12);

  // CAN_TX PA12 AF push-pull 50 MHz
  GPIOA->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12);
  GPIOA->CRH |= (0x0A << 16);

  // Make sure CAN is on default PA11/PA12 mapping.
#ifdef AFIO_MAPR_CAN_REMAP
  AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
#endif

  CAN1->MCR &= ~CAN_MCR_SLEEP;
  CAN1->MCR |= CAN_MCR_INRQ;

  uint32_t timeout = millis() + 100;
  while ((CAN1->MSR & CAN_MSR_INAK) == 0) {
    if ((int32_t)(millis() - timeout) >= 0)
      return false;
  }

  // bxCAN BTR fields:
  // SJW bits 25:24 => 1 means 2 tq
  // TS2 bits 22:20 => 4 means 5 tq
  // TS1 bits 19:16 => 11 means 12 tq
  // BRP bits 9:0   => 3 means prescaler 4
  CAN1->BTR = (1U << 24) | (4U << 20) | (11U << 16) | 3U;

  // Auto bus-off recovery + automatic retransmission enabled.
  CAN1->MCR |= CAN_MCR_ABOM;
  CAN1->MCR &= ~CAN_MCR_NART;

  // Accept all frames.
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

// Wait for a free TX mailbox, load a standard 11-bit frame, then wait until the
// mailbox becomes free again. This prevents dropping frames 4..6 in a burst.
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

  // Wait for completion/free mailbox.
  start = micros();
  uint32_t tmeMask = (mailbox == 0)   ? CAN_TSR_TME0
                     : (mailbox == 1) ? CAN_TSR_TME1
                                      : CAN_TSR_TME2;

  while ((micros() - start) < timeoutUs) {
    if (CAN1->TSR & tmeMask)
      return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
// RS485 REQUEST
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

  // Drain stale RX bytes before a new transaction.
  while (USART2_Available())
    (void)USART2_Read();

  digitalWrite(PIN_RS485_DIR, HIGH);
  delayMicroseconds(100);
  USART2_WriteBuffer(req, sizeof(req));
  USART2_Flush();
  delayMicroseconds(100);
  digitalWrite(PIN_RS485_DIR, LOW);
}

// -----------------------------------------------------------------------------
// PARSE VISION RESPONSE
// -----------------------------------------------------------------------------
bool parseBmsResponse(const uint8_t *buf, uint16_t len, uint8_t expectedAddress,
                      uint8_t slot) {
  if (slot >= MAX_BATTERIES || len != BMS_RESPONSE_LEN)
    return false;

  if (buf[0] != expectedAddress || buf[1] != 0x03 || buf[2] != 0x4E)
    return false;

  uint16_t receivedCrc = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
  if (calculateCRC16(buf, len - 2) != receivedCrc)
    return false;

  BatteryTelemetry &bat = bmsData[slot];
  bat.address = expectedAddress;

  // Register 0..1
  bat.totalVoltageRaw = be16(&buf[3]);
  bat.currentRaw = be16s(&buf[5]);

  // Registers 2..17: cell 1..16, mV
  for (uint8_t i = 0; i < 16; i++)
    bat.cellVoltages[i] = be16(&buf[7 + i * 2]);

  // Registers 18..20
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

  // Basic sanity checks. Reject obviously mis-decoded frames.
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

// Very conservative Pylon alarm mapping.
// Exact bit semantics in 0x359 can be refined after sniffing a known-working
// Felicity/Pylon installation. For now we propagate "alarm present" and retain
// manufacturer signature bytes P,N.
void buildFrame359(uint16_t warning, uint16_t protection, uint8_t out[8]) {
  memset(out, 0, 8);

  if (protection != 0)
    out[0] = 0x01; // generic protection present
  if (warning != 0)
    out[2] = 0x01; // generic warning present

  out[6] = 'P';
  out[7] = 'N';
}

// -----------------------------------------------------------------------------
// AGGREGATE + SEND PYLONTECH CAN
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

    // Vision register 22 is documented as "Max charging current".
    // Unit in the source document is not explicit on the page we have, so we
    // deliberately cap it to a conservative physical limit.
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

  // Vision current is 10 mA. Pylon 0x356 current is 0.1 A => divide by 10.
  int32_t current01A32 = totalCurrentRaw / 10;
  if (current01A32 > 32767)
    current01A32 = 32767;
  if (current01A32 < -32768)
    current01A32 = -32768;
  int16_t current01A = (int16_t)current01A32;

  // Dynamic high-SOC taper. Never increase BMS limit, only reduce it.
  if (soc >= 99) {
    chargeLimitA = std::min<uint32_t>(chargeLimitA, 5);
  } else if (soc >= 97) {
    chargeLimitA = std::min<uint32_t>(chargeLimitA, 10);
  } else if (soc >= 95) {
    chargeLimitA = std::min<uint32_t>(chargeLimitA, 20);
  }

  if (!chargeEnable)
    chargeLimitA = 0;
  if (!dischargeEnable)
    dischargeLimitA = 0;

  // Pylon 0x351 current units = 0.1 A.
  uint16_t chargeLimit01A = clampU16(chargeLimitA * 10U, 65000);
  uint16_t dischargeLimit01A = clampU16(dischargeLimitA * 10U, 65000);

  // 0x359
  uint8_t d359[8];
  buildFrame359(warningOR, protectionOR, d359);

  // 0x351: CVL, CCL, DCL, DVL
  uint8_t d351[8] = {(uint8_t)(CHARGE_VOLTAGE_LIMIT_01V & 0xFF),
                     (uint8_t)(CHARGE_VOLTAGE_LIMIT_01V >> 8),
                     (uint8_t)(chargeLimit01A & 0xFF),
                     (uint8_t)(chargeLimit01A >> 8),
                     (uint8_t)(dischargeLimit01A & 0xFF),
                     (uint8_t)(dischargeLimit01A >> 8),
                     (uint8_t)(DISCHARGE_VOLTAGE_LIMIT_01V & 0xFF),
                     (uint8_t)(DISCHARGE_VOLTAGE_LIMIT_01V >> 8)};

  // 0x355: SOC, SOH
  uint8_t d355[4] = {(uint8_t)(soc & 0xFF), (uint8_t)(soc >> 8),
                     (uint8_t)(soh & 0xFF), (uint8_t)(soh >> 8)};

  // 0x356: voltage 0.01 V, current 0.1 A signed, temperature 0.1 C.
  // Standard Pylon frame length is 6 bytes.
  uint8_t d356[6] = {(uint8_t)(avgVoltage001V & 0xFF),
                     (uint8_t)(avgVoltage001V >> 8),
                     (uint8_t)((uint16_t)current01A & 0xFF),
                     (uint8_t)(((uint16_t)current01A >> 8) & 0xFF),
                     (uint8_t)((uint16_t)avgTemp01C & 0xFF),
                     (uint8_t)(((uint16_t)avgTemp01C >> 8) & 0xFF)};

  // 0x35C operation permission flags.
  uint8_t d35C[8] = {0};
  if (chargeEnable)
    d35C[0] |= 0x80;
  if (dischargeEnable)
    d35C[0] |= 0x40;

  // 0x35E manufacturer.
  uint8_t d35E[8] = {'P', 'Y', 'L', 'O', 'N', ' ', ' ', ' '};

  // Pace frames slightly so a slow transceiver / inverter never loses a burst.
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
    digitalWrite(PIN_LED_CAN, HIGH);
    delay(2);
    digitalWrite(PIN_LED_CAN, LOW);
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

uint8_t activeSlotCount() { return USE_MASTER_ONLY ? 1 : MAX_BATTERIES; }

// -----------------------------------------------------------------------------
// SETUP / LOOP
// -----------------------------------------------------------------------------
bool canReady = false;

void setup() {
  pinMode(PIN_LED_RUN, OUTPUT);
  pinMode(PIN_LED_485, OUTPUT);
  pinMode(PIN_LED_CAN, OUTPUT);
  pinMode(PIN_RS485_DIR, OUTPUT);

  digitalWrite(PIN_LED_RUN, HIGH);
  digitalWrite(PIN_LED_485, LOW);
  digitalWrite(PIN_LED_CAN, LOW);
  digitalWrite(PIN_RS485_DIR, LOW);

  initUSART2_BMS();
  canReady = initCAN_500k();

  for (uint8_t i = 0; i < MAX_BATTERIES; i++) {
    bmsData[i] = BatteryTelemetry();
  }
}

void loop() {
  uint32_t now = millis();

  // Poll BMS.
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
    while (USART2_Available()) {
      uint8_t b = USART2_Read();
      if (rxIndex < sizeof(rxBuffer))
        rxBuffer[rxIndex++] = b;

      if (rxIndex == BMS_RESPONSE_LEN) {
        if (parseBmsResponse(rxBuffer, rxIndex, currentExpectedAddress,
                             currentBatteryIndex)) {
          digitalWrite(PIN_LED_485, HIGH);
          delay(2);
          digitalWrite(PIN_LED_485, LOW);
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

  // Invalidate stale telemetry.
  for (uint8_t i = 0; i < MAX_BATTERIES; i++) {
    if (bmsData[i].valid && (now - bmsData[i].lastSeen > BMS_STALE_MS))
      bmsData[i].valid = false;
  }

  // Send Deye/Pylon CAN periodically.
  static uint32_t lastCanTime = 0;
  if (canReady && now - lastCanTime >= CAN_PERIOD_MS) {
    lastCanTime = now;
    sendDeyeCanFrames();
  }

  // RUN heartbeat.
  static uint32_t lastBlink = 0;
  if (now - lastBlink >= 2000) {
    lastBlink = now;
    digitalWrite(PIN_LED_RUN, !digitalRead(PIN_LED_RUN));
  }
}
