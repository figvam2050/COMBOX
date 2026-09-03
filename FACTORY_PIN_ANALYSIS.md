# Factory Firmware Pin Analysis

Source: `combobox_bkp_40000.bin`, a 262,144-byte full-flash readout of the
original COM-Box, plus PCB photographs.

```text
SHA-256: 2c3c0f22812db4e449603d5e59a879f15b963becd3f8b152cb50e426738d0731
Factory boot vector: 0x08000000
Factory application vector: 0x08004000
```

The addresses below refer to the loaded factory image. They are reverse-
engineering evidence, not addresses in the new standalone firmware.

## Confirmed interfaces

| Function | MCU pins | Factory evidence | Confidence |
| --- | --- | --- | --- |
| BMS-485 | `USART1`: `PA9` TX, `PA10` RX | USART1 `0x40013800`, 9600 8N1; logical serial channel 0; channel 0 generates the BMS Modbus request `1E 03 01 00 00 20 ...` | High |
| PCS-485 | `UART5`: `PC12` TX, `PD2` RX | UART5 `0x40005000`, 9600 8N1; logical serial channel 1 and the separate PCS request handler | High |
| PCS-CAN | `CAN1`: `PB8` RX, `PB9` TX | CAN1 GPIO setup and AFIO remap in the factory initialization near `0x080061A8` | High |
| DIP switch 4 | `PB15` | Active-low input, weight 8 | High |
| DIP switch 3 | `PB14` | Active-low input, weight 4 | High |
| DIP switch 2 | `PB13` | Active-low input, weight 2 | High |
| DIP switch 1 | `PB12` | Active-low input, weight 1 | High |

The DIP reader is at approximately `0x08008E90`. Its numeric value is:

```text
value = (!PB15 * 8) + (!PB14 * 4) + (!PB13 * 2) + (!PB12 * 1)
```

The factory application converts DIP value 0 to communication address `0x10`.

## Indicators

The front-panel order and the MCU outputs are:

| PCB LED | Enclosure label | MCU pin | Factory evidence | Confidence |
| --- | --- | --- | --- | --- |
| `LED5` | PCS-485 Connect | `PC13` | Periodic status routine writes GPIOC bit 13 | High |
| `LED6` | PCS-CAN Connect | `PC5` | Periodic status routine writes GPIOC bit 5; associated with CAN state | High |
| `LED4` | BMS-485 Connect | `PB1` | Periodic status routine writes GPIOB bit 1; associated with BMS state | High |
| `LED3` | Running | `PB0` | Periodic status routine writes GPIOB bit 0; running/heartbeat state | High |

This also agrees with the four transistor drivers `Q3`, `Q5`, `Q4`, and `Q6`
visible on the reverse side of the PCB. `PB2` is configured as an input by the
factory firmware, so it cannot be the PCS-CAN indicator used in the previous
project version.

## RS485 direction and control lines

The factory GPIO initialization sets:

```text
PC3 = LOW
PC4 = HIGH
```

No runtime writes to `PC3` or `PC4` were found around either UART transmitter.
The factory application also does not configure or toggle `PA1`. Therefore the
old project definition `PIN_RS485_DIR PA1` was incorrect and has been removed.

The photographs show isolated half-duplex RS485 transceivers, but a stripped
binary alone cannot prove whether `PC3` and `PC4` are port enables, polarity
controls, or inputs to an external automatic-direction circuit. The new
firmware reproduces the factory levels and relies on the PCB hardware for
direction control. A continuity measurement from the transceiver-side `DE` and
`/RE` pins to the MCU is still required before assigning more specific names.

## Changes applied to the standalone firmware

- BMS polling now uses factory channel 0: `USART1`, `PA9`/`PA10`, 9600 8N1.
- The second RS485 channel remains identified as PCS-485 on `UART5`,
  `PC12`/`PD2`; the current Deye implementation uses CAN and does not initialize
  this UART.
- CAN remains on remapped `CAN1`, `PB8`/`PB9`.
- Indicators use `PB0`, `PB1`, `PC5`, and `PC13` according to the factory code.
- DIP inputs `PB12` through `PB15` are initialized with pull-ups and captured at
  startup; their value is reserved for future selectable profiles.
- The false `PA1` DE/RE switching has been removed.
- `PC3 LOW` and `PC4 HIGH` are established during startup to match the factory
  application.
- The new image is standalone at `0x08000000`; the original factory
  bootloader/application split is not reused.

## Confirmed Live BMS Communication (Official Vision MODbus Protocol V01.01)
 
Документ `MODbus Communication Protocol_15-16S-1.pdf` та натурні випробування з `BMS_TOOLS_2.2.01` підтвердили:
- Опитування здійснюється за адресою **`0x10`** (Master BMS): `10 03 00 00 00 27 06 91` (39 регістрів).
- Батарея повертає 83 байти даних: `10 03 4E 13 3F 00 00 ...` (49.27 В, 31% SOC, 100% SOH).
- **Офіційна карта регістрів:**
  - `0000`: Напруга 49.27 В (10 мВ)
  - `0001`: Струм 0.00 А (10 мА, >0 розряд, <0 заряд)
  - `0002..0017`: 16 комірок (15 активних комірок 3.12 В .. 3.28 В, 16-та = 0)
  - `0018`: Temp of PCB (23 °C)
  - `0019`: Temp Avg (22 °C)
  - `0020`: Temp Max (22 °C)
  - `0021`: Cap Remaining (`0x000F` = 15 Ah)
  - `0022`: Max charging Current (`0x0064` = 100 A)
  - `0023`: SOH (`0x0064` = 100%)
  - `0024`: SOC (`0x001F` = 31% при напрузі 49.27 В = 3.28 В/комірку)
  - `0025..0027`: Status, Warning, Protection
  - `0036`: Cell Num (15 комірок)
  - `0037`: Designed Capacity (100.0 Ah)
- Заводська прошивка Felicity опитувала адресу `0x1E` регістр `0x0100` (`1E 03 01 00 00 20`), через що не бачила батарею Vision. Нова прошивка повністю адаптована під офіційний протокол Vision.
- Повні результати схемотехніки та розпіновки закріплені в еталонному документі `PORT_MAPPING.md`.
