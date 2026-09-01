# Factory Pin Analysis

Source: `combobox_bkp_40000.bin`, a 256 KiB readout of the original COM-Box
firmware, plus the PCB photographs.

## Confirmed MCU connections

| Function | MCU pins | Evidence |
| --- | --- | --- |
| BMS/RS485 UART candidate | `UART5`: `PC12` TX, `PD2` RX | UART5 peripheral `0x40005000`, 9600 baud, GPIO setup in factory code |
| Other RS485 UART candidate | `USART1`: `PA9` TX, `PA10` RX | USART1 peripheral `0x40013800`, 9600 baud, GPIO setup in factory code |
| PCS CAN | `CAN1` remapped: `PB8` RX, `PB9` TX | GPIOB setup and AFIO CAN remap in factory code |
| Four DIP inputs | `PB15`, `PB14`, `PB13`, `PB12` | Direct IDR reads with masks `0x8000`, `0x4000`, `0x2000`, `0x1000` |

The DIP reader uses active-low logic and weights the switches as follows:

```text
PB15 = 8, PB14 = 4, PB13 = 2, PB12 = 1
```

## Indicators

The enclosure labels, from left to right, are:

```text
LED5 = PCS-485
LED6 = PCS-CAN
LED4 = BMS-485
LED3 = Running
```

The factory firmware has a logical LED table for `LED3..LED6`. It directly
configures and updates `PB0`, `PB1`, and `PC13`, but the fourth LED is accessed
through an indirect GPIO/LED layer. Its physical MCU pin is therefore not
declared here without a continuity test.

## Current firmware changes

`src/main.cpp` now uses factory-confirmed `UART5` and CAN `PB8/PB9` instead of
the previous guessed `USART2 PA2/PA3` and CAN `PA11/PA12`.

The linker is configured for a standalone image at `0x08000000`; it no longer
reserves the first 16 KiB for the Felicity bootloader.

The RS485 `DE/RE` pin is deliberately still a build-time constant (`PA1`) and
is marked unconfirmed. Do not flash production hardware until this line is
verified with a continuity test from the transceiver DE/RE pin to the MCU.

The two UARTs cannot be assigned with complete confidence to the enclosure
labels from the stripped binary alone. Confirm `BMS-485` versus `PCS-485` with
a logic analyzer or by tracing each transceiver to its UART pins.
