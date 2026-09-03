# COM-Box Vision -> Deye

> [!CAUTION]
> **CRITICAL BEFORE FLASHING:**
> 1. Make a complete ST-Link backup of the original MCU flash.
> 2. Verify the actual MCU marking and flash size.
> 3. **Do NOT mass-erase the chip.**
> 4. This project is a standalone image linked for `0x08000000`.
> 5. Keep the original full-flash backup before programming.

## Files
- `PORT_MAPPING.md` - **ЕТАЛОННИЙ ДОКУМЕНТ** апаратної продзвонки та розпіновки плати (зміни заборонені)
- `platformio.ini` - PlatformIO configuration
- `GD32F305RC_COMBOX.ld` - Linker script for standalone firmware at flash base
- `src/main.cpp` - Application logic (BMS Modbus RTU -> Deye Pylontech CAN)
- `FACTORY_PIN_ANALYSIS.md` - Аналіз заводської прошивки та реверс-інжиніринг
- `FLASHING_INSTRUCTIONS.md` - Детальна інструкція з прошивки через ST-Link

## Hardware Information
The MCU on the board is **GD32F305RCT6**. We use the `genericSTM32F103RC` PlatformIO board profile for register-level compatibility, but with a custom linker script:
- **Flash:** 256 KiB
- **RAM:** 96 KiB (GD32F30x specific)
- **Bootloader:** not used; complete firmware starts at `0x08000000`
- **Firmware region:** 256 KiB

## Hardware Pinout (Locked per PORT_MAPPING.md)
- **BMS RS485:** `PA9` TX, `PA10` RX (`USART1` / `USART0` 9600 8N1). Апаратне авто-перемикання напрямку від лінії TX. Пін `PA1` не використовується.
- **PCS CAN (Deye):** `PB8` RX, `PB9` TX (`CAN1` / `CAN0`, Partial Remap 500 kbps via ISO1050).
- **Службові лінії:** `PC3 = LOW`, `PC4 = HIGH` (заводські рівні, що підтримуються прошивкою).
- **Світлодіоди:**
  - `PB0` (`LED3` Running): Heartbeat-миготіння кожні 1000 мс.
  - `PB1` (`LED4` BMS-485 Connect): Імпульс 40 мс при отриманні пакету від BMS.
  - `PC5` (`LED6` PCS-CAN Connect): Імпульс 40 мс при відправці CAN-кадрів інвертору.
  - `PC13` (`LED5` PCS-485 Connect): Вимкнено (LOW).
- **DIP-перемикач:** `PB12` (1), `PB13` (2), `PB14` (4), `PB15` (8), active-low з pull-up.

## Verified BMS Communication (Official Vision MODbus Protocol V01.01)
- Документовано в офіційній специфікації `MODbus Communication Protocol_15-16S-1.pdf`:
  - **Запит:** `10 03 00 00 00 27 06 91` (Master Address `0x10`, Read 39 registers from `0x0000`).
  - **Швидкість:** 9600 бод (Default), інтервал читання 300 мс.
  - **Кабель до АКБ (RJ45 Vision BMS):** Pin 1 (RS485-B) -> B COMBOX, Pin 2 (RS485-A) -> A COMBOX, Pin 3 (GND) -> GND COMBOX (або альтернативна заводська пара Pin 8 (B) / Pin 7 (A) / Pin 6 (GND)).
- **Офіційна карта регістрів (39 Holding Registers):**
  - `0000` (байти 3-4): Напруга пака (10 мВ, наприклад `0x133F` = 49.27 В).
  - `0001` (байти 5-6): Струм пака (10 мА, >0 заряд, <0 розряд).
  - `0002..0017` (байти 7..38): Напруги 16 комірок (мВ, для 15S комірка 16 = 0 мВ).
  - `0018` (байти 39-40): Temp of PCB (23 °C).
  - `0019` (байти 41-42): Temp Avg (22 °C) — транслюється в Pylontech 0x356.
  - `0020` (байти 43-44): Temp Max (22 °C).
  - `0021` (байти 45-46): Cap Remaining (`0x000F` = 15 Ah).
  - `0022` (байти 47-48): Max charging Current (`0x0064` = 100 A).
  - `0023` (байти 49-50): SOH (0-100%, `0x0064` = 100%).
  - `0024` (байти 51-52): SOC (0-100%, `0x001F` = 31% при напрузі 49.27 В = 3.28 В/комірку).
  - `0025..0027` (байти 53..58): Status, Warning, Protection прапорці.
  - `0036` (байти 71-72): Cell Num (`0x000F` = 15 комірок).
  - `0037` (байти 73-74): Designed Capacity (`0x03E8` = 1000 -> 100.0 Ah).

## Build Artifacts
- A successful PlatformIO build automatically copies `firmware.bin` and `firmware.hex` to the `compiled_firmware/` directory.
- `firmware.bin` is addressless and **must be programmed at `0x08000000`**.
- `firmware.hex` contains its own absolute addresses starting at `0x08000000`.
- The image is standalone and does not preserve or call the original bootloader.
- Keep `combobox_bkp_40000.bin` for recovery before programming.
