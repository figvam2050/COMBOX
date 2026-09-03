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

## Verified BMS Communication
- Перевірено та підтверджено утилітою `BMS_TOOLS_2.2.01` на реальній батареї:
  - Запит: `10 03 00 00 00 27 06 91` (Slave Address `0x10`, Read 39 registers).
  - Відповідь: `10 03 4E 13 3F 00 00 ...` (49.27V, 100% SOC/SOH, 83 байти).
- Прошивка інвертує знак струму згідно зі специфікацією Pylontech CAN 0x356 (у Vision розряд $> 0$, у Deye розряд $< 0$).

## Build Artifacts
- A successful PlatformIO build automatically copies `firmware.bin` and `firmware.hex` to the `compiled_firmware/` directory.
- `firmware.bin` is addressless and **must be programmed at `0x08000000`**.
- `firmware.hex` contains its own absolute addresses starting at `0x08000000`.
- The image is standalone and does not preserve or call the original bootloader.
- Keep `combobox_bkp_40000.bin` for recovery before programming.
