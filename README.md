# COM-Box Vision -> Deye

> [!CAUTION]
> **CRITICAL BEFORE FLASHING:**
> 1. Make a complete ST-Link backup of the original MCU flash.
> 2. Verify the actual MCU marking and flash size.
> 3. **Do NOT mass-erase the chip.**
> 4. This project is a standalone image linked for `0x08000000`.
> 5. Keep the original full-flash backup before programming.

## Files
- `platformio.ini` - PlatformIO configuration
- `GD32F305RC_COMBOX.ld` - Linker script for standalone firmware at flash base
- `src/main.cpp` - Application logic

## Hardware Information
The MCU on the board is **GD32F305RCT6**. We use the `genericSTM32F103RC` PlatformIO board profile for register-level compatibility, but with a custom linker script:
- **Flash:** 256 KiB
- **RAM:** 96 KiB (GD32F30x specific)
- **Bootloader:** not used; complete firmware starts at `0x08000000`
- **Firmware region:** 256 KiB

## Build Artifacts
- A successful PlatformIO build automatically copies `firmware.bin` and `firmware.hex` to the `compiled_firmware/` directory.
- `firmware.bin` is addressless and **must be programmed at `0x08000000`**.
- `firmware.hex` contains its own absolute addresses starting at `0x08000000`.
- The image is standalone and does not preserve or call the original bootloader.
- Keep `combobox_bkp_40000.bin` for recovery before programming.

> [!WARNING]
> **IMPORTANT STATUS:**
> - The project builds successfully, but this does not prove live hardware compatibility.
> - The `genericSTM32F103RC` board profile and STM32F1 framework are used only as a compatibility layer for the GD32F305RCT6. BMS UART baud timing is calculated from the runtime APB2 clock; CAN currently assumes APB1 = 32 MHz. Verify both buses with an analyzer before relying on communications.
> - Factory analysis identifies BMS-485 as USART1 (`PA9`/`PA10`), PCS-485 as UART5 (`PC12`/`PD2`), and PCS-CAN as CAN1 (`PB8`/`PB9`). See `FACTORY_PIN_ANALYSIS.md` for evidence and remaining electrical checks.
> - The generated compile database is machine-specific and ignored by Git. VS Code/clangd should use the portable `.vscode/settings.json` configuration instead.
