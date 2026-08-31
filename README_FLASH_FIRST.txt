COM-Box Vision -> Deye build notes

Files:
- platformio_fixed.ini        corrected PlatformIO configuration
- GD32F305RC_COMBOX.ld        linker script reserving first 16 KiB for stock bootloader
- main_fixed.cpp              corrected application logic (if included)

CRITICAL BEFORE FLASHING:
1. Make a complete ST-Link backup of the original MCU flash.
2. Verify the actual MCU marking and flash size.
3. Do NOT mass-erase the chip.
4. The application is linked for 0x08004000.
5. First test by programming only the application region.

The MCU on the board is GD32F305RCT6. We use the genericSTM32F103RC PlatformIO board profile for register-level compatibility, but with a custom linker script:
- Flash: 256 KiB
- RAM: 96 KiB (GD32F30x specific)
- Reserved bootloader: 16 KiB
- Application region: 240 KiB
