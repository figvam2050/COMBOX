COM-Box Vision -> Deye build notes

Files:
- platformio_fixed.ini        corrected PlatformIO configuration
- STM32F103RC_COMBOX.ld       linker script reserving first 16 KiB for stock bootloader
- main_fixed.cpp              corrected application logic (if included)

CRITICAL BEFORE FLASHING:
1. Make a complete ST-Link backup of the original MCU flash.
2. Verify the actual MCU marking and flash size.
3. Do NOT mass-erase the chip.
4. The application is linked for 0x08004000.
5. First test by programming only the application region.

The current PlatformIO board profile is genericSTM32F103RC:
- Flash: 256 KiB
- RAM: 48 KiB
- Reserved bootloader: 16 KiB
- Application region: 240 KiB
