COM-Box Vision -> Deye build notes

Files:
- platformio.ini              PlatformIO configuration
- GD32F305RC_COMBOX.ld        linker script reserving first 16 KiB for stock bootloader
- src/main.cpp                application logic

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

BUILD ARTIFACT CHECK:
- The last successful PlatformIO build automatically copied firmware.bin and firmware.hex to the `compiled_firmware/` directory.
- firmware.bin size: 15,204 bytes.
- firmware.hex contains the same 15,204 bytes of application data.
- Both files start at flash address 0x08004000; the bootloader area is not included.
- firmware.bin is addressless and must be programmed at 0x08004000.
- firmware.hex contains its own absolute addresses and can be loaded without entering
  a separate start address. Use sector erase only and preserve 0x08000000..0x08003FFF.

IMPORTANT STATUS:
- A successful build proves that the files were generated, not that the application
  is ready for live use on the COM-Box.
- The current build still uses the genericSTM32F103RC board profile and STM32F1
  framework as a compatibility layer for the GD32F305RCT6. UART/CAN clock settings
  and duplicate VECT_TAB_OFFSET definitions must be verified before flashing.
