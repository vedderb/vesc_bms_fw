# Open Kemove - DFU Firmware Loader

This repository contains a *beta* bootloader that implements the USB-DFU protocol
for the kemove snowfox keyboard. (Or any LPC11U35/401) for tha matter.

The bootloader expect to be loaded at flash address 0x0000_0000. You can do this
by putting the controller into ISP mode, and drag drop the firmware bin file
onto the controller drive.

If you want to build custom firmware using this loader, please have your firmware
expect to be loaded at 0x3000. Also do no use the first 512 bytes of RAM. It will
be used for reset/interrupt vector table. The table will be copied to first
512 bytes of ram.
