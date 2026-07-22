# USB DFU Demo

This is an example software bootloader implementing the USB-DFU 1.1 standard.

You can customize the USB Vendor ID / Product ID in the `source/usbdfu.c` file. 
The VID:PID is default to `04d9:F00D`

It expect that your actual code to be loaded at `0x2000` (APP_BASE)