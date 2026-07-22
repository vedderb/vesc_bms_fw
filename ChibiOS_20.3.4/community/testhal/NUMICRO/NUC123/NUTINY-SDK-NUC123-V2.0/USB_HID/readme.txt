*****************************************************************************
** ChibiOS/HAL - USB driver demo for NUC123.                               **
*****************************************************************************

** TARGET **

The demo runs on a NUTINY-SDK-NUC123-V2.0 board with a NUC123SD4AN0 MCU.

** The Demo **

The application demonstrates the use of the NUC123 USB driver. A successful run of the test
should begin with the on-board LED blinking slowly, then faster when the USB driver initializes.
The host should recognize the board as a USB HID, and when run with the appropriate VID/PID, the
supplied client application should communicate with the board.

** Board Setup **

- None

** Build Procedure **

The demo has been tested using gcc version 9.3.1 (GNU Arm Embedded Toolchain 9-2020-q2-update).
Just modify the TRGT line in the makefile in order to use different GCC ports.

Two versions of the client code are provided. The Linux version uses the kernel's native hidraw API.
The Darwin version uses the hidapi from libusb (https://github.com/libusb/hidapi)

The Darwin client has only been tested using Apple clang version 12.0.0 (clang-1200.0.32.2), on
macOS Catalina 10.15.7. However, it should be easily portable to any platform supported by hidapi.

To build, adjust HIDAPI_HEADER_PATH in Client/darwin/Makefile to the appropriate location.

** Notes **

This test was adapted from Jonathan Struebel's USB_HID test for the KINETIS FRDM-KL25Z. All files 
are copyright their original authors, as indicated in the headers.
