*****************************************************************************
** ChibiOS/HAL - SERIAL driver demo for NUC123.                            **
*****************************************************************************

** TARGET **

The demo runs on a NUTINY-SDK-NUC123-V2.0 board with a NUC123SD4AN0 MCU.

** The Demo **

The application demonstrates the use of the NUC123 SERIAL driver. When successful,
SD0 should be used for serial communication with another serial device. The
target board should output `\nInitialized...\n` upon power-on, and should
immediately echo any character sent to it, with the standard C function toupper
called on it.

** Board Setup **

To use an external serial interface:
- Attach a serial bus to pins 21-24, and the GND:
    21 - NUC123 RX
    22 - NUC123 TX
    23 - NUC123 nRTS (optional)
    24 - NUC123 nCTS (optional)
- Ensure that the interface is set to the same configuration as the demo
  (by default 57600 8N1)

To use the ICE's on-board USB-serial interface:
- Set SW2 1-4 to ON
- Connect a USB cable from a workstation to J5

** Build Procedure **

The demo has been tested using gcc version 9.3.1 (GNU Arm Embedded Toolchain 9-2020-q2-update).
Just modify the TRGT line in the makefile in order to use different GCC ports.
