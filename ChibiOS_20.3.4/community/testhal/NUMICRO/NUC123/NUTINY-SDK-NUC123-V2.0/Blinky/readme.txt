*****************************************************************************
** ChibiOS/HAL - Blinky demo for NUC123.                                   **
*****************************************************************************

** TARGET **

The demo runs on a NUTINY-SDK-NUC123-V2.0 board with a NUC123SD4AN0 MCU.

** The Demo **

The application demonstrates the use of the NUC123 platform driver, and a little
bit of the PAL. A successful run of the test involves the on-board LED blinking at .5 Hz
(on for 1 second, then off for one second).

** Board Setup **

- None

** Build Procedure **

The demo has been tested using gcc version 9.3.1 (GNU Arm Embedded Toolchain 9-2020-q2-update).
Just modify the TRGT line in the makefile in order to use different GCC ports.
