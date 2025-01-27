*****************************************************************************
** ChibiOS/HAL - I2C driver demo for NUC123.                               **
*****************************************************************************

** TARGET **

The demo runs on a NUTINY-SDK-NUC123-V2.0 board with a NUC123SD4AN0 MCU.

** The Demo **

The application demonstrates the use of the NUC123 platform driver, and a little
bit of the PAL. A successful run of the test involves the on-board LED blinking at 2 Hz
(on for 100 ms, then off for 400 ms).


** Board Setup **

- Connect 128x32 OLED module to PA.10=I2C1_SDA and PA.11=I2C1_SCL .
- Connect 128x64 OLED module to PF.2=I2C0_SDA and PF.3=I2C0_SCL .
- If any of the OLED modules are absent, pull-up resistors are required instead.

** Build Procedure **

The demo has been tested using gcc version 9.3.1 (GNU Arm Embedded Toolchain 9-2020-q2-update).
Just add overriding setting for TRGT in the command line in order to use specific version of GCC.
for example: make -j TRGT=/opt/ARM/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-
