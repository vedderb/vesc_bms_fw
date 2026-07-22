*****************************************************************************
** ChibiOS/HAL - PWM driver demo for NUC123.                               **
*****************************************************************************

** TARGET **

The demo runs on a NUTINY-SDK-NUC123-V2.0 board with a NUC123SD4AN0 MCU.

** The Demo **

The application demonstrates the use of the NUC123 PWM driver. When successful,
pins 44 and 45 should carry a 750 Hz square wave, the duty cycle of which 
sinusoidally oscillates between 0% and 100%. One channel is active high, and
the other is active low.

** Board Setup **

There are multiple ways to observe the PWM output:
- Attach channels of a logic analyzer to pins 44 & 45, and a ground pin. Set it
  to trigger on either (but not both) the rising or falling edge of either (but
  not both) channel.

** Build Procedure **

The demo has been tested using gcc version 9.3.1 (GNU Arm Embedded Toolchain 9-2020-q2-update).
Just modify the TRGT line in the makefile in order to use different GCC ports.
