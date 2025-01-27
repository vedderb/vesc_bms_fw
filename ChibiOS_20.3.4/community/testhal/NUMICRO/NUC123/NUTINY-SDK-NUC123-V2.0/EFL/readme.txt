*****************************************************************************
** ChibiOS/HAL - EFL driver demo for NUC123.                               **
*****************************************************************************

** TARGET **

The demo runs on a NUTINY-SDK-NUC123-V2.0 board with a NUC123SD4AN0 MCU.

** The Demo **

The application demonstrates the use of the NUC123 EFL driver. The test exposes
shell access via the SD0 serial port (configured to 57600 8N1), 
accessible through the corresponding pins or via the on-board NuLinkMe.
That shell allows for 3 non-default commands:

    - kvs_put key value     : This command stores value associated with key.
                            value is a string, and key is a numeric value
                            [1, MFS_CFG_MAX_RECORDS]
    - kvs_get key           : This command retrieves the value associated with
                            key.
    - kvs_erase {--all|key} : This command either erases the value associated
                            with key, or all key value pairs.

The data store should persist, even when the board loses power. Try restarting
the board and make sure the state is as you left it.

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

The demo has been tested using gcc version 10.2.1 (GNU Arm Embedded Toolchain 10-2020-q4-major).
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Notes **
