*****************************************************************************
** ChibiOS/HAL - USB-CDC (IAD descriptors) driver demo for STM32F4xx.      **
*****************************************************************************

** TARGET **

The demo runs on an Sipeed Longan Nano GD32VF103CBT6 board.

** The Demo **

The application demonstrates the use of the GD32VF103CBT6 USB driver.

** Build Procedure **

The demo has been tested using the riscv-gnu-toolchain https://github.com/riscv/riscv-gnu-toolchain. 
Flash using a recent dfu-util version wich has the fixes for the DFU Bootloader of the GD32VF103 https://sourceforge.net/projects/dfu-util/.

```shell
make
dfu-util -d 28e9:0189 -s 0x08000000:leave -D ./build/usbcdc.bin -w
```

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com
