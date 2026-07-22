/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for a Generic GD32VF103 board.
 */

/*
 * Board identifier.
 */
#define BOARD_GD32VF103
#define BOARD_NAME              "Sipeed Longan Nano GD32VF103CBT6"
#define BOARD_USBFS_NOVBUSSENS

/*
 * Board frequencies.
 */
#define GD32_LXTALCLK              32768
#define GD32_HXTALCLK            8000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define GD32VF103CB

/*
 * IO pins assignments
 */
#define PIN_GREEN_LED         1
#define PIN_BLUE_LED          2
#define PIN_RED_LED           13

#define PIN_DISPLAY_MISO      6
#define PIN_DISPLAY_MOSI      7
#define PIN_DISPLAY_SCK       5
#define PIN_DISPLAY_CS        2
#define PIN_DISPLAY_DC        0
#define PIN_DISPLAY_RST       1

#define LINE_GREEN_LED         PAL_LINE(GPIOA, PIN_GREEN_LED)
#define LINE_BLUE_LED          PAL_LINE(GPIOA, PIN_BLUE_LED)
#define LINE_RED_LED           PAL_LINE(GPIOC, PIN_RED_LED)

#define LINE_DISPLAY_MISO      PAL_LINE(GPIOA, PIN_DISPLAY_MISO)
#define LINE_DISPLAY_MOSI      PAL_LINE(GPIOA, PIN_DISPLAY_MOSI)
#define LINE_DISPLAY_SCK       PAL_LINE(GPIOA, PIN_DISPLAY_SCK)
#define LINE_DISPLAY_CS        PAL_LINE(GPIOB, PIN_DISPLAY_CS)
#define LINE_DISPLAY_DC        PAL_LINE(GPIOB, PIN_DISPLAY_DC)
#define LINE_DISPLAY_RST       PAL_LINE(GPIOB, PIN_DISPLAY_RST)
/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * A1 - Green LED - Push Pull output 50MHz
 * A2 - Blue LED - Push Pull output 50MHz
 * A5 - Display SCK - Alternate Push Pull output 50MHz.
 * A7 - Display MOSI - Alternate Push Pull output 50MHz.
 */
#define VAL_GPIOACRL            0xB8B88338      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x88888888      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.
 * Everything input with pull-up except:
 * B0 - Display DC - Push Pull output 50Mhz.
 * B1 - Display RST - Push Pull output 50Mhz.
 * B2 - Display CS - Push Pull output 50Mhz.
 */
#define VAL_GPIOBCRL            0x88888333      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x88888888      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 * C13 - RED LED - Push Pull output 50MHz
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88388888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIODCRL            0x88888888      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
