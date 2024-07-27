/*
    ChibiOS                                  - Copyright (C) 2006..2018 Giovanni Di Sirio
    ChibiOS ST32F411-Disco Board Description - Copyright (C) 2019 Tim Rheinfels

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

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

/*
 * Did so... Using the ChibiOS / STM32F4-Disco HAL with the following changes:
 *
 * - STM32F4 -> STM32F411
 * - Removed all pad specializations except for SWD and HSE oscillator
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for STMicroelectronics STM32F411-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F411_DISCOVERY
#define BOARD_NAME                  "STMicroelectronics STM32F411-Discovery"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F411xE

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0U
#define GPIOA_PIN1                  1U
#define GPIOA_PIN2                  2U
#define GPIOA_PIN3                  3U
#define GPIOA_PIN4                  4U
#define GPIOA_PIN5                  5U
#define GPIOA_PIN6                  6U
#define GPIOA_PIN7                  7U
#define GPIOA_PIN8                  8U
#define GPIOA_PIN9                  9U
#define GPIOA_PIN10                 10U
#define GPIOA_PIN11                 11U
#define GPIOA_PIN12                 12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15                 15U

#define GPIOB_PIN0                  0U
#define GPIOB_PIN1                  1U
#define GPIOB_PIN2                  2U
#define GPIOB_PIN3                  3U
#define GPIOB_PIN4                  4U
#define GPIOB_PIN5                  5U
#define GPIOB_PIN6                  6U
#define GPIOB_PIN7                  7U
#define GPIOB_PIN8                  8U
#define GPIOB_PIN9                  9U
#define GPIOB_PIN10                 10U
#define GPIOB_PIN11                 11U
#define GPIOB_PIN12                 12U
#define GPIOB_PIN13                 13U
#define GPIOB_PIN14                 14U
#define GPIOB_PIN15                 15U

#define GPIOC_PIN0                  0U
#define GPIOC_PIN1                  1U
#define GPIOC_PIN2                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_PIN4                  4U
#define GPIOC_PIN5                  5U
#define GPIOC_PIN6                  6U
#define GPIOC_PIN7                  7U
#define GPIOC_PIN8                  8U
#define GPIOC_PIN9                  9U
#define GPIOC_PIN10                 10U
#define GPIOC_PIN11                 11U
#define GPIOC_PIN12                 12U
#define GPIOC_PIN13                 13U
#define GPIOC_PIN14                 14U
#define GPIOC_PIN15                 15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_PIN2                  2U
#define GPIOD_PIN3                  3U
#define GPIOD_PIN4                  4U
#define GPIOD_PIN5                  5U
#define GPIOD_PIN6                  6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_PIN11                 11U
#define GPIOD_PIN12                 12U
#define GPIOD_PIN13                 13U
#define GPIOD_PIN14                 14U
#define GPIOD_PIN15                 15U

#define GPIOE_PIN0                  0U
#define GPIOE_PIN1                  1U
#define GPIOE_PIN2                  2U
#define GPIOE_PIN3                  3U
#define GPIOE_PIN4                  4U
#define GPIOE_PIN5                  5U
#define GPIOE_PIN6                  6U
#define GPIOE_PIN7                  7U
#define GPIOE_PIN8                  8U
#define GPIOE_PIN9                  9U
#define GPIOE_PIN10                 10U
#define GPIOE_PIN11                 11U
#define GPIOE_PIN12                 12U
#define GPIOE_PIN13                 13U
#define GPIOE_PIN14                 14U
#define GPIOE_PIN15                 15U

#define GPIOF_PIN0                  0U
#define GPIOF_PIN1                  1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_PIN2                  2U
#define GPIOG_PIN3                  3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_PIN6                  6U
#define GPIOG_PIN7                  7U
#define GPIOG_PIN8                  8U
#define GPIOG_PIN9                  9U
#define GPIOG_PIN10                 10U
#define GPIOG_PIN11                 11U
#define GPIOG_PIN12                 12U
#define GPIOG_PIN13                 13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
#define GPIOI_PIN6                  6U
#define GPIOI_PIN7                  7U
#define GPIOI_PIN8                  8U
#define GPIOI_PIN9                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_SWDIO                  PAL_LINE(GPIOA, GPIOA_SWDIO)
#define LINE_SWCLK                  PAL_LINE(GPIOA, GPIOA_SWCLK)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - PIN0                      (analog).
 * PA1  - PIN1                      (analog).
 * PA2  - PIN2                      (analog).
 * PA3  - PIN3                      (analog).
 * PA4  - PIN4                      (analog).
 * PA5  - PIN5                      (analog).
 * PA6  - PIN6                      (analog).
 * PA7  - PIN7                      (analog).
 * PA8  - PIN8                      (analog).
 * PA9  - PIN9                      (analog).
 * PA10 - PIN10                     (analog).
 * PA11 - PIN11                     (analog).
 * PA12 - PIN12                     (analog).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - PIN15                     (analog).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_PIN0)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN1)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN2)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN3)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN4)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN5)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN6)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN7)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN8)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN9)      |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN10)     |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN11)     |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN12)     |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO)  |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK)  |  \
                                     PIN_MODE_ANALOG(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN9)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN10)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN11)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN12)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_PIN0)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN1)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN2)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN3)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN4)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN5)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN6)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN7)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN8)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN9)      |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN10)     |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN11)     |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN12)     |  \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO)     |  \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK)     |  \
                                     PIN_OSPEED_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_PIN0)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN1)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN2)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN3)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN6)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN7)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN8)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN9)   |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN10)  |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN11)  |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN12)  |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO)  |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK)  |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN1)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN2)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN3)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN4)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN5)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN6)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN7)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN8)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN9)         |  \
                                     PIN_ODR_HIGH(GPIOA_PIN10)        |  \
                                     PIN_ODR_HIGH(GPIOA_PIN11)        |  \
                                     PIN_ODR_HIGH(GPIOA_PIN12)        |  \
                                     PIN_ODR_HIGH(GPIOA_SWDIO)        |  \
                                     PIN_ODR_HIGH(GPIOA_SWCLK)        |  \
                                     PIN_ODR_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0U)      |  \
                                     PIN_AFIO_AF(GPIOA_PIN1, 0U)      |  \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0U)      |  \
                                     PIN_AFIO_AF(GPIOA_PIN3, 0U)      |  \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0U)      |  \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0U)      |  \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0U)      |  \
                                     PIN_AFIO_AF(GPIOA_PIN7, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0U)      |  \
                                     PIN_AFIO_AF(GPIOA_PIN9, 0U)      |  \
                                     PIN_AFIO_AF(GPIOA_PIN10, 0U)     |  \
                                     PIN_AFIO_AF(GPIOA_PIN11, 0U)     |  \
                                     PIN_AFIO_AF(GPIOA_PIN12, 0U)     |  \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U)     |  \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U)     |  \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - PIN0                      (analog).
 * PB1  - PIN1                      (analog).
 * PB2  - PIN2                      (analog).
 * PB3  - PIN3                      (analog).
 * PB4  - PIN4                      (analog).
 * PB5  - PIN5                      (analog).
 * PB6  - PIN6                      (analog).
 * PB7  - PIN7                      (analog).
 * PB8  - PIN8                      (analog).
 * PB9  - PIN9                      (analog).
 * PB10 - PIN10                     (analog).
 * PB11 - PIN11                     (analog).
 * PB12 - PIN12                     (analog).
 * PB13 - PIN13                     (analog).
 * PB14 - PIN14                     (analog).
 * PB15 - PIN15                     (analog).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ANALOG(GPIOB_PIN0)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN1)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN2)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN3)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN4)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN5)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN6)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN7)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN8)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN9)      |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN10)     |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN11)     |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN12)     |  \
                                     PIN_MODE_ALTERNATE(GPIOB_PIN13)  |  \
                                     PIN_MODE_ALTERNATE(GPIOB_PIN14)  |  \
                                     PIN_MODE_ANALOG(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN3)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_PIN0)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN1)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN2)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN3)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN4)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN5)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN6)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN7)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN8)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN9)      |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN10)     |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN11)     |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN12)     |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN13)     |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN14)     |  \
                                     PIN_OSPEED_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_PIN0)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN2)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN3)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN4)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN5)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN6)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN7)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN9)   |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN10)  |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN11)  |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN12)  |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN13)  |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN14)  |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PIN0)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN1)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN2)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN3)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN4)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN5)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN6)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN7)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN8)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN9)         |  \
                                     PIN_ODR_HIGH(GPIOB_PIN10)        |  \
                                     PIN_ODR_HIGH(GPIOB_PIN11)        |  \
                                     PIN_ODR_HIGH(GPIOB_PIN12)        |  \
                                     PIN_ODR_HIGH(GPIOB_PIN13)        |  \
                                     PIN_ODR_HIGH(GPIOB_PIN14)        |  \
                                     PIN_ODR_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0, 0U)      |  \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0U)      |  \
                                     PIN_AFIO_AF(GPIOB_PIN2, 0U)      |  \
                                     PIN_AFIO_AF(GPIOB_PIN3, 0U)      |  \
                                     PIN_AFIO_AF(GPIOB_PIN4, 0U)      |  \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0U)      |  \
                                     PIN_AFIO_AF(GPIOB_PIN6, 0U)      |  \
                                     PIN_AFIO_AF(GPIOB_PIN7, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0U)      |  \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0U)      |  \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0U)     |  \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0U)     |  \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0U)     |  \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0U)     |  \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0U)     |  \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0                      (analog).
 * PC1  - PIN1                      (analog).
 * PC2  - PIN2                      (analog).
 * PC3  - PIN3                      (analog).
 * PC4  - PIN4                      (analog).
 * PC5  - PIN5                      (analog).
 * PC6  - PIN6                      (analog).
 * PC7  - PIN7                      (analog).
 * PC8  - PIN8                      (analog).
 * PC9  - PIN9                      (analog).
 * PC10 - PIN10                     (analog).
 * PC11 - PIN11                     (analog).
 * PC12 - PIN12                     (analog).
 * PC13 - PIN13                     (analog).
 * PC14 - PIN14                     (analog).
 * PC15 - PIN15                     (analog).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_PIN0)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN1)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN2)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN3)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN4)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN5)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN6)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN7)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN8)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN9)      |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN10)     |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN11)     |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN12)     |  \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN13)  |  \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN14)  |  \
                                     PIN_MODE_ANALOG(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_PIN0)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN1)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN2)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN3)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN4)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN5)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN6)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN7)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN8)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN9)      |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN10)     |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN11)     |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN12)     |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN13)     |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN14)     |  \
                                     PIN_OSPEED_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN1)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN4)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN5)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN6)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN7)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN8)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN9)   |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN10)  |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN11)  |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN12)  |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN13)  |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN14)  |  \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN1)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN2)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN3)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN4)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN5)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN6)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN7)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN8)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN9)         |  \
                                     PIN_ODR_HIGH(GPIOC_PIN10)        |  \
                                     PIN_ODR_HIGH(GPIOC_PIN11)        |  \
                                     PIN_ODR_HIGH(GPIOC_PIN12)        |  \
                                     PIN_ODR_HIGH(GPIOC_PIN13)        |  \
                                     PIN_ODR_HIGH(GPIOC_PIN14)        |  \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0U)      |  \
                                     PIN_AFIO_AF(GPIOC_PIN1, 0U)      |  \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0U)      |  \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0U)      |  \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0U)      |  \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0U)      |  \
                                     PIN_AFIO_AF(GPIOC_PIN6, 0U)      |  \
                                     PIN_AFIO_AF(GPIOC_PIN7, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0U)      |  \
                                     PIN_AFIO_AF(GPIOC_PIN9, 0U)      |  \
                                     PIN_AFIO_AF(GPIOC_PIN10, 0U)     |  \
                                     PIN_AFIO_AF(GPIOC_PIN11, 0U)     |  \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0U)     |  \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0U)     |  \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0U)     |  \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (analog).
 * PD1  - PIN1                      (analog).
 * PD2  - PIN2                      (analog).
 * PD3  - PIN3                      (analog).
 * PD4  - PIN4                      (analog).
 * PD5  - PIN5                      (analog).
 * PD6  - PIN6                      (analog).
 * PD7  - PIN7                      (analog).
 * PD8  - PIN8                      (analog).
 * PD9  - PIN9                      (analog).
 * PD10 - PIN10                     (analog).
 * PD11 - PIN11                     (analog).
 * PD12 - PIN12                     (analog).
 * PD13 - PIN13                     (analog).
 * PD14 - PIN14                     (analog).
 * PD15 - PIN15                     (analog).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ANALOG(GPIOD_PIN0)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN1)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN2)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN3)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN4)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN5)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN6)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN7)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN8)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN9)      |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN10)     |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN11)     |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN12)     |  \
                                     PIN_MODE_ALTERNATE(GPIOD_PIN13)  |  \
                                     PIN_MODE_ALTERNATE(GPIOD_PIN14)  |  \
                                     PIN_MODE_ANALOG(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_PIN0)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN1)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN2)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN3)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN4)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN5)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN6)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN7)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN8)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN9)      |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN10)     |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN11)     |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN12)     |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN13)     |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN14)     |  \
                                     PIN_OSPEED_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_PIN0)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN1)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN2)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN5)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN9)   |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10)  |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11)  |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN12)  |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN13)  |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN14)  |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN1)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN2)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN3)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN4)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN5)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN6)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN7)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN8)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN9)         |  \
                                     PIN_ODR_HIGH(GPIOD_PIN10)        |  \
                                     PIN_ODR_HIGH(GPIOD_PIN11)        |  \
                                     PIN_ODR_HIGH(GPIOD_PIN12)        |  \
                                     PIN_ODR_HIGH(GPIOD_PIN13)        |  \
                                     PIN_ODR_HIGH(GPIOD_PIN14)        |  \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0U)      |  \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0U)      |  \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0U)      |  \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0U)      |  \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0U)      |  \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0U)      |  \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0U)      |  \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0U)      |  \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0U)      |  \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0U)     |  \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0U)     |  \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0U)     |  \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0U)     |  \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0U)     |  \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (analog).
 * PE1  - PIN1                      (analog).
 * PE2  - PIN2                      (analog).
 * PE3  - PIN3                      (analog).
 * PE4  - PIN4                      (analog).
 * PE5  - PIN5                      (analog).
 * PE6  - PIN6                      (analog).
 * PE7  - PIN7                      (analog).
 * PE8  - PIN8                      (analog).
 * PE9  - PIN9                      (analog).
 * PE10 - PIN10                     (analog).
 * PE11 - PIN11                     (analog).
 * PE12 - PIN12                     (analog).
 * PE13 - PIN13                     (analog).
 * PE14 - PIN14                     (analog).
 * PE15 - PIN15                     (analog).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ANALOG(GPIOE_PIN0)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN1)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN2)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN3)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN4)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN5)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN6)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN7)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN8)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN9)      |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN10)     |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN11)     |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN12)     |  \
                                     PIN_MODE_ALTERNATE(GPIOE_PIN13)  |  \
                                     PIN_MODE_ALTERNATE(GPIOE_PIN14)  |  \
                                     PIN_MODE_ANALOG(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_PIN0)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN1)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN2)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN3)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN4)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN5)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN6)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN7)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN8)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN9)      |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN10)     |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN11)     |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN12)     |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN13)     |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN14)     |  \
                                     PIN_OSPEED_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_PIN0)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN1)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN7)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN8)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN9)   |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN10)  |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN11)  |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN12)  |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN13)  |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN14)  |  \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN1)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN2)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN3)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN4)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN5)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN6)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN7)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN8)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN9)         |  \
                                     PIN_ODR_HIGH(GPIOE_PIN10)        |  \
                                     PIN_ODR_HIGH(GPIOE_PIN11)        |  \
                                     PIN_ODR_HIGH(GPIOE_PIN12)        |  \
                                     PIN_ODR_HIGH(GPIOE_PIN13)        |  \
                                     PIN_ODR_HIGH(GPIOE_PIN14)        |  \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0U)      |  \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0U)      |  \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0U)      |  \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0U)      |  \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0U)      |  \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0U)      |  \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0U)      |  \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0U)      |  \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0U)      |  \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0U)     |  \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0U)     |  \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0U)     |  \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0U)     |  \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0U)     |  \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0U))

/*
 * GPIOF setup:
 *
! * PF0  - PIN0                      (analog).
 * PF1  - PIN1                      (analog).
 * PF2  - PIN2                      (analog).
 * PF3  - PIN3                      (analog).
 * PF4  - PIN4                      (analog).
 * PF5  - PIN5                      (analog).
 * PF6  - PIN6                      (analog).
 * PF7  - PIN7                      (analog).
 * PF8  - PIN8                      (analog).
 * PF9  - PIN9                      (analog).
 * PF10 - PIN10                     (analog).
 * PF11 - PIN11                     (analog).
 * PF12 - PIN12                     (analog).
 * PF13 - PIN13                     (analog).
 * PF14 - PIN14                     (analog).
 * PF15 - PIN15                     (analog).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ANALOG(GPIOF_PIN0)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN1)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN2)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN3)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN4)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN5)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN6)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN7)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN8)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN9)      |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN10)     |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN11)     |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN12)     |  \
                                     PIN_MODE_ALTERNATE(GPIOF_PIN13)  |  \
                                     PIN_MODE_ALTERNATE(GPIOF_PIN14)  |  \
                                     PIN_MODE_ANALOG(GPIOF_PIN15))
#define VAL_GPIOF_OTYPFR            (PIN_OTYPF_PUSHPULL(GPIOF_PIN0)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN1)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN2)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN3)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN4)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN5)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN6)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN7)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN8)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN9)   |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN10)  |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN11)  |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN12)  |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN13)  |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN14)  |  \
                                     PIN_OTYPF_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPFEDR           (PIN_OSPFED_HIGH(GPIOF_PIN0)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN1)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN2)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN3)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN4)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN5)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN6)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN7)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN8)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN9)      |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN10)     |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN11)     |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN12)     |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN13)     |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN14)     |  \
                                     PIN_OSPFED_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_PUPFR             (PIN_PUPFR_FLOATING(GPIOF_PIN0)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN1)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN2)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN3)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN4)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN5)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN6)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN7)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN8)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN9)   |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN10)  |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN11)  |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN12)  |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN13)  |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN14)  |  \
                                     PIN_PUPFR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN1)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN2)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN3)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN4)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN5)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN6)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN7)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN8)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN9)         |  \
                                     PIN_ODR_HIGH(GPIOF_PIN10)        |  \
                                     PIN_ODR_HIGH(GPIOF_PIN11)        |  \
                                     PIN_ODR_HIGH(GPIOF_PIN12)        |  \
                                     PIN_ODR_HIGH(GPIOF_PIN13)        |  \
                                     PIN_ODR_HIGH(GPIOF_PIN14)        |  \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0U)      |  \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0U)      |  \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U)      |  \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U)      |  \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U)      |  \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U)      |  \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U)      |  \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U)      |  \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U)      |  \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U)     |  \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U)     |  \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U)     |  \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U)     |  \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U)     |  \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))

/*
 * GPIOG setup:
 *
 * PH0  - PIN0                      (analog).
 * PH1  - PIN1                      (analog).
 * PH2  - PIN2                      (analog).
 * PH3  - PIN3                      (analog).
 * PH4  - PIN4                      (analog).
 * PH5  - PIN5                      (analog).
 * PH6  - PIN6                      (analog).
 * PH7  - PIN7                      (analog).
 * PH8  - PIN8                      (analog).
 * PH9  - PIN9                      (analog).
 * PH10 - PIN10                     (analog).
 * PH11 - PIN11                     (analog).
 * PH12 - PIN12                     (analog).
 * PH13 - PIN13                     (analog).
 * PH14 - PIN14                     (analog).
 * PH15 - PIN15                     (analog).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_ANALOG(GPIOG_PIN0)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN1)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN2)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN3)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN4)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN5)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN6)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN7)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN8)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN9)      |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN10)     |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN11)     |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN12)     |  \
                                     PIN_MODE_ALTERNATE(GPIOG_PIN13)  |  \
                                     PIN_MODE_ALTERNATE(GPIOG_PIN14)  |  \
                                     PIN_MODE_ANALOG(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPHEDR           (PIN_OSPHED_HIGH(GPIOG_PIN0)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN1)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN2)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN3)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN4)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN5)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN6)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN7)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN8)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN9)      |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN10)     |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN11)     |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN12)     |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN13)     |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN14)     |  \
                                     PIN_OSPHED_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_PUPHR             (PIN_PUPHR_FLOATING(GPIOG_PIN0)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN1)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN2)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN3)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN4)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN5)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN6)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN7)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN8)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN9)   |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN10)  |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN11)  |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN12)  |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN13)  |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN14)  |  \
                                     PIN_PUPHR_FLOATING(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN1)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN2)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN3)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN4)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN5)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN6)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN7)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN8)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN9)         |  \
                                     PIN_ODR_HIGH(GPIOG_PIN10)        |  \
                                     PIN_ODR_HIGH(GPIOG_PIN11)        |  \
                                     PIN_ODR_HIGH(GPIOG_PIN12)        |  \
                                     PIN_ODR_HIGH(GPIOG_PIN13)        |  \
                                     PIN_ODR_HIGH(GPIOG_PIN14)        |  \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0U)      |  \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0U)      |  \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0U)      |  \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U)      |  \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U)      |  \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U)      |  \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0U)      |  \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U)      |  \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0U)      |  \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U)     |  \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0U)     |  \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U)     |  \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0U)     |  \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U)     |  \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - PIN2                      (analog).
 * PH3  - PIN3                      (analog).
 * PH4  - PIN4                      (analog).
 * PH5  - PIN5                      (analog).
 * PH6  - PIN6                      (analog).
 * PH7  - PIN7                      (analog).
 * PH8  - PIN8                      (analog).
 * PH9  - PIN9                      (analog).
 * PH10 - PIN10                     (analog).
 * PH11 - PIN11                     (analog).
 * PH12 - PIN12                     (analog).
 * PH13 - PIN13                     (analog).
 * PH14 - PIN14                     (analog).
 * PH15 - PIN15                     (analog).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN)       |  \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT)      |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN2)        |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN3)        |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN4)        |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN5)        |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN6)        |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN7)        |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN8)        |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN9)        |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN10)       |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN11)       |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN12)       |  \
                                     PIN_MODE_ALTERNATE(GPIOH_PIN13)    |  \
                                     PIN_MODE_ALTERNATE(GPIOH_PIN14)    |  \
                                     PIN_MODE_ANALOG(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN)   |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT)  |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2)     |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3)     |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4)     |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5)     |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6)     |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7)     |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8)     |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9)     |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10)    |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11)    |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12)    |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13)    |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14)    |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN)      |  \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT)     |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN2)        |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN3)        |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN4)        |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN5)        |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN6)        |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN7)        |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN8)        |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN9)        |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN10)       |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN11)       |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN12)       |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN13)       |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN14)       |  \
                                     PIN_OSPEED_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN)   |  \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT)  |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2)     |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3)     |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4)     |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN5)     |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN6)     |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN7)     |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN8)     |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN9)     |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN10)    |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN11)    |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12)    |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13)    |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN14)    |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN)         |  \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT)        |  \
                                     PIN_ODR_HIGH(GPIOH_PIN2)           |  \
                                     PIN_ODR_HIGH(GPIOH_PIN3)           |  \
                                     PIN_ODR_HIGH(GPIOH_PIN4)           |  \
                                     PIN_ODR_HIGH(GPIOH_PIN5)           |  \
                                     PIN_ODR_HIGH(GPIOH_PIN6)           |  \
                                     PIN_ODR_HIGH(GPIOH_PIN7)           |  \
                                     PIN_ODR_HIGH(GPIOH_PIN8)           |  \
                                     PIN_ODR_HIGH(GPIOH_PIN9)           |  \
                                     PIN_ODR_HIGH(GPIOH_PIN10)          |  \
                                     PIN_ODR_HIGH(GPIOH_PIN11)          |  \
                                     PIN_ODR_HIGH(GPIOH_PIN12)          |  \
                                     PIN_ODR_HIGH(GPIOH_PIN13)          |  \
                                     PIN_ODR_HIGH(GPIOH_PIN14)          |  \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U)      |  \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U)     |  \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U)        |  \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U)        |  \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U)        |  \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U)        |  \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U)        |  \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U)        |  \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U)        |  \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U)       |  \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U)       |  \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U)       |  \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U)       |  \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U)       |  \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (analog).
 * PI1  - PIN1                      (analog).
 * PI2  - PIN2                      (analog).
 * PI3  - PIN3                      (analog).
 * PI4  - PIN4                      (analog).
 * PI5  - PIN5                      (analog).
 * PI6  - PIN6                      (analog).
 * PI7  - PIN7                      (analog).
 * PI8  - PIN8                      (analog).
 * PI9  - PIN9                      (analog).
 * PI10 - PIN10                     (analog).
 * PI11 - PIN11                     (analog).
 * PI12 - PIN12                     (analog).
 * PI13 - PIN13                     (analog).
 * PI14 - PIN14                     (analog).
 * PI15 - PIN15                     (analog).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_ANALOG(GPIOI_PIN0)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN1)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN2)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN3)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN4)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN5)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN6)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN7)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN8)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN9)      |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN10)     |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN11)     |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN12)     |  \
                                     PIN_MODE_ALTERNATE(GPIOI_PIN13)  |  \
                                     PIN_MODE_ALTERNATE(GPIOI_PIN14)  |  \
                                     PIN_MODE_ANALOG(GPIOI_PIN15))
#define VAL_GPIOI_OTYPIR            (PIN_OTYPI_PUSHPULL(GPIOI_PIN0)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN1)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN2)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN3)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN4)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN5)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN6)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN7)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN8)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN9)   |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN10)  |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN11)  |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN12)  |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN13)  |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN14)  |  \
                                     PIN_OTYPI_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPIEDR           (PIN_OSPIED_HIGH(GPIOI_PIN0)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN1)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN2)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN3)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN4)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN5)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN6)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN7)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN8)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN9)      |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN10)     |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN11)     |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN12)     |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN13)     |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN14)     |  \
                                     PIN_OSPIED_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_PUPIR             (PIN_PUPIR_FLOATING(GPIOI_PIN0)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN1)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN2)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN3)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN4)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN5)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN6)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN7)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN8)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN9)   |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN10)  |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN11)  |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN12)  |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN13)  |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN14)  |  \
                                     PIN_PUPIR_FLOATING(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN1)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN2)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN3)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN4)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN5)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN6)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN7)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN8)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN9)         |  \
                                     PIN_ODR_HIGH(GPIOI_PIN10)        |  \
                                     PIN_ODR_HIGH(GPIOI_PIN11)        |  \
                                     PIN_ODR_HIGH(GPIOI_PIN12)        |  \
                                     PIN_ODR_HIGH(GPIOI_PIN13)        |  \
                                     PIN_ODR_HIGH(GPIOI_PIN14)        |  \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0U)      |  \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0U)      |  \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U)      |  \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U)      |  \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U)      |  \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U)      |  \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0U)      |  \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U)      |  \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0U)      |  \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U)     |  \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U)     |  \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U)     |  \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U)     |  \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U)     |  \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
