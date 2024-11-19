/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio
              Copyright (C) 2020 Yaotian Feng

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

/**
 * @file    HT32F165x/ht32_registry.h
 * @brief   HT32F165x capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#pragma once


/**
 * @brief   Sub-family identifier.
 */
#if defined(HT32F52342) || defined(HT32F52352) || \
defined(__DOXYGEN__)
    #define HT32F523x2
#else
#error unknown/unsupported HT32 microcontroller
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

#if defined(HT32F523x2) || defined(__DOXYGEN__)

/**
 * @brief   Maximum system and core clock (f_SYS) frequency.
 */
#define HT32_SYSCLK_MAX         48000000L

/**
 * @brief   Maximum bus clock (f_BUS) frequency.
 */
#define HT32_BUSCLK_MAX         48000000L

/**
 * @brief   Maximum flash clock (f_FLASH) frequency.
 */
#define HT32_FLASHCLK_MAX       48000000L

/**
 * @name    HT32F52342x attributes
 * @{
 */

/* GPIO attributes.*/

#define HT32_NUM_GPIO               4
#define HT32_GPIO_INDEX_BITS        13
#define HT32_CCR_PAEN               CKCU_AHBCCR_PAEN

/* EXTI attributes */
#define HT32_HAS_EXTI               TRUE
#define HT32_NUM_EXTI               3
#define HT32_EVWUP_IRQ_VECTOR       Vector4C
#define HT32_EXTI0_1_IRQ_VECTOR     Vector50
#define HT32_EXTI2_3_IRQ_VECTOR     Vector54
#define HT32_EXTI4_15_IRQ_VECTOR    Vector58

/* BFTM attributes. */
#define HT32_BFTM0_IRQ_VECTOR       Vector84
#define HT32_BFTM1_IRQ_VECTOR       Vector88

/* I2C attributes.*/
#define HT32_HAS_I2C0               TRUE
#define HT32_I2C0_IRQ_VECTOR        Vector8C
#define HT32_HAS_I2C1               TRUE
#define HT32_I2C1_IRQ_VECTOR        Vector90

/* SPI attributes.*/
#define HT32_HAS_SPI0               TRUE
#define HT32_SPI0_IRQ_VECTOR        Vector94
#define HT32_HAS_SPI1               TRUE
#define HT32_SPI1_IRQ_VECTOR        Vector98

/* UART attributes.*/
#define HT32_HAS_USART0             TRUE
#define HT32_USART0_IRQ_VECTOR      Vector9C
#define HT32_HAS_USART1             TRUE
#define HT32_USART1_IRQ_VECTOR      VectorA0
#define HT32_HAS_UART0              TRUE
#define HT32_UART0_IRQ_VECTOR       VectorA4
#define HT32_HAS_UART1              TRUE
#define HT32_UART1_IRQ_VECTOR       VectorA8

/* USB attributes.*/
#define HT32_HAS_USB                TRUE
#define HT32_USB_IRQ_VECTOR         VectorB4
#define HT32_USB0_IS_USBOTG         FALSE
#define HT32_HAS_USB_CLOCK_RECOVERY FALSE

/** @} */

#endif /* defined(HT32F165x) */

/** @} */
