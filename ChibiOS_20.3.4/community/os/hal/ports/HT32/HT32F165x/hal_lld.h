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
 * @file    hal_lld.h
 * @brief   HT32 HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef _HAL_LLD_H_
#define _HAL_LLD_H_

#include "ht32_registry.h"
#include "nvic.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification macros
 * @{
 */
#define PLATFORM_NAME           "HT32"
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    PLATFORM configuration options
 * @{
 */
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(HT32F165x_MCUCONF) && !defined(HT32F1654_MCUCONF) && \
    !defined(HT32F1653_MCUCONF)
    #error "Using a wrong mcuconf.h file, HT32_MCUCONF not defined"
#endif

#define HT32_CK_HSI_FREQUENCY   8000000UL   // 8 MHz

#if HT32_CKCU_SW == CKCU_GCCR_SW_HSI
    #define HT32_CK_SYS_FREQUENCY   HT32_CK_HSI_FREQUENCY

#elif HT32_CKCU_SW == CKCU_GCCR_SW_HSE
    #if !defined(HT32_CK_HSE_FREQUENCY)
        #error "HT32_CK_HSE_FREQUENCY must be defined"
    #endif
    #define HT32_CK_SYS_FREQUENCY   HT32_CK_HSE_FREQUENCY

#elif HT32_CKCU_SW == CKCU_GCCR_SW_PLL
    #if !defined(HT32_PLL_USE_HSE)
        #error "HT32_PLL_USE_HSE must be defined"
    #endif

    #if HT32_PLL_USE_HSE == TRUE
        #if !defined(HT32_CK_HSE_FREQUENCY)
            #error "HT32_CK_HSE_FREQUENCY must be defined"
        #endif
        #define HT32_PLL_IN_FREQ    HT32_CK_HSE_FREQUENCY
    #else
        #define HT32_PLL_IN_FREQ    HT32_CK_HSI_FREQUENCY
    #endif

    #if !defined(HT32_PLL_FBDIV)
        #error "HT32_PLL_FBDIV must be defined"
    #endif
    #if !defined(HT32_PLL_OTDIV)
        #error "HT32_PLL_OTDIV must be defined"
    #endif

    #define HT32_CK_PLL_FREQUENCY   ((HT32_PLL_IN_FREQ * HT32_PLL_FBDIV) / (1 << HT32_PLL_OTDIV))
    #define HT32_CK_SYS_FREQUENCY   HT32_CK_PLL_FREQUENCY

#else
    #error "HT32_CKCU_SW is invalid"
#endif

#if !defined(HT32_AHB_PRESCALER)
    #define HT32_AHB_PRESCALER 1
#endif

// AHB clock
#define HT32_CK_AHB_FREQUENCY   (HT32_CK_SYS_FREQUENCY / HT32_AHB_PRESCALER) // Max 72 MHz
// SysTick (may also use HCLK)
#define HT32_STCLK_FREQUENCY    (HT32_CK_AHB_FREQUENCY / 8) // Max 8MHz
// CPU clock
#define HT32_HCLK_FREQUENCY     HT32_CK_AHB_FREQUENCY
// Peripheral clocks
#define HT32_PCLK_FREQUENCY     HT32_CK_AHB_FREQUENCY

// Checks
#if HT32_CK_SYS_FREQUENCY > 144000000
    #error "HT32 CK_SYS invalid"
#endif
#if HT32_CK_AHB_FREQUENCY > 72000000
    #error "HT32 CK_AHB invalid"
#endif

#if (HAL_USE_UART == TRUE || HAL_USE_SERIAL == TRUE)
    #define HT32_CK_USART_FREQUENCY (HT32_CK_AHB_FREQUENCY / HT32_USART_PRESCALER) // Max 72 MHz
    #if HT32_CK_USART_FREQUENCY > 72000000
        #error "HT32 CK_USART invalid"
    #endif
#endif

#if HAL_USE_USB == TRUE
    #if HT32_CKCU_SW != CKCU_GCCR_SW_PLL
        #error "HT32 USB requires PLL"
    #endif
    #define HT32_CK_USB_FREQUENCY (HT32_CK_PLL_FREQUENCY / HT32_USB_PRESCALER) // Max 48 MHz

    #if HT32_CK_USB_FREQUENCY > 48000000
        #error "HT32 CK_USB invalid"
    #endif
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void ht32_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* _HAL_LLD_H_ */

/** @} */
