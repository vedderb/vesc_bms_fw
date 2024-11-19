/*
    Copyright (C) 2021 Westberry Technology (ChangZhou) Corp., Ltd

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
 * @file    WB32F3G71xx/hal_lld.h
 * @brief   WB32F3G71xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - WB32_LSECLK.
 *          - WB32_LSE_BYPASS (optionally).
 *          - WB32_HSECLK.
 *          - WB32_HSE_BYPASS (optionally).
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "wb32_registry.h"
#include "wb32_tim.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification
 * @{
 */
#define PLATFORM_NAME                          "WB32F3G71xx"

/**
 * @brief   Sub-family identifier.
 */
#if !defined(WB32F3G71xx) || defined(__DOXYGEN__)
#define WB32F3G71xx                               TRUE
#endif
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */

/**
 * @brief   Maximum HSE clock frequency.
 */
#define WB32_HSECLK_MAX                        16000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define WB32_HSECLK_MIN                        4000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define WB32_LSECLK_MAX                        1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define WB32_LSECLK_MIN                        32768

/**
 * @brief   Maximum PLLs input clock frequency.
 */
#define WB32_PLLIN_MAX                         16000000

/**
 * @brief   Minimum PLLs input clock frequency.
 */
#define WB32_PLLIN_MIN                         2000000

/**
 * @brief   Maximum PLL output clock frequency.
 */
#define WB32_PLLOUT_MAX                        96000000

/**
 * @brief   Minimum PLL output clock frequency.
 */
#define WB32_PLLOUT_MIN                        48000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define WB32_PCLK1_MAX                         96000000

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define WB32_PCLK2_MAX                         96000000
/** @} */


/**
 * @name    RCC_MAINCLKSRC register bits definitions
 * @{
 */
#define WB32_MAINCLKSRC_MHSI                   (0)
#define WB32_MAINCLKSRC_FHSI                   (1)
#define WB32_MAINCLKSRC_PLL                    (2)
#define WB32_MAINCLKSRC_HSE                    (3)
/** @} */

/**
 * @name    RCC_PLLSRC register bits definitions
 * @{
 */
#define WB32_PLLSRC_MHSI                       (0x0U)
#define WB32_PLLSRC_HSE                        (0x1U)
/** @} */

/**
 * @name    RCC_USBPRE register bits definitions
 * @{
 */
#define WB32_USBPRE_MASK                       (0x3U << 1)
#define WB32_USBPRE_DIV1                       (0x0U)
#define WB32_USBPRE_DIV1P5                     (0x5U)
#define WB32_USBPRE_DIV2                       (0x1U)
#define WB32_USBPRE_DIV3                       (0x3U)
/** @} */


/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the PWR/RCC initialization in the HAL.
 */
#if !defined(WB32_NO_INIT) || defined(__DOXYGEN__)
#define WB32_NO_INIT                           FALSE
#endif

/**
 * @brief   Enables or disables the MHSI clock source.
 */
#if !defined(WB32_MHSI_ENABLED) || defined(__DOXYGEN__)
#define WB32_MHSI_ENABLED                      TRUE
#endif

/**
 * @brief   Enables or disables the FHSI clock source.
 */
#if !defined(WB32_FHSI_ENABLED) || defined(__DOXYGEN__)
#define WB32_FHSI_ENABLED                      FALSE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 */
#if !defined(WB32_LSI_ENABLED) || defined(__DOXYGEN__)
#define WB32_LSI_ENABLED                       FALSE
#endif

/**
 * @brief   Enables or disables the HSE clock source.
 */
#if !defined(WB32_HSE_ENABLED) || defined(__DOXYGEN__)
#define WB32_HSE_ENABLED                       FALSE
#endif

/**
 * @brief   Enables or disables the LSE clock source.
 */
#if !defined(WB32_LSE_ENABLED) || defined(__DOXYGEN__)
#define WB32_LSE_ENABLED                       FALSE
#endif

/**
 * @brief   Enables or disables the PLL clock source.
 */
#if !defined(WB32_PLL_ENABLED) || defined(__DOXYGEN__)
#define WB32_PLL_ENABLED                       FALSE
#endif

/**
 * @brief   Main clock source selection.
 * @note    If the selected clock source is not the PLL then the PLL is not
 *          initialized and started.
 * @note    The default value is calculated for a 96MHz system clock from
 *          a 8MHz crystal using the PLL.
 */
#if !defined(WB32_MAINCLKSRC) || defined(__DOXYGEN__)
#define WB32_MAINCLKSRC                        WB32_MAINCLKSRC_PLL
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 */
#if !defined(WB32_PLLSRC) || defined(__DOXYGEN__)
#define WB32_PLLSRC                            WB32_PLLSRC_HSE
#endif

/**
 * @brief   Crystal PLL pre-divider.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 */
#if !defined(WB32_PLLDIV_VALUE) || defined(__DOXYGEN__)
#define WB32_PLLDIV_VALUE                      1
#endif

/**
 * @brief   PLL multiplier value.
 * @note    The allowed value is 12, 16, 20, 24.
 */
#if !defined(WB32_PLLMUL_VALUE) || defined(__DOXYGEN__)
#define WB32_PLLMUL_VALUE                      12
#endif

/**
 * @brief   AHB prescaler value.
 * @note    The default value is calculated for a 96MHz system clock from
 *          a 8MHz crystal using the PLL.
 */
#if !defined(WB32_HPRE) || defined(__DOXYGEN__)
#define WB32_HPRE                              1
#endif

/**
 * @brief   APB1 prescaler value.
 */
#if !defined(WB32_PPRE1) || defined(__DOXYGEN__)
#define WB32_PPRE1                             1
#endif

/**
 * @brief   APB2 prescaler value.
 */
#if !defined(WB32_PPRE2) || defined(__DOXYGEN__)
#define WB32_PPRE2                             1
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/



/*
 * Configuration-related checks.
 */
#if !defined(WB32F3G71xx_MCUCONF)
#error "Using a wrong mcuconf.h file, WB32F3G71xx_MCUCONF not defined"
#endif

/*
 * MHSI related checks.
 */
#if WB32_MHSI_ENABLED
#else /* !WB32_MHSI_ENABLED */

#if (WB32_MAINCLKSRC == WB32_MAINCLKSRC_MHSI)
#error "MHSI not enabled, required by WB32_MAINCLKSRC"
#endif

#if (WB32_MAINCLKSRC == WB32_MAINCLKSRC_PLL) &&                             \
  (WB32_PLLSRC == WB32_PLLSRC_MHSI)
#error "MHSI not enabled, required by WB32_MAINCLKSRC and WB32_PLLSRC"
#endif

#endif

/*
 * FHSI related checks.
 */
#if WB32_FHSI_ENABLED
#else /* !WB32_FHSI_ENABLED */

#if (WB32_MAINCLKSRC == WB32_MAINCLKSRC_FHSI)
#error "FHSI not enabled, required by WB32_MAINCLKSRC"
#endif

#endif /* !WB32_FHSI_ENABLED */

/*
 * HSE related checks.
 */
#if WB32_HSE_ENABLED

#if WB32_HSECLK == 0
#error "HSE frequency not defined"
#elif (WB32_HSECLK < WB32_HSECLK_MIN) || (WB32_HSECLK > WB32_HSECLK_MAX)
#error "WB32_HSECLK outside acceptable range (WB32_HSECLK_MIN...WB32_HSECLK_MAX)"
#endif

#else /* !WB32_HSE_ENABLED */

#if (WB32_MAINCLKSRC == WB32_MAINCLKSRC_HSE)
#error "HSE not enabled, required by WB32_MAINCLKSRC"
#endif

#if ((WB32_MAINCLKSRC == WB32_MAINCLKSRC_PLL) && (WB32_PLLSRC == WB32_PLLSRC_HSE))
#error "HSE not enabled, required by WB32_MAINCLKSRC and WB32_PLLSRC"
#endif

#endif /* !WB32_HSE_ENABLED */

/*
 * LSI related checks.
 */
#if WB32_LSI_ENABLED
#else /* !WB32_LSI_ENABLED */
#endif /* !WB32_LSI_ENABLED */

/*
 * LSE related checks.
 */
#if WB32_LSE_ENABLED

#if (WB32_LSECLK == 0)
#error "LSE frequency not defined"
#endif

#if (WB32_LSECLK < WB32_LSECLK_MIN) || (WB32_LSECLK > WB32_LSECLK_MAX)
#error "WB32_LSECLK outside acceptable range (WB32_LSECLK_MIN...WB32_LSECLK_MAX)"
#endif

#else /* !WB32_LSE_ENABLED */
#endif /* !WB32_LSE_ENABLED */

/**
 * @brief   PLLDIV field.
 */
#if ((WB32_PLLDIV_VALUE >= 1) && (WB32_PLLDIV_VALUE <= 16)) ||              \
    defined(__DOXYGEN__)
#define WB32_PLLDIV                            WB32_PLLDIV_VALUE
#else
#error "invalid WB32_PLLDIV_VALUE value specified"
#endif

/**
 * @brief   PLLMUL field.
 */
#if (WB32_PLLMUL_VALUE == 12) || (WB32_PLLMUL_VALUE == 16) ||               \
    (WB32_PLLMUL_VALUE == 20) || (WB32_PLLMUL_VALUE == 24) ||               \
    defined(__DOXYGEN__)
#define WB32_PLLMUL                            WB32_PLLMUL_VALUE
#else
#error "invalid WB32_PLLMUL_VALUE value specified"
#endif

/**
 * @brief   PLL input clock frequency.
 */
#if (WB32_PLLSRC == WB32_PLLSRC_HSE) || defined(__DOXYGEN__)
#define WB32_PLLCLKIN                          (WB32_HSECLK / WB32_PLLDIV_VALUE)
#elif (WB32_PLLSRC == WB32_PLLSRC_MHSI)
#define WB32_PLLCLKIN                          (8000000 / WB32_PLLDIV_VALUE)
#else
#error "invalid WB32_PLLSRC value specified"
#endif

/* PLL input frequency range check.*/
#if (WB32_PLLCLKIN < WB32_PLLIN_MIN) || (WB32_PLLCLKIN > WB32_PLLIN_MAX)
#error "WB32_PLLCLKIN outside acceptable range (WB32_PLLIN_MIN...WB32_PLLIN_MAX)"
#endif

/**
 * @brief   PLL output clock frequency.
 */
#define WB32_PLLCLKOUT                         (WB32_PLLCLKIN * WB32_PLLMUL_VALUE)

/* PLL output frequency range check.*/
#if (WB32_PLLCLKOUT < WB32_PLLOUT_MIN) || (WB32_PLLCLKOUT > WB32_PLLOUT_MAX)
#error "WB32_PLLCLKOUT outside acceptable range (WB32_PLLOUT_MIN...WB32_PLLOUT_MAX)"
#endif

/**
 * @brief   System clock source.
 */
#if (WB32_MAINCLKSRC == WB32_MAINCLKSRC_PLL) || defined(__DOXYGEN__)
#define WB32_MAINCLK                           WB32_PLLCLKOUT
#elif (WB32_MAINCLKSRC == WB32_MAINCLKSRC_MHSI)
#define WB32_MAINCLK                           8000000
#elif (WB32_MAINCLKSRC == WB32_MAINCLKSRC_FHSI)
#define WB32_MAINCLK                           48000000
#elif (WB32_MAINCLKSRC == WB32_MAINCLKSRC_HSE)
#define WB32_MAINCLK                           WB32_HSECLK
#else
#error "invalid WB32_MAINCLKSRC value specified"
#endif

/**
 * @brief   AHB frequency.
 */
#if ((WB32_HPRE >= 1) && (WB32_HPRE <= 64)) ||                              \
    defined(__DOXYGEN__)
#define WB32_HCLK                              (WB32_MAINCLK / WB32_HPRE)
#else
#error "invalid WB32_HPRE value specified"
#endif


/**
 * @brief   APB1 frequency.
 */
#if ((WB32_PPRE1 >= 1) && (WB32_PPRE1 <= 64)) ||                            \
    defined(__DOXYGEN__)
#define WB32_PCLK1                             (WB32_MAINCLK / WB32_PPRE1)
#else
#error "invalid WB32_PPRE1 value specified"
#endif

/* APB1 frequency check.*/
#if WB32_PCLK1 > WB32_PCLK1_MAX
#error "WB32_PCLK1 exceeding maximum frequency (WB32_PCLK1_MAX)"
#endif

/**
 * @brief   APB2 frequency.
 */
#if ((WB32_PPRE2 >= 1) && (WB32_PPRE2 <= 64)) ||                            \
    defined(__DOXYGEN__)
#define WB32_PCLK2                             (WB32_MAINCLK / WB32_PPRE2)
#else
#error "invalid WB32_PPRE2 value specified"
#endif

/* APB2 frequency check.*/
#if WB32_PCLK2 > WB32_PCLK2_MAX
#error "WB32_PCLK2 exceeding maximum frequency (WB32_PCLK2_MAX)"
#endif

/**
 * @brief   USB frequency.
 */
#if (WB32_USBPRE == WB32_USBPRE_DIV1P5) || defined(__DOXYGEN__)
#define WB32_USBCLK                            ((WB32_MAINCLK * 2) / 3)
#elif (WB32_USBPRE == WB32_USBPRE_DIV1)
#define WB32_USBCLK                            WB32_MAINCLK
#elif (WB32_USBPRE == WB32_USBPRE_DIV2)
#define WB32_USBCLK                            (WB32_MAINCLK / 2)
#elif (WB32_USBPRE == WB32_USBPRE_DIV3)
#define WB32_USBCLK                            (WB32_MAINCLK / 3)
#else
#error "invalid WB32_USBPRE value specified"
#endif

/**
 * @brief   Timers 1, 2, 3, 4 clock.
 */
#define WB32_TIMCLK1                           WB32_PCLK1

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "wb32_isr.h"
#include "wb32_rcc.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void wb32_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
