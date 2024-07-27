/*
    ChibiOS - Copyright (C) 2020 Yaotian Feng / Codetector

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
 * @file    LPC11Uxx/hal_lld.h
 * @brief   NXP LPC 11Uxx HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */
#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "LPC11Uxx.h"
#include "lpc_registry.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification macros
 * @{
 */
#define PLATFORM_NAME           "LPC"
/** @} */


/**
 * @name    PLATFORM configuration options
 * @{
 */
#define LPC_IRC_FREQUENCY   12000000UL // 12MHz
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(LPC11Uxx_MCUCONF) && !defined(LPC11U35_MCUCONF) && \
    !defined(LPC11U35_401_MCUCONF)
    #error "Using a wrong mcuconf.h file, LPC11Uxx_MCUCONF not defined"
#endif

#if defined(LPC_USE_SYSOSC) && LPC_USE_SYSOSC != FALSE && !defined(LPC_SYSOSC_FREQUENCY)
    #error "LPC_SYSOSC_FREQUENCY must be defined if LPC_USE_SYSOSC"
#endif

// SYSTEMPLL SEL
#if defined(LPC_SYSPLLCLKSEL) && LPC_SYSPLLCLKSEL == SYSCON_SYSPLLCLKSEL_IRC
    #define LPC_SYSPLLIN_FREQUENCY      (LPC_IRC_FREQUENCY)
#elif defined(LPC_SYSPLLCLKSEL) && LPC_SYSPLLCLKSEL == SYSCON_SYSPLLCLKSEL_SYSOSC
    #ifndef LPC_USE_SYSOSC
    #error "LPC_USE_SYSOSC must be defined when using SYSOSC"
    #endif
    #define LPC_SYSPLLIN_FREQUENCY      (LPC_SYSOSC_FREQUENCY)
#else
#error "LPC_SYSPLLCLKSEL must be one of SYSCON_SYSPLLCLKSEL_IRC, \
SYSCON_SYSPLLCLKSEL_SYSOSC"
#endif

#if defined(LPC_MAINCLKSEL) && LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_IRC
    // Using Internal OSC
    #define LPC_MAINCLK_FREQUENCY       (LPC_IRC_FREQUENCY)

#elif defined(LPC_MAINCLKSEL) &&  LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_PLLIN
    #define LPC_MAINCLK_FREQUENCY       (LPC_SYSPLLIN_FREQUENCY)
#elif defined(LPC_MAINCLKSEL) &&  LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_WATCHDOG

    #error "Unsupported clock select"

#elif defined(LPC_MAINCLKSEL) &&  LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_PLLOUT
    // PLL OUT
    #if defined(LPC_SYSPLL_MULT) && LPC_SYSPLL_MULT > 0 && LPC_SYSPLL_MULT <= 32 \
    && defined(LPC_SYSPLL_PDIV) && (LPC_SYSPLL_PDIV == 2 || LPC_SYSPLL_PDIV == 4 \
    || LPC_SYSPLL_PDIV == 8 || LPC_SYSPLL_PDIV == 16)

    #if LPC_SYSPLL_PDIV == 2
    #define LPC_SYSPLL_PSEL_VAL     0x0U
    #elif LPC_SYSPLL_PDIV == 4
    #define LPC_SYSPLL_PSEL_VAL     0x1U
    #elif LPC_SYSPLL_PDIV == 8
    #define LPC_SYSPLL_PSEL_VAL     0x2U
    #elif LPC_SYSPLL_PDIV == 16
    #define LPC_SYSPLL_PSEL_VAL     0x3U
    #else
    #error "INVALID PDIV VALUE"
    #endif //LPC_SYSPLL_PDIV == xx

    #if (LPC_SYSPLLIN_FREQUENCY * LPC_SYSPLL_MULT * LPC_SYSPLL_PDIV < 156000000UL) ||\
        (LPC_SYSPLLIN_FREQUENCY * LPC_SYSPLL_MULT * LPC_SYSPLL_PDIV > 320000000UL)
    #error "Please check the documentation about how to pick MULT and PDIV. \
current value gives a out of range CCO frequency"
    #endif

    #define LPC_MAINCLK_FREQUENCY (LPC_SYSPLLIN_FREQUENCY * LPC_SYSPLL_MULT)
    #define LPC_SYSPLL_MSEL_VAL     (LPC_SYSPLL_MULT - 1)

    #else
        #error "LPC_SYSPLL_MULT must be defined and btween 1 and 32, \
                LPC_SYSPLL_PDIV must be one of 2, 4, 8, 16."
    #endif
#else
    #error "Invalid LPC_MAINCLKSEL. Must be one of SYSCON_MAINCLKSEL_IRC, \
SYSCON_MAINCLKSEL_PLLIN, SYSCON_MAINCLKSEL_WATCHDOG, SYSCON_MAINCLKSEL_PLLOUT"
#endif

#if !defined(LPC_SYS_DIV) || LPC_SYS_DIV < 1 || LPC_SYS_DIV > 255
    #error "LPC_SYS_DIV must be between 1 and 255"
#else
    #define LPC_SYS_FREQUENCY   (LPC_MAINCLK_FREQUENCY / LPC_SYS_DIV)
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


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#include "nvic.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void lpc_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H_ */

/** @} */

