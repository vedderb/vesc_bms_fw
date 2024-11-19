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
 * @file    hal_lld.c
 * @brief   PLATFORM HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"


/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void lpc_clock_init(void) {

    #if defined(LPC_USE_SYSOSC) && LPC_USE_SYSOSC != FALSE

    LPC_SYSCON->PDRUNCFG &= ~SYSCON_PDRUNCFG_SYSSOC_PD;
    LPC_SYSCON->SYSOSCCTRL = 0;

    #endif

    #if  LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_PLLOUT ||\
            LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_PLLIN
    // 1. Config PLL input clock
    LPC_SYSCON->SYSPLLCLKSEL = LPC_SYSPLLCLKSEL;
    // Switch Clock
    LPC_SYSCON->SYSPLLCLKUEN = 0;
    LPC_SYSCON->SYSPLLCLKUEN = SYSCON_SYSPLLCLKUEN_ENA;

    #if LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_PLLOUT

    // 2. Config PLL
    // Enable PLL power
    LPC_SYSCON->PDRUNCFG &= (~SYSCON_PDRUNCFG_SYSPLL_PD);
    // Apply PLL Config
    LPC_SYSCON->SYSPLLCTRL = (LPC_SYSPLL_PSEL_VAL << SYSCON_SYSPLLCTRL_PSEL_POS)
        | (LPC_SYSPLL_MSEL_VAL << SYSCON_SYSPLLCTRL_MSEL_POS);
    // Wait for PLLLock
    while(!(LPC_SYSCON->SYSPLLSTAT & SYSCON_SYSPLLSTAT_LOCK)){}
    #endif // LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_PLLOUT

    #endif /* LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_PLLOUT ||
              LPC_MAINCLKSEL == SYSCON_MAINCLKSEL_PLLIN */

    // Config SYSDIV
    LPC_SYSCON->SYSAHBCLKDIV = LPC_SYS_DIV & 0xFF;

    // Select Main Clock
    LPC_SYSCON->MAINCLKSEL = LPC_MAINCLKSEL;
    LPC_SYSCON->MAINCLKUEN = 0;
    LPC_SYSCON->MAINCLKUEN = SYSCON_MAINCLKUEN_ENA;
}

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {
    lpc_clock_init();
}

/** @} */
