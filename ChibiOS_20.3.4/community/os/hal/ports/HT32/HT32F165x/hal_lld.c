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

void ht32_clock_init(void) {
    // Enable backup domain. Needed for USB
    CKCU->LPCR = CKCU_LPCR_BKISO;
    CKCU->APBCCR1 |= CKCU_APBCCR1_BKPREN;
    while (PWRCU->BAKTEST != 0x27);

#if HT32_CKCU_SW == CKCU_GCCR_SW_HSE
    // Enable HSE
    CKCU->GCCR |= CKCU_GCCR_HSEEN;
    while ((CKCU->GCSR & CKCU_GCSR_HSERDY) == 0); // wait for HSE ready
#endif

#if HT32_CKCU_SW == CKCU_GCCR_SW_PLL
    // Configure PLL
    #if HT32_PLL_USE_HSE == TRUE
        CKCU->GCFGR &= ~CKCU_GCFGR_PLLSRC; // HSE as PLL source
    #else
        CKCU->GCFGR |= CKCU_GCFGR_PLLSRC; // HSI as PLL source
    #endif
    CKCU->PLLCFGR = ((HT32_PLL_FBDIV & 0x3F) << 23) | ((HT32_PLL_OTDIV & 0x3) << 21);
    CKCU->GCCR |= CKCU_GCCR_PLLEN; // enable PLL
    while ((CKCU->GCSR & CKCU_GCSR_PLLRDY) == 0); // wait for PLL ready
#endif

    // flash wait states for core clock frequencies
#if HT32_CK_AHB_FREQUENCY > 48000000
    FMC->CFCR = (FMC->CFCR & ~FMC_CFCR_WAIT_MASK) | FMC_CFCR_WAIT_2;
#elif HT32_CK_AHB_FREQUENCY > 24000000
    FMC->CFCR = (FMC->CFCR & ~FMC_CFCR_WAIT_MASK) | FMC_CFCR_WAIT_1;
#else
    FMC->CFCR = (FMC->CFCR & ~FMC_CFCR_WAIT_MASK) | FMC_CFCR_WAIT_0;
#endif

    // AHB prescaler
#if HT32_AHB_PRESCALER == 1 || HT32_AHB_PRESCALER == 2
    CKCU->AHBCFGR = (CKCU->AHBCFGR & ~CKCU_AHBCFGR_AHBPRE_MASK) | (HT32_AHB_PRESCALER - 1);
#elif HT32_AHB_PRESCALER == 4
    CKCU->AHBCFGR = (CKCU->AHBCFGR & ~CKCU_AHBCFGR_AHBPRE_MASK) | (2);
#elif HT32_AHB_PRESCALER == 8
    CKCU->AHBCFGR = (CKCU->AHBCFGR & ~CKCU_AHBCFGR_AHBPRE_MASK) | (3);
#else
#error "Invalid AHB_PRESCALER value"
#endif

    // Clock switch
    CKCU->GCCR = (CKCU->GCCR & ~CKCU_GCCR_SW_MASK) | HT32_CKCU_SW;
    while ((CKCU->GCCR & CKCU_GCCR_SW_MASK) != HT32_CKCU_SW); // wait for clock switch

    // HSI is needed for flash erase/write for some reason.
    // Only disable if you will not need to erase/write memory
    // with your debug probe after this firmware has booted.
    // /* CKCU->GCCR &= ~CKCU_GCCR_HSIEN; */

#if defined(HT32F1653_4)
    // Peripheral prescalers are not available on HT32F1655/6
    // So make sure all prescalers are 1x on HT32F1653/4
    CKCU->APBPCSR0 = 0;
    CKCU->APBPCSR1 = 0;
#endif
}

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {
    ht32_clock_init();
}

/** @} */
