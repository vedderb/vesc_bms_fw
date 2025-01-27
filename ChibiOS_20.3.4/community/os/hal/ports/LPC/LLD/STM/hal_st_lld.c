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
 * @file    hal_st_lld.c
 * @brief   LPC11Uxx ST subsystem low level driver source.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
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

#if (OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC) || defined(__DOXYGEN__)
/**
 * @brief   System Timer vector.
 * @details This interrupt is used for system tick in periodic mode.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SysTick_Handler) {
    OSAL_IRQ_PROLOGUE();

    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();

    OSAL_IRQ_EPILOGUE();
}
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void) {
#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC
    #ifndef LPC_SYS_FREQUENCY
    #error "No LPC_SYS_FREQUENCY defined"
    #endif
    /* Periodic systick mode, the Cortex-Mx internal systick timer is used
     in this mode.*/
    SysTick->LOAD = (LPC_SYS_FREQUENCY / OSAL_ST_FREQUENCY) - 1;
    SysTick->CTRL =   SysTick_CTRL_CLKSOURCE_Msk |
    SysTick_CTRL_ENABLE_Msk |
    SysTick_CTRL_TICKINT_Msk;
    SysTick->VAL = 0;

    /* IRQ enabled.*/
    nvicSetSystemHandlerPriority(HANDLER_SYSTICK, LPC_ST_IRQ_PRIORITY);
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
