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
 * @file    TIMv1/hal_st_lld.c
 * @brief   ST Driver subsystem low level driver code.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING

#if (OSAL_ST_RESOLUTION == 32)
#define ST_ARR_INIT                            0xFFFFFFFFU
#else
#define ST_ARR_INIT                            0x0000FFFFU
#endif

#if WB32_ST_USE_TIMER == 2

#if !WB32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !WB32_TIM2_IS_32BITS
#error "TIM2 is not a 32bits timer"
#endif

#define ST_HANDLER                             WB32_TIM2_IRQ_VECTOR
#define ST_NUMBER                              WB32_TIM2_NUMBER
#define ST_CLOCK_SRC                           WB32_TIMCLK1
#define ST_ENABLE_CLOCK()                      rccEnableTIM2()
#if defined(WB32F3G71xx)
#define ST_ENABLE_STOP()                       DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP
#endif

#elif WB32_ST_USE_TIMER == 3

#if !WB32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !WB32_TIM3_IS_32BITS
#error "TIM3 is not a 32bits timer"
#endif

#define ST_HANDLER                             WB32_TIM3_IRQ_VECTOR
#define ST_NUMBER                              WB32_TIM3_NUMBER
#define ST_CLOCK_SRC                           WB32_TIMCLK1
#define ST_ENABLE_CLOCK()                      rccEnableTIM3()
#if defined(WB32F3G71xx)
#define ST_ENABLE_STOP()                       DBGMCU->CR |= DBGMCU_CR_DBG_TIM3_STOP
#endif

#elif WB32_ST_USE_TIMER == 4

#if !WB32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !WB32_TIM4_IS_32BITS
#error "TIM4 is not a 32bits timer"
#endif

#define ST_HANDLER                             WB32_TIM4_IRQ_VECTOR
#define ST_NUMBER                              WB32_TIM4_NUMBER
#define ST_CLOCK_SRC                           WB32_TIMCLK1
#define ST_ENABLE_CLOCK()                      rccEnableTIM4()
#if defined(WB32F3G71xx)
#define ST_ENABLE_STOP()                       DBGMCU->CR |= DBGMCU_CR_DBG_TIM4_STOP
#endif

#else
#error "WB32_ST_USE_TIMER specifies an unsupported timer"
#endif

#if ST_CLOCK_SRC % OSAL_ST_FREQUENCY != 0
#error "the selected ST frequency is not obtainable because integer rounding"
#endif

#if (ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1 > 0xFFFF
#error "the selected ST frequency is not obtainable because TIM timer prescaler limits"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

#define ST_HANDLER                             SysTick_Handler

#if defined(WB32_CORE_CK)
#define SYSTICK_CK                             WB32_CORE_CK
#else
#define SYSTICK_CK                             WB32_HCLK
#endif

#if SYSTICK_CK % OSAL_ST_FREQUENCY != 0
#error "the selected ST frequency is not obtainable because integer rounding"
#endif

#if (SYSTICK_CK / OSAL_ST_FREQUENCY) - 1 > 0xFFFFFF
#error "the selected ST frequency is not obtainable because SysTick timer counter limits"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

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

#if !defined(WB32_SYSTICK_SUPPRESS_ISR)
/**
 * @brief   Interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ST_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  st_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void) {

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  /* Free running counter mode.*/

  /* Enabling timer clock.*/
  ST_ENABLE_CLOCK();

  /* Enabling the stop mode during debug for this timer.*/
  ST_ENABLE_STOP();

  /* Initializing the counter in free running mode.*/
  WB32_ST_TIM->PSC = (ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1;
  WB32_ST_TIM->ARR = ST_ARR_INIT;
  WB32_ST_TIM->CCMR1 = 0;
  WB32_ST_TIM->CCR[0] = 0;
#if ST_LLD_NUM_ALARMS > 1
  WB32_ST_TIM->CCR[1] = 0;
#endif
#if ST_LLD_NUM_ALARMS > 2
  WB32_ST_TIM->CCR[2] = 0;
#endif
#if ST_LLD_NUM_ALARMS > 3
  WB32_ST_TIM->CCR[3] = 0;
#endif
  WB32_ST_TIM->DIER = 0;
  WB32_ST_TIM->CR2 = 0;
  WB32_ST_TIM->EGR = TIM_EGR_UG;
  WB32_ST_TIM->CR1 = TIM_CR1_CEN;

#if !defined(WB32_SYSTICK_SUPPRESS_ISR)
  /* IRQ enabled.*/
  nvicEnableVector(ST_NUMBER, WB32_ST_IRQ_PRIORITY);
#endif
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC
  /* Periodic systick mode, the Cortex-Mx internal systick timer is
     used in this mode.*/
  SysTick->LOAD = (SYSTICK_CK / OSAL_ST_FREQUENCY) - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_ENABLE_Msk |
                  SysTick_CTRL_TICKINT_Msk;

  /* IRQ enabled.*/
  nvicSetSystemHandlerPriority(HANDLER_SYSTICK, WB32_ST_IRQ_PRIORITY);
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */
}

/**
 * @brief   IRQ handling code.
 */
void st_lld_serve_interrupt(void) {
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  uint32_t sr;
  wb32_tim_t *timp = WB32_ST_TIM;

  sr = timp->SR;
  sr &= timp->DIER & WB32_TIM_DIER_IRQ_MASK;
  timp->SR = ~sr;

  if ((sr & TIM_SR_CC1IF) != 0U)
#endif
  {
    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
  }
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
#if ST_LLD_NUM_ALARMS > 1
  if ((sr & TIM_SR_CC2IF) != 0U) {
    if (st_callbacks[2] != NULL) {
      st_callbacks[0](1U);
    }
  }
#endif
#if ST_LLD_NUM_ALARMS > 2
  if ((sr & TIM_SR_CC3IF) != 0U) {
    if (st_callbacks[2] != NULL) {
      st_callbacks[1](2U);
    }
  }
#endif
#if ST_LLD_NUM_ALARMS > 3
  if ((sr & TIM_SR_CC4IF) != 0U) {
    if (st_callbacks[2] != NULL) {
      st_callbacks[2](3U);
    }
  }
#endif
#endif
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
