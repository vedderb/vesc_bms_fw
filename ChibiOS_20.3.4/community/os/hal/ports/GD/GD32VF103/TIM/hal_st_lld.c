/*
     ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio
     ChibiOS - Copyright (C) 2021 Stefan Kerkmann

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
 * @file    TIM/hal_st_lld.c
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
#define ST_ARR_INIT                         0xFFFFFFFFU
#else
#define ST_ARR_INIT                         0x0000FFFFU
#endif

#if GD32_ST_USE_TIMER == 1

#if !GD32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM1_IS_32BITS
#error "TIM1 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM1_HANDLER
#define ST_NUMBER                           GD32_TIM1_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rcuEnableTIM1(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM1_STOP

#elif GD32_ST_USE_TIMER == 2

#if !GD32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM2_IS_32BITS
#error "TIM2 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM2_HANDLER
#define ST_NUMBER                           GD32_TIM2_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rcuEnableTIM2(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP

#elif GD32_ST_USE_TIMER == 3

#if !GD32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM3_IS_32BITS
#error "TIM3 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM3_HANDLER
#define ST_NUMBER                           GD32_TIM3_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rcuEnableTIM3(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM3_STOP

#elif GD32_ST_USE_TIMER == 4

#if !GD32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM4_IS_32BITS
#error "TIM4 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM4_HANDLER
#define ST_NUMBER                           GD32_TIM4_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rcuEnableTIM4(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM4_STOP

#else
#error "GD32_ST_USE_TIMER specifies an unsupported timer"
#endif

#if ST_CLOCK_SRC % OSAL_ST_FREQUENCY != 0
#error "the selected ST frequency is not obtainable because integer rounding"
#endif

#if (ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1 > 0xFFFF
#error "the selected ST frequency is not obtainable because TIM timer prescaler limits"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

#define ST_HANDLER                          vector7
#define ST_NUMBER                           7

#if defined(GD32_CORE_CK)
#define SYSTICK_CK                          GD32_CORE_CK
#else
#define SYSTICK_CK                          GD32_HCLK
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

#if !defined(GD32_SYSTICK_SUPPRESS_ISR)
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
  GD32_ST_TIM->PSC    = (ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1;
  GD32_ST_TIM->CAR    = ST_ARR_INIT;
  GD32_ST_TIM->CHCTL0  = 0;
  GD32_ST_TIM->CHCV[0] = 0;
#if ST_LLD_NUM_ALARMS > 1
  GD32_ST_TIM->CHCV[1] = 0;
#endif
#if ST_LLD_NUM_ALARMS > 2
  GD32_ST_TIM->CHCV[2] = 0;
#endif
#if ST_LLD_NUM_ALARMS > 3
  GD32_ST_TIM->CHCV[3] = 0;
#endif
  GD32_ST_TIM->DMAINTEN   = 0;
  GD32_ST_TIM->CTL1    = 0;
  GD32_ST_TIM->SWEVG    = TIM_EGR_UG;
  GD32_ST_TIM->CTL0    = TIM_CR1_CEN;

#if !defined(GD32_SYSTICK_SUPPRESS_ISR)
  /* IRQ enabled.*/
  eclicEnableVector(ST_NUMBER, GD32_ST_IRQ_PRIORITY, GD32_ST_IRQ_TRIGGER);
#endif
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC
  /* Periodic systick mode, the RISC-V MTIME internal systick timer is used
     in this mode.*/
  SysTimer_SetCompareValue((SYSTICK_CK / OSAL_ST_FREQUENCY) - 1);
  SysTimer_SetLoadValue(0);
  /* IRQ enabled.*/
  eclicEnableVector(ST_NUMBER, GD32_ST_IRQ_PRIORITY, GD32_ST_IRQ_TRIGGER);
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */
}

/**
 * @brief   IRQ handling code.
 */
void st_lld_serve_interrupt(void) {
#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC
  /* Reload Timer */
  SysTimer_SetCompareValue((SYSTICK_CK / OSAL_ST_FREQUENCY) - 1);
  SysTimer_SetLoadValue(0);
#endif
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  uint32_t sr;
  gd32_tim_t *timp = GD32_ST_TIM;

  sr  = timp->INTF;
  sr &= timp->DMAINTEN & GD32_TIM_DIER_IRQ_MASK;
  timp->INTF = ~sr;

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
