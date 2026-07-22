/*
    Copyright (C) 2019 /u/KeepItUnder
    Copyright (C) 2020 Alex Lewontin

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

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING)

#if (OSAL_ST_RESOLUTION == 32)
#define ST_ARR_INIT                         0xFFFFFFFF
#else
#define ST_ARR_INIT                         0x0000FFFF
#endif

#if NUC123_ST_USE_TIMER == 2
#if (OSAL_ST_RESOLUTION == 32) && !NUC123_TIM2_IS_32BITS
#error "TIM2 is not a 32bits timer"
#endif

#if defined(NUC123_TIM2_IS_USED)
#error "ST requires TIM2 but the timer is already used"
#else
#define NUC123_TIM2_IS_USED
#endif

#define ST_HANDLER                          NUC123_TIM2_HANDLER
#define ST_NUMBER                           NUC123_TIM2_NUMBER
#define ST_USE_TIMER                        TIMER2

#elif NUC123_ST_USE_TIMER == 3
#if (OSAL_ST_RESOLUTION == 32) && !NUC123_TIM3_IS_32BITS
#error "TIM3 is not a 32bits timer"
#endif

#if defined(NUC123_TIM3_IS_USED)
#error "ST requires TIM3 but the timer is already used"
#else
#define NUC123_TIM3_IS_USED
#endif

#define ST_HANDLER                          NUC123_TIM3_HANDLER
#define ST_NUMBER                           NUC123_TIM3_NUMBER
#define ST_USE_TIMER                        TIMER3

#elif NUC123_ST_USE_TIMER == 4
#if (OSAL_ST_RESOLUTION == 32) && !NUC123_TIM4_IS_32BITS
#error "TIM4 is not a 32bits timer"
#endif

#if defined(NUC123_TIM4_IS_USED)
#error "ST requires TIM4 but the timer is already used"
#else
#define NUC123_TIM4_IS_USED
#endif

#define ST_HANDLER                          NUC123_TIM4_HANDLER
#define ST_NUMBER                           NUC123_TIM4_NUMBER
#define ST_USE_TIMER                        TIMER4

#else
#error "NUC123_ST_USE_TIMER specifies an unsupported timer"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

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

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) || defined(__DOXYGEN__)
/**
 * @brief   TIM2 interrupt handler.
 * @details This interrupt is used for system tick in free running mode.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ST_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Note, under rare circumstances an interrupt can remain latched even if
     the timer SR register has been cleared, in those cases the interrupt
     is simply ignored.*/
  if (NUC123_ST_TIM->TISR & TIMER_TISR_TIF_Msk) {

    // TIMER_ClearIntFlag(NUC123_ST_TIM);
    NUC123_ST_TIM->TISR = TIMER_TISR_TIF_Msk;

    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
  }

  OSAL_IRQ_EPILOGUE();
}
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

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
  
  st_lld_timer_open(ST_USE_TIMER, TIMER_MODE_CONTINUOUS, OSAL_ST_FREQUENCY);

  /* Initializing the counter in free running mode.*/
/*  NUC123_ST_TIM->PSC    = (ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1;
  NUC123_ST_TIM->ARR    = ST_ARR_INIT;
  NUC123_ST_TIM->CCMR1  = 0;
  NUC123_ST_TIM->CCR[0] = 0;
  NUC123_ST_TIM->DIER   = 0;
  NUC123_ST_TIM->CR2    = 0;
  NUC123_ST_TIM->EGR    = TIM_EGR_UG;
  NUC123_ST_TIM->CR1    = TIM_CR1_CEN;
*/
  /* IRQ enabled.*/
  nvicEnableVector(ST_NUMBER, NUC123_ST_IRQ_PRIORITY);

  // TIMER_EnableInt(ST_USE_TIMER);
  ST_USE_TIMER->TCSR |= TIMER_TCSR_IE_Msk;

  /* Start the Timer! */
  // TIMER_Start(ST_USE_TIMER);
  ST_USE_TIMER->TCSR |= TIMER_TCSR_CEN_Msk;

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC
  /* Periodic systick mode, the Cortex-Mx internal systick timer is used
     in this mode.*/
  SystemUnlockReg();
  CLK->CLKSEL0 &= ~CLK_CLKSEL0_STCLK_S_Msk;
  CLK->CLKSEL0 |= CLK_CLKSEL0_STCLK_S_HCLK_DIV2;
  LOCKREG();
  
  SysTick->LOAD = ((NUC123_HCLK / 2) / OSAL_ST_FREQUENCY) - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = (~SysTick_CTRL_CLKSOURCE_Msk) &
                  (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);

  /* IRQ enabled.*/
  nvicSetSystemHandlerPriority(HANDLER_SYSTICK, NUC123_ST_IRQ_PRIORITY);
  
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

}

#else /* OSAL_ST_MODE == OSAL_ST_MODE_NONE!!! */
  #error "We can't proceed without an OSAL_ST clock!"

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */



uint32_t st_lld_timer_getmoduleclock(TIMER_T *timer) {
    uint32_t clkSource;

    if (timer == TIMER0) {
        clkSource = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR0_S_Msk) >> CLK_CLKSEL1_TMR0_S_Pos;
    } else if (timer == TIMER1) {
        clkSource = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR1_S_Msk) >> CLK_CLKSEL1_TMR1_S_Pos;
    } else if (timer == TIMER2) {
        clkSource = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR2_S_Msk) >> CLK_CLKSEL1_TMR2_S_Pos;
    } else {
        clkSource = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR3_S_Msk) >> CLK_CLKSEL1_TMR3_S_Pos;
    }

    if (clkSource == 2) {
        return(SystemCoreClock);
    }

    switch (clkSource) {
      case 0: // Clock source is HXT
        return __HXT;
        break;
      case 5: // Clock source is LIRC
        return __LIRC;
        break;
      case 7: // Clock source is HIRC
        return __HIRC;
        break;
      default: // All other clock sources return 0
        return 0;
    }
}


uint32_t st_lld_timer_open(TIMER_T *timer, uint32_t tmrMode, uint32_t tmrFreq) {
    uint32_t tmrClk = st_lld_timer_getmoduleclock(timer);
    uint32_t cmpr = 0;
    uint32_t prescale = 0;

    // Fastest possible timer working freq is (tmrClk / 2). While cmpr = 2, pre-scale = 0.
    if (tmrFreq > (tmrClk / 2)) {
        cmpr = 2;
    } else {
        if (tmrClk >= 0x4000000) {
            prescale = 7;       // prescaler value - 1
            tmrClk >>= 3;
        } else if (tmrClk >= 0x2000000) {
            prescale = 3;       // prescaler value - 1
            tmrClk >>= 2;       // Divide Clock by 4 in preparation
        } else if (tmrClk >= 0x1000000) {
            prescale = 1;       // prescaler value - 1
            tmrClk >>= 1;       // Divide Clock by 2 in preparation
        }

        cmpr = tmrClk / tmrFreq;
    }

    timer->TCSR = tmrMode | prescale;
    timer->TCMPR = cmpr;

    return (tmrClk / (cmpr * (prescale + 1)));
}


void st_lld_timer_close(TIMER_T *timer) {
    timer->TCSR = 0;
    timer->TEXCON = 0;
}

/** @} */
