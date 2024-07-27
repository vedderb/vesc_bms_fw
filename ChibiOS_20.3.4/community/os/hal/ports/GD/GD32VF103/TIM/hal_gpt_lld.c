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
 * @file    TIM/hal_gpt_lld.c
 * @brief   GD32 GPT subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "hal.h"

#if HAL_USE_GPT || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   GPTD1 driver identifier.
 * @note    The driver GPTD1 allocates the complex timer TIM0 when enabled.
 */
#if GD32_GPT_USE_TIM0 || defined(__DOXYGEN__)
GPTDriver GPTD1;
#endif

/**
 * @brief   GPTD2 driver identifier.
 * @note    The driver GPTD2 allocates the timer TIM1 when enabled.
 */
#if GD32_GPT_USE_TIM1 || defined(__DOXYGEN__)
GPTDriver GPTD2;
#endif

/**
 * @brief   GPTD3 driver identifier.
 * @note    The driver GPTD3 allocates the timer TIM2 when enabled.
 */
#if GD32_GPT_USE_TIM2 || defined(__DOXYGEN__)
GPTDriver GPTD3;
#endif

/**
 * @brief   GPTD4 driver identifier.
 * @note    The driver GPTD4 allocates the timer TIM3 when enabled.
 */
#if GD32_GPT_USE_TIM3 || defined(__DOXYGEN__)
GPTDriver GPTD4;
#endif

/**
 * @brief   GPTD5 driver identifier.
 * @note    The driver GPTD5 allocates the timer TIM4 when enabled.
 */
#if GD32_GPT_USE_TIM4 || defined(__DOXYGEN__)
GPTDriver GPTD5;
#endif

/**
 * @brief   GPTD6 driver identifier.
 * @note    The driver GPTD6 allocates the timer TIM5 when enabled.
 */
#if GD32_GPT_USE_TIM5 || defined(__DOXYGEN__)
GPTDriver GPTD6;
#endif

/**
 * @brief   GPTD7 driver identifier.
 * @note    The driver GPTD7 allocates the timer TIM6 when enabled.
 */
#if GD32_GPT_USE_TIM6 || defined(__DOXYGEN__)
GPTDriver GPTD7;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if GD32_GPT_USE_TIM0 || defined(__DOXYGEN__)
#if !defined(GD32_TIM0_SUPPRESS_ISR)
#if !defined(GD32_TIM0_UP_HANDLER)
#error "GD32_TIM0_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM0_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM0_SUPPRESS_ISR) */
#endif /* GD32_GPT_USE_TIM0 */

#if GD32_GPT_USE_TIM1 || defined(__DOXYGEN__)
#if !defined(GD32_TIM1_SUPPRESS_ISR)
#if !defined(GD32_TIM1_HANDLER)
#error "GD32_TIM1_HANDLER not defined"
#endif
/**
 * @brief   TIM1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM1_SUPPRESS_ISR) */
#endif /* GD32_GPT_USE_TIM1 */

#if GD32_GPT_USE_TIM2 || defined(__DOXYGEN__)
#if !defined(GD32_TIM2_SUPPRESS_ISR)
#if !defined(GD32_TIM2_HANDLER)
#error "GD32_TIM2_HANDLER not defined"
#endif
/**
 * @brief   TIM2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM2_SUPPRESS_ISR) */
#endif /* GD32_GPT_USE_TIM2 */

#if GD32_GPT_USE_TIM3 || defined(__DOXYGEN__)
#if !defined(GD32_TIM3_SUPPRESS_ISR)
#if !defined(GD32_TIM3_HANDLER)
#error "GD32_TIM3_HANDLER not defined"
#endif
/**
 * @brief   TIM3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM3_SUPPRESS_ISR) */
#endif /* GD32_GPT_USE_TIM3 */

#if GD32_GPT_USE_TIM4 || defined(__DOXYGEN__)
#if !defined(GD32_TIM4_SUPPRESS_ISR)
#if !defined(GD32_TIM4_HANDLER)
#error "GD32_TIM4_HANDLER not defined"
#endif
/**
 * @brief   TIM4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM4_SUPPRESS_ISR) */
#endif /* GD32_GPT_USE_TIM4 */

#if GD32_GPT_USE_TIM5 || defined(__DOXYGEN__)
#if !defined(GD32_TIM5_SUPPRESS_ISR)
#if !defined(GD32_TIM5_HANDLER)
#error "GD32_TIM5_HANDLER not defined"
#endif
/**
 * @brief   TIM5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD6);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM5_SUPPRESS_ISR) */
#endif /* GD32_GPT_USE_TIM5 */

#if GD32_GPT_USE_TIM6 || defined(__DOXYGEN__)
#if !defined(GD32_TIM6_SUPPRESS_ISR)
#if !defined(GD32_TIM6_HANDLER)
#error "GD32_TIM6_HANDLER not defined"
#endif
/**
 * @brief   TIM6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD7);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM6_SUPPRESS_ISR) */
#endif /* GD32_GPT_USE_TIM6 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {

#if GD32_GPT_USE_TIM0
  /* Driver initialization.*/
  GPTD1.tim = GD32_TIM0;
  gptObjectInit(&GPTD1);
#endif

#if GD32_GPT_USE_TIM1
  /* Driver initialization.*/
  GPTD2.tim = GD32_TIM1;
  gptObjectInit(&GPTD2);
#endif

#if GD32_GPT_USE_TIM2
  /* Driver initialization.*/
  GPTD3.tim = GD32_TIM2;
  gptObjectInit(&GPTD3);
#endif

#if GD32_GPT_USE_TIM3
  /* Driver initialization.*/
  GPTD4.tim = GD32_TIM3;
  gptObjectInit(&GPTD4);
#endif

#if GD32_GPT_USE_TIM4
  /* Driver initialization.*/
  GPTD5.tim = GD32_TIM4;
  gptObjectInit(&GPTD5);
#endif

#if GD32_GPT_USE_TIM5
  /* Driver initialization.*/
  GPTD6.tim = GD32_TIM5;
  gptObjectInit(&GPTD6);
#endif

#if GD32_GPT_USE_TIM6
  /* Driver initialization.*/
  GPTD7.tim = GD32_TIM6;
  gptObjectInit(&GPTD7);
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {
  uint16_t psc;

  if (gptp->state == GPT_STOP) {
    /* Clock activation.*/
#if GD32_GPT_USE_TIM0
    if (&GPTD1 == gptp) {
      rcuEnableTIM0(true);
      rcuResetTIM0();
#if !defined(GD32_TIM0_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM0_UP_NUMBER, GD32_GPT_TIM0_IRQ_PRIORITY, GD32_GPT_TIM0_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM0CLK)
      gptp->clock = GD32_TIM0CLK;
#else
      gptp->clock = GD32_TIMCLK2;
#endif
    }
#endif

#if GD32_GPT_USE_TIM1
    if (&GPTD2 == gptp) {
      rcuEnableTIM1(true);
      rcuResetTIM1();
#if !defined(GD32_TIM1_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM1_NUMBER, GD32_GPT_TIM1_IRQ_PRIORITY, GD32_GPT_TIM1_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM1CLK)
      gptp->clock = GD32_TIM1CLK;
#else
      gptp->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_GPT_USE_TIM2
    if (&GPTD3 == gptp) {
      rcuEnableTIM2(true);
      rcuResetTIM2();
#if !defined(GD32_TIM2_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM2_NUMBER, GD32_GPT_TIM2_IRQ_PRIORITY, GD32_GPT_TIM2_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM2CLK)
      gptp->clock = GD32_TIM2CLK;
#else
      gptp->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_GPT_USE_TIM3
    if (&GPTD4 == gptp) {
      rcuEnableTIM3(true);
      rcuResetTIM3();
#if !defined(GD32_TIM3_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM3_NUMBER, GD32_GPT_TIM3_IRQ_PRIORITY, GD32_GPT_TIM3_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM3CLK)
      gptp->clock = GD32_TIM3CLK;
#else
      gptp->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_GPT_USE_TIM4
    if (&GPTD5 == gptp) {
      rcuEnableTIM4(true);
      rcuResetTIM4();
#if !defined(GD32_TIM4_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM4_NUMBER, GD32_GPT_TIM4_IRQ_PRIORITY, GD32_GPT_TIM4_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM4CLK)
      gptp->clock = GD32_TIM4CLK;
#else
      gptp->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_GPT_USE_TIM5
    if (&GPTD6 == gptp) {
      rcuEnableTIM5(true);
      rcuResetTIM5();
#if !defined(GD32_TIM5_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM5_NUMBER, GD32_GPT_TIM5_IRQ_PRIORITY, GD32_GPT_TIM5_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM5CLK)
      gptp->clock = GD32_TIM5CLK;
#else
      gptp->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_GPT_USE_TIM6
    if (&GPTD7 == gptp) {
      rcuEnableTIM6(true);
      rcuResetTIM6();
#if !defined(GD32_TIM6_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM6_NUMBER, GD32_GPT_TIM6_IRQ_PRIORITY, GD32_GPT_TIM6_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM6CLK)
      gptp->clock = GD32_TIM6CLK;
#else
      gptp->clock = GD32_TIMCLK1;
#endif
    }
#endif

  }

  /* Prescaler value calculation.*/
  psc = (uint16_t)((gptp->clock / gptp->config->frequency) - 1);
  osalDbgAssert(((uint32_t)(psc + 1) * gptp->config->frequency) == gptp->clock,
                "invalid frequency");

  /* Timer configuration.*/
  gptp->tim->CTL0  = 0;                          /* Initially stopped.       */
  gptp->tim->CTL1  = gptp->config->ctl1;
  gptp->tim->PSC  = psc;                        /* Prescaler value.         */
  gptp->tim->INTF   = 0;                          /* Clear pending IRQs.      */
  gptp->tim->DMAINTEN = gptp->config->dmainten &        /* DMA-related DIER bits.   */
                    ~GD32_TIM_DIER_IRQ_MASK;
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {

  if (gptp->state == GPT_READY) {
    gptp->tim->CTL0  = 0;                        /* Timer disabled.          */
    gptp->tim->DMAINTEN = 0;                        /* All IRQs disabled.       */
    gptp->tim->INTF   = 0;                        /* Clear pending IRQs.      */

#if GD32_GPT_USE_TIM0
    if (&GPTD1 == gptp) {
#if !defined(GD32_TIM0_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM0_UP_NUMBER);
#endif
      rcuDisableTIM0();
    }
#endif

#if GD32_GPT_USE_TIM1
    if (&GPTD2 == gptp) {
#if !defined(GD32_TIM1_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM1_NUMBER);
#endif
      rcuDisableTIM1();
    }
#endif

#if GD32_GPT_USE_TIM2
    if (&GPTD3 == gptp) {
#if !defined(GD32_TIM2_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM2_NUMBER);
#endif
      rcuDisableTIM2();
    }
#endif

#if GD32_GPT_USE_TIM3
    if (&GPTD4 == gptp) {
#if !defined(GD32_TIM3_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM3_NUMBER);
#endif
      rcuDisableTIM3();
    }
#endif

#if GD32_GPT_USE_TIM4
    if (&GPTD5 == gptp) {
#if !defined(GD32_TIM4_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM4_NUMBER);
#endif
      rcuDisableTIM4();
    }
#endif

#if GD32_GPT_USE_TIM5
    if (&GPTD6 == gptp) {
#if !defined(GD32_TIM5_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM5_NUMBER);
#endif
      rcuDisableTIM5();
    }
#endif

#if GD32_GPT_USE_TIM6
    if (&GPTD7 == gptp) {
#if !defined(GD32_TIM6_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM6_NUMBER);
#endif
      rcuDisableTIM6();
    }
#endif
  }
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval) {

  gptp->tim->CAR = (uint32_t)(interval - 1U);   /* Time constant.           */
  gptp->tim->SWEVG = GD32_TIM_EGR_UG;            /* Update event.            */
  gptp->tim->CNT = 0;                           /* Reset counter.           */

  /* NOTE: After generating the UG event it takes several clock cycles before
     SR bit 0 goes to 1. This is why the clearing of CNT has been inserted
     before the clearing of SR, to give it some time.*/
  gptp->tim->INTF = 0;                            /* Clear pending IRQs.      */
  if (NULL != gptp->config->callback)
    gptp->tim->DMAINTEN |= GD32_TIM_DIER_UIE;      /* Update Event IRQ enabled.*/
  gptp->tim->CTL0 = GD32_TIM_CR1_ARPE | GD32_TIM_CR1_URS | GD32_TIM_CR1_CEN;
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {

  gptp->tim->CTL0 = 0;                           /* Initially stopped.       */
  gptp->tim->INTF = 0;                            /* Clear pending IRQs.      */

  /* All interrupts disabled.*/
  gptp->tim->DMAINTEN &= ~GD32_TIM_DIER_IRQ_MASK;
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {

  gptp->tim->CAR = (uint32_t)(interval - 1U);   /* Time constant.           */
  gptp->tim->SWEVG = GD32_TIM_EGR_UG;            /* Update event.            */
  gptp->tim->INTF  = 0;                           /* Clear pending IRQs.      */
  gptp->tim->CTL0 = GD32_TIM_CR1_OPM | GD32_TIM_CR1_URS | GD32_TIM_CR1_CEN;
  while (!(gptp->tim->INTF & GD32_TIM_SR_UIF))
    ;
  gptp->tim->INTF = 0;                            /* Clear pending IRQs.      */
}

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_serve_interrupt(GPTDriver *gptp) {
  uint32_t sr;

  sr  = gptp->tim->INTF;
  sr &= gptp->tim->DMAINTEN & GD32_TIM_DIER_IRQ_MASK;
  gptp->tim->INTF = ~sr;
  if ((sr & GD32_TIM_SR_UIF) != 0) {
    _gpt_isr_invoke_cb(gptp);
  }
}

#endif /* HAL_USE_GPT */

/** @} */
