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
 * @file    TIMv1/hal_gpt_lld.c
 * @brief   WB32 GPT subsystem low level driver source.
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
 * @note    The driver GPTD1 allocates the complex timer TIM1 when enabled.
 */
#if WB32_GPT_USE_TIM1 || defined(__DOXYGEN__)
GPTDriver GPTD1;
#endif

/**
 * @brief   GPTD2 driver identifier.
 * @note    The driver GPTD2 allocates the timer TIM2 when enabled.
 */
#if WB32_GPT_USE_TIM2 || defined(__DOXYGEN__)
GPTDriver GPTD2;
#endif

/**
 * @brief   GPTD3 driver identifier.
 * @note    The driver GPTD3 allocates the timer TIM3 when enabled.
 */
#if WB32_GPT_USE_TIM3 || defined(__DOXYGEN__)
GPTDriver GPTD3;
#endif

/**
 * @brief   GPTD4 driver identifier.
 * @note    The driver GPTD4 allocates the timer TIM4 when enabled.
 */
#if WB32_GPT_USE_TIM4 || defined(__DOXYGEN__)
GPTDriver GPTD4;
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

#if WB32_GPT_USE_TIM1 || defined(__DOXYGEN__)
#if !defined(WB32_TIM1_SUPPRESS_ISR)
#if !defined(WB32_TIM1_UP_IRQ_VECTOR)
#error "WB32_TIM1_UP_IRQ_VECTOR not defined"
#endif
/**
 * @brief   TIM1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_TIM1_UP_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(WB32_TIM1_SUPPRESS_ISR) */
#endif /* WB32_GPT_USE_TIM1 */

#if WB32_GPT_USE_TIM2 || defined(__DOXYGEN__)
#if !defined(WB32_TIM2_SUPPRESS_ISR)
#if !defined(WB32_TIM2_IRQ_VECTOR)
#error "WB32_TIM2_IRQ_VECTOR not defined"
#endif
/**
 * @brief   TIM2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_TIM2_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(WB32_TIM2_SUPPRESS_ISR) */
#endif /* WB32_GPT_USE_TIM2 */

#if WB32_GPT_USE_TIM3 || defined(__DOXYGEN__)
#if !defined(WB32_TIM3_SUPPRESS_ISR)
#if !defined(WB32_TIM3_IRQ_VECTOR)
#error "WB32_TIM3_IRQ_VECTOR not defined"
#endif
/**
 * @brief   TIM3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_TIM3_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(WB32_TIM3_SUPPRESS_ISR) */
#endif /* WB32_GPT_USE_TIM3 */

#if WB32_GPT_USE_TIM4 || defined(__DOXYGEN__)
#if !defined(WB32_TIM4_SUPPRESS_ISR)
#if !defined(WB32_TIM4_IRQ_VECTOR)
#error "WB32_TIM4_IRQ_VECTOR not defined"
#endif
/**
 * @brief   TIM4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_TIM4_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  gpt_lld_serve_interrupt(&GPTD4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(WB32_TIM4_SUPPRESS_ISR) */
#endif /* WB32_GPT_USE_TIM4 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {

#if WB32_GPT_USE_TIM1
  /* Driver initialization.*/
  GPTD1.tim = WB32_TIM1;
  gptObjectInit(&GPTD1);
#endif

#if WB32_GPT_USE_TIM2
  /* Driver initialization.*/
  GPTD2.tim = WB32_TIM2;
  gptObjectInit(&GPTD2);
#endif

#if WB32_GPT_USE_TIM3
  /* Driver initialization.*/
  GPTD3.tim = WB32_TIM3;
  gptObjectInit(&GPTD3);
#endif

#if WB32_GPT_USE_TIM4
  /* Driver initialization.*/
  GPTD4.tim = WB32_TIM4;
  gptObjectInit(&GPTD4);
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
#if WB32_GPT_USE_TIM1
    if (&GPTD1 == gptp) {
      rccEnableTIM1();
      rccResetTIM1();
#if !defined(WB32_TIM1_SUPPRESS_ISR)
      nvicEnableVector(WB32_TIM1_UP_NUMBER, WB32_GPT_TIM1_IRQ_PRIORITY);
#endif
#if defined(WB32_TIM1CLK)
      gptp->clock = WB32_TIM1CLK;
#else
      gptp->clock = WB32_TIMCLK1;
#endif
    }
#endif

#if WB32_GPT_USE_TIM2
    if (&GPTD2 == gptp) {
      rccEnableTIM2();
      rccResetTIM2();
#if !defined(WB32_TIM2_SUPPRESS_ISR)
      nvicEnableVector(WB32_TIM2_NUMBER, WB32_GPT_TIM2_IRQ_PRIORITY);
#endif
#if defined(WB32_TIM2CLK)
      gptp->clock = WB32_TIM2CLK;
#else
      gptp->clock = WB32_TIMCLK1;
#endif
    }
#endif

#if WB32_GPT_USE_TIM3
    if (&GPTD3 == gptp) {
      rccEnableTIM3();
      rccResetTIM3();
#if !defined(WB32_TIM3_SUPPRESS_ISR)
      nvicEnableVector(WB32_TIM3_NUMBER, WB32_GPT_TIM3_IRQ_PRIORITY);
#endif
#if defined(WB32_TIM3CLK)
      gptp->clock = WB32_TIM3CLK;
#else
      gptp->clock = WB32_TIMCLK1;
#endif
    }
#endif

#if WB32_GPT_USE_TIM4
    if (&GPTD4 == gptp) {
      rccEnableTIM4();
      rccResetTIM4();
#if !defined(WB32_TIM4_SUPPRESS_ISR)
      nvicEnableVector(WB32_TIM4_NUMBER, WB32_GPT_TIM4_IRQ_PRIORITY);
#endif
#if defined(WB32_TIM4CLK)
      gptp->clock = WB32_TIM4CLK;
#else
      gptp->clock = WB32_TIMCLK1;
#endif
    }
#endif
  }

  /* Prescaler value calculation.*/
  psc = (uint16_t)((gptp->clock / gptp->config->frequency) - 1);
  osalDbgAssert(((uint32_t)(psc + 1) * gptp->config->frequency) == gptp->clock,
                "invalid frequency");

  /* Timer configuration.*/
  /* Initially stopped.*/
  gptp->tim->CR1 = 0;
  gptp->tim->CR2 = gptp->config->cr2;
  /* Prescaler value.*/
  gptp->tim->PSC = psc;
  /* Clear pending IRQs.*/
  gptp->tim->SR = 0;
  /* DMA-related DIER bits.*/
  gptp->tim->DIER = gptp->config->dier & ~WB32_TIM_DIER_IRQ_MASK;
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
    /* Timer disabled.*/
    gptp->tim->CR1 = 0;
    /* All IRQs disabled.*/
    gptp->tim->DIER = 0;
    /* Clear pending IRQs.*/
    gptp->tim->SR = 0;

#if WB32_GPT_USE_TIM1
    if (&GPTD1 == gptp) {
#if !defined(WB32_TIM1_SUPPRESS_ISR)
      nvicDisableVector(WB32_TIM1_UP_NUMBER);
#endif
      rccDisableTIM1();
    }
#endif

#if WB32_GPT_USE_TIM2
    if (&GPTD2 == gptp) {
#if !defined(WB32_TIM2_SUPPRESS_ISR)
      nvicDisableVector(WB32_TIM2_NUMBER);
#endif
      rccDisableTIM2();
    }
#endif

#if WB32_GPT_USE_TIM3
    if (&GPTD3 == gptp) {
#if !defined(WB32_TIM3_SUPPRESS_ISR)
      nvicDisableVector(WB32_TIM3_NUMBER);
#endif
      rccDisableTIM3();
    }
#endif

#if WB32_GPT_USE_TIM4
    if (&GPTD4 == gptp) {
#if !defined(WB32_TIM4_SUPPRESS_ISR)
      nvicDisableVector(WB32_TIM4_NUMBER);
#endif
      rccDisableTIM4();
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

  /* Time constant.*/
  gptp->tim->ARR = (uint32_t)(interval - 1U);
  /* Update event.*/
  gptp->tim->EGR = WB32_TIM_EGR_UG;
  /* Reset counter.*/
  gptp->tim->CNT = 0;

  /* After generating the UG event it takes several clock cycles before SR bit
     0 goes to 1. This is why the clearing of CNT has been inserted before the
     clearing of SR, to give it some time.*/
  /* Clear pending IRQs.*/
  gptp->tim->SR = 0;
  if (NULL != gptp->config->callback) {
    /* Update Event IRQ enabled.*/
    gptp->tim->DIER |= WB32_TIM_DIER_UIE;
  }
  gptp->tim->CR1 = WB32_TIM_CR1_ARPE | WB32_TIM_CR1_URS | WB32_TIM_CR1_CEN;
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {

  /* Initially stopped.*/
  gptp->tim->CR1 = 0;
  /* Clear pending IRQs.*/
  gptp->tim->SR = 0;

  /* All interrupts disabled.*/
  gptp->tim->DIER &= ~WB32_TIM_DIER_IRQ_MASK;
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

  /* Time constant.*/
  gptp->tim->ARR = (uint32_t)(interval - 1U);
  /* Update event.*/
  gptp->tim->EGR = WB32_TIM_EGR_UG;
  /* Clear pending IRQs.*/
  gptp->tim->SR = 0;
  gptp->tim->CR1 = WB32_TIM_CR1_OPM | WB32_TIM_CR1_URS | WB32_TIM_CR1_CEN;
  while (!(gptp->tim->SR & WB32_TIM_SR_UIF))
    ;
  /* Clear pending IRQs.*/
  gptp->tim->SR = 0;
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

  sr = gptp->tim->SR;
  sr &= gptp->tim->DIER & WB32_TIM_DIER_IRQ_MASK;
  gptp->tim->SR = ~sr;
  if ((sr & WB32_TIM_SR_UIF) != 0) {
    _gpt_isr_invoke_cb(gptp);
  }
}

#endif /* HAL_USE_GPT */

/** @} */
