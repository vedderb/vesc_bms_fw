/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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
/*
   Concepts and parts of this file have been contributed by Fabio Utzig and
   Xo Wang.
*/
/*
   Rewritten by Emil Fresk (1/5 - 2014) for extended input capture
   functionality. And fix for spurious callbacks in the interrupt handler.
*/
/*
   Improved by Uladzimir Pylinsky aka barthess (1/3 - 2015) for support of
   32-bit timers and timers with single capture/compare channels.
*/

/*
 * Hardware Abstraction Layer for Extended Input Capture Unit
 */
#include "hal.h"

#if (HAL_USE_EICU == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
/**
 * @brief   Inverts the polarity for the given channel.
 *
 * @param[in] eicup     Pointer to the EICUDriver object.
 * @param[in] channel   The timer channel to invert.
 *
 * @notapi
 */
#define eicu_lld_invert_polarity(eicup, channel)                              \
  (eicup)->tim->CCER ^= ((uint16_t)(STM32_TIM_CCER_CC1P << ((channel) * 4)))

/**
 * @brief   Returns the compare value of the latest cycle.
 *
 * @param[in] chp       Pointer to channel structure that fired the interrupt.
 * @return              The number of ticks.
 *
 * @notapi
 */
#define eicu_lld_get_compare(chp)     (*((chp)->ccrp) + 1)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   EICUD1 driver identifier.
 * @note    The driver EICUD1 allocates the complex timer TIM1 when enabled.
 */
#if STM32_EICU_USE_TIM1 && !defined(__DOXYGEN__)
EICUDriver EICUD1;
#endif

/**
 * @brief   EICUD2 driver identifier.
 * @note    The driver EICUD2 allocates the timer TIM2 when enabled.
 */
#if STM32_EICU_USE_TIM2 && !defined(__DOXYGEN__)
EICUDriver EICUD2;
#endif

/**
 * @brief   EICUD3 driver identifier.
 * @note    The driver EICUD3 allocates the timer TIM3 when enabled.
 */
#if STM32_EICU_USE_TIM3 && !defined(__DOXYGEN__)
EICUDriver EICUD3;
#endif

/**
 * @brief   EICUD4 driver identifier.
 * @note    The driver EICUD4 allocates the timer TIM4 when enabled.
 */
#if STM32_EICU_USE_TIM4 && !defined(__DOXYGEN__)
EICUDriver EICUD4;
#endif

/**
 * @brief   EICUD5 driver identifier.
 * @note    The driver EICUD5 allocates the timer TIM5 when enabled.
 */
#if STM32_EICU_USE_TIM5 && !defined(__DOXYGEN__)
EICUDriver EICUD5;
#endif

/**
 * @brief   EICUD8 driver identifier.
 * @note    The driver EICUD8 allocates the timer TIM8 when enabled.
 */
#if STM32_EICU_USE_TIM8 && !defined(__DOXYGEN__)
EICUDriver EICUD8;
#endif

/**
 * @brief   EICUD9 driver identifier.
 * @note    The driver EICUD9 allocates the timer TIM9 when enabled.
 */
#if STM32_EICU_USE_TIM9 && !defined(__DOXYGEN__)
EICUDriver EICUD9;
#endif

/**
 * @brief   EICUD12 driver identifier.
 * @note    The driver EICUD12 allocates the timer TIM12 when enabled.
 */
#if STM32_EICU_USE_TIM12 && !defined(__DOXYGEN__)
EICUDriver EICUD12;
#endif

/**
 * @brief   EICUD10 driver identifier.
 * @note    The driver EICUD10 allocates the timer TIM10 when enabled.
 */
#if STM32_EICU_USE_TIM10 && !defined(__DOXYGEN__)
EICUDriver EICUD10;
#endif

/**
 * @brief   EICUD11 driver identifier.
 * @note    The driver EICUD11 allocates the timer TIM11 when enabled.
 */
#if STM32_EICU_USE_TIM11 && !defined(__DOXYGEN__)
EICUDriver EICUD11;
#endif

/**
 * @brief   EICUD13 driver identifier.
 * @note    The driver EICUD13 allocates the timer TIM13 when enabled.
 */
#if STM32_EICU_USE_TIM13 && !defined(__DOXYGEN__)
EICUDriver EICUD13;
#endif

/**
 * @brief   EICUD14 driver identifier.
 * @note    The driver EICUD14 allocates the timer TIM14 when enabled.
 */
#if STM32_EICU_USE_TIM14 && !defined(__DOXYGEN__)
EICUDriver EICUD14;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
/**
 * @brief     Returns both pulse width and period.
 * @details   The time is defined as number of ticks.
 *
 * @param[in] eicup     Pointer to the EICUDriver object.
 * @param[in] channel   The timer channel that fired the interrupt.
 * @param[in] compare   Content of the CCR register.
 * @return              The number of ticks.
 *
 * @notapi
 */
static eicuresult_t get_time_both(const EICUDriver *eicup,
                                  eicuchannel_t channel,
                                  eicucnt_t compare) {

  const EICUChannel *chp = &eicup->channel[channel];
  eicuresult_t ret;

  /* Note! there is no overflow check because it handles under the hood of
     unsigned subtraction math.*/

  /* 16-bit timer */
  if (EICU_WIDTH_16 == eicup->width) {
    uint16_t cmp = compare;
    uint16_t la = chp->last_active;
    uint16_t li = chp->last_idle;
    uint16_t w = li - la;
    uint16_t p = cmp - la;
    ret.width  = w;
    ret.period = p;
  }
  /* 32-bit timer */
  else if (EICU_WIDTH_32 == eicup->width) {
    ret.width  = chp->last_idle - chp->last_active;
    ret.period = compare - chp->last_active;
  }
  /* error trap */
  else {
    osalSysHalt("Unhandled width value");
  }

  return ret;
}

/**
 * @brief     Returns pulse width.
 * @details   The time is defined as number of ticks.
 *
 * @param[in] eicup     Pointer to the EICUDriver object.
 * @param[in] channel   The timer channel that fired the interrupt.
 * @param[in] compare   Content of the CCR register.
 * @return              The number of ticks.
 *
 * @notapi
 */
static eicucnt_t get_time_width(const EICUDriver *eicup,
                                eicuchannel_t channel,
                                eicucnt_t compare) {

  const EICUChannel *chp = &eicup->channel[channel];

  /* Note! there is no overflow check because it handles under the hood of
     unsigned subtraction math.*/

  /* 16-bit timer */
  if (EICU_WIDTH_16 == eicup->width) {
    uint16_t cmp = compare;
    uint16_t la  = chp->last_active;
    uint16_t ret = cmp - la;
    return ret;
  }
  /* 32-bit timer */
  else if (EICU_WIDTH_32 == eicup->width) {
    return compare - chp->last_active;
  }
  /* error trap */
  else {
    osalSysHalt("Unhandled width value");
    return 0;
  }
}

/**
 * @brief     Returns pulse period.
 * @details   The time is defined as number of ticks.
 *
 * @param[in] eicup     Pointer to the EICUDriver object.
 * @param[in] channel   The timer channel that fired the interrupt.
 * @param[in] compare   Content of the CCR register.
 * @return              The number of ticks.
 *
 * @notapi
 */
static eicucnt_t get_time_period(const EICUDriver *eicup,
                                 eicuchannel_t channel,
                                 eicucnt_t compare) {

  const EICUChannel *chp = &eicup->channel[channel];

  /* Note! there is no overflow check because it handles under the hood of
     unsigned subtraction math.*/

  /* 16-bit timer */
  if (EICU_WIDTH_16 == eicup->width) {
    uint16_t cmp = compare;
    uint16_t li  = chp->last_idle;
    uint16_t ret = cmp - li;
    return ret;
  }
  /* 32-bit timer */
  else if (EICU_WIDTH_32 == eicup->width) {
    return compare - chp->last_idle;
  }
  /* error trap */
  else {
    osalSysHalt("Unhandled width value");
    return 0;
  }
}

/**
 * @brief   EICU width or (width + period) event.
 * @note    Needs special care since it needs to invert the
 *          correct polarity bit to detect pulses.
 * @note    Assumes that the polarity is not changed by some
 *          external user. It must only be changed using the HAL.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 * @param[in] channel   The timer channel that fired the interrupt.
 *
 * @notapi
 */
static void isr_invoke_pulse_cb(EICUDriver *eicup, eicuchannel_t channel) {
  EICUChannel *chp = &eicup->channel[channel];
  eicucnt_t compare = eicu_lld_get_compare(chp);

  if (EICU_CH_ACTIVE == chp->state) {
    chp->state = EICU_CH_IDLE;
    eicu_lld_invert_polarity(eicup, channel);
    if (EICU_INPUT_PULSE == chp->config->mode) {
      uint32_t width = get_time_width(eicup, channel, compare);
      chp->config->capture_cb(eicup, channel, width, 0);
    }
    chp->last_idle = compare;
  }
  else {
    chp->state = EICU_CH_ACTIVE;
    eicu_lld_invert_polarity(eicup, channel);
    if (EICU_INPUT_BOTH == chp->config->mode) {
      eicuresult_t both = get_time_both(eicup, channel, compare);
      chp->config->capture_cb(eicup, channel, both.width, both.period);
    }
    chp->last_active = compare;
  }
}

/**
 * @brief   EICU Edge detect event.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 * @param[in] channel   The timer channel that fired the interrupt.
 *
 * @notapi
 */
static void isr_invoke_edge_cb(EICUDriver *eicup, eicuchannel_t channel) {
  EICUChannel *chp = &eicup->channel[channel];
  eicucnt_t compare = eicu_lld_get_compare(chp);
  uint32_t period = get_time_period(eicup, channel, compare);

  chp->config->capture_cb(eicup, channel, 0, period);
  chp->last_idle = compare;
}

/**
 * @brief   Common EICU detect call.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 * @param[in] channel   The timer channel that fired the interrupt.
 *
 * @notapi
 */
static void eicu_isr_invoke_cb(EICUDriver *eicup, eicuchannel_t channel) {

  if (EICU_INPUT_EDGE == eicup->channel[channel].config->mode)
    isr_invoke_edge_cb(eicup, channel);
  else /* EICU_INPUT_PULSE || EICU_INPUT_BOTH */
    isr_invoke_pulse_cb(eicup, channel);
}

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 */
static void eicu_lld_serve_interrupt(EICUDriver *eicup) {
  uint16_t sr;
  sr = eicup->tim->SR;

  /* Pick out the interrupts we are interested in by using
     the interrupt enable bits as mask */
  sr &= (eicup->tim->DIER & STM32_TIM_DIER_IRQ_MASK);

  /* Clear interrupts */
  eicup->tim->SR = ~sr;

  if ((sr & STM32_TIM_SR_CC1IF) != 0)
    eicu_isr_invoke_cb(eicup, EICU_CHANNEL_1);
  if ((sr & STM32_TIM_SR_CC2IF) != 0)
    eicu_isr_invoke_cb(eicup, EICU_CHANNEL_2);
  if ((sr & STM32_TIM_SR_CC3IF) != 0)
    eicu_isr_invoke_cb(eicup, EICU_CHANNEL_3);
  if ((sr & STM32_TIM_SR_CC4IF) != 0)
    eicu_isr_invoke_cb(eicup, EICU_CHANNEL_4);
}

/**
 * @brief   Starts every channel.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 */
static void start_channels(EICUDriver *eicup) {

  /* Set each input channel that is used as: a normal input capture channel,
     link the corresponding CCR register and set polarity. */

  /* Input capture channel 1 */
  if (eicup->config->iccfgp[0] != NULL) {
    /* Normal capture input input */
    eicup->tim->CCMR1 |= STM32_TIM_CCMR1_CC1S(1);

    /* Link CCR register */
    eicup->channel[0].ccrp = &eicup->tim->CCR[0];

    /* Set input polarity */
    if (eicup->config->iccfgp[0]->alvl == EICU_INPUT_ACTIVE_HIGH)
      eicup->tim->CCER |= STM32_TIM_CCER_CC1E;
    else
      eicup->tim->CCER |= STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC1P;
  }

  /* Input capture channel 2 */
  if (eicup->config->iccfgp[1] != NULL) {
    /* Normal capture input input */
    eicup->tim->CCMR1 |= STM32_TIM_CCMR1_CC2S(1);

    /* Link CCR register */
    eicup->channel[1].ccrp = &eicup->tim->CCR[1];

    /* Set input polarity */
    if (eicup->config->iccfgp[1]->alvl == EICU_INPUT_ACTIVE_HIGH)
      eicup->tim->CCER |= STM32_TIM_CCER_CC2E;
    else
      eicup->tim->CCER |= STM32_TIM_CCER_CC2E | STM32_TIM_CCER_CC2P;
  }

  /* Input capture channel 3 (not for TIM 9 and 12) */
  if (eicup->config->iccfgp[2] != NULL) {
    /* Normal capture input input */
    eicup->tim->CCMR2 |= STM32_TIM_CCMR2_CC3S(1);

    /* Link CCR register */
    eicup->channel[2].ccrp = &eicup->tim->CCR[2];

    /* Set input polarity */
    if (eicup->config->iccfgp[2]->alvl == EICU_INPUT_ACTIVE_HIGH)
      eicup->tim->CCER |= STM32_TIM_CCER_CC3E;
    else
      eicup->tim->CCER |= STM32_TIM_CCER_CC3E | STM32_TIM_CCER_CC3P;
  }

  /* Input capture channel 4 (not for TIM 9 and 12) */
  if (eicup->config->iccfgp[3] != NULL) {
    /* Normal capture input input */
    eicup->tim->CCMR2 |= STM32_TIM_CCMR2_CC4S(1);

    /* Link CCR register */
    eicup->channel[3].ccrp = &eicup->tim->CCR[3];

    /* Set input polarity */
    if (eicup->config->iccfgp[3]->alvl == EICU_INPUT_ACTIVE_HIGH)
      eicup->tim->CCER |= STM32_TIM_CCER_CC4E;
    else
      eicup->tim->CCER |= STM32_TIM_CCER_CC4E | STM32_TIM_CCER_CC4P;
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_EICU_USE_TIM1
#if !defined(STM32_TIM1_UP_HANDLER)
#error "STM32_TIM1_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM1 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM1_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD1);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(STM32_TIM1_CC_HANDLER)
#error "STM32_TIM1_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM1 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM1_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM1 */

#if STM32_EICU_USE_TIM2

#if !defined(STM32_TIM2_HANDLER)
#error "STM32_TIM2_HANDLER not defined"
#endif
/**
 * @brief   TIM2 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM2 */

#if STM32_EICU_USE_TIM3
#if !defined(STM32_TIM3_HANDLER)
#error "STM32_TIM3_HANDLER not defined"
#endif
/**
 * @brief   TIM3 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM3 */

#if STM32_EICU_USE_TIM4
#if !defined(STM32_TIM4_HANDLER)
#error "STM32_TIM4_HANDLER not defined"
#endif
/**
 * @brief   TIM4 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM4 */

#if STM32_EICU_USE_TIM5
#if !defined(STM32_TIM5_HANDLER)
#error "STM32_TIM5_HANDLER not defined"
#endif
/**
 * @brief   TIM5 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM5 */

#if STM32_EICU_USE_TIM8
#if !defined(STM32_TIM8_UP_HANDLER)
#error "STM32_TIM8_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM8 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD8);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(STM32_TIM8_CC_HANDLER)
#error "STM32_TIM8_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM8 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD8);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM8 */

#if STM32_EICU_USE_TIM9
#if !defined(STM32_TIM9_HANDLER)
#error "STM32_TIM9_HANDLER not defined"
#endif
/**
 * @brief   TIM9 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM9_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD9);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM9 */

#if STM32_EICU_USE_TIM12
#if !defined(STM32_TIM12_HANDLER)
#error "STM32_TIM12_HANDLER not defined"
#endif
/**
 * @brief   TIM12 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM12_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD12);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM12 */

#if STM32_EICU_USE_TIM10
#if !defined(STM32_TIM10_HANDLER)
#error "STM32_TIM10_HANDLER not defined"
#endif
/**
 * @brief   TIM10 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM10_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD10);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM10 */

#if STM32_EICU_USE_TIM11
#if !defined(STM32_TIM11_HANDLER)
#error "STM32_TIM11_HANDLER not defined"
#endif
/**
 * @brief   TIM11 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM11_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD11);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM11 */

#if STM32_EICU_USE_TIM13
#if !defined(STM32_TIM13_HANDLER)
#error "STM32_TIM13_HANDLER not defined"
#endif
/**
 * @brief   TIM13 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM13_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD13);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM13 */

#if STM32_EICU_USE_TIM14
#if !defined(STM32_TIM14_HANDLER)
#error "STM32_TIM14_HANDLER not defined"
#endif
/**
 * @brief   TIM14 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM14_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD14);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM14 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level EICU driver initialization.
 *
 * @notapi
 */
void eicu_lld_init(void) {
#if STM32_EICU_USE_TIM1
  /* Driver initialization.*/
  eicuObjectInit(&EICUD1);
  EICUD1.tim = STM32_TIM1;
#endif

#if STM32_EICU_USE_TIM2
  /* Driver initialization.*/
  eicuObjectInit(&EICUD2);
  EICUD2.tim = STM32_TIM2;
#endif

#if STM32_EICU_USE_TIM3
  /* Driver initialization.*/
  eicuObjectInit(&EICUD3);
  EICUD3.tim = STM32_TIM3;
#endif

#if STM32_EICU_USE_TIM4
  /* Driver initialization.*/
  eicuObjectInit(&EICUD4);
  EICUD4.tim = STM32_TIM4;
#endif

#if STM32_EICU_USE_TIM5
  /* Driver initialization.*/
  eicuObjectInit(&EICUD5);
  EICUD5.tim = STM32_TIM5;
#endif

#if STM32_EICU_USE_TIM8
  /* Driver initialization.*/
  eicuObjectInit(&EICUD8);
  EICUD8.tim = STM32_TIM8;
#endif

#if STM32_EICU_USE_TIM9
  /* Driver initialization.*/
  eicuObjectInit(&EICUD9);
  EICUD9.tim = STM32_TIM9;
#endif

#if STM32_EICU_USE_TIM12
  /* Driver initialization.*/
  eicuObjectInit(&EICUD12);
  EICUD12.tim = STM32_TIM12;
#endif

#if STM32_EICU_USE_TIM10
  /* Driver initialization.*/
  eicuObjectInit(&EICUD10);
  EICUD10.tim = STM32_TIM10;
#endif

#if STM32_EICU_USE_TIM11
  /* Driver initialization.*/
  eicuObjectInit(&EICUD11);
  EICUD11.tim = STM32_TIM11;
#endif

#if STM32_EICU_USE_TIM13
  /* Driver initialization.*/
  eicuObjectInit(&EICUD13);
  EICUD13.tim = STM32_TIM13;
#endif

#if STM32_EICU_USE_TIM14
  /* Driver initialization.*/
  eicuObjectInit(&EICUD14);
  EICUD14.tim = STM32_TIM14;
#endif
}

/**
 * @brief   Configures and activates the EICU peripheral.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @notapi
 */
void eicu_lld_start(EICUDriver *eicup) {
  uint32_t psc;
  size_t ch;

  osalDbgAssert((eicup->config->iccfgp[0] != NULL) ||
                (eicup->config->iccfgp[1] != NULL) ||
                (eicup->config->iccfgp[2] != NULL) ||
                (eicup->config->iccfgp[3] != NULL),
                 "invalid input configuration");

  if (eicup->state == EICU_STOP) {
    /* Clock activation and timer reset.*/
#if STM32_EICU_USE_TIM1
    if (&EICUD1 == eicup) {
      rccEnableTIM1(FALSE);
      rccResetTIM1();
      nvicEnableVector(STM32_TIM1_UP_NUMBER, STM32_EICU_TIM1_IRQ_PRIORITY);
      nvicEnableVector(STM32_TIM1_CC_NUMBER, STM32_EICU_TIM1_IRQ_PRIORITY);
      eicup->channels = 4;
#if defined(STM32_TIM1CLK)
      eicup->clock = STM32_TIM1CLK;
#else
      eicup->clock = STM32_TIMCLK2;
#endif
    }
#endif
#if STM32_EICU_USE_TIM2
    if (&EICUD2 == eicup) {
      rccEnableTIM2(FALSE);
      rccResetTIM2();
      nvicEnableVector(STM32_TIM2_NUMBER, STM32_EICU_TIM2_IRQ_PRIORITY);
      eicup->channels = 4;
      eicup->clock = STM32_TIMCLK1;
    }
#endif
#if STM32_EICU_USE_TIM3
    if (&EICUD3 == eicup) {
      rccEnableTIM3(FALSE);
      rccResetTIM3();
      nvicEnableVector(STM32_TIM3_NUMBER, STM32_EICU_TIM3_IRQ_PRIORITY);
      eicup->channels = 4;
      eicup->clock = STM32_TIMCLK1;
    }
#endif
#if STM32_EICU_USE_TIM4
    if (&EICUD4 == eicup) {
      rccEnableTIM4(FALSE);
      rccResetTIM4();
      nvicEnableVector(STM32_TIM4_NUMBER, STM32_EICU_TIM4_IRQ_PRIORITY);
      eicup->channels = 4;
      eicup->clock = STM32_TIMCLK1;
    }
#endif
#if STM32_EICU_USE_TIM5
    if (&EICUD5 == eicup) {
      rccEnableTIM5(FALSE);
      rccResetTIM5();
      nvicEnableVector(STM32_TIM5_NUMBER, STM32_EICU_TIM5_IRQ_PRIORITY);
      eicup->channels = 4;
      eicup->clock = STM32_TIMCLK1;
    }
#endif
#if STM32_EICU_USE_TIM8
    if (&EICUD8 == eicup) {
      rccEnableTIM8(FALSE);
      rccResetTIM8();
      nvicEnableVector(STM32_TIM8_UP_NUMBER, STM32_EICU_TIM8_IRQ_PRIORITY);
      nvicEnableVector(STM32_TIM8_CC_NUMBER, STM32_EICU_TIM8_IRQ_PRIORITY);
      eicup->channels = 4;
#if defined(STM32_TIM8CLK)
      eicup->clock = STM32_TIM8CLK;
#else
      eicup->clock = STM32_TIMCLK2;
#endif
    }
#endif
#if STM32_EICU_USE_TIM9
    if (&EICUD9 == eicup) {
      rccEnableTIM9(FALSE);
      rccResetTIM9();
      nvicEnableVector(STM32_TIM9_NUMBER, STM32_EICU_TIM9_IRQ_PRIORITY);
      eicup->channels = 2;
      eicup->clock = STM32_TIMCLK2;
    }
#endif
#if STM32_EICU_USE_TIM12
    if (&EICUD12 == eicup) {
      rccEnableTIM12(FALSE);
      rccResetTIM12();
      nvicEnableVector(STM32_TIM12_NUMBER, STM32_EICU_TIM12_IRQ_PRIORITY);
      eicup->channels = 2;
      eicup->clock = STM32_TIMCLK1;
    }
#endif
#if STM32_EICU_USE_TIM10
    if (&EICUD10 == eicup) {
      rccEnableTIM10(FALSE);
      rccResetTIM10();
      nvicEnableVector(STM32_TIM10_NUMBER, STM32_EICU_TIM10_IRQ_PRIORITY);
      eicup->channels = 1;
      eicup->clock = STM32_TIMCLK2;
    }
#endif
#if STM32_EICU_USE_TIM11
    if (&EICUD11 == eicup) {
      rccEnableTIM11(FALSE);
      rccResetTIM11();
      nvicEnableVector(STM32_TIM11_NUMBER, STM32_EICU_TIM11_IRQ_PRIORITY);
      eicup->channels = 1;
      eicup->clock = STM32_TIMCLK2;
    }
#endif
#if STM32_EICU_USE_TIM13
    if (&EICUD13 == eicup) {
      rccEnableTIM13(FALSE);
      rccResetTIM13();
      nvicEnableVector(STM32_TIM13_NUMBER, STM32_EICU_TIM13_IRQ_PRIORITY);
      eicup->channels = 1;
      eicup->clock = STM32_TIMCLK1;
    }
#endif
#if STM32_EICU_USE_TIM14
    if (&EICUD14 == eicup) {
      rccEnableTIM14(FALSE);
      rccResetTIM14();
      nvicEnableVector(STM32_TIM14_NUMBER, STM32_EICU_TIM14_IRQ_PRIORITY);
      eicup->channels = 1;
      eicup->clock = STM32_TIMCLK1;
    }
#endif
  }
  else {
    /* Driver re-configuration scenario, it must be stopped first.*/
    eicup->tim->CR1    = 0;                   /* Timer disabled.              */
    eicup->tim->DIER   = eicup->config->dier &/* DMA-related DIER settings.   */
                        ~STM32_TIM_DIER_IRQ_MASK;
    eicup->tim->SR     = 0;                   /* Clear eventual pending IRQs. */
    eicup->tim->CCR[0] = 0;                   /* Comparator 1 disabled.       */
    eicup->tim->CCR[1] = 0;                   /* Comparator 2 disabled.       */
    eicup->tim->CNT    = 0;                   /* Counter reset to zero.       */
  }

  /* Timer configuration.*/
  psc = (eicup->clock / eicup->config->frequency) - 1;
  chDbgAssert((psc <= 0xFFFF) &&
             ((psc + 1) * eicup->config->frequency) == eicup->clock,
               "invalid frequency");
  eicup->tim->PSC   = (uint16_t)psc;
  eicup->tim->ARR   = (eicucnt_t)-1;

  /* Detect width.*/
  if (0xFFFFFFFF == eicup->tim->ARR)
    eicup->width = EICU_WIDTH_32;
  else if (0xFFFF == eicup->tim->ARR)
    eicup->width = EICU_WIDTH_16;
  else
    osalSysHalt("Unsupported width");

  /* Reset registers */
  eicup->tim->SMCR  = 0;
  eicup->tim->CCMR1 = 0;
  if (eicup->channels > 2)
    eicup->tim->CCMR2 = 0;

  /* clean channel structures and set pointers to channel configs */
  for (ch=0; ch<EICU_CHANNEL_ENUM_END; ch++) {
    eicup->channel[ch].last_active = 0;
    eicup->channel[ch].last_idle = 0;
    eicup->channel[ch].config = eicup->config->iccfgp[ch];
    eicup->channel[ch].state = EICU_CH_IDLE;
  }

  /* TIM9 and TIM12 have only 2 channels.*/
  if (eicup->channels == 2) {
    osalDbgCheck((eicup->config->iccfgp[2] == NULL) &&
                 (eicup->config->iccfgp[3] == NULL));
  }

  /* TIM10, TIM11, TIM13 and TIM14 have only 1 channel.*/
  if (eicup->channels == 1) {
    osalDbgCheck((eicup->config->iccfgp[1] == NULL) &&
                 (eicup->config->iccfgp[2] == NULL) &&
                 (eicup->config->iccfgp[3] == NULL));
  }

  start_channels(eicup);
}

/**
 * @brief   Deactivates the EICU peripheral.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @notapi
 */
void eicu_lld_stop(EICUDriver *eicup) {

  if (eicup->state == EICU_READY) {

    /* Clock deactivation.*/
    eicup->tim->CR1  = 0;                     /* Timer disabled.              */
    eicup->tim->DIER = 0;                     /* All IRQs disabled.           */
    eicup->tim->SR   = 0;                     /* Clear eventual pending IRQs. */

#if STM32_EICU_USE_TIM1
    if (&EICUD1 == eicup) {
      nvicDisableVector(STM32_TIM1_UP_NUMBER);
      nvicDisableVector(STM32_TIM1_CC_NUMBER);
      rccDisableTIM1();
    }
#endif
#if STM32_EICU_USE_TIM2
    if (&EICUD2 == eicup) {
      nvicDisableVector(STM32_TIM2_NUMBER);
      rccDisableTIM2();
    }
#endif
#if STM32_EICU_USE_TIM3
    if (&EICUD3 == eicup) {
      nvicDisableVector(STM32_TIM3_NUMBER);
      rccDisableTIM3();
    }
#endif
#if STM32_EICU_USE_TIM4
    if (&EICUD4 == eicup) {
      nvicDisableVector(STM32_TIM4_NUMBER);
      rccDisableTIM4();
    }
#endif
#if STM32_EICU_USE_TIM5
    if (&EICUD5 == eicup) {
      nvicDisableVector(STM32_TIM5_NUMBER);
      rccDisableTIM5();
    }
#endif
#if STM32_EICU_USE_TIM8
    if (&EICUD8 == eicup) {
      nvicDisableVector(STM32_TIM8_UP_NUMBER);
      nvicDisableVector(STM32_TIM8_CC_NUMBER);
      rccDisableTIM8();
    }
#endif
#if STM32_EICU_USE_TIM9
    if (&EICUD9 == eicup) {
      nvicDisableVector(STM32_TIM9_NUMBER);
      rccDisableTIM9();
    }
#endif
#if STM32_EICU_USE_TIM12
    if (&EICUD12 == eicup) {
      nvicDisableVector(STM32_TIM12_NUMBER);
      rccDisableTIM12();
    }
#endif
  }
#if STM32_EICU_USE_TIM10
    if (&EICUD10 == eicup) {
      nvicDisableVector(STM32_TIM10_NUMBER);
      rccDisableTIM10();
    }
#endif
#if STM32_EICU_USE_TIM11
    if (&EICUD11 == eicup) {
      nvicDisableVector(STM32_TIM11_NUMBER);
      rccDisableTIM11();
    }
#endif
#if STM32_EICU_USE_TIM13
    if (&EICUD13 == eicup) {
      nvicDisableVector(STM32_TIM13_NUMBER);
      rccDisableTIM13();
    }
#endif
#if STM32_EICU_USE_TIM14
    if (&EICUD14 == eicup) {
      nvicDisableVector(STM32_TIM14_NUMBER);
      rccDisableTIM14();
    }
#endif
}

/**
 * @brief   Enables the EICU.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @notapi
 */
void eicu_lld_enable(EICUDriver *eicup) {

  eicup->tim->EGR = STM32_TIM_EGR_UG;
  eicup->tim->SR = 0;                         /* Clear pending IRQs (if any). */

  if ((eicup->config->iccfgp[EICU_CHANNEL_1] != NULL) &&
      (eicup->config->iccfgp[EICU_CHANNEL_1]->capture_cb != NULL))
    eicup->tim->DIER |= STM32_TIM_DIER_CC1IE;
  if ((eicup->config->iccfgp[EICU_CHANNEL_2] != NULL) &&
      (eicup->config->iccfgp[EICU_CHANNEL_2]->capture_cb != NULL))
    eicup->tim->DIER |= STM32_TIM_DIER_CC2IE;
  if ((eicup->config->iccfgp[EICU_CHANNEL_3] != NULL) &&
      (eicup->config->iccfgp[EICU_CHANNEL_3]->capture_cb != NULL))
    eicup->tim->DIER |= STM32_TIM_DIER_CC3IE;
  if ((eicup->config->iccfgp[EICU_CHANNEL_4] != NULL) &&
      (eicup->config->iccfgp[EICU_CHANNEL_4]->capture_cb != NULL))
    eicup->tim->DIER |= STM32_TIM_DIER_CC4IE;

  eicup->tim->CR1 = STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
}

/**
 * @brief   Disables the EICU.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @notapi
 */
void eicu_lld_disable(EICUDriver *eicup) {
  eicup->tim->CR1   = 0;                      /* Initially stopped.           */
  eicup->tim->SR    = 0;                      /* Clear pending IRQs (if any). */

  /* All interrupts disabled.*/
  eicup->tim->DIER &= ~STM32_TIM_DIER_IRQ_MASK;
}

#endif /* HAL_USE_EICU */
