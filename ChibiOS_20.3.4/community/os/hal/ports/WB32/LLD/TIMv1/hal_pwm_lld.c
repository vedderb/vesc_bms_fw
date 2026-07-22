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
 * @file    TIMv1/hal_pwm_lld.c
 * @brief   WB32 PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#include "hal.h"

#if HAL_USE_PWM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PWMD1 driver identifier.
 * @note    The driver PWMD1 allocates the complex timer TIM1 when enabled.
 */
#if WB32_PWM_USE_TIM1 || defined(__DOXYGEN__)
PWMDriver PWMD1;
#endif

/**
 * @brief   PWMD2 driver identifier.
 * @note    The driver PWMD2 allocates the timer TIM2 when enabled.
 */
#if WB32_PWM_USE_TIM2 || defined(__DOXYGEN__)
PWMDriver PWMD2;
#endif

/**
 * @brief   PWMD3 driver identifier.
 * @note    The driver PWMD3 allocates the timer TIM3 when enabled.
 */
#if WB32_PWM_USE_TIM3 || defined(__DOXYGEN__)
PWMDriver PWMD3;
#endif

/**
 * @brief   PWMD4 driver identifier.
 * @note    The driver PWMD4 allocates the timer TIM4 when enabled.
 */
#if WB32_PWM_USE_TIM4 || defined(__DOXYGEN__)
PWMDriver PWMD4;
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

#if WB32_PWM_USE_TIM1 || defined(__DOXYGEN__)
#if !defined(WB32_TIM1_SUPPRESS_ISR)
#if !defined(WB32_TIM1_UP_IRQ_VECTOR)
#error "WB32_TIM1_UP_IRQ_VECTOR not defined"
#endif
/**
 * @brief   TIM1 update interrupt handler.
 * @note    It is assumed that this interrupt is only activated if the callback
 *          pointer is not equal to @p NULL in order to not perform an extra
 *          check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_TIM1_UP_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  pwm_lld_serve_interrupt(&PWMD1);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(WB32_TIM1_CC_IRQ_VECTOR)
#error "WB32_TIM1_CC_IRQ_VECTOR not defined"
#endif
/**
 * @brief   TIM1 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_TIM1_CC_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  pwm_lld_serve_interrupt(&PWMD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(WB32_TIM1_SUPPRESS_ISR) */
#endif /* WB32_PWM_USE_TIM1 */

#if WB32_PWM_USE_TIM2 || defined(__DOXYGEN__)
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

  pwm_lld_serve_interrupt(&PWMD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(WB32_TIM2_SUPPRESS_ISR) */
#endif /* WB32_PWM_USE_TIM2 */

#if WB32_PWM_USE_TIM3 || defined(__DOXYGEN__)
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

  pwm_lld_serve_interrupt(&PWMD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(WB32_TIM3_SUPPRESS_ISR) */
#endif /* WB32_PWM_USE_TIM3 */

#if WB32_PWM_USE_TIM4 || defined(__DOXYGEN__)
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

  pwm_lld_serve_interrupt(&PWMD4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(WB32_TIM4_SUPPRESS_ISR) */
#endif /* WB32_PWM_USE_TIM4 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {

#if WB32_PWM_USE_TIM1
  /* Driver initialization.*/
  pwmObjectInit(&PWMD1);
  PWMD1.channels = WB32_TIM1_CHANNELS;
  PWMD1.tim = WB32_TIM1;
#endif

#if WB32_PWM_USE_TIM2
  /* Driver initialization.*/
  pwmObjectInit(&PWMD2);
  PWMD2.channels = WB32_TIM2_CHANNELS;
  PWMD2.tim = WB32_TIM2;
#endif

#if WB32_PWM_USE_TIM3
  /* Driver initialization.*/
  pwmObjectInit(&PWMD3);
  PWMD3.channels = WB32_TIM3_CHANNELS;
  PWMD3.tim = WB32_TIM3;
#endif

#if WB32_PWM_USE_TIM4
  /* Driver initialization.*/
  pwmObjectInit(&PWMD4);
  PWMD4.channels = WB32_TIM4_CHANNELS;
  PWMD4.tim = WB32_TIM4;
#endif
}

/**
 * @brief   Configures and activates the PWM peripheral.
 * @note    Starting a driver that is already in the @p PWM_READY state
 *          disables all the active channels.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_start(PWMDriver *pwmp) {
  uint32_t psc;
  uint32_t ccer;

  if (pwmp->state == PWM_STOP) {
    /* Clock activation and timer reset.*/
#if WB32_PWM_USE_TIM1
    if (&PWMD1 == pwmp) {
      rccEnableTIM1();
      rccResetTIM1();
#if !defined(WB32_TIM1_SUPPRESS_ISR)
      nvicEnableVector(WB32_TIM1_UP_NUMBER, WB32_PWM_TIM1_IRQ_PRIORITY);
      nvicEnableVector(WB32_TIM1_CC_NUMBER, WB32_PWM_TIM1_IRQ_PRIORITY);
#endif
#if defined(WB32_TIM1CLK)
      pwmp->clock = WB32_TIM1CLK;
#else
      pwmp->clock = WB32_TIMCLK1;
#endif
    }
#endif

#if WB32_PWM_USE_TIM2
    if (&PWMD2 == pwmp) {
      rccEnableTIM2();
      rccResetTIM2();
#if !defined(WB32_TIM2_SUPPRESS_ISR)
      nvicEnableVector(WB32_TIM2_NUMBER, WB32_PWM_TIM2_IRQ_PRIORITY);
#endif
#if defined(WB32_TIM2CLK)
      pwmp->clock = WB32_TIM2CLK;
#else
      pwmp->clock = WB32_TIMCLK1;
#endif
    }
#endif

#if WB32_PWM_USE_TIM3
    if (&PWMD3 == pwmp) {
      rccEnableTIM3();
      rccResetTIM3();
#if !defined(WB32_TIM3_SUPPRESS_ISR)
      nvicEnableVector(WB32_TIM3_NUMBER, WB32_PWM_TIM3_IRQ_PRIORITY);
#endif
#if defined(WB32_TIM3CLK)
      pwmp->clock = WB32_TIM3CLK;
#else
      pwmp->clock = WB32_TIMCLK1;
#endif
    }
#endif

#if WB32_PWM_USE_TIM4
    if (&PWMD4 == pwmp) {
      rccEnableTIM4();
      rccResetTIM4();
#if !defined(WB32_TIM4_SUPPRESS_ISR)
      nvicEnableVector(WB32_TIM4_NUMBER, WB32_PWM_TIM4_IRQ_PRIORITY);
#endif
#if defined(WB32_TIM4CLK)
      pwmp->clock = WB32_TIM4CLK;
#else
      pwmp->clock = WB32_TIMCLK1;
#endif
    }
#endif
    /* All channels configured in PWM1 mode with preload enabled and
       will stay that way until the driver is stopped.*/
    pwmp->tim->CCMR1 = WB32_TIM_CCMR1_OC1M(6) | WB32_TIM_CCMR1_OC1PE |
                       WB32_TIM_CCMR1_OC2M(6) | WB32_TIM_CCMR1_OC2PE;
    pwmp->tim->CCMR2 = WB32_TIM_CCMR2_OC3M(6) | WB32_TIM_CCMR2_OC3PE |
                       WB32_TIM_CCMR2_OC4M(6) | WB32_TIM_CCMR2_OC4PE;
#if WB32_TIM_MAX_CHANNELS > 4
    pwmp->tim->CCMR3 = WB32_TIM_CCMR3_OC5M(6) | WB32_TIM_CCMR3_OC5PE |
                       WB32_TIM_CCMR3_OC6M(6) | WB32_TIM_CCMR3_OC6PE;
#endif
  }
  else {
    /* Driver re-configuration scenario, it must be stopped first.*/
    pwmp->tim->CR1 = 0;             /* Timer disabled.*/
    pwmp->tim->CCR[0] = 0;          /* Comparator 1 disabled.*/
    pwmp->tim->CCR[1] = 0;          /* Comparator 2 disabled.*/
    pwmp->tim->CCR[2] = 0;          /* Comparator 3 disabled.*/
    pwmp->tim->CCR[3] = 0;          /* Comparator 4 disabled.*/
#if WB32_TIM_MAX_CHANNELS > 4
    if (pwmp->channels > 4) {
      pwmp->tim->CCXR[0] = 0;       /* Comparator 5 disabled.*/
      pwmp->tim->CCXR[1] = 0;       /* Comparator 6 disabled.*/
    }
#endif
    pwmp->tim->CNT = 0;             /* Counter reset to zero.*/
  }

  /* Timer configuration.*/
  psc = (pwmp->clock / pwmp->config->frequency) - 1;
  osalDbgAssert((psc <= 0xFFFF) &&
                ((psc + 1) * pwmp->config->frequency) == pwmp->clock,
                "invalid frequency");
  pwmp->tim->PSC  = psc;
  pwmp->tim->ARR  = pwmp->period - 1;
  pwmp->tim->CR2  = pwmp->config->cr2;

  /* Output enables and polarities setup.*/
  ccer = 0;
  switch (pwmp->config->channels[0].mode & PWM_OUTPUT_MASK) {
    case PWM_OUTPUT_ACTIVE_LOW:
      ccer |= WB32_TIM_CCER_CC1P;
    /* Falls through.*/
    case PWM_OUTPUT_ACTIVE_HIGH:
      ccer |= WB32_TIM_CCER_CC1E;
    /* Falls through.*/
    default:
      ;
  }
  switch (pwmp->config->channels[1].mode & PWM_OUTPUT_MASK) {
    case PWM_OUTPUT_ACTIVE_LOW:
      ccer |= WB32_TIM_CCER_CC2P;
    /* Falls through.*/
    case PWM_OUTPUT_ACTIVE_HIGH:
      ccer |= WB32_TIM_CCER_CC2E;
    /* Falls through.*/
    default:
      ;
  }
  switch (pwmp->config->channels[2].mode & PWM_OUTPUT_MASK) {
    case PWM_OUTPUT_ACTIVE_LOW:
      ccer |= WB32_TIM_CCER_CC3P;
    /* Falls through.*/
    case PWM_OUTPUT_ACTIVE_HIGH:
      ccer |= WB32_TIM_CCER_CC3E;
    /* Falls through.*/
    default:
      ;
  }
  switch (pwmp->config->channels[3].mode & PWM_OUTPUT_MASK) {
    case PWM_OUTPUT_ACTIVE_LOW:
      ccer |= WB32_TIM_CCER_CC4P;
    /* Falls through.*/
    case PWM_OUTPUT_ACTIVE_HIGH:
      ccer |= WB32_TIM_CCER_CC4E;
    /* Falls through.*/
    default:
      ;
  }
#if WB32_PWM_USE_ADVANCED
#if WB32_PWM_USE_TIM1 && !WB32_PWM_USE_TIM8 && !WB32_PWM_USE_TIM20
  if (&PWMD1 == pwmp) {
#endif
    switch (pwmp->config->channels[0].mode & PWM_COMPLEMENTARY_OUTPUT_MASK) {
      case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW:
        ccer |= WB32_TIM_CCER_CC1NP;
      /* Falls through.*/
      case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH:
        ccer |= WB32_TIM_CCER_CC1NE;
      /* Falls through.*/
      default:
        ;
    }
    switch (pwmp->config->channels[1].mode & PWM_COMPLEMENTARY_OUTPUT_MASK) {
      case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW:
        ccer |= WB32_TIM_CCER_CC2NP;
      /* Falls through.*/
      case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH:
        ccer |= WB32_TIM_CCER_CC2NE;
      /* Falls through.*/
      default:
        ;
    }
    switch (pwmp->config->channels[2].mode & PWM_COMPLEMENTARY_OUTPUT_MASK) {
      case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW:
        ccer |= WB32_TIM_CCER_CC3NP;
      /* Falls through.*/
      case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH:
        ccer |= WB32_TIM_CCER_CC3NE;
      /* Falls through.*/
      default:
        ;
    }
    switch (pwmp->config->channels[3].mode & PWM_COMPLEMENTARY_OUTPUT_MASK) {
      case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW:
        ccer |= WB32_TIM_CCER_CC4NP;
      /* Falls through.*/
      case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH:
        ccer |= WB32_TIM_CCER_CC4NE;
      /* Falls through.*/
      default:
        ;
    }
  }
#endif /* WB32_PWM_USE_ADVANCED*/

  pwmp->tim->CCER = ccer;
  /* Update event. */
  pwmp->tim->EGR = WB32_TIM_EGR_UG;
  /* Clear pending IRQs. */
  pwmp->tim->SR = 0;
  /* DMA-related DIER settings. */
  pwmp->tim->DIER = pwmp->config->dier & ~WB32_TIM_DIER_IRQ_MASK;
#if WB32_PWM_USE_TIM1
#if WB32_PWM_USE_ADVANCED
  pwmp->tim->BDTR = pwmp->config->bdtr | WB32_TIM_BDTR_MOE;
#else
  pwmp->tim->BDTR = WB32_TIM_BDTR_MOE;
#endif
#endif
  /* Timer configured and started.*/
  pwmp->tim->CR1 = WB32_TIM_CR1_ARPE | WB32_TIM_CR1_URS | WB32_TIM_CR1_CEN;
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmp) {

  /* If in ready state then disables the PWM clock.*/
  if (pwmp->state == PWM_READY) {
    /* Timer disabled.*/
    pwmp->tim->CR1 = 0;
    /* All IRQs disabled.*/
    pwmp->tim->DIER = 0;
    /* Clear eventual pending IRQs.*/
    pwmp->tim->SR = 0;
#if WB32_PWM_USE_TIM1 || WB32_PWM_USE_TIM8 || WB32_PWM_USE_TIM20
    pwmp->tim->BDTR = 0;
#endif

#if WB32_PWM_USE_TIM1
    if (&PWMD1 == pwmp) {
#if !defined(WB32_TIM1_SUPPRESS_ISR)
      nvicDisableVector(WB32_TIM1_UP_NUMBER);
      nvicDisableVector(WB32_TIM1_CC_NUMBER);
#endif
      rccDisableTIM1();
    }
#endif

#if WB32_PWM_USE_TIM2
    if (&PWMD2 == pwmp) {
#if !defined(WB32_TIM2_SUPPRESS_ISR)
      nvicDisableVector(WB32_TIM2_NUMBER);
#endif
      rccDisableTIM2();
    }
#endif

#if WB32_PWM_USE_TIM3
    if (&PWMD3 == pwmp) {
#if !defined(WB32_TIM3_SUPPRESS_ISR)
      nvicDisableVector(WB32_TIM3_NUMBER);
#endif
      rccDisableTIM3();
    }
#endif

#if WB32_PWM_USE_TIM4
    if (&PWMD4 == pwmp) {
#if !defined(WB32_TIM4_SUPPRESS_ISR)
      nvicDisableVector(WB32_TIM4_NUMBER);
#endif
      rccDisableTIM4();
    }
#endif
  }
}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 * @note    The function has effect at the next cycle start.
 * @note    Channel notification is not enabled.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @notapi
 */
void pwm_lld_enable_channel(PWMDriver *pwmp,
                            pwmchannel_t channel,
                            pwmcnt_t width) {

  /* Changing channel duty cycle on the fly.*/
#if WB32_TIM_MAX_CHANNELS <= 4
  pwmp->tim->CCR[channel] = width;
#else
  if (channel < 4)
    pwmp->tim->CCR[channel] = width;
  else
    pwmp->tim->CCXR[channel - 4] = width;
#endif
}

/**
 * @brief   Disables a PWM channel and its notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel) {

#if WB32_TIM_MAX_CHANNELS <= 4
  pwmp->tim->CCR[channel] = 0;
  pwmp->tim->DIER &= ~(2 << channel);
#else
  if (channel < 4) {
    pwmp->tim->CCR[channel] = 0;
    pwmp->tim->DIER &= ~(2 << channel);
  }
  else
    pwmp->tim->CCXR[channel - 4] = 0;
#endif
}

/**
 * @brief   Enables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_enable_periodic_notification(PWMDriver *pwmp) {
  uint32_t dier = pwmp->tim->DIER;

  /* If the IRQ is not already enabled care must be taken to clear it,
     it is probably already pending because the timer is running.*/
  if ((dier & WB32_TIM_DIER_UIE) == 0) {
    pwmp->tim->SR = ~WB32_TIM_SR_UIF;
    pwmp->tim->DIER = dier | WB32_TIM_DIER_UIE;
  }
}

/**
 * @brief   Disables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_disable_periodic_notification(PWMDriver *pwmp) {

  pwmp->tim->DIER &= ~WB32_TIM_DIER_UIE;
}

/**
 * @brief   Enables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                         pwmchannel_t channel) {
  uint32_t dier = pwmp->tim->DIER;

#if WB32_TIM_MAX_CHANNELS > 4
  /* Channels 4 and 5 do not support callbacks.*/
  osalDbgAssert(channel < 4, "callback not supported");
#endif

  /* If the IRQ is not already enabled care must be taken to clear it,
     it is probably already pending because the timer is running.*/
  if ((dier & (2 << channel)) == 0) {
    pwmp->tim->SR = ~(2 << channel);
    pwmp->tim->DIER = dier | (2 << channel);
  }
}

/**
 * @brief   Disables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel_notification(PWMDriver *pwmp,
                                          pwmchannel_t channel) {

  pwmp->tim->DIER &= ~(2 << channel);
}

/**
 * @brief   Common TIM2...TIM5,TIM9 IRQ handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_serve_interrupt(PWMDriver *pwmp) {
  uint32_t sr;

  sr = pwmp->tim->SR;
  sr &= pwmp->tim->DIER & WB32_TIM_DIER_IRQ_MASK;
  pwmp->tim->SR = ~sr;
  if (((sr & WB32_TIM_SR_CC1IF) != 0) &&
      (pwmp->config->channels[0].callback != NULL))
    pwmp->config->channels[0].callback(pwmp);
  if (((sr & WB32_TIM_SR_CC2IF) != 0) &&
      (pwmp->config->channels[1].callback != NULL))
    pwmp->config->channels[1].callback(pwmp);
  if (((sr & WB32_TIM_SR_CC3IF) != 0) &&
      (pwmp->config->channels[2].callback != NULL))
    pwmp->config->channels[2].callback(pwmp);
  if (((sr & WB32_TIM_SR_CC4IF) != 0) &&
      (pwmp->config->channels[3].callback != NULL))
    pwmp->config->channels[3].callback(pwmp);
  if (((sr & WB32_TIM_SR_UIF) != 0) && (pwmp->config->callback != NULL))
    pwmp->config->callback(pwmp);
}

#endif /* HAL_USE_PWM */

/** @} */
