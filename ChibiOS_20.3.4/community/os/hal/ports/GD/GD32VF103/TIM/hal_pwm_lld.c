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
 * @file    TIM/hal_pwm_lld.c
 * @brief   GD32 PWM subsystem low level driver header.
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
 * @note    The driver PWMD1 allocates the complex timer TIM0 when enabled.
 */
#if GD32_PWM_USE_TIM0 || defined(__DOXYGEN__)
PWMDriver PWMD1;
#endif

/**
 * @brief   PWMD2 driver identifier.
 * @note    The driver PWMD2 allocates the timer TIM1 when enabled.
 */
#if GD32_PWM_USE_TIM1 || defined(__DOXYGEN__)
PWMDriver PWMD2;
#endif

/**
 * @brief   PWMD3 driver identifier.
 * @note    The driver PWMD3 allocates the timer TIM2 when enabled.
 */
#if GD32_PWM_USE_TIM2 || defined(__DOXYGEN__)
PWMDriver PWMD3;
#endif

/**
 * @brief   PWMD4 driver identifier.
 * @note    The driver PWMD4 allocates the timer TIM3 when enabled.
 */
#if GD32_PWM_USE_TIM3 || defined(__DOXYGEN__)
PWMDriver PWMD4;
#endif

/**
 * @brief   PWMD5 driver identifier.
 * @note    The driver PWMD5 allocates the timer TIM4 when enabled.
 */
#if GD32_PWM_USE_TIM4 || defined(__DOXYGEN__)
PWMDriver PWMD5;
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

#if GD32_PWM_USE_TIM0 || defined(__DOXYGEN__)
#if !defined(GD32_TIM0_SUPPRESS_ISR)
#if !defined(GD32_TIM0_UP_HANDLER)
#error "GD32_TIM0_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM0 update interrupt handler.
 * @note    It is assumed that this interrupt is only activated if the callback
 *          pointer is not equal to @p NULL in order to not perform an extra
 *          check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM0_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  pwm_lld_serve_interrupt(&PWMD1);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(GD32_TIM0_CC_HANDLER)
#error "GD32_TIM0_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM0 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM0_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  pwm_lld_serve_interrupt(&PWMD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM0_SUPPRESS_ISR) */
#endif /* GD32_PWM_USE_TIM0 */

#if GD32_PWM_USE_TIM1 || defined(__DOXYGEN__)
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

  pwm_lld_serve_interrupt(&PWMD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM1_SUPPRESS_ISR) */
#endif /* GD32_PWM_USE_TIM1 */

#if GD32_PWM_USE_TIM2 || defined(__DOXYGEN__)
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

  pwm_lld_serve_interrupt(&PWMD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM2_SUPPRESS_ISR) */
#endif /* GD32_PWM_USE_TIM2 */

#if GD32_PWM_USE_TIM3 || defined(__DOXYGEN__)
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

  pwm_lld_serve_interrupt(&PWMD4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM3_SUPPRESS_ISR) */
#endif /* GD32_PWM_USE_TIM3 */

#if GD32_PWM_USE_TIM4 || defined(__DOXYGEN__)
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

  pwm_lld_serve_interrupt(&PWMD5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM4_SUPPRESS_ISR) */
#endif /* GD32_PWM_USE_TIM4 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {

#if GD32_PWM_USE_TIM0
  /* Driver initialization.*/
  pwmObjectInit(&PWMD1);
  PWMD1.channels = GD32_TIM0_CHANNELS;
  PWMD1.tim = GD32_TIM0;
#endif

#if GD32_PWM_USE_TIM1
  /* Driver initialization.*/
  pwmObjectInit(&PWMD2);
  PWMD2.channels = GD32_TIM1_CHANNELS;
  PWMD2.tim = GD32_TIM1;
#endif

#if GD32_PWM_USE_TIM2
  /* Driver initialization.*/
  pwmObjectInit(&PWMD3);
  PWMD3.channels = GD32_TIM2_CHANNELS;
  PWMD3.tim = GD32_TIM2;
#endif

#if GD32_PWM_USE_TIM3
  /* Driver initialization.*/
  pwmObjectInit(&PWMD4);
  PWMD4.channels = GD32_TIM3_CHANNELS;
  PWMD4.tim = GD32_TIM3;
#endif

#if GD32_PWM_USE_TIM4
  /* Driver initialization.*/
  pwmObjectInit(&PWMD5);
  PWMD5.channels = GD32_TIM4_CHANNELS;
  PWMD5.tim = GD32_TIM4;
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
#if GD32_PWM_USE_TIM0
    if (&PWMD1 == pwmp) {
      rcuEnableTIM0(true);
      rcuResetTIM0();
#if !defined(GD32_TIM0_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM0_UP_NUMBER, GD32_PWM_TIM0_IRQ_PRIORITY, GD32_PWM_TIM0_IRQ_TRIGGER);
      eclicEnableVector(GD32_TIM0_CC_NUMBER, GD32_PWM_TIM0_IRQ_PRIORITY, GD32_PWM_TIM0_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM0CLK)
      pwmp->clock = GD32_TIM0CLK;
#else
      pwmp->clock = GD32_TIMCLK2;
#endif
    }
#endif

#if GD32_PWM_USE_TIM1
    if (&PWMD2 == pwmp) {
      rcuEnableTIM1(true);
      rcuResetTIM1();
#if !defined(GD32_TIM1_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM1_NUMBER, GD32_PWM_TIM1_IRQ_PRIORITY, GD32_PWM_TIM1_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM1CLK)
      pwmp->clock = GD32_TIM1CLK;
#else
      pwmp->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_PWM_USE_TIM2
    if (&PWMD3 == pwmp) {
      rcuEnableTIM2(true);
      rcuResetTIM2();
#if !defined(GD32_TIM2_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM2_NUMBER, GD32_PWM_TIM2_IRQ_PRIORITY, GD32_PWM_TIM2_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM2CLK)
      pwmp->clock = GD32_TIM2CLK;
#else
      pwmp->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_PWM_USE_TIM3
    if (&PWMD4 == pwmp) {
      rcuEnableTIM3(true);
      rcuResetTIM3();
#if !defined(GD32_TIM3_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM3_NUMBER, GD32_PWM_TIM3_IRQ_PRIORITY, GD32_PWM_TIM3_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM3CLK)
      pwmp->clock = GD32_TIM3CLK;
#else
      pwmp->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_PWM_USE_TIM4
    if (&PWMD5 == pwmp) {
      rcuEnableTIM4(true);
      rcuResetTIM4();
#if !defined(GD32_TIM4_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM4_NUMBER, GD32_PWM_TIM4_IRQ_PRIORITY, GD32_PWM_TIM4_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM4CLK)
      pwmp->clock = GD32_TIM4CLK;
#else
      pwmp->clock = GD32_TIMCLK1;
#endif
    }
#endif

    /* All channels configured in PWM1 mode with preload enabled and will
       stay that way until the driver is stopped.*/
    pwmp->tim->CHCTL0 = GD32_TIM_CCMR1_OC1M(6) | GD32_TIM_CCMR1_OC1PE |
                       GD32_TIM_CCMR1_OC2M(6) | GD32_TIM_CCMR1_OC2PE;
    pwmp->tim->CHCTL1 = GD32_TIM_CCMR2_OC3M(6) | GD32_TIM_CCMR2_OC3PE |
                       GD32_TIM_CCMR2_OC4M(6) | GD32_TIM_CCMR2_OC4PE;
  }
  else {
    /* Driver re-configuration scenario, it must be stopped first.*/
    pwmp->tim->CTL0    = 0;                  /* Timer disabled.              */
    pwmp->tim->CHCV[0] = 0;                  /* Comparator 1 disabled.       */
    pwmp->tim->CHCV[1] = 0;                  /* Comparator 2 disabled.       */
    pwmp->tim->CHCV[2] = 0;                  /* Comparator 3 disabled.       */
    pwmp->tim->CHCV[3] = 0;                  /* Comparator 4 disabled.       */
    pwmp->tim->CNT  = 0;                    /* Counter reset to zero.       */
  }

  /* Timer configuration.*/
  psc = (pwmp->clock / pwmp->config->frequency) - 1;
  osalDbgAssert((psc <= 0xFFFF) &&
                ((psc + 1) * pwmp->config->frequency) == pwmp->clock,
                "invalid frequency");
  pwmp->tim->PSC  = psc;
  pwmp->tim->CAR  = pwmp->period - 1;
  pwmp->tim->CTL1  = pwmp->config->ctl1;

  /* Output enables and polarities setup.*/
  ccer = 0;
  switch (pwmp->config->channels[0].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    ccer |= GD32_TIM_CCER_CC1P;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    ccer |= GD32_TIM_CCER_CC1E;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[1].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    ccer |= GD32_TIM_CCER_CC2P;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    ccer |= GD32_TIM_CCER_CC2E;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[2].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    ccer |= GD32_TIM_CCER_CC3P;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    ccer |= GD32_TIM_CCER_CC3E;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[3].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    ccer |= GD32_TIM_CCER_CC4P;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    ccer |= GD32_TIM_CCER_CC4E;
    /* Falls through.*/
  default:
    ;
  }
#if GD32_PWM_USE_ADVANCED
  if (&PWMD1 == pwmp) {
    switch (pwmp->config->channels[0].mode & PWM_COMPLEMENTARY_OUTPUT_MASK) {
    case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW:
      ccer |= GD32_TIM_CCER_CC1NP;
      /* Falls through.*/
    case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH:
      ccer |= GD32_TIM_CCER_CC1NE;
      /* Falls through.*/
    default:
      ;
    }
    switch (pwmp->config->channels[1].mode & PWM_COMPLEMENTARY_OUTPUT_MASK) {
    case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW:
      ccer |= GD32_TIM_CCER_CC2NP;
      /* Falls through.*/
    case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH:
      ccer |= GD32_TIM_CCER_CC2NE;
      /* Falls through.*/
    default:
      ;
    }
    switch (pwmp->config->channels[2].mode & PWM_COMPLEMENTARY_OUTPUT_MASK) {
    case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW:
      ccer |= GD32_TIM_CCER_CC3NP;
      /* Falls through.*/
    case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH:
      ccer |= GD32_TIM_CCER_CC3NE;
      /* Falls through.*/
    default:
      ;
    }
    switch (pwmp->config->channels[3].mode & PWM_COMPLEMENTARY_OUTPUT_MASK) {
    case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW:
      ccer |= GD32_TIM_CCER_CC4NP;
      /* Falls through.*/
    case PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH:
      ccer |= GD32_TIM_CCER_CC4NE;
      /* Falls through.*/
    default:
      ;
    }
  }
#endif /* GD32_PWM_USE_ADVANCED*/

  pwmp->tim->CHCTL2  = ccer;
  pwmp->tim->SWEVG   = GD32_TIM_EGR_UG;      /* Update event.                */
  pwmp->tim->INTF    = 0;                     /* Clear pending IRQs.          */
  pwmp->tim->DMAINTEN  = pwmp->config->dmainten &   /* DMA-related DIER settings.   */
                     ~GD32_TIM_DIER_IRQ_MASK;
#if GD32_PWM_USE_TIM0 
#if GD32_PWM_USE_ADVANCED
  pwmp->tim->CCHP  = pwmp->config->cchp | GD32_TIM_BDTR_MOE;
#else
  pwmp->tim->CCHP  = GD32_TIM_BDTR_MOE;
#endif
#endif
  /* Timer configured and started.*/
  pwmp->tim->CTL0   = GD32_TIM_CR1_ARPE | GD32_TIM_CR1_URS |
                     GD32_TIM_CR1_CEN;
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
    pwmp->tim->CTL0  = 0;                    /* Timer disabled.              */
    pwmp->tim->DMAINTEN = 0;                    /* All IRQs disabled.           */
    pwmp->tim->INTF   = 0;                    /* Clear eventual pending IRQs. */
#if GD32_PWM_USE_TIM0
    pwmp->tim->CCHP  = 0;
#endif

#if GD32_PWM_USE_TIM0
    if (&PWMD1 == pwmp) {
#if !defined(GD32_TIM0_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM0_UP_NUMBER);
      eclicDisableVector(GD32_TIM0_CC_NUMBER);
#endif
      rcuDisableTIM0();
    }
#endif

#if GD32_PWM_USE_TIM1
    if (&PWMD2 == pwmp) {
#if !defined(GD32_TIM1_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM1_NUMBER);
#endif
      rcuDisableTIM1();
    }
#endif

#if GD32_PWM_USE_TIM2
    if (&PWMD3 == pwmp) {
#if !defined(GD32_TIM2_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM2_NUMBER);
#endif
      rcuDisableTIM2();
    }
#endif

#if GD32_PWM_USE_TIM3
    if (&PWMD4 == pwmp) {
#if !defined(GD32_TIM3_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM3_NUMBER);
#endif
      rcuDisableTIM3();
    }
#endif

#if GD32_PWM_USE_TIM4
    if (&PWMD5 == pwmp) {
#if !defined(GD32_TIM4_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM4_NUMBER);
#endif
      rcuDisableTIM4();
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
  pwmp->tim->CHCV[channel] = width;
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
  pwmp->tim->CHCV[channel] = 0;
  pwmp->tim->DMAINTEN &= ~(2 << channel);
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
  uint32_t dier = pwmp->tim->DMAINTEN;

  /* If the IRQ is not already enabled care must be taken to clear it,
     it is probably already pending because the timer is running.*/
  if ((dier & GD32_TIM_DIER_UIE) == 0) {
    pwmp->tim->INTF   = ~GD32_TIM_SR_UIF;
    pwmp->tim->DMAINTEN = dier | GD32_TIM_DIER_UIE;
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

  pwmp->tim->DMAINTEN &= ~GD32_TIM_DIER_UIE;
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
  uint32_t dier = pwmp->tim->DMAINTEN;

  /* If the IRQ is not already enabled care must be taken to clear it,
     it is probably already pending because the timer is running.*/
  if ((dier & (2 << channel)) == 0) {
    pwmp->tim->INTF   = ~(2 << channel);
    pwmp->tim->DMAINTEN = dier | (2 << channel);
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

  pwmp->tim->DMAINTEN &= ~(2 << channel);
}

/**
 * @brief   Common TIM1...TIM4 IRQ handler.
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

  sr  = pwmp->tim->INTF;
  sr &= pwmp->tim->DMAINTEN & GD32_TIM_DIER_IRQ_MASK;
  pwmp->tim->INTF = ~sr;
  if (((sr & GD32_TIM_SR_CC1IF) != 0) &&
      (pwmp->config->channels[0].callback != NULL))
    pwmp->config->channels[0].callback(pwmp);
  if (((sr & GD32_TIM_SR_CC2IF) != 0) &&
      (pwmp->config->channels[1].callback != NULL))
    pwmp->config->channels[1].callback(pwmp);
  if (((sr & GD32_TIM_SR_CC3IF) != 0) &&
      (pwmp->config->channels[2].callback != NULL))
    pwmp->config->channels[2].callback(pwmp);
  if (((sr & GD32_TIM_SR_CC4IF) != 0) &&
      (pwmp->config->channels[3].callback != NULL))
    pwmp->config->channels[3].callback(pwmp);
  if (((sr & GD32_TIM_SR_UIF) != 0) && (pwmp->config->callback != NULL))
    pwmp->config->callback(pwmp);
}

#endif /* HAL_USE_PWM */

/** @} */
