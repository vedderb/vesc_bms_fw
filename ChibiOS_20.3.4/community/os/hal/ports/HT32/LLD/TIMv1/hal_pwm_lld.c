/*
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
 * @file    hal_pwm_lld.c
 * @brief   HT32 PWM subsystem low level driver source.
 *
 * @addtogroup PWM
 * @{
 */

#include "hal.h"

#if (HAL_USE_PWM == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PWMD0 driver identifier.
 * @note    The driver PWMD0 allocates the complex timer XXX when enabled.
 */
#if (HT32_PWM_USE_MCTM0 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD_MCTM0;
#endif

#if (HT32_PWM_USE_MCTM1 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD_MCTM1;
#endif

#if (HT32_PWM_USE_GPTM0 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD_GPTM0;
#endif

#if (HT32_PWM_USE_GPTM1 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD_GPTM1;
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

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {

#if HT32_PWM_USE_MCTM0 == TRUE
  /* Driver initialization.*/
  pwmObjectInit(&PWMD_MCTM0);
#endif
#if HT32_PWM_USE_MCTM1 == TRUE
  pwmObjectInit(&PWMD_MCTM1);
#endif
#if HT32_PWM_USE_GPTM0 == TRUE
  pwmObjectInit(&PWMD_GPTM0);
#endif
#if HT32_PWM_USE_GPTM1 == TRUE
  pwmObjectInit(&PWMD_GPTM1);
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

  if (pwmp->state == PWM_STOP) {
    /* Clock activation and timer reset.*/
#if HT32_PWM_USE_MCTM0 == TRUE
    if (&PWMD_MCTM0 == pwmp) {
        CKCU->APBCCR1 |= CKCU_APBCCR1_MCTM0EN;
        pwmp->TM = MCTM0;
    }
#endif
#if HT32_PWM_USE_MCTM1 == TRUE
    if (&PWMD_MCTM1 == pwmp) {
        CKCU->APBCCR1 |= CKCU_APBCCR1_MCTM1EN;
        pwmp->TM = MCTM1;
    }
#endif
#if HT32_PWM_USE_GPTM0 == TRUE
    if (&PWMD_GPTM0 == pwmp) {
        CKCU->APBCCR1 |= CKCU_APBCCR1_GPTM0EN;
        pwmp->TM = GPTM0;
    }
#endif
#if HT32_PWM_USE_GPTM1 == TRUE
    if (&PWMD_GPTM1 == pwmp) {
        CKCU->APBCCR1 |= CKCU_APBCCR1_GPTM1EN;
        pwmp->TM = GPTM1;
    }
#endif
  }


  pwmp->TM->CNTCFR = TM_CNTCFR_CMSEL_MODE_0;
  pwmp->TM->PSCR = (HT32_CK_AHB_FREQUENCY / pwmp->config->frequency) - 1;
  pwmp->TM->CRR = pwmp->config->period;
  pwmp->TM->CNTR = 0;
  pwmp->TM->CTR = TM_CTR_TME;
  pwmp->TM->CHCTR = 0;
#if (HT32_PWM_USE_MCTM0 == TRUE) || (HT32_PWM_USE_MCTM1 == TRUE)
  if (pwmp->TM == MCTM0 || pwmp->TM == MCTM1)
    pwmp->TM->CHBRKCTR = TM_CHBRKCTR_CHMOE;
#endif
  for (size_t channel = 0; channel < PWM_CHANNELS; channel++) {
    switch (pwmp->config->channels[channel].mode & PWM_OUTPUT_MASK) {
    case PWM_OUTPUT_DISABLED:
      break;
    case PWM_OUTPUT_ACTIVE_HIGH:
      pwmp->TM->CHPOLR &= ~(1U << (2*channel));
      break;
    case PWM_OUTPUT_ACTIVE_LOW:
      pwmp->TM->CHPOLR |= 1U << (2*channel);
      break;
    default:
      break;
    }
    pwmp->TM->CHnOCFR[channel] = TM_CHnOCFR_CHnPRE|TM_CHnOCFR_CHnOM(6);
  }
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
#if HT32_PWM_USE_MCTM0 == TRUE
    if (&PWMD_MCTM0 == pwmp) {
        CKCU->APBCCR1 &= ~CKCU_APBCCR1_MCTM0EN;
    }
#endif
#if HT32_PWM_USE_MCTM1 == TRUE
    if (&PWMD_MCTM1 == pwmp) {
        CKCU->APBCCR1 &= ~CKCU_APBCCR1_MCTM1EN;
    }
#endif
#if HT32_PWM_USE_GPTM0 == TRUE
    if (&PWMD_GPTM0 == pwmp) {
        CKCU->APBCCR1 &= ~CKCU_APBCCR1_GPTM0EN;
    }
#endif
#if HT32_PWM_USE_GPTM1 == TRUE
    if (&PWMD_GPTM1 == pwmp) {
        CKCU->APBCCR1 &= ~CKCU_APBCCR1_GPTM1EN;
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

  pwmp->TM->CHnCCR[channel] = width;
  pwmp->TM->CHCTR |= 1U << (2*channel);
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

  pwmp->TM->CHCTR &= ~(1U << (2*channel));
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

  (void)pwmp;
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

  (void)pwmp;
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

  (void)pwmp;
  (void)channel;
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

  (void)pwmp;
  (void)channel;
}

#endif /* HAL_USE_PWM == TRUE */

/** @} */
