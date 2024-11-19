/*
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
 * @file    TIMv1/hal_pwm_lld.c
 * @brief   PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#include "hal.h"

#if (HAL_USE_PWM == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define NUC123_PWM_CLKSRC_HSE  0x0UL
#define NUC123_PWM_CLKSRC_HCLK 0x2UL
#define NUC123_PWM_CLKSRC_HSI  0x3UL
#define NUC123_PWM_CLKSRC_LSI  0x7UL

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PWMD1 driver identifier.
 */
#if (NUC123_PWM_USE_PWM1 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

typedef struct {
    __IO uint32_t CNR;          /* Offset: 0x0C  PWM Counter Register 0                                             */
    __IO uint32_t CMR;          /* Offset: 0x10  PWM Comparator Register 0                                          */
    __I  uint32_t PDR;          /* Offset: 0x14  PWM Data Register 0                                                */
} PWM_CHANNEL_T;

#define PWMA_CHANNELS_BASE        (PWMA_BASE + 0xC)
#define PWMA_CHANNELS             ((PWM_CHANNEL_T *) PWMA_CHANNELS_BASE)

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared IRQ handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @param[in] pwmd     pointer to a @p PWMDriver object
 *
 * @notapi
 */
static void pwm_lld_serve_interrupt(PWMDriver *pwmp) {

  uint32_t piir;
  uint32_t pier_clr_msk;
  uint32_t poe_clr_msk;

  pier_clr_msk = 0;
  poe_clr_msk  = 0;
  piir = PWMA->PIIR;
  PWMA->PIIR = piir;

  if (piir & PWM_PIIR_PWMDIF0_Msk) {
    pwmp->config->channels[0].callback(pwmp);
  }

  if (piir & PWM_PIIR_PWMDIF1_Msk) {
    pwmp->config->channels[1].callback(pwmp);
  }

  if (piir & PWM_PIIR_PWMDIF2_Msk) {
    pwmp->config->channels[2].callback(pwmp);
  }

  if (piir & PWM_PIIR_PWMDIF3_Msk) {
    pwmp->config->channels[3].callback(pwmp);
  }

  if (piir & PWM_PIIR_PWMIF0_Msk) {
    if (pwmp->periodic_callback_enabled) {
      pwmp->config->callback(pwmp);
    }
    if (!pwmIsChannelEnabledI(pwmp, 0)) {
      poe_clr_msk |= PWM_POE_PWM1_Msk;
      pier_clr_msk |= PWM_PIER_PWMDIE0_Msk;
    }
  }

  if (piir & PWM_PIIR_PWMIF1_Msk) {
    poe_clr_msk |= PWM_POE_PWM1_Msk;
    pier_clr_msk |= PWM_PIER_PWMDIE1_Msk;
    pier_clr_msk |= PWM_PIER_PWMIE1_Msk;
  }

  if (piir & PWM_PIIR_PWMIF2_Msk) {
    poe_clr_msk |= PWM_POE_PWM2_Msk;
    pier_clr_msk |= PWM_PIER_PWMDIE2_Msk;
    pier_clr_msk |= PWM_PIER_PWMIE2_Msk;
  }

  if (piir & PWM_PIIR_PWMIF3_Msk) {
    poe_clr_msk |= PWM_POE_PWM3_Msk;
    pier_clr_msk |= PWM_PIER_PWMDIE3_Msk;
    pier_clr_msk |= PWM_PIER_PWMIE3_Msk;
  }

  PWMA->POE  &= ~poe_clr_msk;
  PWMA->PIER &= ~pier_clr_msk;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   PWM interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(NUC123_PWMA_HANDLER)
{
  OSAL_IRQ_PROLOGUE();
  pwm_lld_serve_interrupt(&PWMD1);
  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void)
{

#if (NUC123_PWM_USE_PWM1 == TRUE)
  /* Driver initialization.*/
  pwmObjectInit(&PWMD1);
  PWMD1.channels = PWM_CHANNELS;
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
void pwm_lld_start(PWMDriver *pwmp)
{
  uint32_t clksel1;
  uint32_t clksel2;
  uint32_t prescale;
  uint32_t pcr;
  if (pwmp->state == PWM_STOP) {
    clksel1 = CLK->CLKSEL1;
    clksel2 = CLK->CLKSEL2;
    clksel1 = (clksel1 & ~(CLK_CLKSEL1_PWM01_S_Msk)) |
              ((NUC123_PWM_CLKSRC_HCLK << CLK_CLKSEL1_PWM01_S_Pos) &
               CLK_CLKSEL1_PWM01_S_Msk);
    clksel1 = (clksel1 & ~(CLK_CLKSEL1_PWM23_S_Msk)) |
              ((NUC123_PWM_CLKSRC_HCLK << CLK_CLKSEL1_PWM23_S_Pos) &
               CLK_CLKSEL1_PWM23_S_Msk);
    clksel2 = (clksel2 & ~(CLK_CLKSEL2_PWM01_S_E_Msk)) |
              ((NUC123_PWM_CLKSRC_HCLK >> 2) << CLK_CLKSEL2_PWM01_S_E_Pos);
    clksel2 = (clksel2 & ~(CLK_CLKSEL2_PWM23_S_E_Msk)) |
              ((NUC123_PWM_CLKSRC_HCLK >> 2) << CLK_CLKSEL2_PWM23_S_E_Pos);
    CLK->CLKSEL1 = clksel1;
    CLK->CLKSEL2 = clksel2;

    CLK->APBCLK |= (CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_PWM23_EN_Msk);
    nvicEnableVector(NUC123_PWMA_NUMBER, NUC123_PWM_IRQ_PRIORITY);
    SYS->IPRSTC2 |= SYS_IPRSTC2_PWM03_RST_Msk;
    SYS->IPRSTC2 &= ~(SYS_IPRSTC2_PWM03_RST_Msk);

    /* Set clock scaling to 1 */
    PWMA->CSR = (4 << PWM_CSR_CSR0_Pos) | (4 << PWM_CSR_CSR1_Pos) |
                (4 << PWM_CSR_CSR2_Pos) | (4 << PWM_CSR_CSR3_Pos);

    /* Set prescale to set frequency */
    prescale = NUC123_HCLK / pwmp->config->frequency;
    PWMA->PPR = (PWMA->PPR & ~(PWM_PPR_CP01_Msk | PWM_PPR_CP23_Msk)) |
                ((prescale << PWM_PPR_CP01_Pos) & PWM_PPR_CP01_Msk) |
                ((prescale << PWM_PPR_CP23_Pos) & PWM_PPR_CP23_Msk);

    /*
     * PINV == 0 -> active high, PINV == 1 -> active low.
     * CHnINV vs CHnPINV:
     *   - CHnINV inverts before deadzone generation
     *   - CHnPINV inverts after deadzone generation
     * If no deadzone generation, INV(signal) == PINV(signal) and signal == INV(PINV(signal))
     * If there is deadzone generation, things get more complicated.
     */

    pcr = PWMA->PCR;

    pcr &= ~(PWM_PCR_CH0PINV_Msk | PWM_PCR_CH0INV_Msk);
    pcr &= ~(PWM_PCR_CH1PINV_Msk | PWM_PCR_CH1INV_Msk);
    pcr &= ~(PWM_PCR_CH2PINV_Msk | PWM_PCR_CH2INV_Msk);
    pcr &= ~(PWM_PCR_CH3PINV_Msk | PWM_PCR_CH3INV_Msk);

    /* Conditionally (and branchlessly) set the bits for the inverted channels */
    pcr |= ((pwmp->config->channels[0].mode == PWM_OUTPUT_ACTIVE_LOW) * PWM_PCR_CH0PINV_Msk);
    pcr |= ((pwmp->config->channels[1].mode == PWM_OUTPUT_ACTIVE_LOW) * PWM_PCR_CH1PINV_Msk);
    pcr |= ((pwmp->config->channels[2].mode == PWM_OUTPUT_ACTIVE_LOW) * PWM_PCR_CH2PINV_Msk);
    pcr |= ((pwmp->config->channels[3].mode == PWM_OUTPUT_ACTIVE_LOW) * PWM_PCR_CH3PINV_Msk);

    /* Set to auto-reload (this will also reset CMR & CNR) */
    pcr |= (PWM_PCR_CH0MOD_Msk | PWM_PCR_CH1MOD_Msk);
    pcr |= (PWM_PCR_CH2MOD_Msk | PWM_PCR_CH3MOD_Msk);

    PWMA->PCR = pcr;

    /* TODO: Deadzone generation */

    /* Mux the pins per the channel configs */

    if (pwmp->config->channels[0].mode != PWM_OUTPUT_DISABLED) {
      if (pwmp->config->channels[0].pinmask & NUC123_PWM_CH0_PIN_PA12) {
        SYS->GPA_MFP |= (1 << 12);
      }
    }

    if (pwmp->config->channels[1].mode != PWM_OUTPUT_DISABLED) {
      if (pwmp->config->channels[1].pinmask & NUC123_PWM_CH1_PIN_PA13) {
        SYS->GPA_MFP |= (1 << 13);
      }
    }

    if (pwmp->config->channels[2].mode != PWM_OUTPUT_DISABLED) {
      if (pwmp->config->channels[2].pinmask & NUC123_PWM_CH2_PIN_PA14) {
        SYS->GPA_MFP |= (1 << 14);
      }

      if (pwmp->config->channels[2].pinmask & NUC123_PWM_CH2_PIN_PC12) {
        SYS->GPC_MFP |= (1 << 12);
        SYS->ALT_MFP |= SYS_ALT_MFP_PC12_MFP1_Msk;
      }
    }

    if (pwmp->config->channels[3].mode != PWM_OUTPUT_DISABLED) {
      if (pwmp->config->channels[3].pinmask & NUC123_PWM_CH3_PIN_PA15) {
        SYS->GPA_MFP |= (1 << 15);
        SYS->ALT_MFP &= ~SYS_ALT_MFP_PA15_MFP1_Msk;
      }

      if (pwmp->config->channels[3].pinmask & NUC123_PWM_CH3_PIN_PC13) {
        SYS->GPC_MFP |= (1 << 13);
        SYS->ALT_MFP |= SYS_ALT_MFP_PC13_MFP1_Msk;
      }
    }

    #if defined(NUC123xxxAEx)
    /* TODO: Implement PWM0 PC8 & PWM1 PB9 */
    #endif

    PWMA->PIER &= ~(PWM_PIER_INT01TYPE_Msk | PWM_PIER_INT23TYPE_Msk);

    pwm_lld_change_period(pwmp, pwmp->config->period);

    PWMA->PCR |= (PWM_PCR_CH0EN_Msk | PWM_PCR_CH1EN_Msk |
                  PWM_PCR_CH2EN_Msk | PWM_PCR_CH3EN_Msk);
  }
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmp)
{
  /* If in ready state then disables the PWM clock.*/
  if (pwmp->state == PWM_READY) {
#if NUC123_PWM_USE_PWM1 == TRUE
    if (&PWMD1 == pwmp) {
      CLK->APBCLK &= ~(CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_PWM23_EN_Msk);
      nvicDisableVector(NUC123_PWMA_NUMBER);
      return;
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
void pwm_lld_enable_channel(PWMDriver *pwmp, pwmchannel_t channel,
                            pwmcnt_t width)
{
  (void)pwmp;
  PWMA_CHANNELS[channel].CMR = width;
  PWMA->POE |= PWM_POE_PWM0_Msk << channel;
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
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel)
{
  (void)pwmp;
  /* We do not disable immediately, but enable the periodic interrupt
    We will actually disable in the next ISR. */
  PWMA->PIER |= (PWM_PIER_PWMIE0_Msk << channel);
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
  PWMA->PIER |= PWM_PIER_PWMIE0_Msk;
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
  /* TODO: Make sure this isn't in use for disabling the periodic channel */
  PWMA->PIER &= ~PWM_PIER_PWMIE0_Msk;
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
                                         pwmchannel_t channel)
{
  (void)pwmp;
  PWMA->PIER |= (PWM_PIER_PWMDIE0_Msk << channel);
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
void pwm_lld_disable_channel_notification(PWMDriver *  pwmp,
                                          pwmchannel_t channel)
{
  (void)pwmp;
  PWMA->PIER &= ~(PWM_PIER_PWMDIE0_Msk << channel);
}

void _pwm_lld_change_period(PWMDriver *pwmp, pwmcnt_t period) {
  (void)pwmp;
  osalSysLock();
  PWMA_CHANNELS[0].CNR = period - 1;
  PWMA_CHANNELS[1].CNR = period - 1;
  PWMA_CHANNELS[2].CNR = period - 1;
  PWMA_CHANNELS[3].CNR = period - 1;
  osalSysUnlock();
}

#endif /* HAL_USE_PWM */

/** @} */
