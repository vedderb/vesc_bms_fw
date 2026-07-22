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
/*
   Concepts and parts of this file have been contributed by Fabio Utzig and
   Xo Wang.
 */

/**
 * @file    TIM/hal_icu_lld.c
 * @brief   GD32 ICU subsystem low level driver header.
 *
 * @addtogroup ICU
 * @{
 */

#include "hal.h"

#if HAL_USE_ICU || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   ICUD1 driver identifier.
 * @note    The driver ICUD1 allocates the complex timer TIM0 when enabled.
 */
#if GD32_ICU_USE_TIM0 || defined(__DOXYGEN__)
ICUDriver ICUD1;
#endif

/**
 * @brief   ICUD2 driver identifier.
 * @note    The driver ICUD1 allocates the timer TIM1 when enabled.
 */
#if GD32_ICU_USE_TIM1 || defined(__DOXYGEN__)
ICUDriver ICUD2;
#endif

/**
 * @brief   ICUD3 driver identifier.
 * @note    The driver ICUD1 allocates the timer TIM2 when enabled.
 */
#if GD32_ICU_USE_TIM2 || defined(__DOXYGEN__)
ICUDriver ICUD3;
#endif

/**
 * @brief   ICUD4 driver identifier.
 * @note    The driver ICUD4 allocates the timer TIM3 when enabled.
 */
#if GD32_ICU_USE_TIM3 || defined(__DOXYGEN__)
ICUDriver ICUD4;
#endif

/**
 * @brief   ICUD5 driver identifier.
 * @note    The driver ICUD5 allocates the timer TIM4 when enabled.
 */
#if GD32_ICU_USE_TIM4 || defined(__DOXYGEN__)
ICUDriver ICUD5;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static bool icu_lld_wait_edge(ICUDriver *icup) {
  uint32_t sr;
  bool result;

  /* Polled mode so re-enabling the interrupts while the operation is
     performed.*/
  osalSysUnlock();

  /* Polling the right bit depending on the input channel.*/
  if (icup->config->channel == ICU_CHANNEL_1) {
    /* Waiting for an edge.*/
    while (((sr = icup->tim->INTF) &
            (GD32_TIM_SR_CC1IF | GD32_TIM_SR_UIF)) == 0)
      ;
  }
  else {
    /* Waiting for an edge.*/
    while (((sr = icup->tim->INTF) &
            (GD32_TIM_SR_CC2IF | GD32_TIM_SR_UIF)) == 0)
      ;
  }

  /* Edge or overflow?*/
  result = (sr & GD32_TIM_SR_UIF) != 0 ? true : false;

  /* Done, disabling interrupts again.*/
  osalSysLock();

  /* Resetting all flags.*/
  icup->tim->INTF &= ~(GD32_TIM_SR_CC1IF |
                     GD32_TIM_SR_CC2IF |
                     GD32_TIM_SR_UIF);

  return result;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if GD32_ICU_USE_TIM0 || defined(__DOXYGEN__)
#if !defined(GD32_TIM0_SUPPRESS_ISR)
#if !defined(GD32_TIM0_UP_HANDLER)
#error "GD32_TIM0_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM0 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM0_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD1);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(GD32_TIM0_CC_HANDLER)
#error "GD32_TIM0_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM0 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_TIM0_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  icu_lld_serve_interrupt(&ICUD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM0_SUPPRESS_ISR) */
#endif /* GD32_ICU_USE_TIM0 */

#if GD32_ICU_USE_TIM1 || defined(__DOXYGEN__)
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

  icu_lld_serve_interrupt(&ICUD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM1_SUPPRESS_ISR) */
#endif /* GD32_ICU_USE_TIM1 */

#if GD32_ICU_USE_TIM2 || defined(__DOXYGEN__)
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

  icu_lld_serve_interrupt(&ICUD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM2_SUPPRESS_ISR) */
#endif /* GD32_ICU_USE_TIM2 */

#if GD32_ICU_USE_TIM3 || defined(__DOXYGEN__)
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

  icu_lld_serve_interrupt(&ICUD4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM3_SUPPRESS_ISR) */
#endif /* GD32_ICU_USE_TIM3 */

#if GD32_ICU_USE_TIM4 || defined(__DOXYGEN__)
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

  icu_lld_serve_interrupt(&ICUD5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(GD32_TIM4_SUPPRESS_ISR) */
#endif /* GD32_ICU_USE_TIM4 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ICU driver initialization.
 *
 * @notapi
 */
void icu_lld_init(void) {

#if GD32_ICU_USE_TIM0
  /* Driver initialization.*/
  icuObjectInit(&ICUD1);
  ICUD1.tim = GD32_TIM0;
#endif

#if GD32_ICU_USE_TIM1
  /* Driver initialization.*/
  icuObjectInit(&ICUD2);
  ICUD2.tim = GD32_TIM1;
#endif

#if GD32_ICU_USE_TIM2
  /* Driver initialization.*/
  icuObjectInit(&ICUD3);
  ICUD3.tim = GD32_TIM2;
#endif

#if GD32_ICU_USE_TIM3
  /* Driver initialization.*/
  icuObjectInit(&ICUD4);
  ICUD4.tim = GD32_TIM3;
#endif

#if GD32_ICU_USE_TIM4
  /* Driver initialization.*/
  icuObjectInit(&ICUD5);
  ICUD5.tim = GD32_TIM4;
#endif
}

/**
 * @brief   Configures and activates the ICU peripheral.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_start(ICUDriver *icup) {
  uint32_t psc;

  osalDbgAssert((icup->config->channel == ICU_CHANNEL_1) ||
                (icup->config->channel == ICU_CHANNEL_2),
                "invalid input");

  if (icup->state == ICU_STOP) {
    /* Clock activation and timer reset.*/
#if GD32_ICU_USE_TIM0
    if (&ICUD1 == icup) {
      rcuEnableTIM0(true);
      rcuResetTIM0();
#if !defined(GD32_TIM0_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM0_UP_NUMBER, GD32_ICU_TIM0_IRQ_PRIORITY, GD32_ICU_TIM0_IRQ_TRIGGER);
      eclicEnableVector(GD32_TIM0_CC_NUMBER, GD32_ICU_TIM0_IRQ_PRIORITY, GD32_ICU_TIM0_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM0CLK)
      icup->clock = GD32_TIM0CLK;
#else
      icup->clock = GD32_TIMCLK2;
#endif
    }
#endif

#if GD32_ICU_USE_TIM1
    if (&ICUD2 == icup) {
      rcuEnableTIM1(true);
      rcuResetTIM1();
#if !defined(GD32_TIM1_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM1_NUMBER, GD32_ICU_TIM1_IRQ_PRIORITY, GD32_ICU_TIM1_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM1CLK)
      icup->clock = GD32_TIM1CLK;
#else
      icup->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_ICU_USE_TIM2
    if (&ICUD3 == icup) {
      rcuEnableTIM2(true);
      rcuResetTIM2();
#if !defined(GD32_TIM2_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM2_NUMBER, GD32_ICU_TIM2_IRQ_PRIORITY, GD32_ICU_TIM2_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM2CLK)
      icup->clock = GD32_TIM2CLK;
#else
     icup->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_ICU_USE_TIM3
    if (&ICUD4 == icup) {
      rcuEnableTIM3(true);
      rcuResetTIM3();
#if !defined(GD32_TIM3_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM3_NUMBER, GD32_ICU_TIM3_IRQ_PRIORITY, GD32_ICU_TIM3_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM3CLK)
      icup->clock = GD32_TIM3CLK;
#else
      icup->clock = GD32_TIMCLK1;
#endif
    }
#endif

#if GD32_ICU_USE_TIM4
    if (&ICUD5 == icup) {
      rcuEnableTIM4(true);
      rcuResetTIM4();
#if !defined(GD32_TIM4_SUPPRESS_ISR)
      eclicEnableVector(GD32_TIM4_NUMBER, GD32_ICU_TIM4_IRQ_PRIORITY, GD32_ICU_TIM4_IRQ_TRIGGER);
#endif
#if defined(GD32_TIM4CLK)
      icup->clock = GD32_TIM4CLK;
#else
      icup->clock = GD32_TIMCLK1;
#endif
    }
#endif

  }
  else {
    /* Driver re-configuration scenario, it must be stopped first.*/
    icup->tim->CTL0    = 0;                  /* Timer disabled.              */
    icup->tim->CHCV[0] = 0;                  /* Comparator 1 disabled.       */
    icup->tim->CHCV[1] = 0;                  /* Comparator 2 disabled.       */
    icup->tim->CNT    = 0;                  /* Counter reset to zero.       */
  }

  /* Timer configuration.*/
  icup->tim->INTF   = 0;                      /* Clear eventual pending IRQs. */
  icup->tim->DMAINTEN = icup->config->dmainten &    /* DMA-related DIER settings.   */
                    ~GD32_TIM_DIER_IRQ_MASK;
  psc = (icup->clock / icup->config->frequency) - 1;
  osalDbgAssert((psc <= 0xFFFF) &&
                ((psc + 1) * icup->config->frequency) == icup->clock,
                "invalid frequency");
  icup->tim->PSC = psc;
  if (icup->config->car == 0U) {
    /* Zero is an invalid value and is turned in maximum value, also for
       legacy configurations compatibility.*/
    icup->tim->CAR = 0xFFFFFFFFU;
  }
  else {
    icup->tim->CAR = icup->config->car;
  }

  if (icup->config->channel == ICU_CHANNEL_1) {
    /* Selected input 1.
       CCMR1_CC1S = 01 = CH1 Input on TI1.
       CCMR1_CC2S = 10 = CH2 Input on TI1.*/
    icup->tim->CHCTL0 = GD32_TIM_CCMR1_CC1S(1) | GD32_TIM_CCMR1_CC2S(2);

    /* SMCR_TS  = 101, input is TI1FP1.
       SMCR_SMS = 100, reset on rising edge.*/
    icup->tim->SMCFG  = GD32_TIM_SMCR_TS(5) | GD32_TIM_SMCR_SMS(4);

    /* The CCER settings depend on the selected trigger mode.
       ICU_INPUT_ACTIVE_HIGH: Active on rising edge, idle on falling edge.
       ICU_INPUT_ACTIVE_LOW:  Active on falling edge, idle on rising edge.*/
    if (icup->config->mode == ICU_INPUT_ACTIVE_HIGH)
      icup->tim->CHCTL2 = GD32_TIM_CCER_CC1E |
                        GD32_TIM_CCER_CC2E | GD32_TIM_CCER_CC2P;
    else
      icup->tim->CHCTL2 = GD32_TIM_CCER_CC1E | GD32_TIM_CCER_CC1P |
                        GD32_TIM_CCER_CC2E;

    /* Direct pointers to the capture registers in order to make reading
       data faster from within callbacks.*/
    icup->wccrp = &icup->tim->CHCV[1];
    icup->pccrp = &icup->tim->CHCV[0];
  }
  else {
    /* Selected input 2.
       CCMR1_CC1S = 10 = CH1 Input on TI2.
       CCMR1_CC2S = 01 = CH2 Input on TI2.*/
    icup->tim->CHCTL0 = GD32_TIM_CCMR1_CC1S(2) | GD32_TIM_CCMR1_CC2S(1);

    /* SMCR_TS  = 110, input is TI2FP2.
       SMCR_SMS = 100, reset on rising edge.*/
    icup->tim->SMCFG  = GD32_TIM_SMCR_TS(6) | GD32_TIM_SMCR_SMS(4);

    /* The CCER settings depend on the selected trigger mode.
       ICU_INPUT_ACTIVE_HIGH: Active on rising edge, idle on falling edge.
       ICU_INPUT_ACTIVE_LOW:  Active on falling edge, idle on rising edge.*/
    if (icup->config->mode == ICU_INPUT_ACTIVE_HIGH)
      icup->tim->CHCTL2 = GD32_TIM_CCER_CC1E | GD32_TIM_CCER_CC1P |
                        GD32_TIM_CCER_CC2E;
    else
      icup->tim->CHCTL2 = GD32_TIM_CCER_CC1E |
                        GD32_TIM_CCER_CC2E | GD32_TIM_CCER_CC2P;

    /* Direct pointers to the capture registers in order to make reading
       data faster from within callbacks.*/
    icup->wccrp = &icup->tim->CHCV[0];
    icup->pccrp = &icup->tim->CHCV[1];
  }
}

/**
 * @brief   Deactivates the ICU peripheral.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_stop(ICUDriver *icup) {

  if (icup->state == ICU_READY) {
    /* Clock deactivation.*/
    icup->tim->CTL0  = 0;                    /* Timer disabled.              */
    icup->tim->DMAINTEN = 0;                    /* All IRQs disabled.           */
    icup->tim->INTF   = 0;                    /* Clear eventual pending IRQs. */

#if GD32_ICU_USE_TIM0
    if (&ICUD1 == icup) {
#if !defined(GD32_TIM0_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM0_UP_NUMBER);
      eclicDisableVector(GD32_TIM0_CC_NUMBER);
#endif
      rcuDisableTIM0();
    }
#endif

#if GD32_ICU_USE_TIM1
    if (&ICUD2 == icup) {
#if !defined(GD32_TIM1_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM1_NUMBER);
#endif
      rcuDisableTIM1();
    }
#endif

#if GD32_ICU_USE_TIM2
    if (&ICUD3 == icup) {
#if !defined(GD32_TIM2_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM2_NUMBER);
#endif
      rcuDisableTIM2();
    }
#endif

#if GD32_ICU_USE_TIM3
    if (&ICUD4 == icup) {
#if !defined(GD32_TIM3_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM3_NUMBER);
#endif
      rcuDisableTIM3();
    }
#endif

#if GD32_ICU_USE_TIM4
    if (&ICUD5 == icup) {
#if !defined(GD32_TIM4_SUPPRESS_ISR)
      eclicDisableVector(GD32_TIM4_NUMBER);
#endif
      rcuDisableTIM4();
    }
#endif
  }
}

/**
 * @brief   Starts the input capture.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_start_capture(ICUDriver *icup) {

  /* Triggering an UG and clearing the IRQ status.*/
  icup->tim->SWEVG |= GD32_TIM_EGR_UG;
  icup->tim->INTF = 0;

  /* Timer is started.*/
  icup->tim->CTL0 = GD32_TIM_CR1_URS | GD32_TIM_CR1_CEN;
}

/**
 * @brief   Waits for a completed capture.
 * @note    The operation is performed in polled mode.
 * @note    In order to use this function notifications must be disabled.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The capture status.
 * @retval false        if the capture is successful.
 * @retval true         if a timer overflow occurred.
 *
 * @notapi
 */
bool icu_lld_wait_capture(ICUDriver *icup) {

  /* If the driver is still in the ICU_WAITING state then we need to wait
     for the first activation edge.*/
  if (icup->state == ICU_WAITING)
    if (icu_lld_wait_edge(icup))
      return true;

  /* This edge marks the availability of a capture result.*/
  return icu_lld_wait_edge(icup);
}

/**
 * @brief   Stops the input capture.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_stop_capture(ICUDriver *icup) {

  /* Timer stopped.*/
  icup->tim->CTL0   = 0;

  /* All interrupts disabled.*/
  icup->tim->DMAINTEN &= ~GD32_TIM_DIER_IRQ_MASK;
}

/**
 * @brief   Enables notifications.
 * @pre     The ICU unit must have been activated using @p icuStart() and the
 *          capture started using @p icuStartCapture().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_enable_notifications(ICUDriver *icup) {
  uint32_t dier = icup->tim->DMAINTEN;

  /* If interrupts were already enabled then the operation is skipped.
     This is done in order to avoid clearing the SR and risk losing
     pending interrupts.*/
  if ((dier & GD32_TIM_DIER_IRQ_MASK) == 0) {
    /* Previously triggered IRQs are ignored, status cleared.*/
    icup->tim->INTF = 0;

    if (icup->config->channel == ICU_CHANNEL_1) {
      /* Enabling periodic callback on CC1.*/
      dier |= GD32_TIM_DIER_CC1IE;

      /* Optionally enabling width callback on CC2.*/
      if (icup->config->width_cb != NULL)
        dier |= GD32_TIM_DIER_CC2IE;
    }
    else {
      /* Enabling periodic callback on CC2.*/
      dier |= GD32_TIM_DIER_CC2IE;

      /* Optionally enabling width callback on CC1.*/
      if (icup->config->width_cb != NULL)
        dier |= GD32_TIM_DIER_CC1IE;
    }

    /* If an overflow callback is defined then also the overflow callback
       is enabled.*/
    if (icup->config->overflow_cb != NULL)
      dier |= GD32_TIM_DIER_UIE;

    /* One single atomic write.*/
    icup->tim->DMAINTEN = dier;
  }
}

/**
 * @brief   Disables notifications.
 * @pre     The ICU unit must have been activated using @p icuStart() and the
 *          capture started using @p icuStartCapture().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_disable_notifications(ICUDriver *icup) {

  /* All interrupts disabled.*/
  icup->tim->DMAINTEN &= ~GD32_TIM_DIER_IRQ_MASK;
}

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 *
 * @notapi
 */
void icu_lld_serve_interrupt(ICUDriver *icup) {
  uint32_t sr;

  sr  = icup->tim->INTF;
  sr &= icup->tim->DMAINTEN & GD32_TIM_DIER_IRQ_MASK;
  icup->tim->INTF = ~sr;
  if (icup->config->channel == ICU_CHANNEL_1) {
    if ((sr & GD32_TIM_SR_CC2IF) != 0)
      _icu_isr_invoke_width_cb(icup);
    if ((sr & GD32_TIM_SR_CC1IF) != 0)
      _icu_isr_invoke_period_cb(icup);
  }
  else {
    if ((sr & GD32_TIM_SR_CC1IF) != 0)
      _icu_isr_invoke_width_cb(icup);
    if ((sr & GD32_TIM_SR_CC2IF) != 0)
      _icu_isr_invoke_period_cb(icup);
  }
  if ((sr & GD32_TIM_SR_UIF) != 0)
    _icu_isr_invoke_overflow_cb(icup);
}

#endif /* HAL_USE_ICU */

/** @} */
