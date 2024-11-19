/*
    Copyright (C) 2020 Konstantin Oblaukhov

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
 * @file    hal_adc_lld.c
 * @brief   NRF5 ADC subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "hal.h"

#if (HAL_USE_ADC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   ADC1 driver identifier.
 */
#if (NRF5_ADC_USE_ADC1 == TRUE) || defined(__DOXYGEN__)
ADCDriver ADCD1;
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

#if NRF5_ADC_USE_ADC1 || defined(__DOXYGEN__)
/**
 * @brief   ADC interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector5C) {

  ADCDriver *adcp = &ADCD1;
  NRF_SAADC_Type *adc = adcp->adc;
  const ADCConversionGroup *grpp = adcp->grpp;

  OSAL_IRQ_PROLOGUE();

  if (adc->EVENTS_RESULTDONE) {
    adc->EVENTS_RESULTDONE = 0;
    adcp->ch_counter++;

    if (adcp->ch_counter == grpp->num_channels) {
      adcp->counter++;
      adcp->ch_counter = 0;

      if (grpp->circular &
          (adcp->counter == adcp->depth / 2))
        _adc_isr_half_code(adcp)

      if ((adcp->counter < adcp->depth) && !grpp->external) {
        adc->TASKS_SAMPLE = 1;
      }
    }
  }

  if (adc->EVENTS_END) {
    adc->EVENTS_END = 0;
    _adc_isr_full_code(adcp);

    if (grpp->circular) {
      adcp->counter = 0;
      adcp->ch_counter = 0;
      adc->TASKS_START = 1;
      if (!grpp->external)
          adc->TASKS_SAMPLE = 1;
    } else
      adc_lld_stop_conversion(adcp);
  }

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {

#if NRF5_ADC_USE_ADC1 == TRUE
  /* Driver initialization.*/
  adcObjectInit(&ADCD1);
  ADCD1.adc = NRF_SAADC;
#endif
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start(ADCDriver *adcp) {

  if (adcp->state == ADC_STOP) {
    /* Enables the peripheral.*/
#if NRF5_ADC_USE_ADC1 == TRUE
    if (&ADCD1 == adcp) {
      adcp->adc->INTEN = SAADC_INTEN_END_Msk | SAADC_INTEN_RESULTDONE_Msk;
      nvicEnableVector(SAADC_IRQn, NRF5_ADC_IRQ_PRIORITY);
    }
#endif
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Deactivates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop(ADCDriver *adcp) {

  if (adcp->state == ADC_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if NRF5_ADC_USE_ADC1 == TRUE
    if (&ADCD1 == adcp) {
      nvicDisableVector(SAADC_IRQn);
      adcp->adc->INTEN = 0;
    }
#endif
  }
}

/**
 * @brief   Starts an ADC conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start_conversion(ADCDriver *adcp) {
  NRF_SAADC_Type *adc = adcp->adc;
  const ADCConversionGroup *grpp = adcp->grpp;

  adc->RESOLUTION = grpp->resolution;
  adc->OVERSAMPLE = grpp->oversample;
  adc->SAMPLERATE = grpp->samplerate;

  for (int i = 0; i < grpp->num_channels; i++) {
    adc->CH[i].PSELP = grpp->channels[i].pselp;
    adc->CH[i].PSELN = grpp->channels[i].pseln;
    adc->CH[i].CONFIG = grpp->channels[i].config;
  }
  for (int i = grpp->num_channels; i < NRF5_ADC_MAX_CHANNELS; i++) {
    adc->CH[i].PSELP = 0;
    adc->CH[i].PSELN = 0;
  }

  adc->RESULT.PTR = (uint32_t)adcp->samples;
  adc->RESULT.MAXCNT = adcp->depth * grpp->num_channels;

  adcp->counter = 0;
  adcp->ch_counter = 0;

  adc->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;
  adc->TASKS_START = 1;

  if (!grpp->external)
      adc->TASKS_SAMPLE = 1;
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {
  NRF_SAADC_Type *adc = adcp->adc;

  adc->TASKS_STOP = 1;
  adc->ENABLE = SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos;
}

#endif /* HAL_USE_ADC == TRUE */

/** @} */
