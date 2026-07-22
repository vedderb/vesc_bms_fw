/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @brief   PLATFORM ADC subsystem low level driver source.
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
#if (RP_ADC_USE_ADC1 == TRUE) || defined(__DOXYGEN__)
ADCDriver ADCD1;
#endif

#if !defined(RP_IRQ_ADC1_PRIORITY)
#error "RP_IRQ_ADC1_PRIORITY not defined in mcuconf.h"
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*
 * @brief   Start ADC only once.
 */
#define RP_ADC_START_ONCE     adcp->adc->SET.CS = ADC_CS_START_ONCE

/*
 * @brief   Set channel to read.
 */
static void set_channel(ADCDriver *adcp, uint8_t channel) {
  adcp->adc->CS = (adcp->adc->CS & ~ADC_CS_AINSEL_Msk) |
             ((channel << ADC_CS_AINSEL_Pos) & ADC_CS_AINSEL_Msk);
}

/*
 * @brief   Get next channel to read.
 */
static uint8_t get_next_channel_number_from_mask(uint8_t mask, uint8_t current) {
  for (uint8_t i = 0; mask > 0; i++) {
    if (mask & 0x01) {
      if (!current) {
        return i;
      }
      current--;
    }
    mask >>= 1U;
  }
  return -1;
}

/*
 * @brief   Get first channel from channnel_mask.
 */
static inline uint8_t get_first_channel(ADCDriver *adcp) {
  return adcp->grpp->channel_mask & 0x01 ? 0 :
         get_next_channel_number_from_mask(adcp->grpp->channel_mask, 0);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if RP_ADC_USE_ADC1 == TRUE || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(RP_ADC_IRQ_FIFO_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  ADCDriver *adcp = &ADCD1;
  adcerror_t emask = 0U;

  if (adcp->adc->INTS & ADC_INTS_FIFO && !(adcp->adc->FCS & ADC_FCS_EMPTY)) {
    uint16_t value = ADC->FIFO;
    if (value & ADC_FIFO_ERR) {
      emask = ADC_ERR_CONVERSION;
    }

    adcp->samples[adcp->current_buffer_position] = value & ADC_FIFO_VAL_Msk;
    adcp->current_buffer_position += 1;

    size_t bufferSize = adcp->depth * adcp->grpp->num_channels;

    adcp->current_channel += 1;
    if (adcp->current_channel >= adcp->grpp->num_channels) {
      adcp->current_channel = 0;
      adcp->current_iteration += 1;
    }

    if (adcp->grpp->circular && adcp->current_channel == 0 &&
        adcp->current_iteration == adcp->depth / 2) {
      _adc_isr_half_code(adcp);
    }
    if (adcp->current_buffer_position == bufferSize) {
      _adc_isr_full_code(adcp);

      if (adcp->grpp->circular) {
        adcp->current_buffer_position = 0;
        adcp->current_channel = 0;
        adcp->current_iteration = 0;
        set_channel(adcp, get_first_channel(adcp));
        RP_ADC_START_ONCE;
      }
    } else {
      set_channel(adcp, get_next_channel_number_from_mask(
                adcp->grpp->channel_mask, adcp->current_channel));
      RP_ADC_START_ONCE;
    }

    if (emask) {
      _adc_isr_error_code(adcp, emask);

      /* Clear error flag. */
      adcp->adc->CLR.FCS = ADC_FCS_ERR;
    }
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

#if RP_ADC_USE_ADC1 == TRUE
  /* Driver initialization.*/
  adcObjectInit(&ADCD1);
  ADCD1.adc = ADC;

  /* Reset ADC */
  hal_lld_peripheral_reset(RESETS_ALLREG_ADC);
  hal_lld_peripheral_unreset(RESETS_ALLREG_ADC);

  /* Enable irq for ADC. */
  nvicEnableVector(RP_ADC_IRQ_FIFO_NUMBER, RP_IRQ_ADC1_PRIORITY);
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
  uint32_t fcs;

  if (adcp->state == ADC_STOP) {
    /* Enables the peripheral.*/
#if RP_ADC_USE_ADC1 == TRUE
    if (&ADCD1 == adcp) {
      adcp->current_buffer_position = 0;
      adcp->current_channel = 0;
      adcp->current_iteration = 0;

      /* Clear control flags. */
      adcp->adc->CS = 0;

      /* Clock settings. */
      adcp->adc->DIV = ((adcp->config->div_int << ADC_DIV_INT_Pos) & ADC_DIV_INT_Msk) |
                       ((adcp->config->div_frac << ADC_DIV_FRAC_Pos) & ADC_DIV_FRAC_Msk);

      /* Enable FIFO. */
      fcs = ADC_FCS_EN;

      /* Set DREQ/IRQ threshold. */
      fcs |= 1U << ADC_FCS_THRESH_Pos;

      /* 8-bits transfer. */
      if (adcp->config->shift) {
        fcs |= ADC_FCS_SHIFT;
      }

      adcp->adc->FCS = fcs;

      /* Set interrupt flag. */
      adcp->adc->SET.INTE = ADC_INTE_FIFO;

      /* Enable ADC. */
      adcp->adc->SET.CS = ADC_CS_EN;
    }
#endif
  }
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
#if RP_ADC_USE_ADC1 == TRUE
    if (&ADCD1 == adcp) {
      /* Clear all flags and disable ADC. */
      adcp->adc->CS = 0;

      /* Clear flags to disable everything. */
      adcp->adc->FCS = 0;

      /* Clear interrupt flag. */
      adcp->adc->CLR.INTE = ADC_INTE_FIFO;
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

  /* Clear error flags. */
  adcp->adc->CLR.CS = ADC_CS_ERR_STICKY;

  /* Set first channel to read. */
  set_channel(adcp, get_first_channel(adcp));

  /* Start conversion */
  RP_ADC_START_ONCE;
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {
  (void)adcp;
}

/*
 * @brief   Enables the TS_EN bit.
 */
void adcRPEnableTS(ADCDriver *adcp) {
  adcp->adc->SET.CS = ADC_CS_TS_EN;
}

/*
 * @brief   Disables the TS_EN bit.
 */
void adcRPDisableTS(ADCDriver *adcp) {
  adcp->adc->CLR.CS = ADC_CS_TS_EN;
}

#endif /* HAL_USE_ADC == TRUE */

/** @} */
