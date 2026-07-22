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
 * @file    GD32VF103/hal_adc_lld.c
 * @brief   GD32VF103 ADC subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "hal.h"

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief ADC0 driver identifier.*/
#if GD32_ADC_USE_ADC0 || defined(__DOXYGEN__)
ADCDriver ADCD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared ADC DMA ISR service routine.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void adc_lld_serve_rx_interrupt(ADCDriver *adcp, uint32_t flags) {

  /* DMA errors handling.*/
  if ((flags & GD32_DMA_INTF_ERRIF) != 0) {
    /* DMA, this could help only if the DMA tries to access an unmapped
       address space or violates alignment rules.*/
    _adc_isr_error_code(adcp, ADC_ERR_DMAFAILURE);
  }
  else {
    if ((flags & GD32_DMA_INTF_FTFIF) != 0) {
      /* Transfer complete processing.*/
      _adc_isr_full_code(adcp);
    }
    else if ((flags & GD32_DMA_INTF_HTFIF) != 0) {
      /* Half transfer processing.*/
      _adc_isr_half_code(adcp);
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {

#if GD32_ADC_USE_ADC0
  /* Driver initialization.*/
  adcObjectInit(&ADCD1);
  ADCD1.adc = ADC0;
  ADCD1.dmastp  = NULL;
  ADCD1.dmamode = GD32_DMA_CTL_PRIO(GD32_ADC_ADC0_DMA_PRIORITY) |
                  GD32_DMA_CTL_MWIDTH_HWORD | GD32_DMA_CTL_PWIDTH_HWORD |
                  GD32_DMA_CTL_MNAGA        | GD32_DMA_CTL_FTFIE        |
                  GD32_DMA_CTL_ERRIE;

  /* Temporary activation.*/
  rcuEnableADC0(true);
  ADC0->CTL0 = 0;
  ADC0->CTL1 = ADC_CTL1_ADCON;

  /* Reset calibration just to be safe.*/
  ADC0->CTL1 = ADC_CTL1_ADCON | ADC_CTL1_RSTCLB;
  while ((ADC0->CTL1 & ADC_CTL1_RSTCLB) != 0)
    ;

  /* Calibration.*/
  ADC0->CTL1 = ADC_CTL1_ADCON | ADC_CTL1_CLB;
  while ((ADC0->CTL1 & ADC_CTL1_CLB) != 0)
    ;

  /* Return the ADC in low power mode.*/
  ADC0->CTL1 = 0;
  rcuDisableADC0();
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

  /* If in stopped state then enables the ADC and DMA clocks.*/
  if (adcp->state == ADC_STOP) {
#if GD32_ADC_USE_ADC0
    if (&ADCD1 == adcp) {
      adcp->dmastp = dmaStreamAllocI(GD32_DMA_STREAM_ID(0, 0),
                                     GD32_ADC_ADC0_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)adc_lld_serve_rx_interrupt,
                                     (void *)adcp);
      osalDbgAssert(adcp->dmastp != NULL, "unable to allocate stream");
      dmaStreamSetPeripheral(adcp->dmastp, &ADC0->RDATA);
      rcuEnableADC0(true);
    }
#endif

    /* ADC setup, the calibration procedure has already been performed
       during initialization.*/
    adcp->adc->CTL0 = 0;
    adcp->adc->CTL1 = 0;
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

  /* If in ready state then disables the ADC clock.*/
  if (adcp->state == ADC_READY) {
#if GD32_ADC_USE_ADC0
    if (&ADCD1 == adcp) {
      ADC0->CTL0 = 0;
      ADC0->CTL1 = 0;

      dmaStreamFreeI(adcp->dmastp);
      adcp->dmastp = NULL;

      rcuDisableADC0();
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
  uint32_t mode, ctl1;
  const ADCConversionGroup *grpp = adcp->grpp;

  /* DMA setup.*/
  mode = adcp->dmamode;
  if (grpp->circular) {
    mode |= GD32_DMA_CTL_CMEN;
    if (adcp->depth > 1) {
      /* If circular buffer depth > 1, then the half transfer interrupt
         is enabled in order to allow streaming processing.*/
      mode |= GD32_DMA_CTL_HTFIE;
    }
  }
  dmaStreamSetMemory0(adcp->dmastp, adcp->samples);
  dmaStreamSetTransactionSize(adcp->dmastp, (uint32_t)grpp->num_channels *
                                            (uint32_t)adcp->depth);
  dmaStreamSetMode(adcp->dmastp, mode);
  dmaStreamEnable(adcp->dmastp);

  /* ADC setup.*/
  adcp->adc->CTL0   = grpp->ctl0 | ADC_CTL0_SM;
  ctl1 = grpp->ctl1 | ADC_CTL1_DMA | ADC_CTL1_ADCON;
  if ((ctl1 & (ADC_CTL1_ETERC | ADC_CTL1_ETEIC)) == 0)
    ctl1 |= ADC_CTL1_CTN;
  adcp->adc->CTL1   = grpp->ctl1 | ctl1;
  adcp->adc->SAMPT0 = grpp->sampt0;
  adcp->adc->SAMPT1 = grpp->sampt1;
  adcp->adc->RSQ0  = grpp->rsq0;
  adcp->adc->RSQ1  = grpp->rsq1;
  adcp->adc->RSQ2  = grpp->rsq2;

  /* ADC start by writing ADC_CTL1_ADCON a second time.*/
  adcp->adc->CTL1   = ctl1;
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {

  dmaStreamDisable(adcp->dmastp);
  adcp->adc->CTL1 = 0;
}

#endif /* HAL_USE_ADC */

/** @} */
