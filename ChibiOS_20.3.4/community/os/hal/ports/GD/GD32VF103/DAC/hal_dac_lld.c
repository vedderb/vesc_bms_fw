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
 * @file    DAC/hal_dac_lld.c
 * @brief   GD32 DAC subsystem low level driver source.
 *
 * @addtogroup DAC
 * @{
 */

#include "hal.h"

#if HAL_USE_DAC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define DAC_CH1_DMA_CHANNEL                                                \
  GD32_DMA_GETCHANNEL(GD32_DAC_CH1_DMA_STREAM,                       \
                       GD32_DAC_CH1_DMA_CHN)

#define DAC_CH2_DMA_CHANNEL                                                \
  GD32_DMA_GETCHANNEL(GD32_DAC_CH2_DMA_STREAM,                       \
                       GD32_DAC_CH2_DMA_CHN)

#define CHANNEL_DATA_OFFSET 3U

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief DAC CH1 driver identifier.*/
#if GD32_DAC_USE_DAC_CH1 || defined(__DOXYGEN__)
DACDriver DACD1;
#endif

/** @brief DAC CH2 driver identifier.*/
#if (GD32_DAC_USE_DAC_CH2 && !GD32_DAC_DUAL_MODE) || defined(__DOXYGEN__)
DACDriver DACD2;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

#if GD32_DAC_USE_DAC_CH1 == TRUE
static const dacparams_t dac1_ch1_params = {
  .dac          = DAC,
  .dataoffset   = 0U,
  .regshift     = 0U,
  .regmask      = 0xFFFF0000U,
  .dmastream    = GD32_DAC_CH1_DMA_STREAM,
  .dmamode      = GD32_DMA_CTL_CHSEL(DAC_CH1_DMA_CHANNEL) |
                  GD32_DMA_CTL_PRIO(GD32_DAC_CH1_DMA_PRIORITY) |
                  GD32_DMA_CTL_MNAGA | GD32_DMA_CTL_CMEN | GD32_DMA_CTL_DIR_M2P |
                  GD32_DMA_CTL_ERRIE | GD32_DMA_CTL_HTFIE |
                  GD32_DMA_CTL_FTFIE,
  .dmairqprio   = GD32_DAC_CH1_IRQ_PRIORITY
};
#endif

#if GD32_DAC_USE_DAC_CH2 == TRUE
static const dacparams_t dac1_ch2_params = {
  .dac          = DAC,
  .dataoffset   = CHANNEL_DATA_OFFSET,
  .regshift     = 16U,
  .regmask      = 0x0000FFFFU,
  .dmastream    = GD32_DAC_CH2_DMA_STREAM,
  .dmamode      = GD32_DMA_CTL_CHSEL(DAC_CH2_DMA_CHANNEL) |
                  GD32_DMA_CTL_PRIO(GD32_DAC_CH2_DMA_PRIORITY) |
                  GD32_DMA_CTL_MNAGA | GD32_DMA_CTL_CMEN | GD32_DMA_CTL_DIR_M2P |
                  GD32_DMA_CTL_ERRIE | GD32_DMA_CTL_HTFIE |
                  GD32_DMA_CTL_FTFIE,
  .dmairqprio   = GD32_DAC_CH2_IRQ_PRIORITY
};
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared end/half-of-tx service routine.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void dac_lld_serve_tx_interrupt(DACDriver *dacp, uint32_t flags) {

  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    /* DMA errors handling.*/
    dac_lld_stop_conversion(dacp);
    _dac_isr_error_code(dacp, DAC_ERR_DMAFAILURE);
  }
  else {
    if ((flags & GD32_DMA_INTF_HTFIF) != 0) {
      /* Half transfer processing.*/
      _dac_isr_half_code(dacp);
    }
    if ((flags & GD32_DMA_INTF_FTFIF) != 0) {
      /* Transfer complete processing.*/
      _dac_isr_full_code(dacp);
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
 * @brief   Low level DAC driver initialization.
 *
 * @notapi
 */
void dac_lld_init(void) {

#if GD32_DAC_USE_DAC_CH1
  dacObjectInit(&DACD1);
  DACD1.params  = &dac1_ch1_params;
  DACD1.dma = NULL;
#endif

#if GD32_DAC_USE_DAC_CH2
  dacObjectInit(&DACD2);
  DACD2.params  = &dac1_ch2_params;
  DACD2.dma = NULL;
#endif
}

/**
 * @brief   Configures and activates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_start(DACDriver *dacp) {

  /* If the driver is in DAC_STOP state then a full initialization is
     required.*/
  if (dacp->state == DAC_STOP) {
    dacchannel_t channel = 0;

    /* Enabling the clock source.*/
#if GD32_DAC_USE_DAC_CH1
    if (&DACD1 == dacp) {
      rcuEnableDAC(true);
    }
#endif

#if GD32_DAC_USE_DAC_CH2
    if (&DACD2 == dacp) {
      rcuEnableDAC(true);
      channel = 1;
    }
#endif

    /* Enabling DAC in SW triggering mode initially, initializing data to
       zero.*/
#if GD32_DAC_DUAL_MODE == FALSE
    {
      uint32_t ctl;

      ctl = dacp->params->dac->CTL;
      ctl &= dacp->params->regmask;
      ctl |= (DAC_CTL_DEN0 | dacp->config->ctl) << dacp->params->regshift;
      dacp->params->dac->CTL = ctl;
      dac_lld_put_channel(dacp, channel, dacp->config->init);
    }
#else
    if ((dacp->config->datamode == DAC_DHRM_12BIT_RIGHT_DUAL) ||
        (dacp->config->datamode == DAC_DHRM_12BIT_LEFT_DUAL) ||
        (dacp->config->datamode == DAC_DHRM_8BIT_RIGHT_DUAL)) {
      dacp->params->dac->CTL = DAC_CTL_DEN1 | (dacp->config->ctl << 16) | DAC_CTL_DEN0 | dacp->config->ctl;
      dac_lld_put_channel(dacp, 1U, dacp->config->init);
    }
    else {
      dacp->params->dac->CTL = DAC_CTL_DEN0 | dacp->config->ctl;
    }
    dac_lld_put_channel(dacp, channel, dacp->config->init);
#endif
  }
}

/**
 * @brief   Deactivates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_stop(DACDriver *dacp) {

  /* If in ready state then disables the DAC clock.*/
  if (dacp->state == DAC_READY) {

    /* Disabling DAC.*/
    dacp->params->dac->CTL &= dacp->params->regmask;

#if GD32_DAC_USE_DAC_CH1
    if (&DACD1 == dacp) {
      if ((dacp->params->dac->CTL & DAC_CTL_DEN1) == 0U) {
        rcuDisableDAC();
      }
    }
#endif

#if GD32_DAC_USE_DAC_CH2
    if (&DACD2 == dacp) {
      if ((dacp->params->dac->CTL & DAC_CTL_DEN0) == 0U) {
        rcuDisableDAC();
      }
    }
#endif
  }
}

/**
 * @brief   Outputs a value directly on a DAC channel.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] channel   DAC channel number
 * @param[in] sample    value to be output
 *
 * @api
 */
void dac_lld_put_channel(DACDriver *dacp,
                         dacchannel_t channel,
                         dacsample_t sample) {

  switch (dacp->config->datamode) {
  case DAC_DHRM_12BIT_RIGHT:
#if GD32_DAC_DUAL_MODE
  case DAC_DHRM_12BIT_RIGHT_DUAL:
#endif
    if (channel == 0U) {
#if GD32_DAC_DUAL_MODE
      dacp->params->dac->R12DH0 = (uint32_t)sample;
#else
      *(&dacp->params->dac->R12DH0 + dacp->params->dataoffset) = (uint32_t)sample;
#endif
    }
#if (GD32_HAS_DAC_CH2)
    else {
      dacp->params->dac->R12DH1 = (uint32_t)sample;
    }
#endif
    break;
  case DAC_DHRM_12BIT_LEFT:
#if GD32_DAC_DUAL_MODE
  case DAC_DHRM_12BIT_LEFT_DUAL:
#endif
    if (channel == 0U) {
#if GD32_DAC_DUAL_MODE
      dacp->params->dac->L12DH0 = (uint32_t)sample;
#else
      *(&dacp->params->dac->L12DH0 + dacp->params->dataoffset) = (uint32_t)sample;
#endif
    }
#if (GD32_HAS_DAC_CH2)
    else {
      dacp->params->dac->L12DH1 = (uint32_t)sample;
    }
#endif
    break;
  case DAC_DHRM_8BIT_RIGHT:
#if GD32_DAC_DUAL_MODE
  case DAC_DHRM_8BIT_RIGHT_DUAL:
#endif
    if (channel == 0U) {
#if GD32_DAC_DUAL_MODE
      dacp->params->dac->R8DH0 = (uint32_t)sample;
#else
      *(&dacp->params->dac->R8DH0 + dacp->params->dataoffset) = (uint32_t)sample;
#endif
    }
#if (GD32_HAS_DAC_CH2)
    else {
      dacp->params->dac->R8DH1 = (uint32_t)sample;
    }
#endif
    break;
  default:
    osalDbgAssert(false, "unexpected DAC mode");
    break;
  }
}

/**
 * @brief   Starts a DAC conversion.
 * @details Starts an asynchronous conversion operation.
 * @note    In @p DAC_DHRM_8BIT_RIGHT mode the parameters passed to the
 *          callback are wrong because two samples are packed in a single
 *          dacsample_t element. This will not be corrected, do not rely
 *          on those parameters.
 * @note    In @p DAC_DHRM_8BIT_RIGHT_DUAL mode two samples are treated
 *          as a single 16 bits sample and packed into a single dacsample_t
 *          element. The num_channels must be set to one in the group
 *          conversion configuration structure.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_start_conversion(DACDriver *dacp) {
  uint32_t n, ctl, dmamode;

  /* Number of DMA operations per buffer.*/
  n = dacp->depth * dacp->grpp->num_channels;

  /* Allocating the DMA channel.*/
  dacp->dma = dmaStreamAllocI(dacp->params->dmastream,
                              dacp->params->dmairqprio,
                              (gd32_dmaisr_t)dac_lld_serve_tx_interrupt,
                              (void *)dacp);
  osalDbgAssert(dacp->dma != NULL, "unable to allocate stream");

  /* DMA settings depend on the chosen DAC mode.*/
  switch (dacp->config->datamode) {
  /* Sets the DAC data register */
  case DAC_DHRM_12BIT_RIGHT:
    osalDbgAssert(dacp->grpp->num_channels == 1, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->R12DH0 +
                                      dacp->params->dataoffset);
    dmamode = dacp->params->dmamode |
              GD32_DMA_CTL_PWIDTH_HWORD | GD32_DMA_CTL_MWIDTH_HWORD;
    break;
  case DAC_DHRM_12BIT_LEFT:
    osalDbgAssert(dacp->grpp->num_channels == 1, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->L12DH0 +
                                      dacp->params->dataoffset);
    dmamode = dacp->params->dmamode |
              GD32_DMA_CTL_PWIDTH_HWORD | GD32_DMA_CTL_MWIDTH_HWORD;
    break;
  case DAC_DHRM_8BIT_RIGHT:
    osalDbgAssert(dacp->grpp->num_channels == 1, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->R8DH0 +
                                      dacp->params->dataoffset);
    dmamode = dacp->params->dmamode |
              GD32_DMA_CTL_PWIDTH_BYTE | GD32_DMA_CTL_MWIDTH_BYTE;

    /* In this mode the size of the buffer is halved because two samples
       packed in a single dacsample_t element.*/
    n = (n + 1) / 2;
    break;
#if GD32_DAC_DUAL_MODE == TRUE
  case DAC_DHRM_12BIT_RIGHT_DUAL:
    osalDbgAssert(dacp->grpp->num_channels == 2, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->R12DH);
    dmamode = dacp->params->dmamode |
              GD32_DMA_CTL_PWIDTH_WORD | GD32_DMA_CTL_MWIDTH_WORD;
    n /= 2;
    break;
  case DAC_DHRM_12BIT_LEFT_DUAL:
    osalDbgAssert(dacp->grpp->num_channels == 2, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->L12DH);
    dmamode = dacp->params->dmamode |
              GD32_DMA_CTL_PWIDTH_WORD | GD32_DMA_CTL_MWIDTH_WORD;
    n /= 2;
    break;
  case DAC_DHRM_8BIT_RIGHT_DUAL:
    osalDbgAssert(dacp->grpp->num_channels == 1, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->R8DH);
    dmamode = dacp->params->dmamode |
              GD32_DMA_CTL_PWIDTH_HWORD | GD32_DMA_CTL_MWIDTH_HWORD;
    n /= 2;
    break;
#endif
  default:
    osalDbgAssert(false, "unexpected DAC mode");
    return;
  }

  dmaStreamSetMemory0(dacp->dma, dacp->samples);
  dmaStreamSetTransactionSize(dacp->dma, n);
  dmaStreamSetMode(dacp->dma, dmamode            |
                              GD32_DMA_CTL_ERRIE |
                              GD32_DMA_CTL_HTFIE  | GD32_DMA_CTL_FTFIE);
  dmaStreamEnable(dacp->dma);

  /* DAC configuration.*/
  ctl = dacp->params->dac->CTL;

#if GD32_DAC_DUAL_MODE == FALSE
  ctl &= dacp->params->regmask;
  ctl |= (DAC_CTL_DDMAEN0 | (dacp->grpp->trigger << DAC_CTL_DTSEL0_Pos) | DAC_CTL_DTEN0 | DAC_CTL_DEN0 | dacp->config->ctl) << dacp->params->regshift;
#else
  ctl = DAC_CTL_DDMAEN0 | (dacp->grpp->trigger << DAC_CTL_DTSEL0_Pos) | DAC_CTL_DTEN0 | DAC_CTL_DEN0 | dacp->config->ctl
                     | (dacp->grpp->trigger << DAC_CTL_DTSEL1_Pos) | DAC_CTL_DTEN1 | DAC_CTL_DEN1 | (dacp->config->ctl << 16);
#endif

  dacp->params->dac->CTL = ctl;
}

/**
 * @brief   Stops an ongoing conversion.
 * @details This function stops the currently ongoing conversion and returns
 *          the driver in the @p DAC_READY state. If there was no conversion
 *          being processed then the function does nothing.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @iclass
 */
void dac_lld_stop_conversion(DACDriver *dacp) {
  uint32_t ctl;

  /* DMA channel disabled and released.*/
  dmaStreamDisable(dacp->dma);
  dmaStreamFreeI(dacp->dma);
  dacp->dma = NULL;

  ctl = dacp->params->dac->CTL;

#if GD32_DAC_DUAL_MODE == FALSE
  ctl &= dacp->params->regmask;
  ctl |= (DAC_CTL_DEN0 | dacp->config->ctl) << dacp->params->regshift;
#else
  if ((dacp->config->datamode == DAC_DHRM_12BIT_RIGHT_DUAL) ||
      (dacp->config->datamode == DAC_DHRM_12BIT_LEFT_DUAL) ||
      (dacp->config->datamode == DAC_DHRM_8BIT_RIGHT_DUAL)) {
    ctl = DAC_CTL_DEN1 | (dacp->config->ctl << 16) |
         DAC_CTL_DEN0 | dacp->config->ctl;
  }
  else {
    ctl = DAC_CTL_DEN0 | dacp->config->ctl;
  }
#endif

  dacp->params->dac->CTL = ctl;
}

#endif /* HAL_USE_DAC */

/** @} */
