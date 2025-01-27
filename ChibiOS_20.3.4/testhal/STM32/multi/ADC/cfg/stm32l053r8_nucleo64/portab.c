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
 * @file    portab.c
 * @brief   Application portability module code.
 *
 * @addtogroup application_portability
 * @{
 */

#include "hal.h"

#include "portab.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*
 * GPT configuration.
 */
const GPTConfig portab_gptcfg1 = {
  .frequency    =  1000000U,
  .callback     =  NULL,
  .cr2          =  TIM_CR2_MMS_1,   /* MMS = 010 = TRGO on Update Event.    */
  .dier         =  0U
};

const ADCConfig portab_adccfg1 = {
  .dummy        = 0U
};

void adccallback(ADCDriver *adcp);

/*
 * ADC errors callback, should never happen.
 */
void adcerrorcallback(ADCDriver *adcp, adcerror_t err);

/*
 * ADC conversion group 1.
 * Mode:        Linear buffer, 1 channel, SW triggered.
 * Channels:    IN10.
 */
const ADCConversionGroup portab_adcgrpcfg1 = {
  .circular     = false,
  .num_channels = ADC_GRP1_NUM_CHANNELS,
  .end_cb       = NULL,
  .error_cb     = adcerrorcallback,
  .cfgr1        = ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,     /* CFGR1 */
  .cfgr2        = 0,                                        /* CFGR2 */
  .tr           = ADC_TR(0, 0),                             /* TR */
  .smpr         = ADC_SMPR_SMP_1P5,                         /* SMPR */
  .chselr       = ADC_CHSELR_CHSEL10                        /* CHSELR */
};

/*
 * ADC conversion group2.
 * Mode:        Continuous, 4 channels, HW triggered by GPT6-TRGO.
 * Channels:    IN10, IN11, VRef, Sensor.
 */
const ADCConversionGroup portab_adcgrpcfg2 = {
  .circular     = true,
  .num_channels = ADC_GRP2_NUM_CHANNELS,
  .end_cb       = adccallback,
  .error_cb     = adcerrorcallback,
  .cfgr1        = ADC_CFGR1_CONT |
                  ADC_CFGR1_RES_12BIT |
                  ADC_CFGR1_EXTEN_RISING |
                  ADC_CFGR1_EXTSEL_SRC(0),                  /* CFGR1 */
  .cfgr2        = 0,                                        /* CFGR2 */
  .tr           = ADC_TR(0, 0),                             /* TR */
  .smpr         = ADC_SMPR_SMP_39P5,                        /* SMPR */
  .chselr       = ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL11 |
                  ADC_CHSELR_CHSEL17 | ADC_CHSELR_CHSEL18   /* CHSELR */
};

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void portab_setup(void) {

  /* ADC inputs.*/
  palSetGroupMode(GPIOC, PAL_PORT_BIT(0) | PAL_PORT_BIT(1),
                  0, PAL_MODE_INPUT_ANALOG);
}

/** @} */
