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
 * @file    hal_adc_lld.h
 * @brief   PLATFORM ADC subsystem low level driver header.
 *
 * @addtogroup ADC
 * @{
 */

#ifndef HAL_ADC_LLD_H
#define HAL_ADC_LLD_H

#if (HAL_USE_ADC == TRUE) || defined(__DOXYGEN__)

#include "rp2040_adc.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Possible ADC errors mask bits.
 * @{
 */
#define ADC_ERR_DMAFAILURE      1U  /**< DMA operations failure.            */
#define ADC_ERR_OVERFLOW        2U  /**< ADC overflow condition.            */
#define ADC_ERR_AWD             4U  /**< Watchdog triggered.                */
#define ADC_ERR_CONVERSION      8U  /**< Result is undefined or noisy.      */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    PLATFORM configuration options
 * @{
 */
/**
 * @brief   ADC1 driver enable switch.
 * @details If set to @p TRUE the support for ADC1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(RP_ADC_USE_ADC1) || defined(__DOXYGEN__)
#define RP_ADC_USE_ADC1                  FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/**
 * @name    Possible ADC channel mask bits.
 * @{
 */
#define RP_ADC_CH0             (1U << 0)  /**< CH0 */
#define RP_ADC_CH1             (1U << 1)  /**< CH1 */
#define RP_ADC_CH2             (1U << 2)  /**< CH2 */
#define RP_ADC_CH3             (1U << 3)  /**< CH3 */
#define RP_ADC_CH4             (1U << 4)  /**< CH4 */
#define RP_ADC_CHTS            RP_ADC_CH4    /**< Temperature sensor, known as CH4 */
/** @} */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ADC sample data type.
 */
typedef uint16_t adcsample_t;

/**
 * @brief   Channels number in a conversion group.
 */
typedef uint8_t adc_channels_num_t;

/**
 * @brief   Type of an ADC error mask.
 */
typedef uint32_t adcerror_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the ADC driver structure.
 */
#define adc_lld_driver_fields                                               \
  /* ADC register. */                                                       \
  ADC_TypeDef               *adc;                                           \
  /* Current index in the buffer. */                                        \
  size_t                    current_buffer_position;                        \
  /* Current channel index. */                                              \
  size_t                    current_channel;                                \
  /* Current iteration in the depth. */                                     \
  size_t                    current_iteration;

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  /* DIV.INT register value. */                                             \
  uint16_t                  div_int;                                        \
  /* DIV.FRAC register value. */                                            \
  uint8_t                   div_frac;                                       \
  /* Shift 8-bits when result move to FIFO. */                              \
  bool                      shift;

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_configuration_group_fields                                  \
  /* Bitmask of channels for ADC conversion. */                             \
  uint8_t                  channel_mask;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (RP_ADC_USE_ADC1 == TRUE) && !defined(__DOXYGEN__)
extern ADCDriver ADCD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void adc_lld_init(void);
  void adc_lld_start(ADCDriver *adcp);
  void adc_lld_stop(ADCDriver *adcp);
  void adc_lld_start_conversion(ADCDriver *adcp);
  void adc_lld_stop_conversion(ADCDriver *adcp);
  void adcRPEnableTS(ADCDriver *adcp);
  void adcRPDisableTS(ADCDriver *adcp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC == TRUE */

#endif /* HAL_ADC_LLD_H */

/** @} */
