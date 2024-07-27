/*
    Copyright (C) 2015 Stephen Caudle

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
 * @file    ADCv1/adc_lld.h
 * @brief   NRF51 ADC subsystem low level driver header.
 *
 * @addtogroup ADC
 * @{
 */

#ifndef HAL_ADC_LLD_H
#define HAL_ADC_LLD_H

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   ADC1 driver enable switch.
 * @details If set to @p TRUE the support for ADC1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(NRF5_ADC_USE_ADC1) || defined(__DOXYGEN__)
#define NRF5_ADC_USE_ADC1                  FALSE
#endif

/**
 * @brief   ADC interrupt priority level setting.
 */
#if !defined(NRF5_ADC_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define NRF5_ADC_IRQ_PRIORITY              2
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !NRF5_ADC_USE_ADC1
#error "ADC driver activated but no ADC peripheral assigned"
#endif

#if NRF5_ADC_USE_ADC1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(NRF5_ADC_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC1"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   Possible ADC failure causes.
 * @note    Error codes are architecture dependent and should not relied
 *          upon.
 */
typedef enum {
  ADC_ERR_DMAFAILURE = 0,                   /**< DMA operations failure.    */
  ADC_ERR_OVERFLOW = 1                      /**< ADC overflow condition.    */
} adcerror_t;

/**
 * @brief   ADC sample data type.
 */
typedef uint16_t adcsample_t;

/**
 * @brief   Channels number in a conversion group.
 */
typedef uint8_t adc_channels_num_t;

/**
 * @brief   Low level fields of the ADC driver structure.
 */
#define adc_lld_driver_fields                                               \
  /* @brief Pointer to the ADCx registers block. */                         \
  NRF_ADC_Type             *adc;                                            \
  /* @brief Number of samples expected. */                                  \
  size_t                    number_of_samples;                              \
  /* @brief Current position in the buffer. */                              \
  size_t                    current_index;                                  \
  /* @brief Current channel index into group channel_mask. */               \
  size_t                    current_channel;

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  /* Dummy configuration, it is not needed.*/                               \
  uint32_t                  dummy

#define adc_lld_configuration_group_fields                                  \
  /**                                                                       \
   * @brief   Bitmask of channels for ADC conversion.                       \
   */                                                                       \
  uint32_t                  channel_mask;                                   \
  /**                                                                       \
   * @brief   ADC CONFIG register details.                                  \
   * @note    All the required bits must be defined into this field.        \
   */                                                                       \
  uint32_t                  cfg;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if NRF5_ADC_USE_ADC1 && !defined(__DOXYGEN__)
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
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC */

#endif /* HAL_ADC_LLD_H */

/** @} */
