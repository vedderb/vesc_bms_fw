/*
    ChibiOS - Copyright (C) 2014 Derek Mulcahy

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
 * @file    ADCv1/hal_adc_lld.h
 * @brief   KINETIS ADC subsystem low level driver header.
 *
 * @addtogroup ADC
 * @{
 */

#ifndef HAL_ADC_LLD_H_
#define HAL_ADC_LLD_H_

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Minimum ADC clock frequency.
 */
#define KINETIS_ADCCLK_MIN      600000

/**
 * @brief   Maximum ADC clock frequency.
 */
#define KINETIS_ADCCLK_MAX        36000000

#define ADCx_SC3_AVGS_AVERAGE_4_SAMPLES     0
#define ADCx_SC3_AVGS_AVERAGE_8_SAMPLES     1
#define ADCx_SC3_AVGS_AVERAGE_16_SAMPLES    2
#define ADCx_SC3_AVGS_AVERAGE_32_SAMPLES    3

#define ADCx_CFG1_ADIV_DIV_1                0
#define ADCx_CFG1_ADIV_DIV_2                1
#define ADCx_CFG1_ADIV_DIV_4                2
#define ADCx_CFG1_ADIV_DIV_8                3

#define ADCx_CFG1_ADIVCLK_BUS_CLOCK         0
#define ADCx_CFG1_ADIVCLK_BUS_CLOCK_DIV_2   1
#define ADCx_CFG1_ADIVCLK_BUS_ALTCLK        2
#define ADCx_CFG1_ADIVCLK_BUS_ADACK         3

#define ADCx_CFG1_MODE_8_OR_9_BITS          0
#define ADCx_CFG1_MODE_12_OR_13_BITS        1
#define ADCx_CFG1_MODE_10_OR_11_BITS        2
#define ADCx_CFG1_MODE_16_BITS              3

#define ADCx_SC1n_ADCH_DAD0             0
#define ADCx_SC1n_ADCH_DAD1             1
#define ADCx_SC1n_ADCH_DAD2             2
#define ADCx_SC1n_ADCH_DAD3             3
#define ADCx_SC1n_ADCH_DADP0            0
#define ADCx_SC1n_ADCH_DADP1            1
#define ADCx_SC1n_ADCH_DADP2            2
#define ADCx_SC1n_ADCH_DADP3            3
#define ADCx_SC1n_ADCH_AD4              4
#define ADCx_SC1n_ADCH_AD5              5
#define ADCx_SC1n_ADCH_AD6              6
#define ADCx_SC1n_ADCH_AD7              7
#define ADCx_SC1n_ADCH_AD8              8
#define ADCx_SC1n_ADCH_AD9              9
#define ADCx_SC1n_ADCH_AD10             10
#define ADCx_SC1n_ADCH_AD11             11
#define ADCx_SC1n_ADCH_AD12             12
#define ADCx_SC1n_ADCH_AD13             13
#define ADCx_SC1n_ADCH_AD14             14
#define ADCx_SC1n_ADCH_AD15             15
#define ADCx_SC1n_ADCH_AD16             16
#define ADCx_SC1n_ADCH_AD17             17
#define ADCx_SC1n_ADCH_AD18             18
#define ADCx_SC1n_ADCH_AD19             19
#define ADCx_SC1n_ADCH_AD20             20
#define ADCx_SC1n_ADCH_AD21             21
#define ADCx_SC1n_ADCH_AD22             22
#define ADCx_SC1n_ADCH_AD23             23
#define ADCx_SC1n_ADCH_TEMP_SENSOR      26
#define ADCx_SC1n_ADCH_BANDGAP          27
#define ADCx_SC1n_ADCH_VREFSH           29
#define ADCx_SC1n_ADCH_VREFSL           30
#define ADCx_SC1n_ADCH_DISABLED         31

#define ADC_DAD0                        (1 << ADCx_SC1n_ADCH_DAD0)
#define ADC_DAD1                        (1 << ADCx_SC1n_ADCH_DAD1)
#define ADC_DAD2                        (1 << ADCx_SC1n_ADCH_DAD2)
#define ADC_DAD3                        (1 << ADCx_SC1n_ADCH_DAD3)
#define ADC_DADP0                       (1 << ADCx_SC1n_ADCH_DADP0)
#define ADC_DADP1                       (1 << ADCx_SC1n_ADCH_DADP1)
#define ADC_DADP2                       (1 << ADCx_SC1n_ADCH_DADP2)
#define ADC_DADP3                       (1 << ADCx_SC1n_ADCH_DADP3)
#define ADC_AD4                         (1 << ADCx_SC1n_ADCH_AD4)
#define ADC_AD5                         (1 << ADCx_SC1n_ADCH_AD5)
#define ADC_AD6                         (1 << ADCx_SC1n_ADCH_AD6)
#define ADC_AD7                         (1 << ADCx_SC1n_ADCH_AD7)
#define ADC_AD8                         (1 << ADCx_SC1n_ADCH_AD8)
#define ADC_AD9                         (1 << ADCx_SC1n_ADCH_AD9)
#define ADC_AD10                        (1 << ADCx_SC1n_ADCH_AD10)
#define ADC_AD11                        (1 << ADCx_SC1n_ADCH_AD11)
#define ADC_AD12                        (1 << ADCx_SC1n_ADCH_AD12)
#define ADC_AD13                        (1 << ADCx_SC1n_ADCH_AD13)
#define ADC_AD14                        (1 << ADCx_SC1n_ADCH_AD14)
#define ADC_AD15                        (1 << ADCx_SC1n_ADCH_AD15)
#define ADC_AD16                        (1 << ADCx_SC1n_ADCH_AD16)
#define ADC_AD17                        (1 << ADCx_SC1n_ADCH_AD17)
#define ADC_AD18                        (1 << ADCx_SC1n_ADCH_AD18)
#define ADC_AD19                        (1 << ADCx_SC1n_ADCH_AD19)
#define ADC_AD20                        (1 << ADCx_SC1n_ADCH_AD20)
#define ADC_AD21                        (1 << ADCx_SC1n_ADCH_AD21)
#define ADC_AD22                        (1 << ADCx_SC1n_ADCH_AD22)
#define ADC_AD23                        (1 << ADCx_SC1n_ADCH_AD23)
#define ADC_TEMP_SENSOR                 (1 << ADCx_SC1n_ADCH_TEMP_SENSOR)
#define ADC_BANDGAP                     (1 << ADCx_SC1n_ADCH_BANDGAP)
#define ADC_VREFSH                      (1 << ADCx_SC1n_ADCH_VREFSH)
#define ADC_VREFSL                      (1 << ADCx_SC1n_ADCH_VREFSL)
#define ADC_DISABLED                    (1 << ADCx_SC1n_ADCH_DISABLED)

/** @} */

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
 * @note    The default is @p TRUE.
 */
#if !defined(KINETIS_ADC_USE_ADC0) || defined(__DOXYGEN__)
#define KINETIS_ADC_USE_ADC0                FALSE
#endif

/**
 * @brief   ADC interrupt priority level setting.
 */
#if !defined(KINETIS_ADC_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define KINETIS_ADC_IRQ_PRIORITY            5
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if KINETIS_ADC_USE_ADC0 && !KINETIS_HAS_ADC0
#error "ADC1 not present in the selected device"
#endif

#if !KINETIS_ADC_USE_ADC0
#error "ADC driver activated but no ADC peripheral assigned"
#endif

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
typedef uint16_t adc_channels_num_t;

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
 * @brief   Type of a structure representing an ADC driver.
 */
typedef struct hal_adc_driver ADCDriver;

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_configuration_group_fields                                  \
 /**                                                                        \
   * @brief   Bitmask of channels for ADC conversion.                       \
   */                                                                       \
  uint32_t                  channel_mask;                                   \
  /**                                                                       \
   * @brief   ADC CFG1 register initialization data.                        \
   * @note    All the required bits must be defined into this field.        \
   */                                                                       \
  uint32_t                  cfg1;                                           \
  /**                                                                       \
   * @brief   ADC SC3 register initialization data.                         \
   * @note    All the required bits must be defined into this field.        \
   */                                                                       \
  uint32_t                  sc3;


/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  /* Perform first time calibration */                                      \
  bool                      calibrate;

/**
 * @brief   Low level fields of the ADC driver structure.
 */
#define adc_lld_driver_fields                                               \
  /* Pointer to the ADCx registers block.*/                                 \
  ADC_TypeDef               *adc;                                           \
  /* @brief Number of samples expected. */                                  \
  size_t                    number_of_samples;                              \
  /* @brief Current position in the buffer. */                              \
  size_t                    current_index;                                  \
  /* @brief Current channel index into group channel_mask. */               \
  size_t                    current_channel;                                \

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if KINETIS_ADC_USE_ADC0 && !defined(__DOXYGEN__)
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

#endif /* HAL_ADC_LLD_H_ */

/** @} */
