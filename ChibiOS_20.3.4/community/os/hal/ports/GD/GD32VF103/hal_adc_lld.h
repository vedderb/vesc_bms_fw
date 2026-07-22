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
 * @file    GD32VF103/hal_adc_lld.h
 * @brief   GD32VF103 ADC subsystem low level driver header.
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

/**
 * @name    Triggers selection
 * @{
 */
#define ADC_CTL1_ETSRC_SRC(n)   ((n) << 17) /**< @brief Trigger source.     */
#define ADC_CTL1_ETSRC_SWSTART  (7 << 17)   /**< @brief Software trigger.   */
/** @} */

/**
 * @name    Available analog channels
 * @{
 */
#define ADC_CHANNEL_IN0         0   /**< @brief External analog input 0.    */
#define ADC_CHANNEL_IN1         1   /**< @brief External analog input 1.    */
#define ADC_CHANNEL_IN2         2   /**< @brief External analog input 2.    */
#define ADC_CHANNEL_IN3         3   /**< @brief External analog input 3.    */
#define ADC_CHANNEL_IN4         4   /**< @brief External analog input 4.    */
#define ADC_CHANNEL_IN5         5   /**< @brief External analog input 5.    */
#define ADC_CHANNEL_IN6         6   /**< @brief External analog input 6.    */
#define ADC_CHANNEL_IN7         7   /**< @brief External analog input 7.    */
#define ADC_CHANNEL_IN8         8   /**< @brief External analog input 8.    */
#define ADC_CHANNEL_IN9         9   /**< @brief External analog input 9.    */
#define ADC_CHANNEL_IN10        10  /**< @brief External analog input 10.   */
#define ADC_CHANNEL_IN11        11  /**< @brief External analog input 11.   */
#define ADC_CHANNEL_IN12        12  /**< @brief External analog input 12.   */
#define ADC_CHANNEL_IN13        13  /**< @brief External analog input 13.   */
#define ADC_CHANNEL_IN14        14  /**< @brief External analog input 14.   */
#define ADC_CHANNEL_IN15        15  /**< @brief External analog input 15.   */
#define ADC_CHANNEL_SENSOR      16  /**< @brief Internal temperature sensor.*/
#define ADC_CHANNEL_VREFINT     17  /**< @brief Internal reference.         */
/** @} */

/**
 * @name    Sampling rates
 * @{
 */
#define ADC_SAMPLE_1P5          0   /**< @brief 1.5 cycles sampling time.   */
#define ADC_SAMPLE_7P5          1   /**< @brief 7.5 cycles sampling time.   */
#define ADC_SAMPLE_13P5         2   /**< @brief 13.5 cycles sampling time.  */
#define ADC_SAMPLE_28P5         3   /**< @brief 28.5 cycles sampling time.  */
#define ADC_SAMPLE_41P5         4   /**< @brief 41.5 cycles sampling time.  */
#define ADC_SAMPLE_55P5         5   /**< @brief 55.5 cycles sampling time.  */
#define ADC_SAMPLE_71P5         6   /**< @brief 71.5 cycles sampling time.  */
#define ADC_SAMPLE_239P5        7   /**< @brief 239.5 cycles sampling time. */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   ADC0 driver enable switch.
 * @details If set to @p TRUE the support for ADC0 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(GD32_ADC_USE_ADC0) || defined(__DOXYGEN__)
#define GD32_ADC_USE_ADC0                  FALSE
#endif

/**
 * @brief   ADC0 DMA priority (0..3|lowest..highest).
 */
#if !defined(GD32_ADC_ADC0_DMA_PRIORITY) || defined(__DOXYGEN__)
#define GD32_ADC_ADC0_DMA_PRIORITY         2
#endif

/**
 * @brief   ADC0 interrupt priority level setting.
 */
#if !defined(GD32_ADC_ADC0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_ADC_ADC0_IRQ_PRIORITY         5
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if GD32_ADC_USE_ADC0 && !GD32_HAS_ADC0
#error "ADC0 not present in the selected device"
#endif

#if !GD32_ADC_USE_ADC0
#error "ADC driver activated but no ADC peripheral assigned"
#endif

#if !defined(GD32_DMA_REQUIRED)
#define GD32_DMA_REQUIRED
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
  ADC_ERR_DMAFAILURE = 0                    /**< DMA operations failure.    */
} adcerror_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the ADC driver structure.
 */
#define adc_lld_driver_fields                                               \
  /* Pointer to the ADCx registers block.*/                                 \
  ADC_TypeDef               *adc;                                           \
  /* Pointer to associated DMA channel.*/                                   \
  const gd32_dma_stream_t  *dmastp;                                        \
  /* DMA mode bit mask.*/                                                   \
  uint32_t                  dmamode

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  /* Dummy configuration, it is not needed.*/                               \
  uint32_t                  dummy

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_configuration_group_fields                                  \
  /* ADC CTL0 register initialization data.                                  \
     NOTE: All the required bits must be defined into this field except     \
           @p ADC_CTL0_SM that is enforced inside the driver.*/            \
  uint32_t                  ctl0;                                            \
  /* ADC CTL1 register initialization data.                                  \
     NOTE: All the required bits must be defined into this field except     \
           @p ADC_CTL1_DMA, @p ADC_CTL1_CTN and @p ADC_CTL1_ADCON that are     \
           enforced inside the driver.*/                                    \
  uint32_t                  ctl1;                                            \
  /* ADC SAMPT0 register initialization data.                                \
     NOTE: In this field must be specified the sample times for channels    \
           10...17.*/                                                       \
  uint32_t                  sampt0;                                          \
  /* ADC SAMPT1 register initialization data.                                \
     NOTE: In this field must be specified the sample times for channels    \
           0...9.*/                                                         \
  uint32_t                  sampt1;                                          \
  /* ADC RSQ0 register initialization data.                                 \
     NOTE: Conversion group sequence 13...16 + sequence length.*/           \
  uint32_t                  rsq0;                                           \
  /* ADC RSQ1 register initialization data.                                 \
     NOTE: Conversion group sequence 7...12.*/                              \
  uint32_t                  rsq1;                                           \
  /* ADC RSQ2 register initialization data.                                 \
     NOTE: Conversion group sequence 1...6.*/                               \
  uint32_t                  rsq2

/**
 * @name    Sequences building helper macros
 * @{
 */
/**
 * @brief   Number of channels in a conversion sequence.
 */
#define ADC_RSQ0_NUM_CH(n)      (((n) - 1) << 20)

#define ADC_RSQ2_RSQ1_N(n)       ((n) << 0)  /**< @brief 1st channel in seq. */
#define ADC_RSQ2_RSQ2_N(n)       ((n) << 5)  /**< @brief 2nd channel in seq. */
#define ADC_RSQ2_RSQ3_N(n)       ((n) << 10) /**< @brief 3rd channel in seq. */
#define ADC_RSQ2_RSQ4_N(n)       ((n) << 15) /**< @brief 4th channel in seq. */
#define ADC_RSQ2_RSQ5_N(n)       ((n) << 20) /**< @brief 5th channel in seq. */
#define ADC_RSQ2_RSQ6_N(n)       ((n) << 25) /**< @brief 6th channel in seq. */

#define ADC_RSQ1_RSQ7_N(n)       ((n) << 0)  /**< @brief 7th channel in seq. */
#define ADC_RSQ1_RSQ8_N(n)       ((n) << 5)  /**< @brief 8th channel in seq. */
#define ADC_RSQ1_RSQ9_N(n)       ((n) << 10) /**< @brief 9th channel in seq. */
#define ADC_RSQ1_RSQ10_N(n)      ((n) << 15) /**< @brief 10th channel in seq.*/
#define ADC_RSQ1_RSQ11_N(n)      ((n) << 20) /**< @brief 11th channel in seq.*/
#define ADC_RSQ1_RSQ12_N(n)      ((n) << 25) /**< @brief 12th channel in seq.*/

#define ADC_RSQ0_RSQ13_N(n)      ((n) << 0)  /**< @brief 13th channel in seq.*/
#define ADC_RSQ0_RSQ14_N(n)      ((n) << 5)  /**< @brief 14th channel in seq.*/
#define ADC_RSQ0_RSQ15_N(n)      ((n) << 10) /**< @brief 15th channel in seq.*/
#define ADC_RSQ0_RSQ16_N(n)      ((n) << 15) /**< @brief 16th channel in seq.*/
/** @} */

/**
 * @name    Sampling rate settings helper macros
 * @{
 */
#define ADC_SAMPT1_SMP_SPT0(n)    ((n) << 0)  /**< @brief AN0 sampling time.  */
#define ADC_SAMPT1_SMP_SPT1(n)    ((n) << 3)  /**< @brief AN1 sampling time.  */
#define ADC_SAMPT1_SMP_SPT2(n)    ((n) << 6)  /**< @brief AN2 sampling time.  */
#define ADC_SAMPT1_SMP_SPT3(n)    ((n) << 9)  /**< @brief AN3 sampling time.  */
#define ADC_SAMPT1_SMP_SPT4(n)    ((n) << 12) /**< @brief AN4 sampling time.  */
#define ADC_SAMPT1_SMP_SPT5(n)    ((n) << 15) /**< @brief AN5 sampling time.  */
#define ADC_SAMPT1_SMP_SPT6(n)    ((n) << 18) /**< @brief AN6 sampling time.  */
#define ADC_SAMPT1_SMP_SPT7(n)    ((n) << 21) /**< @brief AN7 sampling time.  */
#define ADC_SAMPT1_SMP_SPT8(n)    ((n) << 24) /**< @brief AN8 sampling time.  */
#define ADC_SAMPT1_SMP_SPT9(n)    ((n) << 27) /**< @brief AN9 sampling time.  */

#define ADC_SAMPT0_SMP_SPT10(n)   ((n) << 0)  /**< @brief AN10 sampling time. */
#define ADC_SAMPT0_SMP_SPT11(n)   ((n) << 3)  /**< @brief AN11 sampling time. */
#define ADC_SAMPT0_SMP_SPT12(n)   ((n) << 6)  /**< @brief AN12 sampling time. */
#define ADC_SAMPT0_SMP_SPT13(n)   ((n) << 9)  /**< @brief AN13 sampling time. */
#define ADC_SAMPT0_SMP_SPT14(n)   ((n) << 12) /**< @brief AN14 sampling time. */
#define ADC_SAMPT0_SMP_SPT15(n)   ((n) << 15) /**< @brief AN15 sampling time. */
#define ADC_SAMPT0_SMP_SENSOR(n) ((n) << 18) /**< @brief Temperature Sensor
                                                 sampling time.             */
#define ADC_SAMPT0_SMP_VREF(n)   ((n) << 21) /**< @brief Voltage Reference
                                                 sampling time.             */
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if GD32_ADC_USE_ADC0 && !defined(__DOXYGEN__)
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
