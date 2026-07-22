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
 * @file    TIM/hal_icu_lld.h
 * @brief   GD32 ICU subsystem low level driver header.
 *
 * @addtogroup ICU
 * @{
 */

#ifndef HAL_ICU_LLD_H
#define HAL_ICU_LLD_H

#if HAL_USE_ICU || defined(__DOXYGEN__)

#include "gd32_tim.h"

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
 * @brief   ICUD1 driver enable switch.
 * @details If set to @p TRUE the support for ICUD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_ICU_USE_TIM0) || defined(__DOXYGEN__)
#define GD32_ICU_USE_TIM0                  FALSE
#endif

/**
 * @brief   ICUD2 driver enable switch.
 * @details If set to @p TRUE the support for ICUD2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_ICU_USE_TIM1) || defined(__DOXYGEN__)
#define GD32_ICU_USE_TIM1                  FALSE
#endif

/**
 * @brief   ICUD3 driver enable switch.
 * @details If set to @p TRUE the support for ICUD3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_ICU_USE_TIM2) || defined(__DOXYGEN__)
#define GD32_ICU_USE_TIM2                  FALSE
#endif

/**
 * @brief   ICUD4 driver enable switch.
 * @details If set to @p TRUE the support for ICUD4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_ICU_USE_TIM3) || defined(__DOXYGEN__)
#define GD32_ICU_USE_TIM3                  FALSE
#endif

/**
 * @brief   ICUD5 driver enable switch.
 * @details If set to @p TRUE the support for ICUD5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_ICU_USE_TIM4) || defined(__DOXYGEN__)
#define GD32_ICU_USE_TIM4                  FALSE
#endif

/**
 * @brief   ICUD1 interrupt priority level setting.
 */
#if !defined(GD32_ICU_TIM0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_ICU_TIM0_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD2 interrupt priority level setting.
 */
#if !defined(GD32_ICU_TIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_ICU_TIM1_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD3 interrupt priority level setting.
 */
#if !defined(GD32_ICU_TIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_ICU_TIM2_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD4 interrupt priority level setting.
 */
#if !defined(GD32_ICU_TIM3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_ICU_TIM3_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD5 interrupt priority level setting.
 */
#if !defined(GD32_ICU_TIM4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_ICU_TIM4_IRQ_PRIORITY         7
#endif

/**
 * @brief   ICUD1 interrupt trigger setting.
 */
#if !defined(GD32_ICU_TIM0_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_ICU_TIM0_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   ICUD2 interrupt trigger setting.
 */
#if !defined(GD32_ICU_TIM1_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_ICU_TIM1_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   ICUD3 interrupt trigger setting.
 */
#if !defined(GD32_ICU_TIM2_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_ICU_TIM2_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   ICUD4 interrupt trigger setting.
 */
#if !defined(GD32_ICU_TIM3_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_ICU_TIM3_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   ICUD5 interrupt trigger setting.
 */
#if !defined(GD32_ICU_TIM4_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_ICU_TIM4_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(GD32_HAS_TIM0)
#define GD32_HAS_TIM0                      FALSE
#endif

#if !defined(GD32_HAS_TIM1)
#define GD32_HAS_TIM1                      FALSE
#endif

#if !defined(GD32_HAS_TIM2)
#define GD32_HAS_TIM2                      FALSE
#endif

#if !defined(GD32_HAS_TIM3)
#define GD32_HAS_TIM3                      FALSE
#endif

#if !defined(GD32_HAS_TIM4)
#define GD32_HAS_TIM4                      FALSE
#endif

#if GD32_ICU_USE_TIM0 && !GD32_HAS_TIM0
#error "TIM0 not present in the selected device"
#endif

#if GD32_ICU_USE_TIM1 && !GD32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if GD32_ICU_USE_TIM2 && !GD32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if GD32_ICU_USE_TIM3 && !GD32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if GD32_ICU_USE_TIM4 && !GD32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if !GD32_ICU_USE_TIM0  && !GD32_ICU_USE_TIM1  &&                         \
    !GD32_ICU_USE_TIM2  && !GD32_ICU_USE_TIM3  &&                         \
    !GD32_ICU_USE_TIM4  
#error "ICU driver activated but no TIM peripheral assigned"
#endif

/* Checks on allocation of TIMx units.*/
#if GD32_ICU_USE_TIM0
#if defined(GD32_TIM0_IS_USED)
#error "ICUD1 requires TIM0 but the timer is already used"
#else
#define GD32_TIM0_IS_USED
#endif
#endif

#if GD32_ICU_USE_TIM1
#if defined(GD32_TIM1_IS_USED)
#error "ICUD2 requires TIM1 but the timer is already used"
#else
#define GD32_TIM1_IS_USED
#endif
#endif

#if GD32_ICU_USE_TIM2
#if defined(GD32_TIM2_IS_USED)
#error "ICUD3 requires TIM2 but the timer is already used"
#else
#define GD32_TIM2_IS_USED
#endif
#endif

#if GD32_ICU_USE_TIM3
#if defined(GD32_TIM3_IS_USED)
#error "ICUD4 requires TIM3 but the timer is already used"
#else
#define GD32_TIM3_IS_USED
#endif
#endif

#if GD32_ICU_USE_TIM4
#if defined(GD32_TIM4_IS_USED)
#error "ICUD5 requires TIM4 but the timer is already used"
#else
#define GD32_TIM4_IS_USED
#endif
#endif

/* IRQ priority checks.*/
#if GD32_ICU_USE_TIM0 && !defined(GD32_TIM0_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_ICU_TIM0_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM0"
#endif

#if GD32_ICU_USE_TIM1 && !defined(GD32_TIM1_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_ICU_TIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM1"
#endif

#if GD32_ICU_USE_TIM2 && !defined(GD32_TIM2_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_ICU_TIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM2"
#endif

#if GD32_ICU_USE_TIM3 && !defined(GD32_TIM3_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_ICU_TIM3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM3"
#endif

#if GD32_ICU_USE_TIM4 && !defined(GD32_TIM4_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_ICU_TIM4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM4"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ICU driver mode.
 */
typedef enum {
  ICU_INPUT_ACTIVE_HIGH = 0,        /**< Trigger on rising edge.            */
  ICU_INPUT_ACTIVE_LOW = 1,         /**< Trigger on falling edge.           */
} icumode_t;

/**
 * @brief   ICU frequency type.
 */
typedef uint32_t icufreq_t;

/**
 * @brief   ICU channel type.
 */
typedef enum {
  ICU_CHANNEL_1 = 0,              /**< Use TIMxCH1.      */
  ICU_CHANNEL_2 = 1,              /**< Use TIMxCH2.      */
} icuchannel_t;

/**
 * @brief   ICU counter type.
 */
typedef uint32_t icucnt_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Driver mode.
   */
  icumode_t                 mode;
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  icufreq_t                 frequency;
  /**
   * @brief   Callback for pulse width measurement.
   */
  icucallback_t             width_cb;
  /**
   * @brief   Callback for cycle period measurement.
   */
  icucallback_t             period_cb;
  /**
   * @brief   Callback for timer overflow.
   */
  icucallback_t             overflow_cb;
  /* End of the mandatory fields.*/
  /**
   * @brief   Timer input channel to be used.
   * @note    Only inputs TIMx 1 and 2 are supported.
   */
  icuchannel_t              channel;
  /**
   * @brief TIM DIER register initialization data.
   * @note  The value of this field should normally be equal to zero.
   * @note  Only the DMA-related bits can be specified in this field.
   */
  uint32_t                  dmainten;
  /**
   * @brief TIM ARR register initialization data.
   * @note  The value of this field should normally be equal to 0xFFFFFFFFU.
   */
  uint32_t                  car;
} ICUConfig;

/**
 * @brief   Structure representing an ICU driver.
 */
struct ICUDriver {
  /**
   * @brief Driver state.
   */
  icustate_t                state;
  /**
   * @brief Current configuration data.
   */
  const ICUConfig           *config;
#if defined(ICU_DRIVER_EXT_FIELDS)
  ICU_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Timer base clock.
   */
  uint32_t                  clock;
  /**
   * @brief Pointer to the TIMx registers block.
   */
  gd32_tim_t               *tim;
  /**
   * @brief CCR register used for width capture.
   */
  volatile uint32_t         *wccrp;
  /**
   * @brief CCR register used for period capture.
   */
  volatile uint32_t         *pccrp;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the width of the latest pulse.
 * @details The pulse width is defined as number of ticks between the start
 *          edge and the stop edge.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The number of ticks.
 *
 * @notapi
 */
#define icu_lld_get_width(icup) (*((icup)->wccrp) + 1)

/**
 * @brief   Returns the width of the latest cycle.
 * @details The cycle width is defined as number of ticks between a start
 *          edge and the next start edge.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The number of ticks.
 *
 * @notapi
 */
#define icu_lld_get_period(icup) (*((icup)->pccrp) + 1)

/**
 * @brief   Check on notifications status.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 * @return              The notifications status.
 * @retval false        if notifications are not enabled.
 * @retval true         if notifications are enabled.
 *
 * @notapi
 */
#define icu_lld_are_notifications_enabled(icup)                             \
  (bool)(((icup)->tim->DMAINTEN & GD32_TIM_DIER_IRQ_MASK) != 0)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if GD32_ICU_USE_TIM0 && !defined(__DOXYGEN__)
extern ICUDriver ICUD1;
#endif

#if GD32_ICU_USE_TIM1 && !defined(__DOXYGEN__)
extern ICUDriver ICUD2;
#endif

#if GD32_ICU_USE_TIM2 && !defined(__DOXYGEN__)
extern ICUDriver ICUD3;
#endif

#if GD32_ICU_USE_TIM3 && !defined(__DOXYGEN__)
extern ICUDriver ICUD4;
#endif

#if GD32_ICU_USE_TIM4 && !defined(__DOXYGEN__)
extern ICUDriver ICUD5;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void icu_lld_init(void);
  void icu_lld_start(ICUDriver *icup);
  void icu_lld_stop(ICUDriver *icup);
  void icu_lld_start_capture(ICUDriver *icup);
  bool icu_lld_wait_capture(ICUDriver *icup);
  void icu_lld_stop_capture(ICUDriver *icup);
  void icu_lld_enable_notifications(ICUDriver *icup);
  void icu_lld_disable_notifications(ICUDriver *icup);
  void icu_lld_serve_interrupt(ICUDriver *icup);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ICU */

#endif /* HAL_ICU_LLD_H */

/** @} */
