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
 * @file    TIM/hal_gpt_lld.h
 * @brief   GD32 GPT subsystem low level driver header.
 *
 * @addtogroup GPT
 * @{
 */

#ifndef HAL_GPT_LLD_H
#define HAL_GPT_LLD_H

#include "gd32_tim.h"

#if HAL_USE_GPT || defined(__DOXYGEN__)

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
 * @brief   GPTD1 driver enable switch.
 * @details If set to @p TRUE the support for GPTD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_GPT_USE_TIM0) || defined(__DOXYGEN__)
#define GD32_GPT_USE_TIM0                  FALSE
#endif

/**
 * @brief   GPTD2 driver enable switch.
 * @details If set to @p TRUE the support for GPTD2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_GPT_USE_TIM1) || defined(__DOXYGEN__)
#define GD32_GPT_USE_TIM1                  FALSE
#endif

/**
 * @brief   GPTD3 driver enable switch.
 * @details If set to @p TRUE the support for GPTD3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_GPT_USE_TIM2) || defined(__DOXYGEN__)
#define GD32_GPT_USE_TIM2                  FALSE
#endif

/**
 * @brief   GPTD4 driver enable switch.
 * @details If set to @p TRUE the support for GPTD4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_GPT_USE_TIM3) || defined(__DOXYGEN__)
#define GD32_GPT_USE_TIM3                  FALSE
#endif

/**
 * @brief   GPTD5 driver enable switch.
 * @details If set to @p TRUE the support for GPTD5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_GPT_USE_TIM4) || defined(__DOXYGEN__)
#define GD32_GPT_USE_TIM4                  FALSE
#endif

/**
 * @brief   GPTD6 driver enable switch.
 * @details If set to @p TRUE the support for GPTD6 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_GPT_USE_TIM5) || defined(__DOXYGEN__)
#define GD32_GPT_USE_TIM5                  FALSE
#endif

/**
 * @brief   GPTD7 driver enable switch.
 * @details If set to @p TRUE the support for GPTD7 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_GPT_USE_TIM6) || defined(__DOXYGEN__)
#define GD32_GPT_USE_TIM6                  FALSE
#endif

/**
 * @brief   GPTD1 interrupt priority level setting.
 */
#if !defined(GD32_GPT_TIM0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_GPT_TIM0_IRQ_PRIORITY         7
#endif

/**
 * @brief   GPTD2 interrupt priority level setting.
 */
#if !defined(GD32_GPT_TIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_GPT_TIM1_IRQ_PRIORITY         7
#endif

/**
 * @brief   GPTD3 interrupt priority level setting.
 */
#if !defined(GD32_GPT_TIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_GPT_TIM2_IRQ_PRIORITY         7
#endif

/**
 * @brief   GPTD4 interrupt priority level setting.
 */
#if !defined(GD32_GPT_TIM3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_GPT_TIM3_IRQ_PRIORITY         7
#endif

/**
 * @brief   GPTD5 interrupt priority level setting.
 */
#if !defined(GD32_GPT_TIM4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_GPT_TIM4_IRQ_PRIORITY         7
#endif

/**
 * @brief   GPTD6 interrupt priority level setting.
 */
#if !defined(GD32_GPT_TIM5_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_GPT_TIM5_IRQ_PRIORITY         7
#endif

/**
 * @brief   GPTD7 interrupt priority level setting.
 */
#if !defined(GD32_GPT_TIM6_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_GPT_TIM6_IRQ_PRIORITY         7
#endif

/**
 * @brief   GPTD1 interrupt trigger setting.
 */
#if !defined(GD32_GPT_TIM0_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_GPT_TIM0_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   GPTD2 interrupt trigger setting.
 */
#if !defined(GD32_GPT_TIM1_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_GPT_TIM1_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   GPTD3 interrupt trigger setting.
 */
#if !defined(GD32_GPT_TIM2_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_GPT_TIM2_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   GPTD4 interrupt trigger setting.
 */
#if !defined(GD32_GPT_TIM3_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_GPT_TIM3_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   GPTD5 interrupt trigger setting.
 */
#if !defined(GD32_GPT_TIM4_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_GPT_TIM4_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   GPTD6 interrupt trigger setting.
 */
#if !defined(GD32_GPT_TIM5_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_GPT_TIM5_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   GPTD7 interrupt trigger setting.
 */
#if !defined(GD32_GPT_TIM6_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_GPT_TIM6_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
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

#if !defined(GD32_HAS_TIM5)
#define GD32_HAS_TIM5                      FALSE
#endif

#if !defined(GD32_HAS_TIM6)
#define GD32_HAS_TIM6                      FALSE
#endif

#if GD32_GPT_USE_TIM0 && !GD32_HAS_TIM0
#error "TIM0 not present in the selected device"
#endif

#if GD32_GPT_USE_TIM1 && !GD32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if GD32_GPT_USE_TIM2 && !GD32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if GD32_GPT_USE_TIM3 && !GD32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if GD32_GPT_USE_TIM4 && !GD32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if GD32_GPT_USE_TIM5 && !GD32_HAS_TIM5
#error "TIM5 not present in the selected device"
#endif

#if GD32_GPT_USE_TIM6 && !GD32_HAS_TIM6
#error "TIM6 not present in the selected device"
#endif

#if !GD32_GPT_USE_TIM0  && !GD32_GPT_USE_TIM1 &&                          \
    !GD32_GPT_USE_TIM2  && !GD32_GPT_USE_TIM3 &&                          \
    !GD32_GPT_USE_TIM4  && !GD32_GPT_USE_TIM5 &&                          \
    !GD32_GPT_USE_TIM6  
    #error "GPT driver activated but no TIM peripheral assigned"
#endif

/* Checks on allocation of TIMx units.*/
#if GD32_GPT_USE_TIM0
#if defined(GD32_TIM0_IS_USED)
#error "GPTD1 requires TIM0 but the timer is already used"
#else
#define GD32_TIM0_IS_USED
#endif
#endif

#if GD32_GPT_USE_TIM1
#if defined(GD32_TIM1_IS_USED)
#error "GPTD2 requires TIM1 but the timer is already used"
#else
#define GD32_TIM1_IS_USED
#endif
#endif

#if GD32_GPT_USE_TIM2
#if defined(GD32_TIM2_IS_USED)
#error "GPTD3 requires TIM2 but the timer is already used"
#else
#define GD32_TIM2_IS_USED
#endif
#endif

#if GD32_GPT_USE_TIM3
#if defined(GD32_TIM3_IS_USED)
#error "GPTD4 requires TIM3 but the timer is already used"
#else
#define GD32_TIM3_IS_USED
#endif
#endif

#if GD32_GPT_USE_TIM4
#if defined(GD32_TIM4_IS_USED)
#error "GPTD5 requires TIM4 but the timer is already used"
#else
#define GD32_TIM4_IS_USED
#endif
#endif

#if GD32_GPT_USE_TIM5
#if defined(GD32_TIM5_IS_USED)
#error "GPTD6 requires TIM5 but the timer is already used"
#else
#define GD32_TIM5_IS_USED
#endif
#endif

#if GD32_GPT_USE_TIM6
#if defined(GD32_TIM6_IS_USED)
#error "GPTD7 requires TIM6 but the timer is already used"
#else
#define GD32_TIM6_IS_USED
#endif
#endif

/* IRQ priority checks.*/
#if GD32_GPT_USE_TIM0 && !defined(GD32_TIM0_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_GPT_TIM0_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM0"
#endif

#if GD32_GPT_USE_TIM1 && !defined(GD32_TIM1_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_GPT_TIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM1"
#endif

#if GD32_GPT_USE_TIM2 && !defined(GD32_TIM2_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_GPT_TIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM2"
#endif

#if GD32_GPT_USE_TIM3 && !defined(GD32_TIM_SUPPRESS_ISR) &&               \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_GPT_TIM3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM3"
#endif

#if GD32_GPT_USE_TIM4 && !defined(GD32_TIM4_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_GPT_TIM4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM4"
#endif

#if GD32_GPT_USE_TIM5 && !defined(GD32_TIM5_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_GPT_TIM5_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM5"
#endif

#if GD32_GPT_USE_TIM6 && !defined(GD32_TIM6_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_GPT_TIM6_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM6"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   GPT frequency type.
 */
typedef uint32_t gptfreq_t;

/**
 * @brief   GPT counter type.
 */
typedef uint32_t gptcnt_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  gptfreq_t                 frequency;
  /**
   * @brief   Timer callback pointer.
   * @note    This callback is invoked on GPT counter events.
   * @note    This callback can be set to @p NULL but in that case the
   *          one-shot mode cannot be used.
   */
  gptcallback_t             callback;
  /* End of the mandatory fields.*/
  /**
   * @brief TIM CR2 register initialization data.
   * @note  The value of this field should normally be equal to zero.
   */
  uint32_t                  ctl1;
  /**
   * @brief TIM DIER register initialization data.
   * @note  The value of this field should normally be equal to zero.
   * @note  Only the DMA-related bits can be specified in this field.
   */
  uint32_t                  dmainten;
} GPTConfig;

/**
 * @brief   Structure representing a GPT driver.
 */
struct GPTDriver {
  /**
   * @brief Driver state.
   */
  gptstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const GPTConfig           *config;
#if defined(GPT_DRIVER_EXT_FIELDS)
  GPT_DRIVER_EXT_FIELDS
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
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Changes the interval of GPT peripheral.
 * @details This function changes the interval of a running GPT unit.
 * @pre     The GPT unit must be running in continuous mode.
 * @post    The GPT unit interval is changed to the new value.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @param[in] interval  new cycle time in timer ticks
 *
 * @notapi
 */
#define gpt_lld_change_interval(gptp, interval)                             \
  ((gptp)->tim->CAR = (uint32_t)((interval) - 1U))

/**
 * @brief   Returns the interval of GPT peripheral.
 * @pre     The GPT unit must be running in continuous mode.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @return              The current interval.
 *
 * @notapi
 */
#define gpt_lld_get_interval(gptp) ((gptcnt_t)((gptp)->tim->CAR + 1U))

/**
 * @brief   Returns the counter value of GPT peripheral.
 * @pre     The GPT unit must be running in continuous mode.
 * @note    The nature of the counter is not defined, it may count upward
 *          or downward, it could be continuously running or not.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @return              The current counter value.
 *
 * @notapi
 */
#define gpt_lld_get_counter(gptp) ((gptcnt_t)(gptp)->tim->CNT)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if GD32_GPT_USE_TIM0 && !defined(__DOXYGEN__)
extern GPTDriver GPTD1;
#endif

#if GD32_GPT_USE_TIM1 && !defined(__DOXYGEN__)
extern GPTDriver GPTD2;
#endif

#if GD32_GPT_USE_TIM2 && !defined(__DOXYGEN__)
extern GPTDriver GPTD3;
#endif

#if GD32_GPT_USE_TIM3 && !defined(__DOXYGEN__)
extern GPTDriver GPTD4;
#endif

#if GD32_GPT_USE_TIM4 && !defined(__DOXYGEN__)
extern GPTDriver GPTD5;
#endif

#if GD32_GPT_USE_TIM5 && !defined(__DOXYGEN__)
extern GPTDriver GPTD6;
#endif

#if GD32_GPT_USE_TIM6 && !defined(__DOXYGEN__)
extern GPTDriver GPTD7;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void gpt_lld_init(void);
  void gpt_lld_start(GPTDriver *gptp);
  void gpt_lld_stop(GPTDriver *gptp);
  void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t period);
  void gpt_lld_stop_timer(GPTDriver *gptp);
  void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval);
  void gpt_lld_serve_interrupt(GPTDriver *gptp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_GPT */

#endif /* HAL_GPT_LLD_H */

/** @} */
