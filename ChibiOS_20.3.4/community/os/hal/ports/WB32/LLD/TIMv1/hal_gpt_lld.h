/*
    Copyright (C) 2021 Westberry Technology (ChangZhou) Corp., Ltd

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
 * @file    TIMv1/hal_gpt_lld.h
 * @brief   WB32 GPT subsystem low level driver header.
 *
 * @addtogroup GPT
 * @{
 */

#ifndef HAL_GPT_LLD_H
#define HAL_GPT_LLD_H

#include "wb32_tim.h"

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
#if !defined(WB32_GPT_USE_TIM1) || defined(__DOXYGEN__)
#define WB32_GPT_USE_TIM1                      FALSE
#endif

/**
 * @brief   GPTD2 driver enable switch.
 * @details If set to @p TRUE the support for GPTD2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_GPT_USE_TIM2) || defined(__DOXYGEN__)
#define WB32_GPT_USE_TIM2                      FALSE
#endif

/**
 * @brief   GPTD3 driver enable switch.
 * @details If set to @p TRUE the support for GPTD3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_GPT_USE_TIM3) || defined(__DOXYGEN__)
#define WB32_GPT_USE_TIM3                      FALSE
#endif

/**
 * @brief   GPTD4 driver enable switch.
 * @details If set to @p TRUE the support for GPTD4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_GPT_USE_TIM4) || defined(__DOXYGEN__)
#define WB32_GPT_USE_TIM4                      FALSE
#endif

/**
 * @brief   GPTD1 interrupt priority level setting.
 */
#if !defined(WB32_GPT_TIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_GPT_TIM1_IRQ_PRIORITY             7
#endif

/**
 * @brief   GPTD2 interrupt priority level setting.
 */
#if !defined(WB32_GPT_TIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_GPT_TIM2_IRQ_PRIORITY             7
#endif

/**
 * @brief   GPTD3 interrupt priority level setting.
 */
#if !defined(WB32_GPT_TIM3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_GPT_TIM3_IRQ_PRIORITY             7
#endif

/**
 * @brief   GPTD4 interrupt priority level setting.
 */
#if !defined(WB32_GPT_TIM4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_GPT_TIM4_IRQ_PRIORITY             7
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(WB32_HAS_TIM1)
#define WB32_HAS_TIM1                          FALSE
#endif

#if !defined(WB32_HAS_TIM2)
#define WB32_HAS_TIM2                          FALSE
#endif

#if !defined(WB32_HAS_TIM3)
#define WB32_HAS_TIM3                          FALSE
#endif

#if !defined(WB32_HAS_TIM4)
#define WB32_HAS_TIM4                          FALSE
#endif

#if WB32_GPT_USE_TIM1 && !WB32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if WB32_GPT_USE_TIM2 && !WB32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if WB32_GPT_USE_TIM3 && !WB32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if WB32_GPT_USE_TIM4 && !WB32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if !WB32_GPT_USE_TIM1  && !WB32_GPT_USE_TIM2 &&                            \
    !WB32_GPT_USE_TIM3  && !WB32_GPT_USE_TIM4
#error "GPT driver activated but no TIM peripheral assigned"
#endif

/* Checks on allocation of TIMx units.*/
#if WB32_GPT_USE_TIM1
#if defined(WB32_TIM1_IS_USED)
#error "GPTD1 requires TIM1 but the timer is already used"
#else
#define WB32_TIM1_IS_USED
#endif
#endif

#if WB32_GPT_USE_TIM2
#if defined(WB32_TIM2_IS_USED)
#error "GPTD2 requires TIM2 but the timer is already used"
#else
#define WB32_TIM2_IS_USED
#endif
#endif

#if WB32_GPT_USE_TIM3
#if defined(WB32_TIM3_IS_USED)
#error "GPTD3 requires TIM3 but the timer is already used"
#else
#define WB32_TIM3_IS_USED
#endif
#endif

#if WB32_GPT_USE_TIM4
#if defined(WB32_TIM4_IS_USED)
#error "GPTD4 requires TIM4 but the timer is already used"
#else
#define WB32_TIM4_IS_USED
#endif
#endif

/* IRQ priority checks.*/
#if WB32_GPT_USE_TIM1 && !defined(WB32_TIM1_SUPPRESS_ISR) &&                \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_GPT_TIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM1"
#endif

#if WB32_GPT_USE_TIM2 && !defined(WB32_TIM2_SUPPRESS_ISR) &&                \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_GPT_TIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM2"
#endif

#if WB32_GPT_USE_TIM3 && !defined(WB32_TIM3_SUPPRESS_ISR) &&                \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_GPT_TIM3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM3"
#endif

#if WB32_GPT_USE_TIM4 && !defined(WB32_TIM_SUPPRESS_ISR) &&                 \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_GPT_TIM4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM4"
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
  uint32_t                  cr2;
  /**
   * @brief TIM DIER register initialization data.
   * @note  The value of this field should normally be equal to zero.
   * @note  Only the DMA-related bits can be specified in this field.
   */
  uint32_t                  dier;
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
  wb32_tim_t                *tim;
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
  ((gptp)->tim->ARR = (uint32_t)((interval) - 1U))

/**
 * @brief   Returns the interval of GPT peripheral.
 * @pre     The GPT unit must be running in continuous mode.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @return              The current interval.
 *
 * @notapi
 */
#define gpt_lld_get_interval(gptp) ((gptcnt_t)((gptp)->tim->ARR + 1U))

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

#if WB32_GPT_USE_TIM1 && !defined(__DOXYGEN__)
extern GPTDriver GPTD1;
#endif

#if WB32_GPT_USE_TIM2 && !defined(__DOXYGEN__)
extern GPTDriver GPTD2;
#endif

#if WB32_GPT_USE_TIM3 && !defined(__DOXYGEN__)
extern GPTDriver GPTD3;
#endif

#if WB32_GPT_USE_TIM4 && !defined(__DOXYGEN__)
extern GPTDriver GPTD4;
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
