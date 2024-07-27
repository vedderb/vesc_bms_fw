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
 * @file    TIM/hal_pwm_lld.h
 * @brief   GD32 PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef HAL_PWM_LLD_H
#define HAL_PWM_LLD_H

#if HAL_USE_PWM || defined(__DOXYGEN__)

#include "gd32_tim.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Number of PWM channels per PWM driver.
 */
#define PWM_CHANNELS                            GD32_TIM_MAX_CHANNELS

/**
 * @name    GD32-specific PWM complementary output mode macros
 * @{
 */
/**
 * @brief   Complementary output modes mask.
 * @note    This is an GD32-specific setting.
 */
#define PWM_COMPLEMENTARY_OUTPUT_MASK           0xF0

/**
 * @brief   Complementary output not driven.
 * @note    This is an GD32-specific setting.
 */
#define PWM_COMPLEMENTARY_OUTPUT_DISABLED       0x00

/**
 * @brief   Complementary output, active is logic level one.
 * @note    This is an GD32-specific setting.
 * @note    This setting is only available if the configuration option
 *          @p GD32_PWM_USE_ADVANCED is set to TRUE and only for advanced
 *          timer TIM0.
 */
#define PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH    0x10

/**
 * @brief   Complementary output, active is logic level zero.
 * @note    This is an GD32-specific setting.
 * @note    This setting is only available if the configuration option
 *          @p GD32_PWM_USE_ADVANCED is set to TRUE and only for advanced
 *          timer TIM0.
 */
#define PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW     0x20
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   If advanced timer features switch.
 * @details If set to @p TRUE the advanced features for TIM0 is
 *          enabled.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_PWM_USE_ADVANCED) || defined(__DOXYGEN__)
#define GD32_PWM_USE_ADVANCED              FALSE
#endif

/**
 * @brief   PWMD1 driver enable switch.
 * @details If set to @p TRUE the support for PWMD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_PWM_USE_TIM0) || defined(__DOXYGEN__)
#define GD32_PWM_USE_TIM0                  FALSE
#endif

/**
 * @brief   PWMD2 driver enable switch.
 * @details If set to @p TRUE the support for PWMD2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_PWM_USE_TIM1) || defined(__DOXYGEN__)
#define GD32_PWM_USE_TIM1                  FALSE
#endif

/**
 * @brief   PWMD3 driver enable switch.
 * @details If set to @p TRUE the support for PWMD3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_PWM_USE_TIM2) || defined(__DOXYGEN__)
#define GD32_PWM_USE_TIM2                  FALSE
#endif

/**
 * @brief   PWMD4 driver enable switch.
 * @details If set to @p TRUE the support for PWMD4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_PWM_USE_TIM3) || defined(__DOXYGEN__)
#define GD32_PWM_USE_TIM3                  FALSE
#endif

/**
 * @brief   PWMD5 driver enable switch.
 * @details If set to @p TRUE the support for PWMD5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_PWM_USE_TIM4) || defined(__DOXYGEN__)
#define GD32_PWM_USE_TIM4                  FALSE
#endif

/**
 * @brief   PWMD1 interrupt priority level setting.
 */
#if !defined(GD32_PWM_TIM0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_PWM_TIM0_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD2 interrupt priority level setting.
 */
#if !defined(GD32_PWM_TIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_PWM_TIM1_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD3 interrupt priority level setting.
 */
#if !defined(GD32_PWM_TIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_PWM_TIM2_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD4 interrupt priority level setting.
 */
#if !defined(GD32_PWM_TIM3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_PWM_TIM3_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD5 interrupt priority level setting.
 */
#if !defined(GD32_PWM_TIM4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_PWM_TIM4_IRQ_PRIORITY         7
#endif

/**
 * @brief   PWMD1 interrupt trigger setting.
 */
#if !defined(GD32_PWM_TIM0_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_PWM_TIM0_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   PWMD2 interrupt trigger setting.
 */
#if !defined(GD32_PWM_TIM1_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_PWM_TIM1_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   PWMD3 interrupt trigger setting.
 */
#if !defined(GD32_PWM_TIM2_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_PWM_TIM2_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   PWMD4 interrupt trigger setting.
 */
#if !defined(GD32_PWM_TIM3_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_PWM_TIM3_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   PWMD5 interrupt trigger setting.
 */
#if !defined(GD32_PWM_TIM4_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_PWM_TIM4_IRQ_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif
/** @} */

/*===========================================================================*/
/* Configuration checks.                                                     */
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

#if GD32_PWM_USE_TIM0 && !GD32_HAS_TIM0
#error "TIM0 not present in the selected device"
#endif

#if GD32_PWM_USE_TIM1 && !GD32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if GD32_PWM_USE_TIM2 && !GD32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if GD32_PWM_USE_TIM3 && !GD32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if GD32_PWM_USE_TIM4 && !GD32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if !GD32_PWM_USE_TIM0  && !GD32_PWM_USE_TIM1  &&                         \
    !GD32_PWM_USE_TIM2  && !GD32_PWM_USE_TIM3  &&                         \
    !GD32_PWM_USE_TIM4  
#error "PWM driver activated but no TIM peripheral assigned"
#endif

#if GD32_PWM_USE_ADVANCED && !GD32_PWM_USE_TIM0 
#error "advanced mode selected but no advanced timer assigned"
#endif

/* Checks on allocation of TIMx units.*/
#if GD32_PWM_USE_TIM0
#if defined(GD32_TIM0_IS_USED)
#error "PWMD1 requires TIM0 but the timer is already used"
#else
#define GD32_TIM0_IS_USED
#endif
#endif

#if GD32_PWM_USE_TIM1
#if defined(GD32_TIM1_IS_USED)
#error "PWMD2 requires TIM1 but the timer is already used"
#else
#define GD32_TIM1_IS_USED
#endif
#endif

#if GD32_PWM_USE_TIM2
#if defined(GD32_TIM2_IS_USED)
#error "PWMD3 requires TIM2 but the timer is already used"
#else
#define GD32_TIM2_IS_USED
#endif
#endif

#if GD32_PWM_USE_TIM3
#if defined(GD32_TIM3_IS_USED)
#error "PWMD4 requires TIM3 but the timer is already used"
#else
#define GD32_TIM3_IS_USED
#endif
#endif

#if GD32_PWM_USE_TIM4
#if defined(GD32_TIM4_IS_USED)
#error "PWMD5 requires TIM4 but the timer is already used"
#else
#define GD32_TIM4_IS_USED
#endif
#endif

/* IRQ priority checks.*/
#if GD32_PWM_USE_TIM0 && !defined(GD32_TIM0_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_PWM_TIM0_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM0"
#endif

#if GD32_PWM_USE_TIM1 && !defined(GD32_TIM1_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_PWM_TIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM1"
#endif

#if GD32_PWM_USE_TIM2 && !defined(GD32_TIM2_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_PWM_TIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM2"
#endif

#if GD32_PWM_USE_TIM3 && !defined(GD32_TIM3_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_PWM_TIM3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM3"
#endif

#if GD32_PWM_USE_TIM4 && !defined(GD32_TIM4_SUPPRESS_ISR) &&              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_PWM_TIM4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM4"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a PWM mode.
 */
typedef uint32_t pwmmode_t;

/**
 * @brief   Type of a PWM channel.
 */
typedef uint8_t pwmchannel_t;

/**
 * @brief   Type of a channels mask.
 */
typedef uint32_t pwmchnmsk_t;

/**
 * @brief   Type of a PWM counter.
 */
typedef uint32_t pwmcnt_t;

/**
 * @brief   Type of a PWM driver channel configuration structure.
 */
typedef struct {
  /**
   * @brief Channel active logic level.
   */
  pwmmode_t                 mode;
  /**
   * @brief Channel callback pointer.
   * @note  This callback is invoked on the channel compare event. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /* End of the mandatory fields.*/
} PWMChannelConfig;

/**
 * @brief   Type of a PWM driver configuration structure.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  uint32_t                  frequency;
  /**
   * @brief   PWM period in ticks.
   * @note    The low level can use assertions in order to catch invalid
   *          period specifications.
   */
  pwmcnt_t                  period;
  /**
   * @brief Periodic callback pointer.
   * @note  This callback is invoked on PWM counter reset. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t             callback;
  /**
   * @brief Channels configurations.
   */
  PWMChannelConfig          channels[PWM_CHANNELS];
  /* End of the mandatory fields.*/
  /**
   * @brief TIM CR2 register initialization data.
   * @note  The value of this field should normally be equal to zero.
   */
  uint32_t                  ctl1;
#if GD32_PWM_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief TIM BDTR (break & dead-time) register initialization data.
   * @note  The value of this field should normally be equal to zero.
   */                                                                     \
   uint32_t                 cchp;
#endif
   /**
    * @brief TIM DIER register initialization data.
    * @note  The value of this field should normally be equal to zero.
    * @note  Only the DMA-related bits can be specified in this field.
    */
   uint32_t                 dmainten;
} PWMConfig;

/**
 * @brief   Structure representing a PWM driver.
 */
struct PWMDriver {
  /**
   * @brief Driver state.
   */
  pwmstate_t                state;
  /**
   * @brief Current driver configuration data.
   */
  const PWMConfig           *config;
  /**
   * @brief   Current PWM period in ticks.
   */
  pwmcnt_t                  period;
  /**
   * @brief   Mask of the enabled channels.
   */
  pwmchnmsk_t               enabled;
  /**
   * @brief   Number of channels in this instance.
   */
  pwmchannel_t              channels;
#if defined(PWM_DRIVER_EXT_FIELDS)
  PWM_DRIVER_EXT_FIELDS
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
 * @brief   Changes the period the PWM peripheral.
 * @details This function changes the period of a PWM unit that has already
 *          been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit period is changed to the new value.
 * @note    The function has effect at the next cycle start.
 * @note    If a period is specified that is shorter than the pulse width
 *          programmed in one of the channels then the behavior is not
 *          guaranteed.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] period    new cycle time in ticks
 *
 * @notapi
 */
#define pwm_lld_change_period(pwmp, period)                                 \
  ((pwmp)->tim->CAR = ((period) - 1))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if GD32_PWM_USE_TIM0 && !defined(__DOXYGEN__)
extern PWMDriver PWMD1;
#endif

#if GD32_PWM_USE_TIM1 && !defined(__DOXYGEN__)
extern PWMDriver PWMD2;
#endif

#if GD32_PWM_USE_TIM2 && !defined(__DOXYGEN__)
extern PWMDriver PWMD3;
#endif

#if GD32_PWM_USE_TIM3 && !defined(__DOXYGEN__)
extern PWMDriver PWMD4;
#endif

#if GD32_PWM_USE_TIM4 && !defined(__DOXYGEN__)
extern PWMDriver PWMD5;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void pwm_lld_init(void);
  void pwm_lld_start(PWMDriver *pwmp);
  void pwm_lld_stop(PWMDriver *pwmp);
  void pwm_lld_enable_channel(PWMDriver *pwmp,
                              pwmchannel_t channel,
                              pwmcnt_t width);
  void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel);
  void pwm_lld_enable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_disable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                           pwmchannel_t channel);
  void pwm_lld_disable_channel_notification(PWMDriver *pwmp,
                                            pwmchannel_t channel);
  void pwm_lld_serve_interrupt(PWMDriver *pwmp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PWM */

#endif /* HAL_PWM_LLD_H */

/** @} */
