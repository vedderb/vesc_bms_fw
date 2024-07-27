/*
    ChibiOS - Copyright (C) 2020 Yaotian Feng / Codetector

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
 * @file    hal_st_lld.h
 * @brief   PLATFORM ST subsystem low level driver header.
 * @details This header is designed to be include-able without having to
 *          include other files from the HAL.
 *
 * @addtogroup ST
 * @{
 */

#ifndef HAL_ST_LLD_H
#define HAL_ST_LLD_H

#include "hal.h"

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
 * @brief   SysTick timer IRQ priority.
 */
#if !defined(LPC_ST_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LPC_ST_IRQ_PRIORITY    8
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if OSAL_ST_MODE_PERIODIC == OSAL_ST_MODE_FREERUNNING
  #error "OSAL_ST_MODE_FREERUNNING is not supported. Check CH_CFG_ST_TIMEDELTA"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void st_lld_init(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Driver inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Returns the time counter value.
 *
 * @return              The counter value.
 *
 * @notapi
 */
static inline systime_t st_lld_get_counter(void) {
  return (systime_t) SysTick->VAL;
}

/**
 * @brief   Starts the alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 *
 * @param[in] abstime   the time to be set for the first alarm
 *
 * @notapi
 */
static inline void st_lld_start_alarm(systime_t abstime) {
  SysTick->LOAD = (uint32_t)abstime;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | // No External Clock
                  SysTick_CTRL_ENABLE_Msk |
                  SysTick_CTRL_TICKINT_Msk;
  SysTick->VAL = 0; // Trigger reload
}

/**
 * @brief   Stops the alarm interrupt.
 *
 * @notapi
 */
static inline void st_lld_stop_alarm(void) {
  SysTick->CTRL &= ~((uint32_t)0b10);
}

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm(systime_t abstime) {
  SysTick->VAL = (uint32_t)abstime;
}

/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm(void) {
  return (systime_t)SysTick->LOAD;
}

/**
 * @brief   Determines if the alarm is active.
 *
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @notapi
 */
static inline bool st_lld_is_alarm_active(void) {
  return ((SysTick->CTRL & 0b10) != 0);
}

#endif /* HAL_ST_LLD_H */

/** @} */
