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
 * @file    hal_st.c
 * @brief   ST Driver code.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

#if (ST_LLD_NUM_ALARMS > 1) || defined(__DOXYGEN__)
st_callback_t st_callbacks[ST_LLD_NUM_ALARMS - 1];
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   ST Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void stInit(void) {
#if ST_LLD_NUM_ALARMS > 1
  unsigned i;

  for (i = 0U; i < (unsigned)ST_LLD_NUM_ALARMS - 1U; i++) {
    st_callbacks[i] = NULL;
  }
#endif
  st_lld_init();
}

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) || defined(__DOXYGEN__)
/**
 * @brief   Starts the alarm zero.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] abstime   the time to be set for the first alarm
 *
 * @api
 */
void stStartAlarm(systime_t abstime) {

  osalDbgAssert(stIsAlarmActive() == false, "already active");

  st_lld_start_alarm(abstime);
}

/**
 * @brief   Stops the alarm zero interrupt.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @api
 */
void stStopAlarm(void) {

  st_lld_stop_alarm();
}

/**
 * @brief   Sets the alarm zero time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @api
 */
void stSetAlarm(systime_t abstime) {

  osalDbgAssert(stIsAlarmActive() != false, "not active");

  st_lld_set_alarm(abstime);
}

/**
 * @brief   Returns the alarm zero current time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @return              The currently set alarm time.
 *
 * @api
 */
systime_t stGetAlarm(void) {

  osalDbgAssert(stIsAlarmActive() != false, "not active");

  return st_lld_get_alarm();
}

/**
 * @brief   Returns the time counter value.
 * @note    This functionality is only available in free running mode, the
 *          behaviour in periodic mode is undefined.
 *
 * @return              The counter value.
 *
 * @api
 */
systime_t stGetCounter(void) {

  return st_lld_get_counter();
}

/**
 * @brief   Determines if the alarm zero is active.
 *
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @api
 */
bool stIsAlarmActive(void) {

  return st_lld_is_alarm_active();
}

#if (ST_LLD_NUM_ALARMS > 1) || defined(__DOXYGEN__)
/**
 * @brief   Determines if the specified alarm is active.
 *
 * @param[in] alarm     alarm channel number (1..ST_LLD_NUM_ALARMS)
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @api
 */
bool stIsAlarmActiveN(unsigned alarm) {

  return st_lld_is_alarm_active_n(n);
}

/**
 * @brief   Starts an additional alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] abstime   the time to be set for the first alarm
 * @param[in] alarm     alarm channel number (1..ST_LLD_NUM_ALARMS)
 * @param[in] cb        alarm callback
 *
 * @api
 */
void stStartAlarmN(unsigned alarm, systime_t abstime, st_callback_t cb) {

  osalDbgCheck((alarm > 0U) && (alarm < (unsigned)ST_LLD_NUM_ALARMS));
  osalDbgAssert(stIsAlarmActiveN(alarm) == false, "already active");

  st_callbacks[alarm - 1U] = cb;
  st_lld_start_alarm_n(alarm, abstime);
}

/**
 * @brief   Stops an additional alarm.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number (1..ST_LLD_NUM_ALARMS)
 *
 * @api
 */
void stStopAlarmN(unsigned alarm) {

  osalDbgCheck((alarm > 0U) && (alarm < (unsigned)ST_LLD_NUM_ALARMS));

  st_callbacks[alarm - 1U] = NULL;
  st_lld_stop_alarm_n(alarm);
}

/**
 * @brief   Sets an additional alarm time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number (1..ST_LLD_NUM_ALARMS)
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @api
 */
void stSetAlarmN(unsigned alarm, systime_t abstime) {

  osalDbgCheck((alarm > 0U) && (alarm < (unsigned)ST_LLD_NUM_ALARMS));
  osalDbgAssert(stIsAlarmActiveN(alarm) != false, "not active");

  st_lld_set_alarm_n(alarm, abstime);
}

/**
 * @brief   Returns an additional alarm current time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number (1..ST_LLD_NUM_ALARMS)
 * @return              The currently set alarm time.
 *
 * @api
 */
systime_t stGetAlarmN(unsigned alarm) {

  osalDbgCheck((alarm > 0U) && (alarm < (unsigned)ST_LLD_NUM_ALARMS));
  osalDbgAssert(stIsAlarmActiveN(alarm) != false, "not active");

  return st_lld_get_alarm_n(alarm);
}
#endif /* ST_LLD_NUM_ALARMS > 1 */

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
