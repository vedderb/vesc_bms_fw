/*
    ChibiOS - Copyright (C) 2006,2007,2008,2009,2010,2011,2012,2013,2014,
              2015,2016,2017,2018,2019,2020,2021 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    rt/include/chvt.h
 * @brief   Time and Virtual Timers module macros and structures.
 *
 * @addtogroup time
 * @{
 */

#ifndef CHVT_H
#define CHVT_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (CH_CFG_ST_TIMEDELTA < 0) || (CH_CFG_ST_TIMEDELTA == 1)
#error "invalid CH_CFG_ST_TIMEDELTA specified, must "                       \
       "be zero or greater than one"
#endif

#if (CH_CFG_ST_TIMEDELTA > 0) && (CH_CFG_TIME_QUANTUM > 0)
#error "CH_CFG_TIME_QUANTUM not supported in tickless mode"
#endif

#if (CH_CFG_ST_TIMEDELTA > 0) && (CH_DBG_THREADS_PROFILING == TRUE)
#error "CH_DBG_THREADS_PROFILING not supported in tickless mode"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*
 * Virtual Timers APIs.
 */
#ifdef __cplusplus
extern "C" {
#endif
  void _vt_init(void);
  void chVTDoSetI(virtual_timer_t *vtp, sysinterval_t delay,
                  vtfunc_t vtfunc, void *par);
  void chVTDoResetI(virtual_timer_t *vtp);
  void chVTDoTickI(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Initializes a @p virtual_timer_t object.
 * @note    Initializing a timer object is not strictly required because
 *          the function @p chVTSetI() initializes the object too. This
 *          function is only useful if you need to perform a @p chVTIsArmed()
 *          check before calling @p chVTSetI().
 *
 * @param[out] vtp      the @p virtual_timer_t structure pointer
 *
 * @init
 */
static inline void chVTObjectInit(virtual_timer_t *vtp) {

  vtp->func = NULL;
}

/**
 * @brief   Current system time.
 * @details Returns the number of system ticks since the @p chSysInit()
 *          invocation.
 * @note    The counter can reach its maximum and then restart from zero.
 * @note    This function can be called from any context but its atomicity
 *          is not guaranteed on architectures whose word size is less than
 *          @p systime_t size.
 *
 * @return              The system time in ticks.
 *
 * @xclass
 */
static inline systime_t chVTGetSystemTimeX(void) {

#if CH_CFG_ST_TIMEDELTA == 0
  return ch.vtlist.systime;
#else /* CH_CFG_ST_TIMEDELTA > 0 */
  return port_timer_get_time();
#endif /* CH_CFG_ST_TIMEDELTA > 0 */
}

/**
 * @brief   Current system time.
 * @details Returns the number of system ticks since the @p chSysInit()
 *          invocation.
 * @note    The counter can reach its maximum and then restart from zero.
 *
 * @return              The system time in ticks.
 *
 * @api
 */
static inline systime_t chVTGetSystemTime(void) {
  systime_t systime;

  chSysLock();
  systime = chVTGetSystemTimeX();
  chSysUnlock();

  return systime;
}

/**
 * @brief   Returns the elapsed time since the specified start time.
 *
 * @param[in] start     start time
 * @return              The elapsed time.
 *
 * @xclass
 */
static inline sysinterval_t chVTTimeElapsedSinceX(systime_t start) {

  return chTimeDiffX(start, chVTGetSystemTimeX());
}

/**
 * @brief   Checks if the current system time is within the specified time
 *          window.
 * @note    When start==end then the function returns always false because the
 *          time window has zero size.
 *
 * @param[in] start     the start of the time window (inclusive)
 * @param[in] end       the end of the time window (non inclusive)
 * @retval true         current time within the specified time window.
 * @retval false        current time not within the specified time window.
 *
 * @xclass
 */
static inline bool chVTIsSystemTimeWithinX(systime_t start, systime_t end) {

  return chTimeIsInRangeX(chVTGetSystemTimeX(), start, end);
}

/**
 * @brief   Checks if the current system time is within the specified time
 *          window.
 * @note    When start==end then the function returns always false because the
 *          time window has zero size.
 *
 * @param[in] start     the start of the time window (inclusive)
 * @param[in] end       the end of the time window (non inclusive)
 * @retval true         current time within the specified time window.
 * @retval false        current time not within the specified time window.
 *
 * @api
 */
static inline bool chVTIsSystemTimeWithin(systime_t start, systime_t end) {

  return chTimeIsInRangeX(chVTGetSystemTime(), start, end);
}

/**
 * @brief   Returns the time interval until the next timer event.
 * @note    The return value is not perfectly accurate and can report values
 *          in excess of @p CH_CFG_ST_TIMEDELTA ticks.
 * @note    The interval returned by this function is only meaningful if
 *          more timers are not added to the list until the returned time.
 *
 * @param[out] timep    pointer to a variable that will contain the time
 *                      interval until the next timer elapses. This pointer
 *                      can be @p NULL if the information is not required.
 * @return              The time, in ticks, until next time event.
 * @retval false        if the timers list is empty.
 * @retval true         if the timers list contains at least one timer.
 *
 * @iclass
 */
static inline bool chVTGetTimersStateI(sysinterval_t *timep) {
  virtual_timers_list_t *vtlp = &ch.vtlist;
  delta_list_t *dlp = &vtlp->dlist;

  chDbgCheckClassI();

  if (dlp == dlp->next) {
    return false;
  }

  if (timep != NULL) {
#if CH_CFG_ST_TIMEDELTA == 0
    *timep = dlp->next->delta;
#else
    *timep = (dlp->next->delta + (sysinterval_t)CH_CFG_ST_TIMEDELTA) -
             chTimeDiffX(vtlp->lasttime, chVTGetSystemTimeX());
#endif
  }

  return true;
}

/**
 * @brief   Returns @p true if the specified timer is armed.
 * @pre     The timer must have been initialized using @p chVTObjectInit()
 *          or @p chVTDoSetI().
 *
 * @param[in] vtp       the @p virtual_timer_t structure pointer
 * @return              true if the timer is armed.
 *
 * @iclass
 */
static inline bool chVTIsArmedI(const virtual_timer_t *vtp) {

  chDbgCheckClassI();

  return (bool)(vtp->func != NULL);
}

/**
 * @brief   Returns @p true if the specified timer is armed.
 * @pre     The timer must have been initialized using @p chVTObjectInit()
 *          or @p chVTDoSetI().
 *
 * @param[in] vtp       the @p virtual_timer_t structure pointer
 * @return              true if the timer is armed.
 *
 * @api
 */
static inline bool chVTIsArmed(const virtual_timer_t *vtp) {
  bool b;

  chSysLock();
  b = chVTIsArmedI(vtp);
  chSysUnlock();

  return b;
}

/**
 * @brief   Disables a Virtual Timer.
 * @note    The timer is first checked and disabled only if armed.
 * @pre     The timer must have been initialized using @p chVTObjectInit()
 *          or @p chVTDoSetI().
 *
 * @param[in] vtp       the @p virtual_timer_t structure pointer
 *
 * @iclass
 */
static inline void chVTResetI(virtual_timer_t *vtp) {

  if (chVTIsArmedI(vtp)) {
    chVTDoResetI(vtp);
  }
}

/**
 * @brief   Disables a Virtual Timer.
 * @note    The timer is first checked and disabled only if armed.
 * @pre     The timer must have been initialized using @p chVTObjectInit()
 *          or @p chVTDoSetI().
 *
 * @param[in] vtp       the @p virtual_timer_t structure pointer
 *
 * @api
 */
static inline void chVTReset(virtual_timer_t *vtp) {

  chSysLock();
  chVTResetI(vtp);
  chSysUnlock();
}

/**
 * @brief   Enables a virtual timer.
 * @details If the virtual timer was already enabled then it is re-enabled
 *          using the new parameters.
 * @pre     The timer must have been initialized using @p chVTObjectInit()
 *          or @p chVTDoSetI().
 *
 * @param[in] vtp       the @p virtual_timer_t structure pointer
 * @param[in] delay     the number of ticks before the operation timeouts, the
 *                      special values are handled as follow:
 *                      - @a TIME_INFINITE is allowed but interpreted as a
 *                        normal time specification.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 * @param[in] vtfunc    the timer callback function. After invoking the
 *                      callback the timer is disabled and the structure can
 *                      be disposed or reused.
 * @param[in] par       a parameter that will be passed to the callback
 *                      function
 *
 * @iclass
 */
static inline void chVTSetI(virtual_timer_t *vtp, sysinterval_t delay,
                            vtfunc_t vtfunc, void *par) {

  chVTResetI(vtp);
  chVTDoSetI(vtp, delay, vtfunc, par);
}

/**
 * @brief   Enables a virtual timer.
 * @details If the virtual timer was already enabled then it is re-enabled
 *          using the new parameters.
 * @pre     The timer must have been initialized using @p chVTObjectInit()
 *          or @p chVTDoSetI().
 *
 * @param[in] vtp       the @p virtual_timer_t structure pointer
 * @param[in] delay     the number of ticks before the operation timeouts, the
 *                      special values are handled as follow:
 *                      - @a TIME_INFINITE is allowed but interpreted as a
 *                        normal time specification.
 *                      - @a TIME_IMMEDIATE this value is not allowed.
 *                      .
 * @param[in] vtfunc    the timer callback function. After invoking the
 *                      callback the timer is disabled and the structure can
 *                      be disposed or reused.
 * @param[in] par       a parameter that will be passed to the callback
 *                      function
 *
 * @api
 */
static inline void chVTSet(virtual_timer_t *vtp, sysinterval_t delay,
                           vtfunc_t vtfunc, void *par) {

  chSysLock();
  chVTSetI(vtp, delay, vtfunc, par);
  chSysUnlock();
}

#endif /* CHVT_H */

/** @} */
