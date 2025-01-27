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
 * @file    rt/include/chtm.h
 * @brief   Time Measurement module macros and structures.
 *
 * @addtogroup time_measurement
 * @{
 */

#ifndef CHTM_H
#define CHTM_H

#if (CH_CFG_USE_TM == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if PORT_SUPPORTS_RT == FALSE
#error "CH_CFG_USE_TM requires PORT_SUPPORTS_RT"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a time measurement calibration data.
 */
typedef struct {
  /**
   * @brief   Measurement calibration value.
   */
  rtcnt_t               offset;
} tm_calibration_t;

/**
 * @brief   Type of a Time Measurement object.
 * @note    The maximum measurable time period depends on the implementation
 *          of the realtime counter and its clock frequency.
 * @note    The measurement is not 100% cycle-accurate, it can be in excess
 *          of few cycles depending on the compiler and target architecture.
 * @note    Interrupts can affect measurement if the measurement is performed
 *          with interrupts enabled.
 */
typedef struct {
  rtcnt_t               best;           /**< @brief Best measurement.       */
  rtcnt_t               worst;          /**< @brief Worst measurement.      */
  rtcnt_t               last;           /**< @brief Last measurement.       */
  ucnt_t                n;              /**< @brief Number of measurements. */
  rttime_t              cumulative;     /**< @brief Cumulative measurement. */
} time_measurement_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void _tm_init(void);
  void chTMObjectInit(time_measurement_t *tmp);
  NOINLINE void chTMStartMeasurementX(time_measurement_t *tmp);
  NOINLINE void chTMStopMeasurementX(time_measurement_t *tmp);
  NOINLINE void chTMChainMeasurementToX(time_measurement_t *tmp1,
                                        time_measurement_t *tmp2);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* CH_CFG_USE_TM == TRUE */

#endif /* CHTM_H */

/** @} */
