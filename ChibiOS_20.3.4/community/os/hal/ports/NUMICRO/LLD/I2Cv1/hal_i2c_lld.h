/*
    ChibiOS - Copyright (C) 2019 Ein Terakawa
               Copyright (C) 2014-2015 Fabio Utzig

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
 * @file    NUMICRO/hal_i2c_lld.h
 * @brief   NUMICRO I2C subsystem low level driver header.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef HAL_I2C_LLD_H_
#define HAL_I2C_LLD_H_

#include "hal.h"
#include "NUC123.h"

#if !defined(NUC123_I2C_USE_I2C0)
#define NUC123_I2C_USE_I2C0 FALSE
#endif

#if !defined(NUC123_I2C_USE_I2C1)
#define NUC123_I2C_USE_I2C1 FALSE
#endif

#define NUMICRO_HAS_I2C0 NUC123_HAS_I2C0
#define NUMICRO_HAS_I2C1 NUC123_HAS_I2C1
#define NUMICRO_I2C_USE_I2C0 NUC123_I2C_USE_I2C0
#define NUMICRO_I2C_USE_I2C1 NUC123_I2C_USE_I2C1


/**
 * @name    Wakeup status codes
 * @{
 */
#define MSG_OK              (msg_t)0    /**< @brief Normal wakeup message.  */
#define MSG_TIMEOUT         (msg_t)-1   /**< @brief Wakeup caused by a timeout
                                             condition.                     */
#define MSG_RESET           (msg_t)-2   /**< @brief Wakeup caused by a reset
                                             condition.                     */
#define MSG_UNDEFINED       (msg_t)-3

typedef int32_t             msg_t;          /**< Inter-thread message.      */

#if !defined(CH_CFG_ST_FREQUENCY)
#define CH_CFG_ST_FREQUENCY                 1000
#endif
#define MS2ST(msec)                                                         \
  ((systime_t)(((((uint32_t)(msec)) *                                       \
                 ((uint32_t)CH_CFG_ST_FREQUENCY)) + 999UL) / 1000UL))
typedef uint32_t systime_t;

#define NUMICRO_I2C0_IRQ_VECTOR NUC123_I2C0_HANDLER
#define NUMICRO_I2C0_IRQ_NUMBER NUC123_I2C0_NUMBER
#define NUMICRO_I2C1_IRQ_VECTOR NUC123_I2C1_HANDLER
#define NUMICRO_I2C1_IRQ_NUMBER NUC123_I2C1_NUMBER

#define I2C_PCLK SystemCoreClock /* Assume APBDIV == 0 */
// #define I2C_PCLK (SystemCoreClock/2) /* Assume APBDIV == 1 */

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define STATE_STOP    0x00
#define STATE_SEND    0x01
#define STATE_RECV    0x02

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   I2C0 driver enable switch.
 * @details If set to @p TRUE the support for I2C0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(NUMICRO_I2C_USE_I2C0) || defined(__DOXYGEN__)
#define NUMICRO_I2C_USE_I2C0               FALSE
#endif

/**
 * @brief   I2C1 driver enable switch.
 * @details If set to @p TRUE the support for I2C1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(NUMICRO_I2C_USE_I2C1) || defined(__DOXYGEN__)
#define NUMICRO_I2C_USE_I2C1               FALSE
#endif
/** @} */

/**
 * @brief   I2C0 interrupt priority level setting.
 */
#if !defined(NUMICRO_I2C_I2C0_PRIORITY) || defined(__DOXYGEN__)
#define NUMICRO_I2C_I2C0_PRIORITY        2
#endif

/**
 * @brief   I2C1 interrupt priority level setting.
 */
#if !defined(NUMICRO_I2C_I2C1_PRIORITY) || defined(__DOXYGEN__)
#define NUMICRO_I2C_I2C1_PRIORITY        2
#endif

/**
 * @brief   Timeout for external clearing BUSY bus (in ms).
 */
#if !defined(NUMICRO_I2C_BUSY_TIMEOUT) || defined(__DOXYGEN__)
#define NUMICRO_I2C_BUSY_TIMEOUT 50
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/** @brief  error checks */
#if NUMICRO_I2C_USE_I2C0 && !NUMICRO_HAS_I2C0
#error "I2C0 not present in the selected device"
#endif

#if NUMICRO_I2C_USE_I2C1 && !NUMICRO_HAS_I2C1
#error "I2C1 not present in the selected device"
#endif


#if !(NUMICRO_I2C_USE_I2C0 || NUMICRO_I2C_USE_I2C1)
#error "I2C driver activated but no I2C peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/* @brief Type representing I2C address. */
typedef uint8_t i2caddr_t;

/* @brief Type of I2C Driver condition flags. */
typedef uint32_t i2cflags_t;

/* @brief Type used to control the ISR state machine. */
typedef uint8_t intstate_t;

/**
 * @brief   Driver configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */

/**
 * @brief Driver configuration structure.
 */
typedef struct I2CConfig {

  /* @brief Clock to be used for the I2C bus. */
  uint32_t             clock;

} I2CConfig;

/**
 * @brief   Type of a structure representing an I2C driver.
 */
typedef struct I2CDriver I2CDriver;

typedef I2C_T I2C_TypeDef;

/**
 * @brief Structure representing an I2C driver.
 */
struct I2CDriver {
  /**
   * @brief   Driver state.
   */
  i2cstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const I2CConfig           *config;
  /**
   * @brief   Error flags.
   */
  i2cflags_t                errors;
#if I2C_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the bus.
   */
  mutex_t                   mutex;
#endif /* I2C_USE_MUTUAL_EXCLUSION */
#if defined(I2C_DRIVER_EXT_FIELDS)
  I2C_DRIVER_EXT_FIELDS
#endif
  /* @brief Thread waiting for I/O completion. */
  thread_reference_t        thread;
  /* @brief     Current slave address without R/W bit. */
  i2caddr_t                 addr;

  /* End of the mandatory fields.*/

  /* @brief Pointer to the buffer with data to send. */
  const uint8_t             *txbuf;
  /* @brief Number of bytes of data to send. */
  size_t                    txbytes;
  /* @brief Current index in buffer when sending data. */
  size_t                    txidx;
  /* @brief Pointer to the buffer to put received data. */
  uint8_t                   *rxbuf;
  /* @brief Number of bytes of data to receive. */
  size_t                    rxbytes;
  /* @brief Current index in buffer when receiving data. */
  size_t                    rxidx;
  /* @brief Tracks current ISR state. */
  intstate_t                intstate;
  /* @brief Low-level register access. */
  I2C_TypeDef               *i2c;
  /* @brief If in master state or not. */
  uint8_t                   is_master;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Get errors from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_errors(i2cp) ((i2cp)->errors)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)

#if NUMICRO_I2C_USE_I2C0
extern I2CDriver I2CD0;
#endif

#if NUMICRO_I2C_USE_I2C1
extern I2CDriver I2CD1;
#endif

#endif

#ifdef __cplusplus
extern "C" {
#endif
  void i2c_lld_init(void);
  void i2c_lld_start(I2CDriver *i2cp);
  void i2c_lld_stop(I2CDriver *i2cp);
  msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                        const uint8_t *txbuf, size_t txbytes,
                                        uint8_t *rxbuf, size_t rxbytes,
                                        systime_t timeout);
  msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                       uint8_t *rxbuf, size_t rxbytes,
                                       systime_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C */

#endif /* HAL_I2C_LLD_H_ */

/** @} */
