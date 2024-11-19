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
/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

/**
 * @file    I2Cv1/hal_i2c_lld.h
 * @brief   WB32 I2C subsystem low level driver header.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef HAL_I2C_LLD_H
#define HAL_I2C_LLD_H

#if HAL_USE_I2C || defined(__DOXYGEN__)

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
 * @brief   I2C1 driver enable switch.
 * @details If set to @p TRUE the support for I2C1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_I2C_USE_I2C1) || defined(__DOXYGEN__)
#define WB32_I2C_USE_I2C1                      FALSE
#endif

/**
 * @brief   I2C2 driver enable switch.
 * @details If set to @p TRUE the support for I2C2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_I2C_USE_I2C2) || defined(__DOXYGEN__)
#define WB32_I2C_USE_I2C2                      FALSE
#endif

/**
 * @brief   I2C timeout on busy condition in milliseconds.
 */
#if !defined(WB32_I2C_BUSY_TIMEOUT) || defined(__DOXYGEN__)
#define WB32_I2C_BUSY_TIMEOUT                  50
#endif

/**
 * @brief   I2C1 interrupt priority level setting.
 */
#if !defined(WB32_I2C_I2C1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_I2C_I2C1_IRQ_PRIORITY             10
#endif

/**
 * @brief   I2C2 interrupt priority level setting.
 */
#if !defined(WB32_I2C_I2C2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_I2C_I2C2_IRQ_PRIORITY             10
#endif

#if defined(WB32F3G71xx)
#define WB32F3G71xx_I2C
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*  error checks */
#if WB32_I2C_USE_I2C1 && !WB32_HAS_I2C1
#error "I2C1 not present in the selected device"
#endif

#if WB32_I2C_USE_I2C2 && !WB32_HAS_I2C2
#error "I2C2 not present in the selected device"
#endif

#if !WB32_I2C_USE_I2C1 && !WB32_I2C_USE_I2C2
#error "I2C driver activated but no I2C peripheral assigned"
#endif

#if WB32_I2C_USE_I2C1 &&                                                    \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_I2C_I2C1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to I2C1"
#endif

#if WB32_I2C_USE_I2C2 &&                                                    \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_I2C_I2C2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to I2C2"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type representing an I2C address.
 */
typedef uint16_t i2caddr_t;

/**
 * @brief   Type of I2C driver condition flags.
 */
typedef uint32_t i2cflags_t;

typedef struct {
  const uint8_t             *tx_buf;
  volatile uint32_t         tx_len;
  uint8_t                   *rx_buf;
  volatile uint32_t         rx_len;
  volatile uint32_t         rx_cmd_len;
  uint32_t                  tx_abrt_source;
} i2c_xfer_info_t;

/**
 * @brief   Type of I2C driver configuration structure.
 */
typedef struct {
  /* End of the mandatory fields.*/
  /* specifies the I2C interrupt source.*/
  uint16_t                  i2c_it;
  /* Specifies the I2C mode.*/
  uint32_t                  op_mode;
  /* In host mode set the slave address */
  uint32_t                  target_address;
  /* The Transmit FIFO threshold to set.*/
  uint8_t                   tx_fifo_threshold;
  /* The Receive FIFO threshold to set.*/
  uint8_t                   rx_fifo_threshold;
  /* tHIGH = (ss_scl_hcnt + FS_SPKLEN + 7) / PCLK2.*/
  uint32_t                  ss_scl_hcnt;
  /* tLOW = (ss_scl_lcnt + 1) / PCLK2.*/
  uint32_t                  ss_scl_lcnt;
  /* fs_spklen / PCLK2.*/
  uint32_t                  fs_spklen;
  /* tSU;DAT = sda_setup / PCLK2.*/
  uint32_t                  sda_setup;
  /* tHD;DAT = sda_hold / PCLK2.*/
  uint32_t                  sda_hold;
} I2CConfig;

/**
 * @brief   Type of a structure representing an I2C driver.
 */
typedef struct I2CDriver I2CDriver;

/**
 * @brief   Structure representing an I2C driver.
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
  /* End of the mandatory fields.*/
  /**
   * @brief   Thread waiting for I/O completion.
   */
  thread_reference_t        thread;
  /**
   * @brief   Current slave address without R/W bit.
   */
  i2caddr_t                 addr;
  /**
   * @brief   Data processing.
   */
  i2c_xfer_info_t           xfer;
  /**
   * @brief   Pointer to the I2Cx registers block.
   */
  I2C_TypeDef               *i2c;
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
#if WB32_I2C_USE_I2C1
extern I2CDriver I2CD1;
#endif

#if WB32_I2C_USE_I2C2
extern I2CDriver I2CD2;
#endif

#endif /* !defined(__DOXYGEN__) */

#ifdef __cplusplus
extern "C" {
#endif
  void i2c_lld_init(void);
  void i2c_lld_start(I2CDriver *i2cp);
  void i2c_lld_stop(I2CDriver *i2cp);
  msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                        const uint8_t *txbuf, size_t txbytes,
                                        uint8_t *rxbuf, size_t rxbytes,
                                        sysinterval_t timeout);
  msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                       uint8_t *rxbuf, size_t rxbytes,
                                       sysinterval_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C  */

#endif /* HAL_I2C_LLD_H */

/** @} */
