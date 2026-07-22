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
 * @file    I2Cv1/hal_i2c_lld.c
 * @brief   WB32 I2C subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */

#include "hal.h"

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief I2C1 driver identifier.*/
#if WB32_I2C_USE_I2C1 || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/** @brief I2C2 driver identifier.*/
#if WB32_I2C_USE_I2C2 || defined(__DOXYGEN__)
I2CDriver I2CD2;
#endif

/** @brief I2C3 driver identifier.*/
#if WB32_I2C_USE_I2C3 || defined(__DOXYGEN__)
I2CDriver I2CD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Aborts an I2C transaction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_abort_operation(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* Disable I2c.*/
  dp->ENABLE &= ~I2C_ENABLE_ENABLE;

  /* Disable the selected I2C interrupt all.*/
  dp->INTR_MASK &= ~(0xFFFF);

  /* Clear Combined and Individual Interrupt.*/
  (void)dp->CLR_INTR;

  /* Stops the I2C peripheral.*/
#if WB32_I2C_USE_I2C1
  if (&I2CD1 == i2cp) {
    rccResetI2C1();
  }
#endif

#if WB32_I2C_USE_I2C2
  if (&I2CD2 == i2cp) {
    rccResetI2C2();
  }
#endif
}

/**
 * @brief   i2c peripheral configuration
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_configuration(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  const I2CConfig *cfg = i2cp->config;

  osalDbgCheck((i2cp != NULL) && (dp != NULL) && (cfg != NULL));

  dp->INTR_MASK |= (uint16_t)cfg->i2c_it;

  dp->CON = (uint32_t)cfg->op_mode;

  dp->TAR = (uint32_t)cfg->target_address;

  dp->TX_TL = (uint8_t)cfg->tx_fifo_threshold;
  dp->RX_TL = (uint8_t)cfg->rx_fifo_threshold;

  dp->SS_SCL_HCNT = (uint32_t)cfg->ss_scl_hcnt;
  dp->SS_SCL_LCNT = (uint32_t)cfg->ss_scl_lcnt;
  dp->FS_SPKLEN = (uint32_t)cfg->fs_spklen;
  dp->SDA_SETUP = (uint32_t)cfg->sda_setup;
  dp->SDA_HOLD = (uint32_t)cfg->sda_hold;
}

/**
 * @brief   I2C shared ISR code.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_serve_event_interrupt(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  uint16_t read_rx;
  uint32_t cmd;
  uint32_t tx_limit, rx_limit;

  if ((dp->INTR_STAT & I2C_INTR_TX_ABRT) != RESET) {
    i2cp->xfer.tx_abrt_source = (uint32_t)dp->TX_ABRT_SOURCE;
    /* Disable all interrupt except STOP_DET interrupt.*/
    dp->INTR_MASK = I2C_INTR_STOP_DET;
    goto tx_aborted;
  }

  if ((dp->INTR_STAT & I2C_INTR_RX_FULL) != RESET) {
    while ((dp->STATUS & I2C_STATUS_RFNE) != RESET) {
      read_rx = (uint16_t)(dp->DATA_CMD);
      if (i2cp->xfer.rx_len) {
        *i2cp->xfer.rx_buf = read_rx;
        i2cp->xfer.rx_buf++;
        i2cp->xfer.rx_len--;
      }
    }
  }

  if ((dp->INTR_STAT & I2C_INTR_TX_EMPTY) != RESET) {
    tx_limit = 8 - (uint8_t)(dp->TXFLR);
    rx_limit = 8 - (uint8_t)(dp->RXFLR);
    while ((tx_limit > 0) && (rx_limit > 0)) {
      if ((i2cp->xfer.tx_len +
           i2cp->xfer.rx_cmd_len) == 0) {
        /* Disable TX Empty Interrupt.*/
        dp->INTR_MASK &= ~I2C_INTR_TX_EMPTY;
        break;
      }
      cmd = 0;
      if ((i2cp->xfer.tx_len +
           i2cp->xfer.rx_cmd_len) == 1) {
        cmd |= I2C_DATA_CMD_STOP;
      }
      if (i2cp->xfer.tx_len != 0) {
        dp->DATA_CMD = (cmd | *i2cp->xfer.tx_buf);
        i2cp->xfer.tx_buf++;
        i2cp->xfer.tx_len--;
      }
      else if (i2cp->xfer.rx_cmd_len != 0) {
        dp->DATA_CMD = (cmd | I2C_DATA_CMD_READ);
        i2cp->xfer.rx_cmd_len--;
        rx_limit--;
      }
      tx_limit--;
    }
  }

tx_aborted:
  if ((dp->INTR_STAT & I2C_INTR_STOP_DET) != RESET) {
    /* Clear all interrupt flag.*/
    (void)dp->CLR_INTR;
    /* Disable all interrupt.*/
    dp->INTR_MASK &= ~(0xFFFF);
    if (i2cp->xfer.tx_abrt_source) {
      /* Bus error.*/
      if (i2cp->xfer.tx_abrt_source &
          I2C_TX_ABRT_SOURCE_SLV_ARBLOST) {
        i2cp->errors |= I2C_BUS_ERROR;
      }
      /* Arbitration lost.*/
      if (i2cp->xfer.tx_abrt_source & I2C_TX_ABRT_SOURCE_LOST) {
        i2cp->errors |= I2C_ARBITRATION_LOST;
      }
      /* Acknowledge fail.*/
      if (i2cp->xfer.tx_abrt_source &
          (I2C_TX_ABRT_SOURCE_7B_ADDR_NOACK | I2C_TX_ABRT_SOURCE_10ADDR1_NOACK |
           I2C_TX_ABRT_SOURCE_10ADDR2_NOACK | I2C_TX_ABRT_SOURCE_TXDATA_NOACK |
           I2C_TX_ABRT_SOURCE_GCALL_NOACK | I2C_TX_ABRT_SOURCE_DEVICE_NOACK |
           I2C_TX_ABRT_SOURCE_DEVICE_SLVADDR_NOACK)) {
        i2cp->errors |= I2C_ACK_FAILURE;
      }
      /* Overrun.*/
      if (i2cp->xfer.tx_abrt_source &
          I2C_TX_ABRT_SOURCE_DEVICE_WRITE) {
        i2cp->errors |= I2C_OVERRUN;
      }
      /* SMBus Timeout.*/
      if (i2cp->xfer.tx_abrt_source &
          I2C_TX_ABRT_SOURCE_SDA_STUCK_AT_LOW) {
        i2cp->errors |= I2C_TIMEOUT;
      }
      if (i2cp->xfer.tx_abrt_source != I2C_NO_ERROR) {
        _i2c_wakeup_error_isr(i2cp);
      }
    }
    else {
      _i2c_wakeup_isr(i2cp);
    }
  }
}
/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if WB32_I2C_USE_I2C1 || defined(__DOXYGEN__)

#if defined(WB32_I2C1_IRQ_VECTOR)
/**
 * @brief   I2C1 event interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(WB32_I2C1_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD1);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* WB32_I2C_USE_I2C1 */

#if WB32_I2C_USE_I2C2 || defined(__DOXYGEN__)

#if defined(WB32_I2C2_IRQ_VECTOR)
/**
 * @brief   I2C2 event interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(WB32_I2C2_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD2);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif /* WB32_I2C_USE_I2C2 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {

#if WB32_I2C_USE_I2C1
  i2cObjectInit(&I2CD1);
  I2CD1.thread = NULL;
  I2CD1.i2c = I2C1;
#endif /* WB32_I2C_USE_I2C1 */

#if WB32_I2C_USE_I2C2
  i2cObjectInit(&I2CD2);
  I2CD2.thread = NULL;
  I2CD2.i2c = I2C2;
#endif /* WB32_I2C_USE_I2C2 */
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* If in stopped state then enables the I2C and DMA clocks.*/
  if (i2cp->state == I2C_STOP) {
#if WB32_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      /* I2C1 clock enable.*/
      rccEnableI2C1();
      /* I2C1 DeInit.*/
      rccResetI2C1();

      nvicEnableVector(I2C1_IRQn, WB32_I2C_I2C1_IRQ_PRIORITY);
    }
#endif
#if WB32_I2C_USE_I2C2
    if (&I2CD2 == i2cp) {
      /* I2C2 clock enable.*/
      rccEnableI2C2();
      /* I2C2 DeInit.*/
      rccResetI2C2();

      nvicEnableVector(I2C2_IRQn, WB32_I2C_I2C2_IRQ_PRIORITY);
    }
#endif /* WB32_I2C_USE_I2C2 */
  }

  /* Setup I2C parameters.*/
  i2c_lld_configuration(i2cp);
  (void)dp->CLR_INTR;
  dp->INTR_MASK &= ~(0xFFFF);
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  i2cp->xfer.tx_len = 0;
  i2cp->xfer.rx_len = 0;
  i2cp->xfer.tx_buf = NULL;
  i2cp->xfer.rx_buf = NULL;
  i2cp->xfer.tx_abrt_source = 0;

  /* If not in stopped state then disables the I2C clock.*/
  if (i2cp->state != I2C_STOP) {
    /* I2C disable.*/
    i2c_lld_abort_operation(i2cp);

#if WB32_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      nvicDisableVector(I2C1_IRQn);
      rccDisableI2C1();
    }
#endif

#if WB32_I2C_USE_I2C2
    if (&I2CD2 == i2cp) {
      nvicDisableVector(I2C2_IRQn);
      rccDisableI2C2();
    }
#endif
  }
}

/**
 * @brief   Receives data via the I2C bus as master.
 * @details Number of receiving bytes must be more than 1 on WB32F3G71x.
 *          This is hardware restriction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                     uint8_t *rxbuf, size_t rxbytes,
                                     sysinterval_t timeout) {
  I2C_TypeDef *dp = i2cp->i2c;
  msg_t msg;

#if defined(WB32F3G71xx_I2C)
  osalDbgCheck(rxbytes > 0);
#endif
  /* Resetting error flags for this transfer.*/
  i2cp->errors = I2C_NO_ERROR;

  i2cp->xfer.tx_buf = NULL;
  i2cp->xfer.tx_len = 0;

  i2cp->xfer.rx_buf = rxbuf;
  i2cp->xfer.rx_len = rxbytes;
  i2cp->xfer.rx_cmd_len = rxbytes;

  i2cp->xfer.tx_abrt_source = 0x00;

  /* set slave address.*/
  dp->TAR = (uint32_t)addr;

  /* Enable I2C.*/
  dp->ENABLE |= I2C_ENABLE_ENABLE;

  /* Clear all I2C interrupt flags.*/
  (void)dp->CLR_INTR;
  /* Disable all I2C interrupt.*/
  dp->INTR_MASK &= ~(0xFFFF);
  /* Enable the selected I2C interrupt.*/
  dp->INTR_MASK |= (i2cp->config->i2c_it);

  /* Waits for the operation completion or a timeout.*/
  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);
  if (msg == MSG_OK) {
    /* Clear all I2C interrupt flags.*/
    (void)dp->CLR_INTR;
    /* Disable all I2C interrupt.*/
    dp->INTR_MASK &= ~(0xFFFF);
    /* Disable I2C.*/
    dp->ENABLE &= ~I2C_ENABLE_ENABLE;
  }

  return msg;
}

/**
 * @brief   Transmits data via the I2C bus as master.
 * @details Number of receiving bytes must be 0 or more than 1 on WB32F3G71x.
 *          This is hardware restriction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[in] txbuf     pointer to the transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      sysinterval_t timeout) {

  I2C_TypeDef *dp = i2cp->i2c;
  __IO msg_t msg;

#if defined(WB32F3G71xx_I2C)
  osalDbgCheck((rxbytes == 0) || ((rxbytes > 0) && (rxbuf != NULL)));
#endif
  /* Resetting error flags for this transfer.*/
  i2cp->errors = I2C_NO_ERROR;

  i2cp->xfer.tx_buf = txbuf;
  i2cp->xfer.tx_len = txbytes;

  i2cp->xfer.rx_buf = rxbuf;
  i2cp->xfer.rx_len = rxbytes;
  i2cp->xfer.rx_cmd_len = rxbytes;

  i2cp->xfer.tx_abrt_source = 0x00;

  /* set slave address.*/
  dp->TAR = (uint32_t)addr;

  /* Enable I2C.*/
  dp->ENABLE |= I2C_ENABLE_ENABLE;

  /* Clear all I2C interrupt flags.*/
  (void)dp->CLR_INTR;
  /* Disable all I2C interrupt.*/
  dp->INTR_MASK &= ~(0xFFFF);
  /* Enable the selected I2C interrupt.*/
  dp->INTR_MASK |= (i2cp->config->i2c_it);

  /* Waits for the operation completion or a timeout.*/
  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);

  /* Clear all I2C interrupt flags.*/
  (void)dp->CLR_INTR;
  /* Disable all I2C interrupt.*/
  dp->INTR_MASK &= ~(0xFFFF);
  /* Disable I2C.*/
  dp->ENABLE &= ~I2C_ENABLE_ENABLE;

  return msg;
}

#endif /* HAL_USE_I2C */

/** @} */
