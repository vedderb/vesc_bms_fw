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
 * @file    hal_i2c_lld.c
 * @brief   PLATFORM I2C subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */

#include "hal.h"

#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   I2C1 driver identifier.
 */
#if (RP_I2C_USE_I2C0 == TRUE) || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/**
 * @brief   I2C2 driver identifier.
 */
#if (RP_I2C_USE_I2C1 == TRUE) || defined(__DOXYGEN__)
I2CDriver I2CD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief    Calculates and set up clock settings.
 */
static void setup_frequency(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  //                [us]    SS   FS  FS+
  //  MIN_SCL_LOWtime_FS:  4.7  1.3  0.5
  // MIN_SCL_HIGHtime_FS:  4.0  0.6  0.26
  //             max tSP:    -  0.05 0.05
  //         min tSU:DAT: 0.25  0.1  0.05
  //             tHD:DAT:  0    0    0

  uint32_t minLowfs, minHighfs;
  uint32_t baudrate = i2cp->config->baudrate;
  if (baudrate <= 100000) {
    // ns
    minLowfs = 4700;
    minHighfs = 4000;
  } else if (baudrate <= 400000) {
    minLowfs = 1300;
    minHighfs = 600;
  } else {
    minLowfs = 500;
    minHighfs = 260;
  }

  uint32_t sys_clk = halClockGetPointX(clk_sys) / 100000;
  //uint32_t lcntfs = (minLowfs * sys_clk) / 10000 + 1;
  uint32_t hcntfs = (minHighfs * sys_clk) / 10000 + 1;
  uint32_t lcntfs = (minLowfs * hcntfs) / ((10000000 / baudrate) * 100 - minLowfs) + 1;

  uint32_t sdahd;
  if (baudrate < 1000000) {
    //sdahd = (sys_clk * 300) * (1/1e9) ;
    sdahd = (sys_clk * 3) / 100 + 1;
  } else {
    //sdahd = (sys_clk * 120) * (1/1e9);
    sdahd = (sys_clk * 3) / 250 + 1;
  }

  dp->ENABLE = 0;

  /* Always Fast Mode */
  dp->FSSCLHCNT = hcntfs - 7;
  dp->FSSCLLCNT = lcntfs - 1;
  dp->FSSPKLEN = 2;

  dp->SDAHOLD |= (sdahd << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_Pos) & I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD;

  //dp->ENABLE = 1;
}

/**
 * @brief   Interrupt processing.
 */
static void serve_interrupt(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;

  if (i2cp->state == I2C_ACTIVE_TX) {
    /* Transmission phase. */
    if (dp->INTRSTAT & I2C_IC_INTR_STAT_R_TX_EMPTY_Msk) {
      if (i2cp->txbytes) {
        dp->DATACMD = (uint32_t)i2cp->txbuf[i2cp->txindex] |
                      (i2cp->txbytes == 1 ? I2C_IC_DATA_CMD_STOP : 0);
        i2cp->txindex++;
        i2cp->txbytes--;

        if (i2cp->txbytes == 0U) {
          // Disable TX_EMPTY irq
          dp->CLR.CON = I2C_IC_CON_TX_EMPTY_CTRL;
          dp->CLR.INTRMASK = I2C_IC_INTR_MASK_M_TX_EMPTY;
        }
        return;
      }
    }
  } else {
    /* Receive phase. */
    if (dp->INTRSTAT & I2C_IC_INTR_STAT_R_RX_FULL_Msk) {
      /* Read out received data. */
      i2cp->rxbuf[i2cp->rxindex] = (uint8_t)dp->DATACMD;
      i2cp->rxindex++;
      i2cp->rxbytes--;
      if (i2cp->rxbytes > 0) {
        // Set to master read
        dp->DATACMD = I2C_IC_DATA_CMD_CMD |
                      (i2cp->rxbytes == 1 ? I2C_IC_DATA_CMD_STOP : 0);
        return;
      }
    }
  }

  if (dp->INTRSTAT & I2C_IC_INTR_STAT_R_STOP_DET) {
    /* Clear irq flag. */
    (void)dp->CLRSTOPDET;
    if (i2cp->state == I2C_ACTIVE_TX) {
      /* TX end. */
      if (i2cp->rxbytes > 0U) {
        /* Setup RX. */
        /* Set interrupt mask. */
        dp->INTRMASK = I2C_IC_INTR_MASK_M_STOP_DET |
                       I2C_IC_INTR_MASK_M_TX_ABRT |
                       I2C_IC_INTR_MASK_M_RX_FULL;

        /* Set read flag. */
        dp->DATACMD = I2C_IC_DATA_CMD_CMD |
                      I2C_IC_CON_IC_RESTART_EN |
                      (i2cp->rxbytes == 1 ? I2C_IC_DATA_CMD_STOP : 0);

        /* State change to RX. */
        i2cp->state = I2C_ACTIVE_RX;
        return;
      }
    } else {
      /* RX end. */
      dp->CLR.INTRMASK = I2C_IC_INTR_MASK_M_RX_FULL;
    }
  }

  if (dp->INTRSTAT & I2C_IC_INTR_STAT_R_TX_ABRT_Msk) {
    // Reason, dp->TXABRTSOURCE
    i2cp->errors = dp->TXABRTSOURCE;

    _i2c_wakeup_error_isr(i2cp);
  }

  /* Clear interrupt flags. */
  (void)dp->CLRINTR;

  _i2c_wakeup_isr(i2cp);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (RP_I2C_USE_I2C0 == TRUE) || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(RP_I2C0_IRQ_HANDLER) {

  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&I2CD1);
  OSAL_IRQ_EPILOGUE();
}

#endif

#if (RP_I2C_USE_I2C1 == TRUE) || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(RP_I2C1_IRQ_HANDLER) {

  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&I2CD2);
  OSAL_IRQ_EPILOGUE();
}

#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {

#if RP_I2C_USE_I2C0 == TRUE
  i2cObjectInit(&I2CD1);
  I2CD1.i2c = I2C0;
  I2CD1.thread = NULL;

  /* Reset I2C */
  hal_lld_peripheral_reset(RESETS_ALLREG_I2C0);
#endif

#if RP_I2C_USE_I2C1 == TRUE
  i2cObjectInit(&I2CD2);
  I2CD2.i2c = I2C1;
  I2CD2.thread = NULL;

  /* Reset I2C */
  hal_lld_peripheral_reset(RESETS_ALLREG_I2C1);
#endif
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

  if (i2cp->state == I2C_STOP) {

#if RP_I2C_USE_I2C0 == TRUE
    if (&I2CD1 == i2cp) {
      hal_lld_peripheral_unreset(RESETS_ALLREG_I2C0);

      nvicEnableVector(RP_I2C0_IRQ_NUMBER, RP_IRQ_I2C0_PRIORITY);
    }
#endif

#if RP_I2C_USE_I2C1 == TRUE
    if (&I2CD2 == i2cp) {
      hal_lld_peripheral_unreset(RESETS_ALLREG_I2C1);

      nvicEnableVector(RP_I2C1_IRQ_NUMBER, RP_IRQ_I2C1_PRIORITY);
    }
#endif
  }

  /* Disable for setup. */
  dp->ENABLE = 0;

  dp->CON = I2C_IC_CON_IC_SLAVE_DISABLE |
            I2C_IC_CON_IC_RESTART_EN |
#if RP_I2C_ADDRESS_MODE_10BIT == TRUE
            I2C_IC_CON_IC_10BITADDR_MASTER |
#endif
            (2U << I2C_IC_CON_SPEED_Pos) | // Always Fast Mode
            I2C_IC_CON_MASTER_MODE;

  dp->RXTL = 0;
  dp->TXTL = 0;

  setup_frequency(i2cp);
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  if (i2cp->state != I2C_STOP) {

#if RP_I2C_USE_I2C0 == TRUE
    if (&I2CD1 == i2cp) {
      nvicDisableVector(RP_I2C0_IRQ_NUMBER);

      hal_lld_peripheral_reset(RESETS_ALLREG_I2C0);
    }
#endif

#if RP_I2C_USE_I2C1 == TRUE
    if (&I2CD2 == i2cp) {
      nvicDisableVector(RP_I2C1_IRQ_NUMBER);

      hal_lld_peripheral_reset(RESETS_ALLREG_I2C1);
    }
#endif

  }
}

/**
 * @brief   Receives data via the I2C bus as master.
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

  msg_t msg;
  systime_t start, end;
  I2C_TypeDef *dp = i2cp->i2c;

  /* Releases the lock from high level driver.*/
  osalSysUnlock();

  /* Calculating the time window for the timeout on the busy bus condition.*/
  start = osalOsGetSystemTimeX();
  end = osalTimeAddX(start, OSAL_MS2I(RP_I2C_BUSY_TIMEOUT));

  while (true) {
    osalSysLock();

    if ((dp->STATUS & I2C_IC_STATUS_ACTIVITY) == 0) {
      break;
    }

    /* If the system time went outside the allowed window then a timeout
       condition is returned.*/
    if (!osalTimeIsInRangeX(osalOsGetSystemTimeX(), start, end)) {
      return MSG_TIMEOUT;
    }

    osalSysUnlock();
  }

  i2cp->txbytes = 0U;
  i2cp->rxbytes = rxbytes;
  i2cp->txindex = 0U;
  i2cp->rxindex = 0U;
  i2cp->txbuf = NULL;
  i2cp->rxbuf = rxbuf;

  dp->ENABLE = 0;

  /* Set address. */
  dp->TAR = addr & I2C_IC_TAR_IC_TAR_Msk;

  /* Set interrupt mask, active low. */
  dp->INTRMASK = I2C_IC_INTR_MASK_M_STOP_DET |
                 I2C_IC_INTR_MASK_M_TX_ABRT |
                 I2C_IC_INTR_MASK_M_RX_FULL;

  dp->ENABLE = 1;

  /* Read flag. */
  dp->DATACMD = I2C_IC_DATA_CMD_CMD |
                (i2cp->rxbytes == 1 ? I2C_IC_DATA_CMD_STOP : 0);

  /* Waits for the operation completion or a timeout.*/
  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);

  /* In case of a software timeout a STOP is sent as an extreme attempt
     to release the bus and DMA is forcibly disabled.*/
  if (msg == MSG_TIMEOUT) {
    /* Clear irq flags. */
    (void)dp->CLRINTR;
  }

  dp->ENABLE = 0;

  return msg;
}

/**
 * @brief   Transmits data via the I2C bus as master.
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

  msg_t msg;
  systime_t start, end;
  I2C_TypeDef *dp = i2cp->i2c;

  /* Releases the lock from high level driver.*/
  osalSysUnlock();

  /* Calculating the time window for the timeout on the busy bus condition.*/
  start = osalOsGetSystemTimeX();
  end = osalTimeAddX(start, OSAL_MS2I(RP_I2C_BUSY_TIMEOUT));

  while (true) {
    osalSysLock();

    if ((dp->STATUS & I2C_IC_STATUS_ACTIVITY) == 0) {
      break;
    }

    /* If the system time went outside the allowed window then a timeout
       condition is returned.*/
    if (!osalTimeIsInRangeX(osalOsGetSystemTimeX(), start, end)) {
      return MSG_TIMEOUT;
    }

    osalSysUnlock();
  }

  i2cp->txbytes = txbytes;
  i2cp->rxbytes = rxbytes;
  i2cp->txindex = 0U;
  i2cp->rxindex = 0U;
  i2cp->txbuf = txbuf;
  i2cp->rxbuf = rxbuf;

  dp->ENABLE = 0;

  /* Set target address. */
  dp->TAR = addr & I2C_IC_TAR_IC_TAR_Msk;

  /* TX_EMPTY irq active. */
  dp->CON |= I2C_IC_CON_TX_EMPTY_CTRL;

  /* Set interrupt mask. */
  dp->INTRMASK = I2C_IC_INTR_MASK_M_STOP_DET |
                 I2C_IC_INTR_MASK_M_TX_ABRT |
                 I2C_IC_INTR_MASK_M_TX_EMPTY;

  dp->ENABLE = 1;

  /* Waits for the operation completion or a timeout.*/
  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);

  if (msg == MSG_TIMEOUT) {
    /* Clear irq flags. */
    (void)dp->CLRINTR;
  }

  dp->ENABLE = 0;

  return msg;
}

#endif /* HAL_USE_I2C == TRUE */

/** @} */
