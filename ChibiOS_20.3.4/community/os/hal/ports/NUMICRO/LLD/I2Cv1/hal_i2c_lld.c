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
 * @file    NUMICRO/hal_i2c_lld.c
 * @brief   NUMICRO I2C subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */

#include "hal.h"

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define I2C_I2CON_STA I2C_I2CON_STA_Msk
#define I2C_I2CON_STO I2C_I2CON_STO_Msk
#define I2C_I2CON_SI I2C_I2CON_SI_Msk
#define I2C_I2CON_AA I2C_I2CON_AA_Msk
#define I2C_I2CON_STA_STO_SI_AA (I2C_I2CON_STA|I2C_I2CON_STO|I2C_I2CON_SI|I2C_I2CON_AA)
#define I2C_Trigger(i2c, sta, sto, si, ack) ((i2c)->I2CON = (((i2c)->I2CON & ~(I2C_I2CON_STA_STO_SI_AA)) | ((sta)?(I2C_I2CON_STA):0) | ((sto)?(I2C_I2CON_STO):0) | ((si)?(I2C_I2CON_SI):0) | ((ack)?(I2C_I2CON_AA):0)))

#define I2C_START(i2c) ((i2c)->I2CON = ((i2c)->I2CON | I2C_I2CON_SI) | I2C_I2CON_STA)
#define I2C_STOP(i2c) ((i2c)->I2CON = ((i2c)->I2CON | I2C_I2CON_SI) | I2C_I2CON_STO)
#define I2C_ENABLE(i2c) ((i2c)->I2CON = ((i2c)->I2CON | I2C_I2CON_ENS1_Msk))
#define I2C_DISABLE(i2c) ((i2c)->I2CON = ((i2c)->I2CON & ~I2C_I2CON_ENS1_Msk))
#define I2C_EnableInt(i2c) ((i2c)->I2CON = ((i2c)->I2CON | I2C_I2CON_EI_Msk))
#define I2C_DisableInt(i2c) ((i2c)->I2CON = ((i2c)->I2CON & ~I2C_I2CON_EI_Msk))
#define I2C_ClearTimeoutFlag(i2c) ((i2c)->I2CTOC = ((i2c)->I2CTOC | I2C_I2CTOC_TIF_Msk))
#define I2C_GET_STATUS(i2c) ((i2c)->I2CSTATUS)
#define I2C_SET_DATA(i2c, u8dat) ((i2c)->I2CDAT = (u8dat))
#define I2C_GET_DATA(i2c) ((i2c)->I2CDAT)
#define I2C_GET_TIMEOUT_FLAG(i2c) (((i2c)->I2CTOC & I2C_I2CTOC_TIF_Msk)?1:0)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   I2C0 driver identifier.
 */
#if NUMICRO_I2C_USE_I2C0 || defined(__DOXYGEN__)
I2CDriver I2CD0;
#endif

/**
 * @brief   I2C1 driver identifier.
 */
#if NUMICRO_I2C_USE_I2C1 || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void i2c_config_frequency(I2CDriver *i2cp) {

  uint8_t divisor;

  if (i2cp->config != NULL)
    divisor = (I2C_PCLK * 10 / (i2cp->config->clock * 4) + 5) / 10 - 1;
  else
    divisor = I2C_PCLK / 4 / 100000 - 1;

  i2cp->i2c->I2CLK = divisor;
}


/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *          after.
 *
 * @param[in] i2cp         pointer to an I2CDriver
 */
static void serve_interrupt(I2CDriver *i2cp) {

  I2C_TypeDef *i2c = i2cp->i2c;

  if (I2C_GET_TIMEOUT_FLAG(i2c)) {
    i2cp->errors |= I2C_TIMEOUT;
    I2C_ClearTimeoutFlag(i2c);
    I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 0, /* SI */ 1, /* Ack */ 0);
    return;
  }

  uint8_t status = (uint8_t)I2C_GET_STATUS(i2c);

  if (status == 0x08 || status == 0x10) {
    if (i2cp->intstate == STATE_STOP) {
      I2C_DisableInt(i2c);
      return;
    }
    i2cp->is_master = 1;
  }

  /* check if we're master or slave */
  if (i2cp->is_master) {
    /* master */

    if (status == 0x38 || status == 0x68 || status == 0x78 || status == 0xB0) {
      /* check if we lost arbitration */
      i2cp->errors |= I2C_ARBITRATION_LOST;
      I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 0, /* SI */ 1, /* Ack */ 0);
      /* TODO: may need to do more here, reset bus? */
      /* Perhaps clear MST? */
      i2cp->is_master = 0;
    }
    else if (status == 0x08 || status == 0x10) {
      uint8_t op = (i2cp->intstate == STATE_SEND) ? 0 : 1;
      I2C_SET_DATA(i2c, (i2cp->addr << 1) | op);
      I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 0, /* SI */ 1, /* Ack */ 0);
    }
    else if (status == 0x18 || status == 0x28) {
      /* ACK */
      if (i2cp->txbuf != NULL && i2cp->txidx < i2cp->txbytes) {
            /* slave ACK'd and we want to send more */
          I2C_SET_DATA(i2c, i2cp->txbuf[i2cp->txidx++]);
          I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 0, /* SI */ 1, /* Ack */ 0);
      } else {
        /* slave ACK'd and we are done sending */
        if (i2cp->rxbuf != NULL && i2cp->rxbytes != 0) {
          i2cp->intstate = STATE_RECV;
          I2C_START(i2c); // should we use Repeat-START ?
        } else {
          i2cp->intstate = STATE_STOP;
          I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 1, /* SI */ 1, /* Ack */ 0);
          /* this wakes up the waiting thread at the end of ISR */
          _i2c_wakeup_isr(i2cp);
        }
      }
    }
    else if (status == 0x20 || status == 0x30) {
      /* NAK */
      i2cp->errors |= I2C_ACK_FAILURE;
      i2cp->intstate = STATE_STOP;
      I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 1, /* SI */ 1, /* Ack */ 0); // go to Repeated-START state
    }
    else if (status == 0x40) {
      /* ACK to read request */
#if 1
      if (i2cp->rxidx + 1 == i2cp->rxbytes) {
        /* only one byte to receive */
        I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 0, /* SI */ 1, /* Ack */ 0);
      } else {
        I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 0, /* SI */ 1, /* Ack */ 1);
      }
#endif
    }
    else if (status == 0x50 || status == 0x58) {
      /* one byte done */
      if (i2cp->rxbuf == NULL || i2cp->rxidx >= i2cp->rxbytes) {
        /* this is unexpected. */
        i2cp->errors |= I2C_OVERRUN; // is this the apropriate error code?
        i2cp->intstate = STATE_STOP;
        I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 1, /* SI */ 1, /* Ack */ 0);
      }
      else {
        i2cp->rxbuf[i2cp->rxidx++] = I2C_GET_DATA(i2c);
        if (i2cp->rxidx == i2cp->rxbytes) {
          /* last byte done */
          i2cp->intstate = STATE_STOP;
          I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 1, /* SI */ 1, /* Ack */ 0);
_i2c_wakeup_isr(i2cp);
        }
        else if (i2cp->rxidx + 1 == i2cp->rxbytes) {
          /* next byte is the last */
          I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 0, /* SI */ 1, /* Ack */ 0);
        }
        else {
          I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 0, /* SI */ 1, /* Ack */ 1);
        }
      }
    }
    else if (status == 0x48) {
      /* NAK to read request */
      i2cp->errors |= I2C_ACK_FAILURE;
      i2cp->intstate = STATE_STOP;
      I2C_Trigger(i2c, /* Start */ 0, /* Stop */ 1, /* SI */ 1, /* Ack */ 0);
    }
    else if (status == 0xF8) {
      /* CAUTION! I2CSTATUS == 0xF8 won't trigger interrupt! */
      /* everything completed */
      // i2cp->is_master = 0;
      // i2cp->intstate = STATE_STOP;
    }
    else {
      /* need anything else? */
    }
  } else {
    /* slave */

    /* Not implemented yet */
  }

  /* Reset other interrupt sources */

  /* Reset interrupt flag */

  if (i2cp->errors != I2C_NO_ERROR)
    _i2c_wakeup_error_isr(i2cp);
  else if (i2cp->intstate == STATE_STOP && i2cp->is_master == 0)
    _i2c_wakeup_isr(i2cp);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if NUMICRO_I2C_USE_I2C0 || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(NUMICRO_I2C0_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&I2CD0);
  OSAL_IRQ_EPILOGUE();
}

#endif

#if NUMICRO_I2C_USE_I2C1 || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(NUMICRO_I2C1_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&I2CD1);
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

#if NUMICRO_I2C_USE_I2C0
  i2cObjectInit(&I2CD0);
  I2CD0.thread = NULL;
  I2CD0.i2c = I2C0;
  I2CD0.is_master = 0;
  /* Select I2C0 Pins PF.2 and PF.3 as I2C0_SDA and I2C0_SCL*/
#ifdef NUC123xxxANx
  SYS->GPF_MFP |= 0x000C;
  SYS->ALT_MFP1 = (SYS->ALT_MFP1 & 0xF0FFFFFF) | 0x0A000000;
#endif
#ifdef NUC123xxxAEx
  SYS->GPF_MFPL = (SYS->GPF_MFPL & 0xFFFF00FF) | 0x00002200;
#endif
#endif

#if NUMICRO_I2C_USE_I2C1
  i2cObjectInit(&I2CD1);
  I2CD1.thread = NULL;
  I2CD1.i2c = I2C1;
  I2CD1.is_master = 0;
  /* Select I2C1 Pins PA.10 and PA.11 as I2C1_SDA and I2C1_SCL */
#ifdef NUC123xxxANx
  SYS->GPA_MFP |= 0x0C00;
  SYS->ALT_MFP = (SYS->ALT_MFP & 0xFFFFE7FF) | 0x00000000;
#endif
#ifdef NUC123xxxAEx
  SYS->GPA_MFPH = (SYS->GPA_MFPH & 0xFFFF00FF) | 0x00001100;
#endif
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
  /* We are already covered with osalSysLock() which is same as
   *  __disable_irq(). We don't need farther protection by osalSysDisable().
   */

  if (i2cp->state == I2C_STOP) {

  /* TODO:
   *   The PORT must be enabled somewhere. The PIN multiplexer
   *   will map the I2C functionality to some PORT which must
   *   then be enabled. The easier way is enabling all PORTs at
   *   startup, which is currently being done in __early_init.
   */
#if NUMICRO_I2C_USE_I2C0
    if (&I2CD0 == i2cp) {
      /* Eable I2C0 Module Clock */
      CLK->APBCLK |= CLK_APBCLK_I2C0_EN_Msk;
      /* Reset I2C0 Module */
      SYS->IPRSTC2 |= SYS_IPRSTC2_I2C0_RST_Msk;
      SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;
      nvicEnableVector(NUMICRO_I2C0_IRQ_NUMBER, NUMICRO_I2C_I2C0_PRIORITY);
    }
#endif

#if NUMICRO_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      /* Eable I2C1 Module Clock */
      CLK->APBCLK |= CLK_APBCLK_I2C1_EN_Msk;
      /* Reset I2C1 Module */
      SYS->IPRSTC2 |= SYS_IPRSTC2_I2C1_RST_Msk;
      SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C1_RST_Msk;
      nvicEnableVector(NUMICRO_I2C1_IRQ_NUMBER, NUMICRO_I2C_I2C1_PRIORITY);
    }
#endif

    // uint32_t bus_clock = i2cp->config ? i2cp->config->clock : 100000;
    // I2C_Open(i2cp->i2c, bus_clock);
    i2c_config_frequency(i2cp);
    /* Enable I2Cx Module */
    I2C_ENABLE(i2cp->i2c);
    /* Enable Interrupt for I2Cx */
    I2C_EnableInt(i2cp->i2c);
    i2cp->intstate = STATE_STOP; // internal state
  }
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {
  /* We are already covered with osalSysLock() which is same as
   *  __disable_irq(). We don't need farther protection by osalSysDisable().
   */

  if (i2cp->state != I2C_STOP) {
    /* Disable Interrupt for I2Cx */
    I2C_DisableInt(i2cp->i2c);
#if NUMICRO_I2C_USE_I2C0
    if (&I2CD0 == i2cp) {
      /* Reset I2C0 Module */
      SYS->IPRSTC2 |= SYS_IPRSTC2_I2C0_RST_Msk;
      SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;
    }
#endif

#if NUMICRO_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      /* Reset I2C1 Module */
      SYS->IPRSTC2 |= SYS_IPRSTC2_I2C1_RST_Msk;
      SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C1_RST_Msk;
    }
#endif

    /* Disable I2Cx Module */
    I2C_DISABLE(i2cp->i2c);

#if NUMICRO_I2C_USE_I2C0
    if (&I2CD0 == i2cp) {
      nvicDisableVector(NUMICRO_I2C0_IRQ_NUMBER);
      /* Disable I2C0 Module Clock */
      CLK->APBCLK &= ~CLK_APBCLK_I2C0_EN_Msk;
    }
#endif

#if NUMICRO_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      nvicDisableVector(NUMICRO_I2C1_IRQ_NUMBER);
      /* Disable I2C1 Module Clock */
      CLK->APBCLK &= ~CLK_APBCLK_I2C1_EN_Msk;
    }
#endif
  }
}

static inline msg_t _i2c_txrx_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      systime_t timeout) {

  msg_t msg;
  // systime_t start, end;

  i2cp->errors = I2C_NO_ERROR;
  i2cp->addr = addr;

  i2cp->txbuf = txbuf;
  i2cp->txbytes = txbytes;
  i2cp->txidx = 0;

  i2cp->rxbuf = rxbuf;
  i2cp->rxbytes = rxbytes;
  i2cp->rxidx = 0;

  /* clear status flags */

  /* acquire the bus */
  /* check to see if we already have the bus */
  if(i2cp->is_master) {

    /* send repeated start */
    I2C_START(i2cp->i2c);
    while((uint8_t)I2C_GET_STATUS(i2cp->i2c) == 0xF8);

  } else {
    /* unlock during the wait, so that tasks with
     * higher priority can get attention */
    // osalSysUnlock();

    /* wait until the bus is released */
    /* Calculating the time window for the timeout on the busy bus condition.*/
    // start = osalOsGetSystemTimeX();
    // end = start + OSAL_MS2ST(NUMICRO_I2C_BUSY_TIMEOUT);
    uint8_t status = (uint8_t)(I2C_GET_STATUS(i2cp->i2c));
    if (status == 0x08 || status == 0x10) {
      i2cp->is_master = 1;
      I2C_EnableInt(i2cp->i2c);
    } else {
      I2C_Trigger(i2cp->i2c, /* Start */ 1, /* Stop */ 0, /* SI */ 1, /* Ack */ 0);
    }
  }

  /* wait for the ISR to signal that the transmission (or receive if no transmission) phase is complete */

  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);
  if (msg == MSG_TIMEOUT) {
    /* What to do here? */
    // I2C_DisableInt(i2cp->i2c);
    return msg;
  }
  i2cp->is_master = 0;

  I2C_ClearTimeoutFlag(i2cp->i2c);

  return msg;
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
                                     systime_t timeout) {

  i2cp->intstate = STATE_RECV;
  return _i2c_txrx_timeout(i2cp, addr, NULL, 0, rxbuf, rxbytes, timeout);
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
                                      systime_t timeout) {

  i2cp->intstate = STATE_SEND;
  return _i2c_txrx_timeout(i2cp, addr, txbuf, txbytes, rxbuf, rxbytes, timeout);
}

#endif /* HAL_USE_I2C */

/** @} */
