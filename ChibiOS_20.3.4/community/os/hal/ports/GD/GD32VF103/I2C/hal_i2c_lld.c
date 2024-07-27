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
/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

/**
 * @file    I2C/hal_i2c_lld.c
 * @brief   GD32 I2C subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */

#include "hal.h"

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define I2C0_RX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_I2C_I2C0_RX_DMA_STREAM,                        \
                       GD32_I2C0_RX_DMA_CHN)

#define I2C0_TX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_I2C_I2C0_TX_DMA_STREAM,                        \
                       GD32_I2C0_TX_DMA_CHN)

#define I2C1_RX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_I2C_I2C1_RX_DMA_STREAM,                        \
                       GD32_I2C1_RX_DMA_CHN)

#define I2C1_TX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_I2C_I2C1_TX_DMA_STREAM,                        \
                       GD32_I2C1_TX_DMA_CHN)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define I2C_EV5_MASTER_MODE_SELECT                                          \
  ((uint32_t)(((I2C_STAT1_MASTER | I2C_STAT1_I2CBSY) << 16) | I2C_STAT0_SBSEND))

#define I2C_EV5_MASTER_MODE_SELECT_NO_BUSY                                  \
  ((uint32_t)((I2C_STAT1_MASTER << 16) | I2C_STAT0_SBSEND))

#define I2C_EV6_MASTER_TRA_MODE_SELECTED                                    \
  ((uint32_t)(((I2C_STAT1_MASTER | I2C_STAT1_I2CBSY | I2C_STAT1_TR) << 16) |          \
              I2C_STAT0_ADDSEND | I2C_STAT0_TBE))

#define I2C_EV6_MASTER_REC_MODE_SELECTED                                    \
  ((uint32_t)(((I2C_STAT1_MASTER | I2C_STAT1_I2CBSY)<< 16) | I2C_STAT0_ADDSEND))

#define I2C_EV8_2_MASTER_BYTE_TRANSMITTED                                   \
  ((uint32_t)(((I2C_STAT1_MASTER | I2C_STAT1_I2CBSY | I2C_STAT1_TR) << 16) |          \
              I2C_STAT0_BTC | I2C_STAT0_TBE))

#define I2C_EV9_MASTER_ADD10                                                \
  ((uint32_t)(((I2C_STAT1_MASTER | I2C_STAT1_I2CBSY) << 16) | I2C_STAT0_ADD10SEND))

#define I2C_EV_MASK 0x00FFFFFF

#define I2C_ERROR_MASK                                                      \
  ((uint16_t)(I2C_STAT0_BERR | I2C_STAT0_LOSTARB | I2C_STAT0_AERR | I2C_STAT0_OUERR |      \
              I2C_STAT0_PECERR | I2C_STAT0_SMBTO | I2C_STAT0_SMBALT))

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief I2C0 driver identifier.*/
#if GD32_I2C_USE_I2C0 || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/** @brief I2C1 driver identifier.*/
#if GD32_I2C_USE_I2C1 || defined(__DOXYGEN__)
I2CDriver I2CD2;
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

  /* Stops the I2C peripheral.*/
  dp->CTL0 = I2C_CTL0_SRESET;
  dp->CTL0 = 0;
  dp->CTL1 = 0;
  dp->STAT0 = 0;

  /* Stops the associated DMA streams.*/
  dmaStreamDisable(i2cp->dmatx);
  dmaStreamDisable(i2cp->dmarx);
}

/**
 * @brief   Set clock speed.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_set_clock(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  uint16_t regCKCFG, clock_div;
  int32_t clock_speed = i2cp->config->clock_speed;
  i2cdutycycle_t duty = i2cp->config->duty_cycle;

  osalDbgCheck((i2cp != NULL) &&
               (clock_speed > 0) &&
               (clock_speed <= 1000000));

  /* CR2 Configuration.*/
  dp->CTL1 &= (uint16_t)~I2C_CTL1_I2CCLK;
  dp->CTL1 |= (uint16_t)I2C_CLK_FREQ;

  /* CCR Configuration.*/
  regCKCFG = 0;
  clock_div = I2C_CKCFG_CLKC;

  if (clock_speed <= 100000) {
    /* Configure clock_div in standard mode.*/
    osalDbgAssert(duty == STD_DUTY_CYCLE, "invalid standard mode duty cycle");

    /* Standard mode clock_div calculate: Tlow/Thigh = 1/1.*/
    osalDbgAssert((GD32_PCLK1 % (clock_speed * 2)) == 0,
                  "PCLK1 must be divisible without remainder");
    clock_div = (uint16_t)(GD32_PCLK1 / (clock_speed * 2));

    osalDbgAssert(clock_div >= 0x04,
                  "clock divider less then 0x04 not allowed");
    regCKCFG |= (clock_div & I2C_CKCFG_CLKC);

    /* Sets the Maximum Rise Time for standard mode.*/
    dp->RT = I2C_CLK_FREQ + 1;
  }
  else if (clock_speed <= 1000000) {
    /* Configure clock_div in fast mode and fast mode plus.*/
    osalDbgAssert((duty == FAST_DUTY_CYCLE_2) ||
                  (duty == FAST_DUTY_CYCLE_16_9),
                  "invalid fast mode duty cycle");

    if (duty == FAST_DUTY_CYCLE_2) {
      /* Fast mode clock_div calculate: Tlow/Thigh = 2/1.*/
      osalDbgAssert((GD32_PCLK1 % (clock_speed * 3)) == 0,
                    "PCLK1 must be divided without remainder");
      clock_div = (uint16_t)(GD32_PCLK1 / (clock_speed * 3));
    }
    else if (duty == FAST_DUTY_CYCLE_16_9) {
      /* Fast mode clock_div calculate: Tlow/Thigh = 16/9.*/
      osalDbgAssert((GD32_PCLK1 % (clock_speed * 25)) == 0,
                    "PCLK1 must be divided without remainder");
      clock_div = (uint16_t)(GD32_PCLK1 / (clock_speed * 25));
      regCKCFG |= I2C_CKCFG_DTCY;
    }

    osalDbgAssert(clock_div >= 0x01,
                  "clock divider less then 0x04 not allowed");
    regCKCFG |= (I2C_CKCFG_FAST | (clock_div & I2C_CKCFG_CLKC));

    /* Sets the Maximum Rise Time for fast mode.*/
    dp->RT = (I2C_CLK_FREQ * 300 / 1000) + 1;

    if(clock_speed > 400000) {
      /* Enable Fast mode plus */
      dp->FMPCFG = I2C_FMPCFG_FMPEN;
    }
  }

  osalDbgAssert((clock_div <= I2C_CKCFG_CLKC), "the selected clock is too low");

  dp->CKCFG = regCKCFG;
}

/**
 * @brief   Set operation mode of I2C hardware.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_set_opmode(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  i2copmode_t opmode = i2cp->config->op_mode;
  uint16_t regCR1;

  regCR1 = dp->CTL0;
  switch (opmode) {
  case OPMODE_I2C:
    regCR1 &= (uint16_t)~(I2C_CTL0_SMBEN|I2C_CTL0_SMBSEL);
    break;
  case OPMODE_SMBUS_DEVICE:
    regCR1 |= I2C_CTL0_SMBEN;
    regCR1 &= (uint16_t)~(I2C_CTL0_SMBSEL);
    break;
  case OPMODE_SMBUS_HOST:
    regCR1 |= (I2C_CTL0_SMBEN|I2C_CTL0_SMBSEL);
    break;
  }
  dp->CTL0 = regCR1;
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
   uint32_t regSTAT1 = dp->STAT1;
   uint32_t event = dp->STAT0;

  /* Interrupts are disabled just before dmaStreamEnable() because there
     is no need of interrupts until next transaction begin. All the work is
     done by the DMA.*/
  switch (I2C_EV_MASK & (event | (regSTAT1 << 16))) {
  case I2C_EV5_MASTER_MODE_SELECT:
  case I2C_EV5_MASTER_MODE_SELECT_NO_BUSY:
    if ((i2cp->addr >> 8) > 0) {
      /* 10-bit address: 1 1 1 1 0 X X R/W */
      dp->DATA = 0xF0 | (0x6 & (i2cp->addr >> 8)) | (0x1 & i2cp->addr);
    } else {
      dp->DATA = i2cp->addr;
    }
    break;
  case I2C_EV9_MASTER_ADD10:
    /* Set second addr byte (10-bit addressing)*/
    dp->DATA = (0xFF & (i2cp->addr >> 1));
    break;
  case I2C_EV6_MASTER_REC_MODE_SELECTED:
    dp->CTL1 &= ~I2C_CTL1_EVIE;
	  /* Clear address flags before dma enable */
    (void)dp->STAT0;
    (void)dp->STAT1;
    dmaStreamEnable(i2cp->dmarx);
    dp->CTL1 |= I2C_CTL1_DMALST;                 /* Needed in receiver mode. */
    if (dmaStreamGetTransactionSize(i2cp->dmarx) < 2)
      dp->CTL0 &= ~I2C_CTL0_ACKEN;
    break;
  case I2C_EV6_MASTER_TRA_MODE_SELECTED:
    dp->CTL1 &= ~I2C_CTL1_EVIE;
    /* Clear address flags before dma enable */
    (void)dp->STAT0;
    (void)dp->STAT1;
    dmaStreamEnable(i2cp->dmatx);
    break;
  case I2C_EV8_2_MASTER_BYTE_TRANSMITTED:
    /* Catches BTF event after the end of transmission.*/
    (void)dp->DATA; /* clear BTF.*/
    if (dmaStreamGetTransactionSize(i2cp->dmarx) > 0) {
      /* Starts "read after write" operation, LSB = 1 -> receive.*/
      i2cp->addr |= 0x01;
      dp->CTL0 |= I2C_CTL0_START | I2C_CTL0_ACKEN;
      return;
    }
    dp->CTL1 &= ~I2C_CTL1_EVIE;
    dp->CTL0 |= I2C_CTL0_STOP;
    _i2c_wakeup_isr(i2cp);
    break;
  default:
    break;
  }
  if (event & (I2C_STAT0_ADDSEND | I2C_STAT0_ADD10SEND)){
    (void)dp->STAT0;
    (void)dp->STAT1;
  }

  /* Errata 2.4.6 for STM32F40x, Spurious Bus Error detection in Master mode.*/
  if (event & I2C_STAT0_BERR) {
    dp->STAT0 &= ~I2C_STAT0_BERR;
  }
}

/**
 * @brief   DMA RX end IRQ handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 *
 * @notapi
 */
static void i2c_lld_serve_rx_end_irq(I2CDriver *i2cp, uint32_t flags) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* DMA errors handling.*/
#if defined(GD32_I2C_DMA_ERROR_HOOK)
  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    GD32_I2C_DMA_ERROR_HOOK(i2cp);
  }
#else
  (void)flags;
#endif

  dmaStreamDisable(i2cp->dmarx);

  dp->CTL1 &= ~I2C_CTL1_DMALST;
  dp->CTL0 &= ~I2C_CTL0_ACKEN;
  dp->CTL0 |= I2C_CTL0_STOP;
  _i2c_wakeup_isr(i2cp);
}

/**
 * @brief    DMA TX end IRQ handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_serve_tx_end_irq(I2CDriver *i2cp, uint32_t flags) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* DMA errors handling.*/
#if defined(GD32_I2C_DMA_ERROR_HOOK)
  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    GD32_I2C_DMA_ERROR_HOOK(i2cp);
  }
#else
  (void)flags;
#endif

  dmaStreamDisable(i2cp->dmatx);
  /* Enables interrupts to catch BTF event meaning transmission part complete.
     Interrupt handler will decide to generate STOP or to begin receiving part
     of R/W transaction itself.*/
  dp->CTL1 |= I2C_CTL1_EVIE;
}

/**
 * @brief   I2C error handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in]  sr       content of the SR1 register to be decoded
 *
 * @notapi
 */
static void i2c_lld_serve_error_interrupt(I2CDriver *i2cp, uint16_t sr) {

  /* Clears interrupt flags just to be safe.*/
  dmaStreamDisable(i2cp->dmatx);
  dmaStreamDisable(i2cp->dmarx);

  i2cp->errors = I2C_NO_ERROR;

  if (sr & I2C_STAT0_BERR) {                          /* Bus error.           */
    i2cp->errors |= I2C_BUS_ERROR;
    i2cp->i2c->STAT0 &= ~I2C_STAT0_BERR;
  }

  if (sr & I2C_STAT0_LOSTARB)                            /* Arbitration lost.    */
    i2cp->errors |= I2C_ARBITRATION_LOST;

  if (sr & I2C_STAT0_AERR) {                            /* Acknowledge fail.    */
    i2cp->i2c->CTL1 &= ~I2C_CTL1_EVIE;
    i2cp->i2c->CTL0 |= I2C_CTL0_STOP;                 /* Setting stop bit.    */
    i2cp->errors |= I2C_ACK_FAILURE;
  }

  if (sr & I2C_STAT0_OUERR)                             /* Overrun.             */
    i2cp->errors |= I2C_OVERRUN;

  if (sr & I2C_STAT0_SMBTO)                         /* SMBus Timeout.       */
    i2cp->errors |= I2C_TIMEOUT;

  if (sr & I2C_STAT0_PECERR)                          /* PEC error.           */
    i2cp->errors |= I2C_PEC_ERROR;

  if (sr & I2C_STAT0_SMBALT)                        /* SMBus alert.         */
    i2cp->errors |= I2C_SMB_ALERT;

  /* If some error has been identified then sends wakes the waiting thread.*/
  if (i2cp->errors != I2C_NO_ERROR)
    _i2c_wakeup_error_isr(i2cp);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if GD32_I2C_USE_I2C0 || defined(__DOXYGEN__)
/**
 * @brief   I2C0 event interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(GD32_I2C0_EVENT_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD1);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   I2C0 error interrupt handler.
 */
OSAL_IRQ_HANDLER(GD32_I2C0_ERROR_HANDLER) {
  uint16_t sr = I2CD1.i2c->STAT0;

  OSAL_IRQ_PROLOGUE();

  I2CD1.i2c->STAT0 = ~(sr & I2C_ERROR_MASK);
  i2c_lld_serve_error_interrupt(&I2CD1, sr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_I2C_USE_I2C0 */

#if GD32_I2C_USE_I2C1 || defined(__DOXYGEN__)
/**
 * @brief   I2C1 event interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(GD32_I2C1_EVENT_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD2);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   I2C1 error interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(GD32_I2C1_ERROR_HANDLER) {
  uint16_t sr = I2CD2.i2c->STAT0;

  OSAL_IRQ_PROLOGUE();

  I2CD2.i2c->STAT0 = ~(sr & I2C_ERROR_MASK);
  i2c_lld_serve_error_interrupt(&I2CD2, sr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_I2C_USE_I2C1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {

#if GD32_I2C_USE_I2C0
  i2cObjectInit(&I2CD1);
  I2CD1.thread = NULL;
  I2CD1.i2c    = I2C0;
  I2CD1.dmarx  = NULL;
  I2CD1.dmatx  = NULL;
#endif /* GD32_I2C_USE_I2C0 */

#if GD32_I2C_USE_I2C1
  i2cObjectInit(&I2CD2);
  I2CD2.thread = NULL;
  I2CD2.i2c    = I2C1;
  I2CD2.dmarx  = NULL;
  I2CD2.dmatx  = NULL;
#endif /* GD32_I2C_USE_I2C1 */
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

    i2cp->txdmamode = GD32_DMA_CTL_PWIDTH_BYTE | GD32_DMA_CTL_MWIDTH_BYTE |
                      GD32_DMA_CTL_MNAGA       | 
                      GD32_DMA_CTL_ERRIE       | GD32_DMA_CTL_FTFIE |
                      GD32_DMA_CTL_DIR_M2P;
    i2cp->rxdmamode = GD32_DMA_CTL_PWIDTH_BYTE | GD32_DMA_CTL_MWIDTH_BYTE |
                      GD32_DMA_CTL_MNAGA       | 
                      GD32_DMA_CTL_ERRIE       | GD32_DMA_CTL_FTFIE |
                      GD32_DMA_CTL_DIR_P2M;

#if GD32_I2C_USE_I2C0
    if (&I2CD1 == i2cp) {
      rcuResetI2C0();

      i2cp->dmarx = dmaStreamAllocI(GD32_I2C_I2C0_RX_DMA_STREAM,
                                    GD32_I2C_I2C0_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)i2c_lld_serve_rx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmarx != NULL, "unable to allocate stream");
      i2cp->dmatx = dmaStreamAllocI(GD32_I2C_I2C0_TX_DMA_STREAM,
                                    GD32_I2C_I2C0_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)i2c_lld_serve_tx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmatx != NULL, "unable to allocate stream");

      rcuEnableI2C0(true);
      eclicEnableVector(I2C0_EV_IRQn, GD32_I2C_I2C0_IRQ_PRIORITY, GD32_I2C_I2C0_IRQ_TRIGGER);
      eclicEnableVector(I2C0_ER_IRQn, GD32_I2C_I2C0_IRQ_PRIORITY, GD32_I2C_I2C0_IRQ_TRIGGER);

      i2cp->rxdmamode |= GD32_DMA_CTL_CHSEL(I2C0_RX_DMA_CHANNEL) |
                       GD32_DMA_CTL_PRIO(GD32_I2C_I2C0_DMA_PRIORITY);
      i2cp->txdmamode |= GD32_DMA_CTL_CHSEL(I2C0_TX_DMA_CHANNEL) |
                       GD32_DMA_CTL_PRIO(GD32_I2C_I2C0_DMA_PRIORITY);
    }
#endif /* GD32_I2C_USE_I2C0 */

#if GD32_I2C_USE_I2C1
    if (&I2CD2 == i2cp) {
      rcuResetI2C1();

      i2cp->dmarx = dmaStreamAllocI(GD32_I2C_I2C1_RX_DMA_STREAM,
                                    GD32_I2C_I2C1_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)i2c_lld_serve_rx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmarx != NULL, "unable to allocate stream");
      i2cp->dmatx = dmaStreamAllocI(GD32_I2C_I2C1_TX_DMA_STREAM,
                                    GD32_I2C_I2C1_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)i2c_lld_serve_tx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmatx != NULL, "unable to allocate stream");

      rcuEnableI2C1(true);
      eclicEnableVector(I2C0_EV_IRQn, GD32_I2C_I2C1_IRQ_PRIORITY, GD32_I2C_I2C1_IRQ_TRIGGER);
      eclicEnableVector(I2C0_ER_IRQn, GD32_I2C_I2C1_IRQ_PRIORITY, GD32_I2C_I2C1_IRQ_TRIGGER);

      i2cp->rxdmamode |= GD32_DMA_CTL_CHSEL(I2C1_RX_DMA_CHANNEL) |
                       GD32_DMA_CTL_PRIO(GD32_I2C_I2C1_DMA_PRIORITY);
      i2cp->txdmamode |= GD32_DMA_CTL_CHSEL(I2C1_TX_DMA_CHANNEL) |
                       GD32_DMA_CTL_PRIO(GD32_I2C_I2C1_DMA_PRIORITY);
    }
#endif /* GD32_I2C_USE_I2C1 */
  }

  /* I2C registers pointed by the DMA.*/
  dmaStreamSetPeripheral(i2cp->dmarx, &dp->DATA);
  dmaStreamSetPeripheral(i2cp->dmatx, &dp->DATA);

  /* Reset i2c peripheral.*/
  dp->CTL0 = I2C_CTL0_SRESET;
  dp->CTL0 = 0;
  dp->CTL1 = I2C_CTL1_ERRIE | I2C_CTL1_DMAON;
  /* Setup I2C parameters.*/
  i2c_lld_set_clock(i2cp);
  i2c_lld_set_opmode(i2cp);

  /* Ready to go.*/
  dp->CTL0 |= I2C_CTL0_I2CEN;
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  /* If not in stopped state then disables the I2C clock.*/
  if (i2cp->state != I2C_STOP) {

    /* I2C disable.*/
    i2c_lld_abort_operation(i2cp);
    dmaStreamFreeI(i2cp->dmatx);
    dmaStreamFreeI(i2cp->dmarx);
    i2cp->dmatx = NULL;
    i2cp->dmarx = NULL;

#if GD32_I2C_USE_I2C0
    if (&I2CD1 == i2cp) {
      eclicDisableVector(I2C0_EV_IRQn);
      eclicDisableVector(I2C0_ER_IRQn);
      rcuDisableI2C0();
    }
#endif

#if GD32_I2C_USE_I2C1
    if (&I2CD2 == i2cp) {
      eclicDisableVector(I2C1_EV_IRQn);
      eclicDisableVector(I2C1_ER_IRQn);
      rcuDisableI2C1();
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
  I2C_TypeDef *dp = i2cp->i2c;
  systime_t start, end;
  msg_t msg;

  /* Resetting error flags for this transfer.*/
  i2cp->errors = I2C_NO_ERROR;

  /* Initializes driver fields, LSB = 1 -> receive.*/
  i2cp->addr = (addr << 1) | 0x01;

  /* Releases the lock from high level driver.*/
  osalSysUnlock();

  /* RX DMA setup.*/
  dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
  dmaStreamSetMemory0(i2cp->dmarx, rxbuf);
  dmaStreamSetTransactionSize(i2cp->dmarx, rxbytes);

  /* Calculating the time window for the timeout on the busy bus condition.*/
  start = osalOsGetSystemTimeX();
  end = osalTimeAddX(start, OSAL_MS2I(GD32_I2C_BUSY_TIMEOUT));

  /* Waits until BUSY flag is reset or, alternatively, for a timeout
     condition.*/
  while (true) {
    osalSysLock();

    /* If the bus is not busy then the operation can continue, note, the
       loop is exited in the locked state.*/
    if (!(dp->STAT1 & I2C_STAT1_I2CBSY) && !(dp->CTL0 & I2C_CTL0_STOP))
      break;

    /* If the system time went outside the allowed window then a timeout
       condition is returned.*/
    if (!osalTimeIsInRangeX(osalOsGetSystemTimeX(), start, end)) {
      dmaStreamDisable(i2cp->dmarx);
      return MSG_TIMEOUT;
    }

    osalSysUnlock();
  }

  /* Starts the operation.*/
  dp->CTL1 |= I2C_CTL1_EVIE;
  dp->CTL0 |= I2C_CTL0_START | I2C_CTL0_ACKEN;

  /* Waits for the operation completion or a timeout.*/
  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);
  if (msg != MSG_OK) {
    dmaStreamDisable(i2cp->dmarx);
  }

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
  I2C_TypeDef *dp = i2cp->i2c;
  systime_t start, end;
  msg_t msg;

  /* Resetting error flags for this transfer.*/
  i2cp->errors = I2C_NO_ERROR;

  /* Initializes driver fields, LSB = 0 -> transmit.*/
  i2cp->addr = (addr << 1);

  /* Releases the lock from high level driver.*/
  osalSysUnlock();

  /* TX DMA setup.*/
  dmaStreamSetMode(i2cp->dmatx, i2cp->txdmamode);
  dmaStreamSetMemory0(i2cp->dmatx, txbuf);
  dmaStreamSetTransactionSize(i2cp->dmatx, txbytes);

  /* RX DMA setup.*/
  dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
  dmaStreamSetMemory0(i2cp->dmarx, rxbuf);
  dmaStreamSetTransactionSize(i2cp->dmarx, rxbytes);

  /* Calculating the time window for the timeout on the busy bus condition.*/
  start = osalOsGetSystemTimeX();
  end = osalTimeAddX(start, OSAL_MS2I(GD32_I2C_BUSY_TIMEOUT));

  /* Waits until BUSY flag is reset or, alternatively, for a timeout
     condition.*/
  while (true) {
    osalSysLock();

    /* If the bus is not busy then the operation can continue, note, the
       loop is exited in the locked state.*/
    if (!(dp->STAT1 & I2C_STAT1_I2CBSY) && !(dp->CTL0 & I2C_CTL0_STOP))
      break;

    /* If the system time went outside the allowed window then a timeout
       condition is returned.*/
    if (!osalTimeIsInRangeX(osalOsGetSystemTimeX(), start, end)) {
      dmaStreamDisable(i2cp->dmatx);
      dmaStreamDisable(i2cp->dmarx);
      return MSG_TIMEOUT;
    }

    osalSysUnlock();
  }

  /* Starts the operation.*/
  dp->CTL1 |= I2C_CTL1_EVIE;
  dp->CTL0 |= I2C_CTL0_START;

  /* Waits for the operation completion or a timeout.*/
  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);
  if (msg != MSG_OK) {
    dmaStreamDisable(i2cp->dmatx);
    dmaStreamDisable(i2cp->dmarx);
  }

  return msg;
}

#endif /* HAL_USE_I2C */

/** @} */
