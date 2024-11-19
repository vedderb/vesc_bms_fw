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

/**
 * @file    SPI/hal_i2s_lld.c
 * @brief   GD32 I2S subsystem low level driver source.
 *
 * @addtogroup I2S
 * @{
 */

#include "hal.h"

#if HAL_USE_I2S || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define I2S2_RX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_I2S_SPI1_RX_DMA_STREAM,                        \
                       GD32_SPI1_RX_DMA_CHN)

#define I2S2_TX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_I2S_SPI1_TX_DMA_STREAM,                        \
                       GD32_SPI1_TX_DMA_CHN)

#define I2S3_RX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_I2S_SPI2_RX_DMA_STREAM,                        \
                       GD32_SPI2_RX_DMA_CHN)

#define I2S3_TX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_I2S_SPI2_TX_DMA_STREAM,                        \
                       GD32_SPI2_TX_DMA_CHN)

/*
 * Static I2S settings for I2S2.
 */
#if !GD32_I2S_IS_MASTER(GD32_I2S_SPI1_MODE)
#if GD32_I2S_TX_ENABLED(GD32_I2S_SPI1_MODE)
#define GD32_I2S2_CFGR_CFG                 0
#endif
#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI1_MODE)
#define GD32_I2S2_CFGR_CFG                 SPI_I2SCTL_I2SOPMOD_0
#endif
#else /* !GD32_I2S_IS_MASTER(GD32_I2S_SPI1_MODE) */
#if GD32_I2S_TX_ENABLED(GD32_I2S_SPI1_MODE)
#define GD32_I2S2_CFGR_CFG                 SPI_I2SCTL_I2SOPMOD_1
#endif
#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI1_MODE)
#define GD32_I2S2_CFGR_CFG                 (SPI_I2SCTL_I2SOPMOD_1 |         \
                                             SPI_I2SCTL_I2SOPMOD_0)
#endif
#endif /* !GD32_I2S_IS_MASTER(GD32_I2S_SPI1_MODE) */

/*
 * Static I2S settings for I2S3.
 */
#if !GD32_I2S_IS_MASTER(GD32_I2S_SPI2_MODE)
#if GD32_I2S_TX_ENABLED(GD32_I2S_SPI2_MODE)
#define GD32_I2S3_CFGR_CFG                 0
#endif
#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI2_MODE)
#define GD32_I2S3_CFGR_CFG                 SPI_I2SCTL_I2SOPMOD_0
#endif
#else /* !GD32_I2S_IS_MASTER(GD32_I2S_SPI2_MODE) */
#if GD32_I2S_TX_ENABLED(GD32_I2S_SPI2_MODE)
#define GD32_I2S3_CFGR_CFG                 SPI_I2SCTL_I2SOPMOD_1
#endif
#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI2_MODE)
#define GD32_I2S3_CFGR_CFG                 (SPI_I2SCTL_I2SOPMOD_1 |         \
                                             SPI_I2SCTL_I2SOPMOD_0)
#endif
#endif /* !GD32_I2S_IS_MASTER(GD32_I2S_SPI2_MODE) */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief I2S2 driver identifier.*/
#if GD32_I2S_USE_SPI1 || defined(__DOXYGEN__)
I2SDriver I2SD2;
#endif

/** @brief I2S3 driver identifier.*/
#if GD32_I2S_USE_SPI2 || defined(__DOXYGEN__)
I2SDriver I2SD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI1_MODE) ||                            \
    GD32_I2S_RX_ENABLED(GD32_I2S_SPI2_MODE) || defined(__DOXYGEN__)
/**
 * @brief   Shared end-of-rx service routine.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void i2s_lld_serve_rx_interrupt(I2SDriver *i2sp, uint32_t flags) {

  (void)i2sp;

  /* DMA errors handling.*/
#if defined(GD32_I2S_DMA_ERROR_HOOK)
  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    GD32_I2S_DMA_ERROR_HOOK(i2sp);
  }
#endif

  /* Callbacks handling, note it is portable code defined in the high
     level driver.*/
  if ((flags & GD32_DMA_INTF_FTFIF) != 0) {
    /* Transfer complete processing.*/
    _i2s_isr_full_code(i2sp);
  }
  else if ((flags & GD32_DMA_INTF_HTFIF) != 0) {
    /* Half transfer processing.*/
    _i2s_isr_half_code(i2sp);
  }
}
#endif

#if GD32_I2S_TX_ENABLED(GD32_I2S_SPI0_MODE) ||                            \
    GD32_I2S_TX_ENABLED(GD32_I2S_SPI1_MODE) ||                            \
    GD32_I2S_TX_ENABLED(GD32_I2S_SPI2_MODE) || defined(__DOXYGEN__)
/**
 * @brief   Shared end-of-tx service routine.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void i2s_lld_serve_tx_interrupt(I2SDriver *i2sp, uint32_t flags) {

  (void)i2sp;

  /* DMA errors handling.*/
#if defined(GD32_I2S_DMA_ERROR_HOOK)
  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    GD32_I2S_DMA_ERROR_HOOK(i2sp);
  }
#endif

  /* Callbacks handling, note it is portable code defined in the high
     level driver.*/
  if ((flags & GD32_DMA_INTF_FTFIF) != 0) {
    /* Transfer complete processing.*/
    _i2s_isr_full_code(i2sp);
  }
  else if ((flags & GD32_DMA_INTF_HTFIF) != 0) {
    /* Half transfer processing.*/
    _i2s_isr_half_code(i2sp);
  }
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2S driver initialization.
 *
 * @notapi
 */
void i2s_lld_init(void) {

#if GD32_I2S_USE_SPI1
  i2sObjectInit(&I2SD2);
  I2SD2.spi       = SPI1;
  I2SD2.ctl       = GD32_I2S2_CFGR_CFG;
  I2SD2.dmarx     = NULL;
  I2SD2.dmatx     = NULL;
#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI1_MODE)
  I2SD2.rxdmamode = GD32_DMA_CTL_CHSEL(I2S2_RX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_I2S_SPI1_DMA_PRIORITY) |
                    GD32_DMA_CTL_PWIDTH_HWORD |
                    GD32_DMA_CTL_MWIDTH_HWORD |
                    GD32_DMA_CTL_DIR_P2M |
                    GD32_DMA_CTL_MNAGA |
                    GD32_DMA_CTL_CMEN |
                    GD32_DMA_CTL_HTFIE |
                    GD32_DMA_CTL_FTFIE |
                    
                    GD32_DMA_CTL_ERRIE;
#else
  I2SD2.rxdmamode = 0;
#endif
#if GD32_I2S_TX_ENABLED(GD32_I2S_SPI1_MODE)
  I2SD2.txdmamode = GD32_DMA_CTL_CHSEL(I2S2_TX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_I2S_SPI1_DMA_PRIORITY) |
                    GD32_DMA_CTL_PWIDTH_HWORD |
                    GD32_DMA_CTL_MWIDTH_HWORD |
                    GD32_DMA_CTL_DIR_M2P |
                    GD32_DMA_CTL_MNAGA |
                    GD32_DMA_CTL_CMEN |
                    GD32_DMA_CTL_HTFIE |
                    GD32_DMA_CTL_FTFIE |
                    
                    GD32_DMA_CTL_ERRIE;
#else
  I2SD2.txdmamode = 0;
#endif
#endif

#if GD32_I2S_USE_SPI2
  i2sObjectInit(&I2SD3);
  I2SD3.spi       = SPI2;
  I2SD3.ctl       = GD32_I2S3_CFGR_CFG;
  I2SD3.dmarx     = NULL;
  I2SD3.dmatx     = NULL;
#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI2_MODE)
  I2SD3.rxdmamode = GD32_DMA_CTL_CHSEL(I2S3_RX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_I2S_SPI2_DMA_PRIORITY) |
                    GD32_DMA_CTL_PWIDTH_HWORD |
                    GD32_DMA_CTL_MWIDTH_HWORD |
                    GD32_DMA_CTL_DIR_P2M |
                    GD32_DMA_CTL_MNAGA |
                    GD32_DMA_CTL_CMEN |
                    GD32_DMA_CTL_HTFIE |
                    GD32_DMA_CTL_FTFIE |
                    
                    GD32_DMA_CTL_ERRIE;
#else
  I2SD3.rxdmamode = 0;
#endif
#if GD32_I2S_TX_ENABLED(GD32_I2S_SPI2_MODE)
  I2SD3.txdmamode = GD32_DMA_CTL_CHSEL(I2S3_TX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_I2S_SPI2_DMA_PRIORITY) |
                    GD32_DMA_CTL_PWIDTH_HWORD |
                    GD32_DMA_CTL_MWIDTH_HWORD |
                    GD32_DMA_CTL_DIR_M2P |
                    GD32_DMA_CTL_MNAGA |
                    GD32_DMA_CTL_CMEN |
                    GD32_DMA_CTL_HTFIE |
                    GD32_DMA_CTL_FTFIE |
                    
                    GD32_DMA_CTL_ERRIE;
#else
  I2SD3.txdmamode = 0;
#endif
#endif
}

/**
 * @brief   Configures and activates the I2S peripheral.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_start(I2SDriver *i2sp) {

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (i2sp->state == I2S_STOP) {

#if GD32_I2S_USE_SPI1
    if (&I2SD2 == i2sp) {

      /* Enabling I2S unit clock.*/
      rcuEnableSPI1(true);

#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI1_MODE)
      i2sp->dmarx = dmaStreamAllocI(GD32_I2S_SPI1_RX_DMA_STREAM,
                                    GD32_I2S_SPI1_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)i2s_lld_serve_rx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmarx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CTL0 = 0;
      i2sp->spi->CTL1 = SPI_CTL1_DMAREN;
#endif
#if GD32_I2S_TX_ENABLED(GD32_I2S_SPI1_MODE)
      i2sp->dmatx = dmaStreamAllocI(GD32_I2S_SPI1_TX_DMA_STREAM,
                                    GD32_I2S_SPI1_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)i2s_lld_serve_tx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmatx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CTL0 = 0;
      i2sp->spi->CTL1 = SPI_CTL1_DMATEN;
#endif
    }
#endif

#if GD32_I2S_USE_SPI2
    if (&I2SD3 == i2sp) {

      /* Enabling I2S unit clock.*/
      rcuEnableSPI2(true);

#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI2_MODE)
      i2sp->dmarx = dmaStreamAllocI(GD32_I2S_SPI2_RX_DMA_STREAM,
                                    GD32_I2S_SPI2_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)i2s_lld_serve_rx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmarx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CTL0 = 0;
      i2sp->spi->CTL1 = SPI_CTL1_DMAREN;
#endif
#if GD32_I2S_TX_ENABLED(GD32_I2S_SPI2_MODE)
      i2sp->dmatx = dmaStreamAllocI(GD32_I2S_SPI2_TX_DMA_STREAM,
                                    GD32_I2S_SPI2_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)i2s_lld_serve_tx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmatx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CTL0 = 0;
      i2sp->spi->CTL1 = SPI_CTL1_DMATEN;
#endif
    }
#endif
  }

  /* I2S (re)configuration.*/
  i2sp->spi->I2SPSC   = i2sp->config->i2spsc;
  i2sp->spi->I2SCTL = i2sp->config->i2sctl | i2sp->ctl | SPI_I2SCTL_I2SSEL;
}

/**
 * @brief   Deactivates the I2S peripheral.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_stop(I2SDriver *i2sp) {

  /* If in ready state then disables the SPI clock.*/
  if (i2sp->state == I2S_READY) {

    /* SPI disable.*/
    i2sp->spi->CTL1 = 0;
    if (NULL != i2sp->dmarx) {
      dmaStreamFreeI(i2sp->dmarx);
      i2sp->dmarx = NULL;
    }
    if (NULL != i2sp->dmatx) {
      dmaStreamFreeI(i2sp->dmatx);
      i2sp->dmatx = NULL;
    }

#if GD32_I2S_USE_SPI1
    if (&I2SD2 == i2sp)
      rcuDisableSPI1();
#endif

#if GD32_I2S_USE_SPI2
    if (&I2SD3 == i2sp)
      rcuDisableSPI2();
#endif
  }
}

/**
 * @brief   Starts a I2S data exchange.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_start_exchange(I2SDriver *i2sp) {
  size_t size = i2sp->config->size;

  /* In 32 bit modes the DMA has to perform double operations because fetches
     are always performed using 16 bit accesses.
     DATLEN   CHLEN   SIZE
     00 (16)  0 (16)  16
     00 (16)  1 (32)  16
     01 (24)  X       32
     10 (32)  X       32
     11 (NA)  X       NA
     */
  if ((i2sp->config->i2sctl & SPI_I2SCTL_DTLEN) != 0)
    size *= 2;

  /* RX DMA setup.*/
  if (NULL != i2sp->dmarx) {
    dmaStreamSetMode(i2sp->dmarx, i2sp->rxdmamode);
    dmaStreamSetPeripheral(i2sp->dmarx, &i2sp->spi->DATA);
    dmaStreamSetMemory0(i2sp->dmarx, i2sp->config->rx_buffer);
    dmaStreamSetTransactionSize(i2sp->dmarx, size);
    dmaStreamEnable(i2sp->dmarx);
  }

  /* TX DMA setup.*/
  if (NULL != i2sp->dmatx) {
    dmaStreamSetMode(i2sp->dmatx, i2sp->txdmamode);
    dmaStreamSetPeripheral(i2sp->dmatx, &i2sp->spi->DATA);
    dmaStreamSetMemory0(i2sp->dmatx, i2sp->config->tx_buffer);
    dmaStreamSetTransactionSize(i2sp->dmatx, size);
    dmaStreamEnable(i2sp->dmatx);
  }

  /* Starting transfer.*/
  i2sp->spi->I2SCTL |= SPI_I2SCTL_I2SEN;
}

/**
 * @brief   Stops the ongoing data exchange.
 * @details The ongoing data exchange, if any, is stopped, if the driver
 *          was not active the function does nothing.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_stop_exchange(I2SDriver *i2sp) {

  /* Stop TX DMA, if enabled.*/
  if (NULL != i2sp->dmatx) {
    dmaStreamDisable(i2sp->dmatx);

    /* From the RM: To switch off the I2S, by clearing I2SE, it is mandatory
       to wait for TXE = 1 and BSY = 0.*/
    while ((i2sp->spi->STAT & (SPI_STAT_TBE | SPI_STAT_TRANS)) != SPI_STAT_TBE)
      ;

    /* Stop SPI/I2S peripheral.*/
    i2sp->spi->I2SCTL &= ~SPI_I2SCTL_I2SEN;
  }

  /* Stop RX DMA, if enabled then draining the RX DR.*/
  if (NULL != i2sp->dmarx) {
    dmaStreamDisable(i2sp->dmarx);

    /* Waiting for some data to be present in RX DR.*/
    while ((i2sp->spi->STAT & SPI_STAT_RBNE) != SPI_STAT_RBNE)
      ;

    /* Stop SPI/I2S peripheral.*/
    i2sp->spi->I2SCTL &= ~SPI_I2SCTL_I2SEN;

    /* Purging data in DR.*/
    while ((i2sp->spi->STAT & SPI_STAT_RBNE) != 0)
      (void) i2sp->spi->DATA;
  }
}

#endif /* HAL_USE_I2S */

/** @} */
