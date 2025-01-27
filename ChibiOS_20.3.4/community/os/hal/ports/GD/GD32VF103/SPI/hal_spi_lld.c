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
 * @file    SPI/hal_spi_lld.c
 * @brief   GD32 SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define SPI0_RX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_SPI_SPI0_RX_DMA_STREAM,                        \
                       GD32_SPI0_RX_DMA_CHN)

#define SPI0_TX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_SPI_SPI0_TX_DMA_STREAM,                        \
                       GD32_SPI0_TX_DMA_CHN)

#define SPI1_RX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_SPI_SPI1_RX_DMA_STREAM,                        \
                       GD32_SPI1_RX_DMA_CHN)

#define SPI1_TX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_SPI_SPI1_TX_DMA_STREAM,                        \
                       GD32_SPI1_TX_DMA_CHN)

#define SPI2_RX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_SPI_SPI2_RX_DMA_STREAM,                        \
                       GD32_SPI2_RX_DMA_CHN)

#define SPI2_TX_DMA_CHANNEL                                                 \
  GD32_DMA_GETCHANNEL(GD32_SPI_SPI2_TX_DMA_STREAM,                        \
                       GD32_SPI2_TX_DMA_CHN)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SPI0 driver identifier.*/
#if GD32_SPI_USE_SPI0 || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/** @brief SPI1 driver identifier.*/
#if GD32_SPI_USE_SPI1 || defined(__DOXYGEN__)
SPIDriver SPID2;
#endif

/** @brief SPI2 driver identifier.*/
#if GD32_SPI_USE_SPI2 || defined(__DOXYGEN__)
SPIDriver SPID3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static const uint16_t dummytx = 0xFFFFU;
static uint16_t dummyrx;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared end-of-rx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_rx_interrupt(SPIDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(GD32_SPI_DMA_ERROR_HOOK)
  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    GD32_SPI_DMA_ERROR_HOOK(spip);
  }
#else
  (void)flags;
#endif

  if (spip->config->circular) {
    if ((flags & GD32_DMA_INTF_HTFIF) != 0U) {
      /* Half buffer interrupt.*/
      _spi_isr_half_code(spip);
    }
    if ((flags & GD32_DMA_INTF_FTFIF) != 0U) {
      /* End buffer interrupt.*/
      _spi_isr_full_code(spip);
    }
  }
  else {
    /* Stopping DMAs.*/
    dmaStreamDisable(spip->dmatx);
    dmaStreamDisable(spip->dmarx);

    /* Portable SPI ISR code defined in the high level driver, note, it is
       a macro.*/
    _spi_isr_code(spip);
  }
}

/**
 * @brief   Shared end-of-tx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_tx_interrupt(SPIDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(GD32_SPI_DMA_ERROR_HOOK)
  (void)spip;
  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    GD32_SPI_DMA_ERROR_HOOK(spip);
  }
#else
  (void)spip;
  (void)flags;
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {

#if GD32_SPI_USE_SPI0
  spiObjectInit(&SPID1);
  SPID1.spi       = SPI0;
  SPID1.dmarx     = NULL;
  SPID1.dmatx     = NULL;
  SPID1.rxdmamode = GD32_DMA_CTL_CHSEL(SPI0_RX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_SPI_SPI0_DMA_PRIORITY) |
                    GD32_DMA_CTL_DIR_P2M |
                    GD32_DMA_CTL_FTFIE |
                    
                    GD32_DMA_CTL_ERRIE;
  SPID1.txdmamode = GD32_DMA_CTL_CHSEL(SPI0_TX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_SPI_SPI0_DMA_PRIORITY) |
                    GD32_DMA_CTL_DIR_M2P |
                    
                    GD32_DMA_CTL_ERRIE;
#endif

#if GD32_SPI_USE_SPI1
  spiObjectInit(&SPID2);
  SPID2.spi       = SPI1;
  SPID2.dmarx     = NULL;
  SPID2.dmatx     = NULL;
  SPID2.rxdmamode = GD32_DMA_CTL_CHSEL(SPI1_RX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_SPI_SPI1_DMA_PRIORITY) |
                    GD32_DMA_CTL_DIR_P2M |
                    GD32_DMA_CTL_FTFIE |
                    
                    GD32_DMA_CTL_ERRIE;
  SPID2.txdmamode = GD32_DMA_CTL_CHSEL(SPI1_TX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_SPI_SPI1_DMA_PRIORITY) |
                    GD32_DMA_CTL_DIR_M2P |
                    
                    GD32_DMA_CTL_ERRIE;
#endif

#if GD32_SPI_USE_SPI2
  spiObjectInit(&SPID3);
  SPID3.spi       = SPI2;
  SPID3.dmarx     = NULL;
  SPID3.dmatx     = NULL;
  SPID3.rxdmamode = GD32_DMA_CTL_CHSEL(SPI2_RX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_SPI_SPI2_DMA_PRIORITY) |
                    GD32_DMA_CTL_DIR_P2M |
                    GD32_DMA_CTL_FTFIE |
                    
                    GD32_DMA_CTL_ERRIE;
  SPID3.txdmamode = GD32_DMA_CTL_CHSEL(SPI2_TX_DMA_CHANNEL) |
                    GD32_DMA_CTL_PRIO(GD32_SPI_SPI2_DMA_PRIORITY) |
                    GD32_DMA_CTL_DIR_M2P |
                    
                    GD32_DMA_CTL_ERRIE;
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (spip->state == SPI_STOP) {
#if GD32_SPI_USE_SPI0
    if (&SPID1 == spip) {
      spip->dmarx = dmaStreamAllocI(GD32_SPI_SPI0_RX_DMA_STREAM,
                                    GD32_SPI_SPI0_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaStreamAllocI(GD32_SPI_SPI0_TX_DMA_STREAM,
                                    GD32_SPI_SPI0_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      rcuEnableSPI0(true);
    }
#endif
#if GD32_SPI_USE_SPI1
    if (&SPID2 == spip) {
      spip->dmarx = dmaStreamAllocI(GD32_SPI_SPI1_RX_DMA_STREAM,
                                    GD32_SPI_SPI1_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaStreamAllocI(GD32_SPI_SPI1_TX_DMA_STREAM,
                                    GD32_SPI_SPI1_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      rcuEnableSPI1(true);
    }
#endif
#if GD32_SPI_USE_SPI2
    if (&SPID3 == spip) {
      spip->dmarx = dmaStreamAllocI(GD32_SPI_SPI2_RX_DMA_STREAM,
                                    GD32_SPI_SPI2_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaStreamAllocI(GD32_SPI_SPI2_TX_DMA_STREAM,
                                    GD32_SPI_SPI2_IRQ_PRIORITY,
                                    (gd32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      rcuEnableSPI2(true);
    }
#endif
    /* DMA setup.*/
    dmaStreamSetPeripheral(spip->dmarx, &spip->spi->DATA);
    dmaStreamSetPeripheral(spip->dmatx, &spip->spi->DATA);
  }

  /* Configuration-specific DMA setup.*/
  if ((spip->config->ctl0 & SPI_CTL0_FF16) == 0) {
    /* Frame width is 8 bits or smaller.*/
    spip->rxdmamode = (spip->rxdmamode & ~GD32_DMA_CTL_SIZE_MASK) |
                      GD32_DMA_CTL_PWIDTH_BYTE | GD32_DMA_CTL_MWIDTH_BYTE;
    spip->txdmamode = (spip->txdmamode & ~GD32_DMA_CTL_SIZE_MASK) |
                      GD32_DMA_CTL_PWIDTH_BYTE | GD32_DMA_CTL_MWIDTH_BYTE;
  }
  else {
    /* Frame width is larger than 8 bits.*/
    spip->rxdmamode = (spip->rxdmamode & ~GD32_DMA_CTL_SIZE_MASK) |
                      GD32_DMA_CTL_PWIDTH_HWORD | GD32_DMA_CTL_MWIDTH_HWORD;
    spip->txdmamode = (spip->txdmamode & ~GD32_DMA_CTL_SIZE_MASK) |
                      GD32_DMA_CTL_PWIDTH_HWORD | GD32_DMA_CTL_MWIDTH_HWORD;
  }

  if (spip->config->circular) {
    spip->rxdmamode |= (GD32_DMA_CTL_CMEN | GD32_DMA_CTL_HTFIE);
    spip->txdmamode |= (GD32_DMA_CTL_CMEN | GD32_DMA_CTL_HTFIE);
  }
  else {
    spip->rxdmamode &= ~(GD32_DMA_CTL_CMEN | GD32_DMA_CTL_HTFIE);
    spip->txdmamode &= ~(GD32_DMA_CTL_CMEN | GD32_DMA_CTL_HTFIE);
  }

  /* SPI setup and enable.*/
  spip->spi->CTL0 &= ~SPI_CTL0_SPIEN;
  spip->spi->CTL0  = spip->config->ctl0 | SPI_CTL0_MSTMOD | SPI_CTL0_SWNSSEN |
                    SPI_CTL0_SWNSS;
  spip->spi->CTL1  = spip->config->ctl1 | SPI_CTL1_NSSDRV | SPI_CTL1_DMAREN |
                    SPI_CTL1_DMATEN;
  spip->spi->CTL0 |= SPI_CTL0_SPIEN;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  /* If in ready state then disables the SPI clock.*/
  if (spip->state == SPI_READY) {

    /* SPI disable.*/
    spip->spi->CTL0 &= ~SPI_CTL0_SPIEN;
    spip->spi->CTL0  = 0;
    spip->spi->CTL1  = 0;
    dmaStreamFreeI(spip->dmarx);
    dmaStreamFreeI(spip->dmatx);
    spip->dmarx = NULL;
    spip->dmatx = NULL;

#if GD32_SPI_USE_SPI0
    if (&SPID1 == spip)
      rcuDisableSPI0();
#endif
#if GD32_SPI_USE_SPI1
    if (&SPID2 == spip)
      rcuDisableSPI1();
#endif
#if GD32_SPI_USE_SPI2
    if (&SPID3 == spip)
      rcuDisableSPI2();
#endif
  }
}

#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

  /* No implementation on GD32.*/
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {

  /* No implementation on GD32.*/
}
#endif

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spip, size_t n) {

  osalDbgAssert(n < 65536, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, &dummyrx);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode);

  dmaStreamSetMemory0(spip->dmatx, &dummytx);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  osalDbgAssert(n < 65536, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, rxbuf);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode| GD32_DMA_CTL_MNAGA);

  dmaStreamSetMemory0(spip->dmatx, txbuf);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode | GD32_DMA_CTL_MNAGA);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

  osalDbgAssert(n < 65536, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, &dummyrx);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode);

  dmaStreamSetMemory0(spip->dmatx, txbuf);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode | GD32_DMA_CTL_MNAGA);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {

  osalDbgAssert(n < 65536, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, rxbuf);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode | GD32_DMA_CTL_MNAGA);

  dmaStreamSetMemory0(spip->dmatx, &dummytx);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
}

#if (SPI_SUPPORTS_CIRCULAR == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Aborts the ongoing SPI operation, if any.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_abort(SPIDriver *spip) {

  /* Stopping DMAs.*/
  dmaStreamDisable(spip->dmatx);
  dmaStreamDisable(spip->dmarx);
}
#endif /* SPI_SUPPORTS_CIRCULAR == TRUE */

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

  spip->spi->DATA = frame;
  while ((spip->spi->STAT & SPI_STAT_RBNE) == 0)
    ;
  return spip->spi->DATA;
}

#endif /* HAL_USE_SPI */

/** @} */
