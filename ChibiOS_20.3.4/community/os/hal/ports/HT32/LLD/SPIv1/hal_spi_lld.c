/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio
              Copyright (C) 2020 Yaotian Feng

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
 * @file    hal_spi_lld.c
 * @brief   HT32 SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SPI0 driver identifier.
 */
#if (HT32_SPI_USE_SPI0 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID0;
#endif
#if (HT32_SPI_USE_SPI1 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if (HT32_SPI_USE_SPI0 == TRUE) || (HT32_SPI_USE_SPI1 == TRUE) || defined(__DOXYGEN__)
static void spi_lld_rx(SPIDriver * const spip) {
    uint32_t fd;
    uint32_t sr;

    while (spip->rxcnt) {
        sr = spip->SPI->SR;
        if ((sr & SPI_SR_RXBNE) == 0)
            return;
        fd = spip->SPI->DR;
        if (spip->rxptr) {
            *spip->rxptr++ = fd & 0xff;
        }
        spip->rxcnt--;
    }
}

static void spi_lld_tx(SPIDriver * const spip) {
    uint32_t fd;
    uint32_t sr;

    while (spip->txcnt) {
        sr = spip->SPI->SR;
        if ((sr & SPI_SR_TXBE) == 0)
            return;
        if (spip->txptr) {
            fd = *spip->txptr++;
        } else {
            fd = '\xff';
        }
        spip->SPI->DR = fd;
        spip->txcnt--;
    }
}

static void spi_lld_handler(SPIDriver * const spip) {
    //uint32_t sr = spip->SPI->SR; // & ((1U<<8)|spip->SPI->IER);
    spi_lld_rx(spip);
    spi_lld_tx(spip);
    if (spip->rxcnt == 0) {
        spip->SPI->IER = 0;
        _spi_isr_code(spip);
    }
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (HT32_SPI_USE_SPI0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(HT32_SPI0_IRQ_VECTOR) {
    OSAL_IRQ_PROLOGUE();
    spi_lld_handler(&SPID0);
    OSAL_IRQ_EPILOGUE();
}
#endif

#if (HT32_SPI_USE_SPI1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(HT32_SPI1_IRQ_VECTOR) {
    OSAL_IRQ_PROLOGUE();
    spi_lld_handler(&SPID1);
    OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {
    /* Driver initialization.*/
#if HT32_SPI_USE_SPI0 == TRUE
    spiObjectInit(&SPID0);
    SPID0.SPI = SPI0;
//    CKCU->APBCCR0 |= CKCU_APBCCR0_SPI0EN;
#endif
#if HT32_SPI_USE_SPI1 == TRUE
    spiObjectInit(&SPID1);
    SPID1.SPI = SPI1;
//    CKCU->APBCCR0 |= CKCU_APBCCR0_SPI1EN;
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
    if (spip->state == SPI_STOP) {
        /* Enables the peripheral.*/
#if HT32_SPI_USE_SPI0 == TRUE
        if (&SPID0 == spip) {
            CKCU->APBCCR0 |= CKCU_APBCCR0_SPI0EN;
            nvicEnableVector(SPI0_IRQn, HT32_SPI0_IRQ_PRIORITY);
        }
#endif
#if HT32_SPI_USE_SPI1 == TRUE
        if (&SPID1 == spip) {
            CKCU->APBCCR0 |= CKCU_APBCCR0_SPI1EN;
            nvicEnableVector(SPI1_IRQn, HT32_SPI1_IRQ_PRIORITY);
        }
#endif
    }

    /* Configures the peripheral.*/
    spip->SPI->CR0 = spip->config->cr0;
    spip->SPI->CR1 = spip->config->cr1;
    spip->SPI->CPR = spip->config->cpr;
    //spip->SPI->FCR = 0; //SPI_FCR_FIFOEN | (1U << 4) | (1U << 0);
    spip->SPI->FCR = spip->config->fcr;
    spip->SPI->CR0 |= SPI_CR0_SPIEN;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {
    if (spip->state == SPI_READY) {
        /* Disables the peripheral.*/
#if HT32_SPI_USE_SPI0 == TRUE
        if (&SPID0 == spip) {
            RSTCU->APBPRSTR0 = RSTCU_APBPRSTR0_SPI0RST;
            CKCU->APBCCR0 &= ~CKCU_APBCCR0_SPI0EN;
            nvicDisableVector(SPI0_IRQn);
        }
#endif
#if HT32_SPI_USE_SPI1 == TRUE
        if (&SPID1 == spip) {
            RSTCU->APBPRSTR0 = RSTCU_APBPRSTR0_SPI1RST;
            CKCU->APBCCR0 &= ~CKCU_APBCCR0_SPI1EN;
            nvicDisableVector(SPI1_IRQn);
        }
#endif
    }
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {
    spip->SPI->CR0 |= SPI_CR0_SSELC;
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
    spip->SPI->CR0 &= ~SPI_CR0_SSELC;
}

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
    spi_lld_exchange(spip, n, NULL, NULL);
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
    spip->txptr = txbuf;
    spip->rxptr = rxbuf;
    spip->rxcnt = spip->txcnt = n;
    spip->SPI->IER = SPI_IER_RXBNEIEN | SPI_IER_TXBEIEN;
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
    spi_lld_exchange(spip, n, txbuf, NULL);
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
    spi_lld_exchange(spip, n, NULL, rxbuf);
}

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
    spip->SPI->DR = frame;
    while((spip->SPI->SR & SPI_SR_RXBNE) == 0);
    return (spip->SPI->DR & 0xffff);
}

#endif /* HAL_USE_SPI == TRUE */

/** @} */
