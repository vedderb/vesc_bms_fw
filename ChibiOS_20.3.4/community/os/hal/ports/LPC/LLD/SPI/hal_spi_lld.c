/*
    ChibiOS - Copyright (C) 2020 Yaotian Feng / Codetector

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
 * @brief   LPC11Uxx SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#define int_enable(SSP)                                                   \
do {(SSP)->IMSC = SSP_INT_RTMIS | SSP_INT_RXMIS | SSP_INT_TXMIS; } while(0)

#define int_disable(SSP)                                                  \
do {(SSP)->IMSC = 0; } while(0)
/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SPI1 driver identifier.
 */
#if (LPC_SPI_USE_SPI1 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void ssp_irq_handler_impl(SPIDriver *spip, LPC_SSPx_Type* ssp) {
  while (ssp->SR & SSP_SR_RxNotEmpty){
    if (spip->rxbuf) {
      if (spip->config->data_size > 8) {
        ((uint16_t*)spip->txbuf)[spip->cnt] = (uint16_t)ssp->DR;
      } else {
        ((uint8_t*)spip->txbuf)[spip->cnt] = (uint8_t)ssp->DR;
      }
    } else {
      (void) ssp->DR;
    }
    spip->cnt++;
    if (spip->cnt < spip->size) {
      if (spip->txbuf) {
        if (spip->config->data_size > 8) {
          LPC_SSP1->DR = ((uint16_t*)spip->txbuf)[spip->cnt];
        } else {
          LPC_SSP1->DR = ((uint8_t*)spip->txbuf)[spip->cnt];
        }
      } else {
        ssp->DR = 0;
      }
    } else {
      break;
    }
  }

  if ((ssp->SR & SSP_SR_BUSY) == 0 && spip->cnt >= spip->size) {
    // Transaction Complete
    int_disable(ssp);
    _spi_isr_code(spip);
  }

  if (ssp->RIS & SSP_INT_ROR) {
    ssp->ICR = SSP_INT_ROR;
  }

  if (ssp->RIS & SSP_INT_RTMIS) {
    ssp->ICR = SSP_INT_RTMIS;
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if LPC_SPI_USE_SPI1 == TRUE

#ifndef LPC_SSP1_IRQ_VECTOR
#error "LPC_SSP1_IRQ_VECTOR not defined"
#endif

OSAL_IRQ_HANDLER(LPC_SSP1_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  ssp_irq_handler_impl(&SPID1, LPC_SSP1);
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

#if LPC_SPI_USE_SPI1 == TRUE
  /* Driver initialization.*/
  spiObjectInit(&SPID1);
  // Enable AHBCLK
  LPC_SYSCON->SYSAHBCLKCTRL |= SYSCON_SYSAHBCLKCTRL_SSP1;
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
#if LPC_SPI_USE_SPI1 == TRUE
    if (&SPID1 == spip) {
      // Set Clock Divider
      LPC_SYSCON->SSP1CLKDIV = (uint32_t) spip->config->clock_divider;
      LPC_SYSCON->PRESETCTRL |= SYSCON_PRESETCTRL_SSP1_RSTn; // Clear Reset

      LPC_SSP1->CR0 = SSP_CR0_DSS(spip->config->data_size - 1) |
        SSP_CR0_SCR(spip->config->clock_rate);
      LPC_SSP1->CPSR = spip->config->clock_prescaler;

      int_disable(LPC_SSP1);
      LPC_SSP1->CR1 = SSP_CR1_SPI_EN; // Run in Master mode

      #if !defined(LPC_SPI_SPI1_IRQ_PRIORITY)
      #error "LPC_SPI_SPI1_IRQ_PRIORITY is not defined"
      #endif
      nvicEnableVector(SSP1_IRQn, LPC_SPI_SPI1_IRQ_PRIORITY);
    }
#endif
  }
  /* Configures the peripheral.*/

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
#if LPC_SPI_USE_SPI1 == TRUE
    if (&SPID1 == spip) {
      LPC_SSP1->CR1 = 0; // Disable SSP1
      LPC_SYSCON->PRESETCTRL &= ~SYSCON_PRESETCTRL_SSP1_RSTn; // Reset SSP1
      nvicDisableVector(SSP1_IRQn);
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
  (void)spip;
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
  (void)spip;
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
  if (n == 0) {
    return;
  }
  spip->txbuf = txbuf;
  spip->rxbuf = rxbuf;
  spip->size = n;
  spip->cnt = 0;
#if LPC_SPI_USE_SPI1 == TRUE
  if (spip == &SPID1) {
    // Wait for SPI IDLE. Empty RX buffer
    while(LPC_SSP1->SR & (SSP_SR_BUSY | SSP_SR_RxNotEmpty)) {
      if (LPC_SSP1->SR) {
        (void)LPC_SSP1->DR;
      }
    }

    // Enable Interrupts
    LPC_SSP1->ICR = 0x3; // Clear Interrutps
    int_enable(LPC_SSP1);
    if (txbuf) {
      if (spip->config->data_size > 8) {
        LPC_SSP1->DR = ((uint16_t*)spip->txbuf)[spip->cnt];
      } else {
        LPC_SSP1->DR = ((uint8_t*)spip->txbuf)[spip->cnt];
      }
    } else {
      LPC_SSP1->DR = 0;
    }
  }
#endif
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

#if (SPI_SUPPORTS_CIRCULAR == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Aborts the ongoing SPI operation, if any.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_abort(SPIDriver *spip) {

  (void)spip;
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
 *
 * @notapi
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

#if LPC_SPI_USE_SPI1 == TRUE
  if (spip == &SPID1) {
    // Wait for SPI IDLE. Empty RX buffer
    while(LPC_SSP1->SR & (SSP_SR_BUSY | SSP_SR_RxNotEmpty)) {
      if (LPC_SSP1->SR) {
        (void)LPC_SSP1->DR;
      }
    }
    LPC_SSP1->DR = frame;
    while((LPC_SSP1->SR & SSP_SR_RxNotEmpty) != 0){} // Wait till RxNotEmpty
    return (uint16_t) LPC_SSP1->DR;
  }
#endif

  return 0;
}

#endif /* HAL_USE_SPI == TRUE */

/** @} */
