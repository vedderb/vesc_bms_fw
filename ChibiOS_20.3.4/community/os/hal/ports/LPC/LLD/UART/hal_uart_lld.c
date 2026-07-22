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
 * @file    hal_uart_lld.c
 * @brief   LPC11Uxx UART subsystem low level driver source.
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"

#if (HAL_USE_UART == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   UART1 driver identifier.
 */
#if (LPC_UART_USE_UART1 == TRUE) || defined(__DOXYGEN__)
UARTDriver UARTD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#define uart_enter_rx_idle_loop(uartp)                             (void)uartp

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if LPC_UART_USE_UART1 == TRUE

#ifndef LPC_UART_IRQ_VECTOR
#error "LPC_UART_IRQ_VECTOR not defined"
#endif

OSAL_IRQ_HANDLER(LPC_UART_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();

  // UARTDriver *uartp = &UARTD1;
  // // Pending Recieve


  // while ((uartp->rx_cnt < uartp->rx_size) &&
  //        (LPC_USART->LSR & USART_LSR_RDR))
  // {
  //   uint8_t tmp = LPC_USART->RBR;
  //   if (uartp->rxbuf != NULL) {
  //     uartp->rxbuf[uartp->rx_cnt] = tmp;
  //   }
  //   uartp->rx_cnt++;
  // }

  // if (uartp->rx_size > 0 && uartp->rx_cnt >= uartp->rx_size) {
  //   uartp->rx_cnt = 0;
  //   uartp->rx_size = 0;
  //   LPC_USART->IER &= ~USART_IER_RBRINTEN; // Disable Rx Interrupt
  //   _uart_rx_complete_isr_code(uartp);
  // }

  // // Check if a pending transimit
  // while ((uartp->txbuf != NULL) && // have a buffer to Tx
  //       (uartp->tx_cnt < uartp->tx_size) && // More bytes needs to be sent
  //       (LPC_USART->LSR & USART_LSR_THRE)) // have space in buffer
  // {
  //   LPC_USART->THR = uartp->txbuf[uartp->tx_cnt];
  //   uartp->tx_cnt++;
  // }

  // if (uartp->tx_size > 0 && uartp->tx_cnt >= uartp->tx_size) {
  //   uartp->tx_cnt = 0;
  //   uartp->tx_size = 0;
  //   uartp->txbuf = NULL;
  //   LPC_USART->IER &= ~USART_IER_THRINTEN; // Disable Tx Interrupt
  //   _uart_tx1_isr_code(uartp);
  // }

  OSAL_IRQ_EPILOGUE();
}

#endif


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {

#if LPC_UART_USE_UART1 == TRUE
  /* Driver initialization.*/
  uartObjectInit(&UARTD1);
  /* Initialize Clock */
  LPC_SYSCON->UARTCLKDIV = LPC_UART_PCLK_DIV; // Set Proper Clock Divider
  LPC_SYSCON->SYSAHBCLKCTRL |= SYSCON_SYSAHBCLKCTRL_USART; // Enable USART Clock
#endif
}

/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_start(UARTDriver *uartp) {

  if (uartp->state == UART_STOP) {
    /* Enables the peripheral.*/
#if LPC_UART_USE_UART1 == TRUE
    if (&UARTD1 == uartp) {
      // Disable Auto Baudrate. Not helpful
      LPC_USART->ACR = 0;
      LPC_USART->TER = USART_TER_TXEN;
      /* Clock should be set to 16x baudrate */
      uint16_t uart_div = (LPC_UART_PCLK_FREQUENCY / 16) / uartp->config->baudrate;
      LPC_USART->LCR = USART_LCR_DLAB;
      LPC_USART->DLL = (uint8_t)uart_div;
      LPC_USART->DLM = (uint8_t)(uart_div >> 8);
      // TODO: Support user configurable Parity and word length.
      LPC_USART->LCR = USART_LCR_WLS_8B;
      // FIFO Reset & Enable
      LPC_USART->FCR = USART_FCR_FIFOEN | USART_FCR_TX_RST | USART_FCR_RX_RST |
        USART_FCR_RXTL_1B;

      #if !defined(LPC_UART_UART1_IRQ_PRIORITY)
      #error "LPC_UART_UART1_IRQ_PRIORITY is not defined"
      #endif
      nvicEnableVector(UART_IRQn, LPC_UART_UART1_IRQ_PRIORITY);
      LPC_USART->IER = USART_IER_RBRINTEN;
    }
#endif
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_stop(UARTDriver *uartp) {

  if (uartp->state == UART_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if LPC_UART_USE_UART1 == TRUE
    if (&UARTD1 == uartp) {
      LPC_USART->IER = 0;
      nvicDisableVector(UART_IRQn);
    }
#endif
  }
}

/**
 * @brief   Starts a transmission on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void uart_lld_start_send(UARTDriver *uartp, size_t n, const void *txbuf) {
  const uint8_t* buf = txbuf;
  if (buf == NULL) {
    return;
  }
  size_t i = 0;
  while(n > 0) {
    if(LPC_USART->LSR & USART_LSR_THRE) {
      LPC_USART->THR = buf[i++];
      n--;
    }
  }
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 *
 * @notapi
 */
size_t uart_lld_stop_send(UARTDriver *uartp) {
  (void)uartp;
  return 0;
}

/**
 * @brief   Starts a receive operation on the UART peripheral.
 * @note    The buffers are organized as uint8_t arrays for data sizes below
 *          or equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] n         number of data frames to send
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf) {
  const uint8_t* buf = rxbuf;
  if (buf == NULL) {
    return;
  }
  size_t i = 0;
  while(n > 0) {
    if(LPC_USART->LSR & USART_LSR_RDR) {
      buf[i++] = LPC_USART->RBR;
      n--;
    }
  }
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 *
 * @notapi
 */
size_t uart_lld_stop_receive(UARTDriver *uartp) {
  (void)uartp;
  return 0;
}

#endif /* HAL_USE_UART == TRUE */

/** @} */
