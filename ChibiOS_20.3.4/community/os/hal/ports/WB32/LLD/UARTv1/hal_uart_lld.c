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

/**
 * @file    UARTv1/hal_uart_lld.c
 * @brief   WB32 low level UART driver code.
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"

#if HAL_USE_UART || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART1 UART driver identifier.*/
#if WB32_UART_USE_UART1 || defined(__DOXYGEN__)
UARTDriver UARTD1;
#endif

/** @brief UART2 UART driver identifier.*/
#if WB32_UART_USE_UART2 || defined(__DOXYGEN__)
UARTDriver UARTD2;
#endif

/** @brief UART3 UART driver identifier.*/
#if WB32_UART_USE_UART3 || defined(__DOXYGEN__)
UARTDriver UARTD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Status bits translation.
 *
 * @param[in] sr        UART SR register value
 *
 * @return  The error flags.
 */
static uartflags_t translate_errors(uint16_t sr) {
  uartflags_t sts = 0;

  if (sr & UART_LSR_OE)
    sts |= UART_OVERRUN_ERROR;
  if (sr & UART_LSR_PE)
    sts |= UART_PARITY_ERROR;
  if (sr & UART_LSR_FE)
    sts |= UART_FRAMING_ERROR;
  if (sr & UART_LSR_DR)
    sts |= UART_NOISE_ERROR;
  if (sr & UART_LSR_BI)
    sts |= UART_BREAK_DETECTED;
  return sts;
}

/**
 * @brief   Puts the receiver in the UART_RX_IDLE state.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void uart_enter_rx_idle_loop(UARTDriver *uartp) {

  uartp->xfer.rx_len = 1;
  uartp->xfer.rx_buf = (uint8_t *)&uartp->rxbuf;

  uartp->uart->IER |= UART_IER_RDAIE;
}

/**
 * @brief   UART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void uart_stop(UARTDriver *uartp) {

  /* Disable the UART Interrupt.*/
  uartp->uart->IER &= ~(0xFF);
}

/**
 * @brief   UART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void uart_start(UARTDriver *uartp) {
  uint32_t divider, apbclock;
  UART_TypeDef *u = uartp->uart;

  uart_stop(uartp);

  u->MCR = (u->MCR & 0x50) | uartp->config->UART_AutoFlowControl;

  if (u == UART1) {
    apbclock = WB32_PCLK1;
  }
  else {
    apbclock = WB32_PCLK2;
  }

  /* round off.*/
  divider = (apbclock + (uartp->config->UART_BaudRate >> 1)) /
            uartp->config->UART_BaudRate;

  u->DLF = divider & 0x0F;
  u->LCR = UART_LCR_DLAB;
  u->DLL = (uint8_t)(divider >> 4);
  u->DLH = (uint8_t)(divider >> 12);
  u->LCR = 0x00;
  u->LCR = uartp->config->UART_WordLength | uartp->config->UART_StopBits |
           uartp->config->UART_Parity;

  u->SRT = UART_RxFIFOThreshold_1;
  u->SFE = 0x01;

  /* Starting the receiver idle loop.*/
  uart_enter_rx_idle_loop(uartp);
}

/**
 * @brief   UART common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void serve_uart_irq(UARTDriver *uartp) {
  UART_TypeDef *u = uartp->uart;
  uint32_t sr;
  uint8_t rbyte;
  uint8_t int_id;

  int_id = (u->IIR & UART_IIR_INTID_Msk);

  /* FIFO Time out.*/
  if (int_id == UART_IIR_INTID_CTI) {
    while ((u->USR & UART_USR_RFNE) != RESET) {
      rbyte = (uint16_t)u->RBR;

      if (uartp->xfer.rx_len) {
        *uartp->xfer.rx_buf = rbyte;
        uartp->xfer.rx_buf++;
        uartp->xfer.rx_len--;
        if (uartp->xfer.rx_len == 0) {
          if (uartp->rxstate == UART_RX_IDLE) {
            /* Receiver in idle state, a callback is generated,
               if enabled, for each received character and then the
               driver stays in the same state.*/
            _uart_rx_idle_code(uartp);
          }
          else {
            _uart_rx_complete_isr_code(uartp);
          }
        }
      }
    }
    /* Timeout interrupt sources are only checked if enabled in IIR.*/
    _uart_timeout_isr_code(uartp);
  }
  else if (int_id == UART_IIR_INTID_RDA) { /* Receive One Byte.*/
    rbyte = (uint16_t)u->RBR;

    if (uartp->xfer.rx_len) {
      *uartp->xfer.rx_buf = rbyte;
      uartp->xfer.rx_buf++;
      uartp->xfer.rx_len--;

      if (uartp->xfer.rx_len == 0) {
        if (uartp->rxstate == UART_RX_IDLE) {
          /* Receiver in idle state, a callback is generated, if enabled,
             for each received character and then the driver stays in the
             same state.*/
          _uart_rx_idle_code(uartp);
        }
        else {
          _uart_rx_complete_isr_code(uartp);
        }
      }
    }
  }
  /* Send One Byte.*/
  else if (int_id == UART_IIR_INTID_THRE) {
    if (uartp->xfer.tx_len) {
      u->THR = (uint16_t)*uartp->xfer.tx_buf;
      uartp->xfer.tx_buf++;
      uartp->xfer.tx_len--;
      if (uartp->xfer.tx_len == 0) {
        /* A callback is generated, if enabled, after a completed
           transfer.*/
        _uart_tx1_isr_code(uartp);
        /* End of transmission, a callback is generated.*/
        _uart_tx2_isr_code(uartp);
        /* Disable tx interrupt.*/
        u->IER &= ~UART_IER_THREIE;
      }
    }
  }
  sr = (uint32_t)u->LSR;
  if (sr & (UART_LSR_PE | UART_LSR_FE | UART_LSR_OE |
            UART_LSR_DR | UART_LSR_BI)) {
    _uart_rx_error_isr_code(uartp, translate_errors(sr));
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if WB32_UART_USE_UART1 || defined(__DOXYGEN__)
#if !defined(WB32_UART1_IRQ_VECTOR)
#error "WB32_UART1_IRQ_VECTOR not defined"
#endif
/**
 * @brief   UART1 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_UART1_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  serve_uart_irq(&UARTD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* WB32_UART_USE_UART1 */

#if WB32_UART_USE_UART2 || defined(__DOXYGEN__)
#if !defined(WB32_UART2_IRQ_VECTOR)
#error "WB32_UART2_IRQ_VECTOR not defined"
#endif
/**
 * @brief   UART2 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_UART2_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  serve_uart_irq(&UARTD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* WB32_UART_USE_UART2 */

#if WB32_UART_USE_UART3 || defined(__DOXYGEN__)
#if !defined(WB32_UART3_IRQ_VECTOR)
#error "WB32_UART3_IRQ_VECTOR not defined"
#endif
/**
 * @brief   UART3 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_UART3_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  serve_uart_irq(&UARTD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* WB32_UART_USE_UART3 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {

#if WB32_UART_USE_UART1
  uartObjectInit(&UARTD1);
  UARTD1.uart = UART1;
#endif

#if WB32_UART_USE_UART2
  uartObjectInit(&UARTD2);
  UARTD2.uart = UART2;
#endif

#if WB32_UART_USE_UART3
  uartObjectInit(&UARTD3);
  UARTD3.uart = UART3;
#endif
}

/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_start(UARTDriver *uartp) {

  if (uartp->state == UART_STOP) {
#if WB32_UART_USE_UART1
    if (&UARTD1 == uartp) {
      /* UART1 clock enable.*/
      rccEnableUART1();
      /* UART1 DeInit.*/
      rccResetUART1();
      nvicEnableVector(UART1_IRQn, WB32_UART_UART1_IRQ_PRIORITY);
    }
#endif

#if WB32_UART_USE_UART2
    if (&UARTD2 == uartp) {
      /* UART2 clock enable.*/
      rccEnableUART2();
      /* UART2 DeInit.*/
      rccResetUART2();

      nvicEnableVector(UART2_IRQn, WB32_UART_UART2_IRQ_PRIORITY);
    }
#endif

#if WB32_UART_USE_UART3
    if (&UARTD3 == uartp) {
      /* UART3 clock enable.*/
      rccEnableUART3();
      /* UART3 DeInit.*/
      rccResetUART3();

      nvicEnableVector(UART3_IRQn, WB32_UART_UART3_IRQ_PRIORITY);
    }
#endif
  }
  uartp->rxstate = UART_RX_IDLE;
  uartp->txstate = UART_TX_IDLE;
  uart_start(uartp);
}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_stop(UARTDriver *uartp) {

  if (uartp->state == UART_READY) {
    uartp->xfer.tx_len = 0;
    uartp->xfer.rx_len = 0;
    uartp->xfer.tx_buf = NULL;
    uartp->xfer.rx_buf = NULL;
    uartp->xfer.tx_abrt_source = 0;

    uart_stop(uartp);

#if WB32_UART_USE_UART1
    if (&UARTD1 == uartp) {
      nvicDisableVector(UART1_IRQn);
      rccDisableUART1();
      return;
    }
#endif

#if WB32_UART_USE_UART2
    if (&UARTD2 == uartp) {
      nvicDisableVector(UART2_IRQn);
      rccDisableUART2();
      return;
    }
#endif

#if WB32_UART_USE_UART3
    if (&UARTD3 == uartp) {
      nvicDisableVector(UART3_IRQn);
      rccDisableUART3();
      return;
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

  uartp->xfer.tx_len = n;
  uartp->xfer.tx_buf = txbuf;

  uartp->uart->IER |= UART_IER_THREIE;
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 *
 * @notapi
 */
size_t uart_lld_stop_send(UARTDriver *uartp) {

  uartp->uart->IER &= ~(UART_IER_THREIE);

  return (size_t)(uartp->xfer.tx_len);
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

  uartp->xfer.rx_len = n;
  uartp->xfer.rx_buf = rxbuf;

  uartp->uart->IER |= UART_IER_RDAIE;
}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 *
 * @notapi
 */
size_t uart_lld_stop_receive(UARTDriver *uartp) {
  uartp->uart->IER &= ~(UART_IER_RDAIE);

  return (size_t)(uartp->xfer.rx_len);
}

#endif /* HAL_USE_UART */

/** @} */
