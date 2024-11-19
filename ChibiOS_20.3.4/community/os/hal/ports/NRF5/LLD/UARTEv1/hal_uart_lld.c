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
 * @file    UARTEv1/hal_uart_lld.c
 * @brief   NRF5 low level UART driver code.
 * @author	andru
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"
#include <string.h>

#if HAL_USE_UART || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART1 UART driver identifier.*/
#if NRF5_UART_USE_UART0 || defined(__DOXYGEN__)
UARTDriver UARTD1;
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
 * @param[in] sr        UART ERRORSRC register value
 *
 * @return  The error flags.
 */
static uartflags_t translate_errors(uint32_t sr) {
  uartflags_t sts = 0;

  if (sr & UARTE_ERRORSRC_OVERRUN_Msk)
    sts |= UART_OVERRUN_ERROR;
  if (sr & UARTE_ERRORSRC_PARITY_Msk)
    sts |= UART_PARITY_ERROR;
  if (sr & UARTE_ERRORSRC_FRAMING_Msk)
    sts |= UART_FRAMING_ERROR;
  if (sr & UARTE_ERRORSRC_BREAK_Msk)
    sts |= UART_BREAK_DETECTED;
  return sts;
}

/**
 * @brief   Puts the receiver in the UART_RX_IDLE state.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void uart_enter_rx_idle_loop(UARTDriver *uartp) {
  NRF_UARTE_Type *u = uartp->uart;
  
  /* RX DMA channel preparation, if the char callback is defined then the
     interrupt is enabled too.*/
  if (uartp->config->rxchar_cb == NULL)
    u->INTENCLR = UARTE_INTENCLR_ENDRX_Msk;
  else
    u->INTENSET = UARTE_INTENSET_ENDRX_Msk;

  u->SHORTS = UARTE_SHORTS_ENDRX_STARTRX_Msk;

  uartp->rxbuf = 0;
  u->RXD.PTR = (uint32_t) &uartp->rxbuf;
  u->RXD.MAXCNT = 1;

  u->TASKS_STARTRX = 1;
}

/*
 * @brief Maps a baudrate speed to a BAUDRATE register value.
 */

/**
 * @brief   Common UART configuration.
 *
 */
static void configure_uart(UARTDriver *uartp, const UARTConfig *config)
{
  NRF_UARTE_Type *u = uartp->uart;
  uint32_t speed = UARTE_BAUDRATE_BAUDRATE_Baud250000;

  osalDbgAssert(
	 (config->rx_pad < TOTAL_GPIO_PADS) || (config->tx_pad < TOTAL_GPIO_PADS),
	  "must configure at least an RX or TX pad");

  switch (config->speed) {
    case 1200:    speed = UARTE_BAUDRATE_BAUDRATE_Baud1200; break;
    case 2400:    speed = UARTE_BAUDRATE_BAUDRATE_Baud2400; break;
    case 4800:    speed = UARTE_BAUDRATE_BAUDRATE_Baud4800; break;
    case 9600:    speed = UARTE_BAUDRATE_BAUDRATE_Baud9600; break;
    case 14400:   speed = UARTE_BAUDRATE_BAUDRATE_Baud14400; break;
    case 19200:   speed = UARTE_BAUDRATE_BAUDRATE_Baud19200; break;
    case 28800:   speed = UARTE_BAUDRATE_BAUDRATE_Baud28800; break;
    case 38400:   speed = UARTE_BAUDRATE_BAUDRATE_Baud38400; break;
    case 57600:   speed = UARTE_BAUDRATE_BAUDRATE_Baud57600; break;
    case 76800:   speed = UARTE_BAUDRATE_BAUDRATE_Baud76800; break;
    case 115200:  speed = UARTE_BAUDRATE_BAUDRATE_Baud115200; break;
    case 230400:  speed = UARTE_BAUDRATE_BAUDRATE_Baud230400; break;
    case 250000:  speed = UARTE_BAUDRATE_BAUDRATE_Baud250000; break;
    case 460800:  speed = UARTE_BAUDRATE_BAUDRATE_Baud460800; break;
    case 921600:  speed = UARTE_BAUDRATE_BAUDRATE_Baud921600; break;
    case 1000000: speed = UARTE_BAUDRATE_BAUDRATE_Baud1M; break;
    default: osalDbgAssert(0, "invalid baudrate"); break;
  };

  /* Configure PINs mode */
  if (config->tx_pad != NRF5_UART_PAD_DISCONNECTED) {
    palSetPadMode(IOPORT1, config->tx_pad, PAL_MODE_OUTPUT_PUSHPULL);
  }
  if (config->rx_pad != NRF5_UART_PAD_DISCONNECTED) {
    palSetPadMode(IOPORT1, config->rx_pad, PAL_MODE_INPUT);
  }
#if (NRF5_UART_USE_HWFLOWCTRL == TRUE)
  if (config->rts_pad != NRF5_UART_PAD_DISCONNECTED) {
    palSetPadMode(IOPORT1, config->rts_pad, PAL_MODE_OUTPUT_PUSHPULL);
  }
  if (config->cts_pad != NRF5_UART_PAD_DISCONNECTED) {
    palSetPadMode(IOPORT1, config->cts_pad, PAL_MODE_INPUT);
  }
#endif

  /* Select PINs used by UART */
  u->PSEL.TXD = config->tx_pad;
  u->PSEL.RXD = config->rx_pad;
#if (NRF5_UART_USE_HWFLOWCTRL == TRUE)
  u->PSEL.RTS = config->rts_pad;
  u->PSEL.CTS = config->cts_pad;
#else
  u->PSEL.RTS = NRF5_UART_PAD_DISCONNECTED;
  u->PSEL.CTS = NRF5_UART_PAD_DISCONNECTED;
#endif

  /* Set baud rate */
  u->BAUDRATE = speed;

  /* Set config */
  u->CONFIG = (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos);

  /* Adjust flow control */
#if (NRF5_UART_USE_HWFLOWCTRL == TRUE)
  if ((config->rts_pad < TOTAL_GPIO_PADS) ||
      (config->cts_pad < TOTAL_GPIO_PADS)) {
    u->CONFIG |= UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos;
  } else {
    u->CONFIG &= UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos;
  }
#else
  u->CONFIG &= UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos;
#endif

  /* Enable UART and clear events */
  u->ENABLE = UARTE_ENABLE_ENABLE_Enabled;

  u->EVENTS_ENDRX = 0;
  u->EVENTS_ENDTX = 0;
  u->EVENTS_ERROR = 0;
  u->EVENTS_RXTO = 0;
  u->EVENTS_TXSTARTED = 0;
  u->EVENTS_TXSTOPPED = 0;
#if CORTEX_MODEL >= 4
  (void)u->EVENTS_ENDRX;
  (void)u->EVENTS_ENDTX;
  (void)u->EVENTS_ERROR;
  (void)u->EVENTS_RXTO;
  (void)u->EVENTS_TXSTARTED;
  (void)u->EVENTS_TXSTOPPED;
#endif

#if (NRF5_UART_USE_HWFLOWCTRL == TRUE)
  u->EVENTS_CTS = 0;
  u->EVENTS_NCTS = 0;
#if CORTEX_MODEL >= 4
  (void)u->EVENTS_CTS;
  (void)u->EVENTS_NCTS;
#endif
#endif
}

/**
 * @brief   UART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void uart_stop(UARTDriver *uartp) {
  NRF_UARTE_Type *u = uartp->uart;

  /* Stops RX and TX.*/
  u->TASKS_STOPRX = 1;
  u->TASKS_STOPTX = 1;
}

/**
 * @brief   UART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void uart_start(UARTDriver *uartp) {
  /* Defensive programming, starting from a clean state.*/
  uart_stop(uartp);

  configure_uart(uartp, uartp->config);

  /* Starting the receiver idle loop.*/
  uart_enter_rx_idle_loop(uartp);
}

/**
 * @brief   RX DMA common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void uart_lld_serve_rx_end_irq(UARTDriver *uartp, uint32_t flags) {
  (void)flags;
  NRF_UARTE_Type *u = uartp->uart;

  if (uartp->rxstate == UART_RX_IDLE) {
    /* Receiver in idle state, a callback is generated, if enabled, for each
       received character and then the driver stays in the same state.*/
    if (uartp->config->rxchar_cb != NULL) {
    	_uart_rx_idle_code(uartp);
    }
  }
  else {
    /* Receiver in active state, a callback is generated, if enabled, after
       a completed transfer.*/
	u->TASKS_STOPRX = 1;
    _uart_rx_complete_isr_code(uartp);
  }
}

/**
 * @brief   TX DMA common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void uart_lld_serve_tx_end_irq(UARTDriver *uartp, uint32_t flags) {
  (void)flags;
  NRF_UARTE_Type *u = uartp->uart;

  /* A callback is generated, if enabled, after a completed transfer.*/
  u->TASKS_STOPTX = 1;
  _uart_tx2_isr_code(uartp);
}

/**
 * @brief   UART common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void serve_uart_irq(UARTDriver *uartp) {
  NRF_UARTE_Type *u = uartp->uart;
  uint32_t isr = u->INTENSET;

  if (u->EVENTS_ERROR) {
	uint32_t sr = u->ERRORSRC;

	u->EVENTS_ERROR = 0;
#if CORTEX_MODEL >= 4
	(void)u->EVENTS_ERROR;
#endif

    _uart_rx_error_isr_code(uartp, translate_errors(sr));
  }

  if (u->EVENTS_TXSTARTED && isr & UARTE_INTENSET_TXSTARTED_Msk) {
	u->INTENCLR &= UARTE_INTENCLR_TXSTARTED_Msk;

	u->EVENTS_TXSTARTED = 0;
#if CORTEX_MODEL >= 4
	(void)u->EVENTS_TXSTARTED;
#endif

    _uart_tx1_isr_code(uartp);
  }

  if (u->EVENTS_ENDTX && isr & UARTE_INTENSET_ENDTX_Msk) {
    /* interrupt cleared and disabled.*/
	u->INTENCLR &= UARTE_INTENCLR_ENDTX_Msk;

	u->EVENTS_ENDTX = 0;
#if CORTEX_MODEL >= 4
	(void)u->EVENTS_ENDTX;
#endif

    /* End of transmission, a callback is generated.*/
    uart_lld_serve_tx_end_irq(uartp, isr);
  }

  if (u->EVENTS_ENDRX && isr & UARTE_INTENSET_ENDRX_Msk) {
    /* interrupt cleared and disabled.*/

	u->EVENTS_ENDRX = 0;
#if CORTEX_MODEL >= 4
	(void)u->EVENTS_ENDRX;
#endif

    /* End of reception, a callback is generated.*/
    uart_lld_serve_rx_end_irq(uartp, isr);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   UART0 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector48) {

  OSAL_IRQ_PROLOGUE();

  serve_uart_irq(&UARTD1);

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {

#if NRF5_UART_USE_UART0
  uartObjectInit(&UARTD1);
  UARTD1.uart = NRF_UARTE0;
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
  NRF_UARTE_Type *u = uartp->uart;
  const UARTConfig *config = uartp->config;

  if (uartp->state == UART_STOP) {
      // Enable UART interrupt
      u->INTENCLR = (uint32_t)-1;
      u->INTENSET = UARTE_INTENSET_ERROR_Msk;
      if (config->rx_pad != NRF5_UART_PAD_DISCONNECTED) {
          u->INTENSET |= UARTE_INTENSET_ENDRX_Msk;
          u->INTENSET |= UARTE_INTENSET_RXTO_Msk;
      }
      if (config->tx_pad != NRF5_UART_PAD_DISCONNECTED) {
          u->INTENSET |= UARTE_INTENSET_TXSTARTED_Msk;
          u->INTENSET |= UARTE_INTENSET_ENDTX_Msk;
      }

      nvicEnableVector(UARTE0_UART0_IRQn, NRF5_UART_UART0_IRQ_PRIORITY);
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
  NRF_UARTE_Type *u=uartp->uart;

  if (uartp->state == UART_READY) {
    uart_stop(uartp);

    if (&UARTD1 == uartp) {
      nvicDisableVector(UARTE0_UART0_IRQn);
      u->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
    }
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
void uart_lld_start_send(UARTDriver *uartp, uint8_t n, const void *txbuf) {
  NRF_UARTE_Type *u=uartp->uart;

  /* RAM buffer for Easy DMA.*/
  uint8_t buf[n];
  memcpy(&buf, txbuf, n);

  /* TX DMA channel preparation.*/
  u->TXD.PTR = (uint32_t) &buf;
  u->TXD.MAXCNT = n;

  u->INTENSET = UARTE_INTENSET_TXSTARTED_Msk;
  /* Only enable ENDTX interrupt if there's a callback attached to it or
     if called from uartSendFullTimeout().*/
#if UART_USE_WAIT == TRUE
  if ((uartp->config->txend2_cb != NULL) || (uartp->early == false)) {
#else
  if (uartp->config->txend2_cb != NULL) {
#endif
    u->INTENSET = UARTE_INTENSET_ENDTX_Msk;
  }

  u->EVENTS_ENDTX = 0;
  u->EVENTS_ERROR = 0;
  u->EVENTS_TXSTARTED = 0;
  u->EVENTS_TXSTOPPED = 0;
#if CORTEX_MODEL >= 4
  (void)u->EVENTS_ENDTX;
  (void)u->EVENTS_ERROR;
  (void)u->EVENTS_TXSTARTED;
  (void)u->EVENTS_TXSTOPPED;
#endif

  /* Starting transfer.*/
  u->TASKS_STARTTX = 1;
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
  NRF_UARTE_Type *u=uartp->uart;

  u->TASKS_STOPTX = 1;

  return u->TXD.AMOUNT;
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
void uart_lld_start_receive(UARTDriver *uartp, uint8_t n, void *rxbuf) {
  NRF_UARTE_Type *u=uartp->uart;

  /* Stopping previous activity (idle state).*/
  u->TASKS_STOPRX = 1;
  u->SHORTS = 0;

  /* RX DMA channel preparation.*/
  u->RXD.PTR = (uint32_t) rxbuf;
  u->RXD.MAXCNT = n;

  u->EVENTS_ENDRX = 0;
  u->EVENTS_ERROR = 0;
  u->EVENTS_RXTO = 0;
#if CORTEX_MODEL >= 4
  (void)u->EVENTS_ENDRX;
  (void)u->EVENTS_ERROR;
  (void)u->EVENTS_RXTO;
#endif

  /* Starting transfer.*/
  u->TASKS_STARTRX = 1;
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
  NRF_UARTE_Type *u=uartp->uart;
  size_t n;

  u->TASKS_STOPRX = 1;
  n = u->RXD.AMOUNT;

  uart_enter_rx_idle_loop(uartp);

  return n;
}

#endif /* HAL_USE_UART */

/** @} */
