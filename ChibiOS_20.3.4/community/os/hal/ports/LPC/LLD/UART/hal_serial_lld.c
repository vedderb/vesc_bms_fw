/*
    ChibiOS - Copyright (C) 2020 Yaotian Feng

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
 * @file    hal_serial_lld.c
 * @brief   LPC11Uxx serial subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 serial driver identifier.*/
#if (LPC_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  9600
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void load(SerialDriver *sdp) {
  if (&SD1 == sdp) {
    if (LPC_USART->LSR & USART_LSR_THRE) {
      msg_t b;
      osalSysLock();
      b = oqGetI(&sdp->oqueue);
      osalSysUnlock();
      if (b >= MSG_OK) {
        LPC_USART->THR = b;
      }
    }
    LPC_USART->IER |= USART_IER_THRINTEN; // Enable Tx Empty
  }
}

#if LPC_SERIAL_USE_UART1 == TRUE
static void notify1(io_queue_t *qp) {
  (void)qp;
  load(&SD1);
}
#endif

static void serial_interrupt(SerialDriver *sdp) {
  if (LPC_USART->LSR & USART_LSR_RDR) {
    // Rx Pending
    osalSysLockFromISR();
    if (iqIsEmptyI(&sdp->iqueue)) {
      chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
    }
    osalSysUnlockFromISR();

    while(LPC_USART->LSR & USART_LSR_RDR) {
      osalSysLockFromISR();
      if (iqPutI(&sdp->iqueue, LPC_USART->RBR) < MSG_OK) {
        chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
      }
      osalSysUnlockFromISR();
    }
  }

  if (LPC_USART->LSR & USART_LSR_THRE) {
    msg_t b;
    osalSysLockFromISR();
    b = oqGetI(&sdp->oqueue);
    osalSysUnlockFromISR();
    if (b >= MSG_OK) {
      LPC_USART->THR = b;
    } else {
      LPC_USART->IER &= ~USART_IER_THRINTEN;
      osalSysLockFromISR();
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      osalSysUnlockFromISR();
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (LPC_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
CH_IRQ_HANDLER(LPC_UART_IRQ_VECTOR) {
  CH_IRQ_PROLOGUE();
  serial_interrupt(&SD1);
  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {
#if LPC_SERIAL_USE_UART1 == TRUE
  sdObjectInit(&SD1, NULL, notify1);
  LPC_SYSCON->UARTCLKDIV = LPC_UART_PCLK_DIV; // Set Proper Clock Divider
  LPC_SYSCON->SYSAHBCLKCTRL |= SYSCON_SYSAHBCLKCTRL_USART; // Enable USART Clock
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL) {
    config = &default_config;
  }

  if (sdp->state == SD_STOP) {
#if LPC_SERIAL_USE_UART1 == TRUE
    if (&SD1 == sdp) {
      // Disable Auto Baudrate. Not helpful
      LPC_USART->ACR = 0;
      LPC_USART->TER = USART_TER_TXEN;
      /* Clock should be set to 16x baudrate */
      uint16_t uart_div = (LPC_UART_PCLK_FREQUENCY / 16) / config->speed;
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
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
#if LPC_SERIAL_USE_UART1 == TRUE
    if (&SD1 == sdp) {
      nvicDisableVector(UART_IRQn);
      LPC_USART->IER = 0;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
