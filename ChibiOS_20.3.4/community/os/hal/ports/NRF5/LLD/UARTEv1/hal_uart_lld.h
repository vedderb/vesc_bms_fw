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
 * @file    UARTEv1/hal_uart_lld.h
 * @brief   NRF5 low level UART driver header.
 * @author	andru
 *
 * @addtogroup UART
 * @{
 */

#ifndef HAL_UART_LLD_H
#define HAL_UART_LLD_H

#if HAL_USE_UART || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   UART driver on UARTE enable switch.
 * @details If set to @p TRUE the support for UART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(NRF5_UART_USE_UART0) || defined(__DOXYGEN__)
#define NRF5_UART_USE_UART0               FALSE
#endif

/**
 * @brief   UART driver on UARTE enable hardware flow control switch.
 * @details If set to @p TRUE the support hardware flow control is included.
 * @note    The default is @p FALSE.
 */
#if !defined(NRF5_UART_USE_HWFLOWCTRL) || defined(__DOXYGEN__)
#define NRF5_UART_USE_HWFLOWCTRL          FALSE
#endif

/**
 * @brief   UART0 interrupt priority level setting.
 */
#if !defined(NRF5_UART_UART0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define NRF5_UART_UART0_IRQ_PRIORITY      3
#endif

/* Value indicating that no pad is connected to this UART register. */
#define  NRF5_UART_PAD_DISCONNECTED 0xFFFFFFFFU
#define  NRF5_UART_INVALID_BAUDRATE 0xFFFFFFFFU

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if HAL_USE_SERIAL
#error "Only one UART or SERIAL driver can be activated"
#endif
#if !NRF5_UART_USE_UART0
#error "UART driver activated but no USART/UART peripheral assigned"
#endif

#if NRF5_UART_USE_UART0 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(NRF5_UART_UART0_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to UART0"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   UART driver condition flags type.
 */
typedef uint32_t uartflags_t;

/**
 * @brief   Structure representing an UART driver.
 */
typedef struct UARTDriver UARTDriver;

/**
 * @brief   Generic UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
typedef void (*uartcb_t)(UARTDriver *uartp);

/**
 * @brief   Character received UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] c         received character
 */
typedef void (*uartccb_t)(UARTDriver *uartp, uint16_t c);

/**
 * @brief   Receive error UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] e         receive error mask
 */
typedef void (*uartecb_t)(UARTDriver *uartp, uartflags_t e);

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief Start of transmission callback.
   */
  uartcb_t                  txend1_cb;
  /**
   * @brief Physical end of transmission callback
   */
  uartcb_t                  txend2_cb;
  /**
   * @brief Receive buffer filled callback.
   */
  uartcb_t                  rxend_cb;
  /**
   * @brief Character received while out if the @p UART_RECEIVE state.
   */
  uartccb_t                 rxchar_cb;
  /**
   * @brief Receive error callback.
   */
  uartecb_t                 rxerr_cb;
  /* End of the mandatory fields.*/
  /**
   * @brief Bit rate.
   */
  uint32_t                  speed;
  /**
   * @brief Pin select for TXD signal.
   */
  uint32_t                  tx_pad;
  /**
   * @brief Pin select for RXD signal.
   */
  uint32_t                  rx_pad;
#if (NRF5_UART_USE_HWFLOWCTRL == TRUE)
  /**
   * @brief Pin select for RTS signal.
   */
  uint32_t                  rts_pad;
  /**
   * @brief Pin select for CTS signal.
   */
  uint32_t                  cts_pad;
#endif
} UARTConfig;

/**
 * @brief   Structure representing an UART driver.
 */
struct UARTDriver {
  /**
   * @brief Driver state.
   */
  uartstate_t               state;
  /**
   * @brief Transmitter state.
   */
  uarttxstate_t             txstate;
  /**
   * @brief Receiver state.
   */
  uartrxstate_t             rxstate;
  /**
   * @brief Current configuration data.
   */
  const UARTConfig          *config;
#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Synchronization flag for transmit operations.
   */
  bool                      early;
  /**
   * @brief   Waiting thread on RX.
   */
  thread_reference_t        threadrx;
  /**
   * @brief   Waiting thread on TX.
   */
  thread_reference_t        threadtx;
#endif /* UART_USE_WAIT */
#if (UART_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the peripheral.
   */
  mutex_t                   mutex;
#endif /* UART_USE_MUTUAL_EXCLUSION */
#if defined(UART_DRIVER_EXT_FIELDS)
  UART_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the UARTE registers block.
   */
  NRF_UARTE_Type            *uart;
  /**
   * @brief Default receive buffer while into @p UART_RX_IDLE state.
   */
  volatile uint32_t         rxbuf;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if NRF5_UART_USE_UART0 && !defined(__DOXYGEN__)
extern UARTDriver UARTD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void uart_lld_init(void);
  void uart_lld_start(UARTDriver *uartp);
  void uart_lld_stop(UARTDriver *uartp);
  void uart_lld_start_send(UARTDriver *uartp, uint8_t n, const void *txbuf);
  size_t uart_lld_stop_send(UARTDriver *uartp);
  void uart_lld_start_receive(UARTDriver *uartp, uint8_t n, void *rxbuf);
  size_t uart_lld_stop_receive(UARTDriver *uartp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_UART */

#endif /* HAL_UART_LLD_H */

/** @} */
