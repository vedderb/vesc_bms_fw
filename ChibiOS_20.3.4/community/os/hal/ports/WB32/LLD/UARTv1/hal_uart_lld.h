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
 * @file    UARTv1/hal_uart_lld.h
 * @brief   WB32 low level UART driver header.
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

/** @defgroup UART_Exported_Constants
  * @{
  */

/** @defgroup UART_Word_Length
  * @{
  */
#define UART_WordLength_5b                     UART_LCR_WLS_5BIT
#define UART_WordLength_6b                     UART_LCR_WLS_6BIT
#define UART_WordLength_7b                     UART_LCR_WLS_7BIT
#define UART_WordLength_8b                     UART_LCR_WLS_8BIT
/**
  * @}
  */


/** @defgroup UART_Stop_Bits
  * @{
  */
#define UART_StopBits_One                      UART_LCR_SBS_1BIT
#define UART_StopBits_Two                      UART_LCR_SBS_2BIT
#define UART_StopBits_OnePointFive             UART_LCR_SBS_2BIT
/**
  * @}
  */


/** @defgroup UART_Parity
  * @{
  */
#define UART_Parity_None                       (0x00U)
#define UART_Parity_Odd                        (UART_LCR_PARITY_ODD)
#define UART_Parity_Even                       (UART_LCR_PARITY_EVEN)
#define UART_Parity_Mark                       (UART_LCR_PARITY_MARK)
#define UART_Parity_Space                      (UART_LCR_PARITY_SPACE)
/**
  * @}
  */


/** @defgroup UART_AutoFlowControl
  * @{
  */
#define UART_AutoFlowControl_None              (0x00)
#define UART_AutoFlowControl_CTS               (UART_MCR_AFCE)
#define UART_AutoFlowControl_RTS_CTS           (UART_MCR_AFCE | UART_MCR_RTS)
/**
  * @}
  */


/** @defgroup UART_RxFIFOThreshold
  * @{
  */
#define UART_RxFIFOThreshold_1                 0x00
#define UART_RxFIFOThreshold_4                 0x01
#define UART_RxFIFOThreshold_8                 0x02
#define UART_RxFIFOThreshold_14                0x03
/**
  * @}
  */


/** @defgroup UART_TxFIFOThreshold
  * @{
  */
#define UART_TxFIFOThreshold_0                 0x00
#define UART_TxFIFOThreshold_2                 0x01
#define UART_TxFIFOThreshold_4                 0x02
#define UART_TxFIFOThreshold_8                 0x03
/**
  * @}
  */

/**
 * @}
 */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   UART driver on UART1 enable switch.
 * @details If set to @p TRUE the support for UART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_UART_USE_UART1) || defined(__DOXYGEN__)
#define WB32_UART_USE_UART1                    FALSE
#endif

/**
 * @brief   UART driver on UART2 enable switch.
 * @details If set to @p TRUE the support for UART2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_UART_USE_UART2) || defined(__DOXYGEN__)
#define WB32_UART_USE_UART2                    FALSE
#endif

/**
 * @brief   UART driver on UART3 enable switch.
 * @details If set to @p TRUE the support for UART3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_UART_USE_UART3) || defined(__DOXYGEN__)
#define WB32_UART_USE_UART3                    FALSE
#endif

/**
 * @brief   UART1 interrupt priority level setting.
 */
#if !defined(WB32_UART_UART1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_UART_UART1_IRQ_PRIORITY          12
#endif

/**
 * @brief   UART2 interrupt priority level setting.
 */
#if !defined(WB32_UART_UART2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_UART_UART2_IRQ_PRIORITY          12
#endif

/**
 * @brief   UART3 interrupt priority level setting.
 */
#if !defined(WB32_UART_UART3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_UART_UART3_IRQ_PRIORITY          12
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if WB32_UART_USE_UART1 && !WB32_HAS_UART1
#error "UART1 not present in the selected device"
#endif

#if WB32_UART_USE_UART2 && !WB32_HAS_UART2
#error "UART2 not present in the selected device"
#endif

#if WB32_UART_USE_UART3 && !WB32_HAS_UART3
#error "UART3 not present in the selected device"
#endif

#if !WB32_UART_USE_UART1 && !WB32_UART_USE_UART2 &&                         \
    !WB32_UART_USE_UART3
#error "UART driver activated but no UART/UART peripheral assigned"
#endif

#if WB32_UART_USE_UART1 &&                                                  \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_UART_UART1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to UART1"
#endif

#if WB32_UART_USE_UART2 &&                                                  \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_UART_UART2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to UART2"
#endif

#if WB32_UART_USE_UART3 &&                                                  \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_UART_UART3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to UART3"
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

typedef struct {
  const uint8_t             *tx_buf;
  volatile uint32_t         tx_len;
  uint8_t                   *rx_buf;
  volatile uint32_t         rx_len;
  uint32_t                  tx_abrt_source;
} uart_xfer_info_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief End of transmission buffer callback.
   */
  uartcb_t                  txend1_cb;
  /**
   * @brief Physical end of transmission callback.
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
   * @brief   Receiver timeout callback.
   * @details Handles idle interrupts depending on configured
   *          flags in CR registers and supported hardware features.
   */
  uartcb_t                  timeout_cb;
  /**
   * @brief This member configures the UART communication baud rate.
   */
  uint32_t                  UART_BaudRate;
  /**
   * @brief Specifies the number of data bits transmitted or received in a frame.
   *        This parameter can be a value of @ref UART_Word_Length
   */
  uint8_t                   UART_WordLength;
  /**
  * @brief Specifies the number of stop bits transmitted.
  *        This parameter can be a value of @ref UART_Stop_Bits
  */
  uint8_t                   UART_StopBits;
  /**
   * @brief Specifies the parity mode.
   *        This parameter can be a value of @ref UART_Parity
   */
  uint8_t                   UART_Parity;
  /**
   * @brief Specifies the auto flow control mode is enabled or disabled.
   *        This parameter can be a value of @ref UART_AutoFlowControl
   */
  uint8_t                   UART_AutoFlowControl;
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
   * @brief Pointer to the UART registers block.
   */
  UART_TypeDef             *uart;
  /**
   * @brief Default receive buffer while into @p UART_RX_IDLE state.
   */
  volatile uint16_t         rxbuf;
  uart_xfer_info_t          xfer;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if WB32_UART_USE_UART1 && !defined(__DOXYGEN__)
extern UARTDriver UARTD1;
#endif

#if WB32_UART_USE_UART2 && !defined(__DOXYGEN__)
extern UARTDriver UARTD2;
#endif

#if WB32_UART_USE_UART3 && !defined(__DOXYGEN__)
extern UARTDriver UARTD3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void uart_lld_init(void);
  void uart_lld_start(UARTDriver *uartp);
  void uart_lld_stop(UARTDriver *uartp);
  void uart_lld_start_send(UARTDriver *uartp, size_t n, const void *txbuf);
  size_t uart_lld_stop_send(UARTDriver *uartp);
  void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf);
  size_t uart_lld_stop_receive(UARTDriver *uartp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_UART */

#endif /* HAL_UART_LLD_H */

/** @} */
