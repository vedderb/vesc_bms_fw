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
 * @file    UARTv1/hal_serial_lld.h
 * @brief   WB32 low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

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
 * @brief   UART1 driver enable switch.
 * @details If set to @p TRUE the support for UART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_SERIAL_USE_UART1) || defined(__DOXYGEN__)
#define WB32_SERIAL_USE_UART1                  FALSE
#endif

/**
 * @brief   UART2 driver enable switch.
 * @details If set to @p TRUE the support for UART2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_SERIAL_USE_UART2) || defined(__DOXYGEN__)
#define WB32_SERIAL_USE_UART2                  FALSE
#endif

/**
 * @brief   UART3 driver enable switch.
 * @details If set to @p TRUE the support for UART3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_SERIAL_USE_UART3) || defined(__DOXYGEN__)
#define WB32_SERIAL_USE_UART3                  FALSE
#endif

/**
 * @brief   UART1 interrupt priority level setting.
 */
#if !defined(WB32_SERIAL_UART1_PRIORITY) || defined(__DOXYGEN__)
#define WB32_SERIAL_UART1_PRIORITY             12
#endif

/**
 * @brief   UART2 interrupt priority level setting.
 */
#if !defined(WB32_SERIAL_UART2_PRIORITY) || defined(__DOXYGEN__)
#define WB32_SERIAL_UART2_PRIORITY             12
#endif

/**
 * @brief   UART3 interrupt priority level setting.
 */
#if !defined(WB32_SERIAL_UART3_PRIORITY) || defined(__DOXYGEN__)
#define WB32_SERIAL_UART3_PRIORITY             12
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if WB32_SERIAL_USE_UART1 && !WB32_HAS_UART1
#error "UART1 not present in the selected device"
#endif

#if WB32_SERIAL_USE_UART2 && !WB32_HAS_UART2
#error "UART2 not present in the selected device"
#endif

#if WB32_SERIAL_USE_UART3 && !WB32_HAS_UART3
#error "UART3 not present in the selected device"
#endif

#if !WB32_SERIAL_USE_UART1 && !WB32_SERIAL_USE_UART2 &&                     \
    !WB32_SERIAL_USE_UART3
#error "SERIAL driver activated but no UART/UART peripheral assigned"
#endif

#if WB32_SERIAL_USE_UART1 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_SERIAL_UART1_PRIORITY)
#error "Invalid IRQ priority assigned to UART1"
#endif

#if WB32_SERIAL_USE_UART2 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_SERIAL_UART2_PRIORITY)
#error "Invalid IRQ priority assigned to UART2"
#endif

#if WB32_SERIAL_USE_UART3 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(WB32_SERIAL_UART3_PRIORITY)
#error "Invalid IRQ priority assigned to UART3"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   WB32 Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 * @note    This structure content is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct {
  /**
   * @brief This member configures the UART communication baud rate.
   */
  uint32_t                  speed;
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
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  input_queue_t             iqueue;                                         \
  /* Output queue.*/                                                        \
  output_queue_t            oqueue;                                         \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                         \
  /* Pointer to the UART registers block.*/                                 \
  UART_TypeDef              *uart;                                          \
  /* Mask to be applied on received frames.*/                               \
  uint8_t                   rxmask;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if WB32_SERIAL_USE_UART1 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if WB32_SERIAL_USE_UART2 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif
#if WB32_SERIAL_USE_UART3 && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
