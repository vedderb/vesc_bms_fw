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
 * @file    USART/hal_uart_lld.h
 * @brief   GD32 low level UART driver header.
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
 * @brief   UART driver on USART0 enable switch.
 * @details If set to @p TRUE the support for USART0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_UART_USE_USART0) || defined(__DOXYGEN__)
#define GD32_UART_USE_USART0               FALSE
#endif

/**
 * @brief   UART driver on USART1 enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_UART_USE_USART1) || defined(__DOXYGEN__)
#define GD32_UART_USE_USART1               FALSE
#endif

/**
 * @brief   UART driver on USART2 enable switch.
 * @details If set to @p TRUE the support for USART2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_UART_USE_USART2) || defined(__DOXYGEN__)
#define GD32_UART_USE_USART2               FALSE
#endif

/**
 * @brief   UART driver on UART3 enable switch.
 * @details If set to @p TRUE the support for UART3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_UART_USE_UART3) || defined(__DOXYGEN__)
#define GD32_UART_USE_UART3                FALSE
#endif

/**
 * @brief   UART driver on UART4 enable switch.
 * @details If set to @p TRUE the support for UART4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_UART_USE_UART4) || defined(__DOXYGEN__)
#define GD32_UART_USE_UART4                FALSE
#endif

/**
 * @brief   USART0 interrupt priority level setting.
 */
#if !defined(GD32_UART_USART0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_USART0_IRQ_PRIORITY      12
#endif

/**
 * @brief   USART1 interrupt priority level setting.
 */
#if !defined(GD32_UART_USART1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_USART1_IRQ_PRIORITY      12
#endif

/**
 * @brief   USART2 interrupt priority level setting.
 */
#if !defined(GD32_UART_USART2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_USART2_IRQ_PRIORITY      12
#endif

/**
 * @brief   UART3 interrupt priority level setting.
 */
#if !defined(GD32_UART_UART3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_UART3_IRQ_PRIORITY       12
#endif

/**
 * @brief   UART4 interrupt priority level setting.
 */
#if !defined(GD32_UART_UART4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_UART4_IRQ_PRIORITY       12
#endif

/**
 * @brief   USART0 interrupt trigger setting.
 */
#if !defined(GD32_UART_USART0_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_UART_USART0_IRQ_TRIGGER      ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   USART1 interrupt trigger setting.
 */
#if !defined(GD32_UART_USART1_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_UART_USART1_IRQ_TRIGGER      ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   USART2 interrupt trigger setting.
 */
#if !defined(GD32_UART_USART2_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_UART_USART2_IRQ_TRIGGER      ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   UART3 interrupt trigger setting.
 */
#if !defined(GD32_UART_UART3_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_UART_UART3_IRQ_TRIGGER       ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   UART4 interrupt trigger setting.
 */
#if !defined(GD32_UART_UART4_IRQ_TRIGGER) || defined(__DOXYGEN__)
#define GD32_UART_UART4_IRQ_TRIGGER       ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   USART0 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(GD32_UART_USART0_DMA_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_USART0_DMA_PRIORITY      0
#endif

/**
 * @brief   USART1 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(GD32_UART_USART1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_USART1_DMA_PRIORITY      0
#endif

/**
 * @brief   USART2 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(GD32_UART_USART2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_USART2_DMA_PRIORITY      0
#endif

/**
 * @brief   UART3 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(GD32_UART_UART3_DMA_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_UART3_DMA_PRIORITY       0
#endif

/**
 * @brief   UART4 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(GD32_UART_UART4_DMA_PRIORITY) || defined(__DOXYGEN__)
#define GD32_UART_UART4_DMA_PRIORITY       0
#endif

/**
 * @brief   USART DMA error hook.
 * @note    The default action for DMA errors is a system halt because DMA
 *          error can only happen because programming errors.
 */
#if !defined(GD32_UART_DMA_ERROR_HOOK) || defined(__DOXYGEN__)
#define GD32_UART_DMA_ERROR_HOOK(uartp)    osalSysHalt("DMA failure")
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if GD32_UART_USE_USART0 && !GD32_HAS_USART0
#error "USART0 not present in the selected device"
#endif

#if GD32_UART_USE_USART1 && !GD32_HAS_USART1
#error "USART1 not present in the selected device"
#endif

#if GD32_UART_USE_USART2 && !GD32_HAS_USART2
#error "USART2 not present in the selected device"
#endif

#if GD32_UART_USE_UART3 && !GD32_HAS_UART3
#error "UART3 not present in the selected device"
#endif

#if GD32_UART_USE_UART4 && !GD32_HAS_UART4
#error "UART4 not present in the selected device"
#endif

#if GD32_UART_USE_UART4
#if !GD32_HAS_UART4
#error "UART4 not present in the selected device"
#endif

#if defined(GD32VF103) 
#error "UART4 DMA access not supported in this platform"
#endif
#endif /* GD32_UART_USE_UART4 */

#if !GD32_UART_USE_USART0 && !GD32_UART_USE_USART1 &&                     \
    !GD32_UART_USE_USART2 && !GD32_UART_USE_UART3 &&                      \
    !GD32_UART_USE_UART4  
#error "UART driver activated but no USART/UART peripheral assigned"
#endif

#if GD32_UART_USE_USART0 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_UART_USART0_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to USART0"
#endif

#if GD32_UART_USE_USART1 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_UART_USART1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to USART1"
#endif

#if GD32_UART_USE_USART2 &&                                                \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_UART_USART2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to USART2"
#endif

#if GD32_UART_USE_UART3 &&                                                 \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_UART_UART3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to UART3"
#endif

#if GD32_UART_USE_UART4 &&                                                 \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_UART_UART4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to UART4"
#endif

#if GD32_UART_USE_USART0 &&                                                \
    !GD32_DMA_IS_VALID_PRIORITY(GD32_UART_USART0_DMA_PRIORITY)
#error "Invalid DMA priority assigned to USART0"
#endif

#if GD32_UART_USE_USART1 &&                                                \
    !GD32_DMA_IS_VALID_PRIORITY(GD32_UART_USART1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to USART1"
#endif

#if GD32_UART_USE_USART2 &&                                                \
    !GD32_DMA_IS_VALID_PRIORITY(GD32_UART_USART2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to USART2"
#endif

#if GD32_UART_USE_UART3 &&                                                 \
    !GD32_DMA_IS_VALID_PRIORITY(GD32_UART_UART3_DMA_PRIORITY)
#error "Invalid DMA priority assigned to UART3"
#endif

#if GD32_UART_USE_UART4 &&                                                 \
    !GD32_DMA_IS_VALID_PRIORITY(GD32_UART_UART4_DMA_PRIORITY)
#error "Invalid DMA priority assigned to UART4"
#endif

#if !defined(GD32_DMA_REQUIRED)
#define GD32_DMA_REQUIRED
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
   * @brief Bit rate.
   */
  uint32_t                  speed;
  /**
   * @brief Initialization value for the CTL0 register.
   */
  uint16_t                  ctl0;
  /**
   * @brief Initialization value for the CTL1 register.
   */
  uint16_t                  ctl1;
  /**
   * @brief Initialization value for the CTL2 register.
   */
  uint16_t                  ctl2;
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
   * @brief Pointer to the USART registers block.
   */
  USART_TypeDef             *usart;
  /**
   * @brief Receive DMA mode bit mask.
   */
  uint32_t                  dmarxmode;
  /**
   * @brief Send DMA mode bit mask.
   */
  uint32_t                  dmatxmode;
  /**
   * @brief Receive DMA channel.
   */
  const gd32_dma_stream_t  *dmarx;
  /**
   * @brief Transmit DMA channel.
   */
  const gd32_dma_stream_t  *dmatx;
  /**
   * @brief Default receive buffer while into @p UART_RX_IDLE state.
   */
  volatile uint16_t         rxbuf;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if GD32_UART_USE_USART0 && !defined(__DOXYGEN__)
extern UARTDriver UARTD1;
#endif

#if GD32_UART_USE_USART1 && !defined(__DOXYGEN__)
extern UARTDriver UARTD2;
#endif

#if GD32_UART_USE_USART2 && !defined(__DOXYGEN__)
extern UARTDriver UARTD3;
#endif

#if GD32_UART_USE_UART3 && !defined(__DOXYGEN__)
extern UARTDriver UARTD4;
#endif

#if GD32_UART_USE_UART4 && !defined(__DOXYGEN__)
extern UARTDriver UARTD5;
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
