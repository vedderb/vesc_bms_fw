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
 * @file    USART/hal_serial_lld.h
 * @brief   GD32 low level serial driver header.
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

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   USART0 driver enable switch.
 * @details If set to @p TRUE the support for USART0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_SERIAL_USE_USART0) || defined(__DOXYGEN__)
#define GD32_SERIAL_USE_USART0             FALSE
#endif

/**
 * @brief   USART1 driver enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_SERIAL_USE_USART1) || defined(__DOXYGEN__)
#define GD32_SERIAL_USE_USART1             FALSE
#endif

/**
 * @brief   USART2 driver enable switch.
 * @details If set to @p TRUE the support for USART2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_SERIAL_USE_USART2) || defined(__DOXYGEN__)
#define GD32_SERIAL_USE_USART2             FALSE
#endif

/**
 * @brief   UART3 driver enable switch.
 * @details If set to @p TRUE the support for UART3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_SERIAL_USE_UART3) || defined(__DOXYGEN__)
#define GD32_SERIAL_USE_UART3              FALSE
#endif

/**
 * @brief   UART4 driver enable switch.
 * @details If set to @p TRUE the support for UART4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_SERIAL_USE_UART4) || defined(__DOXYGEN__)
#define GD32_SERIAL_USE_UART4              FALSE
#endif

/**
 * @brief   USART0 interrupt priority level setting.
 */
#if !defined(GD32_SERIAL_USART0_PRIORITY) || defined(__DOXYGEN__)
#define GD32_SERIAL_USART0_PRIORITY        12
#endif

/**
 * @brief   USART1 interrupt priority level setting.
 */
#if !defined(GD32_SERIAL_USART1_PRIORITY) || defined(__DOXYGEN__)
#define GD32_SERIAL_USART1_PRIORITY        12
#endif

/**
 * @brief   USART2 interrupt priority level setting.
 */
#if !defined(GD32_SERIAL_USART2_PRIORITY) || defined(__DOXYGEN__)
#define GD32_SERIAL_USART2_PRIORITY        12
#endif

/**
 * @brief   UART3 interrupt priority level setting.
 */
#if !defined(GD32_SERIAL_UART3_PRIORITY) || defined(__DOXYGEN__)
#define GD32_SERIAL_UART3_PRIORITY         12
#endif

/**
 * @brief   UART4 interrupt priority level setting.
 */
#if !defined(GD32_SERIAL_UART4_PRIORITY) || defined(__DOXYGEN__)
#define GD32_SERIAL_UART4_PRIORITY         12
#endif

/**
 * @brief   USART0 interrupt trigger setting.
 */
#if !defined(GD32_SERIAL_USART0_TRIGGER) || defined(__DOXYGEN__)
#define GD32_SERIAL_USART0_TRIGGER        ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   USART1 interrupt trigger setting.
 */
#if !defined(GD32_SERIAL_USART1_TRIGGER) || defined(__DOXYGEN__)
#define GD32_SERIAL_USART1_TRIGGER        ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   USART2 interrupt trigger setting.
 */
#if !defined(GD32_SERIAL_USART2_TRIGGER) || defined(__DOXYGEN__)
#define GD32_SERIAL_USART2_TRIGGER        ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   UART3 interrupt trigger setting.
 */
#if !defined(GD32_SERIAL_UART3_TRIGGER) || defined(__DOXYGEN__)
#define GD32_SERIAL_UART3_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   UART4 interrupt trigger setting.
 */
#if !defined(GD32_SERIAL_UART4_TRIGGER) || defined(__DOXYGEN__)
#define GD32_SERIAL_UART4_TRIGGER         ECLIC_TRIGGER_DEFAULT
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if GD32_SERIAL_USE_USART0 && !GD32_HAS_USART0
#error "USART0 not present in the selected device"
#endif

#if GD32_SERIAL_USE_USART1 && !GD32_HAS_USART1
#error "USART1 not present in the selected device"
#endif

#if GD32_SERIAL_USE_USART2 && !GD32_HAS_USART2
#error "USART2 not present in the selected device"
#endif

#if GD32_SERIAL_USE_UART3 && !GD32_HAS_UART3
#error "UART3 not present in the selected device"
#endif

#if GD32_SERIAL_USE_UART4 && !GD32_HAS_UART4
#error "UART4 not present in the selected device"
#endif

#if !GD32_SERIAL_USE_USART0 && !GD32_SERIAL_USE_USART1 &&                 \
    !GD32_SERIAL_USE_USART2 && !GD32_SERIAL_USE_UART3  &&                 \
    !GD32_SERIAL_USE_UART4
#error "SERIAL driver activated but no USART/UART peripheral assigned"
#endif

#if GD32_SERIAL_USE_USART0 &&                                              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_SERIAL_USART0_PRIORITY)
#error "Invalid IRQ priority assigned to USART0"
#endif

#if GD32_SERIAL_USE_USART1 &&                                              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_SERIAL_USART1_PRIORITY)
#error "Invalid IRQ priority assigned to USART1"
#endif

#if GD32_SERIAL_USE_USART2 &&                                              \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_SERIAL_USART2_PRIORITY)
#error "Invalid IRQ priority assigned to USART2"
#endif

#if GD32_SERIAL_USE_UART3 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_SERIAL_UART3_PRIORITY)
#error "Invalid IRQ priority assigned to UART3"
#endif

#if GD32_SERIAL_USE_UART4 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_SERIAL_UART4_PRIORITY)
#error "Invalid IRQ priority assigned to UART4"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   GD32 Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 * @note    This structure content is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct {
  /**
   * @brief Bit rate.
   */
  uint32_t                  speed;
  /* End of the mandatory fields.*/
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
  /* Pointer to the USART registers block.*/                                \
  USART_TypeDef             *usart;                                         \
  /* Mask to be applied on received frames.*/                               \
  uint8_t                   rxmask;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * Extra USARTs definitions here (missing from the ST header file).
 */
#define USART_CTL1_STB1_BITS    (0 << 12)   /**< @brief CR2 1 stop bit value.*/
#define USART_CTL1_STB0P5_BITS  (1 << 12)   /**< @brief CR2 0.5 stop bit value.*/
#define USART_CTL1_STB2_BITS    (2 << 12)   /**< @brief CR2 2 stop bit value.*/
#define USART_CTL1_STB1P5_BITS  (3 << 12)   /**< @brief CR2 1.5 stop bit value.*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if GD32_SERIAL_USE_USART0 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if GD32_SERIAL_USE_USART1 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif
#if GD32_SERIAL_USE_USART2 && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif
#if GD32_SERIAL_USE_UART3 && !defined(__DOXYGEN__)
extern SerialDriver SD4;
#endif
#if GD32_SERIAL_USE_UART4 && !defined(__DOXYGEN__)
extern SerialDriver SD5;
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
