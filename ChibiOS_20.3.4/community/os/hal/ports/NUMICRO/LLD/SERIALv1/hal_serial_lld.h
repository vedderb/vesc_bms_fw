/*
    Copyright (C) 2020 Alex Lewontin

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
 * @file    hal_serial_lld.h
 * @brief   NUC123 serial subsystem low level driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if (HAL_USE_SERIAL) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   UART clock sources.
 */
#define NUC123_SERIAL_CLKSRC_HSE (0UL << CLK_CLKSEL1_UART_S_Pos)
#define NUC123_SERIAL_CLKSRC_PLL (1UL << CLK_CLKSEL1_UART_S_Pos)
#define NUC123_SERIAL_CLKSRC_HSI (3UL << CLK_CLKSEL1_UART_S_Pos)

#define NUC123_SERIAL_MODE_UART (0UL << UART_FUN_SEL_FUN_SEL_Pos)
/*
TODO: add IrDA and RS485 support
#define NUC123_SERIAL_MODE_IrDA             (2UL << UART_FUN_SEL_FUN_SEL_Pos)
#define NUC123_SERIAL_MODE_RS485            (3UL << UART_FUN_SEL_FUN_SEL_Pos)
*/

#define NUC123_SERIAL_DATA_5BITS (0U << UART_LCR_WLS_Pos)
#define NUC123_SERIAL_DATA_6BITS (1U << UART_LCR_WLS_Pos)
#define NUC123_SERIAL_DATA_7BITS (2U << UART_LCR_WLS_Pos)
#define NUC123_SERIAL_DATA_8BITS (3U << UART_LCR_WLS_Pos)

#define NUC123_SERIAL_PARITY_N (0U << UART_LCR_PBE_Pos)
#define NUC123_SERIAL_PARITY_O (1U << UART_LCR_PBE_Pos)
#define NUC123_SERIAL_PARITY_E (3U << UART_LCR_PBE_Pos)
#define NUC123_SERIAL_PARITY_M (5U << UART_LCR_PBE_Pos)
#define NUC123_SERIAL_PARITY_S (7U << UART_LCR_PBE_Pos)

#define NUC123_SERIAL_STOP_1     (0U << UART_LCR_NSB_Pos)
#define NUC123_SERIAL_STOP_MULTI (1U << UART_LCR_NSB_Pos)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    NUC123 configuration options
 * @{
 */
/**
 * @brief   UART0 driver enable switch.
 * @details If set to @p TRUE the support for USART0 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(NUC123_SERIAL_USE_UART0) || defined(__DOXYGEN__)
#define NUC123_SERIAL_USE_UART0 TRUE
#endif

/**
 * @brief   UART1 driver enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(NUC123_SERIAL_USE_UART1) || defined(__DOXYGEN__)
#define NUC123_SERIAL_USE_UART1 FALSE
#endif

/**
 * @brief   UART0 mode
 */
#if !defined(NUC123_SERIAL_MODE_DEFAULT) || defined(__DOXYGEN__)
#define NUC123_SERIAL_MODE_DEFAULT NUC123_SERIAL_MODE_UART
#endif

/**
 * @brief   UART clock source
 */
#if !defined(NUC123_SERIAL_CLKSRC) || defined(__DOXYGEN__)
#if NUC123_PLL_ENABLED
#define NUC123_SERIAL_CLKSRC NUC123_SERIAL_CLKSRC_PLL
#elif NUC123_HSI_ENABLED
#define NUC123_SERIAL_CLKSRC NUC123_SERIAL_CLKSRC_HSI
#elif NUC123_HSE_ENABLED
#define NUC123_SERIAL_CLKSRC NUC123_SERIAL_CLKSRC_HSE
#else
#error "Using the serial module requires either the HSE, the HSI, or the PLL to be enabled."
#endif
#endif

#if !defined(NUC123_SERIAL_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define NUC123_SERIAL_IRQ_PRIORITY 3
#endif

#if !defined(NUC123_SERIAL_UART0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define NUC123_SERIAL_UART0_IRQ_PRIORITY NUC123_SERIAL_IRQ_PRIORITY
#endif

#if !defined(NUC123_SERIAL_UART1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define NUC123_SERIAL_UART1_IRQ_PRIORITY NUC123_SERIAL_IRQ_PRIORITY
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !NUC123_SERIAL_USE_UART0 && !NUC123_SERIAL_USE_UART1
#error "SERIAL driver activated but no USART/UART peripheral in use"
#endif

#if (NUC123_SERIAL_CLKSRC != NUC123_SERIAL_CLKSRC_PLL) &&                                          \
    (NUC123_SERIAL_CLKSRC != NUC123_SERIAL_CLKSRC_HSE) &&                                          \
    (NUC123_SERIAL_CLKSRC != NUC123_SERIAL_CLKSRC_HSI)
#error                                                                                             \
    "NUC123_SERIAL_CLKSRC must be NUC123_SERIAL_CLKSRC_PLL, NUC123_SERIAL_CLKSRC_HSE, or NUC123_SERIAL_CLKSRC_HSI"
#endif

#if NUC123_SERIAL_CLKSRC == NUC123_SERIAL_CLKSRC_PLL
#define NUC123_SERIAL_CLK NUC123_PLLCLK
#elif NUC123_SERIAL_CLKSRC == NUC123_SERIAL_CLKSRC_HSE
#define NUC123_SERIAL_CLK NUC123_HSECLK
#elif NUC123_SERIAL_CLKSRC == NUC123_SERIAL_CLKSRC_HSI
#define NUC123_SERIAL_CLK NUC123_HSICLK
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Serial modes
 */
typedef uint32_t serial_mode_t;

/**
 * @brief   NUC123 Serial Driver configuration structure.
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
  uint32_t speed;
  /* End of the mandatory fields.*/

  /**
   * @brief Mode
   */
  serial_mode_t mode;
  uint8_t       data;
  uint8_t       parity;
  uint8_t       stop;

} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                                        \
  _base_asynchronous_channel_data /* Driver state.*/                                               \
      sdstate_t state;                                                                             \
  /* Input queue.*/                                                                                \
  input_queue_t iqueue;                                                                            \
  /* Output queue.*/                                                                               \
  output_queue_t oqueue;                                                                           \
  /* Input circular buffer.*/                                                                      \
  uint8_t ib[SERIAL_BUFFERS_SIZE];                                                                 \
  /* Output circular buffer.*/                                                                     \
  uint8_t ob[SERIAL_BUFFERS_SIZE];                                                                 \
  /* End of the mandatory fields.*/                                                                \
  UART_T* uart;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (NUC123_SERIAL_USE_UART0) && !defined(__DOXYGEN__)
extern SerialDriver SD0;
#endif

#if (NUC123_SERIAL_USE_UART1) && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
void sd_lld_init(void);
void sd_lld_start(SerialDriver* sdp, const SerialConfig* config);
void sd_lld_stop(SerialDriver* sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
