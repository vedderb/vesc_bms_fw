/*
    ChibiOS - Copyright (C) 2013-2015 Fabio Utzig

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
 * @brief   Kinetis KL2x Serial Driver subsystem low level driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H_
#define HAL_SERIAL_LLD_H_

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

#include "fsl_lpuart.h"

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
 * @brief   SD1 driver enable switch.
 * @details If set to @p TRUE the support for SD1 is included.
 */
#if !defined(MIMXRT1062_SERIAL_USE_UART0) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_USE_UART0             TRUE
#endif
/**
 * @brief   SD2 driver enable switch.
 * @details If set to @p TRUE the support for SD2 is included.
 */
#if !defined(MIMXRT1062_SERIAL_USE_UART1) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_USE_UART1             FALSE
#endif
/**
 * @brief   SD3 driver enable switch.
 * @details If set to @p TRUE the support for SD3 is included.
 */
#if !defined(MIMXRT1062_SERIAL_USE_UART2) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_USE_UART2             TRUE
#endif
/**
 * @brief   SD4 driver enable switch.
 * @details If set to @p TRUE the support for SD4 is included.
 */
#if !defined(MIMXRT1062_SERIAL_USE_UART3) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_USE_UART3             TRUE
#endif
/**
 * @brief   SD5 driver enable switch.
 * @details If set to @p TRUE the support for SD5 is included.
 */
#if !defined(MIMXRT1062_SERIAL_USE_UART4) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_USE_UART4             FALSE
#endif
/**
 * @brief   SD6 driver enable switch.
 * @details If set to @p TRUE the support for SD6 is included.
 */
#if !defined(MIMXRT1062_SERIAL_USE_UART5) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_USE_UART5             FALSE
#endif

/**
 * @brief   UART0 interrupt priority level setting.
 */
#if !defined(MIMXRT1062_SERIAL_UART0_PRIORITY) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_UART0_PRIORITY        12
#endif

/**
 * @brief   UART1 interrupt priority level setting.
 */
#if !defined(MIMXRT1062_SERIAL_UART1_PRIORITY) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_UART1_PRIORITY        12
#endif

/**
 * @brief   UART2 interrupt priority level setting.
 */
#if !defined(MIMXRT1062_SERIAL_UART2_PRIORITY) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_UART2_PRIORITY        12
#endif

/**
 * @brief   UART3 interrupt priority level setting.
 */
#if !defined(MIMXRT1062_SERIAL_UART3_PRIORITY) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_UART3_PRIORITY        12
#endif

/**
 * @brief   UART4 interrupt priority level setting.
 */
#if !defined(MIMXRT1062_SERIAL_UART4_PRIORITY) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_UART4_PRIORITY        12
#endif

/**
 * @brief   UART5 interrupt priority level setting.
 */
#if !defined(MIMXRT1062_SERIAL_UART5_PRIORITY) || defined(__DOXYGEN__)
#define MIMXRT1062_SERIAL_UART5_PRIORITY        12
#endif

/**
 * @brief   UART0 clock source.
 */
#if !defined(MIMXRT1062_UART0_CLOCK_SRC) || defined(__DOXYGEN__)
#define MIMXRT1062_UART0_CLOCK_SRC              1 /* MCGFLLCLK clock, or MCGPLLCLK/2; or IRC48M */
#endif

/**
 * @brief   UART1 clock source.
 */
#if !defined(MIMXRT1062_UART1_CLOCK_SRC) || defined(__DOXYGEN__)
#define MIMXRT1062_UART1_CLOCK_SRC              1 /* IRC48M */
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/** @brief  error checks */
#if MIMXRT1062_SERIAL_USE_UART0 && !MIMXRT1062_HAS_SERIAL0
#error "UART0 not present in the selected device"
#endif

#if MIMXRT1062_SERIAL_USE_UART1 && !MIMXRT1062_HAS_SERIAL1
#error "UART1 not present in the selected device"
#endif

#if MIMXRT1062_SERIAL_USE_UART2 && !MIMXRT1062_HAS_SERIAL2
#error "UART2 not present in the selected device"
#endif

#if MIMXRT1062_SERIAL_USE_UART3 && !MIMXRT1062_HAS_SERIAL3
#error "UART3 not present in the selected device"
#endif

#if MIMXRT1062_SERIAL_USE_UART4 && !MIMXRT1062_HAS_SERIAL4
#error "UART4 not present in the selected device"
#endif

#if MIMXRT1062_SERIAL_USE_UART5 && !MIMXRT1062_HAS_SERIAL5
#error "UART5 not present in the selected device"
#endif

#if !(MIMXRT1062_SERIAL_USE_UART0 || MIMXRT1062_SERIAL_USE_UART1 || \
      MIMXRT1062_SERIAL_USE_UART2 || MIMXRT1062_SERIAL_USE_UART3 || \
      MIMXRT1062_SERIAL_USE_UART4 || MIMXRT1062_SERIAL_USE_UART5)
#error "Serial driver activated but no UART peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Generic Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {
  /**
   * @brief Bit rate.
   */
  uint32_t                  sc_speed;
  /* End of the mandatory fields.*/
} SerialConfig;

/**
 * @brief   Generic UART register structure.
 * @note    Individual UART register blocks (even within the same chip) can differ.
 */

typedef struct {
  LPUART_Type *lpuart_p;
  lpuart_handle_t handle;
} UART_w_TypeDef;

/**
 * @brief @p SerialDriver specific data.
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
  UART_w_TypeDef            uart;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if MIMXRT1062_SERIAL_USE_UART0 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#if MIMXRT1062_SERIAL_USE_UART1 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif

#if MIMXRT1062_SERIAL_USE_UART2 && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif

#if MIMXRT1062_SERIAL_USE_UART3 && !defined(__DOXYGEN__)
extern SerialDriver SD4;
#endif

#if MIMXRT1062_SERIAL_USE_UART4 && !defined(__DOXYGEN__)
extern SerialDriver SD5;
#endif

#if MIMXRT1062_SERIAL_USE_UART5 && !defined(__DOXYGEN__)
extern SerialDriver SD6;
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

#endif /* HAL_SERIAL_LLD_H_ */

/** @} */
