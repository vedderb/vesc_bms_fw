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
 * @file    USART/hal_uart_lld.c
 * @brief   GD32 low level UART driver code.
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"

#if HAL_USE_UART || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define USART0_RX_DMA_CHANNEL                                               \
  GD32_DMA_GETCHANNEL(GD32_UART_USART0_RX_DMA_STREAM,                     \
                       GD32_USART0_RX_DMA_CHN)

#define USART0_TX_DMA_CHANNEL                                               \
  GD32_DMA_GETCHANNEL(GD32_UART_USART0_TX_DMA_STREAM,                     \
                       GD32_USART0_TX_DMA_CHN)

#define USART1_RX_DMA_CHANNEL                                               \
  GD32_DMA_GETCHANNEL(GD32_UART_USART1_RX_DMA_STREAM,                     \
                       GD32_USART1_RX_DMA_CHN)

#define USART1_TX_DMA_CHANNEL                                               \
  GD32_DMA_GETCHANNEL(GD32_UART_USART1_TX_DMA_STREAM,                     \
                       GD32_USART1_TX_DMA_CHN)

#define USART2_RX_DMA_CHANNEL                                               \
  GD32_DMA_GETCHANNEL(GD32_UART_USART2_RX_DMA_STREAM,                     \
                       GD32_USART2_RX_DMA_CHN)

#define USART2_TX_DMA_CHANNEL                                               \
  GD32_DMA_GETCHANNEL(GD32_UART_USART2_TX_DMA_STREAM,                     \
                       GD32_USART2_TX_DMA_CHN)

#define UART3_RX_DMA_CHANNEL                                                \
  GD32_DMA_GETCHANNEL(GD32_UART_UART3_RX_DMA_STREAM,                      \
                       GD32_UART3_RX_DMA_CHN)

#define UART3_TX_DMA_CHANNEL                                                \
  GD32_DMA_GETCHANNEL(GD32_UART_UART3_TX_DMA_STREAM,                      \
                       GD32_UART3_TX_DMA_CHN)

#define UART4_RX_DMA_CHANNEL                                                \
  GD32_DMA_GETCHANNEL(GD32_UART_UART4_RX_DMA_STREAM,                      \
                       GD32_UART4_RX_DMA_CHN)

#define UART4_TX_DMA_CHANNEL                                                \
  GD32_DMA_GETCHANNEL(GD32_UART_UART4_TX_DMA_STREAM,                      \
                       GD32_UART4_TX_DMA_CHN)

#define GD32_UART34_CR2_CHECK_MASK                                         \
  (USART_CTL1_STB_0 | USART_CTL1_CKEN | USART_CTL1_CPL | USART_CTL1_CPH |   \
   USART_CTL1_CLEN)

#define GD32_UART35_CR3_CHECK_MASK                                         \
  (USART_CTL2_CTSIE | USART_CTL2_CTSEN | USART_CTL2_RTSEN | USART_CTL2_SCEN |     \
   USART_CTL2_NKEN)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART0 UART driver identifier.*/
#if GD32_UART_USE_USART0 || defined(__DOXYGEN__)
UARTDriver UARTD1;
#endif

/** @brief USART1 UART driver identifier.*/
#if GD32_UART_USE_USART1 || defined(__DOXYGEN__)
UARTDriver UARTD2;
#endif

/** @brief USART2 UART driver identifier.*/
#if GD32_UART_USE_USART2 || defined(__DOXYGEN__)
UARTDriver UARTD3;
#endif

/** @brief UART3 UART driver identifier.*/
#if GD32_UART_USE_UART3 || defined(__DOXYGEN__)
UARTDriver UARTD4;
#endif

/** @brief UART4 UART driver identifier.*/
#if GD32_UART_USE_UART4 || defined(__DOXYGEN__)
UARTDriver UARTD5;
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
 * @param[in] stat        USART STAT register value
 *
 * @return  The error flags.
 */
static uartflags_t translate_errors(uint16_t stat) {
  uartflags_t sts = 0;

  if (stat & USART_STAT_ORERR)
    sts |= UART_OVERRUN_ERROR;
  if (stat & USART_STAT_PERR)
    sts |= UART_PARITY_ERROR;
  if (stat & USART_STAT_FERR)
    sts |= UART_FRAMING_ERROR;
  if (stat & USART_STAT_NERR)
    sts |= UART_NOISE_ERROR;
  if (stat & USART_STAT_LBDF)
    sts |= UART_BREAK_DETECTED;
  return sts;
}

/**
 * @brief   Puts the receiver in the UART_RX_IDLE state.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void uart_enter_rx_idle_loop(UARTDriver *uartp) {
  uint32_t mode;

  /* RX DMA channel preparation, if the char callback is defined then the
     TCIE interrupt is enabled too.*/
  if (uartp->config->rxchar_cb == NULL)
    mode = GD32_DMA_CTL_DIR_P2M | GD32_DMA_CTL_CMEN;
  else
    mode = GD32_DMA_CTL_DIR_P2M | GD32_DMA_CTL_CMEN | GD32_DMA_CTL_FTFIE;
  dmaStreamSetMemory0(uartp->dmarx, &uartp->rxbuf);
  dmaStreamSetTransactionSize(uartp->dmarx, 1);
  dmaStreamSetMode(uartp->dmarx, uartp->dmarxmode | mode);
  dmaStreamEnable(uartp->dmarx);
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_stop(UARTDriver *uartp) {

  /* Stops RX and TX DMA channels.*/
  dmaStreamDisable(uartp->dmarx);
  dmaStreamDisable(uartp->dmatx);

  /* Stops USART operations.*/
  uartp->usart->CTL0 = 0;
  uartp->usart->CTL1 = 0;
  uartp->usart->CTL2 = 0;
}

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void usart_start(UARTDriver *uartp) {
  uint32_t fck;
  uint16_t ctl0;
  USART_TypeDef *u = uartp->usart;

  /* Defensive programming, starting from a clean state.*/
  usart_stop(uartp);

  /* Baud rate setting.*/
  if (uartp->usart == USART0)
    fck = GD32_PCLK2 / uartp->config->speed;
  else
    fck = GD32_PCLK1 / uartp->config->speed;

  u->BAUD = fck;

  /* Resetting eventual pending status flags.*/
  (void)u->STAT;  /* SR reset step 1.*/
  (void)u->DATA;  /* SR reset step 2.*/
  u->STAT = 0;

  /* Note that some bits are enforced because required for correct driver
     operations.*/
  u->CTL1 = uartp->config->ctl1 | USART_CTL1_LBDIE;
  u->CTL2 = uartp->config->ctl2 | USART_CTL2_DENT | USART_CTL2_DENR |
                                USART_CTL2_ERRIE;

  /* Mustn't ever set TCIE here - if done, it causes an immediate
     interrupt.*/
  ctl0 = USART_CTL0_UEN | USART_CTL0_PERRIE | USART_CTL0_TEN | USART_CTL0_REN;
  u->CTL0 = uartp->config->ctl0 | ctl0;

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

  /* DMA errors handling.*/
#if defined(GD32_UART_DMA_ERROR_HOOK)
  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    GD32_UART_DMA_ERROR_HOOK(uartp);
  }
#else
  (void)flags;
#endif

  if (uartp->rxstate == UART_RX_IDLE) {
    /* Receiver in idle state, a callback is generated, if enabled, for each
       received character and then the driver stays in the same state.*/
    _uart_rx_idle_code(uartp);
  }
  else {
    /* Receiver in active state, a callback is generated, if enabled, after
       a completed transfer.*/
    dmaStreamDisable(uartp->dmarx);
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

  /* DMA errors handling.*/
#if defined(GD32_UART_DMA_ERROR_HOOK)
  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    GD32_UART_DMA_ERROR_HOOK(uartp);
  }
#else
  (void)flags;
#endif

  dmaStreamDisable(uartp->dmatx);

  /* A callback is generated, if enabled, after a completed transfer.*/
  _uart_tx1_isr_code(uartp);
}

/**
 * @brief   USART common service routine.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
static void serve_usart_irq(UARTDriver *uartp) {
  uint16_t stat;
  USART_TypeDef *u = uartp->usart;
  uint32_t ctl0 = u->CTL0;

  stat = u->STAT;   /* SR reset step 1.*/
  (void)u->DATA;  /* SR reset step 2.*/

  if (stat & (USART_STAT_LBDF | USART_STAT_ORERR | USART_STAT_NERR |
            USART_STAT_FERR  | USART_STAT_PERR)) {
    u->STAT = ~USART_STAT_LBDF;
    _uart_rx_error_isr_code(uartp, translate_errors(stat));
  }

  if ((stat & USART_STAT_TC) && (ctl0 & USART_CTL0_TCIE)) {
    /* TC interrupt cleared and disabled.*/
    u->STAT = ~USART_STAT_TC;
    u->CTL0 = ctl0 & ~USART_CTL0_TCIE;

    /* End of transmission, a callback is generated.*/
    _uart_tx2_isr_code(uartp);
  }

  /* Timeout interrupt sources are only checked if enabled in CR1.*/
  if ((ctl0 & USART_CTL0_IDLEIE) && (stat & USART_STAT_IDLEF)) {
    _uart_timeout_isr_code(uartp);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if GD32_UART_USE_USART0 || defined(__DOXYGEN__)
#if !defined(GD32_USART0_HANDLER)
#error "GD32_USART0_HANDLER not defined"
#endif
/**
 * @brief   USART0 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_USART0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_usart_irq(&UARTD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_UART_USE_USART0 */

#if GD32_UART_USE_USART1 || defined(__DOXYGEN__)
#if !defined(GD32_USART1_HANDLER)
#error "GD32_USART1_HANDLER not defined"
#endif
/**
 * @brief   USART1 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_USART1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_usart_irq(&UARTD2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_UART_USE_USART1 */

#if GD32_UART_USE_USART2 || defined(__DOXYGEN__)
#if !defined(GD32_USART2_HANDLER)
#error "GD32_USART2_HANDLER not defined"
#endif
/**
 * @brief   USART2 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_USART2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_usart_irq(&UARTD3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_UART_USE_USART2 */

#if GD32_UART_USE_UART3 || defined(__DOXYGEN__)
#if !defined(GD32_UART3_HANDLER)
#error "GD32_UART3_HANDLER not defined"
#endif
/**
 * @brief   UART3 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_UART3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_usart_irq(&UARTD4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_UART_USE_UART3 */

#if GD32_UART_USE_UART4 || defined(__DOXYGEN__)
#if !defined(GD32_UART4_HANDLER)
#error "GD32_UART4_HANDLER not defined"
#endif
/**
 * @brief   UART4 IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_UART4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_usart_irq(&UARTD5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_UART_USE_UART4 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {

#if GD32_UART_USE_USART0
  uartObjectInit(&UARTD1);
  UARTD1.usart   = USART0;
  UARTD1.dmarxmode = GD32_DMA_CTL_ERRIE;
  UARTD1.dmatxmode = GD32_DMA_CTL_ERRIE;
  UARTD1.dmarx   = NULL;
  UARTD1.dmatx   = NULL;
#endif

#if GD32_UART_USE_USART1
  uartObjectInit(&UARTD2);
  UARTD2.usart   = USART1;
  UARTD2.dmarxmode = GD32_DMA_CTL_ERRIE;
  UARTD2.dmatxmode = GD32_DMA_CTL_ERRIE;
  UARTD2.dmarx   = NULL;
  UARTD2.dmatx   = NULL;
#endif

#if GD32_UART_USE_USART2
  uartObjectInit(&UARTD3);
  UARTD3.usart   = USART2;
  UARTD3.dmarxmode = GD32_DMA_CTL_ERRIE;
  UARTD3.dmatxmode = GD32_DMA_CTL_ERRIE;
  UARTD3.dmarx   = NULL;
  UARTD3.dmatx   = NULL;
#endif

#if GD32_UART_USE_UART3
  uartObjectInit(&UARTD4);
  UARTD4.usart   = UART3;
  UARTD4.dmarxmode = GD32_DMA_CTL_ERRIE;
  UARTD4.dmatxmode = GD32_DMA_CTL_ERRIE;
  UARTD4.dmarx   = NULL;
  UARTD4.dmatx   = NULL;
#endif

#if GD32_UART_USE_UART4
  uartObjectInit(&UARTD5);
  UARTD5.usart   = UART4;
  UARTD5.dmarxmode = GD32_DMA_CTL_ERRIE;
  UARTD5.dmatxmode = GD32_DMA_CTL_ERRIE;
  UARTD5.dmarx   = NULL;
  UARTD5.dmatx   = NULL;
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
#if GD32_UART_USE_USART0
    if (&UARTD1 == uartp) {
      uartp->dmarx = dmaStreamAllocI(GD32_UART_USART0_RX_DMA_STREAM,
                                     GD32_UART_USART0_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(GD32_UART_USART0_TX_DMA_STREAM,
                                     GD32_UART_USART0_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      rcuEnableUSART0(true);
      eclicEnableVector(GD32_USART0_NUMBER, GD32_UART_USART0_IRQ_PRIORITY, GD32_UART_USART0_IRQ_TRIGGER);
      uartp->dmarxmode |= GD32_DMA_CTL_CHSEL(USART0_RX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_USART0_DMA_PRIORITY);
      uartp->dmatxmode |= GD32_DMA_CTL_CHSEL(USART0_TX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_USART0_DMA_PRIORITY);
    }
#endif

#if GD32_UART_USE_USART1
    if (&UARTD2 == uartp) {
      uartp->dmarx = dmaStreamAllocI(GD32_UART_USART1_RX_DMA_STREAM,
                                     GD32_UART_USART1_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(GD32_UART_USART1_TX_DMA_STREAM,
                                     GD32_UART_USART1_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      rcuEnableUSART1(true);
      eclicEnableVector(GD32_USART1_NUMBER, GD32_UART_USART1_IRQ_PRIORITY, GD32_UART_USART1_IRQ_TRIGGER);
      uartp->dmarxmode |= GD32_DMA_CTL_CHSEL(USART1_RX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_USART1_DMA_PRIORITY);
      uartp->dmatxmode |= GD32_DMA_CTL_CHSEL(USART1_TX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_USART1_DMA_PRIORITY);
    }
#endif

#if GD32_UART_USE_USART2
    if (&UARTD3 == uartp) {
      uartp->dmarx = dmaStreamAllocI(GD32_UART_USART2_RX_DMA_STREAM,
                                     GD32_UART_USART2_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(GD32_UART_USART2_TX_DMA_STREAM,
                                     GD32_UART_USART2_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      rcuEnableUSART2(true);
      eclicEnableVector(GD32_USART2_NUMBER, GD32_UART_USART2_IRQ_PRIORITY, GD32_UART_USART2_IRQ_TRIGGER);
      uartp->dmarxmode |= GD32_DMA_CTL_CHSEL(USART2_RX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_USART2_DMA_PRIORITY);
      uartp->dmatxmode |= GD32_DMA_CTL_CHSEL(USART2_TX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_USART2_DMA_PRIORITY);
    }
#endif

#if GD32_UART_USE_UART3
    if (&UARTD4 == uartp) {

      osalDbgAssert((uartp->config->ctl1 & GD32_UART34_CR2_CHECK_MASK) == 0,
                    "specified invalid bits in UART3 CR2 register settings");
      osalDbgAssert((uartp->config->ctl2 & GD32_UART35_CR3_CHECK_MASK) == 0,
                    "specified invalid bits in UART3 CR3 register settings");

      uartp->dmarx = dmaStreamAllocI(GD32_UART_UART3_RX_DMA_STREAM,
                                     GD32_UART_UART3_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(GD32_UART_UART3_TX_DMA_STREAM,
                                     GD32_UART_UART3_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      rcuEnableUART3(true);
      eclicEnableVector(GD32_UART3_NUMBER, GD32_UART_UART3_IRQ_PRIORITY, GD32_UART_UART3_IRQ_TRIGGER);
      uartp->dmarxmode |= GD32_DMA_CTL_CHSEL(UART3_RX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_UART3_DMA_PRIORITY);
      uartp->dmatxmode |= GD32_DMA_CTL_CHSEL(UART3_TX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_UART3_DMA_PRIORITY);
    }
#endif

#if GD32_UART_USE_UART4
    if (&UARTD5 == uartp) {

      osalDbgAssert((uartp->config->ctl1 & GD32_UART34_CR2_CHECK_MASK) == 0,
                    "specified invalid bits in UART4 CR2 register settings");
      osalDbgAssert((uartp->config->ctl2 & GD32_UART35_CR3_CHECK_MASK) == 0,
                    "specified invalid bits in UART4 CR3 register settings");

      uartp->dmarx = dmaStreamAllocI(GD32_UART_UART4_RX_DMA_STREAM,
                                     GD32_UART_UART4_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_rx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmarx != NULL, "unable to allocate stream");
      uartp->dmatx = dmaStreamAllocI(GD32_UART_UART4_TX_DMA_STREAM,
                                     GD32_UART_UART4_IRQ_PRIORITY,
                                     (gd32_dmaisr_t)uart_lld_serve_tx_end_irq,
                                     (void *)uartp);
      osalDbgAssert(uartp->dmatx != NULL, "unable to allocate stream");

      rcuEnableUART4(true);
      eclicEnableVector(GD32_UART4_NUMBER, GD32_UART_UART4_IRQ_PRIORITY, GD32_UART_UART4_IRQ_TRIGGER);
      uartp->dmarxmode |= GD32_DMA_CTL_CHSEL(UART4_RX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_UART4_DMA_PRIORITY);
      uartp->dmatxmode |= GD32_DMA_CTL_CHSEL(UART4_TX_DMA_CHANNEL) |
                          GD32_DMA_CTL_PRIO(GD32_UART_UART4_DMA_PRIORITY);
    }
#endif

    /* Static DMA setup, the transfer size depends on the USART settings,
       it is 16 bits if M=1 and PCE=0 else it is 8 bits.*/
    if ((uartp->config->ctl0 & (USART_CTL0_WL | USART_CTL0_PCEN)) == USART_CTL0_WL) {
      uartp->dmarxmode |= GD32_DMA_CTL_PWIDTH_HWORD | GD32_DMA_CTL_MWIDTH_HWORD;
      uartp->dmatxmode |= GD32_DMA_CTL_PWIDTH_HWORD | GD32_DMA_CTL_MWIDTH_HWORD;
    }
    dmaStreamSetPeripheral(uartp->dmarx, &uartp->usart->DATA);
    dmaStreamSetPeripheral(uartp->dmatx, &uartp->usart->DATA);
    uartp->rxbuf = 0;
  }

  uartp->rxstate = UART_RX_IDLE;
  uartp->txstate = UART_TX_IDLE;
  usart_start(uartp);
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
    usart_stop(uartp);
    dmaStreamFreeI(uartp->dmarx);
    dmaStreamFreeI(uartp->dmatx);
    uartp->dmarx = NULL;
    uartp->dmatx = NULL;

#if GD32_UART_USE_USART0
    if (&UARTD1 == uartp) {
      eclicDisableVector(GD32_USART0_NUMBER);
      rcuDisableUSART0();
      return;
    }
#endif

#if GD32_UART_USE_USART1
    if (&UARTD2 == uartp) {
      eclicDisableVector(GD32_USART1_NUMBER);
      rcuDisableUSART1();
      return;
    }
#endif

#if GD32_UART_USE_USART2
    if (&UARTD3 == uartp) {
      eclicDisableVector(GD32_USART2_NUMBER);
      rcuDisableUSART2();
      return;
    }
#endif

#if GD32_UART_USE_UART3
    if (&UARTD4 == uartp) {
      eclicDisableVector(GD32_UART3_NUMBER);
      rcuDisableUART3();
      return;
    }
#endif

#if GD32_UART_USE_UART4
    if (&UARTD5 == uartp) {
      eclicDisableVector(GD32_UART4_NUMBER);
      rcuDisableUART4();
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

  /* TX DMA channel preparation.*/
  dmaStreamSetMemory0(uartp->dmatx, txbuf);
  dmaStreamSetTransactionSize(uartp->dmatx, n);
  dmaStreamSetMode(uartp->dmatx, uartp->dmatxmode  | GD32_DMA_CTL_DIR_M2P |
                                 GD32_DMA_CTL_MNAGA | GD32_DMA_CTL_FTFIE);

  /* Only enable TC interrupt if there's a callback attached to it or
     if called from uartSendFullTimeout(). Also we need to clear TC flag
     which could be set before.*/
#if UART_USE_WAIT == TRUE
  if ((uartp->config->txend2_cb != NULL) || (uartp->early == false)) {
#else
  if (uartp->config->txend2_cb != NULL) {
#endif
    uartp->usart->STAT = ~USART_STAT_TC;
    uartp->usart->CTL0 |= USART_CTL0_TCIE;
  }

  /* Starting transfer.*/
  dmaStreamEnable(uartp->dmatx);
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

  dmaStreamDisable(uartp->dmatx);

  return dmaStreamGetTransactionSize(uartp->dmatx);
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

  /* Stopping previous activity (idle state).*/
  dmaStreamDisable(uartp->dmarx);

  /* RX DMA channel preparation.*/
  dmaStreamSetMemory0(uartp->dmarx, rxbuf);
  dmaStreamSetTransactionSize(uartp->dmarx, n);
  dmaStreamSetMode(uartp->dmarx, uartp->dmarxmode  | GD32_DMA_CTL_DIR_P2M |
                                 GD32_DMA_CTL_MNAGA | GD32_DMA_CTL_FTFIE);

  /* Starting transfer.*/
  dmaStreamEnable(uartp->dmarx);
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
  size_t n;

  dmaStreamDisable(uartp->dmarx);
  n = dmaStreamGetTransactionSize(uartp->dmarx);
  uart_enter_rx_idle_loop(uartp);

  return n;
}

#endif /* HAL_USE_UART */

/** @} */
