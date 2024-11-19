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
 * @file    UARTv1/hal_serial_lld.c
 * @brief   WB32 low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART1 serial driver identifier.*/
#if WB32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/** @brief UART2 serial driver identifier.*/
#if WB32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/** @brief UART3 serial driver identifier.*/
#if WB32_SERIAL_USE_UART3 || defined(__DOXYGEN__)
SerialDriver SD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/* Driver default configuration.*/
static const SerialConfig default_config = {SERIAL_DEFAULT_BITRATE,
                                            UART_WordLength_8b,
                                            UART_StopBits_One,
                                            UART_Parity_None,
                                            UART_AutoFlowControl_None};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   UART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {
  uint32_t divider, apbclock;
  UART_TypeDef *u = sdp->uart;

  u->MCR = (u->MCR & 0x50) | config->UART_AutoFlowControl;

  /* Baud rate setting.*/
  if (u == UART1) {
    apbclock = WB32_PCLK1;
  }
  else {
    apbclock = WB32_PCLK2;
  }

  /* round off */
  divider = (apbclock + (config->speed >> 1)) / config->speed;

  u->DLF = divider & 0x0F;
  u->LCR = UART_LCR_DLAB;
  u->DLL = (uint8_t)(divider >> 4);
  u->DLH = (uint8_t)(divider >> 12);
  u->LCR = 0x00;

  u->LCR = config->UART_WordLength |
  	       config->UART_StopBits |
           config->UART_Parity;

  u->SRT = UART_RxFIFOThreshold_1;
  u->SFE = 0x01;

  /* Note that some bits are enforced.*/
  u->IER |= UART_IER_RDAIE | UART_IER_RLSIE;

  /* Deciding mask to be applied on the data register on receive,
     this is required in order to mask out the parity bit.*/
  sdp->rxmask = 0xFF;
}

/**
 * @brief   UART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(UART_TypeDef *u) {

#if WB32_SERIAL_USE_UART1
  if (u == UART1) {
    /* UART1 DeInit.*/
    rccResetUART1()
  }
#endif

#if WB32_SERIAL_USE_UART2
  else if (u == UART2) {
    /* UART2 DeInit.*/
    rccResetUART2()
  }
#endif

#if WB32_SERIAL_USE_UART3
  else if (u == UART3) {
    /* UART3 DeInit.*/
    rccResetUART3()
  }
#endif
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        UART SR register value
 */
static void set_error(SerialDriver *sdp, uint16_t sr) {
  eventflags_t sts = 0;

  if (sr & UART_LSR_OE)
    sts |= SD_OVERRUN_ERROR;
  if (sr & UART_LSR_PE)
    sts |= SD_PARITY_ERROR;
  if (sr & UART_LSR_FE)
    sts |= SD_FRAMING_ERROR;
  if (sr & UART_LSR_DR)
    sts |= SD_NOISE_ERROR;
  chnAddFlagsI(sdp, sts);
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
#define UART_SR_STATUS (UART_LSR_PE | UART_LSR_FE | UART_LSR_OE | UART_LSR_DR)
  UART_TypeDef *u = sdp->uart;
  uint8_t sr;

  sr = (uint8_t)u->LSR;

  /* Special case, LIN break detection.*/
  if ((sr & UART_LSR_BI) != RESET) {
    osalSysLockFromISR();
    chnAddFlagsI(sdp, SD_BREAK_DETECTED);
    osalSysUnlockFromISR();
  }

  /* Data available.*/
  osalSysLockFromISR();
  while ((sr & UART_SR_STATUS) | ((sr & UART_LSR_DR) != RESET)) {
    uint8_t b;

    /* Error condition detection.*/
    if (sr & UART_SR_STATUS)
      set_error(sdp, sr);
    b = (uint8_t)u->RBR & sdp->rxmask;
    if ((sr & UART_LSR_DR) != RESET)
      sdIncomingDataI(sdp, b);
    sr = (uint32_t)u->LSR;
  }
  osalSysUnlockFromISR();

  /* Transmission buffer empty.*/
  if (((u->IER & UART_IER_THREIE) != RESET)
      && (sr & UART_LSR_THRE) != RESET) {
    msg_t b;
    osalSysLockFromISR();
    b = oqGetI(&sdp->oqueue);
    if (b < MSG_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      u->IER &= ~(UART_IER_THREIE);
    }
    else
      u->THR = b;
    osalSysUnlockFromISR();
  }
  /* Physical transmission end.*/
  if (((u->IER & UART_IER_THREIE) != RESET)
      && (sr & UART_LSR_TEMT) != RESET) {
    osalSysLockFromISR();
    if (oqIsEmptyI(&sdp->oqueue)) {
      chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
    }
    osalSysUnlockFromISR();
  }
}

#if WB32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  UART1->IER |= UART_IER_THREIE;
}
#endif

#if WB32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  UART2->IER |= UART_IER_THREIE;
}
#endif

#if WB32_SERIAL_USE_UART3 || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

  (void)qp;
  UART3->IER |= UART_IER_THREIE;
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if WB32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
#if !defined(WB32_UART1_IRQ_VECTOR)
#error "WB32_UART1_IRQ_VECTOR not defined"
#endif
/**
 * @brief   UART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_UART1_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if WB32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
#if !defined(WB32_UART2_IRQ_VECTOR)
#error "WB32_UART2_IRQ_VECTOR not defined"
#endif
/**
 * @brief   UART2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_UART2_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if WB32_SERIAL_USE_UART3 || defined(__DOXYGEN__)
#if !defined(WB32_UART3_IRQ_VECTOR)
#error "WB32_UART3_IRQ_VECTOR not defined"
#endif
/**
 * @brief   UART3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_UART3_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD3);

  OSAL_IRQ_EPILOGUE();
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

#if WB32_SERIAL_USE_UART1
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = UART1;
#endif

#if WB32_SERIAL_USE_UART2
  sdObjectInit(&SD2, NULL, notify2);
  SD2.uart = UART2;
#endif

#if WB32_SERIAL_USE_UART3
  sdObjectInit(&SD3, NULL, notify3);
  SD3.uart = UART3;
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

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if WB32_SERIAL_USE_UART1
    if (&SD1 == sdp) {
      /* UART1 clock enable.*/
      rccEnableUART1();
      /* UART1 DeInit.*/
      rccResetUART1();
      nvicEnableVector(WB32_UART1_NUMBER, WB32_SERIAL_UART1_PRIORITY);
    }
#endif
#if WB32_SERIAL_USE_UART2
    if (&SD2 == sdp) {
      /* UART2 clock enable.*/
      rccEnableUART2();
      /* UART2 DeInit.*/
      rccResetUART2();
      nvicEnableVector(WB32_UART2_NUMBER, WB32_SERIAL_UART2_PRIORITY);
    }
#endif
#if WB32_SERIAL_USE_UART3
    if (&SD3 == sdp) {
      /* UART3 clock enable.*/
      rccEnableUART3();
      /* UART3 DeInit.*/
      rccResetUART3();
      nvicEnableVector(WB32_UART3_NUMBER, WB32_SERIAL_UART3_PRIORITY);
    }
#endif
  }
  uart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);
#if WB32_SERIAL_USE_UART1
    if (&SD1 == sdp) {
      /* UART1 DeInit.*/
      rccDisableUART1();
      nvicDisableVector(WB32_UART1_NUMBER);
      return;
    }
#endif
#if WB32_SERIAL_USE_UART2
    if (&SD2 == sdp) {
      /* UART2 DeInit.*/
      rccDisableUART2();
      nvicDisableVector(WB32_UART2_NUMBER);
      return;
    }
#endif
#if WB32_SERIAL_USE_UART3
    if (&SD3 == sdp) {
      /* UART3 DeInit.*/
      rccDisableUART3();
      nvicDisableVector(WB32_UART3_NUMBER);
      return;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
