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
 * @file    USART/hal_serial_lld.c
 * @brief   GD32 low level serial driver code.
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

/** @brief USART0 serial driver identifier.*/
#if GD32_SERIAL_USE_USART0 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/** @brief USART1 serial driver identifier.*/
#if GD32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/** @brief USART2 serial driver identifier.*/
#if GD32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
SerialDriver SD3;
#endif

/** @brief UART3 serial driver identifier.*/
#if GD32_SERIAL_USE_UART3 || defined(__DOXYGEN__)
SerialDriver SD4;
#endif

/** @brief UART4 serial driver identifier.*/
#if GD32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
SerialDriver SD5;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config =
{
  SERIAL_DEFAULT_BITRATE,
  0,
  USART_CTL1_STB1_BITS,
  0
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void usart_init(SerialDriver *sdp, const SerialConfig *config) {
  uint32_t fck;
  USART_TypeDef *u = sdp->usart;

  /* Baud rate setting.*/
  if (sdp->usart == USART0)
    fck = GD32_PCLK2 / config->speed;
  else
    fck = GD32_PCLK1 / config->speed;

  u->BAUD = fck;

  /* Note that some bits are enforced.*/
  u->CTL1 = config->ctl1 | USART_CTL1_LBDIE;
  u->CTL2 = config->ctl2 | USART_CTL2_ERRIE;
  u->CTL0 = config->ctl0 | USART_CTL0_UEN | USART_CTL0_PERRIE |
                         USART_CTL0_RBNEIE | USART_CTL0_TEN |
                         USART_CTL0_REN;
  u->STAT = 0;
  (void)u->STAT;  /* SR reset step 1.*/
  (void)u->DATA;  /* SR reset step 2.*/

  /* Deciding mask to be applied on the data register on receive, this is
     required in order to mask out the parity bit.*/
  if ((config->ctl0 & (USART_CTL0_WL | USART_CTL0_PCEN)) == USART_CTL0_PCEN) {
    sdp->rxmask = 0x7F;
  }
  else {
    sdp->rxmask = 0xFF;
  }
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an USART I/O block
 */
static void usart_deinit(USART_TypeDef *u) {

  u->CTL0 = 0;
  u->CTL1 = 0;
  u->CTL2 = 0;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] stat        USART STAT register value
 */
static void set_error(SerialDriver *sdp, uint16_t stat) {
  eventflags_t sts = 0;

  if (stat & USART_STAT_ORERR)
    sts |= SD_OVERRUN_ERROR;
  if (stat & USART_STAT_PERR)
    sts |= SD_PARITY_ERROR;
  if (stat & USART_STAT_FERR)
    sts |= SD_FRAMING_ERROR;
  if (stat & USART_STAT_NERR)
    sts |= SD_NOISE_ERROR;
  chnAddFlagsI(sdp, sts);
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the USART
 */
static void serve_interrupt(SerialDriver *sdp) {
  USART_TypeDef *u = sdp->usart;
  uint16_t ctl0 = u->CTL0;
  uint16_t stat = u->STAT;

  /* Special case, LIN break detection.*/
  if (stat & USART_STAT_LBDF) {
    osalSysLockFromISR();
    chnAddFlagsI(sdp, SD_BREAK_DETECTED);
    u->STAT = ~USART_STAT_LBDF;
    osalSysUnlockFromISR();
  }

  /* Data available.*/
  osalSysLockFromISR();
  while (stat & (USART_STAT_RBNE | USART_STAT_ORERR | USART_STAT_NERR | USART_STAT_FERR |
               USART_STAT_PERR)) {
    uint8_t b;

    /* Error condition detection.*/
    if (stat & (USART_STAT_ORERR | USART_STAT_NERR | USART_STAT_FERR  | USART_STAT_PERR))
      set_error(sdp, stat);
    b = (uint8_t)u->DATA & sdp->rxmask;
    if (stat & USART_STAT_RBNE)
      sdIncomingDataI(sdp, b);
    stat = u->STAT;
  }
  osalSysUnlockFromISR();

  /* Transmission buffer empty.*/
  if ((ctl0 & USART_CTL0_TBEIE) && (stat & USART_STAT_TBE)) {
    msg_t b;
    osalSysLockFromISR();
    b = oqGetI(&sdp->oqueue);
    if (b < MSG_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      u->CTL0 = ctl0 & ~USART_CTL0_TBEIE;
    }
    else
      u->DATA = b;
    osalSysUnlockFromISR();
  }

  /* Physical transmission end.*/
  if ((ctl0 & USART_CTL0_TCIE) && (stat & USART_STAT_TC)) {
    osalSysLockFromISR();
    if (oqIsEmptyI(&sdp->oqueue)) {
      chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
      u->CTL0 = ctl0 & ~USART_CTL0_TCIE;
    }
    osalSysUnlockFromISR();
  }
}

#if GD32_SERIAL_USE_USART0 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  USART0->CTL0 |= USART_CTL0_TBEIE | USART_CTL0_TCIE;
}
#endif

#if GD32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  USART1->CTL0 |= USART_CTL0_TBEIE | USART_CTL0_TCIE;
}
#endif

#if GD32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

  (void)qp;
  USART2->CTL0 |= USART_CTL0_TBEIE | USART_CTL0_TCIE;
}
#endif

#if GD32_SERIAL_USE_UART3 || defined(__DOXYGEN__)
static void notify4(io_queue_t *qp) {

  (void)qp;
  UART3->CTL0 |= USART_CTL0_TBEIE | USART_CTL0_TCIE;
}
#endif

#if GD32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
static void notify5(io_queue_t *qp) {

  (void)qp;
  UART4->CTL0 |= USART_CTL0_TBEIE | USART_CTL0_TCIE;
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if GD32_SERIAL_USE_USART0 || defined(__DOXYGEN__)
#if !defined(GD32_USART0_HANDLER)
#error "GD32_USART0_HANDLER not defined"
#endif
/**
 * @brief   USART0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_USART0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if GD32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
#if !defined(GD32_USART1_HANDLER)
#error "GD32_USART1_HANDLER not defined"
#endif
/**
 * @brief   USART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_USART1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if GD32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
#if !defined(GD32_USART2_HANDLER)
#error "GD32_USART2_HANDLER not defined"
#endif
/**
 * @brief   USART2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_USART2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if GD32_SERIAL_USE_UART3 || defined(__DOXYGEN__)
#if !defined(GD32_UART3_HANDLER)
#error "GD32_UART3_HANDLER not defined"
#endif
/**
 * @brief   UART3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_UART3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if GD32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
#if !defined(GD32_UART4_HANDLER)
#error "GD32_UART4_HANDLER not defined"
#endif
/**
 * @brief   UART4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_UART4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD5);

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

#if GD32_SERIAL_USE_USART0
  sdObjectInit(&SD1, NULL, notify1);
  SD1.usart = USART0;
#endif

#if GD32_SERIAL_USE_USART1
  sdObjectInit(&SD2, NULL, notify2);
  SD2.usart = USART1;
#endif

#if GD32_SERIAL_USE_USART2
  sdObjectInit(&SD3, NULL, notify3);
  SD3.usart = USART2;
#endif

#if GD32_SERIAL_USE_UART3
  sdObjectInit(&SD4, NULL, notify4);
  SD4.usart = UART3;
#endif

#if GD32_SERIAL_USE_UART4
  sdObjectInit(&SD5, NULL, notify5);
  SD5.usart = UART4;
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
#if GD32_SERIAL_USE_USART0
    if (&SD1 == sdp) {
      rcuEnableUSART0(true);
      eclicEnableVector(GD32_USART0_NUMBER, GD32_SERIAL_USART0_PRIORITY, GD32_SERIAL_USART0_TRIGGER);
    }
#endif
#if GD32_SERIAL_USE_USART1
    if (&SD2 == sdp) {
      rcuEnableUSART1(true);
      eclicEnableVector(GD32_USART1_NUMBER, GD32_SERIAL_USART1_PRIORITY, GD32_SERIAL_USART1_TRIGGER);
    }
#endif
#if GD32_SERIAL_USE_USART2
    if (&SD3 == sdp) {
      rcuEnableUSART2(true);
      eclicEnableVector(GD32_USART2_NUMBER, GD32_SERIAL_USART2_PRIORITY, GD32_SERIAL_USART2_TRIGGER);
    }
#endif
#if GD32_SERIAL_USE_UART3
    if (&SD4 == sdp) {
      rcuEnableUART3(true);
      eclicEnableVector(GD32_UART3_NUMBER, GD32_SERIAL_UART3_PRIORITY, GD32_SERIAL_UART3_TRIGGER);
    }
#endif
#if GD32_SERIAL_USE_UART4
    if (&SD5 == sdp) {
      rcuEnableUART4(true);
      eclicEnableVector(GD32_UART4_NUMBER, GD32_SERIAL_UART4_PRIORITY, GD32_SERIAL_UART4_TRIGGER);
    }
#endif
  }
  usart_init(sdp, config);
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
    usart_deinit(sdp->usart);
#if GD32_SERIAL_USE_USART0
    if (&SD1 == sdp) {
      rcuDisableUSART0();
      eclicDisableVector(GD32_USART0_NUMBER);
      return;
    }
#endif
#if GD32_SERIAL_USE_USART1
    if (&SD2 == sdp) {
      rcuDisableUSART1();
      eclicDisableVector(GD32_USART1_NUMBER);
      return;
    }
#endif
#if GD32_SERIAL_USE_USART2
    if (&SD3 == sdp) {
      rcuDisableUSART2();
      eclicDisableVector(GD32_USART2_NUMBER);
      return;
    }
#endif
#if GD32_SERIAL_USE_UART3
    if (&SD4 == sdp) {
      rcuDisableUART3();
      eclicDisableVector(GD32_UART3_NUMBER);
      return;
    }
#endif
#if GD32_SERIAL_USE_UART4
    if (&SD5 == sdp) {
      rcuDisableUART4();
      eclicDisableVector(GD32_UART4_NUMBER);
      return;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
