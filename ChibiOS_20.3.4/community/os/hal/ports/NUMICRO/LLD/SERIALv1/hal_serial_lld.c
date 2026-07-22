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
 * @file    hal_serial_lld.c
 * @brief   NUC123 serial subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_SERIAL) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define NUC123_BRD_MAX       0xFFFFUL
#define NUC123_DIVIDER_X_MAX 0xF

#define NUC123_BRD_MODE0 ((0UL << UART_BAUD_DIV_X_EN_Pos) | (0UL << UART_BAUD_DIV_X_ONE_Pos))
#define NUC123_BRD_MODE1 ((1UL << UART_BAUD_DIV_X_EN_Pos) | (0UL << UART_BAUD_DIV_X_ONE_Pos))
#define NUC123_BRD_MODE2 ((1UL << UART_BAUD_DIV_X_EN_Pos) | (1UL << UART_BAUD_DIV_X_ONE_Pos))

/* Initially, these may not be defined in the header file so that they are not
used while unsupported. They are defined here because they are needed for safety
checks internally. When they are eventually
defined in the header file, they can be removed here */
#if !defined(NUC123_SERIAL_MODE_IrDA)
#define NUC123_SERIAL_MODE_IrDA (2UL << UART_FUN_SEL_FUN_SEL_Pos)
#endif

#if !defined(NUC123_SERIAL_MODE_RS485)
#define NUC123_SERIAL_MODE_RS485 (3UL << UART_FUN_SEL_FUN_SEL_Pos)
#endif

#define UART1_nCTS_PinSelect()                                              \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB7_MFP1_Msk;                              \
    SYS->GPB_MFP |= (1 << 7);                                               \
  } while (0)

#define UART1_nCTS_PinUnselect()                                            \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB7_MFP1_Msk;                              \
    SYS->GPB_MFP &= ~(1 << 7);                                              \
  } while (0)

#define UART1_nRTS_PinSelect()                                              \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB6_MFP1_Msk;                              \
    SYS->GPB_MFP |= (1 << 6);                                               \
  } while (0)

#define UART1_nRTS_PinUnselect()                                            \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB6_MFP1_Msk;                              \
    SYS->GPB_MFP &= ~(1 << 6);                                              \
  } while (0)

#define UART1_TXD_PinSelect()                                               \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB5_MFP1_Msk;                              \
    SYS->GPB_MFP |= (1 << 5);                                               \
  } while (0)

#define UART1_TXD_PinUnselect()                                             \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB5_MFP1_Msk;                              \
    SYS->GPB_MFP &= ~(1 << 5);                                              \
  } while (0)

#define UART1_RXD_PinSelect()                                               \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB4_MFP1_Msk;                              \
    SYS->GPB_MFP |= (1 << 4);                                               \
  } while (0)

#define UART1_RXD_PinUnselect()                                             \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB4_MFP1_Msk;                              \
    SYS->GPB_MFP &= ~(1 << 4);                                              \
  } while (0)

#define UART0_nCTS_PinSelect()                                              \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB3_MFP1_Msk;                              \
    SYS->GPB_MFP |= (1 << 3);                                               \
  } while (0)

#define UART0_nCTS_PinUnselect()                                            \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB3_MFP1_Msk;                              \
    SYS->GPB_MFP &= ~(1 << 3);                                              \
  } while (0)

#define UART0_nRTS_PinSelect()                                              \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB2_MFP1_Msk;                              \
    SYS->GPB_MFP |= (1 << 2);                                               \
  } while (0)

#define UART0_nRTS_PinUnselect()                                            \
  do {                                                                      \
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB2_MFP1_Msk;                              \
    SYS->GPB_MFP &= ~(1 << 2);                                              \
  } while (0)

#define UART0_TXD_PinSelect()                                               \
  do {                                                                      \
    SYS->GPB_MFP |= (1 << 1);                                               \
  } while (0)

#define UART0_TXD_PinUnselect()                                             \
  do {                                                                      \
    SYS->GPB_MFP &= ~(1 << 1);                                              \
  } while (0)

#define UART0_RXD_PinSelect()                                               \
  do {                                                                      \
    SYS->GPB_MFP |= (1 << 0);                                               \
  } while (0)

#define UART0_RXD_PinUnselect()                                             \
  do {                                                                      \
    SYS->GPB_MFP &= ~(1 << 0);                                              \
  } while (0)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART0 serial driver identifier.*/
#if (NUC123_SERIAL_USE_UART0) || defined(__DOXYGEN__)
SerialDriver SD0;
#endif

/** @brief UART1 serial driver identifier.*/
#if (NUC123_SERIAL_USE_UART1) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {SERIAL_DEFAULT_BITRATE,
                                            NUC123_SERIAL_MODE_DEFAULT,
                                            NUC123_SERIAL_DATA_8BITS,
                                            NUC123_SERIAL_PARITY_N,
                                            NUC123_SERIAL_STOP_1};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 *
 * @notapi
 */
static void serial_lld_serve_interrupt(SerialDriver* sdp)
{
  /* For now, this assumes UART mode */
  uint32_t isr = sdp->uart->ISR;
  uint32_t fsr = sdp->uart->FSR;

  if (isr & (UART_ISR_RLS_IF_Msk | UART_ISR_BUF_ERR_IF_Msk)) {
    osalSysLockFromISR();

    if (fsr & UART_FSR_BIF_Msk) {
      chnAddFlagsI(sdp, SD_BREAK_DETECTED);
    }

    if (fsr & UART_FSR_FEF_Msk) {
      chnAddFlagsI(sdp, SD_FRAMING_ERROR);
    }

    if (fsr & UART_FSR_PEF_Msk) {
      chnAddFlagsI(sdp, SD_PARITY_ERROR);
    }

    if (fsr & UART_FSR_RX_OVER_IF_Msk) {
      chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
    }

    /* Technically, a TX_OVER would trigger BUFF_ERR, and reach this code.
    We'll come back to how to handle that.
    */

    sdp->uart->FSR |=
        (UART_FSR_BIF_Msk | UART_FSR_FEF_Msk | UART_FSR_PEF_Msk | UART_FSR_RX_OVER_IF_Msk);
    osalSysUnlockFromISR();
  }

  osalSysLockFromISR();
  while (!(sdp->uart->FSR & UART_FSR_RX_EMPTY_Msk)) {
    /* Originally
      sdIncomingDataI(sdp, sdp->uart->RBR);
    copied as per instructions in hal_serial.c for performance gain */
    osalDbgCheckClassI();
    osalDbgCheck(sdp != NULL);

    if (iqIsEmptyI(&sdp->iqueue))
      chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
    if (iqPutI(&sdp->iqueue, sdp->uart->RBR) < MSG_OK)
      chnAddFlagsI(sdp, SD_QUEUE_FULL_ERROR);
  }
  osalSysUnlockFromISR();

  osalSysLockFromISR();
  while (!(sdp->uart->FSR & UART_FSR_TX_FULL_Msk)) {
    msg_t b = oqGetI(&(sdp->oqueue));
    if (b < MSG_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      sdp->uart->IER &= ~(UART_IER_THRE_IEN_Msk);
      break;
    }
    sdp->uart->THR = b;
  }
  chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
  osalSysUnlockFromISR();

}

#if NUC123_SERIAL_USE_UART0 || defined(__DOXYGEN__)
static void notify0(io_queue_t* qp)
{
  (void)qp;
  UART0->IER |= UART_IER_THRE_IEN_Msk;
}
#endif

#if NUC123_SERIAL_USE_UART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t* qp)
{
  (void)qp;
  UART1->IER |= UART_IER_THRE_IEN_Msk;
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   UART0 interrupt handler.
 *
 * @isr
 */
#if NUC123_SERIAL_USE_UART0 || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(NUC123_UART0_HANDLER)
{
  OSAL_IRQ_PROLOGUE();
  serial_lld_serve_interrupt(&SD0);
  OSAL_IRQ_EPILOGUE();
}

#endif

/**
 * @brief   UART1 interrupt handler.
 *
 * @isr
 */
#if NUC123_SERIAL_USE_UART1 || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(NUC123_UART1_HANDLER)
{
  OSAL_IRQ_PROLOGUE();
  serial_lld_serve_interrupt(&SD1);
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
void sd_lld_init(void)
{
#if NUC123_SERIAL_USE_UART0 || NUC123_SERIAL_USE_UART1
  CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_UART_S_Msk)) | NUC123_SERIAL_CLKSRC;
#endif

#if NUC123_SERIAL_USE_UART0
  sdObjectInit(&SD0, NULL, notify0);
  /* Select UART0 Pins */
#if defined(NUC123xxxANx) && !defined(NUC123xxxAEx)
  SYS->ALT_MFP &= ~(SYS_ALT_MFP_PB3_MFP1_Msk | SYS_ALT_MFP_PB2_MFP1_Msk);
  /* SYS->GPB_MFP |= SYS_GPB_MFP_PB1_UART0_TXD | SYS_GPB_MFP_PB0_UART0_RXD |
                    SYS_GPB_MFP_PB3_UART0_nCTS | SYS_GPB_MFP_PB2_UART0_nRTS; */
  SYS->GPB_MFP |= 0x0FUL;
#endif
#if defined(NUC123xxxAEx)
  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~0x000000FF) | 0x00000011;
#endif
  SD0.uart = UART0;
#endif

#if NUC123_SERIAL_USE_UART1
  sdObjectInit(&SD1, NULL, notify1);
  /* Select UART1 Pins */
#if defined(NUC123xxxANx) && !defined(NUC123xxxAEx)
  SYS->ALT_MFP &= ~(SYS_ALT_MFP_PB7_MFP1_Msk | SYS_ALT_MFP_PB6_MFP1_Msk |
                    SYS_ALT_MFP_PB5_MFP1_Msk | SYS_ALT_MFP_PB4_MFP1_Msk);
  /* SYS->GPB_MFP |= SYS_GPB_MFP_PB5_UART1_TXD | SYS_GPB_MFP_PB4_UART1_RXD |
                    SYS_GPB_MFP_PB7_UART1_nCTS | SYS_GPB_MFP_PB6_UART1_nRTS; */
  SYS->GPB_MFP |= 0xF0UL;
#endif
#if defined(NUC123xxxAEx)
  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~0x00FF0000) | 0x00110000;
#endif
  SD1.uart = UART1;
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
void sd_lld_start(SerialDriver* sdp, const SerialConfig* config)
{

  if (!config) {
    config = &default_config;
  }

  if (sdp->state == SD_STOP) {
#if NUC123_SERIAL_USE_UART0
    if (&SD0 == sdp) {
      CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;
      nvicEnableVector(NUC123_UART0_NUMBER, NUC123_SERIAL_UART0_IRQ_PRIORITY);
      SYS->IPRSTC2 |= SYS_IPRSTC2_UART0_RST_Msk;
      SYS->IPRSTC2 &= ~(SYS_IPRSTC2_UART0_RST_Msk);
    }
#endif
#if NUC123_SERIAL_USE_UART1
    if (&SD1 == sdp) {
      CLK->APBCLK |= CLK_APBCLK_UART1_EN_Msk;
      nvicEnableVector(NUC123_UART1_NUMBER, NUC123_SERIAL_UART1_IRQ_PRIORITY);
      SYS->IPRSTC2 |= SYS_IPRSTC2_UART1_RST_Msk;
      SYS->IPRSTC2 &= ~(SYS_IPRSTC2_UART1_RST_Msk);
    }
#endif
  }

  /* Configures the peripheral.*/

  uint32_t baud_found = false;

  /* Speed is controlled by the expression
      BAUD = UART_CLK / (m * (BRD + 2))
  Here, baud_denom represents the (m * (BRD + 2)) part of the expression */
  uint32_t baudrate = config->speed;
  uint32_t baud_denom = (NUC123_SERIAL_CLK + (baudrate - 1) / 2) / baudrate;

  /* Generally it is better to have whatever output than nothing. */
#if FALSE
  osalDbgAssert((NUC123_SERIAL_CLK / baud_denom) >= baudrate - (baudrate>>6) &&
                (NUC123_SERIAL_CLK / baud_denom) <= baudrate + (baudrate>>6),
                "Error is too large with the BAUDRATE");
#endif

  /* Mode 0: m = 16 */
  if ((baud_denom % 16 == 0) && (((baud_denom / 16) - 2) <= NUC123_BRD_MAX)) {
    sdp->uart->BAUD = NUC123_BRD_MODE0 | ((baud_denom / 16) - 2);
    baud_found      = true;
  }

  /* If the peripheral is in IrDA mode, it can only use baud generation mode 0 */
  else if (config->mode != NUC123_SERIAL_MODE_IrDA) {

    /* Mode 2: m = 1. The exact behavior of mode 2 varies depending on
        a) the model of the chip
        b) UART_CLK relative to HCLK
    */
#if defined(NUC123xxxANx)
#if NUC123_SERIAL_CLK <= NUC123_HCLK
    if ((9 <= (baud_denom - 2)) && ((baud_denom - 2) <= NUC123_BRD_MAX)) {
      sdp->uart->BAUD = NUC123_BRD_MODE2 | (baud_denom - 2);
      baud_found      = true;
    }
#elif NUC123_SERIAL_CLK <= (2 * NUC123_HCLK)
    if ((15 <= (baud_denom - 2)) && ((baud_denom - 2) <= NUC123_BRD_MAX)) {
      sdp->uart->BAUD = NUC123_BRD_MODE2 | (baud_denom - 2);
      baud_found      = true;
    }
#elif NUC123_SERIAL_CLK <= (3 * NUC123_HCLK)
    if ((21 <= (baud_denom - 2)) && ((baud_denom - 2) <= NUC123_BRD_MAX)) {
      sdp->uart->BAUD = NUC123_BRD_MODE2 | (baud_denom - 2);
      baud_found      = true;
    }
#else
    /* Dummy statement to prevent an orphaned else at the begining of the Mode 1 block */
    if (FALSE) {
    }
#endif
#elif defined(NUC123xxxAEx)
#if NUC123_SERIAL_CLK <= (3 * NUC123_HCLK)
    if ((9 <= (baud_denom - 2)) && ((baud_denom - 2) <= NUC123_BRD_MAX)) {
      sdp->uart->BAUD = NUC123_BRD_MODE2 | (baud_denom - 2);
      baud_found      = true;
    }
#elif NUC123_SERIAL_CLK <= (4 * NUC123_HCLK)
    if ((11 <= (baud_denom - 2)) && ((baud_denom - 2) <= NUC123_BRD_MAX)) {
      sdp->uart->BAUD = NUC123_BRD_MODE2 | (baud_denom - 2);
      baud_found      = true;
    }
#elif NUC123_SERIAL_CLK <= (5 * NUC123_HCLK)
    if ((14 <= (baud_denom - 2)) && ((baud_denom - 2) <= NUC123_BRD_MAX)) {
      sdp->uart->BAUD = NUC123_BRD_MODE2 | (baud_denom - 2);
      baud_found      = true;
    }
#else
    /* Dummy statement to prevent an orphaned else at the begining of the Mode 1 block */
    if (FALSE) {
    }
#endif
#endif
    /* Mode 1: m = (DIVIDER_X + 1),  8 <= DIVIDER_X <= 16 */
    else {
      for (uint32_t m = 9; m <= (NUC123_DIVIDER_X_MAX + 1); ++m) {
        if ((baud_denom % m == 0) && ((baud_denom / m) - 2 <= NUC123_BRD_MAX)) {
          sdp->uart->BAUD =
              NUC123_BRD_MODE1 | ((m - 1) << UART_BAUD_DIVIDER_X_Pos) | ((baud_denom / m) - 2);
          baud_found = true;
          break;
        }
      }
    }
  }

  /* If we couldn't generate the desired frequency from any of the available modes, halt */
  osalDbgCheck(baud_found);

  sdp->uart->FUN_SEL = config->mode & UART_FUN_SEL_FUN_SEL_Msk;
  sdp->uart->LCR     = config->data | config->parity | config->stop;
  sdp->uart->IER     = UART_IER_RLS_IEN_Msk | UART_IER_RDA_IEN_Msk | UART_IER_BUF_ERR_IEN_Msk;
  sdp->uart->FCR &= ~UART_FCR_RFITL_Msk;
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
void sd_lld_stop(SerialDriver* sdp)
{

  if (sdp->state == SD_READY) {
#if NUC123_SERIAL_USE_UART0
    if (&SD0 == sdp) {
      CLK->APBCLK &= ~CLK_APBCLK_UART0_EN_Msk;
      nvicDisableVector(NUC123_UART0_NUMBER);
      return;
    }
#endif

#if NUC123_SERIAL_USE_UART1
    if (&SD1 == sdp) {
      CLK->APBCLK &= ~CLK_APBCLK_UART1_EN_Msk;
      nvicDisableVector(NUC123_UART1_NUMBER);
      return;
    }
#endif

    sdp->uart->FUN_SEL = 0;
    sdp->uart->LCR     = 0;
    sdp->uart->BAUD    = 0;
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
