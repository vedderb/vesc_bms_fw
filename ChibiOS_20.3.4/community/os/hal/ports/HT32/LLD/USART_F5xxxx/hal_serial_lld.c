/*
    Copyright (C) 2020 Yaotian Feng

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
 * @brief   HT32 serial subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART0 serial driver identifier.*/
#    if (HT32_SERIAL_USE_USART0 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD0;
#    endif
#    if (HT32_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#    endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {SERIAL_DEFAULT_BITRATE};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void load(SerialDriver *sdp) {
    USART_TypeDef *u = sdp->usart;

    u->IER |= (USART_IER_TXDEIE | USART_IER_TXCIE);
}

#    if HT32_SERIAL_USE_USART0 == TRUE
static void notify0(io_queue_t *qp) {
    (void)qp;
    load(&SD0);
}
#    endif

#    if HT32_SERIAL_USE_USART1 == TRUE
static void notify1(io_queue_t *qp) {
    (void)qp;
    load(&SD1);
}
#    endif

static void serialInterrupt(SerialDriver *pSd) {
    USART_TypeDef *u = pSd->usart;

    uint32_t sifr = u->SIFR;
    // Rx has data, timeout or crossed threashold
    // TX Below threash or complete
    if ((sifr & USART_SIFR_TXDE) || (sifr & USART_SIFR_TXC)) {
        msg_t b;
        osalSysLockFromISR();
        {
            b = oqGetI(&(pSd->oqueue));
            if (b < MSG_OK) {
                u->IER &= ~(USART_IER_TXDEIE | USART_IER_TXCIE);
                chnAddFlagsI(pSd, CHN_OUTPUT_EMPTY);
            } else {
                u->DR = b;
            }
        }
        osalSysUnlockFromISR();
    }

    if (sifr & (USART_SIFR_RXDNE | USART_SIFR_RXDR | USART_SIFR_RXTOF)) {
        osalSysLockFromISR();
        {
            if (iqIsEmptyI(&pSd->iqueue)) chnAddFlagsI(pSd, CHN_INPUT_AVAILABLE);

            while ((u->SIFR) & USART_SIFR_RXDNE) {
                if (iqPutI(&(pSd->iqueue), u->DR) < MSG_OK) chnAddFlagsI(pSd, SD_OVERRUN_ERROR);
            }

            // Clear Timeout if it occours
            if (sifr & USART_SIFR_RXTOF) u->SIFR = USART_SIFR_RXTOF;
        }
        osalSysUnlockFromISR();
    }

    // Clear all remaining flags.
    u->SIFR &= 0x3FF;
}

static void usartInit(SerialDriver *sdp, const SerialConfig *config) {
    USART_TypeDef *u = sdp->usart;

    u->DLR = (uint32_t)(HT32_CK_USART_FREQUENCY / config->speed);                      /* Data Rate */
    u->CR  = (UART_CR_MODE_NORMAL | UART_CR_URTXEN | UART_CR_URRXEN | UART_CR_WLS_8B); /* 8-bit, one stop, no parity */
    u->FCR = USART_FCR_TXR | USART_FCR_RXR;                                            // Reset FIFOs.
    u->IER = USART_IER_RXDRIE | USART_IER_TXDEIE | USART_IER_TXCIE;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#    if HT32_SERIAL_USE_USART0 == TRUE
CH_IRQ_HANDLER(HT32_USART0_IRQ_VECTOR) {
    CH_IRQ_PROLOGUE();
    serialInterrupt(&SD0);
    CH_IRQ_EPILOGUE();
}
#    endif

#    if HT32_SERIAL_USE_USART1 == TRUE
CH_IRQ_HANDLER(HT32_USART1_IRQ_VECTOR) {
    CH_IRQ_PROLOGUE();
    serialInterrupt(&SD1);
    CH_IRQ_EPILOGUE();
}
#    endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {
#    if HT32_SERIAL_USE_USART0 == TRUE
    sdObjectInit(&SD0, NULL, notify0);
    SD0.usart = USART0;
#    endif
#    if HT32_SERIAL_USE_USART1 == TRUE
    sdObjectInit(&SD1, NULL, notify1);
    SD1.usart = USART1;
#    endif
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
    if (config == NULL) {
        config = &default_config;
    }

    if (sdp->state == SD_STOP) {
#    if HT32_SERIAL_USE_USART0 == TRUE
        if (&SD0 == sdp) {
            CKCU->APBCCR0 |= (1U << 8);
            nvicEnableVector(USART0_IRQn, HT32_USART0_IRQ_PRIORITY);
        }
#    endif
#    if HT32_SERIAL_USE_USART1 == TRUE
        if (&SD1 == sdp) {
            CKCU->APBCCR0 |= (1U << 9);
            nvicEnableVector(USART1_IRQn, HT32_USART1_IRQ_PRIORITY);
        }
#    endif
    }
    usartInit(sdp, config);
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
        USART_TypeDef *u = sdp->usart;

#    if HT32_SERIAL_USE_USART0 == TRUE
        if (&SD0 == sdp) {
            nvicDisableVector(USART0_IRQn);
            u -> IER = 0; // Disable all interrupts
        }
#    endif  // HT32_SERIAL_USE_USART0 == TRUE
#    if HT32_SERIAL_USE_USART1 == TRUE
        if (&SD1 == sdp) {
            nvicDisableVector(USART1_IRQn);
            u -> IER = 0; // Disable all interrupts
        }
#    endif  // HT32_SERIAL_USE_USART1 == TRUE
    }
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
