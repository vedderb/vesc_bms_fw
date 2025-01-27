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
 * @file    hal_uart_lld.c
 * @brief   PLATFORM UART subsystem low level driver source.
 *
 * @addtogroup UART
 * @{
 */

#include "hal.h"

#if (HAL_USE_UART == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   UART1 driver identifier.
 */
#if (HT32_UART_USE_UART0 == TRUE) || defined(__DOXYGEN__)
UARTDriver UARTD0;
#endif
#if (HT32_UART_USE_UART1 == TRUE) || defined(__DOXYGEN__)
UARTDriver UARTD1;
#endif
#if (HT32_UART_USE_USART0 == TRUE) || defined(__DOXYGEN__)
UARTDriver USARTD0;
#endif
#if (HT32_UART_USE_USART1 == TRUE) || defined(__DOXYGEN__)
UARTDriver USARTD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void uart_lld_handler(UARTDriver *uartp) {

}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (HT32_UART_USE_UART0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(HT32_UART0_IRQ_VECTOR) {
    OSAL_IRQ_PROLOGUE();
    uart_lld_handler(&UARTD0);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (HT32_UART_USE_UART1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(HT32_UART1_IRQ_VECTOR) {
    OSAL_IRQ_PROLOGUE();
    uart_lld_handler(&UARTD1);
    OSAL_IRQ_EPILOGUE();
}
#endif

#if (HT32_UART_USE_USART0 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(HT32_USART0_IRQ_VECTOR) {
    OSAL_IRQ_PROLOGUE();
    uart_lld_handler(&USARTD0);
    OSAL_IRQ_EPILOGUE();
}
#endif
#if (HT32_UART_USE_USART1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(HT32_USART1_IRQ_VECTOR) {
    OSAL_IRQ_PROLOGUE();
    uart_lld_handler(&USARTD1);
    OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level UART driver initialization.
 *
 * @notapi
 */
void uart_lld_init(void) {
    /* Driver initialization. */
#if HT32_UART_USE_UART0 == TRUE
    uartObjectInit(&UARTD0);
    UARTD0.UART = UART0;
#endif
#if HT32_UART_USE_UART1 == TRUE
    uartObjectInit(&UARTD1);
    UARTD1.UART = UART1;
#endif
#if HT32_UART_USE_USART0 == TRUE
    uartObjectInit(&USARTD0);
    USARTD0.UART = USART0;
#endif
#if HT32_UART_USE_USART1 == TRUE
    uartObjectInit(&USARTD1);
    USARTD1.UART = USART1;
#endif

    /* Peripheral initialization. */
    CKCU->GCFGR = (CKCU->GCFGR & ~CKCU_GCFGR_URPRE_MASK) | ((HT32_USART_PRESCALER - 1) << 22);
}

/**
 * @brief   Configures and activates the UART peripheral.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_start(UARTDriver *uartp) {
    if (uartp->state == UART_STOP) {
        /* Enables the peripheral.*/
#if HT32_UART_USE_UART0 == TRUE
        if (&UARTD1 == uartp) {
            CKCU->APBCCR0 |= CKCU_APBCCR0_UR0EN;
            nvicEnableVector(UART0_IRQn, HT32_UART0_IRQ_PRIORITY);
        }
#endif
#if HT32_UART_USE_UART1 == TRUE
        if (&UARTD1 == uartp) {
            CKCU->APBCCR0 |= CKCU_APBCCR0_UR1EN;
            nvicEnableVector(UART1_IRQn, HT32_UART1_IRQ_PRIORITY);
        }
#endif
#if HT32_UART_USE_USART0 == TRUE
        if (&USARTD0 == uartp) {
            CKCU->APBCCR0 |= CKCU_APBCCR0_USR0EN;
            nvicEnableVector(USART0_IRQn, HT32_USART0_IRQ_PRIORITY);
        }
#endif
#if HT32_UART_USE_USART1 == TRUE
        if (&USARTD1 == uartp) {
            CKCU->APBCCR0 |= CKCU_APBCCR0_USR1EN;
            nvicEnableVector(USART1_IRQn, HT32_USART1_IRQ_PRIORITY);
        }
#endif
    }

    /* Configures the peripheral.*/
    uartp->UART->FCR = uartp->config->fcr;
    uartp->UART->LCR = uartp->config->lcr;
    uartp->UART->MDR = uartp->config->mdr;
    // baud = ck_ahb / dlr value
    uartp->UART->DLR = HT32_CK_AHB_FREQUENCY / uartp->config->baud;
}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @notapi
 */
void uart_lld_stop(UARTDriver *uartp) {
    if (uartp->state == UART_READY) {
        /* Resets the peripheral.*/
        /* Disables the peripheral.*/
#if HT32_UART_USE_UART0 == TRUE
        if (&UARTD0 == uartp) {
            RSTCU->APBPRSTR0 = RSTCU_APBPRSTR0_UR0RST;
            CKCU->APBCCR0 &= ~CKCU_APBCCR0_UR0EN;
            nvicDisableVector(UART0_IRQn);
        }
#endif
#if HT32_UART_USE_UART1 == TRUE
        if (&UARTD1 == uartp) {
            RSTCU->APBPRSTR0 = RSTCU_APBPRSTR0_UR1RST;
            CKCU->APBCCR0 &= ~CKCU_APBCCR0_UR1EN;
            nvicDisableVector(UART1_IRQn);
        }
#endif
#if HT32_UART_USE_USART0 == TRUE
        if (&USARTD0 == uartp) {
            RSTCU->APBPRSTR0 = RSTCU_APBPRSTR0_USR0RST;
            CKCU->APBCCR0 &= ~CKCU_APBCCR0_USR0EN;
            nvicDisableVector(USART0_IRQn);
        }
#endif
#if HT32_UART_USE_USART1 == TRUE
        if (&USARTD1 == uartp) {
            RSTCU->APBPRSTR0 = RSTCU_APBPRSTR0_USR1RST;
            CKCU->APBCCR0 &= ~CKCU_APBCCR0_USR1EN;
            nvicDisableVector(USART1_IRQn);
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
    if ((uartp->config->lcr & 3) == 2) {
        uint16_t *txdata = (uint16_t *)txbuf;
        while(n > 0){
            uartp->UART->TBR = *txdata;
            while(!(uartp->UART->LSR & USART_LSR_TXEMPT));
            txdata++;
            n--;
        }
    } else {
        uint8_t *txdata = (uint8_t *)txbuf;
        while(n > 0){
            uartp->UART->TBR = *txdata;
            while(!(uartp->UART->LSR & USART_LSR_TXEMPT));
            txdata++;
            n--;
        }
    }
}

/**
 * @brief   Stops any ongoing transmission.
 * @note    Stopping a transmission also suppresses the transmission callbacks.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not transmitted by the
 *                      stopped transmit operation.
 *
 * @notapi
 */
size_t uart_lld_stop_send(UARTDriver *uartp) {

    (void)uartp;

    return 0;
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

    (void)uartp;
    (void)n;
    (void)rxbuf;

}

/**
 * @brief   Stops any ongoing receive operation.
 * @note    Stopping a receive operation also suppresses the receive callbacks.
 *
 * @param[in] uartp      pointer to the @p UARTDriver object
 *
 * @return              The number of data frames not received by the
 *                      stopped receive operation.
 *
 * @notapi
 */
size_t uart_lld_stop_receive(UARTDriver *uartp) {

    (void)uartp;

    return 0;
}

#endif /* HAL_USE_UART == TRUE */

/** @} */
