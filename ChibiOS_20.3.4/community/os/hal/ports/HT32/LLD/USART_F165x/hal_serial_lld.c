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
#if (HT32_SERIAL_USE_USART0 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD0;
#endif
#if (HT32_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void load(SerialDriver *sdp)
{
	USART_TypeDef *u = sdp->usart;

#define TFTLIE (1U << 1)
	u->IER |= TFTLIE;
}

#if HT32_SERIAL_USE_USART0 == TRUE
static void notify0(io_queue_t *qp)
{
	(void)qp;
	load(&SD0);
}
#endif

#if HT32_SERIAL_USE_USART1 == TRUE
static void notify1(io_queue_t *qp)
{
	(void)qp;
	load(&SD1);
}
#endif

static void serial_interrupt(SerialDriver *sdp)
{
	USART_TypeDef *u = sdp->usart;

	while ((u->IIR & 1) == 0) {
		switch ((u->IIR >> 1) & 7) {
		case 0:
			/* modem status */
			break;
		case 1:
			/* TX FIFO level */
			{
			do {
				msg_t b;
				osalSysLockFromISR();
				b = oqGetI(&sdp->oqueue);
				osalSysUnlockFromISR();
				if (b < MSG_OK) {
					u->IER &= ~TFTLIE;
					osalSysLockFromISR();
					chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
					osalSysUnlockFromISR();
					break;
				}
				u->TBR = b;
			} while (0);
			}
			break;
		case 2:
			/* RX FIFO level */
		case 6:
			/* RX FIFO timeout */
			osalSysLockFromISR();
			if (iqIsEmptyI(&sdp->iqueue))
				chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
			osalSysUnlockFromISR();
			while ((u->LSR & 1) != 0) {
				osalSysLockFromISR();
				if (iqPutI(&sdp->iqueue, u->RBR) < MSG_OK)
					chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
				osalSysUnlockFromISR();
			}
			break;
		case 3:
			/* RX line status */
			break;
		}
	}
}

static void usart_init(SerialDriver *sdp, const SerialConfig *config)
{
	USART_TypeDef *u = sdp->usart;

	u->DLR = (uint32_t)(HT32_CK_USART_FREQUENCY / config->speed); /* XXX */
	u->LCR = 0x01; /* 8-bit, one stop, no parity */
	u->FCR = 0x301;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if HT32_SERIAL_USE_USART0 == TRUE
CH_IRQ_HANDLER(HT32_USART0_IRQ_VECTOR) {
	CH_IRQ_PROLOGUE();
	serial_interrupt(&SD0);
	CH_IRQ_EPILOGUE();
}
#endif

#if HT32_SERIAL_USE_USART1 == TRUE
CH_IRQ_HANDLER(HT32_USART1_IRQ_VECTOR) {
	CH_IRQ_PROLOGUE();
	serial_interrupt(&SD1);
	CH_IRQ_EPILOGUE();
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

#if HT32_SERIAL_USE_USART0 == TRUE
  sdObjectInit(&SD0, NULL, notify0);
  SD0.usart = USART0;
#endif
#if HT32_SERIAL_USE_USART1 == TRUE
  sdObjectInit(&SD1, NULL, notify1);
  SD1.usart = USART1;
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

  if (config == NULL) {
    config = &default_config;
  }


  if (sdp->state == SD_STOP) {
#if HT32_SERIAL_USE_USART0 == TRUE
    if (&SD0 == sdp) {
	CKCU->APBCCR0 |= (1U << 8);
        nvicEnableVector(USART0_IRQn, HT32_USART0_IRQ_PRIORITY);
    }
#endif
#if HT32_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {
	CKCU->APBCCR0 |= (1U << 9);
        nvicEnableVector(USART1_IRQn, HT32_USART1_IRQ_PRIORITY);
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
#if PLATFORM_SERIAL_USE_USART0 == TRUE
    if (&SD0 == sdp) {

    }
#endif
#if PLATFORM_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {

    }
#endif
  }
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
