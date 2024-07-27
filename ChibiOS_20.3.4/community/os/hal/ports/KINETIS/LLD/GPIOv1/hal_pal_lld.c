/*
    ChibiOS - Copyright (C) 2014-2015 Fabio Utzig

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
 * @file    GPIOv1/hal_pal_lld.c
 * @brief   PAL subsystem low level driver.
 *
 * @addtogroup PAL
 * @{
 */

#include "osal.h"
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
#define PCR_IRQC_DISABLED           0x0
#define PCR_IRQC_DMA_RISING_EDGE    0x1
#define PCR_IRQC_DMA_FALLING_EDGE   0x2
#define PCR_IRQC_DMA_EITHER_EDGE    0x3

#define PCR_IRQC_LOGIC_ZERO         0x8
#define PCR_IRQC_RISING_EDGE        0x9
#define PCR_IRQC_FALLING_EDGE       0xA
#define PCR_IRQC_EITHER_EDGE        0xB
#define PCR_IRQC_LOGIC_ONE          0xC
#endif /* (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
palevent_t _pal_events[TOTAL_PORTS * PADS_PER_PORT];
#endif /* (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
static inline PORT_TypeDef* _pal_lld_ext_port(ioportid_t port) {
    switch ((uint32_t)port) {
        case GPIOA_BASE:
            return PORTA;
        case GPIOB_BASE:
            return PORTB;
        case GPIOC_BASE:
            return PORTC;
        case GPIOD_BASE:
            return PORTD;
        case GPIOE_BASE:
            return PORTE;
        default:
            return NULL;
    }
}
#endif /* (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) */

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
/*
 * Generic interrupt handler.
 */
static inline void irq_handler(PORT_TypeDef* const port,
                               palevent_t* events) {
    iopadid_t pad;
    uint32_t pad_mask;

    chSysLockFromISR();
    uint32_t isfr = port->ISFR; /* Get pending interrupts on this port. */
    port->ISFR = 0xFFFFFFFFU; /* Clear all pending interrupts on this port. */
    chSysUnlockFromISR();

    /* invoke callbacks for pending interrupts */
    for (pad = 0, pad_mask = 1U;
         pad < PAL_IOPORTS_WIDTH;
         ++pad, ++events, pad_mask <<= 1) {
        if (events->cb && (isfr & pad_mask)) {
            events->cb(events->arg);
        }
    }
}

/**
 * @brief   PORTA interrupt handler.
 *
 * @isr
 */
#if defined(KINETIS_PORTA_IRQ_VECTOR)
OSAL_IRQ_HANDLER(KINETIS_PORTA_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  irq_handler(PORTA, _pal_events);
  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(KINETIS_PORTA_IRQ_VECTOR) */

#if KINETIS_EXT_HAS_COMMON_BCDE_IRQ

#if defined(KINETIS_PORTD_IRQ_VECTOR)
OSAL_IRQ_HANDLER(KINETIS_PORTD_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  irq_handler(PORTB, _pal_events + PAL_IOPORTS_WIDTH);
  irq_handler(PORTC, _pal_events + PAL_IOPORTS_WIDTH * 2);
  irq_handler(PORTD, _pal_events + PAL_IOPORTS_WIDTH * 3);
  irq_handler(PORTE, _pal_events + PAL_IOPORTS_WIDTH * 4);
  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(KINETIS_PORTD_IRQ_VECTOR) */

#elif KINETIS_EXT_HAS_COMMON_CD_IRQ /* KINETIS_EXT_HAS_COMMON_BCDE_IRQ */

#if defined(KINETIS_PORTD_IRQ_VECTOR)
OSAL_IRQ_HANDLER(KINETIS_PORTD_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  irq_handler(PORTC, _pal_events + PAL_IOPORTS_WIDTH * 2);
  irq_handler(PORTD, _pal_events + PAL_IOPORTS_WIDTH * 3);
  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(KINETIS_PORTD_IRQ_VECTOR) */


#else /* KINETIS_EXT_HAS_COMMON_CD_IRQ */

/**
 * @brief   PORTB interrupt handler.
 *
 * @isr
 */
#if defined(KINETIS_PORTB_IRQ_VECTOR)
OSAL_IRQ_HANDLER(KINETIS_PORTB_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  irq_handler(PORTB, _pal_events + PAL_IOPORTS_WIDTH);
  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(KINETIS_EXT_PORTB_IRQ_VECTOR */

/**
 * @brief   PORTC interrupt handler.
 *
 * @isr
 */
#if defined(KINETIS_PORTC_IRQ_VECTOR)
OSAL_IRQ_HANDLER(KINETIS_PORTC_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  irq_handler(PORTC, _pal_events + PAL_IOPORTS_WIDTH * 2);
  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(KINETIS_PORTC_IRQ_VECTOR) */

/**
 * @brief   PORTD interrupt handler.
 *
 * @isr
 */
#if defined(KINETIS_PORTD_IRQ_VECTOR)
OSAL_IRQ_HANDLER(KINETIS_PORTD_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  irq_handler(PORTD, _pal_events + PAL_IOPORTS_WIDTH * 3);
  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(KINETIS_PORTD_IRQ_VECTOR) */

/**
 * @brief   PORTE interrupt handler.
 *
 * @isr
 */
#if defined(KINETIS_PORTE_IRQ_VECTOR)
OSAL_IRQ_HANDLER(KINETIS_PORTE_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  irq_handler(PORTE, _pal_events + PAL_IOPORTS_WIDTH * 4);
  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(KINETIS_PORTE_IRQ_VECTOR) */

#endif /* !KINETIS_EXT_HAS_COMMON_CD_IRQ */

#endif /* (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Kinetis I/O ports configuration.
 * @details Ports A-E clocks enabled.
 *
 * @param[in] config    the Kinetis ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config) {
  int i, j;

  /* Enable clocking on all Ports */
  SIM->SCGC5 |= SIM_SCGC5_PORTA |
                SIM_SCGC5_PORTB |
                SIM_SCGC5_PORTC |
                SIM_SCGC5_PORTD |
                SIM_SCGC5_PORTE;

  /* Initial PORT and GPIO setup */
  for (i = 0; i < TOTAL_PORTS; i++) {
    for (j = 0; j < PADS_PER_PORT; j++) {
      pal_lld_setpadmode(config->ports[i].port,
                         j,
                         config->ports[i].pads[j]);
#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
      _pal_init_event(PADS_PER_PORT * i + j);
#endif /* (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) */
    }
  }

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
  nvicEnableVector(PINA_IRQn, KINETIS_EXT_PORTA_IRQ_PRIORITY);
#if KINETIS_EXT_HAS_COMMON_BCDE_IRQ
  nvicEnableVector(PINBCDE_IRQn, KINETIS_EXT_PORTD_IRQ_PRIORITY);
#elif KINETIS_EXT_HAS_COMMON_CD_IRQ /* KINETIS_EXT_HAS_COMMON_BCDE_IRQ */
  nvicEnableVector(PINCD_IRQn, KINETIS_EXT_PORTD_IRQ_PRIORITY);
#else /* KINETIS_EXT_HAS_COMMON_CD_IRQ */
  nvicEnableVector(PINB_IRQn, KINETIS_EXT_PORTB_IRQ_PRIORITY);
  nvicEnableVector(PINC_IRQn, KINETIS_EXT_PORTC_IRQ_PRIORITY);
  nvicEnableVector(PIND_IRQn, KINETIS_EXT_PORTD_IRQ_PRIORITY);
  nvicEnableVector(PINE_IRQn, KINETIS_EXT_PORTE_IRQ_PRIORITY);
#endif /* KINETIS_EXT_HAS_COMMON_BCDE_IRQ */
#endif /* (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) */
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {
  int i;
  (void)mask;

  for (i = 0; i < PADS_PER_PORT; i++) {
    pal_lld_setpadmode(port, i, mode);
  }
}

/**
 * @brief   Pad mode setup.
 * @details This function programs a pad with the specified mode.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad mode
 *
 * @notapi
 */
void _pal_lld_setpadmode(ioportid_t port,
                         uint8_t pad,
                         iomode_t mode) {
  PORT_TypeDef *portcfg = NULL;

  osalDbgAssert(pad < PADS_PER_PORT, "pal_lld_setpadmode() #1, invalid pad");

  if (mode == PAL_MODE_OUTPUT_PUSHPULL)
    port->PDDR |= ((uint32_t) 1 << pad);
  else
    port->PDDR &= ~((uint32_t) 1 << pad);

  if (port == IOPORT1)
    portcfg = PORTA;
  else if (port == IOPORT2)
    portcfg = PORTB;
  else if (port == IOPORT3)
    portcfg = PORTC;
  else if (port == IOPORT4)
    portcfg = PORTD;
  else if (port == IOPORT5)
    portcfg = PORTE;

  osalDbgAssert(portcfg != NULL, "pal_lld_setpadmode() #2, invalid port");

  switch (mode) {
  case PAL_MODE_RESET:
  case PAL_MODE_INPUT:
  case PAL_MODE_OUTPUT_PUSHPULL:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(1);
    break;
#if KINETIS_GPIO_HAS_OPENDRAIN
  case PAL_MODE_OUTPUT_OPENDRAIN:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(1) |
                        PORTx_PCRn_ODE;
    break;
#else
#undef PAL_MODE_OUTPUT_OPENDRAIN
#endif
  case PAL_MODE_INPUT_PULLUP:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(1) |
                        PORTx_PCRn_PE |
                        PORTx_PCRn_PS;
      break;
  case PAL_MODE_INPUT_PULLDOWN:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(1) |
                        PORTx_PCRn_PE;
      break;
  case PAL_MODE_UNCONNECTED:
  case PAL_MODE_INPUT_ANALOG:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(0);
    break;
  case PAL_MODE_ALTERNATIVE_1:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(1);
    break;
  case PAL_MODE_ALTERNATIVE_2:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(2);
    break;
  case PAL_MODE_ALTERNATIVE_3:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(3);
    break;
  case PAL_MODE_ALTERNATIVE_4:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(4);
    break;
  case PAL_MODE_ALTERNATIVE_5:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(5);
    break;
  case PAL_MODE_ALTERNATIVE_6:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(6);
    break;
  case PAL_MODE_ALTERNATIVE_7:
    portcfg->PCR[pad] = PIN_MUX_ALTERNATIVE(7);
    break;
  }
}

/**
 * @brief Reads a logical state from an I/O pad.
 * @note The @ref PAL provides a default software implementation of this
 * functionality, implement this function if can optimize it by using
 * special hardware functionalities or special coding.
 *
 * @param[in] port port identifier
 * @param[in] pad pad number within the port
 * @return The logical state.
 * @retval PAL_LOW low logical state.
 * @retval PAL_HIGH high logical state.
 *
 * @notapi
 */
uint8_t _pal_lld_readpad(ioportid_t port,
                         uint8_t pad) {
  return (port->PDIR & ((uint32_t) 1 << pad)) ? PAL_HIGH : PAL_LOW;
}

/**
 * @brief Writes a logical state on an output pad.
 * @note This function is not meant to be invoked directly by the
 * application code.
 * @note The @ref PAL provides a default software implementation of this
 * functionality, implement this function if can optimize it by using
 * special hardware functionalities or special coding.
 *
 * @param[in] port port identifier
 * @param[in] pad pad number within the port
 * @param[in] bit logical value, the value must be @p PAL_LOW or
 * @p PAL_HIGH
 *
 * @notapi
 */
void _pal_lld_writepad(ioportid_t port,
                       uint8_t pad,
                       uint8_t bit) {
  if (bit == PAL_HIGH)
    port->PDOR |= ((uint32_t) 1 << pad);
  else
    port->PDOR &= ~((uint32_t) 1 << pad);
}

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
/**
 * @brief   Returns a PAL event structure associated to a pad.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
palevent_t* _pal_lld_get_pad_event(ioportid_t port,
                                   iopadid_t pad) {
    switch ((uint32_t)port) {
        case GPIOA_BASE:
            return _pal_events + pad;
        case GPIOB_BASE:
            return _pal_events + PADS_PER_PORT + pad;
        case GPIOC_BASE:
            return _pal_events + PADS_PER_PORT * 2 + pad;
        case GPIOD_BASE:
            return _pal_events + PADS_PER_PORT * 3 + pad;
        case GPIOE_BASE:
            return _pal_events + PADS_PER_PORT * 4 + pad;
        default:
            return NULL;
    }
}

/**
 * @brief   Enables a pad event.
 *
 * @param[in] port   port containing pad whose event is to be enabled
 * @param[in] pad    pad whose event is to be enabled
 * @param[in] mode   mode of the event
 *
 * @notapi
 */
void _pal_lld_enablepadevent(ioportid_t ioport,
                             iopadid_t pad,
                             iomode_t mode) {
    PORT_TypeDef *port = _pal_lld_ext_port(ioport);
    iomode_t pcr_mode;
    switch(mode) {
        case PAL_EVENT_MODE_RISING_EDGE:
            pcr_mode = PCR_IRQC_RISING_EDGE;
            break;
        case PAL_EVENT_MODE_FALLING_EDGE:
            pcr_mode = PCR_IRQC_FALLING_EDGE;
            break;
        case PAL_EVENT_MODE_BOTH_EDGES:
            pcr_mode = PCR_IRQC_EITHER_EDGE;
            break;
        case PAL_EVENT_MODE_DISABLED:
            pcr_mode = PCR_IRQC_DISABLED;
            break;
        default:
            return;
    }
    port->PCR[pad] |= PORTx_PCRn_IRQC(pcr_mode);
}

/**
 * @brief   Disables a pad event
 *
 * @param[in] ioport    port containing pad whose event is to be disabled
 * @param[in] pad       pad whose event is to be disabled
 *
 * @notapi
 */
void _pal_lld_disablepadevent(ioportid_t ioport, iopadid_t pad) {
    _pal_lld_enablepadevent(ioport, pad, PAL_EVENT_MODE_DISABLED);
}

/**
 * @brief   Returns whether a pad event is enabled
 *
 * @param[in] ioport   port containing the pad
 * @param[in] pad      pad whose event is to be queried
 *
 * @notapi
 */
bool _pal_lld_ispadeventenabled(ioportid_t ioport, iopadid_t pad) {
    PORT_TypeDef *port = _pal_lld_ext_port(ioport);
    uint32_t pcr = port->PCR[pad];
    return ((pcr & PORTx_PCRn_IRQC_MASK) >> PORTx_PCRn_IRQC_SHIFT) !=
        PCR_IRQC_DISABLED;
}
#endif /* (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) */
#endif /* HAL_USE_PAL */

/** @} */
