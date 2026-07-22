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
 * @file    GPIO/hal_pal_lld.c
 * @brief   GD32 PAL low level driver code.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if GD32_HAS_GPIOE
#define APB2_EN_MASK  (RCU_APB2EN_PAEN | RCU_APB2EN_PBEN |            \
                       RCU_APB2EN_PCEN | RCU_APB2EN_PDEN |            \
                       RCU_APB2EN_PEEN | RCU_APB2EN_AFEN)
#else
#define APB2_EN_MASK  (RCU_APB2EN_PAEN | RCU_APB2EN_PBEN |            \
                       RCU_APB2EN_PCEN | RCU_APB2EN_PDEN |            \
                       RCU_APB2EN_AFEN)
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Event records for the 16 GPIO EXTI channels.
 */
palevent_t _pal_events[16];
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   GD32 I/O ports configuration.
 * @details Ports A-D(E) clocks enabled, AFIO clock enabled.
 *
 * @param[in] config    the GD32 ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config) {

#if PAL_USE_CALLBACKS || PAL_USE_WAIT || defined(__DOXYGEN__)
  unsigned i;

  for (i = 0; i < 16; i++) {
    _pal_init_event(i);
  }
#endif

  /*
   * Enables the GPIO related clocks.
   */
  rcuEnableAPB2(APB2_EN_MASK, true);

  /*
   * Initial GPIO setup.
   */
  GPIOA->OCTL = config->PAData.octl;
  GPIOA->CTL1 = config->PAData.ctl1;
  GPIOA->CTL0 = config->PAData.ctl0;
  GPIOB->OCTL = config->PBData.octl;
  GPIOB->CTL1 = config->PBData.ctl1;
  GPIOB->CTL0 = config->PBData.ctl0;
  GPIOC->OCTL = config->PCData.octl;
  GPIOC->CTL1 = config->PCData.ctl1;
  GPIOC->CTL0 = config->PCData.ctl0;
  GPIOD->OCTL = config->PDData.octl;
  GPIOD->CTL1 = config->PDData.ctl1;
  GPIOD->CTL0 = config->PDData.ctl0;
#if GD32_HAS_GPIOE || defined(__DOXYGEN__)
  GPIOE->OCTL = config->PEData.octl;
  GPIOE->CTL1 = config->PEData.ctl1;
  GPIOE->CTL0 = config->PEData.ctl0;
#endif
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    @p PAL_MODE_UNCONNECTED is implemented as push pull output at 2MHz.
 * @note    Writing on pads programmed as pull-up or pull-down has the side
 *          effect to modify the resistor setting because the output latched
 *          data is used for the resistor selection.
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
  static const uint8_t cfgtab[] = {
    4,          /* PAL_MODE_RESET, implemented as input.*/
    2,          /* PAL_MODE_UNCONNECTED, implemented as push pull output 2MHz.*/
    4,          /* PAL_MODE_INPUT */
    8,          /* PAL_MODE_INPUT_PULLUP */
    8,          /* PAL_MODE_INPUT_PULLDOWN */
    0,          /* PAL_MODE_INPUT_ANALOG */
    3,          /* PAL_MODE_OUTPUT_PUSHPULL, 50MHz.*/
    7,          /* PAL_MODE_OUTPUT_OPENDRAIN, 50MHz.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    0xB,        /* PAL_MODE_GD32_ALTERNATE_PUSHPULL, 50MHz.*/
    0xF,        /* PAL_MODE_GD32_ALTERNATE_OPENDRAIN, 50MHz.*/
  };
  uint32_t mh, ml, ctl1, ctl0, cfg;
  unsigned i;

  if (mode == PAL_MODE_INPUT_PULLUP)
    port->BOP = mask;
  else if (mode == PAL_MODE_INPUT_PULLDOWN)
    port->BC = mask;
  cfg = cfgtab[mode];
  mh = ml = ctl1 = ctl0 = 0;
  for (i = 0; i < 8; i++) {
    ml <<= 4;
    mh <<= 4;
    ctl0 <<= 4;
    ctl1 <<= 4;
    if ((mask & 0x0080) == 0)
      ml |= 0xf;
    else
      ctl0 |= cfg;
    if ((mask & 0x8000) == 0)
      mh |= 0xf;
    else
      ctl1 |= cfg;
    mask <<= 1;
  }
  port->CTL1 = (port->CTL1 & mh) | ctl1;
  port->CTL0 = (port->CTL0 & ml) | ctl0;
}

#if PAL_USE_CALLBACKS || PAL_USE_WAIT || defined(__DOXYGEN__)
/**
 * @brief   Pad event enable.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad event mode
 *
 * @notapi
 */
void _pal_lld_enablepadevent(ioportid_t port,
                             iopadid_t pad,
                             ioeventmode_t mode) {

  uint32_t padmask, cridx, croff, crmask, portidx;

  /* Mask of the pad.*/
  padmask = 1U << (uint32_t)pad;

  /* Multiple channel setting of the same channel not allowed, first disable
     it. This is done because on GD32 the same channel cannot be mapped on
     multiple ports.*/
  osalDbgAssert(((EXTI->RTEN & padmask) == 0U) &&
                ((EXTI->FTEN & padmask) == 0U), "channel already in use");

  /* Index and mask of the SYSCFG CR register to be used.*/
  cridx  = (uint32_t)pad >> 2U;
  croff = ((uint32_t)pad & 3U) * 4U;
  crmask = ~(0xFU << croff);

  /* Port index is obtained assuming that GPIO ports are placed at regular
     0x400 intervals in memory space. So far this is true for all devices.*/
  portidx = (((uint32_t)port - (uint32_t)GPIOA) >> 10U) & 0xFU;

  /* Port selection in SYSCFG.*/
  AFIO->EXTISS[cridx] = (AFIO->EXTISS[cridx] & crmask) | (portidx << croff);

  /* Programming edge registers.*/
  if (mode & PAL_EVENT_MODE_RISING_EDGE)
    EXTI->RTEN |= padmask;
  else
    EXTI->RTEN &= ~padmask;
  if (mode & PAL_EVENT_MODE_FALLING_EDGE)
    EXTI->FTEN |= padmask;
  else
    EXTI->FTEN &= ~padmask;

  /* Programming interrupt and event registers.*/
  EXTI->INTEN |= padmask;
  EXTI->EVEN &= ~padmask;
}

/**
 * @brief   Pad event disable.
 * @details This function disables previously programmed event callbacks.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
void _pal_lld_disablepadevent(ioportid_t port, iopadid_t pad) {
  uint32_t padmask, rtsr1, ftsr1;

  rtsr1 = EXTI->RTEN;
  ftsr1 = EXTI->FTEN;

  /* Mask of the pad.*/
  padmask = 1U << (uint32_t)pad;

  /* If either RTRS1 or FTSR1 is enabled then the channel is in use.*/
  if (((rtsr1 | ftsr1) & padmask) != 0U) {
    uint32_t cridx, croff, crport, portidx;

    /* Index and mask of the SYSCFG CR register to be used.*/
    cridx  = (uint32_t)pad >> 2U;
    croff = ((uint32_t)pad & 3U) * 4U;

    /* Port index is obtained assuming that GPIO ports are placed at regular
       0x400 intervals in memory space. So far this is true for all devices.*/
    portidx = (((uint32_t)port - (uint32_t)GPIOA) >> 10U) & 0xFU;

    crport = (AFIO->EXTISS[cridx] >> croff) & 0xFU;

    osalDbgAssert(crport == portidx, "channel mapped on different port");

    /* Disabling channel.*/
    EXTI->INTEN  &= ~padmask;
    EXTI->EVEN  &= ~padmask;
    EXTI->RTEN  = rtsr1 & ~padmask;
    EXTI->FTEN  = ftsr1 & ~padmask;
    EXTI->PD    = padmask;

#if PAL_USE_CALLBACKS || PAL_USE_WAIT
  /* Callback cleared and/or thread reset.*/
  _pal_clear_event(pad);
#endif
  }
}
#endif /* PAL_USE_CALLBACKS || PAL_USE_WAIT */

#endif /* HAL_USE_PAL */

/** @} */
