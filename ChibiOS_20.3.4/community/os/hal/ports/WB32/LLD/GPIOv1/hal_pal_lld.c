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
 * @file    GPIOv1/hal_pal_lld.c
 * @brief   WB32 PAL low level driver code.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

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
 * @brief   PAL driver initialization.
 *
 * @notapi
 */
void _pal_lld_init(void) {

#if PAL_USE_CALLBACKS || PAL_USE_WAIT || defined(__DOXYGEN__)
  unsigned i;

  for (i = 0; i < 16; i++) {
    _pal_init_event(i);
  }
#endif
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    @p PAL_MODE_UNCONNECTED is implemented as push pull at minimum
 *          speed.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port, ioportmask_t mask, iomode_t mode) {
  uint32_t tmp = mode;

  uint32_t moder = (mode & PAL_WB32_MODE_MASK) >> 0;
  uint32_t otyper = (mode & PAL_WB32_OTYPE_MASK) >> 2;
  uint32_t ospeedr = (mode & PAL_WB32_OSPEED_MASK) >> 3;
  uint32_t pupdr = (mode & PAL_WB32_PUPDR_MASK) >> 5;
  uint32_t altr = (mode & PAL_WB32_ALTERNATE_MASK) >> 7;

  port->CFGMSK = ~mask;
  port->MODER = (moder & 0x3) * 0x55555555U;
  port->OTYPER = (otyper & 0x1) * 0xFFFFFFFFU;
  port->OSPEEDR = (ospeedr & 0x3) * 0x55555555U;
  port->PUPDR = (pupdr & 0x3) * 0x55555555U;

  tmp = altr * 0x11111111U;
  port->AFRL = tmp;
  port->AFRH = tmp;
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

  /* Enable EXTI clock.*/
  rccEnableEXTI();

  /* Mask of the pad.*/
  padmask = 1U << (uint32_t)pad;

  /* Multiple channel setting of the same channel not allowed, first
     disable it. This is done because on WB32 the same channel cannot
     be mapped on multiple ports.*/
  osalDbgAssert(((EXTI->RTSR & padmask) == 0U) &&
                ((EXTI->FTSR & padmask) == 0U), "channel already in use");

  /* Index and mask of the SYSCFG CR register to be used.*/
  cridx = (uint32_t)pad >> 2U;
  croff = ((uint32_t)pad & 3U) * 4U;
  crmask = ~(0xFU << croff);

  /* Port index is obtained assuming that GPIO ports are placed at  regular
     0x400 intervals in memory space. So far this is true for all devices.*/
  portidx = (((uint32_t)port - (uint32_t)GPIOA) >> 10U) & 0xFU;

  /* Port selection in SYSCFG.*/
  AFIO->EXTICR[cridx] = (AFIO->EXTICR[cridx] & crmask) | (portidx << croff);

  /* Programming edge registers.*/
  if (mode & PAL_EVENT_MODE_RISING_EDGE)
    EXTI->RTSR |= padmask;
  else
    EXTI->RTSR &= ~padmask;

  if (mode & PAL_EVENT_MODE_FALLING_EDGE)
    EXTI->FTSR |= padmask;
  else
    EXTI->FTSR &= ~padmask;

  /* Programming interrupt and event registers.*/
  EXTI->IMR |= padmask;
  EXTI->EMR &= ~padmask;
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

  rtsr1 = EXTI->RTSR;
  ftsr1 = EXTI->FTSR;

  /* Mask of the pad.*/
  padmask = 1U << (uint32_t)pad;

  /* If either RTRS1 or FTSR1 is enabled then the channel is in use.*/
  if (((rtsr1 | ftsr1) & padmask) != 0U) {
    uint32_t cridx, croff, crport, portidx;

    /* Index and mask of the SYSCFG CR register to be used.*/
    cridx = (uint32_t)pad >> 2U;
    croff = ((uint32_t)pad & 3U) * 4U;

    /* Port index is obtained assuming that GPIO ports are placed at regular
       0x400 intervals in memory space. So far this is true for all devices.*/
    portidx = (((uint32_t)port - (uint32_t)GPIOA) >> 10U) & 0xFU;
    crport = (AFIO->EXTICR[cridx] >> croff) & 0xFU;

    osalDbgAssert(crport == portidx, "channel mapped on different port");

    /* Disabling channel.*/
    EXTI->IMR &= ~padmask;
    EXTI->EMR &= ~padmask;
    EXTI->RTSR = rtsr1 & ~padmask;
    EXTI->FTSR = ftsr1 & ~padmask;
    EXTI->PR = padmask;

    /* Disable EXTI clock.*/
    rccDisableEXTI();

#if PAL_USE_CALLBACKS || PAL_USE_WAIT
    /* Callback cleared and/or thread reset.*/
    _pal_clear_event(pad);
#endif
  }
}
#endif /* PAL_USE_CALLBACKS || PAL_USE_WAIT */

#endif /* HAL_USE_PAL */

/** @} */
