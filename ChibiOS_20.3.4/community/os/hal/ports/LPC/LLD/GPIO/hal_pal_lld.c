/*
    ChibiOS - Copyright (C) 2020 Yaotian Feng / Codetector

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
 * @file    hal_pal_lld.c
 * @brief   LPC11Uxx PAL subsystem low level driver source.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

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
 * @brief   LPC1Uxx I/O ports configuration.
 *
 * @notapi
 */
void _pal_lld_init(void) {
  // Enable GPIO / IOCON CLK
  LPC_SYSCON->SYSAHBCLKCTRL |= SYSCON_SYSAHBCLKCTRL_GPIO | SYSCON_SYSAHBCLKCTRL_IOCON;
}

/**
 * @brief   Group Pads mode setup.
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
  for (uint8_t i = 0; i < PAL_IOPORTS_WIDTH; ++i)
  {
    if (mask & (1U << i)) {
      _pal_lld_setpadmode(port, i, mode);
    }
  }
}


/**
 * @brief   Pads mode setup.
 * @details This function programs a pads with the specified mode.
 *
 * @param[in] port      the port identifier
 * @param[in] pad       the pad id
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setpadmode(ioportid_t port, iopadid_t pad, iomode_t mode) {
  while (pad > 0x1F){}
  uint32_t* base = (uint32_t*)0x40044000;
  if (LPC_IOPORT_NUM(port) == 1) {
    base = (uint32_t*)0x40044060;
  }
  base[pad & 0x1F]  = (mode & MODE_IOCONF_MASK);
  if (mode & MODE_DIR_MASK) {
    LPC_GPIO->DIR[LPC_IOPORT_NUM(port)] |= (uint32_t) 1U << pad;
  } else {
    LPC_GPIO->DIR[LPC_IOPORT_NUM(port)] &= ~(((uint32_t)1U) << pad);
  }
}

#endif /* HAL_USE_PAL == TRUE */

/** @} */
