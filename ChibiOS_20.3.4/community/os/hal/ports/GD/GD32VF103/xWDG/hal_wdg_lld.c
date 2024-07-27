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
 * @file    xWDG/hal_wdg_lld.c
 * @brief   WDG Driver subsystem low level driver source.
 *
 * @addtogroup WDG
 * @{
 */

#include "hal.h"

#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define CTL_CMD_RELOAD                       0xAAAAU
#define CTL_CMD_ENABLE                       0xCCCCU
#define CTL_CMD_WRITE                        0x5555U
#define CTL_CMD_PROTECT                      0x0000U

#if !defined(FWDGT) && defined(FWDGT1)
#define FWDGT                                FWDGT1
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if GD32_WDG_USE_FWDGT || defined(__DOXYGEN__)
WDGDriver WDGD1;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
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
 * @brief   Low level WDG driver initialization.
 *
 * @notapi
 */
void wdg_lld_init(void) {

#if GD32_WDG_USE_FWDGT
  WDGD1.state = WDG_STOP;
  WDGD1.wdg   = FWDGT;
#endif
}

/**
 * @brief   Configures and activates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_start(WDGDriver *wdgp) {

  /* Enable FWDGT and unlock for write.*/
  wdgp->wdg->CTL   = CTL_CMD_ENABLE;
  wdgp->wdg->CTL   = CTL_CMD_WRITE;

  /* Write configuration.*/
  wdgp->wdg->PSC   = wdgp->config->psc;
  wdgp->wdg->RLD  = wdgp->config->rld;

  /* Wait the registers to be updated.*/
  while (wdgp->wdg->STAT != 0)
    ;

  wdgp->wdg->CTL   = CTL_CMD_RELOAD;
}

/**
 * @brief   Deactivates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_stop(WDGDriver *wdgp) {

  osalDbgAssert(wdgp->state == WDG_STOP,
                "FWDGT cannot be stopped once activated");
}

/**
 * @brief   Reloads WDG's counter.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_reset(WDGDriver * wdgp) {

  wdgp->wdg->CTL = CTL_CMD_RELOAD;
}

#endif /* HAL_USE_WDG == TRUE */

/** @} */
