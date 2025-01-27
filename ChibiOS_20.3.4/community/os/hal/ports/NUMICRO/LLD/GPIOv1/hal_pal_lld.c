/*
    Copyright (C) 2019 /u/KeepItUnder

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
 * @brief   PLATFORM PAL subsystem low level driver source.
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

/* DEBUG

OSAL_IRQ_HANDLER(NUC123_GPIOAB_HANDLER){
    OSAL_IRQ_PROLOGUE();

  GPIO_TOGGLE(PB4);
  GPIO_TOGGLE(PB5);
  GPIO_TOGGLE(PB6);
  GPIO_TOGGLE(PB7);
  GPIO_TOGGLE(PB8);

    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(NUC123_GPIOCDF_HANDLER){
    OSAL_IRQ_PROLOGUE();

  GPIO_TOGGLE(PB4);
  GPIO_TOGGLE(PB5);
  GPIO_TOGGLE(PB6);
  GPIO_TOGGLE(PB7);
  GPIO_TOGGLE(PB8);

    OSAL_IRQ_EPILOGUE();
}
*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   NUC123 I/O ports configuration.
 * @details Ports A-D(E, F, G, H) clocks enabled.
 *
 * @param[in] config    the NUC123 ports configuration
 *
 * @notapi
 */
#if defined(PAL_NEW_INIT)
void _pal_lld_init(void) {
  /* Set DeBounce conditions */
  GPIO->DBNCECON = 0x04u;
}
#else
void _pal_lld_init(const PALConfig *config) {

  /* (void)config; */
  /* Turn on GPIO subsystem
   * Set all GPIO to Input/HZ
   * Clear all GPIO Interrupts
   * Set all GPIO Interrupt Modes to Level
   * Turn off DeBounce
   * Zero all GPIO Outputs (just in case)
   */

  GPIOA->PMD = config->PAData.PMD;
  GPIOA->OFFD = config->PAData.OFFD;
  GPIOA->DMASK = config->PAData.DMASK;
  GPIOA->DBEN = config->PAData.DBEN;
  GPIOA->IMD = config->PAData.IMD;
  GPIOA->IEN = config->PAData.IEN;
  GPIOA->ISRC = config->PAData.ISRC;
  GPIOA->DOUT = config->PAData.DOUT;

  GPIOB->PMD = config->PBData.PMD;
  GPIOB->OFFD = config->PBData.OFFD;
  GPIOB->DMASK = config->PBData.DMASK;
  GPIOB->DBEN = config->PBData.DBEN;
  GPIOB->IMD = config->PBData.IMD;
  GPIOB->IEN = config->PBData.IEN;
  GPIOB->ISRC = config->PBData.ISRC;
  GPIOB->DOUT = config->PBData.DOUT;

  GPIOC->PMD = config->PCData.PMD;
  GPIOC->OFFD = config->PCData.OFFD;
  GPIOC->DMASK = config->PCData.DMASK;
  GPIOC->DBEN = config->PCData.DBEN;
  GPIOC->IMD = config->PCData.IMD;
  GPIOC->IEN = config->PCData.IEN;
  GPIOC->ISRC = config->PCData.ISRC;
  GPIOC->DOUT = config->PCData.DOUT;

  GPIOD->PMD = config->PDData.PMD;
  GPIOD->OFFD = config->PDData.OFFD;
  GPIOD->DMASK = config->PDData.DMASK;
  GPIOD->DBEN = config->PDData.DBEN;
  GPIOD->IMD = config->PDData.IMD;
  GPIOD->IEN = config->PDData.IEN;
  GPIOD->ISRC = config->PDData.ISRC;
  GPIOD->DOUT = config->PDData.DOUT;

  GPIOF->PMD = config->PFData.PMD;
  GPIOF->OFFD = config->PFData.OFFD;
  GPIOF->DMASK = config->PFData.DMASK;
  GPIOF->DBEN = config->PFData.DBEN;
  GPIOF->IMD = config->PFData.IMD;
  GPIOF->IEN = config->PFData.IEN;
  GPIOF->ISRC = config->PFData.ISRC;
  GPIOF->DOUT = config->PFData.DOUT;

  /* Set DeBounce conditions */
  GPIO->DBNCECON = 0x04u;
}
#endif

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

  uint32_t nucMode = 0;

  if (mode == PAL_MODE_INPUT)
      nucMode = GPIO_PMD_INPUT;
  else if (mode == PAL_MODE_OUTPUT_OPENDRAIN)
      nucMode = GPIO_PMD_OPEN_DRAIN;
  else if (mode == PAL_MODE_OUTPUT_PUSHPULL)
      nucMode = GPIO_PMD_OUTPUT;
  else /* mode == PAL_MODE_INPUT_PULLUP */
      nucMode = GPIO_PMD_QUASI;

  for (uint32_t i = 0; i < PAL_IOPORTS_WIDTH; i++) {
    if (mask & (1 << i)) {
      port->PMD = (port->PMD & ~(0x03ul << (i << 1))) | (nucMode << (i << 1));
    }
  }

  if (nucMode == GPIO_PMD_QUASI) {
    port->DOUT |= (uint32_t)(uint16_t)mask;
  }
}

#endif /* HAL_USE_PAL == TRUE */

/** @} */
