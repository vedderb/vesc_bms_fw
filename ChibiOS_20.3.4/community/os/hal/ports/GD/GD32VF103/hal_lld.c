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
 * @file    GD32VF103/hal_lld.c
 * @brief   GD32VF103 HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   NMSIS system core clock variable.
 * @note    It is declared in system_gd32vf103.h.
 */
uint32_t SystemCoreClock = GD32_HCLK;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Initializes the backup domain.
 * @note    WARNING! Changing clock source impossible without resetting
 *          of the whole BKP domain.
 */
static void hal_lld_backup_domain_init(void) {

  /* Backup domain access enabled and left open.*/
  PMU->CTL |= PMU_CTL_BKPWEN;

#if HAL_USE_RTC
  /* Reset BKP domain if different clock source selected.*/
  if ((RCU->BDCTL & GD32_RTCSRC_MASK) != GD32_RTCSRC) {
    /* Backup domain reset.*/
    RCU->BDCTL = RCU_BDCTL_BKPRST;
    RCU->BDCTL = 0;
  }

  /* If enabled then the LXTAL is started.*/
#if GD32_LXTAL_ENABLED
#if defined(GD32_LXTAL_BYPASS)
  /* LXTAL Bypass.*/
  RCU->BDCTL |= RCU_BDCTL_LXTALEN | RCU_BDCTL_LXTALBPS;
#else
  /* No LXTAL Bypass.*/
  RCU->BDCTL |= RCU_BDCTL_LXTALEN;
#endif
  while ((RCU->BDCTL & RCU_BDCTL_LXTALSTB) == 0)
    ;                                     /* Waits until LXTAL is stable.   */
#endif /* GD32_LXTAL_ENABLED */

#if GD32_RTCSRC != GD32_RTCSRC_NOCLOCK
  /* If the backup domain hasn't been initialized yet then proceed with
     initialization.*/
  if ((RCU->BDCTL & RCU_BDCTL_RTCEN) == 0) {
    /* Selects clock source.*/
    RCU->BDCTL |= GD32_RTCSRC;

    /* Prescaler value loaded in registers.*/
    rtc_lld_set_prescaler();

    /* RTC clock enabled.*/
    RCU->BDCTL |= RCU_BDCTL_RTCEN;
  }
#endif /* GD32_RTCSRC != GD32_RTCSRC_NOCLOCK */
#endif /* HAL_USE_RTC */
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  /* Reset of all peripherals.*/
  rcuResetAPB1(0xFFFFFFFF);
  rcuResetAPB2(0xFFFFFFFF);

  /* PMU and BD clocks enabled.*/
  rcuEnablePMUInterface(true);
  rcuEnableBKPInterface(true);

  /* Initializes the backup domain.*/
  hal_lld_backup_domain_init();

  /* DMA subsystems initialization.*/
#if defined(GD32_DMA_REQUIRED)
  dmaInit();
#endif

  /* IRQ subsystem initialization.*/
  irqInit();

  /* Programmable voltage detector enable.*/
#if GD32_PVD_ENABLE
  PMU->CTL |= PMU_CTL_LVDEN | (GD32_LVDT & GD32_LVDT_MASK);
#endif /* GD32_PVD_ENABLE */
}

/**
 * @brief   GD32 clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */

/*
 * Clocks initialization for the CL sub-family.
 */
void gd32_clock_init(void) {

#if !GD32_NO_INIT
  /* IRC8M setup, it enforces the reset situation in order to handle possible
     problems with JTAG probes and re-initializations.*/
  RCU->CTL |= RCU_CTL_IRC8MEN;                  /* Make sure IRC8M is ON.         */
  while (!(RCU->CTL & RCU_CTL_IRC8MSTB))
    ;                                       /* Wait until IRC8M is stable.    */

  /* IRC8M is selected as new source without touching the other fields in
     CFGR. Clearing the register has to be postponed after IRC8M is the
     new source.*/
  RCU->CFG0 &= ~RCU_CFG0_SCS;                /* Reset SW, selecting IRC8M.     */
  while ((RCU->CFG0 & RCU_CFG0_SCSS) != RCU_CFG0_SCSS_IRC8M)
    ;                                       /* Wait until IRC8M is selected.  */

  /* Registers finally cleared to reset values.*/
  RCU->CTL &= RCU_CTL_IRC8MADJ | RCU_CTL_IRC8MEN; /* CR Reset value.              */
  RCU->CFG0 = 0;                            /* CFGR reset value.            */

#if GD32_HXTAL_ENABLED
#if defined(GD32_HXTAL_BYPASS)
  /* HXTAL Bypass.*/
  RCU->CTL |= RCU_CTL_HXTALBPS;
#endif
  /* HXTAL activation.*/
  RCU->CTL |= RCU_CTL_HXTALEN;
  while (!(RCU->CTL & RCU_CTL_HXTALSTB))
    ;                                       /* Waits until HXTAL is stable.   */
#endif

#if GD32_IRC40K_ENABLED
  /* IRC40K activation.*/
  RCU->RSTSCK |= RCU_RSTSCK_IRC40KEN;
  while ((RCU->RSTSCK & RCU_RSTSCK_IRC40KSTB) == 0)
    ;                                       /* Waits until IRC40K is stable.   */
#endif

  /* Settings of various dividers and multipliers in CFGR2.*/
  RCU->CFG1 = GD32_PLL2MF | GD32_PLL1MF | GD32_PREDV1 |
               GD32_PREDV0 | GD32_PREDV0SEL;

  /* PLL1 setup, if activated.*/
#if GD32_ACTIVATE_PLL1
  RCU->CTL |= RCU_CTL_PLL1EN;
  while (!(RCU->CTL & RCU_CTL_PLL1STB))
    ;                                        /* Waits until PLL1 is stable. */
#endif

  /* PLL2 setup, if activated.*/
#if GD32_ACTIVATE_PLL2
  RCU->CTL |= RCU_CTL_PLL2EN;
  while (!(RCU->CTL & RCU_CTL_PLL2STB))
    ;                                        /* Waits until PLL2 is stable. */
#endif

  /* PLL setup, if activated.*/
#if GD32_ACTIVATE_PLL
  RCU->CFG0 |= GD32_PLLMF | GD32_PLLSEL;
  RCU->CTL   |= RCU_CTL_PLLEN;
  while (!(RCU->CTL & RCU_CTL_PLLSTB))
    ;                           /* Waits until PLL is stable.              */
#endif

  /* Clock settings.*/
#if GD32_HAS_USBFS
  RCU->CFG0 = GD32_CKOUT0SEL | GD32_USBFSPSC   | GD32_PLLMF | GD32_PLLSEL |
              GD32_ADCPSC | GD32_APB2PSC    | GD32_APB1PSC  | GD32_AHBPSC;
#else
  RCU->CFG0 = GD32_MCO    |                  GD32_PLLMF | GD32_PLLSEL |
              GD32_ADCPSC | GD32_APB2PSC    | GD32_APB1PSC  | GD32_AHBPSC;
#endif

  /* Flash setup and final clock selection.   */
  FLASH->WS = GD32_FLASHBITS; /* Flash wait states depending on clock.    */
  while ((FLASH->WS & FLASH_WS_WSCNT_Msk) !=
         (GD32_FLASHBITS & FLASH_WS_WSCNT_Msk)) {
  }

  /* Switching to the configured clock source if it is different from IRC8M.*/
#if (GD32_SCS != GD32_SCS_IRC8M)
  RCU->CFG0 |= GD32_SCS;        /* Switches on the selected clock source.   */
  while ((RCU->CFG0 & RCU_CFG0_SCSS) != (GD32_SCS << 2))
    ;
#endif

#if !GD32_IRC8M_ENABLED
  RCU->CTL &= ~RCU_CTL_IRC8MEN;
#endif
#endif /* !GD32_NO_INIT */
}

/** @} */
