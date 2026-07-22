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
 * @file    WB32F3G71xx/hal_lld.c
 * @brief   WB32F3G71xx HAL subsystem low level driver source.
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
 * @brief   System Clock Frequency (Core Clock)
 */
uint32_t SystemCoreClock = WB32_MAINCLK;

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
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  void SystemCoreClockUpdate(void);
  SystemCoreClockUpdate();
}

/**
 * @brief  Update SystemCoreClock variable according to Clock Register Values.
 *         The SystemCoreClock variable contains the core clock (HCLK), it can
 *         be used by the user application to setup the SysTick timer or configure
 *         other parameters.
 * @note   Each time the core clock (HCLK) changes, this function must be called
 *         to update SystemCoreClock variable value. Otherwise, any configuration
 *         based on this variable will be incorrect.
 * @param  None
 * @return None
 */
void SystemCoreClockUpdate(void) {

  uint32_t ahbprediv, pllprediv, pllmul, mainclk;

  switch (RCC->MAINCLKSRC) {
    case 0x00: /* MHSI used as main clock */
      mainclk = 8000000;
      break;
    case 0x01: /* FHSI used as main clock */
      mainclk = 48000000;
      break;
    case 0x03: /* HSE used as main clock */
      mainclk = WB32_HSECLK;
      break;
    case 0x02: /* PLL used as main clock */
      pllprediv =
      (((RCC->PLLPRE & (RCC_PLLPRE_RATIO_Msk | RCC_PLLPRE_DIVEN)) + 1) >> 1) + 1;
      pllmul = (0x03 - ((ANCTL->PLLCR >> 6) & 0x03)) * 4 + 12;
      if (RCC->PLLSRC == RCC_PLLSRC_HSE) {
        mainclk = WB32_HSECLK * pllmul / pllprediv;
      }
      else {
        mainclk = 8000000 * pllmul / pllprediv;
      }
      break;
    default:
      mainclk = 8000000;
      break;
  }

  ahbprediv =
    (((RCC->AHBPRE & (RCC_AHBPRE_RATIO_Msk | RCC_AHBPRE_DIVEN)) + 1) >> 1) + 1;
  SystemCoreClock = mainclk / ahbprediv;
}

#if defined(WB32F3G71xx)

/**
 * @brief  Configures the main clock frequency, AHBCLK, APB1CLK and APB2CLK prescalers.
 * @note   This function should be used only after reset.
 * @param  None
 * @return None
 */
static void SetSysClock(void) {
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* Unlocks write to ANCTL registers */
  PWR->ANAKEY1 = 0x03;
  PWR->ANAKEY2 = 0x0C;

  /* APB1CLK = MAINCLK / WB32_PPRE1*/
  RCC->APB1PRE = RCC_APB1PRE_SRCEN;
#if WB32_PPRE1 == 1
  RCC->APB1PRE |= 0x00;
#else
  RCC->APB1PRE |= (WB32_PPRE1 - 2);
  RCC->APB1PRE |= 0x01;
#endif /* WB32_PPRE1 == 1 */

#if WB32_HSE_ENABLED == TRUE
  /* Configure PD0 and PD1 to analog mode */
  RCC->APB1ENR = RCC_APB1ENR_BMX1EN | RCC_APB1ENR_GPIODEN;
  GPIOD->CFGMSK = 0xFFFC;
  GPIOD->MODER = 0x0F;

  /* Enable HSE */
  ANCTL->HSECR1 = ANCTL_HSECR1_PADOEN;
  ANCTL->HSECR0 = ANCTL_HSECR0_HSEON;

  /* Wait till HSE is ready and if Time out is reached exit */
  do {
    HSEStatus = ANCTL->HSESR & ANCTL_HSESR_HSERDY;
    StartUpCounter++;
  } while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if (HSEStatus == 0) {
    /* If HSE fails to start-up, the application will have wrong clock
     * configuration. User can add here some code to deal with this error */
    while (1)
      ;
  }
#endif /* WB32_HSE_ENABLED == TRUE */
  /* Configure Flash prefetch, Cache and wait state */
#if WB32_MAINCLK <= 32000000
  CACHE->CR = CACHE_CR_LATENCY_0WS;
#elif WB32_MAINCLK <= 48000000
  CACHE->CR = CACHE_CR_CHEEN | CACHE_CR_PREFEN_ON | CACHE_CR_LATENCY_1WS;
#elif WB32_MAINCLK <= 72000000
  CACHE->CR = CACHE_CR_CHEEN | CACHE_CR_PREFEN_ON | CACHE_CR_LATENCY_2WS;
#else
  CACHE->CR = CACHE_CR_CHEEN | CACHE_CR_PREFEN_ON | CACHE_CR_LATENCY_3WS;
#endif

  /* AHBCLK = WB32_HPRE */
#if WB32_HPRE == 1
  RCC->AHBPRE = 0x00;
#else
  RCC->AHBPRE = (WB32_HPRE - 2);
  RCC->AHBPRE |= 0x01;
#endif /* WB32_HPRE == 1 */

  /* APB2CLK = MAINCLK / WB32_PPRE2 */
  RCC->APB2PRE = RCC_APB2PRE_SRCEN;
#if WB32_PPRE2 == 1
  RCC->APB2PRE |= 0x00;
#else
  RCC->APB2PRE |= (WB32_PPRE2 - 2);
  RCC->APB2PRE |= 0x01;
#endif /* WB32_PPRE2 == 1 */

#if WB32_PLL_ENABLED == TRUE
  /* PLL configuration:
     PLLCLK = WB32_HSECLK / WB32_PLLDIV_VALUE * WB32_PLLMUL_VALUE*/
  RCC->PLLSRC = WB32_PLLSRC;
  RCC->PLLPRE = RCC_PLLPRE_SRCEN;

#if WB32_PLLDIV_VALUE == 1
  RCC->PLLPRE |= 0x00;
#else
  RCC->PLLPRE |= (WB32_PLLDIV_VALUE - 2);
  RCC->PLLPRE |= 0x01;
#endif /* WB32_PLLDIV_VALUE == 1 */

#if WB32_PLLMUL_VALUE == 12
  ANCTL->PLLCR = (0x3U << 6);
#elif WB32_PLLMUL_VALUE == 16
  ANCTL->PLLCR = (0x2U << 6);
#elif WB32_PLLMUL_VALUE == 20
  ANCTL->PLLCR = (0x1U << 6);
#elif WB32_PLLMUL_VALUE == 24
  ANCTL->PLLCR = (0x0U << 6);
#endif

  /* Enable PLL */
  ANCTL->PLLENR = ANCTL_PLLENR_PLLON;
  /* Wait till PLL is ready */
  while (ANCTL->PLLSR != 0x03) {
  }
#endif /* WB32_PLL_ENABLED == TRUE */

  /* Select WB32_MAINCLKSRC as system clock source */
  RCC->MAINCLKSRC = WB32_MAINCLKSRC;
  RCC->MAINCLKUEN = RCC_MAINCLKUEN_ENA;

  /* Locks write to ANCTL registers */
  PWR->ANAKEY1 = 0x00;
  PWR->ANAKEY2 = 0x00;
}

/**
 * @brief  Clocks initialization.
 * @note   None
 * @param  None
 * @return None
 */
void wb32_clock_init(void) {

#if WB32_NO_INIT == FALSE
  /* Unlocks write to ANCTL registers */
  PWR->ANAKEY1 = 0x03;
  PWR->ANAKEY2 = 0x0C;

  /* Turn off POR */
  ANCTL->PORCR = 0x7BE;

  /* Locks write to ANCTL registers */
  PWR->ANAKEY1 = 0x00;
  PWR->ANAKEY2 = 0x00;

  SetSysClock();

  rccEnableAPB1(RCC_APB1ENR_BMX1EN);
  rccEnableAPB2(RCC_APB2ENR_BMX2EN);

  SCB->VTOR = FLASH_BASE; /* Vector Table Relocation in Internal FLASH. */

#endif /* WB32_NO_INIT == FALSE */
}

#if HAL_USE_USB || defined(__DOXYGEN__)

/**
 * @brief  wb32 usb initialization.
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return None
 */
void wb32_usb_init(USBDriver *usbp) {

  /* Clock activation.*/
#if WB32_USB_USE_USB1
  if (&USBD1 == usbp) {
    RCC->AHBENR1 |= RCC_AHBENR1_CRCSFMEN;

    /* Enable USB peripheral clock */
    RCC->AHBENR1 |= RCC_AHBENR1_USBEN;

    /* Configure USB FIFO clock source */
    RCC->USBFIFOCLKSRC = RCC_USBFIFOCLKSRC_USBCLK;

    /* Enable USB FIFO clock */
    RCC->USBFIFOCLKENR = RCC_USBFIFOCLKENR_CLKEN;

    /* Configure and enable USB PHY */
    SFM->USBPCON = 0x02;

    /* Configure and enable USBCLK */
#if (WB32_USBPRE == WB32_USBPRE_DIV1P5)
    RCC->USBCLKENR = RCC_USBCLKENR_CLKEN;
    RCC->USBPRE = RCC_USBPRE_SRCEN;
    RCC->USBPRE |= RCC_USBPRE_RATIO_1_5;
    RCC->USBPRE |= RCC_USBPRE_DIVEN;
#elif (WB32_USBPRE == WB32_USBPRE_DIV1)
    RCC->USBCLKENR = RCC_USBCLKENR_CLKEN;
    RCC->USBPRE = RCC_USBPRE_SRCEN;
    RCC->USBPRE |= 0x00;
#elif (WB32_USBPRE == WB32_USBPRE_DIV2)
    RCC->USBCLKENR = RCC_USBCLKENR_CLKEN;
    RCC->USBPRE = RCC_USBPRE_SRCEN;
    RCC->USBPRE |= RCC_USBPRE_RATIO_2;
    RCC->USBPRE |= RCC_USBPRE_DIVEN;
#elif (WB32_USBPRE == WB32_USBPRE_DIV3)
    RCC->USBCLKENR = RCC_USBCLKENR_CLKEN;
    RCC->USBPRE = RCC_USBPRE_SRCEN;
    RCC->USBPRE |= RCC_USBPRE_RATIO_3;
    RCC->USBPRE |= RCC_USBPRE_DIVEN;
#else
#error "invalid WB32_USBPRE value specified"
#endif
  }
#endif
}

/**
 * @brief  wb32 usb deinitialization.
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return None
 */
void wb32_usb_deinit(USBDriver *usbp) {

#if WB32_USB_USE_USB1
  if (&USBD1 == usbp) {
    /* Disable USBCLK */
    RCC->USBPRE &= RCC_USBPRE_SRCEN;
    RCC->USBPRE = 0x00;
    RCC->USBCLKENR = 0x00;

    /* Disable USB FIFO clock */
    RCC->USBFIFOCLKENR = 0x0000;

    /* Disable USB peripheral clock */
    RCC->AHBENR1 &= ~RCC_AHBENR1_USBEN;
  }
#endif
}

/**
 * @brief  wb32 usb connect.
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return None
 */
void wb32_usb_connect(USBDriver *usbp) {

  /* Enable BMX1, GPIOA clock */
  RCC->APB1ENR |= RCC_APB1ENR_BMX1EN | RCC_APB1ENR_GPIOAEN;

  GPIOA->CFGMSK = (~(GPIO_CFGMSK_CFGMSK11 | GPIO_CFGMSK_CFGMSK12));
  /* Configure the drive current of PA11 and PA12 */
  GPIOA->CURRENT = (0x3 << 22) | (0x3 << 24);
  /* Configure PA11 and PA12 as Alternate function mode */
  GPIOA->MODER = (0x2 << 22) | (0x2 << 24);
  GPIOA->OTYPER = 0x00;
  GPIOA->OSPEEDR = 0x00;
  GPIOA->PUPDR = 0x00;
  GPIOA->AFRH = (3 << 12) | (3 << 16);

  USB->POWER = USB_POWER_SUSEN;
  USB->INTRUSBE = USB_INTRUSBE_RSTIE | USB_INTRUSBE_RSUIE | USB_INTRUSBE_SUSIE;
}

/**
 * @brief  wb32 usb disconnect.
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return None
 */
void wb32_usb_disconnect(USBDriver *usbp) {

  /* Enable BMX1, GPIOA clock */
  RCC->APB1ENR |= RCC_APB1ENR_BMX1EN | RCC_APB1ENR_GPIOAEN;

  GPIOA->CFGMSK = (~(GPIO_CFGMSK_CFGMSK11 | GPIO_CFGMSK_CFGMSK12));
  /* Configure PA11 and PA12 as input mode */
  GPIOA->MODER = 0x00;
  GPIOA->OSPEEDR = 0x00;
  GPIOA->PUPDR = 0x00;
  /* Configure PA12(D+) as open-drain output mode and output low level */
  GPIOA->CFGMSK = (~GPIO_CFGMSK_CFGMSK12);
  GPIOA->MODER = (0x1 << 24);
  GPIOA->OTYPER = (0x1 << 12);
  GPIOA->AFRH = 0x00;
  GPIOA->BSRR = (0x1000 << 16);
}
#endif

#else

#error "not defined wb32_clock_init"

#endif

/** @} */
