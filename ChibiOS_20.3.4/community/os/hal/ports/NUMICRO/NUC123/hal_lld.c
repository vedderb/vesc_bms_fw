/*
  Copyright (C) 2020 Alex Lewontin
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
 * @file    hal_lld.c
 * @brief   NUC123 HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#define FREQ_25MHZ  25000000
#define FREQ_50MHZ  50000000
#define FREQ_72MHZ  72000000
#define FREQ_100MHZ 100000000
#define FREQ_200MHZ 200000000

#define CLK_CLKDIV_HCLK(x) (((x)-1) << CLK_CLKDIV_HCLK_N_Pos)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

_Bool clock_initialized = FALSE;

uint32_t SystemCoreClock = __HSI; /* System Clock Frequency (Core Clock)*/
uint32_t CyclesPerUs     = (__HSI / 1000000); /* Cycles per micro second */
uint32_t PllClock        = __HSI;             /*!< PLL Clock Frequency */

#if (NUC123_CONFIG_ENABLED == TRUE)

static volatile const uint32_t config0 __attribute__((used, unused, section(".nuc123_config0"))) = NUC123_CONFIG0;
static volatile const uint32_t config1 __attribute__((used, unused, section(".nuc123_config1"))) = NUC123_CONFIG1;

#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void SystemCoreClockUpdate(void) /* Get Core Clock Frequency      */
{
  /* ToDo: add code to calculate the system frequency based upon the current
         register settings.
         This function can be used to retrieve the system core clock frequeny
         after user changed register sittings. */
  /* SystemCoreClock = SYSTEM_CLOCK; */

  uint32_t clkFreq;
  uint32_t PllReg;

  uint32_t pllFIN, pllNF, pllNR, pllNO;

  /* Update PLL Clock */
  /* PllClock = clks_lld_get_pll_clock_freq(); */
  PllReg = CLK->PLLCON;

  if (PllReg & (CLK_PLLCON_PD_Msk | CLK_PLLCON_OE_Msk)) {
    PllClock = 0; /* PLL is off. */
  } else {

    if (PllReg & 0x00080000ul) {
      pllFIN = __HIRC; /* Use HXT for PLL clock */
    } else {
      pllFIN = NUC123_HSECLK; /* Use HXT for PLL clock */
    }

    if (PllReg & CLK_PLLCON_BP_Msk) {
      PllClock = pllFIN;
    } else {
      switch (((PllReg & CLK_PLLCON_OUT_DV_Msk) >> CLK_PLLCON_OUT_DV_Pos)) {
      case 0: /* OUT_DIV == 00 : NO = 1 */
        pllNO = 1;
        break;
      case 3: /* OUT_DIV == 11 : NO = 4 */
        pllNO = 4;
        break;
      default: /* OUT_DIV == 01 or 10 : NO = 2 */
        pllNO = 2;
        break;
      }

      pllNF = ((PllReg & CLK_PLLCON_FB_DV_Msk) >> CLK_PLLCON_FB_DV_Pos) + 2;
      pllNR = ((PllReg & CLK_PLLCON_IN_DV_Msk) >> CLK_PLLCON_IN_DV_Pos) + 2;

      /* Shift right to avoid overflow condition */
      PllClock = (((pllFIN >> 2) * pllNF) / (pllNR * pllNO) << 2);
    }
  }

  /* Pick Clock Source */
  switch (CLK->CLKSEL0 & CLK_CLKSEL0_HCLK_S_Msk) {
  case 0: /* External HF Xtal */
    clkFreq = NUC123_HSECLK;
    break;
  case 1: /* PLL clock / 2 */
    clkFreq = PllClock >> 1;
    break;
  case 3: /* Internal 10kHz */
    clkFreq = __LIRC;
    break;
  case 2: /* PLL clock */
    clkFreq = PllClock;
    break;
  case 7: /* Internal 22.184MHz */
    clkFreq = __HIRC;
    break;
  default:
    clkFreq = 0;
    break;
  }

  SystemCoreClock = clkFreq / ((CLK->CLKDIV & CLK_CLKDIV_HCLK_N_Msk) + 1);
  CyclesPerUs = SystemCoreClock / 1000000;
}

/**
  * @brief      Get PLL clock frequency
  * @param      None
  * @return     PLL frequency
  * @details    This function get PLL frequency. The frequency unit is Hz.
  */
static inline uint32_t get_pll_clock_freq(void)
{
  uint32_t PllReg;
  uint32_t pllFIN, pllNF, pllNR, pllNO;

  PllReg = CLK->PLLCON;

  if (PllReg & (CLK_PLLCON_PD_Msk | CLK_PLLCON_OE_Msk)) {
    PllClock = 0; /* PLL is in power down mode or fix low */
  } else {

    if (PllReg & NUC123_PLLSRC_HSI) {
      pllFIN = __HIRC; /* Use HXT for PLL clock */
    } else {
      pllFIN = NUC123_HSECLK; /* Use HXT for PLL clock */
    }

    if (PllReg & CLK_PLLCON_BP_Msk) {
      PllClock = pllFIN;
    } else {
      switch (((PllReg & CLK_PLLCON_OUT_DV_Msk) >> CLK_PLLCON_OUT_DV_Pos)) {
      case 0: /* OUT_DIV == 00 : NO = 1 */
        pllNO = 1;
        break;
      case 3: /* OUT_DIV == 11 : NO = 4 */
        pllNO = 4;
        break;
      default: /* OUT_DIV == 01 or 10 : NO = 2 */
        pllNO = 2;
        break;
      }

      pllNF = ((PllReg & CLK_PLLCON_FB_DV_Msk) >> CLK_PLLCON_FB_DV_Pos) + 2;
      pllNR = ((PllReg & CLK_PLLCON_IN_DV_Msk) >> CLK_PLLCON_IN_DV_Pos) + 2;

      /* Shift to avoid overflow condition */
      PllClock = (((pllFIN >> 2) * pllNF) / (pllNR * pllNO) << 2);
    }
  }

  return PllClock;
}

/**
  * @brief         Wait for stable clock
  *
  * @description   Always wait around 300ms for clock to be stable
  *
  */
static uint32_t wait_for_clock_ready(uint32_t clkMask)
{
  int32_t timeout = 2180000;

  while (timeout-- > 0) {
    if ((CLK->CLKSTATUS & clkMask) == clkMask) {
      return 1;
    }
  }

  return 0;
}

/** @brief Set system HCLK
 *
 * @description Setup HCLK source and divider
 *
 * Always switch to a known stable clock source before changing a
 * system clock, to avoid issues related to the original clock's
 * speed/settings.
 *
 */
static void set_HCLK(uint32_t clkSource, uint32_t clkDivider)
{
  uint32_t stableHIRC;

  /* Read HIRC clock source stable flag */
  stableHIRC = CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk;

  /* Setup __HIRC */
  CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

  wait_for_clock_ready(CLK_CLKSTATUS_OSC22M_STB_Msk);

  /* Use __HIRC as HCLK, temporarily */
  CLK->CLKSEL0 =
      (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | NUC123_HCLKSRC_HSI;

  /* Set new clock divider */
  CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | clkDivider;

  /* Switch HCLK to new HCLK source */
  CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | clkSource;

  /* Update System Core Clock */
  SystemCoreClockUpdate();

  /* Disable HIRC if HIRC was disabled before we started */
  if (stableHIRC == 0) {
    CLK->PWRCON &= ~CLK_PWRCON_OSC22M_EN_Msk;
  }
}

#if NUC123_PLL_ENABLED
static uint32_t enable_pll(uint32_t pllSrc, uint32_t pllFreq)
{
  /* Disable PLL first to avoid unstable when setting PLL. */
  CLK->PLLCON = CLK_PLLCON_PD_Msk;

  /* Check and setup correct clock source */
  switch (pllSrc) {
  case NUC123_PLLSRC_HSE:
    /* Use HXT clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Wait for stable HXT */
    wait_for_clock_ready(CLK_CLKSTATUS_XTL12M_STB_Msk);

    break;
  case NUC123_PLLSRC_HSI:
    /* Use HIRC clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Wait for stable HIRC */
    wait_for_clock_ready(CLK_CLKSTATUS_OSC22M_STB_Msk);

    break;
  }

  /**
     * Calculate best PLL variables from requested frequency
     *
     * See NUC123 Technical Reference Manual 5.4.8 PLL Control Register Description, page 124
     *
     *                NF     1
     * FOUT = FIN  x  --  x  --
     *                NR     NO
     *
     */

  uint32_t NO      = 0;
  uint32_t NR      = 0;
  uint32_t clkCalc = 0;

  /* Set "NO" for requested frequency */
  /* We're using "NO" first to set the PLLCON - so make it "NO" - 1; */
  if (pllFreq >= FREQ_25MHZ && pllFreq <= FREQ_50MHZ) {
    /* Low frequency - use full variable headroom */
    pllFreq <<= 2;
    NO = 3;
  } else if (pllFreq > FREQ_50MHZ && pllFreq <= FREQ_100MHZ) {
    /* Medium frequency - use full variable headroom */
    pllFreq <<= 1;
    NO = 1;
  } else if (pllFreq > FREQ_100MHZ && pllFreq <= FREQ_200MHZ) {
    /* High frequency - full variable headroom already used */
    NO = 0;
  } else {
    /* Frequency out of range - use default PLL settings
         *
         * See NUC123 Technical Reference Manual PLL COntrol Register Description, page 124
         * The default value: 0xC22E
         *   FIN = 12 MHz
         *   NR = (1+2) = 3
         *   NF = (46+2) = 48
         *   NO = 4
         *   FOUT = 12/4 x 48 x 1/3 = 48 MHz
         */
    if (pllSrc == NUC123_PLLSRC_HSE) {
      CLK->PLLCON = 0xC22E;
    } else {
      CLK->PLLCON = 0xD66F;
    }

    /* Wait for stable PLL clock */
    wait_for_clock_ready(CLK_CLKSTATUS_PLL_STB_Msk);

    return get_pll_clock_freq();
  }

  /* Setup "NR" and clkCalc */
  switch (pllSrc) {
  case NUC123_PLLSRC_HSE:
    NR      = 2;
    clkCalc = NUC123_HSECLK;
    break;
  case NUC123_PLLSRC_HSI:
    NR      = 4;
    clkCalc = __HIRC;
    break;
  }

  /**
     * Loop to calculate best/lowest NR (between 0 or 2 and 31) and best/lowest NF (between 0 and 511)
     *
     * Best results are off-by-2 until final equation calculation (to allow use in PLLCON)
     *
     */
  uint32_t bestNR   = 0;
  uint32_t bestNF   = 0;
  uint32_t minLimit = -1;

  while (NR <= 33) {
    uint32_t tmpCalc1 = clkCalc / NR;

    if (tmpCalc1 > 1600000 && tmpCalc1 < 16000000) {
      uint32_t NF = 2;

      while (NF <= 513) {
        uint32_t tmpCalc2 = tmpCalc1 * NF;

        if (tmpCalc2 >= 100000000 && tmpCalc2 <= 200000000) {
          uint32_t tmpCalc3;

          if (tmpCalc2 > pllFreq) {
            tmpCalc3 = tmpCalc2 - pllFreq;
          } else {
            tmpCalc3 = pllFreq - tmpCalc2;
          }

          if (tmpCalc3 < minLimit) {
            minLimit = tmpCalc3;
            bestNF   = NF;
            bestNR   = NR;

            /* Stop NF calc loop when minLimit tends back to 0 */
            if (minLimit == 0)
              break;
          }
        }

        NF++;
      }
    }

    NR++;
  }

  /* Enable and apply new PLL setting. */
  CLK->PLLCON = pllSrc | (NO << 14) | ((bestNR - 2) << 9) | (bestNF - 2);

  /* Wait for stable PLL clock */
  wait_for_clock_ready(CLK_CLKSTATUS_PLL_STB_Msk);

  /* Return equation result */
  return (clkCalc / ((NO + 1) * bestNR) * bestNF);
}

/** @brief Set Core Clock
 *
 * @description Set the core system clock some reference speed (Hz).
 *              This should be between 25MHz and 72MHz for the NUC123SD4AN0.
 *
 *              Use either the HXT (exact) or HIRC (nearest using 22.1184MHz)
 *              as the clock source.
 *
 */
static uint32_t set_core_clock(uint32_t clkCore)
{
  uint32_t stableHIRC;

  /* Read HIRC clock source stable flag */
  stableHIRC = CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk;

  /* Setup __HIRC */
  CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

  wait_for_clock_ready(CLK_CLKSTATUS_OSC22M_STB_Msk);

  /* Use __HIRC as HCLK temporarily */
  CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_Msk;
  CLK->CLKDIV &= (~CLK_CLKDIV_HCLK_N_Msk);

  /* Is HXT stable ? */
  if (CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk) {
    /* Use NUC123_HSECLK as PLL source */
    clkCore = enable_pll(NUC123_PLLSRC_HSE, (2 * clkCore));
  } else {
    /* Use __HIRC as PLL source */
    clkCore = enable_pll(NUC123_PLLSRC_HSI, (2 * clkCore));

    /* Read HIRC clock source stable flag again (since we're using it now) */
    stableHIRC = CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk;
  }

  /* Set HCLK clock source to PLL */
  set_HCLK(NUC123_HCLKSRC_PLL_2, CLK_CLKDIV_HCLK(1));

  /* Disable HIRC if HIRC was disabled before we started */
  if (stableHIRC == 0) {
    CLK->PWRCON &= ~CLK_PWRCON_OSC22M_EN_Msk;
  }

  /* Return actual HCLK frequency is PLL frequency divide 2 */
  return (clkCore >> 1);
}
#endif

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
void hal_lld_init(void)
{
  if (!clock_initialized) {
    NUC123_clock_init();
  }
}

void NUC123_clock_init(void)
{
  clock_initialized = TRUE;
  UNLOCKREG();

  /* Always initialize HSI and go from there, things can change later */
  /* TODO: Technically this could also be the crystal, figure out how to allow
   * config in linker? */
  /* Enable HSI */
  CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;
  wait_for_clock_ready(CLK_CLKSTATUS_OSC22M_STB_Msk);

  set_HCLK(NUC123_HCLKSRC_HSI, CLK_CLKDIV_HCLK(1));

#if NUC123_HSE_ENABLED
  /* SYS->GPF_MFP |= (SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN); */
  SYS->GPF_MFP |= (SYS_GPF_MFP_GPF_MFP0_Msk | SYS_GPF_MFP_GPF_MFP1_Msk);

  CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;
  wait_for_clock_ready(CLK_CLKSTATUS_XTL12M_STB_Msk);
#endif /* NUC123_HSE_ENABLED */

#if NUC123_LSI_ENABLED
  CLK->PWRCON |= CLK_PWRCON_IRC10K_EN_Msk;
  wait_for_clock_ready(CLK_CLKSTATUS_IRC10K_STB_Msk);
#endif /* NUC123_LSI_ENABLED */

#if NUC123_PLL_ENABLED
  set_core_clock(NUC123_HCLK);
#endif /* NUC123_PLL_ENABLED */

  LOCKREG();
}

/** @} */
