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

    Portions Copyright (C) 2017 PJRC.COM, LLC.

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the
    "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish,
    distribute, sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so, subject to
    the following conditions:
    
    1. The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.
    
    2. If the Software is incorporated into a build system that allows
    selection among a list of target devices, then similar target
    devices manufactured by PJRC.COM must be included in the list of
    target devices and selectable in the same manner.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
    BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
    ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

*/

/**
 * @file    MIMXRT1062/hal_lld.c
 * @brief   MIMXRT1062 HAL Driver subsystem low level driver source template.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

#include "printf_debug.h"

#include "clock_config.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#ifdef __CC_ARM
__attribute__ ((section(".ARM.__at_0x400")))
#else
__attribute__ ((used,section(".cfmconfig")))
#endif
const uint8_t _cfm[0x10] = {
  0xFF,  /* NV_BACKKEY3: KEY=0xFF */
  0xFF,  /* NV_BACKKEY2: KEY=0xFF */
  0xFF,  /* NV_BACKKEY1: KEY=0xFF */
  0xFF,  /* NV_BACKKEY0: KEY=0xFF */
  0xFF,  /* NV_BACKKEY7: KEY=0xFF */
  0xFF,  /* NV_BACKKEY6: KEY=0xFF */
  0xFF,  /* NV_BACKKEY5: KEY=0xFF */
  0xFF,  /* NV_BACKKEY4: KEY=0xFF */
  0xFF,  /* NV_FPROT3: PROT=0xFF */
  0xFF,  /* NV_FPROT2: PROT=0xFF */
  0xFF,  /* NV_FPROT1: PROT=0xFF */
  0xFF,  /* NV_FPROT0: PROT=0xFF */
  0x7E,  /* NV_FSEC: KEYEN=1,MEEN=3,FSLACC=3,SEC=2 */
  0xFF,  /* NV_FOPT: ??=1,??=1,FAST_INIT=1,LPBOOT1=1,RESET_PIN_CFG=1,
                      NMI_DIS=1,EZPORT_DIS=1,LPBOOT0=1 */
  0xFF,
  0xFF
};

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
 * @todo    Use a macro to define the system clock frequency.
 *
 * @notapi
 */
void hal_lld_init(void) {
}


#define NVIC_NUM_INTERRUPTS 160
extern uint32_t _vectors[NVIC_NUM_INTERRUPTS];

uint32_t SystemCoreClock = 528000000UL; // default system clock
  
void MIMXRT1062_clock_init(void) {
  // We need the IOMUXC for GPIO to work. We do not need BOARD_InitPins() from
  // the NXP SDK, we use the ChibiOS PAL HAL instead.
  CLOCK_EnableClock(kCLOCK_Iomuxc);

  BOARD_InitBootClocks();
  //SystemCoreClockUpdate();

  // TODO: is the following covered by NXP startup code somewhere?

  // Configure the GPIO pins from the standard-speed GPIOs onto the high-speed
  // GPIOs GPIO6, GPIO7, GPIO8, and GPIO9:
  //
  // See also IMXRT1060RM, page 949, 12.1 Chip-specific GPIO information.
  IOMUXC_GPR->GPR26 = 0xFFFFFFFF;
  IOMUXC_GPR->GPR27 = 0xFFFFFFFF;
  IOMUXC_GPR->GPR28 = 0xFFFFFFFF;
  IOMUXC_GPR->GPR29 = 0xFFFFFFFF;
  
  // Turn on the Teensy power LED on pin 13, which users generally expect:
  IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03] =
    PIN_MUX_ALTERNATIVE(5);
  GPIO7->GDIR |= ((uint32_t) 1 << 3);
  GPIO7->DR_SET = ((uint32_t) 1 << 3);
  
  printf_debug_init();
  printf_debug("\n***********IMXRT Chibi Startup**********\n");
  printf_debug("test %d %d %d\n", 1, -1234567, 3);

  // Explicitly warn for a few issues I ran into, to catch possible
  // problems automatically when working on startup code:
  if (SCB->VTOR != (uint32_t)&_vectors) {
    printf_debug("WARNING: unexpected SCB->VTOR value %x, expected &_vectors = %x\n", SCB->VTOR, (uint32_t)&_vectors);
  }
  if ((uint32_t)&_vectors % 4 != 0) {
    printf_debug("WARNING: &_vectors = %x is unexpectedly not aligned by 4\n", (uint32_t)&_vectors);
  }
  if (CORTEX_NUM_VECTORS != 160) {
    printf_debug("WARNING: unexpected CORTEX_NUM_VECTORS = %d, want %d", CORTEX_NUM_VECTORS, 160);
  }
}

/** @} */
