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
 * @file    MIMXRT1062/mpu.c
 * @brief   Memory Protection Unit (MPU) configuration.
 * @note    Be careful when changing the linker script or FlexRAM config:
 *          do not configure MPU regions for memory regions that are
 *          not currently available, otherwise the device may hang.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

#include "printf_debug.h"

#include "clock_config.h"

// TODO: port this to CMSIS style headers
#define SCB_MPU_TYPE            (*(volatile uint32_t *)0xE000ED90) // 
#define SCB_MPU_CTRL            (*(volatile uint32_t *)0xE000ED94) // 
#define SCB_MPU_CTRL_PRIVDEFENA         ((uint32_t)(1<<2)) // Enables default memory map
#define SCB_MPU_CTRL_HFNMIENA           ((uint32_t)(1<<1)) // Use MPU for HardFault & NMI
#define SCB_MPU_CTRL_ENABLE             ((uint32_t)(1<<0)) // Enables MPU
#define SCB_MPU_RNR             (*(volatile uint32_t *)0xE000ED98) // 
#define SCB_MPU_RBAR            (*(volatile uint32_t *)0xE000ED9C) // 
#define SCB_MPU_RBAR_ADDR_MASK          ((uint32_t)(0xFFFFFFE0))
#define SCB_MPU_RBAR_VALID              ((uint32_t)(1<<4))
#define SCB_MPU_RBAR_REGION(n)          ((uint32_t)((n) & 15))
#define SCB_MPU_RASR            (*(volatile uint32_t *)0xE000EDA0) // ARM DDI0403E, pg 696
#define SCB_MPU_RASR_XN                 ((uint32_t)(1<<28))
#define SCB_MPU_RASR_AP(n)              ((uint32_t)(((n) & 7) << 24))
#define SCB_MPU_RASR_TEX(n)             ((uint32_t)(((n) & 7) << 19))
#define SCB_MPU_RASR_S                  ((uint32_t)(1<<18))
#define SCB_MPU_RASR_C                  ((uint32_t)(1<<17))
#define SCB_MPU_RASR_B                  ((uint32_t)(1<<16))
#define SCB_MPU_RASR_SRD(n)             ((uint32_t)(((n) & 255) << 8))
#define SCB_MPU_RASR_SIZE(n)            ((uint32_t)(((n) & 31) << 1))
#define SCB_MPU_RASR_ENABLE             ((uint32_t)(1<<0))
#define SCB_MPU_RBAR_A1         (*(volatile uint32_t *)0xE000EDA4) // 
#define SCB_MPU_RASR_A1         (*(volatile uint32_t *)0xE000EDA8) // 
#define SCB_MPU_RBAR_A2         (*(volatile uint32_t *)0xE000EDAC) // 
#define SCB_MPU_RASR_A2         (*(volatile uint32_t *)0xE000EDB0) // 
#define SCB_MPU_RBAR_A3         (*(volatile uint32_t *)0xE000EDB4) // 
#define SCB_MPU_RASR_A3         (*(volatile uint32_t *)0xE000EDB8) // 


// concise defines for SCB_MPU_RASR and SCB_MPU_RBAR, ARM DDI0403E, pg 696
#define NOEXEC		SCB_MPU_RASR_XN
#define READONLY	SCB_MPU_RASR_AP(7)
#define READWRITE	SCB_MPU_RASR_AP(3)
#define NOACCESS	SCB_MPU_RASR_AP(0)
#define MEM_CACHE_WT	SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C
#define MEM_CACHE_WB	SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_CACHE_WBWA	SCB_MPU_RASR_TEX(1) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_NOCACHE	SCB_MPU_RASR_TEX(1)
#define DEV_NOCACHE	SCB_MPU_RASR_TEX(2)
#define SIZE_32B	(SCB_MPU_RASR_SIZE(4) | SCB_MPU_RASR_ENABLE)
#define SIZE_64B	(SCB_MPU_RASR_SIZE(5) | SCB_MPU_RASR_ENABLE)
#define SIZE_128B	(SCB_MPU_RASR_SIZE(6) | SCB_MPU_RASR_ENABLE)
#define SIZE_256B	(SCB_MPU_RASR_SIZE(7) | SCB_MPU_RASR_ENABLE)
#define SIZE_512B	(SCB_MPU_RASR_SIZE(8) | SCB_MPU_RASR_ENABLE)
#define SIZE_1K		(SCB_MPU_RASR_SIZE(9) | SCB_MPU_RASR_ENABLE)
#define SIZE_2K		(SCB_MPU_RASR_SIZE(10) | SCB_MPU_RASR_ENABLE)
#define SIZE_4K		(SCB_MPU_RASR_SIZE(11) | SCB_MPU_RASR_ENABLE)
#define SIZE_8K		(SCB_MPU_RASR_SIZE(12) | SCB_MPU_RASR_ENABLE)
#define SIZE_16K	(SCB_MPU_RASR_SIZE(13) | SCB_MPU_RASR_ENABLE)
#define SIZE_32K	(SCB_MPU_RASR_SIZE(14) | SCB_MPU_RASR_ENABLE)
#define SIZE_64K	(SCB_MPU_RASR_SIZE(15) | SCB_MPU_RASR_ENABLE)
#define SIZE_128K	(SCB_MPU_RASR_SIZE(16) | SCB_MPU_RASR_ENABLE)
#define SIZE_256K	(SCB_MPU_RASR_SIZE(17) | SCB_MPU_RASR_ENABLE)
#define SIZE_512K	(SCB_MPU_RASR_SIZE(18) | SCB_MPU_RASR_ENABLE)
#define SIZE_1M		(SCB_MPU_RASR_SIZE(19) | SCB_MPU_RASR_ENABLE)
#define SIZE_2M		(SCB_MPU_RASR_SIZE(20) | SCB_MPU_RASR_ENABLE)
#define SIZE_4M		(SCB_MPU_RASR_SIZE(21) | SCB_MPU_RASR_ENABLE)
#define SIZE_8M		(SCB_MPU_RASR_SIZE(22) | SCB_MPU_RASR_ENABLE)
#define SIZE_16M	(SCB_MPU_RASR_SIZE(23) | SCB_MPU_RASR_ENABLE)
#define SIZE_32M	(SCB_MPU_RASR_SIZE(24) | SCB_MPU_RASR_ENABLE)
#define SIZE_64M	(SCB_MPU_RASR_SIZE(25) | SCB_MPU_RASR_ENABLE)
#define SIZE_128M	(SCB_MPU_RASR_SIZE(26) | SCB_MPU_RASR_ENABLE)
#define SIZE_256M	(SCB_MPU_RASR_SIZE(27) | SCB_MPU_RASR_ENABLE)
#define SIZE_512M	(SCB_MPU_RASR_SIZE(28) | SCB_MPU_RASR_ENABLE)
#define SIZE_1G		(SCB_MPU_RASR_SIZE(29) | SCB_MPU_RASR_ENABLE)
#define SIZE_2G		(SCB_MPU_RASR_SIZE(30) | SCB_MPU_RASR_ENABLE)
#define SIZE_4G		(SCB_MPU_RASR_SIZE(31) | SCB_MPU_RASR_ENABLE)
#define REGION(n)	(SCB_MPU_RBAR_REGION(n) | SCB_MPU_RBAR_VALID)

void MIMXRT1062_MPU_init(void)
{
  MPU->CTRL = 0; // turn off MPU

  uint32_t i = 0;
  MPU->RBAR = 0x00000000 | REGION(i++); //https://developer.arm.com/docs/146793866/10/why-does-the-cortex-m7-initiate-axim-read-accesses-to-memory-addresses-that-do-not-fall-under-a-defined-mpu-region
  MPU->RASR = SCB_MPU_RASR_TEX(0) | NOACCESS | NOEXEC | SIZE_4G;
	
  /* MPU->RBAR = 0x00000000 | REGION(i++); // ITCM */
  /* MPU->RASR = MEM_NOCACHE | READWRITE | SIZE_512K; */

  // TODO: trap regions should be created last, because the hardware gives
  //  priority to the higher number ones.
  MPU->RBAR = 0x00000000 | REGION(i++); // trap NULL pointer deref
  MPU->RASR =  DEV_NOCACHE | NOACCESS | SIZE_32B;

  MPU->RBAR = 0x00200000 | REGION(i++); // Boot ROM
  MPU->RASR = MEM_CACHE_WT | READONLY | SIZE_128K;

  MPU->RBAR = 0x20000000 | REGION(i++); // DTCM
  MPU->RASR = MEM_NOCACHE | READWRITE | NOEXEC | SIZE_512K;

  // teensy4/startup.c sets up a region on the stack to detect stack overflow.
  // ChibiOS, at the time of writing, does not.
  //	MPU->RBAR = ((uint32_t)&_ebss) | REGION(i++); // trap stack overflow
  //	MPU->RASR = SCB_MPU_RASR_TEX(0) | NOACCESS | NOEXEC | SIZE_32B;

  MPU->RBAR = 0x20200000 | REGION(i++); // RAM (AXI bus)
  MPU->RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_1M;

  MPU->RBAR = 0x40000000 | REGION(i++); // Peripherals
  MPU->RASR = DEV_NOCACHE | READWRITE | NOEXEC | SIZE_64M;

  MPU->RBAR = 0x60000000 | REGION(i++); // QSPI Flash
  MPU->RASR = MEM_CACHE_WBWA | READONLY | SIZE_16M;

  MPU->RBAR = 0x70000000 | REGION(i++); // FlexSPI2
  MPU->RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_16M;

  // TODO: protect access to power supply config

  MPU->CTRL = SCB_MPU_CTRL_ENABLE;
}

/** @} */
