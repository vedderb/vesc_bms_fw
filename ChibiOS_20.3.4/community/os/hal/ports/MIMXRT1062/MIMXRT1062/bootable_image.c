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
 * @file    MIMXRT1062/bootable_image.c
 * @brief   Additional information for the NXP BootROM.
 *          With these extra sections, we define a bootable image,
 *          allowing the BootROM to start our code from flash.
 * @note    https://www.nxp.com/docs/en/nxp/application-notes/AN12238.pdf
 * @note    https://www.nxp.com/docs/en/application-note/AN12107.pdf
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

#include "printf_debug.h"

#include "clock_config.h"

#define NVIC_NUM_INTERRUPTS 160
extern uint32_t _vectors[NVIC_NUM_INTERRUPTS];

extern void Reset_Handler(void);

// trampoline_reset_handler initializes FlexRAM, then jumps to the ChibiOS
// Reset_Handler. This is required because the ChibiOS crt0 does not provide a
// hook in a suitable place to integrate FlexRAM configuration.
//
// Note that when loading an image into the debugger, the ELF entry point
// (specified by ENTRY(Reset_Handler) in rules_code.ld) is used directly, and
// our trampoline_reset_handler that we configure in the Image Vector Table
// (IVT) is not called. Instead, the debugger Connect Script
// (e.g. rt1060_connect.scp) is responsible for setting up FlexRAM.  Instead of
// modifying the Connect Script accordingly, it might be easier to change
// ENTRY(Reset_Handler) to ENTRY(trampoline_reset_handler) for debugging.
__attribute__((target("thumb"), aligned(2)))
void trampoline_reset_handler(void) {
  __disable_irq();

  // Switch to final VTOR as quickly as possible to have fault handlers set up
  // (e.g. HardFault) for easier debugging. When encountering a fault without a
  // usable VTOR table, the MCU will raise another fault and end up in lockup.
  SCB->VTOR = (uint32_t)&_vectors;

  IOMUXC_GPR->GPR17 = 0xaaaaaaaa;
  __DSB();
  __ISB();
  IOMUXC_GPR->GPR16 &= ~IOMUXC_GPR_GPR16_INIT_ITCM_EN_MASK;
  IOMUXC_GPR->GPR16 |=
    IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL(1) |
    IOMUXC_GPR_GPR16_INIT_DTCM_EN(1) |
    IOMUXC_GPR_GPR16_INIT_ITCM_EN(0);

  __DSB();
  __ISB();

  uint32_t current_gpr14 = IOMUXC_GPR->GPR14;
  current_gpr14 &= ~IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ_MASK;
  current_gpr14 |= IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ(10);
  current_gpr14 &= ~IOMUXC_GPR_GPR14_CM7_CFGITCMSZ_MASK;
  current_gpr14 |= IOMUXC_GPR_GPR14_CM7_CFGITCMSZ(0);
  IOMUXC_GPR->GPR14 = current_gpr14;

  __DSB();
  __ISB();

  Reset_Handler();
}

// IMXRT1060RM: 9.5.5 Exception handling
// A minimal vector table with only the first 2 elements, i.e. initial SP (stack
// pointer) and PC (program count) values.
__attribute__ ((section(".vectors"), used, aligned(1024)))
const uint32_t vector_table[] = {
  // Initial SP (stack pointer) value when booting.
  // Will be updated to point to DTCM FlexRAM by ChibiOS crt0_v7m.S.
  0x20201000, // OCRAM, always available regardless of FlexRAM setting.

  // Initial PC (program count) value when booting.
  (uint32_t)&trampoline_reset_handler, // jumps to Reset_Handler
};

/* See section 2.5.2 in https://www.nxp.com/docs/en/application-note/AN12107.pdf */
__attribute__ ((section(".bootdata"), used))
const uint32_t BootData[3] = {
  // destination address is equal to the external flash address, so the i.MX
  // BootROM will skip any remaining memory copies and start up the application
  // binary directly in the flash address space.
  // See section 3.2.2 in https://www.nxp.com/docs/en/nxp/application-notes/AN12238.pdf
  0x60000000,                // absolute address of the bootable image
  // The following size normally determines how many bytes are copied from
  // flash to RAM, but because the destination address is equal to the
  // flash address, no copying is taking place.
  //
  // So, logically, we would set this field to 0, but 32 is the minimum
  // size that works in practice. My guess is that 32 is the size of the
  // IVT, and perhaps the BootROM code needs the IVT to be present and
  // accounted for. The NXP examples just set this to the size of the
  // flash, so we do the same:
  1984*1024,
  0,                         // plugin flag, 0 = normal boot image
};


/* See section 2.5.1 in https://www.nxp.com/docs/en/application-note/AN12107.pdf */
__attribute__ ((section(".ivt"), used))
const uint32_t ImageVectorTable[8] = {
  0x402000D1,		// header
  (uint32_t)vector_table, // docs are wrong, needs to be vec table, not start addr
  0,			// reserved

  // DCD (Device Configuration Data), e.g. for SDRAM during boot.  Note
  // that the BootROM of the i.MX RT 1060 does not actually support DCD,
  // so this field must always be set to 0:
  //
  // IMXRT1060RM: NOTE: The DCD is not supported in the BootROM, in this
  // device. It must be set to 0x00.
  0,

  (uint32_t)BootData,	// abs address of boot data
  (uint32_t)ImageVectorTable, // self
  0,			// command sequence file
  0			// reserved
};

__attribute__ ((section(".flashconfig"), used))
uint32_t FlexSPI_NOR_Config[128] = {
  // 448 byte common FlexSPI configuration block, 8.6.3.1 page 223 (RT1060 rev 0)
  // MCU_Flashloader_Reference_Manual.pdf, 8.2.1, Table 8-2, page 72-75
  0x42464346,		// Tag				0x00
  0x56010000,		// Version
  0,			// reserved
  0x00030301,		// columnAdressWidth,dataSetupTime,dataHoldTime,readSampleClkSrc

  0x00000000,		// waitTimeCfgCommands,-,deviceModeCfgEnable
  0,			// deviceModeSeq
  0,			// deviceModeArg
  0x00000000,		// -,-,-,configCmdEnable

  0,			// configCmdSeqs		0x20
  0,
  0,
  0,

  0,			// cfgCmdArgs			0x30
  0,
  0,
  0,

  0x00000000,		// controllerMiscOption		0x40
  // The Teensy 4 config used to run the FlexSPI Serial Clock at 60 MHz:
  // https://github.com/PaulStoffregen/cores/commit/c346fc36ed97dcaed2fa1d70626fbd80cf35586d
  // whereas NXP is running it with 100 MHz. With the old default of 60
  // MHz, I occasionally get hard faults when reading data from flash.
  0x00080401,		// lutCustomSeqEnable,serialClkFreq,sflashPadType,deviceType
  0,			// reserved
  0,			// reserved
  // TODO:
#define ARDUINO_TEENSY41 1

#if defined(ARDUINO_TEENSY40)
  0x00200000,		// sflashA1Size			0x50
#elif defined(ARDUINO_TEENSY41)
  0x00800000,		// sflashA1Size			0x50
#else
#error "Unknow flash chip size";
#endif
  0,			// sflashA2Size
  0,			// sflashB1Size
  0,			// sflashB2Size

  0,			// csPadSettingOverride		0x60
  0,			// sclkPadSettingOverride
  0,			// dataPadSettingOverride
  0,			// dqsPadSettingOverride

  0,			// timeoutInMs			0x70
  0,			// commandInterval
  0,			// dataValidTime
  0x00000000,		// busyBitPolarity,busyOffset

  0x0A1804EB,		// lookupTable[0]		0x80
  0x26043206,		// lookupTable[1]
  0,			// lookupTable[2]
  0,			// lookupTable[3]

  0x24040405,		// lookupTable[4]		0x90
  0,			// lookupTable[5]
  0,			// lookupTable[6]
  0,			// lookupTable[7]

  0,			// lookupTable[8]		0xA0
  0,			// lookupTable[9]
  0,			// lookupTable[10]
  0,			// lookupTable[11]

  0x00000406,		// lookupTable[12]		0xB0
  0,			// lookupTable[13]
  0,			// lookupTable[14]
  0,			// lookupTable[15]

  0,			// lookupTable[16]		0xC0
  0,			// lookupTable[17]
  0,			// lookupTable[18]
  0,			// lookupTable[19]

  0x08180420,		// lookupTable[20]		0xD0
  0,			// lookupTable[21]
  0,			// lookupTable[22]
  0,			// lookupTable[23]

  0,			// lookupTable[24]		0xE0
  0,			// lookupTable[25]
  0,			// lookupTable[26]
  0,			// lookupTable[27]

  0,			// lookupTable[28]		0xF0
  0,			// lookupTable[29]
  0,			// lookupTable[30]
  0,			// lookupTable[31]

  0x081804D8,		// lookupTable[32]		0x100
  0,			// lookupTable[33]
  0,			// lookupTable[34]
  0,			// lookupTable[35]

  0x08180402,		// lookupTable[36]		0x110
  0x00002004,		// lookupTable[37]
  0,			// lookupTable[38]
  0,			// lookupTable[39]

  0,			// lookupTable[40]		0x120
  0,			// lookupTable[41]
  0,			// lookupTable[42]
  0,			// lookupTable[43]

  0x00000460,		// lookupTable[44]		0x130
  0,			// lookupTable[45]
  0,			// lookupTable[46]
  0,			// lookupTable[47]

  0,			// lookupTable[48]		0x140
  0,			// lookupTable[49]
  0,			// lookupTable[50]
  0,			// lookupTable[51]

  0,			// lookupTable[52]		0x150
  0,			// lookupTable[53]
  0,			// lookupTable[54]
  0,			// lookupTable[55]

  0,			// lookupTable[56]		0x160
  0,			// lookupTable[57]
  0,			// lookupTable[58]
  0,			// lookupTable[59]

  0,			// lookupTable[60]		0x170
  0,			// lookupTable[61]
  0,			// lookupTable[62]
  0,			// lookupTable[63]

  0,			// LUT 0: Read			0x180
  0,			// LUT 1: ReadStatus
  0,			// LUT 3: WriteEnable
  0,			// LUT 5: EraseSector

  0,			// LUT 9: PageProgram		0x190
  0,			// LUT 11: ChipErase
  0,			// LUT 15: Dummy
  0,			// LUT unused?

  0,			// LUT unused?			0x1A0
  0,			// LUT unused?
  0,			// LUT unused?
  0,			// LUT unused?

  0,			// reserved			0x1B0
  0,			// reserved
  0,			// reserved
  0,			// reserved

  // 64 byte Serial NOR configuration block, 8.6.3.2, page 346

  256,			// pageSize			0x1C0
  4096,			// sectorSize
  1,			// ipCmdSerialClkFreq
  0,			// reserved

  0x00010000,		// block size			0x1D0
  0,			// reserved
  0,			// reserved
  0,			// reserved

  0,			// reserved			0x1E0
  0,			// reserved
  0,			// reserved
  0,			// reserved

  0,			// reserved			0x1F0
  0,			// reserved
  0,			// reserved
  0			// reserved
};
