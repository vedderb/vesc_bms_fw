/* Copyright (C) 2018  Adam Green (https://github.com/adamgreen)
				 2019 Diego Ismirlian (dismirlian(at)google's mail)

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

/*
	This code is heavily based on the work of Adam Green (https://github.com/adamgreen)
*/


#include <ch.h>
#include <hal.h>

// Assembly language routines defined in tests.S
void testMspMultipleOf8(void);
void testMspNotMultipleOf8(void);
void testPspMultipleOf8(void);
void testInitFPURegisters(void);
void testBreakpoints(void);

static void enable8ByteStackAlignment(void);
static void crashWithFPUDisabled(void);
static void crashWithFPUAutoStackingDisabled(void);
static void crashWithFPUAutoStackEnabled(void);
static void crashWithFPULazyAutoStacking(void);

#define CRASH_CATCHER_READ_FAULT()          (*(volatile unsigned int*)0x55555550)
#define CRASH_CATCHER_WRITE_FAULT()         (*(volatile unsigned int*)0x55555550 = 0x0)
#define CRASH_CATCHER_INVALID_INSTRUCTION() { __asm volatile (".word 0xDE00"); }

/* Macro used to insert hardcoded breakpoint into user's code. */
#define CRASH_CATCHER_BREAKPOINT()          { __asm volatile ("bkpt #0"); }

int crash(int option) {

	enable8ByteStackAlignment();

	switch (option) {
	case 1:
		testMspMultipleOf8();
		break;
	case 2:
		testMspNotMultipleOf8();
		break;
	case 3:
		testPspMultipleOf8();
		break;
	case 4:
		CRASH_CATCHER_READ_FAULT();
		break;
	case 5:
		CRASH_CATCHER_WRITE_FAULT();
		break;
	case 6:
		crashWithFPUDisabled();
		break;
	case 7:
		crashWithFPUAutoStackingDisabled();
		break;
	case 8:
		crashWithFPUAutoStackEnabled();
		break;
	case 9:
		crashWithFPULazyAutoStacking();
		break;
	case 10:
		testBreakpoints();
		break;
	default:
		break;
	}

	return 0;
}

static void enable8ByteStackAlignment()
{
    SCB->CCR |= SCB_CCR_STKALIGN_Msk;
}

#if CORTEX_HAS_FPU
static void disableFPU(void)
{
    static const uint32_t FPCA = 1 << 2;
    SCB->CPACR &= ~(0xF << 20);
    __set_CONTROL(__get_CONTROL() & ~FPCA);
}

static void enableFPU(void)
{
    SCB->CPACR |= (0xF << 20);
}

static void crashWithFPUDisabled(void)
{
    disableFPU();
    __disable_irq();
    testInitFPURegisters();
    CRASH_CATCHER_READ_FAULT();
}

static const uint32_t ASPEN = 1 << 31;
static const uint32_t LSPEN = 1 << 30;

static void crashWithFPUAutoStackingDisabled(void)
{
    disableFPU();
    FPU->FPCCR &= ~(ASPEN | LSPEN);
    enableFPU();
    __disable_irq();
    testInitFPURegisters();
    CRASH_CATCHER_READ_FAULT();
}

static void crashWithFPUAutoStackEnabled(void)
{
    disableFPU();
        FPU->FPCCR |= ASPEN;
        FPU->FPCCR &= ~LSPEN;
    enableFPU();
    __disable_irq();
    testInitFPURegisters();
    CRASH_CATCHER_READ_FAULT();
}

static void crashWithFPULazyAutoStacking(void)
{
    disableFPU();
    FPU->FPCCR |= (ASPEN | LSPEN);
    enableFPU();
    __disable_irq();
    testInitFPURegisters();
    CRASH_CATCHER_READ_FAULT();
}

#else

static void crashWithFPUDisabled(void)
{
    return;
}

static void crashWithFPUAutoStackingDisabled(void)
{
    return;
}

static void crashWithFPUAutoStackEnabled(void)
{
    return;
}

static void crashWithFPULazyAutoStacking(void)
{
    return;
}

#endif // !defined(CORTEX_M4)
