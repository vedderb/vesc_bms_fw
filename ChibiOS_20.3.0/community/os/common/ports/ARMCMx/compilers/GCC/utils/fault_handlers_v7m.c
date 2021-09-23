/*
    ChibiOS - Copyright (C) 2019 Diego Ismirlian (dismirlian(at)google's mail)

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

#include "fault_handlers.h"
#include <hal.h>
#include <string.h>

#ifndef FAULT_NO_PRINT
#include <chprintf.h>
#include <memstreams.h>

#define fault_printf(f, ...) 										\
	chprintf((BaseSequentialStream *)(&ms), 	\
			f "\n", ##__VA_ARGS__)

static MemoryStream ms;
#else
#define fault_printf(f, ...) do {} while(0)
#endif

static struct fault_info fault_info;

static void _mem_fault(void) {
	fault_printf("== Mem faults follow ==");

	if (SCB->CFSR & SCB_CFSR_MSTKERR_Msk) {
		fault_printf("Stacking error");
		fault_info.decoded_fault_registers.memfault.stacking_error = true;
	}
	if (SCB->CFSR & SCB_CFSR_MUNSTKERR_Msk) {
		fault_printf("Unstacking error");
		fault_info.decoded_fault_registers.memfault.unstacking_error = true;
	}
	if (SCB->CFSR & SCB_CFSR_DACCVIOL_Msk) {
		fault_printf("Data Access Violation");
		fault_info.decoded_fault_registers.memfault.data_access_violation = true;
	}
	if (SCB->CFSR & SCB_CFSR_MMARVALID_Msk) {
		fault_printf("Address: 0x%08x", (uint32_t)SCB->MMFAR);
		fault_info.decoded_fault_registers.memfault.data_access_violation_address = (uint32_t)SCB->MMFAR;
	} else {
		fault_printf("Address: unknown");
		fault_info.decoded_fault_registers.memfault.data_access_violation_address = 0xffffffff;
	}
	if (SCB->CFSR & SCB_CFSR_IACCVIOL_Msk) {
		fault_printf("Instruction Access Violation");
		fault_info.decoded_fault_registers.memfault.instruction_access_violation = true;
	}
}

static void _bus_fault(void) {
	fault_printf("== Bus faults follow ==");

	if (SCB->CFSR & SCB_CFSR_STKERR_Msk) {
		fault_printf("Stacking error");
		fault_info.decoded_fault_registers.busfault.stacking_error = true;
	}
	if (SCB->CFSR & SCB_CFSR_UNSTKERR_Msk) {
		fault_printf("Unstacking error");
		fault_info.decoded_fault_registers.busfault.unstacking_error = true;
	}
	if (SCB->CFSR & SCB_CFSR_PRECISERR_Msk) {
		fault_printf("Precise data bus error");
		fault_info.decoded_fault_registers.busfault.precise_data_bus_error = true;
	}
	if (SCB->CFSR & SCB_CFSR_BFARVALID_Msk) {
		fault_printf("Address: 0x%08x", (uint32_t)SCB->BFAR);
		fault_info.decoded_fault_registers.busfault.precise_data_bus_error_address = (uint32_t)SCB->BFAR;
	} else {
		fault_printf("Address: unknown");
		fault_info.decoded_fault_registers.busfault.precise_data_bus_error_address = 0xffffffff;
	}
	if (SCB->CFSR & SCB_CFSR_IMPRECISERR_Msk) {
		fault_printf("Imprecise data bus error");
		fault_info.decoded_fault_registers.busfault.imprecise_data_bus_error = true;
	}
	if (SCB->CFSR & SCB_CFSR_IBUSERR_Msk) {
		fault_printf("Instruction bus error");
		fault_info.decoded_fault_registers.busfault.instruction_bus_error = true;
	}
}

static void _usage_fault(void) {
	fault_printf("== Usage faults follow ==");

	if (SCB->CFSR & SCB_CFSR_DIVBYZERO_Msk) {
		fault_printf("Division by zero");
		fault_info.decoded_fault_registers.usagefault.division_by_zero = true;
	}
	if (SCB->CFSR & SCB_CFSR_UNALIGNED_Msk) {
		fault_printf("Unaligned memory access");
		fault_info.decoded_fault_registers.usagefault.unaligned_memory_access = true;
	}
	if (SCB->CFSR & SCB_CFSR_NOCP_Msk) {
		fault_printf("No coprocessor instructions");
		fault_info.decoded_fault_registers.usagefault.no_coprocessor_instructions = true;
	}
	if (SCB->CFSR & SCB_CFSR_INVPC_Msk) {
		fault_printf("Invalid load of PC");
		fault_info.decoded_fault_registers.usagefault.invalid_load_of_pc = true;
	}
	if (SCB->CFSR & SCB_CFSR_INVSTATE_Msk) {
		fault_printf("Invalid state");
		fault_info.decoded_fault_registers.usagefault.invalid_state = true;
	}
	if (SCB->CFSR & SCB_CFSR_UNDEFINSTR_Msk) {
		fault_printf("Undefined instruction");
		fault_info.decoded_fault_registers.usagefault.undefined_instruction = true;
	}
}

static void _init_fault_info(void) {
#ifndef FAULT_NO_PRINT
	msObjectInit(&ms,
			(uint8_t *)fault_info.decoded_info_string,
			sizeof(fault_info.decoded_info_string) - 1, 0);
#endif
}

static void _save_fault_info(void) {
	memset(&fault_info, 0, sizeof(fault_info));

	if (ch.rlist.current) {
		fault_printf("Thread: 0x%08x, %s",
				ch.rlist.current, ch.rlist.current->name);

		fault_info.decoded_fault_registers.general.current_thread_address = (uint32_t)ch.rlist.current;
		fault_info.decoded_fault_registers.general.current_thread_name = ch.rlist.current->name;
	} else {
		fault_printf("Thread: unknown");
	}

	if (SCB->HFSR & SCB_HFSR_VECTTBL_Msk) {
		fault_printf("Bus fault on vector table read");
		fault_info.decoded_fault_registers.general.bus_fault_on_ivt_read = true;
	}

	if (SCB->HFSR & SCB_HFSR_FORCED_Msk) {
		fault_info.decoded_fault_registers.general.escalation = true;
		_mem_fault();
		_bus_fault();
		_usage_fault();
	}
	if (!(SCB->HFSR &
			(SCB_HFSR_VECTTBL_Msk | SCB_HFSR_FORCED_Msk))) {
		fault_printf("No fault info");
	}
}

#if defined(FAULT_INFO_HOOK)
void FAULT_INFO_HOOK(const struct fault_info *info);
#endif

void _hardfault_info(void) {
	_init_fault_info();
	fault_printf("HardFault Handler");
	_save_fault_info();

#if defined(FAULT_INFO_HOOK)
	FAULT_INFO_HOOK(&fault_info);
#endif
}

void _hardfault_epilogue(void) __attribute__((used, naked));
void _hardfault_epilogue(void) {

	/* This is part of the HardFault handler
	 *
	 * You may inspect fault_info.decoded_fault_registers and
	 * fault_info.decoded_info_string to get a description of the fault that
	 * occurred.
	 *
	 * Also, the debugger should show an artificial call stack that led to the
	 * fault. This stack is reconstructed in assembly code, until GDB includes
	 * a way of automatically unwind an exception stack.
	 *
	 */
	__asm volatile(
			"bkpt 	#0					\n"
			"b 		_hardfault_exit		\n"
	);
}

void _unhandled_exception(void) {
	/* This is an unhandled exception
	 *
	 * Once the breakpoint is hit, the debugger should show the ISR number
	 * in the vector_number variable. Don't trust the debugger's stack unwind;
	 * the _unhandled_exception ISR is shared among all undefined vectors.
	 */

	volatile uint32_t vector_number = SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;
	(void)vector_number;

	__asm volatile("bkpt #0");

	/* we are here if there is no debugger attached */
	chSysHalt("unhandled exception");
}
