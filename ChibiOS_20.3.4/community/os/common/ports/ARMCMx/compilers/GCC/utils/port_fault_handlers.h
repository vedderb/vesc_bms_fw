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

#ifndef FAULT_HANDLERS_v7m_H_
#define FAULT_HANDLERS_v7m_H_

struct decoded_fault_registers {
	struct general {
		bool bus_fault_on_ivt_read;
		bool escalation;
		uint32_t current_thread_address;
		const char *current_thread_name;
	} general;
	struct memfault {
		bool stacking_error;
		bool unstacking_error;
		bool data_access_violation;
		uint32_t data_access_violation_address;
		bool instruction_access_violation;
	} memfault;
	struct busfault {
		bool stacking_error;
		bool unstacking_error;
		bool precise_data_bus_error;
		uint32_t precise_data_bus_error_address;
		bool imprecise_data_bus_error;
		bool instruction_bus_error;
	} busfault;
	struct usagefault {
		bool division_by_zero;
		bool unaligned_memory_access;
		bool no_coprocessor_instructions;
		bool invalid_load_of_pc;
		bool invalid_state;
		bool undefined_instruction;
	} usagefault;
};

#endif /* FAULT_HANDLERS_v7m_H_ */
