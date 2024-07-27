#ifndef FAULT_HANDLERS_H_
#define FAULT_HANDLERS_H_

#include <ch.h>
#include "port_fault_handlers.h"

/*
 * Notes:
 *
 * 1) #define FAULT_NO_PRINT to remove chprintf, etc
 * 2) #define FAULT_INFO_HOOK(fault_info) to receive a struct fault_info when
 *    a fault is produced.
 */

struct fault_info {
	struct decoded_fault_registers decoded_fault_registers;
#ifndef FAULT_NO_PRINT
	char decoded_info_string[300];
#endif
};

#endif /* FAULT_HANDLERS_H_ */
