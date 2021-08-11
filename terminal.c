/*
	Copyright 2019 - 2021 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC BMS firmware.

	The VESC BMS firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC BMS firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "ch.h"
#include "hal.h"
#include "terminal.h"
#include "commands.h"
#include "hw.h"
#include "comm_can.h"
#include "utils.h"
#include "comm_usb.h"
#include "mempools.h"
#include "bms_if.h"
#include "main.h"
#include "sleep.h"
#include "flash_helper.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

// Settings
#define FAULT_VEC_LEN						25
#define CALLBACK_LEN						40

// Private types
typedef struct _terminal_callback_struct {
	const char *command;
	const char *help;
	const char *arg_names;
	void(*cbf)(int argc, const char **argv);
} terminal_callback_struct;

// Private variables
static volatile fault_data fault_vec[FAULT_VEC_LEN];
static volatile int fault_vec_write = 0;
static terminal_callback_struct callbacks[CALLBACK_LEN];
static int callback_write = 0;

void terminal_process_string(char *str) {
	enum { kMaxArgs = 64 };
	int argc = 0;
	char *argv[kMaxArgs];

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}

	if (argc == 0) {
		commands_printf("No command received\n");
		return;
	}

	for (int i = 0;i < callback_write;i++) {
		if (callbacks[i].cbf != 0 && strcmp(argv[0], callbacks[i].command) == 0) {
			callbacks[i].cbf(argc, (const char**)argv);
			return;
		}
	}

	if (strcmp(argv[0], "ping") == 0) {
		commands_printf("pong\n");
	} else if (strcmp(argv[0], "mem") == 0) {
		size_t n, size, sizel;
		n = chHeapStatus(NULL, &size, &sizel);
		commands_printf("core free memory  : %u bytes", chCoreGetStatusX());
		commands_printf("heap fragments    : %u", n);
		commands_printf("heap free largest : %u bytes", sizel);
		commands_printf("heap free total   : %u bytes\n", size);
	} else if (strcmp(argv[0], "threads") == 0) {
		thread_t *tp;
		static const char *states[] = {CH_STATE_NAMES};
		commands_printf("    addr prio refs     state           name time    ");
		commands_printf("-------------------------------------------------------------------");
		tp = chRegFirstThread();
		do {
			commands_printf("%.8lx %4lu %4lu %9s %14s %lu (%.1f %%)",
					(uint32_t)tp,
					(uint32_t)tp->prio, (uint32_t)(tp->refs - 1),
					states[tp->state], tp->name, (uint32_t)tp->time,
					(double)(100.0 * (float)tp->time / (float)chVTGetSystemTimeX()));
			tp = chRegNextThread(tp);
		} while (tp != NULL);
		commands_printf(" ");
	} else if (strcmp(argv[0], "fault") == 0) {
		commands_printf("%s\n", utils_fault_to_string(bms_if_fault_now()));
	} else if (strcmp(argv[0], "faults") == 0) {
		if (fault_vec_write == 0) {
			commands_printf("No faults registered since startup\n");
		} else {
			commands_printf("The following faults were registered since start:\n");
			for (int i = 0;i < fault_vec_write;i++) {
				commands_printf("Fault            : %s", utils_fault_to_string(fault_vec[i].fault));
				commands_printf("Fault Age        : %.1f s", (double)UTILS_AGE_S(fault_vec[i].fault_time));
				commands_printf("Current          : %.1f A", (double)fault_vec[i].current);
				commands_printf("Current IC       : %.1f A", (double)fault_vec[i].current_ic);
				commands_printf("Temp Batt        : %.1f degC", (double)fault_vec[i].temp_batt);
				commands_printf("Temp IC          : %.1f degC", (double)fault_vec[i].temp_ic);
				commands_printf("Temp PCB         : %.1f degC", (double)fault_vec[i].temp_pcb);
				commands_printf("V Cell Min       : %.3f V", (double)fault_vec[i].v_cell_min);
				commands_printf("V Cell Max       : %.3f V", (double)fault_vec[i].v_cell_max);
				commands_printf(" ");
			}
		}
	} else if (strcmp(argv[0], "volt") == 0) {
		commands_printf("Input voltage: %.2f\n", (double)bms_if_get_v_tot());
	} else if (strcmp(argv[0], "can_devs") == 0) {
		commands_printf("CAN devices seen on the bus the past second:\n");
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *msg = comm_can_get_status_msg_index(i);

			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 1.0) {
				commands_printf("ID                 : %i", msg->id);
				commands_printf("RX Time            : %i", msg->rx_time);
				commands_printf("Age (milliseconds) : %.2f", (double)(UTILS_AGE_S(msg->rx_time) * 1000.0));
				commands_printf("RPM                : %.2f", (double)msg->rpm);
				commands_printf("Current            : %.2f", (double)msg->current);
				commands_printf("Duty               : %.2f\n", (double)msg->duty);
			}
		}

		commands_printf("BMS devices seen on the bus the past second:\n");
		for (int i = 0;i < CAN_BMS_STATUS_MSGS_TO_STORE;i++) {
			bms_soc_soh_temp_stat *msg = comm_can_get_bms_soc_soh_temp_stat_index(i);

			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 1.0) {
				commands_printf("ID                 : %i", msg->id);
				commands_printf("RX Time            : %i", msg->rx_time);
				commands_printf("Age (milliseconds) : %.2f", (double)(UTILS_AGE_S(msg->rx_time) * 1000.0));
				commands_printf("SOC                : %.2f", (double)msg->soc);
				commands_printf("SOH                : %.2f", (double)msg->soh);
				commands_printf("V cell min         : %.2f", (double)msg->v_cell_min);
				commands_printf("V cell max         : %.2f", (double)msg->v_cell_max);
				commands_printf("Temp max           : %.2f", (double)msg->t_cell_max);
				commands_printf("Balancing          : %d", msg->is_balancing);
				commands_printf("Charging           : %d", msg->is_charging);
				commands_printf("Charging Allowed   : %d\n", msg->is_charge_allowed);
			}
		}
	} else if (strcmp(argv[0], "hw_status") == 0) {
		commands_printf("Firmware: %d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR);
		commands_printf("Charging: %s", bms_if_is_charging() ? "true" : "false");
		commands_printf("Charge Allowed: %s", bms_if_is_charge_allowed() ? "true" : "false");
#ifdef HW_NAME
		commands_printf("Hardware: %s", HW_NAME);
#endif
		commands_printf("Flash size: %d K", *STM32_FLASH_SIZE);
		commands_printf("UUID: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
				STM32_UUID_8[0], STM32_UUID_8[1], STM32_UUID_8[2], STM32_UUID_8[3],
				STM32_UUID_8[4], STM32_UUID_8[5], STM32_UUID_8[6], STM32_UUID_8[7],
				STM32_UUID_8[8], STM32_UUID_8[9], STM32_UUID_8[10], STM32_UUID_8[11]);

		commands_printf("Mempool conf now: %d highest: %d (max %d)",
				mempools_conf_allocated_num(), mempools_conf_highest(), MEMPOOLS_CONF_NUM - 1);

		commands_printf("Configuration flash write counter: %d", backup.conf_flash_write_cnt);

		commands_printf(" ");
	} else if (strcmp(argv[0], "can_scan") == 0) {
		bool found = false;
		for (int i = 0;i < 254;i++) {
			HW_TYPE hw_type;
			if (comm_can_ping(i, &hw_type)) {
				commands_printf("Found %s with ID: %d", utils_hw_type_to_string(hw_type), i);
				found = true;
			}
		}

		if (found) {
			commands_printf("Done\n");
		} else {
			commands_printf("No CAN devices found\n");
		}
	} else if (strcmp(argv[0], "uptime") == 0) {
		commands_printf("Uptime: %.2f s", (double)chVTGetSystemTimeX() / (double)CH_CFG_ST_FREQUENCY);
		commands_printf("Sleep in: %.2f s", (double)((float)sleep_time_left() / 1000.0));
	} else if (strcmp(argv[0], "reset_cnt_chg_total") == 0) {
		backup.ah_cnt_chg_total = 0.0;
		backup.wh_cnt_chg_total = 0.0;
		flash_helper_store_backup_data();
		commands_printf("Done!\n");
	} else if (strcmp(argv[0], "reset_cnt_dis_total") == 0) {
		backup.ah_cnt_dis_total = 0.0;
		backup.wh_cnt_dis_total = 0.0;
		flash_helper_store_backup_data();
		commands_printf("Done!\n");
	} else if (strcmp(argv[0], "hum") == 0) {
		commands_printf("Hum1: %.2f (temp: %.2f)", bms_if_get_humitidy(), bms_if_get_humidity_sensor_temp());
		commands_printf("Hum2: %.2f (temp: %.2f)", bms_if_get_humitidy_2(), bms_if_get_humidity_sensor_temp_2());
		commands_printf(" ");
	}

	// The help command
	else if (strcmp(argv[0], "help") == 0) {
		commands_printf("Valid commands are:");
		commands_printf("help");
		commands_printf("  Show this help");

		commands_printf("ping");
		commands_printf("  Print pong here to see if the reply works");

		commands_printf("mem");
		commands_printf("  Show memory usage");

		commands_printf("threads");
		commands_printf("  List all threads");

		commands_printf("fault");
		commands_printf("  Prints the current fault code");

		commands_printf("faults");
		commands_printf("  Prints all stored fault codes and conditions when they arrived");

		commands_printf("volt");
		commands_printf("  Prints different voltages");

		commands_printf("can_devs");
		commands_printf("  Prints all CAN devices seen on the bus the past second");

		commands_printf("hw_status");
		commands_printf("  Print some hardware status information.");

		commands_printf("can_scan");
		commands_printf("  Scan CAN-bus using ping commands, and print all devices that are found.");

		commands_printf("uptime");
		commands_printf("  Prints how many seconds have passed since boot.");

		commands_printf("reset_cnt_chg_total");
		commands_printf("  Reset charge counters.");

		commands_printf("reset_cnt_dis_total");
		commands_printf("  Reset discharge counters.");

		commands_printf("hum");
		commands_printf("  Print humidity sensor readings.");

		for (int i = 0;i < callback_write;i++) {
			if (callbacks[i].cbf == 0) {
				continue;
			}

			if (callbacks[i].arg_names) {
				commands_printf("%s %s", callbacks[i].command, callbacks[i].arg_names);
			} else {
				commands_printf(callbacks[i].command);
			}

			if (callbacks[i].help) {
				commands_printf("  %s", callbacks[i].help);
			} else {
				commands_printf("  There is no help available for this command.");
			}
		}

		commands_printf(" ");
	} else {
		commands_printf("Invalid command: %s\n"
				"type help to list all available commands\n", argv[0]);
	}
}

void terminal_add_fault_data(fault_data *data) {
	fault_vec[fault_vec_write++] = *data;
	if (fault_vec_write >= FAULT_VEC_LEN) {
		fault_vec_write = 0;
	}
}

/**
 * Register a custom command  callback to the terminal. If the command
 * is already registered the old command callback will be replaced.
 *
 * @param command
 * The command name.
 *
 * @param help
 * A help text for the command. Can be NULL.
 *
 * @param arg_names
 * The argument names for the command, e.g. [arg_a] [arg_b]
 * Can be NULL.
 *
 * @param cbf
 * The callback function for the command.
 */
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(int argc, const char **argv)) {

	int callback_num = callback_write;

	for (int i = 0;i < callback_write;i++) {
		// First check the address in case the same callback is registered more than once.
		if (callbacks[i].command == command) {
			callback_num = i;
			break;
		}

		// Check by string comparison.
		if (strcmp(callbacks[i].command, command) == 0) {
			callback_num = i;
			break;
		}

		// Check if the callback is empty (unregistered)
		if (callbacks[i].cbf == 0) {
			callback_num = i;
			break;
		}
	}

	callbacks[callback_num].command = command;
	callbacks[callback_num].help = help;
	callbacks[callback_num].arg_names = arg_names;
	callbacks[callback_num].cbf = cbf;

	if (callback_num == callback_write) {
		callback_write++;
		if (callback_write >= CALLBACK_LEN) {
			callback_write = 0;
		}
	}
}

void terminal_unregister_callback(void(*cbf)(int argc, const char **argv)) {
	for (int i = 0;i < callback_write;i++) {
		if (callbacks[i].cbf == cbf) {
			callbacks[i].cbf = 0;
		}
	}
}

