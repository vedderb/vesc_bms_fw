/*
	Copyright 2019 - 2020 Benjamin Vedder	benjamin@vedder.se

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

#include "hw.h"
#include "ch.h"
#include "hal.h"
#include "terminal.h"
#include "commands.h"
#include "pwr.h"

#include <stdio.h>

// Types
typedef enum {
	CONN_STATE_UNPLUGGED = 0,
	CONN_STATE_JETPACK,
	CONN_STATE_CHARGER
} CONN_STATE;

// Functions
static void terminal_rl_main_set(int argc, const char **argv);
static void terminal_rl_pch_set(int argc, const char **argv);
static void terminal_info(int argc, const char **argv);

// Threads
static THD_WORKING_AREA(hw_thd_wa, 2048);
static THD_FUNCTION(hw_thd, p);

// Variables
static volatile CONN_STATE m_conn_state = CONN_STATE_UNPLUGGED;

void hw_board_init(void) {
	palSetLineMode(LINE_CURR_MEASURE_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_5V_HP_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_3V3P_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_12V_HP_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_AFTER_FUSE_EN, PAL_MODE_OUTPUT_PUSHPULL);

	palSetLine(LINE_5V_HP_EN);
	palSetLine(LINE_3V3P_EN);
	palSetLine(LINE_12V_HP_EN);
	palSetLine(LINE_LED_BLUE);
	palSetLine(LINE_AFTER_FUSE_EN);

	palSetLineMode(LINE_PWRKEY_1, PAL_MODE_INPUT);
	palSetLineMode(LINE_PWRKEY_2, PAL_MODE_INPUT);

	terminal_register_command_callback(
			"main_rl_set",
			"Set main relay",
			"[0 or 1]",
			terminal_rl_main_set);

	terminal_register_command_callback(
			"pch_rl_set",
			"Set precharge relay",
			"[0 or 1]",
			terminal_rl_pch_set);

	terminal_register_command_callback(
			"rd_info",
			"Print HW info",
			0,
			terminal_info);

	chThdCreateStatic(hw_thd_wa, sizeof(hw_thd_wa), NORMALPRIO, hw_thd, 0);
}

void hw_board_chg_en(bool enable) {
	if (m_conn_state == CONN_STATE_CHARGER) {
		if (enable) {
			HW_RELAY_MAIN_ON();
		} else {
			HW_RELAY_MAIN_OFF();
		}
	}
}

static THD_FUNCTION(hw_thd, p) {
	(void)p;
	chRegSetThreadName("HW");

	for(;;) {
		bool pwr_key_1 = !palReadLine(LINE_PWRKEY_1);
		bool pwr_key_2 = !palReadLine(LINE_PWRKEY_2);

		if (!pwr_key_1 && !pwr_key_2) {
			m_conn_state = CONN_STATE_UNPLUGGED;
			HW_RELAY_MAIN_OFF();
			HW_RELAY_PCH_OFF();
		} else if (!pwr_key_1 && pwr_key_2) {
			m_conn_state = CONN_STATE_JETPACK;
		} else if (pwr_key_1 && pwr_key_2) {
			m_conn_state = CONN_STATE_CHARGER;
		}

		chThdSleepMilliseconds(10);
	}
}

static void terminal_rl_main_set(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		if (d) {
			HW_RELAY_MAIN_ON();
		} else {
			HW_RELAY_MAIN_OFF();
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

static void terminal_rl_pch_set(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		if (d) {
			HW_RELAY_PCH_ON();
		} else {
			HW_RELAY_PCH_OFF();
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

static void terminal_info(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("V Fuse: %.2f V", pwr_get_vfuse());

	switch (m_conn_state) {
	case CONN_STATE_UNPLUGGED:
		commands_printf("CONN_STATE_UNPLUGGED\n");
		break;
	case CONN_STATE_JETPACK:
		commands_printf("CONN_STATE_JETPACK\n");
		break;
	case CONN_STATE_CHARGER:
		commands_printf("CONN_STATE_CHARGER\n");
		break;
	}
}
