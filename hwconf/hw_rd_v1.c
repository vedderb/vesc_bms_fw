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
#include "bms_if.h"

#include <stdio.h>

// Types
typedef enum {
	CONN_STATE_UNPLUGGED = 0,
	CONN_STATE_JETPACK,
	CONN_STATE_CHARGER
} CONN_STATE;

// Functions
static void terminal_psw_set(int argc, const char **argv);
static void terminal_info(int argc, const char **argv);

// Threads
static THD_WORKING_AREA(hw_thd_wa, 2048);
static THD_FUNCTION(hw_thd, p);

// Variables
static volatile CONN_STATE m_conn_state = CONN_STATE_UNPLUGGED;
static mutex_t m_sw_mutex;
static volatile bool terminal_override_psw = false;

void hw_board_init(void) {
	chMtxObjectInit(&m_sw_mutex);

	HW_RELAY_MAIN_OFF();
	HW_RELAY_PCH_OFF();

	palSetLineMode(LINE_CURR_MEASURE_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_5V_HP_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_3V3P_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_12V_HP_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_AFTER_FUSE_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_RELAY_PCH, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_RELAY_MAIN, PAL_MODE_OUTPUT_PUSHPULL);

	palSetLine(LINE_5V_HP_EN);
	palSetLine(LINE_3V3P_EN);
	palSetLine(LINE_12V_HP_EN);
	palSetLine(LINE_LED_BLUE);
	palSetLine(LINE_AFTER_FUSE_EN);

	palSetLineMode(LINE_PWRKEY_1, PAL_MODE_INPUT);
	palSetLineMode(LINE_PWRKEY_2, PAL_MODE_INPUT);

	terminal_register_command_callback(
			"rd_psw_set",
			"Switch output on or off",
			"[0 or 1]",
			terminal_psw_set);

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

bool hw_psw_switch_on(void) {
	chMtxLock(&m_sw_mutex);

	if (HW_RALAY_MAIN_IS_ON()) {
		chMtxUnlock(&m_sw_mutex);
		return true;
	}

	HW_RELAY_PCH_ON();

	float timeout = 0;
	// Wait for output voltage to rise to 90 % of battery voltage
	while (fabsf(pwr_get_vcharge() - bms_if_get_v_tot()) > (bms_if_get_v_tot() / 10)) {
		chThdSleepMilliseconds(25);
		timeout += 25.0 / 1000.0;
		if (timeout >= 2.5) {
			// Timed out
			HW_RELAY_PCH_OFF();
			chMtxUnlock(&m_sw_mutex);
			return false;
		}
	}

	HW_RELAY_MAIN_ON();
	HW_RELAY_PCH_OFF();

	chMtxUnlock(&m_sw_mutex);
	return true;
}

void hw_psw_switch_off(void) {
	chMtxLock(&m_sw_mutex);

	if (!HW_RALAY_MAIN_IS_ON()) {
		chMtxUnlock(&m_sw_mutex);
		return;
	}

	HW_RELAY_MAIN_OFF();
	chMtxUnlock(&m_sw_mutex);
}

static THD_FUNCTION(hw_thd, p) {
	(void)p;
	chRegSetThreadName("HW");

	int jetpack_delay_cnt = 0;
	bool psw_ok = true;

	for(;;) {
		bool pwr_key_1 = !palReadLine(LINE_PWRKEY_1);
		bool pwr_key_2 = !palReadLine(LINE_PWRKEY_2);

		if (!pwr_key_1 && !pwr_key_2) {
			m_conn_state = CONN_STATE_UNPLUGGED;
			if (!terminal_override_psw) {
				HW_RELAY_MAIN_OFF();
				HW_RELAY_PCH_OFF();
			}
		} else if (!pwr_key_1 && pwr_key_2) {
			m_conn_state = CONN_STATE_JETPACK;
		} else if (pwr_key_1 && pwr_key_2) {
			m_conn_state = CONN_STATE_CHARGER;
		}

		/*
		 * If the jetpack is detected, wait 100 iterations (about 1s) then start the precharge and
		 * main contactor switching sequence. If switching on fails (which can happen if the
		 * precharge times out due to e.g. a short) the jetpack must be unplugged and replugged
		 * to make the next attempt.
		 */
		if (m_conn_state == CONN_STATE_JETPACK) {
			if (jetpack_delay_cnt < 100) {
				jetpack_delay_cnt++;
			} else {
				if (psw_ok) {
					psw_ok = hw_psw_switch_on();
				}
			}
		} else {
			jetpack_delay_cnt = 0;
			psw_ok = true;
		}

		chThdSleepMilliseconds(10);
	}
}

static void terminal_psw_set(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		if (d) {
			terminal_override_psw = true;
			commands_printf("Switching on...");
			bool res = hw_psw_switch_on();
			if (res) {
				commands_printf("Power is now on");
			} else {
				commands_printf("Precharge timed out");
			}
		} else {
			hw_psw_switch_off();
			commands_printf("Power is now off");
			terminal_override_psw = false;
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
