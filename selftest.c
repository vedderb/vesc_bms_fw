/*
	Copyright 2021 Benjamin Vedder	benjamin@vedder.se

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

#include "selftest.h"
#include "terminal.h"
#include "commands.h"
#include "ltc6813.h"
#include "bms_if.h"
#include "main.h"
#include "ch.h"
#include "hal.h"

// Private functions
static void terminal_st(int argc, const char **argv);

void selftest_init(void) {
	terminal_register_command_callback(
			"hw_st",
			"Run hardware self test",
			0,
			terminal_st);
}

static void terminal_st(int argc, const char **argv) {
	(void)argc; (void)argv;

	bool res_ok = ltc_self_test();

	commands_printf("Testing LTC6813... %s\n", res_ok ? "Ok" : "Failed");

	for (int i = 0;i < backup.config.cell_num;i++) {
		bms_if_set_balance_override(i, 1);
	}

	chThdSleepMilliseconds(500);

	commands_printf("Testing balancing and open connections...\n");
	commands_printf("Cell NoBal   Bal     Diff    Result");
	commands_printf("===================================");

	for (int i = backup.config.cell_first_index;
			i < (backup.config.cell_first_index + backup.config.cell_num);i++) {
		bms_if_set_balance_override(i, 2);
		ltc_set_dsc(i, 1);
		chThdSleepMilliseconds(200);

		float no_bal = ltc_last_cell_voltage(i);
		float bal = ltc_last_cell_voltage_no_mute(i);
		float diff = fabsf(bal - no_bal);
		bool ok = (diff / no_bal > 0.03 && bal > 2.0);

#ifdef HW_NO_CH0_TEST
		if (i == 0) {
			ok = bal > 2.0;
		}
#endif

		commands_printf("%02d   %.4f  %.4f  %.4f  %s",
				i + 1,
				no_bal,
				bal,
				diff,
				ok ? "Ok" : "Failed");

		if (!ok) {
			res_ok = false;
		}

		bms_if_set_balance_override(i, 0);
		ltc_set_dsc(i, 0);
	}

	commands_printf(res_ok ? "\nAll tests passed!\n" : "\nOne or more tests failed...\n");
}
