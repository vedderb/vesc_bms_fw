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
#include "buffer.h"

typedef struct {
	float no_bal;
	float bal;
} cell_test_voltage_t;

// Private functions
static void terminal_st(int argc, const char **argv);

void selftest_init(void) {
	terminal_register_command_callback(
			"hw_st",
			"Run hardware self test",
			0,
			terminal_st);
}

static int32_t balancing_selftest(cell_test_voltage_t *p_test_v) {
	if ((backup.config.cell_first_index + backup.config.cell_num) >= LTC_MAX_NBR_CELLS)
		return -1;

	for (int i = backup.config.cell_first_index;
			i < (backup.config.cell_first_index + backup.config.cell_num);i++) {
		bms_if_set_balance_override(i, 1);
	}

	chThdSleepMilliseconds(500);

	for (int i = backup.config.cell_first_index;
			i < (backup.config.cell_first_index + backup.config.cell_num);i++) {
		bms_if_set_balance_override(i, 2);
		ltc_set_dsc(i, 1);
		chThdSleepMilliseconds(200);

		p_test_v[i].no_bal = ltc_last_cell_voltage(i);
		p_test_v[i].bal = ltc_last_cell_voltage_no_mute(i);
	}

	for (int i = backup.config.cell_first_index;
			i < (backup.config.cell_first_index + backup.config.cell_num);i++) {
		bms_if_set_balance_override(i, 0);
		ltc_set_dsc(i, 0);
	}

	return 0;
}

static void terminal_st(int argc, const char **argv) {
	(void)argc; (void)argv;

	cell_test_voltage_t cell_v[LTC_MAX_NBR_CELLS];
	bool res_ok = ltc_self_test();

	commands_printf("Testing LTC6813... %s\n", res_ok ? "Ok" : "Failed");

	balancing_selftest(cell_v);

	commands_printf("Testing balancing and open connections...\n");
	commands_printf("Cell NoBal   Bal     Diff    Result");
	commands_printf("===================================");

	for (int i = backup.config.cell_first_index;
			i < (backup.config.cell_first_index + backup.config.cell_num);i++) {

		float diff = fabsf(cell_v[i].bal - cell_v[i].no_bal);
		bool ok = (diff / cell_v[i].no_bal > 0.03 && cell_v[i].bal > 2.0);

#ifdef HW_NO_CH0_TEST
		if (i == 0) {
			ok = cell_v[i].bal > 2.0;
		}
#endif

		commands_printf("%02d   %.4f  %.4f  %.4f  %s",
				i + 1,
				cell_v[i].no_bal,
				cell_v[i].bal,
				diff,
				ok ? "Ok" : "Failed");

		if (!ok) {
			res_ok = false;
		}
	}

	commands_printf(res_ok ? "\nAll tests passed!\n" : "\nOne or more tests failed...\n");
}

int32_t selftest_serialize_result(uint8_t *buffer, uint32_t buffer_size) {
	cell_test_voltage_t cell_v[LTC_MAX_NBR_CELLS];
	int32_t len;
	bool res_ok;

	if (buffer_size < (sizeof(uint16_t) +
			((backup.config.cell_num - backup.config.cell_first_index) * sizeof(float))))
		return 0;

	len = 0;
	res_ok = ltc_self_test();
	buffer_append_int16(buffer, res_ok, &len);

	if (balancing_selftest(cell_v) < 0)
		return len;

	for (int i = backup.config.cell_first_index;
			i < (backup.config.cell_first_index + backup.config.cell_num);i++) {
		buffer_append_float32_auto(buffer, cell_v[i].no_bal, &len);
		buffer_append_float32_auto(buffer, cell_v[i].bal, &len);
	}

	return len;
}
