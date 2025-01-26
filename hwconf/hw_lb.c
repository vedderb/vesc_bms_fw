/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#include "bms_if.h"
#include "pwr.h"
#include "terminal.h"
#include "commands.h"
#include "ltc6813.h"
#include "main.h"
#include "sleep.h"
#include "comm_can.h"
#include <stdio.h>
#include <stdlib.h>
#include "buzzer.c"


// Threads
static THD_WORKING_AREA(hw_thd_wa, 2048);
static THD_WORKING_AREA(hw_thd_mon_wa, 1024);
static THD_FUNCTION(hw_thd, p);
static THD_FUNCTION(hw_thd_mon, p);

// Private variables
static float m_temps[HW_TEMP_SENSORS];
static bool awake_block = false;
static float temp_v_lo[12][4] = {{-1.0}};
static float temp_v_hi[12][4] = {{-1.0}};
static bool jet_was_connected = false;

// Melodies
const char* MEL_JET_CONNECTED = "C5 E5 G5/2";
const char* MEL_JET_DISCONNECTED = "G5 E5 C5/2";
const char* MEL_ERROR = "C/1 P/1 C/1";

// Private functions
static void terminal_mc_en(int argc, const char **argv);
static void terminal_chg_en(int argc, const char **argv);
static void terminal_buzzer_test(int argc, const char **argv);
static void terminal_hw_info(int argc, const char **argv);
static void terminal_test_if_conn(int argc, const char **argv);
static void terminal_reset_pwr(int argc, const char **argv);

void hw_board_init(void) {
	palSetLineMode(LINE_CAN_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_CURR_MEASURE_EN, PAL_MODE_OUTPUT_OPENDRAIN);

	palClearLine(LINE_MC_EN);
	palClearLine(LINE_BATT_OUT_EN);
	palClearLine(LINE_12V_EN);
	palSetLine(LINE_12V_SENSE_EN);
	palClearLine(LINE_ESP_EN);

	palSetLineMode(LINE_MC_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_BATT_OUT_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_12V_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_12V_SENSE_EN, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetLineMode(LINE_ESP_EN, PAL_MODE_OUTPUT_PUSHPULL);

	palSetLineMode(LINE_SR_SER, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_SR_RCLK, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_SR_SCLK, PAL_MODE_OUTPUT_PUSHPULL);

	palClearLine(LINE_SR_SER);
	palClearLine(LINE_SR_RCLK);
	palClearLine(LINE_SR_SCLK);

	chThdCreateStatic(hw_thd_wa, sizeof(hw_thd_wa), NORMALPRIO, hw_thd, 0);
	chThdCreateStatic(hw_thd_mon_wa, sizeof(hw_thd_mon_wa), NORMALPRIO, hw_thd_mon, 0);

	terminal_register_command_callback(
			"mc_en",
			"Enable motor controller regulator",
			"[en]",
			terminal_mc_en);

	terminal_register_command_callback(
			"chg_en",
			"Enable charge input",
			"[en]",
			terminal_chg_en);

	terminal_register_command_callback(
			"buzzer_test",
			"Test the buzzer",
			NULL,
			terminal_buzzer_test);

	terminal_register_command_callback(
			"hw_info",
			"Print hw-specific info",
			NULL,
			terminal_hw_info);

	terminal_register_command_callback(
			"test_if_conn",
			"Test if interface is connected",
			NULL,
			terminal_test_if_conn);

	terminal_register_command_callback(
			"reset_pwr",
			"Reset power rails",
			NULL,
			terminal_reset_pwr);
}

// Power management
void hw_board_sleep(void) {
	palClearLine(LINE_BATT_OUT_EN);
	palClearLine(LINE_12V_EN);
	palSetLine(LINE_12V_SENSE_EN);
	palClearLine(LINE_MC_EN);
	palClearLine(LINE_ESP_EN);
}

void hw_stay_awake(void) {
	if (awake_block) {
		return;
	}
	palSetLine(LINE_MC_EN);
	palSetLine(LINE_ESP_EN);
}

void hw_test_wake_up(void) {
	if (hw_test_if_conn(false)) {
		sleep_reset();
	} else {
		if (sleep_time_left() < 300) {
			sleep_set_timer(0);
		}
	}
}

static void shift_out_data(uint16_t bits) {
	palClearLine(LINE_SR_SER);
	palClearLine(LINE_SR_RCLK);
	palClearLine(LINE_SR_SCLK);

	for (int i = 0;i < 16;i++) {
		palClearLine(LINE_SR_SCLK);
		chThdSleep(1);
		palWriteLine(LINE_SR_SER, (bits >> (15 - i)) & 1);
		chThdSleep(1);
		palSetLine(LINE_SR_SCLK);
		chThdSleep(1);
	}

	palSetLine(LINE_SR_RCLK);
	chThdSleep(1);
	palClearLine(LINE_SR_RCLK);
	chThdSleep(1);
}

typedef enum {
	lm_hi = 0,
	lm_lo,
	lm_adc
} line_mode_t;

static void set_temp_lines(line_mode_t l0, line_mode_t l1, line_mode_t l2, line_mode_t l3) {
	switch (l0) {
	case lm_hi:
		palSetLineMode(LINE_TEMP_0, PAL_MODE_OUTPUT_PUSHPULL);
		palSetLine(LINE_TEMP_0);
		break;
	case lm_lo:
		palSetLineMode(LINE_TEMP_0, PAL_MODE_OUTPUT_PUSHPULL);
		palClearLine(LINE_TEMP_0);
		break;
	case lm_adc:
		palSetLineMode(LINE_TEMP_0, PAL_MODE_INPUT_ANALOG);
		break;
	}

	switch (l1) {
	case lm_hi:
		palSetLineMode(LINE_TEMP_1, PAL_MODE_OUTPUT_PUSHPULL);
		palSetLine(LINE_TEMP_1);
		break;
	case lm_lo:
		palSetLineMode(LINE_TEMP_1, PAL_MODE_OUTPUT_PUSHPULL);
		palClearLine(LINE_TEMP_1);
		break;
	case lm_adc:
		palSetLineMode(LINE_TEMP_1, PAL_MODE_INPUT_ANALOG);
		break;
	}

	switch (l2) {
	case lm_hi:
		palSetLineMode(LINE_TEMP_2, PAL_MODE_OUTPUT_PUSHPULL);
		palSetLine(LINE_TEMP_2);
		break;
	case lm_lo:
		palSetLineMode(LINE_TEMP_2, PAL_MODE_OUTPUT_PUSHPULL);
		palClearLine(LINE_TEMP_2);
		break;
	case lm_adc:
		palSetLineMode(LINE_TEMP_2, PAL_MODE_INPUT_ANALOG);
		break;
	}

	switch (l3) {
	case lm_hi:
		palSetLineMode(LINE_TEMP_3, PAL_MODE_OUTPUT_PUSHPULL);
		palSetLine(LINE_TEMP_3);
		break;
	case lm_lo:
		palSetLineMode(LINE_TEMP_3, PAL_MODE_OUTPUT_PUSHPULL);
		palClearLine(LINE_TEMP_3);
		break;
	case lm_adc:
		palSetLineMode(LINE_TEMP_3, PAL_MODE_INPUT_ANALOG);
		break;
	}
}

static THD_FUNCTION(hw_thd, p) {
	(void)p;
	chRegSetThreadName("HW");

	for(;;) {
		for (int i = 0;i < 12;i++) {
			// T Charge
			m_temps[0] = pwr_get_temp(4);

			uint16_t bits = 0xFFFF & ~(1 << i);
			shift_out_data(bits);
			chThdSleepMilliseconds(10);

			systime_t delay_ms = 20;

			{
				set_temp_lines(lm_adc, lm_lo, lm_lo, lm_lo);
				chThdSleepMilliseconds(delay_ms);
				float v_lo = pwr_get_temp_volt(0);
				temp_v_lo[i][0] = v_lo;
				set_temp_lines(lm_adc, lm_hi, lm_hi, lm_hi);
				chThdSleepMilliseconds(delay_ms);
				float v_hi = pwr_get_temp_volt(0);
				temp_v_hi[i][0] = v_hi;
				m_temps[i * 4 + 1] = NTC_TEMP_FROM_RES((10e3 * v_lo) / (3.3 - v_hi));
			}

			{
				set_temp_lines(lm_lo, lm_adc, lm_lo, lm_lo);
				chThdSleepMilliseconds(delay_ms);
				float v_lo = pwr_get_temp_volt(1);
				temp_v_lo[i][1] = v_lo;
				set_temp_lines(lm_hi, lm_adc, lm_hi, lm_hi);
				chThdSleepMilliseconds(delay_ms);
				float v_hi = pwr_get_temp_volt(1);
				temp_v_hi[i][1] = v_hi;
				m_temps[i * 4 + 2] = NTC_TEMP_FROM_RES((10e3 * v_lo) / (3.3 - v_hi));
			}

			{
				set_temp_lines(lm_lo, lm_lo, lm_adc, lm_lo);
				chThdSleepMilliseconds(delay_ms);
				float v_lo = pwr_get_temp_volt(2);
				temp_v_lo[i][2] = v_lo;
				set_temp_lines(lm_hi, lm_hi, lm_adc, lm_hi);
				chThdSleepMilliseconds(delay_ms);
				float v_hi = pwr_get_temp_volt(2);
				temp_v_hi[i][2] = v_hi;
				m_temps[i * 4 + 3] = NTC_TEMP_FROM_RES((10e3 * v_lo) / (3.3 - v_hi));
			}

			{
				set_temp_lines(lm_lo, lm_lo, lm_lo, lm_adc);
				chThdSleepMilliseconds(delay_ms);
				float v_lo = pwr_get_temp_volt(3);
				temp_v_lo[i][3] = v_lo;
				set_temp_lines(lm_hi, lm_hi, lm_hi, lm_adc);
				chThdSleepMilliseconds(delay_ms);
				float v_hi = pwr_get_temp_volt(3);
				temp_v_hi[i][3] = v_hi;
				m_temps[i * 4 + 4] = NTC_TEMP_FROM_RES((10e3 * v_lo) / (3.3 - v_hi));
			}
		}
	}
}

static THD_FUNCTION(hw_thd_mon, p) {
	(void)p;
	chRegSetThreadName("HW Mon");

	for(;;) {
		io_board_adc_values *adc = comm_can_get_io_board_adc_1_4_index(0);

		if (UTILS_AGE_S(0) > 1.0 && adc && UTILS_AGE_S(adc->rx_time) < 0.1) {
			sleep_reset();
		}

		// Uncomment to disable sleep
		sleep_reset();

		if (hw_temp_cell_max() > 60.0 && 0) {
			play_melody(MEL_ERROR);
		} else {
			chThdSleepMilliseconds(1);
		}

		bool jet_is_connected = (adc->rx_time != 0 && (UTILS_AGE_S(adc->rx_time)<0.5));
		if (jet_is_connected) {
			if (!jet_was_connected) {
				jet_was_connected = true;
				play_melody(MEL_JET_CONNECTED);
			}
		} else if (jet_was_connected) {
			jet_was_connected = false;
			play_melody(MEL_JET_DISCONNECTED);
		}

		// Check for IF board and disable 12V output in case there is a short
		if (!awake_block && hw_get_v_12v() < 5.0) {
			hw_test_if_conn(false);
		}
	}
}

float hw_temp_cell_max(void) {
	float res = -250.0;

	// We skip the first and last four sensors for the purpose of
	// measuring the cell temperature, because they are close to
	// components that generate heat, i.e.
	// fuses and shunts (N-4) and the antenna board (1-5)

	for (int i = 1 + 4; i < HW_TEMP_SENSORS - 4;i++) {
		if (bms_if_get_temp(i) > res) {
			res = bms_if_get_temp(i);
		}
	}

	return res;
}

float hw_get_temp(int sensor) {
	if (sensor < HW_TEMP_SENSORS) {
		return m_temps[sensor];
	} else {
		return -1.0;
	}
}

static void terminal_mc_en(int argc, const char **argv) {
	if (argc == 2) {
		int en = -1;
		sscanf(argv[1], "%d", &en);

		if (en >= 0) {
			palWriteLine(LINE_MC_EN, en ? 1 : 0);
			commands_printf("OK\n");
			return;
		}
	}

	commands_printf("Invalid arguments\n");
}

static void terminal_chg_en(int argc, const char **argv) {
	if (argc == 2) {
		int en = -1;
		sscanf(argv[1], "%d", &en);

		if (en >= 0) {
			palWriteLine(LINE_BATT_OUT_EN, en ? 1 : 0);
			commands_printf("OK\n");
			return;
		}
	}

	commands_printf("Invalid arguments\n");
}

static void terminal_buzzer_test(int argc, const char **argv) {
	for (size_t i = 1; i < (size_t)argc; i++) {
		play_melody(argv[i]);
	}
}

static void terminal_hw_info(int argc, const char **argv) {
	(void)argc; (void)argv;

	float v1 = ltc_last_gpio_voltage(LTC_GPIO_CURR_MON);
	float v2 = ltc_last_gpio_voltage(LTC_GPIO_CURR_MON_2);

	float i1 = (v1 - 1.65) * (1.0 / HW_SHUNT_AMP_GAIN) * (1.0 / backup.config.ext_shunt_res);
	float i2 = (v2 - 1.65) * (1.0 / HW_SHUNT_AMP_GAIN) * (1.0 / backup.config.ext_shunt_res);

	commands_printf("I1         : %.3f A (%.3f V)", i1, v1);
	commands_printf("I2         : %.3f A (%.3f V)\n", i2, v2);
	commands_printf("12 V Port  : %.2f V", hw_get_v_12v());
	commands_printf("Charge Port: %.2f V", pwr_get_vcharge());
	commands_printf("Chg Port   : %d\n", palReadLine(LINE_BATT_OUT_EN));

	commands_printf("Index  ADC1_LO_HI  ADC2_LO_HI  ADC3_LO_HI  ADC4_LO_HI");
	for (int i = 0; i < 12;i++) {
		commands_printf("%2d     %.2f--%.2f  %.2f--%.2f  %.2f--%.2f  %.2f--%.2f", i,
				temp_v_lo[i][0], temp_v_hi[i][0],
				temp_v_lo[i][1], temp_v_hi[i][1],
				temp_v_lo[i][2], temp_v_hi[i][2],
				temp_v_lo[i][3], temp_v_hi[i][3]);
	}
}

static void terminal_test_if_conn(int argc, const char **argv) {
	(void)argc; (void)argv;
	hw_test_if_conn(true);
}

// Check if interface is connected or we have a short
// Enables 12v output if output is not shorted
// Returns true if:
// - Interface is detected
// - Output is shorted (could be in charger)
bool hw_test_if_conn(bool print) {
	bool res = false;

	awake_block = true;

	palClearLine(LINE_12V_EN);
	palSetLine(LINE_12V_SENSE_EN); // Disable (active low)
	chThdSleepMilliseconds(100);

	if (print) {
		commands_printf("Disabling power...");
		chThdSleepMilliseconds(2000);
	}

	float v_off = hw_get_v_12v();

	palClearLine(LINE_12V_SENSE_EN); // Enable (active low)
	chThdSleepMilliseconds(100); // Allow time for capacitors charge

	float v_sense = hw_get_v_12v();
	bool sense_short = true;

	if (v_sense > 0.5) {
		sense_short = false;
		palSetLine(LINE_12V_EN);
		chThdSleepMilliseconds(100); // Allow time for capacitors charge
	}else{
		res = true;
	}

	float v_after = hw_get_v_12v();

	palSetLine(LINE_12V_SENSE_EN); // Disable

	awake_block = false;

	if (print) {
		commands_printf("Voltage off  : %.2f", v_off);
		commands_printf("Voltage sense: %.2f", v_sense);
		commands_printf("Voltage after: %.2f", v_after);

		if (sense_short) {
			commands_printf("Output shorted, could be in charger");
		}
	}

	systime_t t_start = chVTGetSystemTimeX();

	io_board_adc_values *adc = 0;

	// < 0.5 V is a short
	// > 3.0 V nothing plugged
	// < 2.5 V and > 0.5 V means something is plugged in
	if (v_sense > 0.5 && v_sense < 2.5) {
		if (print){
			commands_printf("Waiting for interface response...");
		}
		// Allow up to 2.0 seconds for IO board to update the adc rx_time
		while (UTILS_AGE_S(t_start) < 2.0) {
			adc = comm_can_get_io_board_adc_1_4_index(0);

			if (adc && adc->rx_time > t_start) {
				res = true;
				break;
			}

			chThdSleepMilliseconds(1);
		}
	}

	if (print) {
		if (res) {
			commands_printf("Interface woke up in %.2f seconds!", UTILS_AGE_S(t_start));
		} else {
			commands_printf("Waiting for interface timed out.");
		}

		commands_printf(" ");
	}
	return res;
}

static void terminal_reset_pwr(int argc, const char **argv) {
	(void)argc; (void)argv;

	awake_block = true;
	palClearLine(LINE_12V_EN);
	palClearLine(LINE_MC_EN);
	palClearLine(LINE_ESP_EN);
	palSetLine(LINE_CURR_MEASURE_EN);

	commands_printf("Disabling power...");
	chThdSleepMilliseconds(2000);
	commands_printf("Enabling power");

	awake_block = false;
	palClearLine(LINE_CURR_MEASURE_EN);
	palSetLine(LINE_12V_EN);
	palSetLine(LINE_MC_EN);
	palSetLine(LINE_ESP_EN);
}



float hw_get_v_12v(void) {
	return pwr_get_temp_volt(5) / 2.2 * (39.0 + 2.2);
}
