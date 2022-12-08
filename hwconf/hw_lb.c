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
#include <stdio.h>

// Threads
static THD_WORKING_AREA(hw_thd_wa, 2048);
static THD_WORKING_AREA(hw_thd_mon_wa, 1024);
static THD_FUNCTION(hw_thd, p);
static THD_FUNCTION(hw_thd_mon, p);

// Private variables
static float m_temps[HW_TEMP_SENSORS];
static float m_v_charge = 0.0;

// Private functions
static void terminal_mc_en(int argc, const char **argv);
static void terminal_buzzer_test(int argc, const char **argv);
static void terminal_hw_info(int argc, const char **argv);
static void terminal_set_v_charge(int argc, const char **argv);

void hw_board_init(void) {
	palSetLineMode(LINE_CAN_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_CURR_MEASURE_EN, PAL_MODE_OUTPUT_OPENDRAIN);

	palClearLine(LINE_MC_EN);
	palClearLine(LINE_BATT_OUT_EN);
	palClearLine(LINE_12V_EN);

	palSetLineMode(LINE_MC_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_BATT_OUT_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_12V_EN, PAL_MODE_OUTPUT_PUSHPULL);

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
			"set_v_charge",
			"Set Charger Voltage Emulation",
			"[volts]",
			terminal_set_v_charge);
}

void hw_board_sleep(void) {
	palClearLine(LINE_BATT_OUT_EN);
	palClearLine(LINE_12V_EN);
	palClearLine(LINE_MC_EN);
}

void hw_stay_awake(void) {
	palSetLine(LINE_MC_EN);
	palSetLine(LINE_12V_EN);
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
			uint16_t bits = 0xFFFF & ~(1 << i);
			shift_out_data(bits);
			chThdSleepMilliseconds(10);

			systime_t delay_ms = 20;

			{
				set_temp_lines(lm_adc, lm_lo, lm_lo, lm_lo);
				chThdSleepMilliseconds(delay_ms);
				float v_lo = pwr_get_temp_volt(0);
				set_temp_lines(lm_adc, lm_hi, lm_hi, lm_hi);
				chThdSleepMilliseconds(delay_ms);
				float v_hi = pwr_get_temp_volt(0);
				m_temps[i * 4 + 0] = NTC_TEMP_FROM_RES((10e3 * v_lo) / (3.3 - v_hi));
			}

			{
				set_temp_lines(lm_lo, lm_adc, lm_lo, lm_lo);
				chThdSleepMilliseconds(delay_ms);
				float v_lo = pwr_get_temp_volt(1);
				set_temp_lines(lm_hi, lm_adc, lm_hi, lm_hi);
				chThdSleepMilliseconds(delay_ms);
				float v_hi = pwr_get_temp_volt(1);
				m_temps[i * 4 + 1] = NTC_TEMP_FROM_RES((10e3 * v_lo) / (3.3 - v_hi));
			}

			{
				set_temp_lines(lm_lo, lm_lo, lm_adc, lm_lo);
				chThdSleepMilliseconds(delay_ms);
				float v_lo = pwr_get_temp_volt(2);
				set_temp_lines(lm_hi, lm_hi, lm_adc, lm_hi);
				chThdSleepMilliseconds(delay_ms);
				float v_hi = pwr_get_temp_volt(2);
				m_temps[i * 4 + 2] = NTC_TEMP_FROM_RES((10e3 * v_lo) / (3.3 - v_hi));
			}

			{
				set_temp_lines(lm_lo, lm_lo, lm_lo, lm_adc);
				chThdSleepMilliseconds(delay_ms);
				float v_lo = pwr_get_temp_volt(3);
				set_temp_lines(lm_hi, lm_hi, lm_hi, lm_adc);
				chThdSleepMilliseconds(delay_ms);
				float v_hi = pwr_get_temp_volt(3);
				m_temps[i * 4 + 3] = NTC_TEMP_FROM_RES((10e3 * v_lo) / (3.3 - v_hi));
			}
		}
	}
}

static THD_FUNCTION(hw_thd_mon, p) {
	(void)p;
	chRegSetThreadName("HW Mon");

	for(;;) {
		if (hw_temp_cell_max() > 60.0) {
			BUZZER_ON();
			chThdSleepMilliseconds(500);
			BUZZER_OFF();
			chThdSleepMilliseconds(500);
		} else {
			chThdSleepMilliseconds(1);
		}
	}
}

float hw_temp_cell_max(void) {
	float res = -250.0;

	for (int i = 1;i < HW_TEMP_SENSORS;i++) {
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

float hw_get_v_charge(void) {
	return m_v_charge;
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

static void terminal_buzzer_test(int argc, const char **argv) {
	(void)argc; (void)argv;

	for (int i = 0;i < 3;i++) {
		BUZZER_ON();
		chThdSleepMilliseconds(500);
		BUZZER_OFF();
		chThdSleepMilliseconds(500);
	}
}

static void terminal_hw_info(int argc, const char **argv) {
	(void)argc; (void)argv;

	float v1 = ltc_last_gpio_voltage(LTC_GPIO_CURR_MON);
	float v2 = ltc_last_gpio_voltage(LTC_GPIO_CURR_MON_2);

	float i1 = (v1 - 1.65) * (1.0 / HW_SHUNT_AMP_GAIN) * (1.0 / backup.config.ext_shunt_res);
	float i2 = (v2 - 1.65) * (1.0 / HW_SHUNT_AMP_GAIN) * (1.0 / backup.config.ext_shunt_res);

	commands_printf("I1: %.3f A (%.3f V)", i1, v1);
	commands_printf("I2: %.3f A (%.3f V)\n", i2, v2);
}

static void terminal_set_v_charge(int argc, const char **argv) {
	if (argc == 2) {
		float vch = -1;
		sscanf(argv[1], "%f", &vch);

		if (vch >= 0.0 && vch < 52.0) {
			m_v_charge = vch;
			commands_printf("OK\n");
			return;
		}
	}

	commands_printf("Invalid arguments\n");
}
