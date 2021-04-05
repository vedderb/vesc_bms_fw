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

#include "utils.h"

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
int utils_middle_of_3_int(int a, int b, int c) {
	int middle;

	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else if ((b <= a) && (b <= c)) {
		middle = (a <= c) ? a : c;
	} else {
		middle = (a <= b) ? a : b;
	}
	return middle;
}

uint32_t utils_crc32c(uint8_t *data, uint32_t len) {
	uint32_t crc = 0xFFFFFFFF;

	for (uint32_t i = 0; i < len;i++) {
		uint32_t byte = data[i];
		crc = crc ^ byte;

		for (int j = 7;j >= 0;j--) {
			uint32_t mask = -(crc & 1);
			crc = (crc >> 1) ^ (0x82F63B78 & mask);
		}
	}

	return ~crc;
}

const char* utils_fault_to_string(bms_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE"; break;
	case FAULT_CODE_CHARGE_OVERCURRENT: return "FAULT_CODE_CHARGE_OVERCURRENT"; break;
	case FAULT_CODE_CHARGE_OVERTEMP: return "FAULT_CODE_CHARGE_OVERTEMP"; break;
	default: return "FAULT_UNKNOWN"; break;
	}
}

const char* utils_hw_type_to_string(HW_TYPE hw) {
	switch (hw) {
	case HW_TYPE_VESC: return "HW_TYPE_VESC"; break;
	case HW_TYPE_VESC_BMS: return "HW_TYPE_VESC_BMS"; break;
	case HW_TYPE_CUSTOM_MODULE: return "HW_TYPE_CUSTOM_MODULE"; break;
	default: return "FAULT_HARDWARE"; break;
	}
}

float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int utils_truncate_number(float *number, float min, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

int utils_truncate_number_int(int *number, int min, int max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

// A mapping of a samsung 30q cell for % remaining capacity vs. voltage from
// 4.2 to 3.2, note that the you lose 15% of the 3Ah rated capacity in this range
float utils_batt_liion_norm_v_to_capacity(float norm_v) {
	// constants for polynomial fit of lithium ion battery
	const float li_p[] = {
						  -2.979767, 5.487810, -3.501286, 1.675683, 0.317147};
	utils_truncate_number(&norm_v,0.0,1.0);
	float v2 = norm_v*norm_v;
	float v3 = v2*norm_v;
	float v4 = v3*norm_v;
	float v5 = v4*norm_v;
	float capacity = li_p[0] * v5 + li_p[1] * v4 + li_p[2] * v3 + li_p[3] * v2 + li_p[4] * norm_v;
	return capacity;
}
