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

#include "commands.h"
#include "bms_if.h"
#include "buffer.h"
#include "main.h"
#include "comm_can.h"
#include "mempools.h"
#include "confparser.h"
#include "packet.h"
#include "terminal.h"
#include "confxml.h"
#include "flash_helper.h"
#include "timeout.h"
#include "sleep.h"
#include "utils.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Private variables
static uint8_t send_buffer_global[PACKET_MAX_PL_LEN];
static mutex_t send_buffer_mutex;
static mutex_t print_mutex;
static mutex_t terminal_mutex;
static uint8_t blocking_thread_cmd_buffer[PACKET_MAX_PL_LEN];
static volatile unsigned int blocking_thread_cmd_len = 0;
static volatile bool is_blocking = false;

// Threads
static THD_FUNCTION(blocking_thread, arg);
static THD_WORKING_AREA(blocking_thread_wa, 2048);
static thread_t *blocking_tp;

// Function pointers
static void(* volatile send_func)(unsigned char *data, unsigned int len) = 0;
static void(* volatile send_func_blocking)(unsigned char *data, unsigned int len) = 0;

void commands_init(void) {
	chMtxObjectInit(&send_buffer_mutex);
	chMtxObjectInit(&print_mutex);
	chMtxObjectInit(&terminal_mutex);
	chThdCreateStatic(blocking_thread_wa, sizeof(blocking_thread_wa), NORMALPRIO, blocking_thread, NULL);
}

void commands_process_packet(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {
	if (len < 1 || reply_func == 0) {
		return;
	}

	send_func = reply_func;

	COMM_PACKET_ID packet_id = data[0];
	data++;
	len--;

	sleep_reset();

	switch (packet_id) {
	case COMM_FW_VERSION: {
		int32_t ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;

		strcpy((char*)(send_buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;

		memcpy(send_buffer + ind, STM32_UUID_8, 12);
		ind += 12;

		send_buffer[ind++] = 0;
		send_buffer[ind++] = FW_TEST_VERSION_NUMBER;

		send_buffer[ind++] = HW_TYPE_VESC_BMS;

		send_buffer[ind++] = 1; // One custom config

		reply_func(send_buffer, ind);
	} break;

	case COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW:
		data[-1] = COMM_JUMP_TO_BOOTLOADER_HW;
		comm_can_send_buffer(255, data - 1, len + 1, 2);
		chThdSleepMilliseconds(100);
		/* Falls through. */
		/* no break */
	case COMM_JUMP_TO_BOOTLOADER_HW: {
		int32_t ind = 0;
		HW_TYPE hw = data[ind++];
		char *hw_name = (char*)data + ind;
		ind += strlen(hw_name) + 1;

		if (hw != HW_TYPE_VESC_BMS || strcmp(hw_name, HW_NAME) != 0) {
			break;
		}

		data += ind;
		len -= ind;
	}
		/* Falls through. */
		/* no break */
	case COMM_JUMP_TO_BOOTLOADER:
		flash_helper_jump_to_bootloader();
		break;

	case COMM_ERASE_NEW_APP_ALL_CAN_HW:
		data[-1] = COMM_ERASE_NEW_APP_HW;
		comm_can_send_buffer(255, data - 1, len + 1, 2);
		chThdSleepMilliseconds(1500);
		/* Falls through. */
		/* no break */
	case COMM_ERASE_NEW_APP_HW: {
		int32_t ind = 0;
		HW_TYPE hw = data[ind++];
		char *hw_name = (char*)data + ind;
		ind += strlen(hw_name) + 1;

		if (hw != HW_TYPE_VESC_BMS || strcmp(hw_name, HW_NAME) != 0) {
			break;
		}

		data += ind;
		len -= ind;
	}
	/* Falls through. */
	/* no break */
	case COMM_ERASE_NEW_APP: {
		int32_t ind = 0;

		uint16_t flash_res = flash_helper_erase_new_app(buffer_get_uint32(data, &ind));

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_ERASE_NEW_APP;
		send_buffer[ind++] = flash_res == HAL_OK ? 1 : 0;
		reply_func(send_buffer, ind);
	} break;

	case COMM_ERASE_BOOTLOADER_ALL_CAN_HW:
		data[-1] = COMM_ERASE_BOOTLOADER_HW;
		comm_can_send_buffer(255, data - 1, len + 1, 2);
		chThdSleepMilliseconds(1500);
		/* Falls through. */
		/* no break */
	case COMM_ERASE_BOOTLOADER_HW: {
		int32_t ind = 0;
		HW_TYPE hw = data[ind++];
		char *hw_name = (char*)data + ind;
		ind += strlen(hw_name) + 1;

		if (hw != HW_TYPE_VESC_BMS || strcmp(hw_name, HW_NAME) != 0) {
			break;
		}

		data += ind;
		len -= ind;
	}
	/* Falls through. */
	/* no break */
	case COMM_ERASE_BOOTLOADER: {
		int32_t ind = 0;

		uint16_t flash_res = flash_helper_erase_bootloader();

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_ERASE_BOOTLOADER;
		send_buffer[ind++] = flash_res == HAL_OK ? 1 : 0;
		reply_func(send_buffer, ind);
	} break;

	case COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW:
		data[-1] = COMM_WRITE_NEW_APP_DATA_HW;

		comm_can_send_buffer(255, data - 1, len + 1, 2);
		/* Falls through. */
		/* no break */
	case COMM_WRITE_NEW_APP_DATA_HW: {
		int32_t ind = 0;
		HW_TYPE hw = data[ind++];
		char *hw_name = (char*)data + ind;
		ind += strlen(hw_name) + 1;

		if (hw != HW_TYPE_VESC_BMS || strcmp(hw_name, HW_NAME) != 0) {
			break;
		}

		data += ind;
		len -= ind;
	}
	/* Falls through. */
	/* no break */
	case COMM_WRITE_NEW_APP_DATA: {
		int32_t ind = 0;
		uint32_t new_app_offset = buffer_get_uint32(data, &ind);

		// Pad to multiple of 8 bytes
		while (((len - ind) % 8) != 0) {
			data[len++] = 0;
		}

		uint16_t flash_res = flash_helper_write_new_app_data(new_app_offset, data + ind, len - ind);

		ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_WRITE_NEW_APP_DATA;
		send_buffer[ind++] = flash_res == HAL_OK ? 1 : 0;
		buffer_append_uint32(send_buffer, new_app_offset, &ind);
		reply_func(send_buffer, ind);
	} break;

	case COMM_BMS_GET_VALUES: {
		int32_t ind = 0;
		uint8_t send_buffer[128];

		send_buffer[ind++] = packet_id;

		buffer_append_float32(send_buffer, bms_if_get_v_tot(), 1e6, &ind);
		buffer_append_float32(send_buffer, bms_if_get_v_charge(), 1e6, &ind);
		buffer_append_float32(send_buffer, bms_if_get_i_in(), 1e6, &ind);
		buffer_append_float32(send_buffer, bms_if_get_i_in_ic(), 1e6, &ind);
		buffer_append_float32(send_buffer, bms_if_get_ah_cnt(), 1e3, &ind);
		buffer_append_float32(send_buffer, bms_if_get_wh_cnt(), 1e3, &ind);

		// Cell voltages
		send_buffer[ind++] = backup.config.cell_num;
		for (int i = backup.config.cell_first_index;i <
		(backup.config.cell_num + backup.config.cell_first_index);i++) {
			buffer_append_float16(send_buffer, bms_if_get_v_cell(i), 1e3, &ind);
		}

		// Balancing state
		for (int i = backup.config.cell_first_index;i <
		(backup.config.cell_num + backup.config.cell_first_index);i++) {
			send_buffer[ind++] = bms_if_is_balancing_cell(i);
		}

		// Temperatures
		send_buffer[ind++] = HW_ADC_TEMP_SENSORS;
		for (int i = 0;i < HW_ADC_TEMP_SENSORS;i++) {
			buffer_append_float16(send_buffer, bms_if_get_temp(i), 1e2, &ind);
		}
		buffer_append_float16(send_buffer, bms_if_get_temp_ic(), 1e2, &ind);

		// Humidity
		buffer_append_float16(send_buffer, bms_if_get_humidity_sensor_temp(), 1e2, &ind);
		buffer_append_float16(send_buffer, bms_if_get_humitidy(), 1e2, &ind);

		// Highest cell temperature
		buffer_append_float16(send_buffer, HW_TEMP_CELLS_MAX(), 1e2, &ind);

		// State of charge and state of health
		buffer_append_float16(send_buffer, bms_if_get_soc(), 1e3, &ind);
		buffer_append_float16(send_buffer, bms_if_get_soh(), 1e3, &ind);

		reply_func(send_buffer, ind);
	} break;

	case COMM_FORWARD_CAN:
		comm_can_send_buffer(data[0], data + 1, len - 1, 0);
		break;

	case COMM_BMS_SET_CHARGE_ALLOWED:
		bms_if_set_charge_allowed(data[0]);
		break;

	case COMM_BMS_SET_BALANCE_OVERRIDE:
		bms_if_set_balance_override(data[0], data[1]);
		break;

	case COMM_BMS_RESET_COUNTERS:
		if (data[0]) {
			bms_if_reset_counter_ah();
		}

		if (data[1]) {
			bms_if_reset_counter_wh();
		}
		break;

	case COMM_BMS_FORCE_BALANCE:
		bms_if_force_balance(data[0]);
		break;

	case COMM_GET_CUSTOM_CONFIG:
	case COMM_GET_CUSTOM_CONFIG_DEFAULT: {
		main_config_t *conf = mempools_alloc_conf();

		int conf_ind = data[0];

		if (conf_ind != 0) {
			break;
		}

		if (packet_id == COMM_GET_CUSTOM_CONFIG) {
			*conf = backup.config;
		} else {
			confparser_set_defaults_main_config_t(conf);
		}

		chMtxLock(&send_buffer_mutex);
		int32_t ind = 0;
		send_buffer_global[ind++] = packet_id;
		send_buffer_global[ind++] = conf_ind;
		int32_t len = confparser_serialize_main_config_t(send_buffer_global + ind, conf);
		commands_send_packet(send_buffer_global, len + ind);
		chMtxUnlock(&send_buffer_mutex);

		mempools_free_conf(conf);
	} break;

	case COMM_SET_CUSTOM_CONFIG: {
		main_config_t *conf = mempools_alloc_conf();
		*conf = backup.config;

		int conf_ind = data[0];

		if (conf_ind == 0 && confparser_deserialize_main_config_t(data + 1, conf)) {
			conf_general_apply_hw_limits(conf);
			backup.config = *conf;
			flash_helper_store_backup_data();
			comm_can_set_baud(backup.config.can_baud_rate);

			int32_t ind = 0;
			uint8_t send_buffer[50];
			send_buffer[ind++] = packet_id;
			reply_func(send_buffer, ind);
		} else {
			commands_printf("Warning: Could not set configuration");
		}

		mempools_free_conf(conf);
	} break;

	case COMM_GET_CUSTOM_CONFIG_XML: {
		int32_t ind = 0;

		int conf_ind = data[ind++];

		if (conf_ind != 0) {
			break;
		}

		int32_t len_conf = buffer_get_int32(data, &ind);
		int32_t ofs_conf = buffer_get_int32(data, &ind);

		if ((len_conf + ofs_conf) > DATA_MAIN_CONFIG_T__SIZE || len_conf > (PACKET_MAX_PL_LEN - 10)) {
			break;
		}

		chMtxLock(&send_buffer_mutex);
		ind = 0;
		send_buffer_global[ind++] = packet_id;
		send_buffer_global[ind++] = conf_ind;
		buffer_append_int32(send_buffer_global, DATA_MAIN_CONFIG_T__SIZE, &ind);
		buffer_append_int32(send_buffer_global, ofs_conf, &ind);
		memcpy(send_buffer_global + ind, data_main_config_t_ + ofs_conf, len_conf);
		ind += len_conf;
		reply_func(send_buffer_global, ind);

		chMtxUnlock(&send_buffer_mutex);
	} break;

	case COMM_TERMINAL_CMD_SYNC:
		data[len] = '\0';
		chMtxLock(&terminal_mutex);
		terminal_process_string((char*)data);
		chMtxUnlock(&terminal_mutex);
		break;

		// Power switch
	case COMM_PSW_GET_STATUS: {
		int32_t ind = 0;
		bool by_id = data[ind++];
		int id_ind = buffer_get_int16(data, &ind);

		int psws_num = 0;
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			psw_status *stat = comm_can_get_psw_status_index(i);
			if (stat->id >= 0) {
				psws_num++;
			} else {
				break;
			}
		}

		psw_status *stat = 0;
		if (by_id) {
			stat = comm_can_get_psw_status_id(id_ind);
		} else if (id_ind < psws_num) {
			stat = comm_can_get_psw_status_index(id_ind);
		}

		if (stat) {
			ind = 0;
			uint8_t send_buffer[70];

			send_buffer[ind++] = packet_id;
			buffer_append_int16(send_buffer, stat->id, &ind);
			buffer_append_int16(send_buffer, psws_num, &ind);
			buffer_append_float32_auto(send_buffer, UTILS_AGE_S(stat->rx_time), &ind);
			buffer_append_float32_auto(send_buffer, stat->v_in, &ind);
			buffer_append_float32_auto(send_buffer, stat->v_out, &ind);
			buffer_append_float32_auto(send_buffer, stat->temp, &ind);
			send_buffer[ind++] = stat->is_out_on;
			send_buffer[ind++] = stat->is_pch_on;
			send_buffer[ind++] = stat->is_dsc_on;

			reply_func(send_buffer, ind);
		}
	} break;

	case COMM_PSW_SWITCH: {
		int32_t ind = 0;
		int id = buffer_get_int16(data, &ind);
		bool is_on = data[ind++];
		bool plot = data[ind++];
		comm_can_psw_switch(id, is_on, plot);
	} break;

		// Blocking commands. Only one of them runs at any given time, in their
		// own thread. If other blocking commands come before the previous one has
		// finished, they are discarded.
	case COMM_TERMINAL_CMD:
	case COMM_PING_CAN:
	case COMM_BMS_ZERO_CURRENT_OFFSET:
		if (!is_blocking) {
			memcpy(blocking_thread_cmd_buffer, data - 1, len + 1);
			blocking_thread_cmd_len = len + 1;
			is_blocking = true;
			send_func_blocking = reply_func;
			chEvtSignal(blocking_tp, (eventmask_t)1);
		}
		break;

	default:
		break;
	}
}

void commands_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

void commands_printf(const char* format, ...) {
	chMtxLock(&print_mutex);

	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer + 1, 254, format, arg);
	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer,
				(len < 254) ? len + 1 : 255);
	}

	chMtxUnlock(&print_mutex);
}

static THD_FUNCTION(blocking_thread, arg) {
	(void)arg;

	chRegSetThreadName("comm_block");

	blocking_tp = chThdGetSelfX();

	for(;;) {
		is_blocking = false;

		chEvtWaitAny((eventmask_t) 1);

		uint8_t *data = blocking_thread_cmd_buffer;
		unsigned int len = blocking_thread_cmd_len;

		COMM_PACKET_ID packet_id;
		static uint8_t send_buffer[512];

		packet_id = data[0];
		data++;
		len--;

		switch (packet_id) {
		case COMM_BMS_ZERO_CURRENT_OFFSET:
			bms_if_zero_current_offset();
			break;

		case COMM_TERMINAL_CMD:
			data[len] = '\0';
			chMtxLock(&terminal_mutex);
			terminal_process_string((char*)data);
			chMtxUnlock(&terminal_mutex);
			break;

		case COMM_PING_CAN: {
			int32_t ind = 0;
			send_buffer[ind++] = COMM_PING_CAN;

			for (uint8_t i = 0;i < 255;i++) {
				HW_TYPE hw_type;
				if (comm_can_ping(i, &hw_type)) {
					send_buffer[ind++] = i;
				}
			}

			if (send_func_blocking) {
				send_func_blocking(send_buffer, ind);
			}
		} break;

		default:
			break;
		}
	}
}
