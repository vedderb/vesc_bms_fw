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

#include "conf_general.h"
#include "usbcfg.h"
#include "pwr.h"
#include "comm_can.h"
#include "utils.h"
#include "comm_usb.h"
#include "bms_if.h"
#include "hdc1080.h"
#include "confparser.h"
#include "commands.h"
#include "timeout.h"
#include "sleep.h"
#include "flash_helper.h"
#include "comm_uart.h"
#include "hw.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

__attribute__((section(".ram4"))) volatile backup_data backup;

#ifndef VAR_INIT_CODE_HW_CONF
#define VAR_INIT_CODE_HW_CONF		VAR_INIT_CODE
#endif

int main(void) {
	halInit();
	chSysInit();

	// Stop debug mode in case no power cycle has been done after upload. This
	// saves power.
	DBGMCU->CR = DBGMCU_CR_DBG_STOP;

	// If there is no backup data in RAM try to load it from flash. This can
	// happen if power was lost.
	if (backup.ah_cnt_init_flag != VAR_INIT_CODE ||
			backup.wh_cnt_init_flag != VAR_INIT_CODE ||
			backup.ic_i_sens_v_ofs_init_flag != VAR_INIT_CODE ||
			backup.controller_id_init_flag != VAR_INIT_CODE ||
			backup.send_can_status_rate_hz_init_flag != VAR_INIT_CODE ||
			backup.can_baud_rate_init_flag != VAR_INIT_CODE ||
			backup.conf_flash_write_cnt_init_flag != VAR_INIT_CODE ||
			backup.usb_cnt_init_flag != VAR_INIT_CODE ||
			backup.hw_config_init_flag != VAR_INIT_CODE_HW_CONF ||
			backup.ah_cnt_chg_total_init_flag != VAR_INIT_CODE ||
			backup.wh_cnt_chg_total_init_flag != VAR_INIT_CODE ||
			backup.ah_cnt_dis_total_init_flag != VAR_INIT_CODE ||
			backup.wh_cnt_dis_total_init_flag != VAR_INIT_CODE) {
		flash_helper_load_backup_data();
	}

	// Reset backup counters that haven't been set. Should work across firmware uploads.
	if (backup.ah_cnt_init_flag != VAR_INIT_CODE) {
		backup.ah_cnt = 0.0;
		backup.ah_cnt_init_flag = VAR_INIT_CODE;
	}

	if (backup.wh_cnt_init_flag != VAR_INIT_CODE) {
		backup.wh_cnt = 0.0;
		backup.wh_cnt_init_flag = VAR_INIT_CODE;
	}

	if (backup.ic_i_sens_v_ofs_init_flag != VAR_INIT_CODE) {
		backup.ic_i_sens_v_ofs = 0.0;
		backup.ic_i_sens_v_ofs_init_flag = VAR_INIT_CODE;
	}

	if (backup.controller_id_init_flag != VAR_INIT_CODE) {
		backup.controller_id = HW_DEFAULT_ID;
		backup.controller_id_init_flag = VAR_INIT_CODE;
	}

	if (backup.send_can_status_rate_hz_init_flag != VAR_INIT_CODE) {
		backup.send_can_status_rate_hz = CONF_SEND_CAN_STATUS_RATE_HZ;
		backup.send_can_status_rate_hz_init_flag = VAR_INIT_CODE;
	}

	if (backup.can_baud_rate_init_flag != VAR_INIT_CODE) {
		backup.can_baud_rate = CONF_CAN_BAUD_RATE;
		backup.can_baud_rate_init_flag = VAR_INIT_CODE;
	}

	if (backup.conf_flash_write_cnt_init_flag != VAR_INIT_CODE) {
		backup.conf_flash_write_cnt = 0;
		backup.conf_flash_write_cnt_init_flag = VAR_INIT_CODE;
	}

	if (backup.usb_cnt_init_flag != VAR_INIT_CODE) {
		backup.usb_cnt = 0;
		backup.usb_cnt_init_flag = VAR_INIT_CODE;
	}

	if (backup.hw_config_init_flag != VAR_INIT_CODE_HW_CONF) {
		memset((void*)backup.hw_config, 0, sizeof(backup.hw_config));
		backup.hw_config_init_flag = VAR_INIT_CODE_HW_CONF;
	}

	if (backup.ah_cnt_chg_total_init_flag != VAR_INIT_CODE) {
		backup.ah_cnt_chg_total = 0.0;
		backup.ah_cnt_chg_total_init_flag = VAR_INIT_CODE;
	}

	if (backup.wh_cnt_chg_total_init_flag != VAR_INIT_CODE) {
		backup.wh_cnt_chg_total = 0.0;
		backup.wh_cnt_chg_total_init_flag = VAR_INIT_CODE;
	}

	if (backup.ah_cnt_dis_total_init_flag != VAR_INIT_CODE) {
		backup.ah_cnt_dis_total = 0.0;
		backup.ah_cnt_dis_total_init_flag = VAR_INIT_CODE;
	}

	if (backup.wh_cnt_dis_total_init_flag != VAR_INIT_CODE) {
		backup.wh_cnt_dis_total = 0.0;
		backup.wh_cnt_dis_total_init_flag = VAR_INIT_CODE;
	}

	if (backup.config_init_flag != MAIN_CONFIG_T_SIGNATURE) {
		confparser_set_defaults_main_config_t((main_config_t*)(&backup.config));
		backup.config_init_flag = MAIN_CONFIG_T_SIGNATURE;
		backup.config.controller_id = backup.controller_id;
		backup.config.send_can_status_rate_hz = backup.send_can_status_rate_hz;
		backup.config.can_baud_rate = backup.can_baud_rate;

	}

	conf_general_apply_hw_limits((main_config_t*)&backup.config);

	palSetLineMode(EN_A, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(EN_B, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(ON_CANBUS, PAL_MODE_OUTPUT_PUSHPULL);

	LED_OFF(EN_A);
	LED_OFF(EN_B);
	LED_ON(ON_CANBUS);

	palSetLineMode(LINE_LED_RED, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);

	LED_OFF(LINE_LED_RED);
	LED_OFF(LINE_LED_GREEN);

	// USB needs some time to detect if a cable is connected, so start it before powering the regulators
	// to not waste too much power.
	commands_init();
	comm_usb_init();

	// Only wait for USB every 3 boots
	if (backup.usb_cnt >= 3) {
		chThdSleepMilliseconds(500);
		backup.usb_cnt = 0;
	} else {
		backup.usb_cnt++;
	}

	pwr_init();
	bms_if_init();
	comm_can_init();
	comm_can_set_baud(backup.config.can_baud_rate);

#ifdef HDC1080_SDA_GPIO
	hdc1080_init(HDC1080_SDA_GPIO, HDC1080_SDA_PIN,
			HDC1080_SCL_GPIO, HDC1080_SCL_PIN);
#endif

#ifdef BQ76940_SDA_GPIO
	bq76940_init(BQ76940_SDA_GPIO, BQ76940_SDA_PIN,
	BQ76940_SCL_GPIO , BQ76940_SCL_PIN,
	BQ76940_ALERT_GPIO , BQ76940_ALERT_PIN,
	BQ76940_LRD_GPIO , BQ76940_LRD_PIN,
	HW_SHUNT_RES);
#endif

#ifdef HW_UART_DEV
	comm_uart_init();
#endif

	sleep_init();
	timeout_init();

	for(;;) {
		backup.controller_id = backup.config.controller_id;
		backup.send_can_status_rate_hz = backup.config.send_can_status_rate_hz;
		backup.can_baud_rate = backup.config.can_baud_rate;
		chThdSleepMilliseconds(1);
	}

	return 0;
}
