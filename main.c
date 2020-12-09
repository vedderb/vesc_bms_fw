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

#include "ltc6813.h"
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

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

__attribute__((section(".ram4"))) volatile backup_data backup;

static PWMConfig pwmcfg = {
		1000000,
		500,
		NULL,
		{
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL}
		},
		0,
		0
};

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
			backup.controller_id_init_flag != VAR_INIT_CODE) {
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

	if (backup.conf_flash_write_cnt_init_flag != VAR_INIT_CODE) {
		backup.conf_flash_write_cnt = 0;
		backup.conf_flash_write_cnt_init_flag = VAR_INIT_CODE;
	}

	if (backup.config_init_flag != MAIN_CONFIG_T_SIGNATURE) {
		confparser_set_defaults_main_config_t((main_config_t*)(&backup.config));
		backup.config_init_flag = MAIN_CONFIG_T_SIGNATURE;
		backup.config.controller_id = backup.controller_id;
	}

	conf_general_apply_hw_limits((main_config_t*)&backup.config);

	palSetLineMode(LINE_LED_RED, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);

	LED_OFF(LINE_LED_RED);
	LED_OFF(LINE_LED_GREEN);

	// USB needs some time to detect if a cable is connected, so start it before powering the regulators
	// to not waste to much power.
	commands_init();
	comm_usb_init();
	chThdSleepMilliseconds(500);

	pwr_init();
	ltc_init();
	bms_if_init();
	comm_can_init();
	comm_can_set_baud(backup.config.can_baud_rate);

#ifdef HDC1080_SDA_GPIO
	hdc1080_init(HDC1080_SDA_GPIO, HDC1080_SDA_PIN,
			HDC1080_SCL_GPIO, HDC1080_SCL_PIN);
#endif

	// Buzzer
	pwmStart(&PWMD4, &pwmcfg);
//	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(2));
//	pwmEnableChannel(&PWMD4, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 5000));

	sleep_init();
	timeout_init();

	for(;;) {
		backup.controller_id = backup.config.controller_id;
		chThdSleepMilliseconds(1);
	}

	return 0;
}
