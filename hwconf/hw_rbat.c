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
#include "main.h"
#include "sleep.h"
#include "buffer.h"
#include "ltc6813.h"
#include "comm_can.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define MAX_RESP_BUF_LEN  		16
#define CMD_SET_RELAY_NBR_ARGS	3

// Types
typedef enum {
	CONN_STATE_UNPLUGGED = 0,
	CONN_STATE_JETPACK,
	CONN_STATE_CHARGER,
	NUM_CONN_STATES,
} CONN_STATE;

typedef enum {
	CMD_SET_DECOMM_STATE = 0,
	CMD_GET_CONN_STATE,
	NUM_COMMANDS,
} CMD;

typedef enum {
	RSP_OK = 0,
	RSP_INVALID_CMD,
	RSP_INVALID_ARG,
	NUM_RESPONSES,
} RSP;

typedef struct __attribute__((packed)) {
	// If the battery is decommissioned it is permanently disabled.
	bool is_decommissioned;

	// A timeout means that the charger or jetpack was connected for a long time
	// without drawing power, leading to switching off the main contactor and entering sleep
	// mode. If this happens, the battery must be disconnected at least once to reset the
	// timeout and allow switching on the contactor again.
	bool did_timeout_charger;
	bool did_timeout_jetpack;
} hw_config;

// Functions
static void append_hw_data_to_buffer(uint8_t *buffer, int32_t *len);
static void update_conn_state(void);
static void terminal_psw_set(int argc, const char **argv);
static void terminal_info(int argc, const char **argv);
static void terminal_get_relay_state(int argc, const char **argv);
static void terminal_set_relay_state(int argc, const char **argv);

// Threads
static THD_WORKING_AREA(hw_thd_wa, 2048);
static THD_FUNCTION(hw_thd, p);

// Variables
static volatile CONN_STATE m_conn_state = CONN_STATE_UNPLUGGED;
static mutex_t m_sw_mutex;

// If the terminal is used to control the power switch the connector will be ignored.
static volatile bool m_terminal_override_psw = false;

// Reporting 0 SOC will prevent the VESC from drawing power. That can be used to switch off the
// contactor safely and preventing current draw during the precharge sequence.
static volatile float m_soc_override = -1.0;

// The config is stored in the backup struct so that it is retained while sleeping.
static volatile hw_config *m_config = (hw_config*)&backup.hw_config[0];

static void app_data_cmd_handler(unsigned char *data, unsigned int len) {
	uint8_t resp_buf[MAX_RESP_BUF_LEN];
	int32_t idx;
	CMD cmd;

	if (!data || !len)
		return;

	cmd = data[0];
	data++;
	idx = 0;

	switch (cmd) {
	case CMD_SET_DECOMM_STATE:
		m_config->is_decommissioned = !!data[0];
		resp_buf[idx++] = cmd;
		resp_buf[idx++] = RSP_OK;
		break;
	case CMD_GET_CONN_STATE:
		resp_buf[idx++] = cmd;
		buffer_append_int16(resp_buf, (uint16_t)m_conn_state, &idx);
		break;
	default:
		resp_buf[idx++] = cmd;
		resp_buf[idx++] = RSP_INVALID_CMD;
		break;
	}

	if (idx)
		commands_send_app_data(resp_buf, idx);
}

void hw_board_init(void) {
	chMtxObjectInit(&m_sw_mutex);

	HW_RELAY_MAIN_OFF();
	HW_RELAY_PCH_OFF();

	palSetLineMode(LINE_CURR_MEASURE_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_CAN_EN, PAL_MODE_OUTPUT_PUSHPULL);

#ifdef LINE_5V_HP_EN
	palSetLineMode(LINE_5V_HP_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLine(LINE_5V_HP_EN);
#endif
#ifdef LINE_3V3P_EN
	palSetLineMode(LINE_3V3P_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLine(LINE_3V3P_EN);
#endif

	palSetLineMode(LINE_12V_HP_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_AFTER_FUSE_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_RELAY_PCH, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_RELAY_MAIN, PAL_MODE_OUTPUT_PUSHPULL);

	palSetLine(LINE_12V_HP_EN);
	palSetLine(LINE_LED_BLUE);
	palSetLine(LINE_AFTER_FUSE_EN);

	palSetLineMode(LINE_PWRKEY_1, PAL_MODE_INPUT);
	palSetLineMode(LINE_PWRKEY_2, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAN_EN, PAL_MODE_OUTPUT_OPENDRAIN);

	terminal_register_command_callback(
			"rbat_psw_set",
			"Switch output on or off",
			"[0 or 1]",
			terminal_psw_set);

	terminal_register_command_callback(
			"rbat_info",
			"Print HW info",
			0,
			terminal_info);

	terminal_register_command_callback(
			"rbat_get_relays",
			"Print state of relays",
			0,
			terminal_get_relay_state);

	terminal_register_command_callback(
			"rbat_set_relay",
			"Set state of relay",
			"[main or pch] [on or off]",
			terminal_set_relay_state);

	commands_set_app_data_handler(app_data_cmd_handler);

	chThdSleepMilliseconds(10);
	update_conn_state();

	chThdCreateStatic(hw_thd_wa, sizeof(hw_thd_wa), NORMALPRIO, hw_thd, 0);
}

void hw_board_chg_en(bool enable) {
	if (m_conn_state == CONN_STATE_CHARGER &&
			!m_config->did_timeout_charger &&
			!m_config->is_decommissioned) {
		if (enable) {
			hw_psw_switch_on(false);
		} else {
			hw_psw_switch_off(true);
		}
	}
}

/*
 * Note that the battery voltage is reported if charging is OK. Then the contactor will be switched on
 * as soon as the charger is connected even if the charger doesn't output voltage right away.
 */
float hw_board_get_vcharge(void) {
	if (m_conn_state == CONN_STATE_CHARGER &&
			!m_config->did_timeout_charger &&
			!m_config->is_decommissioned) {
		return bms_if_get_v_tot();
	} else {
		return 0.0;
	}
}

/**
 * Switch on precharge and then the output.
 *
 * check_rise_rate
 * If true the precharge sequence will fail if the voltage rises too fast. That
 * indicates that there is no capacitive load on the output, indicating that
 * there is a connection problem.
 *
 * return
 * 0: Success
 * -1: Precharge rise rate too fast (no capacitance detected)
 * -2: Precharge timeout (too much load or short)
 * -3: Precharge relay timeout
 */
int hw_psw_switch_on(bool check_rise_rate) {
	chMtxLock(&m_sw_mutex);

	if (HW_RELAY_MAIN_IS_ON()) {
		chMtxUnlock(&m_sw_mutex);
		return true;
	}

	if (m_conn_state == CONN_STATE_JETPACK) {
		m_soc_override = 0.0;
	}

	HW_RELAY_PCH_ON();

	if (check_rise_rate) {
		// Wait for relay to start switching on
		float v_start = pwr_get_vcharge();
		float timeout_relay = 0;
		while (pwr_get_vcharge() < (v_start + 5.0)) {
			chThdSleepMilliseconds(2);
			timeout_relay += 2.0 / 1000.0;
			if (timeout_relay >= 1.0) {
				// Voltage never starts rising
				HW_RELAY_PCH_OFF();
				chMtxUnlock(&m_sw_mutex);
				return -3;
			}
		}

		// The voltage actually rises much faster than this when there is no capacitor, but there is a HW low-pass filter
		// on v_charge that delays the measured signal.
		chThdSleepMilliseconds(30);

		if (fabsf(pwr_get_vcharge() - bms_if_get_v_tot()) < (bms_if_get_v_tot() / 10)) {
			// Rise rate too fast
			HW_RELAY_PCH_OFF();
			chMtxUnlock(&m_sw_mutex);
			return -1;
		}
	}

	float timeout = 0;
	// Wait for output voltage to rise to 90 % of battery voltage
	while (fabsf(pwr_get_vcharge() - bms_if_get_v_tot()) > (bms_if_get_v_tot() / 10)) {
		chThdSleepMilliseconds(25);
		timeout += 25.0 / 1000.0;
		if (timeout >= 2.5) {
			// Timed out
			HW_RELAY_PCH_OFF();
			chMtxUnlock(&m_sw_mutex);
			return -2;
		}
	}

	HW_RELAY_MAIN_ON();
	HW_RELAY_PCH_OFF();

	chThdSleepMilliseconds(10);

	m_soc_override = -1.0;

	chMtxUnlock(&m_sw_mutex);
	return 0;
}

void hw_psw_switch_off(bool safe) {
	chMtxLock(&m_sw_mutex);

	if (!HW_RELAY_MAIN_IS_ON()) {
		chMtxUnlock(&m_sw_mutex);
		return;
	}

	// Give the VESC some time to stop drawing power before switching off. This is done to prevent
	// damage by switching off the contactor while drawing current. Sending 0 SOC should stop the VESC.
	if (safe) {
		if (m_conn_state == CONN_STATE_JETPACK) {
			m_soc_override = 0.0;
			chThdSleepMilliseconds(500);
		}
	}

	HW_RELAY_MAIN_OFF();
	chMtxUnlock(&m_sw_mutex);
}

float hw_soc_override(void) {
	return m_soc_override;
}

float hw_temp_cell_max(void) {
	float res = 1.0;

	for (int i = 0;i < 6;i++) {
		if (bms_if_get_temp(i) > res) {
			res = bms_if_get_temp(i);
		}
	}

	return res;
}

void hw_send_data(void(*reply_func)(unsigned char *data, unsigned int len)) {
	uint8_t buffer[8]; int32_t len = 0;
	append_hw_data_to_buffer(buffer, &len);
	reply_func(buffer, len);
}

void hw_send_can_data(void) {
	uint8_t buffer[8]; int32_t len = 0;
	append_hw_data_to_buffer(buffer, &len);
	comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_HW_DATA_1 << 8), buffer, len);
}

bool hw_charger_detected(void) {
	return m_conn_state == CONN_STATE_CHARGER &&
			!m_config->did_timeout_charger &&
			!m_config->is_decommissioned;
}

static void append_hw_data_to_buffer(uint8_t *buffer, int32_t *len) {
	*len = 0;

	buffer_append_float16(buffer, pwr_get_vfuse(), 1e2, len);
	buffer_append_float16(buffer, pwr_get_vcharge(), 1e2, len);
	buffer_append_float16(buffer, HW_GET_V_MOIST_SENSE(), 1e4, len);

	/*
	 * State bitfield
	 * B0 - B1: Conn state
	 * B2     : is_decommissioned
	 * B3     : did_timeout_charger
	 * B4     : did_timeout_jetpack
	 */
	uint8_t state = 0;
	state |= (m_conn_state & 0x03) << 0;
	state |= (m_config->is_decommissioned & 0x01) << 2;
	state |= (m_config->did_timeout_charger & 0x01) << 3;
	state |= (m_config->did_timeout_jetpack & 0x01) << 4;
	buffer[(*len)++] = state;
}

static void update_conn_state(void) {
	bool pwr_key_1 = !palReadLine(LINE_PWRKEY_1);
	bool pwr_key_2 = !palReadLine(LINE_PWRKEY_2);

	if (!pwr_key_1 && !pwr_key_2) {
		m_conn_state = CONN_STATE_UNPLUGGED;
		if (!m_terminal_override_psw) {
			hw_psw_switch_off(true);
		}

		m_config->did_timeout_charger = false;
		m_config->did_timeout_jetpack = false;
	} else if (!pwr_key_1 && pwr_key_2) {
		m_conn_state = CONN_STATE_JETPACK;
	} else if (pwr_key_1 && pwr_key_2) {
		m_conn_state = CONN_STATE_CHARGER;
	}

	// Restore SOC override if the jetpack isn't connected.
	if (m_conn_state != CONN_STATE_JETPACK) {
		m_soc_override = -1.0;
	}
}

static THD_FUNCTION(hw_thd, p) {
	(void)p;
	chRegSetThreadName("HW");

	int jetpack_delay_cnt = 0;
	bool psw_ok = true;
	float conn_timeout = 0.0;

	for(;;) {
		update_conn_state();

		bool sw_on_ok = true;
		if (bms_if_get_v_cell_min() < S_MIN_CELL_VOLTAGE_ON) {
			sw_on_ok = false;
		} else  if (m_config->did_timeout_jetpack) {
			sw_on_ok = false;
		} else if (m_config->is_decommissioned) {
			sw_on_ok = false;
		}

		/*
		 * If the jetpack is detected, wait 100 iterations (about 1s) then start the precharge and
		 * main contactor switching sequence. If switching on fails (which can happen if the
		 * precharge times out due to e.g. a short) the jetpack must be unplugged and replugged
		 * to make the next attempt.
		 */
		if (m_conn_state == CONN_STATE_JETPACK && sw_on_ok) {
			sleep_reset();

			if (jetpack_delay_cnt < 100) {
				jetpack_delay_cnt++;
			} else {
				if (psw_ok) {
					psw_ok = hw_psw_switch_on(true) == 0;
				}
			}
		} else {
			jetpack_delay_cnt = 0;
			psw_ok = true;
		}

		if (HW_RELAY_MAIN_IS_ON()) {
			sleep_reset();

			/*
			 * If the jetpack or charger is left connected for too long without any current flow
			 * the contactor is switched off and sleep mode is entered. The timeout state is used
			 * to prevent switching the contactor on until the charger or jetpack has been unplugged.
			 */
			if (fabsf(bms_if_get_i_in_ic()) < S_NO_USE_CURRENT_TRES) {
				conn_timeout += 0.01;
			} else {
				conn_timeout = 0.0;
			}

			if (conn_timeout >= S_NO_USE_TIMEOUT) {
				if (m_conn_state == CONN_STATE_JETPACK) {
					m_config->did_timeout_jetpack = true;
					hw_psw_switch_off(true);
				} else if (m_conn_state == CONN_STATE_CHARGER) {
					m_config->did_timeout_charger = true;
					hw_psw_switch_off(true);
				}
			}
		} else {
			conn_timeout = 0.0;
		}

		// Decommission check
		if (!m_config->is_decommissioned &&
				bms_if_get_humsens_hum_pcb() >= S_DECOMMISSION_HUM &&
				bms_if_get_humsens_temp_pcb() >= S_DECOMMISSION_TEMP) {
			m_config->is_decommissioned = true;
			hw_psw_switch_off(true);
		}

		// Low voltage check
		if (bms_if_get_v_cell_min() < S_V_CONTACTOR_OFF) {
			hw_psw_switch_off(true);
		}

		// Overcurrent check. Not that safe off is not used to prevent the delay. This can
		// cause damage, but we assume that when a current this high is measured damage has
		// already happened and this is a last effort to reduce it a bit.
		if (fabsf(bms_if_get_i_in_ic()) > S_CONTACTOR_OFF_CURRENT) {
			hw_psw_switch_off(false);
		}

		// Overtemp check. Also set the timeout flags so that the battery has to be disconnected
		// before switching on again.
		if (hw_temp_cell_max() > S_CONTACTOR_OFF_OVERTEMP) {
			hw_psw_switch_off(true);
			m_config->did_timeout_jetpack = true;
			m_config->did_timeout_charger = true;
		}

		chThdSleepMilliseconds(10);
	}
}

static void terminal_psw_set(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		d = atoi(argv[1]);

		if (d) {
			m_terminal_override_psw = true;
			commands_printf("Switching on...");
			int res = hw_psw_switch_on(true);
			if (res == 0) {
				commands_printf("Power is now on");
			} else if (res == -1) {
				commands_printf("Rise rate too fast");
			} else if (res == -2) {
				commands_printf("Precharge timed out");
			} else if (res == -3) {
				commands_printf("Precharge relay timed out");
			}
		} else {
			hw_psw_switch_off(true);
			commands_printf("Power is now off");
			m_terminal_override_psw = false;
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

static void terminal_info(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("V Fuse  : %.2f V", pwr_get_vfuse());
	commands_printf("V Charge: %.2f V", pwr_get_vcharge());
	commands_printf("V Cells : %.2f V", bms_if_get_v_tot());
	commands_printf("V Moist : %.4f V", HW_GET_V_MOIST_SENSE());
	commands_printf("TO_CHG  : %d", m_config->did_timeout_charger);
	commands_printf("TO_JP   : %d", m_config->did_timeout_jetpack);
	commands_printf("DECOMM  : %d", m_config->is_decommissioned);

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
	default:
		commands_printf("Invalid CONN_STATE\n");
		break;
	}
}

static void terminal_get_relay_state(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("Main relay: %s", (HW_RELAY_MAIN_IS_ON() ? "On" : "Off"));
	commands_printf("Precharge relay: %s\n", (HW_RELAY_PCH_IS_ON() ? "On" : "Off"));
}

static void terminal_set_relay_state(int argc, const char **argv) {
	if (argc == 3) {
		commands_printf("rbat_set_relay: %s %s\n", argv[1], argv[2]);

		if (strcmp(argv[1], "pch") == 0) {
			if (strcmp(argv[2], "on") == 0) {
				HW_RELAY_PCH_ON();
				return;
			}
			if (strcmp(argv[2], "off") == 0) {
				HW_RELAY_PCH_OFF();
				return;
			}
		} else 	if (strcmp(argv[1], "main") == 0) {
			if (strcmp(argv[2], "on") == 0) {
				m_terminal_override_psw = true;
				HW_RELAY_MAIN_ON();
				return;
			}
			if (strcmp(argv[2], "off") == 0) {
				m_terminal_override_psw = false;
				HW_RELAY_MAIN_OFF();
				return;
			}
		}
	}

	commands_printf("Invalid arguments");
}