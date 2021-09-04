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

#include "main.h"
#include "bms_if.h"
#include "pwr.h"
#include "utils.h"
#include "hdc1080.h"
#include "comm_can.h"
#include "timeout.h"
#include "sleep.h"
#include "terminal.h"
#include "flash_helper.h"
#include "hw.h"

#include <math.h>

// Settings
#define I_IN_FILTER_CONST			0.006
#define I_IN_FILTER_CONST_IC		0.006
#define IC_ISENSE_I_GAIN_CORR		0.997 // This gain correction is a hack and should probably be set in config or in hw config

// Private variables
static volatile float m_i_in_filter = 0.0;
static volatile float m_i_in_filter_ic = 0.0;
static volatile bool m_charge_allowed = true;
static volatile bool m_is_charging = false;
static volatile bool m_is_balancing = false;
static volatile float m_voltage_cell_min = 0.0;
static volatile float m_voltage_cell_max = 0.0;
static volatile int m_balance_override[HW_CELLS_SERIES] = {0};
static volatile bool m_bal_ok = false;
static volatile bool m_was_charge_overcurrent = false;
static float m_soc_filtered = 0.0;
static bool m_soc_filter_init_done = false;

// Threads
static THD_WORKING_AREA(if_thd_wa, 2048);
static THD_FUNCTION(if_thd, p);
static THD_WORKING_AREA(charge_thd_wa, 2048);
static THD_FUNCTION(charge_thd, p);
static THD_WORKING_AREA(balance_thd_wa, 2048);
static THD_FUNCTION(balance_thd, p);

void bms_if_init(void) {
	chThdCreateStatic(if_thd_wa, sizeof(if_thd_wa), NORMALPRIO, if_thd, 0);
	chThdCreateStatic(charge_thd_wa, sizeof(charge_thd_wa), NORMALPRIO, charge_thd, 0);
	chThdCreateStatic(balance_thd_wa, sizeof(balance_thd_wa), NORMALPRIO, balance_thd, 0);
}

static bool charge_ok(void) {

	float max = m_is_charging ? backup.config.vc_charge_end : backup.config.vc_charge_start;
	return HW_GET_V_CHARGE() > backup.config.v_charge_detect &&
			m_voltage_cell_min > backup.config.vc_charge_min &&
			m_voltage_cell_max < max &&
			HW_TEMP_CELLS_MAX() < backup.config.t_charge_max &&
			HW_TEMP_CELLS_MAX() > backup.config.t_charge_min;
}

static THD_FUNCTION(charge_thd, p) {
	(void)p;
	chRegSetThreadName("Charge");

	int no_charge_cnt = 0;

	for (;;) {
		if (m_is_charging && HW_TEMP_CELLS_MAX() >= backup.config.t_charge_max) {
			bms_if_fault_report(FAULT_CODE_CHARGE_OVERTEMP);
		}

		if (charge_ok() && m_charge_allowed && !m_was_charge_overcurrent) {
			if (!m_is_charging) {
				sleep_reset();
				chThdSleepMilliseconds(2000);
				if (charge_ok()) {
					m_is_charging = true;
					CHARGE_ENABLE();
				}
			}
		} else {
			m_is_charging = false;
			CHARGE_DISABLE();
		}

		chThdSleepMilliseconds(10);

		if (m_i_in_filter > -0.5 && m_is_charging && !HW_CHARGER_DETECTED()) {
			no_charge_cnt++;

			if (no_charge_cnt > 100) {
				no_charge_cnt = 0;
				m_is_charging = false;
				CHARGE_DISABLE();
				chThdSleepMilliseconds(5000);
			}
		} else {
			no_charge_cnt = 0;
		}

		if (m_is_charging) {
			if (fabsf(m_i_in_filter) > backup.config.max_charge_current) {
				m_was_charge_overcurrent = true;
				m_is_charging = false;
				CHARGE_DISABLE();
				bms_if_fault_report(FAULT_CODE_CHARGE_OVERCURRENT);
			}

			sleep_reset();
		}

		// Charger must be disconnected and reconnected on charge overcurrent events
		if (m_was_charge_overcurrent && HW_GET_V_CHARGE() < backup.config.v_charge_detect) {
			m_was_charge_overcurrent = false;
		}

		// Store data and counters to flash every time charger is disconnected
		static bool charger_connected_last = false;
		if (charger_connected_last && HW_GET_V_CHARGE() < backup.config.v_charge_detect) {
			flash_helper_store_backup_data();
		}
		charger_connected_last = HW_GET_V_CHARGE() > backup.config.v_charge_detect;
	}
}

static THD_FUNCTION(balance_thd, p) {
	(void)p;
	chRegSetThreadName("Balance");

	systime_t last_charge_time = 0.0;

	while (!chThdShouldTerminateX()) {
		float v_min = 10.0;
		float v_max = 0.0;

		// Allow some time to start balancing after unplugging the charger. This is useful if it is unplugged
		// while the current was so high that balancing was prevented.
		float time_since_charge = 1000.0;
		if (UTILS_AGE_S(0) > 1.0) {
			if (m_is_charging) {
				last_charge_time = chVTGetSystemTimeX();
			}

			time_since_charge = UTILS_AGE_S(last_charge_time);
		}

		switch (backup.config.balance_mode) {
		case BALANCE_MODE_DISABLED:
			m_bal_ok = false;
			break;

		case BALANCE_MODE_CHARGING_ONLY:
			if (m_is_charging) {
				m_bal_ok = true;
			} else {
				m_bal_ok = false;
			}
			break;

		case BALANCE_MODE_DURING_AND_AFTER_CHARGING:
			if (time_since_charge < 2.0) {
				m_bal_ok = true;
			}
			break;

		case BALANCE_MODE_ALWAYS:
			m_bal_ok = true;
			break;
		}

		for (int i = backup.config.cell_first_index;i <
		(backup.config.cell_num + backup.config.cell_first_index);i++) {
			if (HW_LAST_CELL_VOLTAGE(i) > v_max) {
				v_max = HW_LAST_CELL_VOLTAGE(i);
			}
			if (HW_LAST_CELL_VOLTAGE(i) < v_min) {
				v_min = HW_LAST_CELL_VOLTAGE(i);
			}
		}

		m_voltage_cell_min = v_min;
		m_voltage_cell_max = v_max;
		m_is_balancing = false;

		bool is_balance_override = false;
		int bal_ch = 0;
		for (int i = backup.config.cell_first_index;i <
		(backup.config.cell_num + backup.config.cell_first_index);i++) {
			for (int i = backup.config.cell_first_index;i <
			(backup.config.cell_num + backup.config.cell_first_index);i++) {
				if (m_balance_override[i] == 1) {
					is_balance_override = true;
					HW_SET_DSC(i, false);
				} else if (m_balance_override[i] == 2) {
					is_balance_override = true;
					HW_SET_DSC(i, true);
					bal_ch++;
					m_is_balancing = true;
				}
			}
		}

		if (backup.config.dist_bal) {
			bms_soc_soh_temp_stat *msg = comm_can_get_bms_stat_v_cell_min();

			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 10.0 && msg->v_cell_min < v_min) {
				v_min = msg->v_cell_min;
			}
		}

		if (v_min > backup.config.vc_balance_min &&
				m_bal_ok &&
				!is_balance_override &&
				fabsf(bms_if_get_i_in_ic()) < backup.config.balance_max_current) {

			bal_ch = 0;

			for (int i = backup.config.cell_first_index;i <
			(backup.config.cell_num + backup.config.cell_first_index);i++) {
				float limit = HW_GET_DSC(i) ? backup.config.vc_balance_end : backup.config.vc_balance_start;
				limit += v_min;
				
				if (HW_LAST_CELL_VOLTAGE(i) >= limit) {
					HW_SET_DSC(i, true);
					bal_ch++;
					m_is_balancing = true;
				} else {
					HW_SET_DSC(i, false);
				}
			}
		}

		float t_bal = HW_GET_BAL_TEMP();
		float t_bal_start = backup.config.t_bal_lim_start;
		float t_bal_end = backup.config.t_bal_lim_end;
		int bal_ch_max = backup.config.max_bal_ch;

		if (t_bal > (t_bal_end - 0.5)) {
			bal_ch_max = 0;
		} else if (t_bal > t_bal_start) {
			bal_ch_max = utils_map_int(t_bal, t_bal_start, t_bal_end, bal_ch_max, 0);
		}

		// Limit number of simultaneous balancing channels by disabling
		// balancing on the cells with the highest voltage.
		while (bal_ch > bal_ch_max) {
			float v_min = 100.0;
			int v_min_cell = 0;
			for (int i = backup.config.cell_first_index;i <
			(backup.config.cell_num + backup.config.cell_first_index);i++) {
				if (HW_LAST_CELL_VOLTAGE(i) < v_min && HW_GET_DSC(i)) {
					v_min = HW_LAST_CELL_VOLTAGE(i);
					v_min_cell = i;
				}
			}
			HW_SET_DSC(v_min_cell, false);
			bal_ch--;
		}

		if (m_is_balancing) {
			sleep_reset();
		} else {
			m_bal_ok = false;

			for (int i = 0;i < HW_CELLS_SERIES;i++) {
				HW_SET_DSC(i, false);
			}
		}

		timeout_feed_WDT(THREAD_BAL);
		chThdSleepMilliseconds(50);
	}
}

static THD_FUNCTION(if_thd, p) {
	(void)p;
	chRegSetThreadName("IfThd");

	systime_t tick_last = chVTGetSystemTimeX();

	chThdSleepMilliseconds(2000);

	for(;;) {
		float i_bms_ic = 0.1;//-(ltc_last_gpio_voltage(LTC_GPIO_CURR_MON) - 1.65 + backup.ic_i_sens_v_ofs) *
					//(1.0 / HW_SHUNT_AMP_GAIN) * (1.0 / backup.config.ext_shunt_res) * IC_ISENSE_I_GAIN_CORR;
		float i_adc = pwr_get_iin();

		if (backup.config.i_measure_mode == I_MEASURE_MODE_VESC) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg_4 *msg = comm_can_get_status_msg_4_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 2.0) {
					i_bms_ic += msg->current_in;
				}
			}
		}

		UTILS_LP_FAST(m_i_in_filter, i_adc, I_IN_FILTER_CONST);
		UTILS_LP_FAST(m_i_in_filter_ic, i_bms_ic, I_IN_FILTER_CONST_IC);

		double time = (double)TIME_I2US(chVTTimeElapsedSinceX(tick_last)) *
				(double)1.0e-6 * ((double)1.0 / (double)60.0) * ((double)1.0 / (double)60.0);
		tick_last = chVTGetSystemTimeX();

		if (fabsf(m_i_in_filter_ic) > backup.config.min_current_ah_wh_cnt) {
			double d_ah = (double)bms_if_get_i_in_ic() * time;
			double d_wh = (double)bms_if_get_i_in_ic() * (double)bms_if_get_v_tot() * time;

			backup.ah_cnt += d_ah;
			backup.wh_cnt += d_wh;

			if (m_i_in_filter_ic > 0.0) {
				backup.ah_cnt_dis_total += d_ah;
				backup.wh_cnt_dis_total += d_wh;
			} else {
				backup.ah_cnt_chg_total -= d_ah;
				backup.wh_cnt_chg_total -= d_wh;
			}
		}

		if (fabsf(bms_if_get_i_in_ic()) > backup.config.min_current_sleep) {
			sleep_reset();
		}

		float soc_now = utils_batt_liion_norm_v_to_capacity(utils_map(m_voltage_cell_min, 3.2, 4.2, 0.0, 1.0));
		if (!m_soc_filter_init_done) {
			m_soc_filter_init_done = true;
			m_soc_filtered = soc_now;
		} else {
			UTILS_LP_FAST(m_soc_filtered, soc_now, backup.config.soc_filter_const);
		}

		// RED LED
		if (m_was_charge_overcurrent) {
			// Prevent sleeping to keep the charge input disabled (as long as the battery does not run too low)
			if (bms_if_get_soc() > 0.3) {
				sleep_reset();
			}

			// Blink out fault on red LED
			static int blink = 0;
			blink++;
			if (blink > 200) {
				blink = 0;
			}

			if (blink < 100) {
				LED_ON(LINE_LED_RED);
			} else {
				LED_OFF(LINE_LED_RED);
			}
		} else {
			if (m_is_balancing) {
				LED_ON(LINE_LED_RED);
			} else {
				LED_OFF(LINE_LED_RED);
			}
		}

		chThdSleepMilliseconds(1);
	}
}

float bms_if_get_i_in(void) {
	return m_i_in_filter;
}

float bms_if_get_i_in_ic(void) {
#ifndef	AFE
	return m_i_in_filter_ic;
#endif
#ifdef AFE
	return get_current();
#endif
}

float bms_if_get_v_cell(int cell) {
	return HW_LAST_CELL_VOLTAGE(cell);
}

float bms_if_get_v_cell_min(void) {
	return m_voltage_cell_min;
}

float bms_if_get_v_cell_max(void) {
	return m_voltage_cell_max;
}

float bms_if_get_v_tot(void) {
	float ret = 0.0;
	for (int i = backup.config.cell_first_index;i <
	(backup.config.cell_num + backup.config.cell_first_index);i++) {
		ret += bms_if_get_v_cell(i);
	}
	return ret;
}

float bms_if_get_v_charge(void) {
	return HW_GET_V_CHARGE();
}

float bms_if_get_temp(int sensor) {
	return HW_GET_TEMP(sensor);
}

float bms_if_get_temp_ic(void) {
	return ltc_last_temp();
}

bool bms_if_is_balancing_cell(int cell) {
	return HW_GET_DSC(cell);
}

double bms_if_get_ah_cnt(void) {
	return backup.ah_cnt;
}

double bms_if_get_wh_cnt(void) {
	return backup.wh_cnt;
}

double bms_if_get_ah_cnt_chg_total(void) {
	return backup.ah_cnt_chg_total;
}

double bms_if_get_wh_cnt_chg_total(void) {
	return backup.wh_cnt_chg_total;
}

double bms_if_get_ah_cnt_dis_total(void) {
	return backup.ah_cnt_dis_total;
}

double bms_if_get_wh_cnt_dis_total(void) {
	return backup.wh_cnt_dis_total;
}

bool bms_if_is_charge_allowed(void) {
	return m_charge_allowed;
}

void bms_if_set_charge_allowed(bool allowed) {
	m_charge_allowed = allowed;
}

bool bms_if_is_charging(void) {
	return m_is_charging;
}

bool bms_if_is_balancing(void) {
	return m_is_balancing;
}

void bms_if_set_balance_override(int cell, int override) {
	if (cell >= 0 && cell < HW_CELLS_SERIES) {
		m_balance_override[cell] = override;
	}
}

void bms_if_reset_counter_ah(void) {
	backup.ah_cnt = 0.0;
}

void bms_if_reset_counter_wh(void) {
	backup.wh_cnt = 0.0;
}

void bms_if_force_balance(bool bal_en) {
	if (bal_en) {
		m_bal_ok = true;
	} else {
		m_bal_ok = false;
		for (int i = 0;i < HW_CELLS_SERIES;i++) {
			HW_SET_DSC(i, false);
		}
	}
}

void bms_if_zero_current_offset(void) {
	float ofs_avg = 0.0;
	float samples = 0.0;

	for (int i = 0;i < 20;i++) {
		ofs_avg -= ltc_last_gpio_voltage(LTC_GPIO_CURR_MON) - 1.65;
		samples += 1.0;
		chThdSleepMilliseconds(100);
	}

	backup.ic_i_sens_v_ofs = ofs_avg / samples;
	flash_helper_store_backup_data();
}

float bms_if_get_humitidy(void) {
	return hdc1080_get_hum();
}

float bms_if_get_humidity_sensor_temp(void) {
	return hdc1080_get_temp();
}

float bms_if_get_soc(void) {
	// TODO: Estimate and compensate for ESR
	if (HW_SOC_OVERRIDE() >= 0.0) {
		return HW_SOC_OVERRIDE();
	} else {
		return m_soc_filtered;
	}
}

float bms_if_get_soh(void) {
	// TODO!
	return 1.0;
}

void bms_if_sleep(void) {
	ltc_sleep();
}

void bms_if_fault_report(bms_fault_code fault) {
	fault_data f;

	f.fault = fault;
	f.fault_time = chVTGetSystemTimeX();
	f.current = bms_if_get_i_in();
	f.current_ic = bms_if_get_i_in_ic();
	f.temp_batt = HW_TEMP_CELLS_MAX();
	f.temp_pcb = bms_if_get_temp(0);
	f.temp_ic = bms_if_get_temp_ic();
	f.v_cell_min = bms_if_get_v_cell_min();
	f.v_cell_max = bms_if_get_v_cell_max();

	terminal_add_fault_data(&f);
}

bms_fault_code bms_if_fault_now(void) {
	bms_fault_code res = FAULT_CODE_NONE;

	if (m_was_charge_overcurrent) {
		res = FAULT_CODE_CHARGE_OVERCURRENT;
	}

	return res;
}
