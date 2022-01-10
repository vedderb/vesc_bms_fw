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

#ifndef BMS_IF_H_
#define BMS_IF_H_

#include "conf_general.h"

typedef void (*bms_if_fault_cb) (fault_data *p_data);

// Functions
void bms_if_init(void);
bool bms_if_charge_ok(void);
float bms_if_get_i_in(void);
float bms_if_get_i_in_ic(void);
float bms_if_get_v_cell(int cell);
float bms_if_get_v_cell_min(void);
float bms_if_get_v_cell_max(void);
float bms_if_get_v_tot(void);
float bms_if_get_v_charge(void);
float bms_if_get_temp(int sensor);
float bms_if_get_temp_ic(void);
bool bms_if_is_balancing_cell(int cell);
double bms_if_get_ah_cnt(void);
double bms_if_get_wh_cnt(void);
double bms_if_get_ah_cnt_chg_total(void);
double bms_if_get_wh_cnt_chg_total(void);
double bms_if_get_ah_cnt_dis_total(void);
double bms_if_get_wh_cnt_dis_total(void) ;
bool bms_if_is_charge_allowed(void);
void bms_if_set_charge_allowed(bool allowed);
bool bms_if_is_charging(void);
bool bms_if_is_balancing(void);
void bms_if_set_balance_override(int cell, int override);
void bms_if_reset_counter_ah(void);
void bms_if_reset_counter_wh(void);
void bms_if_force_balance(bool bal_en);
void bms_if_zero_current_offset(void);
float bms_if_get_humsens_hum_pcb(void);
float bms_if_get_humsens_temp_pcb(void);
float bms_if_get_humsens_hum_ext(void);
float bms_if_get_humsens_temp_ext(void);
float bms_if_get_soc(void);
float bms_if_get_soh(void);
void bms_if_sleep(void);
void bms_if_fault_report(bms_fault_code fault);
bms_fault_code bms_if_fault_now(void);
void bms_if_register_fault_cb(const bms_if_fault_cb cb);

#endif /* BMS_IF_H_ */
