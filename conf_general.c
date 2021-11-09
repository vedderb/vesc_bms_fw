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

#include "conf_general.h"
#include "utils.h"

void conf_general_apply_hw_limits(main_config_t *config) {
#ifndef DISABLE_HW_LIMITS
	utils_truncate_number_int(&config->cell_first_index, 0, HW_CELLS_SERIES);
	utils_truncate_number_int(&config->cell_num, 2, HW_CELLS_SERIES - config->cell_first_index);
#ifdef HW_LIM_MAX_BAL_CHANNEL
	utils_truncate_number_int(&config->max_bal_ch, HW_LIM_MAX_BAL_CHANNEL);
#endif
#ifdef HW_LIM_CHARGE_START
	utils_truncate_number(&config->vc_charge_start, HW_LIM_CHARGE_START);
#endif
#ifdef HW_LIM_CHARGE_MIN
	utils_truncate_number(&config->vc_charge_min, HW_LIM_CHARGE_MIN);
#endif
#ifdef HW_LIM_CHARGE_END
	utils_truncate_number(&config->vc_charge_end, HW_LIM_CHARGE_END);
#endif
#ifdef HW_LIM_T_CHARGE_MIN
	utils_truncate_number(&config->t_charge_min, HW_LIM_T_CHARGE_MIN);
#endif
#ifdef HW_LIM_T_CHARGE_MAX
	utils_truncate_number(&config->t_charge_max, HW_LIM_T_CHARGE_MAX);
#endif
#ifdef HW_LIM_MAX_CHRG_CURR
	utils_truncate_number(&config->max_charge_current, HW_LIM_MAX_CHRG_CURR);
#endif
#ifdef HW_LIM_V_CHARGE_DETECT
	utils_truncate_number(&config->v_charge_detect, HW_LIM_V_CHARGE_DETECT);
#endif
#ifdef HW_LIM_T_CHARGE_MON_EN
	utils_truncate_number_int((int *)&config->t_charge_mon_en, HW_LIM_T_CHARGE_MON_EN);
#endif
#ifdef HW_LIM_T_BAL_LIM_START
	utils_truncate_number(&config->t_bal_lim_end, HW_LIM_T_BAL_LIM_START);
#endif
#ifdef HW_LIM_T_BAL_LIM_END
	utils_truncate_number(&config->t_bal_lim_end, HW_LIM_T_BAL_LIM_END);
#endif
#ifdef HW_LIM_VC_BALANCE_START
	utils_truncate_number(&config->vc_balance_start, HW_LIM_VC_BALANCE_START);
#endif
#ifdef HW_LIM_VC_BALANCE_END
	utils_truncate_number(&config->vc_balance_end, HW_LIM_VC_BALANCE_END);
#endif
#ifdef HW_LIM_BALANCE_MIN
	utils_truncate_number(&config->vc_balance_min, HW_LIM_VC_BALANCE_MIN);
#endif
#ifdef HW_LIM_MAX_BAL_CH
	utils_truncate_number_int(&config->max_bal_ch, HW_LIM_MAX_BAL_CH);
#endif
#ifdef HW_LIM_MAX_BALANCE_CURR
	utils_truncate_number(&config->balance_max_current, HW_LIM_MAX_BALANCE_CURR);
#endif
#ifdef HW_LIM_BALANCE_MODE
	utils_truncate_number_int(config->balance_mode, HW_LIM_BALANCE_MODE);
#endif
#ifdef HW_LIM_DIST_BAL
	utils_truncate_number_int(config->dist_bal, HW_LIM_DIST_BAL);
#endif
#ifdef HW_LIM_ENTER_SLEEP_CURR
	utils_truncate_number(&config->min_current_sleep, HW_LIM_ENTER_SLEEP_CURR);
#endif
#ifdef HW_LIM_SLEEP_CNT
	utils_truncate_number_int(&config->sleep_timeout_reset_ms, HW_LIM_SLEEP_CNT);
#endif
#ifdef HW_LIM_CONTROLLER_ID
	utils_truncate_number_int((int *)&config->controller_id, HW_LIM_CONTROLLER_ID);
#endif
#ifdef HW_LIM_CAN_BAUD_RATE
	utils_truncate_number(&config->can_baud_rate, HW_LIM_CAN_BAUD_RATE);
#endif
#ifdef HW_LIM_STATUS_MSG_RATE_HZ
	utils_truncate_number_int((int *)&config->send_can_status_rate_hz, HW_LIM_STATUS_MSG_RATE_HZ);
#endif
#ifdef HW_LIM_MIN_CURRENT_AH_WH_CNT
	utils_truncate_number(&config->min_current_ah_wh_cnt, HW_LIM_MIN_CURRENT_AH_WH_CNT);
#endif
#ifdef HW_LIM_I_MEASURE_MODE
	utils_truncate_number_int(&config->i_measure_mode, HW_LIM_I_MEASURE_MODE);
#endif
#ifdef HW_LIM_EXT_SHUNT_RES
	utils_truncate_number(&config->ext_shunt_res, HW_LIM_EXT_SHUNT_RES);
#endif
#ifdef HW_LIM_EXT_SHUNT_GAIN
	utils_truncate_number(&config->ext_shunt_gain, HW_LIM_EXT_SHUNT_GAIN);
#endif
#ifdef HW_LIM_EXT_PCH_R_TOP
	utils_truncate_number(&config->ext_pch_r_top, HW_LIM_EXT_PCH_R_TOP);
#endif
#ifdef HW_LIM_EXT_PCH_R_BOT
	utils_truncate_number(&config->ext_pch_r_bot, HW_LIM_EXT_PCH_R_BOT);
#endif
#ifdef HW_LIM_SOC_FILTER_CONST
	utils_truncate_number(&config->soc_filter_const, HW_LIM_SOC_FILTER_CONST);
#endif
#endif
}


