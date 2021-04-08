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

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include "ch.h"
#include "hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
	BALANCE_MODE_DISABLED = 0,
	BALANCE_MODE_CHARGING_ONLY,
	BALANCE_MODE_DURING_AND_AFTER_CHARGING,
	BALANCE_MODE_ALWAYS
} BMS_BALANCE_MODE;

typedef enum {
	HW_TYPE_VESC = 0,
	HW_TYPE_VESC_BMS,
	HW_TYPE_CUSTOM_MODULE
} HW_TYPE;

typedef enum {
	I_MEASURE_MODE_BMS = 0,
	I_MEASURE_MODE_VESC
} I_MEASURE_MODE;

typedef enum {
	CAN_BAUD_125K = 0,
	CAN_BAUD_250K,
	CAN_BAUD_500K,
	CAN_BAUD_1M,
	CAN_BAUD_10K,
	CAN_BAUD_20K,
	CAN_BAUD_50K,
	CAN_BAUD_75K
} CAN_BAUD;

typedef struct __attribute__((packed)) {
	// ID if this BMS (e.g. on the CAN-bus)
	uint8_t controller_id;

	// Rate at which status messages are sent on the CAN-bus
	uint32_t send_can_status_rate_hz;

	CAN_BAUD can_baud_rate;

	BMS_BALANCE_MODE balance_mode;

	// Number of cells in series
	int cell_num;

	// Channel of the first cell
	int cell_first_index;

	// Maximum simultaneous balancing channels
	int max_bal_ch;

	// Distributed balancing
	bool dist_bal;

	// Start balancing if cell voltage is this much above the minimum cell voltage
	float vc_balance_start;

	// Stop balancing when cell voltage is this much above the minimum cell voltage
	float vc_balance_end;

	// Start charging when max cell voltage is below this voltage
	float vc_charge_start;

	// End charging when max cell voltage is above this voltage
	float vc_charge_end;

	// Only allow charging if all cells are above this voltage
	float vc_charge_min;

	// Only allow balancing if all cells are above this voltage
	float vc_balance_min;

	// Only allow balancing when the current magnitude is below this value
	float balance_max_current;

	// Current must be above this magnitude for the Ah and Wh couters to run
	float min_current_ah_wh_cnt;

	// Enter sleep mode when the current magnitude is below this value
	float min_current_sleep;

	// Charge port voltage at which a charger is considered plugged in
	float v_charge_detect;

	// Only allow charging when the cell temperature is below this value
	float t_charge_max;

	// Current measurement mode
	I_MEASURE_MODE i_measure_mode;

	// Shunt resistance on external PCB
	float ext_shunt_res;

	// Shunt amplifier gain on external PCB
	float ext_shunt_gain;

	// Precharge output voltage divider top resistor
	float ext_pch_r_top; // TODO

	// Precharge output voltage divider bottom resistor
	float ext_pch_r_bot; // TODO

	// Reset sleep timeout to this value at events that prevent sleeping
	int sleep_timeout_reset_ms;

	// Maximum allowed charging current
	float max_charge_current;

	// Filter constant for SoC filter
	float soc_filter_const;

	// Start limiting the number of balancing channels at this temperature
	float t_bal_lim_start;

	// Disable all balancing channels above this temperature
	float t_bal_lim_end;

	// Only allow charging when the cell temperature is above this value
	float t_charge_min;
} main_config_t;

typedef struct __attribute__((packed)) {
	// Ah counter
	uint32_t ah_cnt_init_flag;
	double ah_cnt;

	// Wh counter
	uint32_t wh_cnt_init_flag;
	double wh_cnt;

	// Current sense offset voltage
	uint32_t ic_i_sens_v_ofs_init_flag;
	float ic_i_sens_v_ofs;

	// ID. Use separate ID from config to retain it even when the
	// config version changes.
	uint32_t controller_id_init_flag;
	uint16_t controller_id;

	// Counter for how many times backup data has been written to flash
	uint32_t conf_flash_write_cnt_init_flag;
	uint32_t conf_flash_write_cnt;

	// Counter to not wait for USB on every boot
	uint32_t usb_cnt_init_flag;
	uint32_t usb_cnt;

	// HW-specific data
	uint32_t hw_config_init_flag;
	uint8_t hw_config[128];

	// Ah counter charge total
	uint32_t ah_cnt_chg_total_init_flag;
	double ah_cnt_chg_total;

	// Wh counter charge total
	uint32_t wh_cnt_chg_total_init_flag;
	double wh_cnt_chg_total;

	// Ah counter discharge total
	uint32_t ah_cnt_dis_total_init_flag;
	double ah_cnt_dis_total;

	// Wh counter discharge total
	uint32_t wh_cnt_dis_total_init_flag;
	double wh_cnt_dis_total;

	// BMS configuration structure
	uint32_t config_init_flag;
	main_config_t config;

	// Pad data to align with flash
	volatile uint32_t pad1;
	volatile uint32_t pad2;
} backup_data;

typedef struct {
	int id;
	systime_t rx_time;
	float rpm;
	float current;
	float duty;
} can_status_msg;

typedef struct {
	int id;
	systime_t rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct {
	int id;
	systime_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct {
	int id;
	systime_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;

typedef struct {
	int id;
	systime_t rx_time;
	float v_in;
	int32_t tacho_value;
} can_status_msg_5;

typedef struct {
	int id;
	systime_t rx_time;
	float v_cell_min;
	float v_cell_max;
	float t_cell_max;
	float soc;
	float soh;
	bool is_charging;
	bool is_balancing;
	bool is_charge_allowed;
} bms_soc_soh_temp_stat;

typedef struct {
	int id;
	systime_t rx_time;
	float v_in;
	float v_out;
	float temp;
	bool is_out_on;
	bool is_pch_on;
	bool is_dsc_on;
} psw_status;

typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_CHARGE_OVERCURRENT,
	FAULT_CODE_CHARGE_OVERTEMP
} bms_fault_code;

// Logged fault data
typedef struct {
	bms_fault_code fault;
	systime_t fault_time;
	float current;
	float current_ic;
	float temp_batt;
	float temp_pcb;
	float temp_ic;
	float v_cell_min;
	float v_cell_max;
} fault_data;

// CAN commands
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS_2,
	CAN_PACKET_STATUS_3,
	CAN_PACKET_STATUS_4,
	CAN_PACKET_PING,
	CAN_PACKET_PONG,
	CAN_PACKET_DETECT_APPLY_ALL_FOC,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
	CAN_PACKET_CONF_CURRENT_LIMITS,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_FOC_ERPMS,
	CAN_PACKET_CONF_STORE_FOC_ERPMS,
	CAN_PACKET_STATUS_5,
	CAN_PACKET_POLL_TS5700N8501_STATUS,
	CAN_PACKET_CONF_BATTERY_CUT,
	CAN_PACKET_CONF_STORE_BATTERY_CUT,
	CAN_PACKET_SHUTDOWN,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12,
	CAN_PACKET_IO_BOARD_DIGITAL_IN,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM,
	CAN_PACKET_BMS_V_TOT,
	CAN_PACKET_BMS_I,
	CAN_PACKET_BMS_AH_WH,
	CAN_PACKET_BMS_V_CELL,
	CAN_PACKET_BMS_BAL,
	CAN_PACKET_BMS_TEMPS,
	CAN_PACKET_BMS_HUM,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT,
	CAN_PACKET_PSW_STAT,
	CAN_PACKET_PSW_SWITCH,
	CAN_PACKET_BMS_HW_DATA_1,
	CAN_PACKET_BMS_HW_DATA_2,
	CAN_PACKET_BMS_HW_DATA_3,
	CAN_PACKET_BMS_HW_DATA_4,
	CAN_PACKET_BMS_HW_DATA_5,
	CAN_PACKET_BMS_AH_WH_CHG_TOTAL,
	CAN_PACKET_BMS_AH_WH_DIS_TOTAL,
} CAN_PACKET_ID;

// Communication commands
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_HANDBRAKE,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA,
	COMM_NRF_START_PAIRING,
	COMM_GPD_SET_FSW,
	COMM_GPD_BUFFER_NOTIFY,
	COMM_GPD_BUFFER_SIZE_LEFT,
	COMM_GPD_FILL_BUFFER,
	COMM_GPD_OUTPUT_SAMPLE,
	COMM_GPD_SET_MODE,
	COMM_GPD_FILL_BUFFER_INT8,
	COMM_GPD_FILL_BUFFER_INT16,
	COMM_GPD_SET_BUFFER_INT_SCALE,
	COMM_GET_VALUES_SETUP,
	COMM_SET_MCCONF_TEMP,
	COMM_SET_MCCONF_TEMP_SETUP,
	COMM_GET_VALUES_SELECTIVE,
	COMM_GET_VALUES_SETUP_SELECTIVE,
	COMM_EXT_NRF_PRESENT,
	COMM_EXT_NRF_ESB_SET_CH_ADDR,
	COMM_EXT_NRF_ESB_SEND_DATA,
	COMM_EXT_NRF_ESB_RX_DATA,
	COMM_EXT_NRF_SET_ENABLED,
	COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
	COMM_DETECT_APPLY_ALL_FOC,
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
	COMM_ERASE_NEW_APP_ALL_CAN,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN,
	COMM_PING_CAN,
	COMM_APP_DISABLE_OUTPUT,
	COMM_TERMINAL_CMD_SYNC,
	COMM_GET_IMU_DATA,
	COMM_BM_CONNECT,
	COMM_BM_ERASE_FLASH_ALL,
	COMM_BM_WRITE_FLASH,
	COMM_BM_REBOOT,
	COMM_BM_DISCONNECT,
	COMM_BM_MAP_PINS_DEFAULT,
	COMM_BM_MAP_PINS_NRF5X,
	COMM_ERASE_BOOTLOADER,
	COMM_ERASE_BOOTLOADER_ALL_CAN,
	COMM_PLOT_INIT,
	COMM_PLOT_DATA,
	COMM_PLOT_ADD_GRAPH,
	COMM_PLOT_SET_GRAPH,
	COMM_GET_DECODED_BALANCE,
	COMM_BM_MEM_READ,
	COMM_WRITE_NEW_APP_DATA_LZO,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO,
	COMM_BM_WRITE_FLASH_LZO,
	COMM_SET_CURRENT_REL,
	COMM_CAN_FWD_FRAME,
	COMM_SET_BATTERY_CUT,
	COMM_SET_BLE_NAME,
	COMM_SET_BLE_PIN,
	COMM_SET_CAN_MODE,
	COMM_GET_IMU_CALIBRATION,
	COMM_GET_MCCONF_TEMP,

	// Custom configuration for hardware
	COMM_GET_CUSTOM_CONFIG_XML,
	COMM_GET_CUSTOM_CONFIG,
	COMM_GET_CUSTOM_CONFIG_DEFAULT,
	COMM_SET_CUSTOM_CONFIG,

	// BMS commands
	COMM_BMS_GET_VALUES,
	COMM_BMS_SET_CHARGE_ALLOWED,
	COMM_BMS_SET_BALANCE_OVERRIDE,
	COMM_BMS_RESET_COUNTERS,
	COMM_BMS_FORCE_BALANCE,
	COMM_BMS_ZERO_CURRENT_OFFSET,

	// FW updates commands for different HW types
	COMM_JUMP_TO_BOOTLOADER_HW,
	COMM_ERASE_NEW_APP_HW,
	COMM_WRITE_NEW_APP_DATA_HW,
	COMM_ERASE_BOOTLOADER_HW,
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW,
	COMM_ERASE_NEW_APP_ALL_CAN_HW,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW,
	COMM_ERASE_BOOTLOADER_ALL_CAN_HW,

	COMM_SET_ODOMETER,

	// Power switch commands
	COMM_PSW_GET_STATUS,
	COMM_PSW_SWITCH,

	COMM_BMS_FWD_CAN_RX,
	COMM_BMS_HW_DATA,
} COMM_PACKET_ID;

#endif /* DATATYPES_H_ */
