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
#include "comm_can.h"
#include "crc.h"
#include "packet.h"
#include "commands.h"
#include "buffer.h"
#include "bms_if.h"
#include "utils.h"
#include "timeout.h"
#include "sleep.h"

#include <string.h>

// Settings
#define RX_FRAMES_SIZE				100
#define RX_BUFFER_SIZE				PACKET_MAX_PL_LEN

// Private variables
static can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_2 stat_msgs_2[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_3 stat_msgs_3[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_4 stat_msgs_4[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_5 stat_msgs_5[CAN_STATUS_MSGS_TO_STORE];
static bms_soc_soh_temp_stat bms_stat_msgs[CAN_BMS_STATUS_MSGS_TO_STORE];
static bms_soc_soh_temp_stat bms_stat_v_cell_min;
static psw_status psw_stat[CAN_STATUS_MSGS_TO_STORE];

static mutex_t can_mtx;
static mutex_t can_rx_mtx;
static CANRxFrame rx_frames[RX_FRAMES_SIZE];
static int rx_frame_read;
static int rx_frame_write;
static thread_t *process_tp = 0;
static thread_t *ping_tp = 0;
static volatile HW_TYPE ping_hw_last = HW_TYPE_VESC;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static unsigned int rx_buffer_last_id;

// Threads
static THD_WORKING_AREA(cancom_read_thread_wa, 512);
static THD_WORKING_AREA(cancom_process_thread_wa, 4096);
static THD_WORKING_AREA(cancom_status_thread_wa, 512);
static THD_FUNCTION(cancom_read_thread, arg);
static THD_FUNCTION(cancom_process_thread, arg);
static THD_FUNCTION(cancom_status_thread, arg);

// Private functions
static void set_timing(int brp, int ts1, int ts2);
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void decode_msg(uint32_t eid, uint8_t *data8, int len, bool is_replaced);

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static CANConfig cancfg = {
		CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
		CAN_BTR_SJW(3) | CAN_BTR_TS2(3) |
		CAN_BTR_TS1(14) | CAN_BTR_BRP(7)
};

// Function pointers
static void(*sid_callback)(uint32_t id, uint8_t *data, uint8_t len) = 0;

void comm_can_init(void) {
	chMtxObjectInit(&can_mtx);
	chMtxObjectInit(&can_rx_mtx);

	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		stat_msgs[i].id = -1;
		stat_msgs_2[i].id = -1;
		stat_msgs_3[i].id = -1;
		stat_msgs_4[i].id = -1;
		stat_msgs_5[i].id = -1;

		psw_stat[i].id = -1;
	}

	bms_stat_v_cell_min.id = -1;

	for (int i = 0;i < CAN_BMS_STATUS_MSGS_TO_STORE;i++) {
		bms_stat_msgs[i].id = -1;
	}

	palSetLineMode(LINE_CAN_RX, PAL_MODE_ALTERNATE(HW_CAN_AF));
	palSetLineMode(LINE_CAN_TX, PAL_MODE_ALTERNATE(HW_CAN_AF));

	canStart(&HW_CAN_DEV, &cancfg);

	chThdCreateStatic(cancom_read_thread_wa, sizeof(cancom_read_thread_wa), NORMALPRIO + 1,
			cancom_read_thread, NULL);
	chThdCreateStatic(cancom_process_thread_wa, sizeof(cancom_process_thread_wa), NORMALPRIO,
			cancom_process_thread, NULL);
	chThdCreateStatic(cancom_status_thread_wa, sizeof(cancom_status_thread_wa), NORMALPRIO,
			cancom_status_thread, NULL);
}

void comm_can_set_baud(CAN_BAUD baud) {
	switch (baud) {
	case CAN_BAUD_125K:	set_timing(31, 14, 3); break;
	case CAN_BAUD_250K:	set_timing(15, 14, 3); break;
	case CAN_BAUD_500K:	set_timing(7, 14, 3); break;
	case CAN_BAUD_1M:	set_timing(3, 14, 3); break;
	case CAN_BAUD_10K:	set_timing(399, 14, 3); break;
	case CAN_BAUD_20K:	set_timing(199, 14, 3); break;
	case CAN_BAUD_50K:	set_timing(79, 14, 3); break;
	case CAN_BAUD_75K:	set_timing(118, 5, 1); break; // 74.697...
	default: break;
	}
}

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (len > 8) {
		len = 8;
	}

	CANTxFrame txmsg;
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.EID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	chMtxLock(&can_mtx);
	canTransmit(&HW_CAN_DEV, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(5));
	chMtxUnlock(&can_mtx);
}

void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (len > 8) {
		len = 8;
	}

	CANTxFrame txmsg;
	txmsg.IDE = CAN_IDE_STD;
	txmsg.SID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	chMtxLock(&can_mtx);
	canTransmit(&HW_CAN_DEV, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(5));
	chMtxUnlock(&can_mtx);
}

void comm_can_set_sid_rx_callback(void (*p_func)(uint32_t id, uint8_t *data, uint8_t len)) {
	sid_callback = p_func;
}

void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = backup.config.controller_id;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;

		comm_can_transmit_eid(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = backup.config.controller_id;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

		comm_can_transmit_eid(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
	}
}

can_status_msg *comm_can_get_status_msg_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs[index];
	} else {
		return 0;
	}
}

can_status_msg *comm_can_get_status_msg_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs[i].id == id) {
			return &stat_msgs[i];
		}
	}

	return 0;
}

can_status_msg_2 *comm_can_get_status_msg_2_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_2[index];
	} else {
		return 0;
	}
}

can_status_msg_2 *comm_can_get_status_msg_2_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_2[i].id == id) {
			return &stat_msgs_2[i];
		}
	}

	return 0;
}

can_status_msg_3 *comm_can_get_status_msg_3_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_3[index];
	} else {
		return 0;
	}
}

can_status_msg_3 *comm_can_get_status_msg_3_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_3[i].id == id) {
			return &stat_msgs_3[i];
		}
	}

	return 0;
}

can_status_msg_4 *comm_can_get_status_msg_4_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_4[index];
	} else {
		return 0;
	}
}

can_status_msg_4 *comm_can_get_status_msg_4_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_4[i].id == id) {
			return &stat_msgs_4[i];
		}
	}

	return 0;
}

can_status_msg_5 *comm_can_get_status_msg_5_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_5[index];
	} else {
		return 0;
	}
}

can_status_msg_5 *comm_can_get_status_msg_5_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_5[i].id == id) {
			return &stat_msgs_5[i];
		}
	}

	return 0;
}

bms_soc_soh_temp_stat *comm_can_get_bms_soc_soh_temp_stat_index(int index) {
	if (index < CAN_BMS_STATUS_MSGS_TO_STORE) {
		return &bms_stat_msgs[index];
	} else {
		return 0;
	}
}

bms_soc_soh_temp_stat *comm_can_get_bms_soc_soh_temp_stat_id(int id) {
	for (int i = 0;i < CAN_BMS_STATUS_MSGS_TO_STORE;i++) {
		if (bms_stat_msgs[i].id == id) {
			return &bms_stat_msgs[i];
		}
	}

	return 0;
}

bms_soc_soh_temp_stat *comm_can_get_bms_stat_v_cell_min(void) {
	return &bms_stat_v_cell_min;
}

psw_status *comm_can_get_psw_status_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &psw_stat[index];
	} else {
		return 0;
	}
}

psw_status *comm_can_get_psw_status_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (psw_stat[i].id == id) {
			return &psw_stat[i];
		}
	}

	return 0;
}

void comm_can_psw_switch(int id, bool is_on, bool plot) {
	int32_t send_index = 0;
	uint8_t buffer[8];

	buffer[send_index++] = is_on ? 1 : 0;
	buffer[send_index++] = plot ? 1 : 0;

	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_PSW_SWITCH << 8),
			buffer, send_index);
}

/**
 * Check if a VESC on the CAN-bus responds.
 *
 * @param controller_id
 * The ID of the VESC.
 *
 * @param hw_type
 * The hardware type of the CAN device.
 *
 * @return
 * True for success, false otherwise.
 */
bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type) {
	ping_tp = chThdGetSelfX();
	chEvtGetAndClearEvents(ALL_EVENTS);

	uint8_t buffer[1];
	buffer[0] = backup.config.controller_id;
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_PING << 8), buffer, 1);

	int ret = chEvtWaitAnyTimeout(1 << 29, TIME_MS2I(10));
	ping_tp = 0;

	if (ret != 0) {
		if (hw_type) {
			*hw_type = ping_hw_last;
		}
	}

	return ret != 0;
}

static THD_FUNCTION(cancom_read_thread, arg) {
	(void)arg;
	chRegSetThreadName("CAN read");

	event_listener_t el;
	CANRxFrame rxmsg;

	chEvtRegister(&HW_CAN_DEV.rxfull_event, &el, 0);

	while(!chThdShouldTerminateX()) {
		timeout_feed_WDT(THREAD_CANBUS);

		chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		msg_t result = canReceive(&HW_CAN_DEV, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);

		while (result == MSG_OK) {
			chMtxLock(&can_rx_mtx);
			rx_frames[rx_frame_write++] = rxmsg;
			if (rx_frame_write == RX_FRAMES_SIZE) {
				rx_frame_write = 0;
			}
			chMtxUnlock(&can_rx_mtx);

			chEvtSignal(process_tp, (eventmask_t) 1);

			result = canReceive(&HW_CAN_DEV, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);
		}
	}

	chEvtUnregister(&HW_CAN_DEV.rxfull_event, &el);
}

static THD_FUNCTION(cancom_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("CAN process");
	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		for(;;) {
			chMtxLock(&can_rx_mtx);
			if (rx_frame_read != rx_frame_write) {
				CANRxFrame rxmsg = rx_frames[rx_frame_read++];
				if (rx_frame_read == RX_FRAMES_SIZE) {
					rx_frame_read = 0;
				}
				chMtxUnlock(&can_rx_mtx);

				if (rxmsg.IDE == CAN_IDE_EXT) {
					decode_msg(rxmsg.EID, rxmsg.data8, rxmsg.DLC, false);
				} else {
					if (sid_callback) {
						sid_callback(rxmsg.SID, rxmsg.data8, rxmsg.DLC);
					}
				}
			} else {
				chMtxUnlock(&can_rx_mtx);
				break;
			}
		}
	}
}

static THD_FUNCTION(cancom_status_thread, arg) {
	int32_t send_index;
	uint8_t buffer[8];
	(void)arg;

	chRegSetThreadName("CAN status");

	memset(buffer, 0, sizeof(buffer));
	buffer[0] = backup.config.controller_id;
	/* Transmit bms boot frame to notify the nodes on the bus that the bms has booted */
	comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_BOOT << 8), buffer, 1);
	for(;;) {
		send_index = 0;
		buffer_append_float32_auto(buffer, bms_if_get_v_tot(), &send_index);
		buffer_append_float32_auto(buffer, bms_if_get_v_charge(), &send_index);
		comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_V_TOT << 8), buffer, send_index);

		send_index = 0;
		buffer_append_float32_auto(buffer, bms_if_get_i_in(), &send_index);
		buffer_append_float32_auto(buffer, bms_if_get_i_in_ic(), &send_index);
		comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_I << 8), buffer, send_index);

		send_index = 0;
		buffer_append_float32_auto(buffer, bms_if_get_ah_cnt(), &send_index);
		buffer_append_float32_auto(buffer, bms_if_get_wh_cnt(), &send_index);
		comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_AH_WH << 8), buffer, send_index);

		int cell_now = backup.config.cell_first_index;
		int cell_max = (backup.config.cell_first_index + backup.config.cell_num);
		while (cell_now < cell_max) {
			send_index = 0;
			buffer[send_index++] = cell_now - backup.config.cell_first_index;
			buffer[send_index++] = backup.config.cell_num;
			if (cell_now < cell_max) {
				buffer_append_float16(buffer, bms_if_get_v_cell(cell_now++), 1e3, &send_index);
			}
			if (cell_now < cell_max) {
				buffer_append_float16(buffer, bms_if_get_v_cell(cell_now++), 1e3, &send_index);
			}
			if (cell_now < cell_max) {
				buffer_append_float16(buffer, bms_if_get_v_cell(cell_now++), 1e3, &send_index);
			}
			comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_V_CELL << 8), buffer, send_index);
		}

#if HW_CELLS_SERIES > 56
#error "Add support for more cells"
#endif
		send_index = 0;
		buffer[send_index++] = backup.config.cell_num;
		uint64_t bal_state = 0;
		for (int i = backup.config.cell_first_index;i < cell_max;i++) {
			bal_state |= (uint64_t)bms_if_is_balancing_cell(i) << i;
		}
		buffer[send_index++] = (bal_state >> 48) & 0xFF;
		buffer[send_index++] = (bal_state >> 40) & 0xFF;
		buffer[send_index++] = (bal_state >> 32) & 0xFF;
		buffer[send_index++] = (bal_state >> 24) & 0xFF;
		buffer[send_index++] = (bal_state >> 16) & 0xFF;
		buffer[send_index++] = (bal_state >> 8) & 0xFF;
		buffer[send_index++] = (bal_state >> 0) & 0xFF;
		comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_BAL << 8), buffer, send_index);

		int temp_now = 0;
		while (temp_now < HW_ADC_TEMP_SENSORS) {
			send_index = 0;
			buffer[send_index++] = temp_now;
			buffer[send_index++] = HW_ADC_TEMP_SENSORS;
			if (temp_now < HW_ADC_TEMP_SENSORS) {
				buffer_append_float16(buffer, bms_if_get_temp(temp_now++), 1e2, &send_index);
			}
			if (temp_now < HW_ADC_TEMP_SENSORS) {
				buffer_append_float16(buffer, bms_if_get_temp(temp_now++), 1e2, &send_index);
			}
			if (temp_now < HW_ADC_TEMP_SENSORS) {
				buffer_append_float16(buffer, bms_if_get_temp(temp_now++), 1e2, &send_index);
			}
			comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_TEMPS << 8), buffer, send_index);
		}

		send_index = 0;
		buffer_append_float16(buffer, bms_if_get_humsens_temp_pcb(), 1e2, &send_index);
		buffer_append_float16(buffer, bms_if_get_humsens_hum_pcb(), 1e2, &send_index);
		buffer_append_float16(buffer, bms_if_get_temp_ic(), 1e2, &send_index); // Put IC temp here instead of making mew msg
		comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_HUM << 8), buffer, send_index);

		/*
		 * CAN_PACKET_BMS_SOC_SOH_TEMP_STAT
		 *
		 * b[0] - b[1]: V_CELL_MIN (mV)
		 * b[2] - b[3]: V_CELL_MAX (mV)
		 * b[4]: SoC (0 - 255)
		 * b[5]: SoH (0 - 255)
		 * b[6]: T_CELL_MAX (-128 to +127 degC)
		 * b[7]: State bitfield:
		 * [B7      B6      B5      B4      B3      B2      B1      B0      ]
		 * [RSV     RSV     RSV     RSV     RSV     CHG_OK  IS_BAL  IS_CHG  ]
		 */
		send_index = 0;
		buffer_append_float16(buffer, bms_if_get_v_cell_min(), 1e3, &send_index);
		buffer_append_float16(buffer, bms_if_get_v_cell_max(), 1e3, &send_index);
		buffer[send_index++] = (uint8_t)(bms_if_get_soc() * 255.0);
		buffer[send_index++] = (uint8_t)(bms_if_get_soh() * 255.0);
		buffer[send_index++] = (int8_t)HW_TEMP_CELLS_MAX();
		buffer[send_index++] =
				((bms_if_is_charging() ? 1 : 0) << 0) |
				((bms_if_is_balancing() ? 1 : 0) << 1) |
				((bms_if_is_charge_allowed() ? 1 : 0) << 2);
		comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_SOC_SOH_TEMP_STAT << 8), buffer, send_index);

		send_index = 0;
		buffer_append_float32_auto(buffer, bms_if_get_ah_cnt_chg_total(), &send_index);
		buffer_append_float32_auto(buffer, bms_if_get_wh_cnt_chg_total(), &send_index);
		comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_AH_WH_CHG_TOTAL << 8), buffer, send_index);

		send_index = 0;
		buffer_append_float32_auto(buffer, bms_if_get_ah_cnt_dis_total(), &send_index);
		buffer_append_float32_auto(buffer, bms_if_get_wh_cnt_dis_total(), &send_index);
		comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_BMS_AH_WH_DIS_TOTAL << 8), buffer, send_index);

		HW_SEND_CAN_DATA();

		while (backup.config.send_can_status_rate_hz == 0) {
			chThdSleepMilliseconds(10);
		}

		systime_t sleep_time = CH_CFG_ST_FREQUENCY / backup.config.send_can_status_rate_hz;
		if (sleep_time == 0) {
			sleep_time = 1;
		}

		chThdSleep(sleep_time);
	}
}

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	comm_can_send_buffer(rx_buffer_last_id, data, len, 1);
}

static void decode_msg(uint32_t eid, uint8_t *data8, int len, bool is_replaced) {
	int32_t ind = 0;
	unsigned int rxbuf_len;
	unsigned int rxbuf_ind;
	uint8_t crc_low;
	uint8_t crc_high;
	uint8_t commands_send;

	uint8_t id = eid & 0xFF;
	CAN_PACKET_ID cmd = eid >> 8;

	if (id == 255 || id == backup.config.controller_id) {
		switch (cmd) {
		case CAN_PACKET_FILL_RX_BUFFER:
			memcpy(rx_buffer + data8[0], data8 + 1, len - 1);
			break;

		case CAN_PACKET_FILL_RX_BUFFER_LONG:
			rxbuf_ind = (unsigned int)data8[0] << 8;
			rxbuf_ind |= data8[1];
			if (rxbuf_ind < RX_BUFFER_SIZE) {
				memcpy(rx_buffer + rxbuf_ind, data8 + 2, len - 2);
			}
			break;

		case CAN_PACKET_PROCESS_RX_BUFFER:
			ind = 0;
			rx_buffer_last_id = data8[ind++];
			commands_send = data8[ind++];
			rxbuf_len = (unsigned int)data8[ind++] << 8;
			rxbuf_len |= (unsigned int)data8[ind++];

			if (rxbuf_len > RX_BUFFER_SIZE) {
				break;
			}

			crc_high = data8[ind++];
			crc_low = data8[ind++];

			if (crc16(rx_buffer, rxbuf_len)
					== ((unsigned short) crc_high << 8
							| (unsigned short) crc_low)) {

				if (is_replaced) {
					if (rx_buffer[0] == COMM_JUMP_TO_BOOTLOADER ||
							rx_buffer[0] == COMM_ERASE_NEW_APP ||
							rx_buffer[0] == COMM_WRITE_NEW_APP_DATA ||
							rx_buffer[0] == COMM_WRITE_NEW_APP_DATA_LZO ||
							rx_buffer[0] == COMM_ERASE_BOOTLOADER) {
						break;
					}
				}

				sleep_reset();

				switch (commands_send) {
				case 0:
					commands_process_packet(rx_buffer, rxbuf_len, send_packet_wrapper);
					break;
				case 1:
					commands_send_packet(rx_buffer, rxbuf_len);
					break;
				case 2:
					commands_process_packet(rx_buffer, rxbuf_len, 0);
					break;
				default:
					break;
				}
			}
			break;

		case CAN_PACKET_PROCESS_SHORT_BUFFER:
			ind = 0;
			rx_buffer_last_id = data8[ind++];
			commands_send = data8[ind++];

			if (is_replaced) {
				if (data8[ind] == COMM_JUMP_TO_BOOTLOADER ||
						data8[ind] == COMM_ERASE_NEW_APP ||
						data8[ind] == COMM_WRITE_NEW_APP_DATA ||
						data8[ind] == COMM_WRITE_NEW_APP_DATA_LZO ||
						data8[ind] == COMM_ERASE_BOOTLOADER) {
					break;
				}
			}

			switch (commands_send) {
			case 0:
				commands_process_packet(data8 + ind, len - ind, send_packet_wrapper);
				break;
			case 1:
				commands_send_packet(data8 + ind, len - ind);
				break;
			case 2:
				commands_process_packet(data8 + ind, len - ind, 0);
				break;
			default:
				break;
			}
			break;

			case CAN_PACKET_PING: {
				uint8_t buffer[2];
				buffer[0] = backup.config.controller_id;
				buffer[1] = HW_TYPE_VESC_BMS;
				comm_can_transmit_eid(data8[0] |
						((uint32_t)CAN_PACKET_PONG << 8), buffer, 2);
			} break;

			case CAN_PACKET_PONG:
				// data8[0]; // Sender ID
				if (ping_tp) {
					if (len >= 2) {
						ping_hw_last = data8[1];
					} else {
						ping_hw_last = HW_TYPE_VESC_BMS;
					}
					chEvtSignal(ping_tp, 1 << 29);
				}
				break;

			case CAN_PACKET_SHUTDOWN: {
				// TODO: Implement when hw has power switch
			} break;

			default:
				break;
		}
	}

	switch (cmd) {
	case CAN_PACKET_PING:
		sleep_reset();
		break;

	case CAN_PACKET_STATUS:
		sleep_reset();

		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *stat_tmp = &stat_msgs[i];
			if (stat_tmp->id == id || stat_tmp->id == -1) {
				ind = 0;
				stat_tmp->id = id;
				stat_tmp->rx_time = chVTGetSystemTime();
				stat_tmp->rpm = (float)buffer_get_int32(data8, &ind);
				stat_tmp->current = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp->duty = (float)buffer_get_int16(data8, &ind) / 1000.0;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_2:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_2 *stat_tmp_2 = &stat_msgs_2[i];
			if (stat_tmp_2->id == id || stat_tmp_2->id == -1) {
				ind = 0;
				stat_tmp_2->id = id;
				stat_tmp_2->rx_time = chVTGetSystemTime();
				stat_tmp_2->amp_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
				stat_tmp_2->amp_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_3:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_3 *stat_tmp_3 = &stat_msgs_3[i];
			if (stat_tmp_3->id == id || stat_tmp_3->id == -1) {
				ind = 0;
				stat_tmp_3->id = id;
				stat_tmp_3->rx_time = chVTGetSystemTime();
				stat_tmp_3->watt_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
				stat_tmp_3->watt_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_4:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_4 *stat_tmp_4 = &stat_msgs_4[i];
			if (stat_tmp_4->id == id || stat_tmp_4->id == -1) {
				ind = 0;
				stat_tmp_4->id = id;
				stat_tmp_4->rx_time = chVTGetSystemTime();
				stat_tmp_4->temp_fet = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->temp_motor = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->current_in = (float)buffer_get_int16(data8, &ind) / 10.0;
				stat_tmp_4->pid_pos_now = (float)buffer_get_int16(data8, &ind) / 50.0;
				break;
			}
		}
		break;

	case CAN_PACKET_STATUS_5:
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg_5 *stat_tmp_5 = &stat_msgs_5[i];
			if (stat_tmp_5->id == id || stat_tmp_5->id == -1) {
				ind = 0;
				stat_tmp_5->id = id;
				stat_tmp_5->rx_time = chVTGetSystemTime();
				stat_tmp_5->tacho_value = buffer_get_int32(data8, &ind);
				stat_tmp_5->v_in = (float)buffer_get_int16(data8, &ind) / 1e1;
				break;
			}
		}
		break;

	case CAN_PACKET_BMS_SOC_SOH_TEMP_STAT: {
		int32_t ind = 0;
		bms_soc_soh_temp_stat msg;
		msg.id = id;
		msg.rx_time = chVTGetSystemTime();
		msg.v_cell_min = buffer_get_float16(data8, 1e3, &ind);
		msg.v_cell_max = buffer_get_float16(data8, 1e3, &ind);
		msg.soc = ((float)((uint8_t)data8[ind++])) / 255.0;
		msg.soh = ((float)((uint8_t)data8[ind++])) / 255.0;
		msg.t_cell_max = (float)((int8_t)data8[ind++]);
		uint8_t stat = data8[ind++];
		msg.is_charging = (stat >> 0) & 1;
		msg.is_balancing = (stat >> 1) & 1;
		msg.is_charge_allowed = (stat >> 2) & 1;

		// Do not go to sleep when some other pack is charging or balancing.
		if (msg.is_charging || msg.is_balancing) {
			sleep_reset();
		}

		// Find BMS with lowest cell voltage
		if (bms_stat_v_cell_min.id < 0 ||
				UTILS_AGE_S(bms_stat_v_cell_min.rx_time) > 10.0 ||
				bms_stat_v_cell_min.v_cell_min > msg.v_cell_min) {
			bms_stat_v_cell_min = msg;
		} else if (bms_stat_v_cell_min.id == msg.id) {
			bms_stat_v_cell_min = msg;
		}

		for (int i = 0;i < CAN_BMS_STATUS_MSGS_TO_STORE;i++) {
			bms_soc_soh_temp_stat *msg_buf = &bms_stat_msgs[i];

			// Reset ID after 10 minutes of silence
			if (msg_buf->id != -1 && UTILS_AGE_S(msg_buf->rx_time) > 60 * 10) {
				msg_buf->id = -1;
			}

			if (msg_buf->id == id || msg_buf->id == -1) {
				*msg_buf = msg;
				break;
			}
		}
	} break;

	case CAN_PACKET_PSW_STAT: {
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			psw_status *msg = &psw_stat[i];
			if (msg->id == id || msg->id == -1) {
				ind = 0;
				msg->id = id;
				msg->rx_time = chVTGetSystemTime();

				msg->v_in = buffer_get_float16(data8, 10.0, &ind);
				msg->v_out = buffer_get_float16(data8, 10.0, &ind);
				msg->temp = buffer_get_float16(data8, 10.0, &ind);
				msg->is_out_on = (data8[ind] >> 0) & 1;
				msg->is_pch_on = (data8[ind] >> 1) & 1;
				msg->is_dsc_on = (data8[ind] >> 2) & 1;
				ind++;
				break;
			}
		}
	} break;

	default:
		break;
	}
}

/**
 * Set the CAN timing. The CAN is clocked at 80 MHz, and the baud rate can be
 * calculated with
 *
 * 80000000 / ((brp + 1) * (ts1 + ts2 + 3))
 *
 * ts1 should be larger than ts2 in general to take the sample after the
 * signal had time to stabilize.
 *
 * @param brp
 * Prescaler.
 *
 * @param ts1
 * TS1.
 *
 * @param ts2
 * TS2.
 */
static void set_timing(int brp, int ts1, int ts2) {
	brp &= 0b1111111111;
	ts1 &= 0b1111;
	ts2 &= 0b111;

	cancfg.btr = CAN_BTR_SJW(3) | CAN_BTR_TS2(ts2) |
		CAN_BTR_TS1(ts1) | CAN_BTR_BRP(brp);

	canStop(&HW_CAN_DEV);
	canStart(&HW_CAN_DEV, &cancfg);
}
