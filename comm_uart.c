/*
	Copyright 2021 Benjamin Vedder	benjamin@vedder.se

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

#ifdef HW_UART_DEV

#include "comm_uart.h"
#include "ch.h"
#include "hal.h"
#include "commands.h"
#include "packet.h"
#include "sleep.h"

// Threads
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);

// Variables
static PACKET_STATE_t packet_state;
static mutex_t send_mutex;
static SerialConfig uart_cfg = {
		CONF_UART_BAUD_RATE,
		0,
		USART_CR2_LINEN,
		0
};

// Functions
static void write_packet(unsigned char *data, unsigned int len);
static void process_packet(unsigned char *data, unsigned int len);

void comm_uart_init(void) {
	chMtxObjectInit(&send_mutex);

	packet_init(write_packet, process_packet, &packet_state);

	palSetLineMode(LINE_UART_RX, PAL_MODE_ALTERNATE(HW_UART_AF));
	palSetLineMode(LINE_UART_TX, PAL_MODE_ALTERNATE(HW_UART_AF));

	sdStart(&HW_UART_DEV, &uart_cfg);

	chThdCreateStatic(packet_process_thread_wa, sizeof(packet_process_thread_wa),
					NORMALPRIO, packet_process_thread, NULL);
}

void comm_uart_send_packet(unsigned char *data, unsigned int len) {
	chMtxLock(&send_mutex);
	packet_send_packet(data, len, &packet_state);
	chMtxUnlock(&send_mutex);
}

static void write_packet(unsigned char *data, unsigned int len) {
	sdWrite(&HW_UART_DEV, data, len);
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_uart_send_packet);
}

static THD_FUNCTION(packet_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("comm_uart");

	event_listener_t el;
	chEvtRegisterMaskWithFlags(&HW_UART_DEV.event, &el, EVENT_MASK(0), CHN_INPUT_AVAILABLE);

	for(;;) {
		chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		bool rx = true;
		while (rx) {
			rx = false;
			msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_IMMEDIATE);
			if (res != MSG_TIMEOUT) {
				packet_process_byte(res, &packet_state);
				rx = true;
			}
		}
	}
}

#endif
