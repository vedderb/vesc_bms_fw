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

#include "ch.h"
#include "hal.h"
#include "comm_usb.h"
#include "packet.h"
#include "usbcfg.h"
#include "commands.h"

// Private variables
#define SERIAL_RX_BUFFER_SIZE		2048
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static THD_WORKING_AREA(serial_read_thread_wa, 512);
static THD_WORKING_AREA(serial_process_thread_wa, 4096);
static mutex_t send_mutex;
static thread_t *process_tp;
static volatile unsigned int write_timeout_cnt = 0;
static volatile bool was_timeout = false;
static PACKET_STATE_t packet_state;
static mutex_t rx_mtx;

// Private functions
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_raw(unsigned char *buffer, unsigned int len);

static THD_FUNCTION(serial_read_thread, arg) {
	(void)arg;

	chRegSetThreadName("USB read");

	uint8_t buffer[102];
	int had_data = 0;

	for(;;) {
		// http://forum.chibios.org/viewtopic.php?f=25&t=3938&start=10

		int len = chnReadTimeout(&SDU1, (uint8_t*) buffer, 100, 1);
		chThdSleep(1);

		for (int i = 0;i < len;i++) {
			chMtxLock(&rx_mtx);
			serial_rx_buffer[serial_rx_write_pos++] = buffer[i];

			if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_write_pos = 0;
			}
			chMtxUnlock(&rx_mtx);

			had_data = 1;
		}

		if (had_data) {
			chEvtSignal(process_tp, (eventmask_t) 1);
			had_data = 0;
		}
	}
}

static THD_FUNCTION(serial_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("USB process");

	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		for (;;) {
			chMtxLock(&rx_mtx);
			if (serial_rx_read_pos != serial_rx_write_pos) {
				uint8_t byte = serial_rx_buffer[serial_rx_read_pos++];

				if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
					serial_rx_read_pos = 0;
				}
				chMtxUnlock(&rx_mtx);

				packet_process_byte(byte, &packet_state);
			} else {
				chMtxUnlock(&rx_mtx);
				break;
			}
		}
	}
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_usb_send_packet);
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
	/*
	 * Only write to USB if the cable has been connected at least once. If a timeout occurs
	 * make sure that this call does not stall on the next call, as the timeout probably occurred
	 * because noone is listening on the USB.
	 */
	if (usb_cdc_configured_cnt() > 0) {
		unsigned int written = 0;
		if (was_timeout) {
			written = SDU1.vmt->writet(&SDU1, buffer, len, TIME_IMMEDIATE);
		} else {
			written = SDU1.vmt->writet(&SDU1, buffer, len, TIME_MS2I(100));
		}

		was_timeout = written != len;
		if (was_timeout) {
			write_timeout_cnt++;
		}
	}
}

void comm_usb_init(void) {
	usb_cdc_init();
	packet_init(send_packet_raw, process_packet, &packet_state);

	chMtxObjectInit(&send_mutex);
	chMtxObjectInit(&rx_mtx);

	// Threads
	chThdCreateStatic(serial_read_thread_wa, sizeof(serial_read_thread_wa), NORMALPRIO + 1, serial_read_thread, NULL);
	chThdCreateStatic(serial_process_thread_wa, sizeof(serial_process_thread_wa), NORMALPRIO, serial_process_thread, NULL);
}

void comm_usb_send_packet(unsigned char *data, unsigned int len) {
	chMtxLock(&send_mutex);
	packet_send_packet(data, len, &packet_state);
	chMtxUnlock(&send_mutex);
}

unsigned int comm_usb_get_write_timeout_cnt(void) {
	return write_timeout_cnt;
}
