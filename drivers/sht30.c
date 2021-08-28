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

#include "i2c_bb.h"
#include "sht30.h"
#include "string.h"

// Private variables
static i2c_bb_state m_i2c;
static float m_last_temp = 0.0;
static float m_last_hum = 0.0;

// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
static uint8_t crc8(const uint8_t *data, uint8_t len);

void sht30_init(
		stm32_gpio_t *sda_gpio, int sda_pin,
		stm32_gpio_t *scl_gpio, int scl_pin) {

	memset(&m_i2c, 0, sizeof(i2c_bb_state));
	m_i2c.sda_gpio = sda_gpio;
	m_i2c.sda_pin = sda_pin;
	m_i2c.scl_gpio = scl_gpio;
	m_i2c.scl_pin = scl_pin;

	i2c_bb_init(&m_i2c);

	chThdCreateStatic(sample_thread_wa, sizeof(sample_thread_wa), LOWPRIO, sample_thread, NULL);
}

float sht30_get_hum(void) {
	return m_last_hum;
}

float sht30_get_temp(void) {
	return m_last_temp;
}

static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("SHT30");

	for(;;) {
		m_i2c.has_error = 0;
		uint8_t rxbuf[6];

		(void)i2c_bb_tx_rx(&m_i2c, 0x44, 0, 0, rxbuf, 6);

		if (rxbuf[2] == crc8(rxbuf, 2) && rxbuf[5] == crc8(rxbuf + 3, 2)) {
			uint16_t temp = (uint16_t)rxbuf[0] << 8 | (uint16_t)rxbuf[1];
			uint16_t hum = (uint16_t)rxbuf[3] << 8 | (uint16_t)rxbuf[4];

			m_last_temp = (float)temp / 65535.0 * 175.0 - 45.0;
			m_last_hum = (float)hum / 65535.0 * 100.0;
		} else {
			m_last_temp = 0.0;
			m_last_hum = 0.0;
		}

		// Start next measurement
		m_i2c.has_error = 0;
		uint8_t txbuf[2];
		txbuf[0] = 0x24;
		txbuf[1] = 0x00;
		i2c_bb_tx_rx(&m_i2c, 0x44, txbuf, 2, 0, 0);

		chThdSleepMilliseconds(1000);
	}
}

static uint8_t crc8(const uint8_t *data, uint8_t len)  {
	const uint8_t poly = 0x31;
	uint8_t crc = 0xFF;

	for (uint8_t j = len; j; --j) {
		crc ^= *data++;

		for (uint8_t i = 8; i; --i) {
			crc = (crc & 0x80) ? (crc << 1) ^ poly : (crc << 1);
		}
	}

	return crc;
}
