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
#include "bq76940.h"
#include "string.h"

// Private variables
static i2c_bb_state m_i2c;
//static float m_last_temp = 0.0;
//static float m_last_hum = 0.0;

// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
static void write_reg(uint8_t reg, uint16_t val);


void bq76940_init(
		stm32_gpio_t *sda_gpio, int sda_pin,
		stm32_gpio_t *scl_gpio, int scl_pin) {

	memset(&m_i2c, 0, sizeof(i2c_bb_state));
	m_i2c.sda_gpio = sda_gpio;
	m_i2c.sda_pin = sda_pin;
	m_i2c.scl_gpio = scl_gpio;
	m_i2c.scl_pin = scl_pin;

	i2c_bb_init(&m_i2c);

	uint16_t conf = 0;
	conf |= (1 << 12); // Read temperature and humidity
	write_reg(0x02, conf);

	chThdCreateStatic(sample_thread_wa, sizeof(sample_thread_wa), LOWPRIO, sample_thread, NULL);
}



static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("BQ76940");

	for(;;) {
		m_i2c.has_error = 0;
		uint8_t rxbuf[4];
		i2c_bb_tx_rx(&m_i2c, 0x40, 0, 0, rxbuf, 4);
		//uint16_t temp = (uint16_t)rxbuf[0] << 8 | (uint16_t)rxbuf[1];
		//uint16_t hum = (uint16_t)rxbuf[2] << 8 | (uint16_t)rxbuf[3];

		//m_last_temp = (float)temp / 65536.0 * 165.0 - 40.0;
		//m_last_hum = (float)hum / 65536.0 * 100.0;

		// Start next measurement
		m_i2c.has_error = 0;
		uint8_t txbuf[1];
		txbuf[0] = 0x0;
		i2c_bb_tx_rx(&m_i2c, 0x40, txbuf, 1, 0, 0);

		chThdSleepMilliseconds(1000);
	}
}

static void write_reg(uint8_t reg, uint16_t val) {
	m_i2c.has_error = 0;
	uint8_t txbuf[3];
	txbuf[0] = reg;
	txbuf[1] = val >> 8;
	txbuf[2] = val & 0xFF;
	i2c_bb_tx_rx(&m_i2c, 0x40, txbuf, 3, 0, 0);
}
