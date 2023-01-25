/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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
#include "string.h"
#include "bme280_if.h"
#include "bme280.h"

// Private variables
static i2c_bb_state m_i2c;
static float m_last_temp = 0.0;
static float m_last_hum = 0.0;
static float m_last_pres = 0.0;

// Threads
static THD_WORKING_AREA(sample_thread_wa, 1024);
static THD_FUNCTION(sample_thread, arg);

void bme280_if_init(
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

float bme280_if_get_hum(void) {
	return m_last_hum;
}

float bme280_if_get_temp(void) {
	return m_last_temp;
}

float bme280_if_get_pres(void) {
	return m_last_pres;
}

static void user_delay_us(uint32_t period, void *intf_ptr) {
	(void)intf_ptr;
	chThdSleepMicroseconds(period);
}

static int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	(void)intf_ptr;

	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

	m_i2c.has_error = 0;
	uint8_t txbuf[1];
	txbuf[0] = reg_addr;

	rslt = i2c_bb_tx_rx(&m_i2c, BME280_I2C_ADDR_PRIM, txbuf, 1, reg_data, len) ? 0 : -1;

	return rslt;
}

static int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	(void)intf_ptr;

	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

	m_i2c.has_error = 0;
	uint8_t txbuf[len + 1];
	txbuf[0] = reg_addr;
	memcpy(txbuf + 1, reg_data, len);

	rslt = i2c_bb_tx_rx(&m_i2c, BME280_I2C_ADDR_PRIM, txbuf, len + 1, 0, 0) ? 0 : -1;

	return rslt;
}

static THD_FUNCTION(sample_thread, arg) {
	(void)arg;

	chRegSetThreadName("BME280");

	struct bme280_dev dev;
	uint8_t dev_addr = BME280_I2C_ADDR_PRIM;

	dev.intf_ptr = &dev_addr;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_us = user_delay_us;

	bme280_init(&dev);

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;

	uint8_t settings_sel;
	uint32_t req_delay;
	struct bme280_data comp_data;

	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
	bme280_set_sensor_settings(settings_sel, &dev);
	req_delay = bme280_cal_meas_delay(&dev.settings);

	for(;;) {
		bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
		chThdSleepMilliseconds(req_delay);

		bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		m_last_hum = comp_data.humidity;
		m_last_temp = comp_data.temperature;
		m_last_pres = comp_data.pressure;
	}
}
