/*
	Copyright 2018 - 2020 Benjamin Vedder	benjamin@vedder.se

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
#ifndef LTC_C_
#define LTC_C_


#include "ltc6813.h"

#include "utils.h"
#include "main.h"


// Macros
#define APPEND_16b(cmd, data, ind)	data[ind++] = ((cmd) >> 8) & 0xFF;data[ind++] = (cmd) & 0xFF
#define SET4_LO_HI(byte, lo, hi)	byte = (((hi) & 0x0F) << 4) | ((lo) & 0x0F)
#define CLEAR_BUFFER_6(buffer)		buffer[0] = 0; buffer[1] = 0; buffer[2] = 0; buffer[3] = 0; buffer[4] = 0; buffer[5] = 0
#undef SET_BIT
#define SET_BIT(byte, bit, set)		byte |= set ? 1 << bit : 0

// Private variables
#ifndef AFE
static volatile float m_v_cell[18] = {0.0};
#endif

static THD_WORKING_AREA(ltc_thd_wa, 2048);
static volatile float m_v_pack = 0.0;
static volatile float m_v_cell_pu_diff[18] = {0.0};
static volatile float m_last_temp = 0.0;
static volatile float m_v_gpio[9] = {0.0};
static volatile bool m_discharge_state[18] = {false};


// Private functions
#ifndef AFE
static void read_cell_voltages(float *cells);
#endif
static THD_FUNCTION(ltc_thd, arg);
static void write_cmd(uint16_t cmd);
static bool read_reg_group(uint16_t cmd, uint8_t *buffer);
static bool write_reg_group(uint16_t cmd, const uint8_t *buffer);
static void ltc_wakeup(void);
static void poll_adc(void);
static uint16_t calc_pec(uint8_t len, const uint8_t *data);
static uint8_t spi_exchange(uint8_t x);
static void spi_transfer(uint8_t *in_buf, const uint8_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);



void ltc_init(void) {
	palSetLineMode(LINE_LTC_MISO, PAL_MODE_INPUT_PULLUP);
	palSetLineMode(LINE_LTC_SCLK, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_LTC_CS, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_LTC_MOSI, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLine(LINE_LTC_MOSI);
	
	chThdCreateStatic(ltc_thd_wa, sizeof(ltc_thd_wa), NORMALPRIO, ltc_thd, 0);

	(void)ltc_wakeup();
}




float ltc_last_temp(void) {
	return m_last_temp;
}

#ifndef AFE
float ltc_last_cell_voltage(int cell) { //disable for my
	if (cell < 0 || cell > 17) {
		return -1.0;
	}

	return m_v_cell[cell];
}


float ltc_last_pack_voltage(void) { //disable for my
	return m_v_pack;
}
#endif

float ltc_last_pu_diff_voltage(int cell) {
	if (cell < 0 || cell > 17) {
		return -1.0;
	}

	return m_v_cell_pu_diff[cell];
}



static THD_FUNCTION(ltc_thd, p) {
	(void)p;
	chRegSetThreadName("LTC");

	while (!chThdShouldTerminateX()) {
		uint8_t buffer[100];

//		ltc_wakeup();

		CLEAR_BUFFER_6(buffer);
		buffer[0] |= LTC_REFON; // Keep reference on, otherwise some measurements flicker
		buffer[0] |= LTC_GPIO1 | LTC_GPIO2 | LTC_GPIO3 | LTC_GPIO4 | LTC_GPIO5;
		SET_BIT(buffer[4], 0, m_discharge_state[0]);
		SET_BIT(buffer[4], 1, m_discharge_state[1]);
		SET_BIT(buffer[4], 2, m_discharge_state[2]);
		SET_BIT(buffer[4], 3, m_discharge_state[3]);
		SET_BIT(buffer[4], 4, m_discharge_state[4]);
		SET_BIT(buffer[4], 5, m_discharge_state[5]);
		SET_BIT(buffer[4], 6, m_discharge_state[6]);
		SET_BIT(buffer[4], 7, m_discharge_state[7]);
		SET_BIT(buffer[5], 3, m_discharge_state[11]);
		SET_BIT(buffer[5], 2, m_discharge_state[10]);
		SET_BIT(buffer[5], 1, m_discharge_state[9]);
		SET_BIT(buffer[5], 0, m_discharge_state[8]);
		write_reg_group(LTC_WRCFGA, buffer);

		CLEAR_BUFFER_6(buffer);
		SET_BIT(buffer[0], 7, m_discharge_state[15]);
		SET_BIT(buffer[0], 6, m_discharge_state[14]);
		SET_BIT(buffer[0], 5, m_discharge_state[13]);
		SET_BIT(buffer[0], 4, m_discharge_state[12]);
		SET_BIT(buffer[1], 1, m_discharge_state[17]);
		SET_BIT(buffer[1], 0, m_discharge_state[16]);
		write_reg_group(LTC_WRCFGB, buffer);

		write_cmd(LTC_MUTE);
		chThdSleepMilliseconds(1);

		write_cmd(LTC_ADCV | LTC_MD10);
		poll_adc();
#ifndef AFE
		read_cell_voltages((float*)m_v_cell);
#endif
		// Open wire check
		float cells_pu[18], cells_pd[18];
		write_cmd(LTC_ADOW | LTC_MD10);
		poll_adc();
		read_cell_voltages(cells_pu);
		write_cmd(LTC_ADOW | LTC_MD10 | LTC_PUP);
		poll_adc();
		read_cell_voltages(cells_pd);
		for (int i = 0;i < 18;i++) {
			m_v_cell_pu_diff[i] = cells_pu[i] - cells_pd[i];
		}

		write_cmd(LTC_ADSTAT | LTC_MD10);
		poll_adc();

		write_cmd(LTC_ADAX | LTC_MD10 | LTC_CHG000);
		poll_adc();

		write_cmd(LTC_UNMUTE);

		if (read_reg_group(LTC_RDSTATA, buffer)) {
			m_v_pack = (float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4 * 30.0;
			m_last_temp = (float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) * 100e-6 / 7.6e-3 - 276.0;
		}

		if (read_reg_group(LTC_RDAUXA, buffer)) {
			m_v_gpio[0] = (float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4;
			m_v_gpio[1] = (float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) / 1e4;
			m_v_gpio[2] = (float)((uint16_t)buffer[4] | (uint16_t)buffer[5] << 8) / 1e4;
		}

		if (read_reg_group(LTC_RDAUXB, buffer)) {
			m_v_gpio[3] = (float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4;
			m_v_gpio[4] = (float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) / 1e4;
		}

		chThdSleepMilliseconds(100);
	}
}

static void write_cmd(uint16_t cmd) {
	uint8_t buffer[4];
	int ind = 0;
	APPEND_16b(cmd, buffer, ind);
	uint16_t pec = calc_pec(ind, buffer);
	APPEND_16b(pec, buffer, ind);
	spi_begin();
	spi_transfer(0, buffer, ind);
	spi_end();
}

static bool read_reg_group(uint16_t cmd, uint8_t *buffer) {
	uint8_t buffer_send[4];
	int ind = 0;
	uint16_t pec = 0;

	APPEND_16b(cmd, buffer_send, ind);
	pec = calc_pec(ind, buffer_send);
	APPEND_16b(pec, buffer_send, ind);
	spi_begin();
	spi_transfer(0, buffer_send, ind);
	spi_transfer(buffer, 0, 8);
	spi_end();

	pec = calc_pec(6, buffer);

	if (pec == (((uint16_t)buffer[6] << 8) | (uint16_t)buffer[7])) {
		return true;
	} else {
		return false;
	}
}

static bool write_reg_group(uint16_t cmd, const uint8_t *buffer) {
	uint8_t buffer_send[4];
	int ind = 0;
	uint16_t pec = 0;

	APPEND_16b(cmd, buffer_send, ind);
	pec = calc_pec(ind, buffer_send);
	APPEND_16b(pec, buffer_send, ind);

	spi_begin();
	spi_transfer(0, buffer_send, ind);
	spi_transfer(0, buffer, 6);
	pec = calc_pec(6, buffer);
	ind = 0;
	APPEND_16b(pec, buffer_send, ind);
	spi_transfer(0, buffer_send, ind);
	spi_end();

	return true;
}

static void poll_adc(void) {
	systime_t start = chVTGetSystemTimeX();

	uint8_t buffer[4];
	int ind = 0;
	APPEND_16b(LTC_PLADC, buffer, ind);
	uint16_t pec = calc_pec(ind, buffer);
	APPEND_16b(pec, buffer, ind);
	spi_begin();
	spi_transfer(0, buffer, ind);

	int sleep_cnt = 0;

	uint8_t res;
	spi_transfer(&res, 0, 1);

	while (!res) {
		chThdSleep(1);
		spi_transfer(&res, 0, 1);
		sleep_cnt++;

		// Timeout
		if (TIME_I2MS(chVTTimeElapsedSinceX(start)) > 1500) {
			break;
		}
	}

//	main_printf_usb("Elapsed uS: %d\r\n", TIME_I2US(chVTTimeElapsedSinceX(start)));

	spi_end();
}

static void ltc_wakeup(void) {
	spi_begin();
	spi_exchange(0xFF);
	spi_end();
}

static uint16_t calc_pec(uint8_t len, const uint8_t *data) {
	uint16_t crc = 0x10;

	while (len--) {
		crc ^= ((uint16_t)(*data++) << 7);

		for (int i = 0;i < 8;i++) {
			uint16_t eor = crc & 0x4000 ? 0x4599 : 0;
			crc <<= 1;
			crc ^= eor;
		}
	}

	return (crc & 0x7fff) << 1;
};

#ifndef AFE
static void read_cell_voltages(float *cells) {
	uint8_t buffer[8];

	if (read_reg_group(LTC_RDCVA, buffer)) {
		cells[0] = (float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4;
		cells[1] = (float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) / 1e4;
		cells[2] = (float)((uint16_t)buffer[4] | (uint16_t)buffer[5] << 8) / 1e4;
	}

	if (read_reg_group(LTC_RDCVB, buffer)) {
		cells[3] = (float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4;
		cells[4] = (float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) / 1e4;
		cells[5] = (float)((uint16_t)buffer[4] | (uint16_t)buffer[5] << 8) / 1e4;
	}

	if (read_reg_group(LTC_RDCVC, buffer)) {
		cells[6] = (float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4;
		cells[7] = (float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) / 1e4;
		cells[8] = (float)((uint16_t)buffer[4] | (uint16_t)buffer[5] << 8) / 1e4;
	}

	if (read_reg_group(LTC_RDCVD, buffer)) {
		cells[9] = (float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4;
		cells[10] = (float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) / 1e4;
		cells[11] = (float)((uint16_t)buffer[4] | (uint16_t)buffer[5] << 8) / 1e4;
	}

	if (read_reg_group(LTC_RDCVE, buffer)) {
		cells[12] = (float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4;
		cells[13] = (float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) / 1e4;
		cells[14] = (float)((uint16_t)buffer[4] | (uint16_t)buffer[5] << 8) / 1e4;
	}

	if (read_reg_group(LTC_RDCVF, buffer)) {
		cells[15] = (float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4;
		cells[16] = (float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) / 1e4;
		cells[17] = (float)((uint16_t)buffer[4] | (uint16_t)buffer[5] << 8) / 1e4;
	}
}

#endif

// Software SPI
static uint8_t spi_exchange(uint8_t x) {
	uint8_t rx;
	spi_transfer(&rx, &x, 1);
	return rx;
}

static void spi_transfer(uint8_t *in_buf, const uint8_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint8_t send = out_buf ? out_buf[i] : 0xFF;
		uint8_t receive = 0;

		for (int bit = 0;bit < 8;bit++) {
			palWriteLine(LINE_LTC_MOSI, send >> 7);
			send <<= 1;

			palClearLine(LINE_LTC_SCLK);
			spi_delay();
			palSetLine(LINE_LTC_SCLK);

			int samples = 0;
			samples += palReadLine(LINE_LTC_MISO);
			__NOP();
			samples += palReadLine(LINE_LTC_MISO);
			__NOP();
			samples += palReadLine(LINE_LTC_MISO);
			__NOP();
			samples += palReadLine(LINE_LTC_MISO);
			__NOP();
			samples += palReadLine(LINE_LTC_MISO);

			receive <<= 1;
			if (samples > 2) {
				receive |= 1;
			}

			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = receive;
		}
	}
}

static void spi_begin(void) {
	palClearLine(LINE_LTC_CS);
}

static void spi_end(void) {
	palSetLine(LINE_LTC_CS);
}

static void spi_delay(void) {
	for (volatile int i = 0;i < 3;i++) {
		__NOP();
	}
}

void ltc_sleep(void) {
	uint8_t buffer[6];
	CLEAR_BUFFER_6(buffer);
	buffer[0] |= LTC_GPIO1 | LTC_GPIO2 | LTC_GPIO3 | LTC_GPIO4 | LTC_GPIO5;
	write_reg_group(LTC_WRCFGA, buffer);
}


float ltc_last_gpio_voltage(int gpio) {
	if (gpio <= 0 || gpio > 9) {
			return -1.0;
		}

		return m_v_gpio[gpio - 1];
}

void ltc_set_dsc(int cell, bool set) {
	if (cell < 0 || cell > 17) {
		return;
	}

	m_discharge_state[cell] = set;
}

bool ltc_get_dsc(int cell) {
	if (cell < 0 || cell > 17) {
		return false;
	}

	return m_discharge_state[cell];
}



#endif /* LTC_C_ */


