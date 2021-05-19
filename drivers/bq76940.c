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
static i2c_bb_state  m_i2c;
static uint16_t 	 cell_voltage[14];

// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
static void write_reg_bq76940(uint8_t reg, uint16_t val);

void bq76940_init(
		stm32_gpio_t *sda_gpio, int sda_pin,
		stm32_gpio_t *scl_gpio, int scl_pin) {

	memset(&m_i2c, 0, sizeof(i2c_bb_state));
	m_i2c.sda_gpio = sda_gpio;
	m_i2c.sda_pin = sda_pin;
	m_i2c.scl_gpio = scl_gpio;
	m_i2c.scl_pin = scl_pin;

	i2c_bb_init(&m_i2c);

	//Read SYS_STAT
	uint16_t conf = 0;
	write_reg(SYS_STAT, conf); // Reset XREADY's register
	conf |= (1 << 5);
	write_reg(SYS_STAT, conf); // Reset XREADY's register
	//Wait the bit to 0

	//Enable Cell 1, Cell 2 and Cell 3
	conf = 0x1F;//if dont balance
	write_reg(CELLBAL1, conf);
	write_reg(CELLBAL2, conf);
	write_reg(CELLBAL3, conf);
	//Read and configure SYS_CTRL1
	conf = 0x00;//
	write_reg(SYS_CTRL1, conf);
	//Read and configure SYS_CTRL2
	conf = 0x00;//
	write_reg(SYS_CTRL2, conf);
	chThdSleep(1);
	conf = 0x02;//
	write_reg(SYS_CTRL2, conf);
	//Read and configure PROTECT1
	conf = 0x00;//
	write_reg(PROTECT1, conf);
	//Read and configure PROTECT2
	conf = 0x0F;//
	write_reg(PROTECT2, conf);
	//Read and configure PROTECT3
	conf = 0x00;//
	write_reg(PROTECT3, conf);
	//Read OV_TRIP
	conf = 0xAC;//
	write_reg(OV_TRIP, conf);
	//Read UV_TRIP
	conf = 0x97;//
	write_reg(UV_TRIP, conf);
	//Configure CC_CFG
	conf = 0x19;//
    write_reg(CC_CFG, conf);



	chThdCreateStatic(sample_thread_wa, sizeof(sample_thread_wa), LOWPRIO, sample_thread, NULL);
}



static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("BQ76940");

	static uint16_t register_bq76940[NUM_REG];

	for(;;) {
		m_i2c.has_error = 0;
		for(uint8_t i = 0 ; i < NUM_REG; i++){
			register_bq76940[i]=i2c_bb_tx_rx(&m_i2c, 0x40, 0, 0, register_bq76940, NUM_REG);
		}
		uint8_t j = 0;
		//Provisory
		//Convert two register in one, and calculate the value of cell voltage
		j=0;
		for(uint8_t i = VC1_HI ; i < VC15_LO; i+2){
			register_bq76940[i]=register_bq76940[i]<<10;
			register_bq76940[i]=register_bq76940[i]>>2;
			cell_voltage[j]=cell_voltage[j] | register_bq76940[i];
			cell_voltage[j]=cell_voltage[j] | register_bq76940[i+1];
			j++;
		}
		//Measure current

		chThdSleepMilliseconds(1000);
	}
}

static void write_reg_bq76940(uint8_t reg, uint16_t val) {
	m_i2c.has_error = 0;
	uint8_t txbuf[2];
	txbuf[0] = 0x00;
	txbuf[1] = reg;
	txbuf[2] = val >> 8;
	txbuf[3] = 0x00; //Here CRC

	i2c_bb_tx_rx(&m_i2c, 0x40, txbuf, 4, 0, 0);
}


