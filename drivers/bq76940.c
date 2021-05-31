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
#include "stdbool.h"

// Private variables
static i2c_bb_state  m_i2c;
static uint16_t 	 cell_voltage[14];

// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
static void write_reg(uint8_t reg, uint16_t val);
static void read_reg(i2c_bb_state *s,uint8_t reg, uint16_t val);
uint8_t CRC8(unsigned char *ptr, unsigned char len,unsigned char key);

void bq76940_init(
		stm32_gpio_t *sda_gpio, int sda_pin,
		stm32_gpio_t *scl_gpio, int scl_pin) {

	memset(&m_i2c, 0, sizeof(i2c_bb_state));
	m_i2c.sda_gpio = sda_gpio;
	m_i2c.sda_pin = sda_pin;
	m_i2c.scl_gpio = scl_gpio;
	m_i2c.scl_pin = scl_pin;

	i2c_bb_init(&m_i2c);

	//chThdSleepMilliseconds(2000);

	chThdSleepMilliseconds(30);
	write_reg(SYS_STAT, 0xFF);

	chThdSleepMilliseconds(30);
	write_reg(CELLBAL1, 0x00);

	chThdSleepMilliseconds(30);
	write_reg(CELLBAL2, 0x00);

	chThdSleepMilliseconds(30);//20
	write_reg(CELLBAL3, 0x00);

	chThdSleepMilliseconds(30);//20
	write_reg(CC_CFG, 0x19);

	uint8_t val;
	chThdSleepMilliseconds(30);//20
	read_reg(&m_i2c,VC1_HI,val);


	chThdCreateStatic(sample_thread_wa, sizeof(sample_thread_wa), LOWPRIO, sample_thread, NULL);
}



static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("BQ76940");

	for(;;) {
		m_i2c.has_error = 0;

		chThdSleepMilliseconds(1000);
	}
}

static void write_reg(uint8_t reg, uint16_t val) {
	m_i2c.has_error = 0;
	uint8_t txbuf[3];
	uint8_t buff[4];

	buff[0]=0x08<<1;
	buff[1]=reg;
	buff[2]=val;

	txbuf[0] = reg;
	txbuf[1] = val;
	uint8_t key=0x7;
	txbuf[2]=CRC8(buff, 3, key);
	i2c_bb_tx_rx(&m_i2c, 0x08, txbuf, 3, 0, 0);
}

static void read_reg(i2c_bb_state *s, uint8_t reg, uint16_t val){
	//m_i2c.has_error = 0;

	uint8_t rxbuf[55],txbuf;

	txbuf=reg;
	i2c_bb_tx_rx(&m_i2c, 0x08, txbuf, 1, rxbuf, 55);

}

uint8_t CRC8(uint8_t *ptr, uint8_t len,uint8_t key){
	uint8_t  i;
	uint8_t  crc=0;

    while(len--!=0){
        for(i=0x80; i!=0; i/=2){
            if((crc & 0x80) != 0){
                crc *= 2;
                crc ^= key;
            }
            else
                crc *= 2;
            if((*ptr & i)!=0) //
                crc ^= key;
        }
        ptr++;
    }
return(crc);
}


