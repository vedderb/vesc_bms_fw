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

//Macro
#define APPEND_16b(cmd, data, ind)	data[ind++] = ((cmd) >> 8) & 0xFF;data[ind++] = (cmd) & 0xFF
#define SET4_LO_HI(byte, lo, hi)	byte = (((hi) & 0x0F) << 4) | ((lo) & 0x0F)
#define CLEAR_BUFFER_6(buffer)		buffer[0] = 0; buffer[1] = 0; buffer[2] = 0; buffer[3] = 0; buffer[4] = 0; buffer[5] = 0
#undef SET_BIT
#define SET_BIT(byte, bit, set)		byte |= set ? 1 << bit : 0

// Private variables
static i2c_bb_state  m_i2c;
volatile uint8_t gain_offset = 0;
static volatile float m_v_cell[14] = {0.0};
// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
static void write_reg(uint8_t reg, uint16_t val);
static bool read_reg_group(uint16_t cmd, uint8_t *buffer);
static void read_cell_voltages(float *cells);
uint8_t read_reg(i2c_bb_state *s,uint8_t reg);
uint8_t CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
uint16_t gainRead(void);



void bq76940_init(
		stm32_gpio_t *sda_gpio, int sda_pin,
		stm32_gpio_t *scl_gpio, int scl_pin) {

	memset(&m_i2c, 0, sizeof(i2c_bb_state));
	m_i2c.sda_gpio = sda_gpio;
	m_i2c.sda_pin = sda_pin;
	m_i2c.scl_gpio = scl_gpio;
	m_i2c.scl_pin = scl_pin;

	i2c_bb_init(&m_i2c);

	volatile uint8_t error = 0;
	volatile uint8_t data = 0;


	chThdSleepMilliseconds(30);
	write_reg(SYS_CTRL1,ADC_EN);


	chThdSleepMilliseconds(30);
	write_reg(CC_CFG, 0x19);

	chThdSleepMilliseconds(30);
	write_reg(SYS_CTRL2, 0x60);

	chThdSleepMilliseconds(30);
	write_reg(PROTECT1, 0x00);

	chThdSleepMilliseconds(30);
	write_reg(PROTECT2, 0x0F);

	chThdSleepMilliseconds(30);
	write_reg(PROTECT3, 0xF0);

	chThdSleepMilliseconds(30);
	write_reg(OV_TRIP, 0xAC);

	chThdSleepMilliseconds(30);
	write_reg(UV_TRIP, 0x97);

	chThdSleepMilliseconds(30);
	data=read_reg(&m_i2c,ADCOFFSET); //read the offset register
	gain_offset=data;

	chThdSleepMilliseconds(30);
	write_reg(SYS_STAT, 0xFF);

	//chThdSleepMilliseconds(2000);
	chThdSleepMilliseconds(30);
	read_reg(&m_i2c,ADCGAIN1); //read the offset register

	chThdSleepMilliseconds(30);
	read_reg(&m_i2c,SYS_STAT);

	chThdCreateStatic(sample_thread_wa, sizeof(sample_thread_wa), LOWPRIO, sample_thread, NULL);
}



static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("BQ76940");

	for(;;) {
		m_i2c.has_error = 0;

		chThdSleepMilliseconds(250); // time to read the cells
		read_cell_voltages((float*)m_v_cell);

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

uint8_t read_reg(i2c_bb_state *s, uint8_t reg){

	uint8_t rxbuf[2],txbuf[1];
	uint8_t data;

	txbuf[0]=reg;
 	i2c_bb_tx_rx(&m_i2c, 0x08, txbuf, 1, rxbuf, 2);
 	data=rxbuf[0];

 	return data;

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

uint16_t gainRead(void){
	//uint8_t reg1;
	//chThdSleepMilliseconds(30);
	//read_reg(&m_i2c,ADCGAIN1);
	//uint8_t reg2;
	//chThdSleepMilliseconds(30);
	//read_reg(&m_i2c,ADCGAIN2);


	//reg1 &= 0b00001100;

	//return (365 + ((reg1 << 1)|(reg2 >> 5)));
}

static void read_cell_voltages(float *cells) {
	uint8_t buffer[21];

	//Cell 1
	//buffer[0]=read_reg(&m_i2c,VC1_LO); //
	//buffer[1]=read_reg(&m_i2c,VC1_HI); //
	cells[0] = 4.1;//(float)((uint16_t)buffer[0] | (uint16_t)buffer[1] << 8) / 1e4;

	//Cell 2
	//buffer[2]=read_reg(&m_i2c,VC2_LO); //
	//buffer[3]=read_reg(&m_i2c,VC2_HI); //
	cells[1] = 4.1;//(float)((uint16_t)buffer[2] | (uint16_t)buffer[3] << 8) / 1e4;

	//Cell 3
	//buffer[4]=read_reg(&m_i2c,VC3_LO); //
	//buffer[5]=read_reg(&m_i2c,VC3_HI); //
	cells[2] = 4.1;//(float)((uint16_t)buffer[4] | (uint16_t)buffer[5] << 8) / 1e4;

	//Cell 4
	//buffer[6]=read_reg(&m_i2c,VC4_LO); //
	//buffer[7]=read_reg(&m_i2c,VC4_HI); //
	cells[3] = 4.1;//(float)((uint16_t)buffer[6] | (uint16_t)buffer[7] << 8) / 1e4;

	//Cell 5
	//buffer[8]=read_reg(&m_i2c,VC5_LO); //
	//buffer[9]=read_reg(&m_i2c,VC5_HI); //
	cells[4] = 4.1;//(float)((uint16_t)buffer[8] | (uint16_t)buffer[9] << 8) / 1e4;

	//Cell 6
	//buffer[10]=read_reg(&m_i2c,VC6_LO); //
	//buffer[11]=read_reg(&m_i2c,VC6_HI); //
	cells[5] = 4.1;//(float)((uint16_t)buffer[10] | (uint16_t)buffer[11] << 8) / 1e4;

	//Cell 7
	//buffer[12]=read_reg(&m_i2c,VC7_LO); //
	//buffer[13]=read_reg(&m_i2c,VC7_HI); //
	cells[6] = 4.1;//(float)((uint16_t)buffer[12] | (uint16_t)buffer[13] << 8) / 1e4;

	//Cell 8
	//buffer[14]=read_reg(&m_i2c,VC8_LO); //
	//buffer[15]=read_reg(&m_i2c,VC8_HI); //
	cells[7] = 4.1;//(float)((uint16_t)buffer[14] | (uint16_t)buffer[15] << 8) / 1e4;

	//Cell 9
	//buffer[16]=read_reg(&m_i2c,VC9_LO); //
	//buffer[17]=read_reg(&m_i2c,VC9_HI); //
	cells[8] = 4.1;//(float)((uint16_t)buffer[16] | (uint16_t)buffer[17] << 8) / 1e4;

	//Cell 10
	//buffer[18]=read_reg(&m_i2c,VC10_LO); //
	//buffer[19]=read_reg(&m_i2c,VC10_HI); //
	cells[9] = 4.1;//(float)((uint16_t)buffer[18] | (uint16_t)buffer[19] << 8) / 1e4;

	//Cell 11
	//buffer[20]=read_reg(&m_i2c,VC11_LO); //
	//buffer[21]=read_reg(&m_i2c,VC11_HI); //
	cells[10] = 4.1;//(float)((uint16_t)buffer[20] | (uint16_t)buffer[21] << 8) / 1e4;


	cells[11] = 4.1;
	cells[12] = 4.1;
	cells[13] = 4.1;
}

static bool read_reg_group(uint16_t cmd, uint8_t *buffer) {
/*	uint8_t buffer_send[4];
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
*/
}

float ltc_last_cell_voltage(int cell) {
	if (cell < 0 || cell > 17) {
		return -1.0;
	}

	return m_v_cell[cell];
}

float ltc_last_pack_voltage(void) {
	return 27,6;//m_v_pack;
}


