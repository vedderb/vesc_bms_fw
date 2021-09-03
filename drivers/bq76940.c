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
#include "utils.h"
#include "main.h"
#include "math.h"

#define MAX_CELL_NUM	15

// Private variables
static i2c_bb_state  m_i2c;
volatile uint8_t gain_offset = 0;
static volatile float gain = 0;
static volatile float offset = 0;
static volatile float m_v_cell[MAX_CELL_NUM];
static volatile float measurement_temp[5];
static volatile float i_in = 0;
static volatile bool m_discharge_state[MAX_CELL_NUM] = {false};
// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
float gainRead(void);
float offsetRead(void);
void read_temp(volatile float *measurement_temp);
void iin_measure(float *value_iin);
uint8_t write_reg(uint8_t reg, uint16_t val);
static void read_cell_voltages(volatile float *m_v_cell);
uint8_t read_reg(uint8_t reg);
uint8_t CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
void balance(volatile bool *m_discharge_state);

uint8_t bq76940_init(
		stm32_gpio_t *sda_gpio, int sda_pin,
		stm32_gpio_t *scl_gpio, int scl_pin) {

	memset(&m_i2c, 0, sizeof(i2c_bb_state));
	m_i2c.sda_gpio = sda_gpio;
	m_i2c.sda_pin = sda_pin;
	m_i2c.scl_gpio = scl_gpio;
	m_i2c.scl_pin = scl_pin;

	i2c_bb_init(&m_i2c);

	uint8_t error = 0;

	// make sure the bq is booted up--->set TS1 to 3.3V and back to VSS
	// maybe set temp-mode for internal or external temp here

	// enable ADC
	error |= write_reg(BQ_SYS_CTRL1, ADC_EN);
	
	// check if ADC is active
	error |= read_reg(BQ_SYS_CTRL1) & ADC_EN;
	if(error == 1) { return 0;}//ERROR_ADC; }

	// write 0x19 to CC_CFG according to datasheet page 39
	error |= write_reg(BQ_CC_CFG, 0x19);

	gain = gainRead();      				// get gain
	if( (gain<365) | (gain>396) ) return 0;	//ERROR_GAIN check gain

	offset = offsetRead();  // get offset
	if( (offset<0x00) | (offset>0xFF) ) return 1;// dont check offset

	//HERE set OVERVOLTAGE and UNDERVOLTAGE, I'll change
	write_reg(BQ_OV_TRIP, 0xBC);//BC
	write_reg(BQ_UV_TRIP, 0x97);

	// for 190 A shutdown a main mosfet
	error |= write_reg(BQ_PROTECT1, 0x02);//registerWrite(PROTECT1, 0x0B);	// write PROTECT1
	error |= write_reg(BQ_PROTECT2, 0x01);//registerWrite(PROTECT2, 0x9C);	// write PROTECT2
	error |= write_reg(BQ_PROTECT3, 0x00);//registerWrite(PROTECT3, 0x50);	// write PROTECT3

	// clear SYS-STAT for init
	write_reg(BQ_SYS_STAT,0xFF);

	// doublecheck if bq is ready
	if(read_reg(BQ_SYS_STAT) & DEVICE_XREADY){
		// DEVICE_XREADY is set
		// write 1 in DEVICE_XREADY to clear it
		error |= write_reg(BQ_SYS_STAT, DEVICE_XREADY);
		// check again
		if(read_reg(BQ_SYS_STAT) & DEVICE_XREADY) return 1; // ERROR_XREADY;
	}

	// enable countinous reading of the Coulomb Counter
	error |= write_reg(BQ_SYS_CTRL2, CC_EN);	// sets ALERT at 250ms interval to high

	chThdSleepMilliseconds(10);
	write_reg(BQ_SYS_STAT,0xFF);
	chThdSleepMilliseconds(10);
	DISCHARGE_ON();
	chThdSleepMilliseconds(40);
	read_reg(BQ_SYS_STAT);

	chThdCreateStatic(sample_thread_wa, sizeof(sample_thread_wa), LOWPRIO, sample_thread, NULL);

	//return error; // 0 if successful
}

static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("BQ76940");

	while (!chThdShouldTerminateX()) {
		m_i2c.has_error = 0;

		chThdSleepMilliseconds(250); 	// time to read the cells
		read_cell_voltages(m_v_cell); 	//read cell voltages
		//chThdSleepMilliseconds(250); 	// time to read the thermistors
		//read_temp(measurement_temp);  	//read temperature
		//chThdSleepMilliseconds(30);
		balance(m_discharge_state);
		iin_measure(&i_in);				//measure current
	}
}

uint8_t write_reg(uint8_t reg, uint16_t val) {
	m_i2c.has_error = 0;
	uint8_t txbuf[3];
	uint8_t buff[4];

	buff[0] = 0x08 << 1;
	buff[1] = reg;
	buff[2] = val;

	txbuf[0] = reg;
	txbuf[1] = val;
	uint8_t key = 0x7;
	txbuf[2] = CRC8(buff, 3, key);
	i2c_bb_tx_rx(&m_i2c, 0x08, txbuf, 3, 0, 0);

	return 0;
}

uint8_t read_reg(uint8_t reg){
	uint8_t data;
 	i2c_bb_tx_rx(&m_i2c, 0x08, &reg, 1, &data, 2);
 	return data;
}

float gainRead(void){
	uint8_t reg1 = read_reg(BQ_ADCGAIN1);
	uint8_t reg2 = read_reg(BQ_ADCGAIN2);

	reg1 &= 0x0C;

	return (365.0 + ((reg1 << 1) | (reg2 >> 5)));
}

float offsetRead(void){
	return ((float)read_reg(BQ_ADCOFFSET) / 1000.0);
}


uint8_t CRC8(uint8_t *ptr, uint8_t len,uint8_t key){
	uint8_t  i;
	uint8_t  crc=0;

    while(len-- != 0){
        for(i=0x80; i!=0; i/=2){
            if((crc & 0x80) != 0){
                crc *= 2;
                crc ^= key;
            }
            else
                crc *= 2;
            if((*ptr & i) != 0)
                crc ^= key;
        }
        ptr++;
    }

    return(crc);
}

static void read_cell_voltages(volatile float *m_v_cell) {
	for (int i=0; i<MAX_CELL_NUM; i++) {
		uint16_t VCx_lo = read_reg(BQ_VC1_LO + i * 2);
		uint16_t VCx_hi = read_reg(BQ_VC1_HI + i * 2);
		m_v_cell[i] = (((((float)(VCx_lo | (VCx_hi << 8))) * gain) / 1e6)) + offset;
	}
	
	// For 14s setups, handle the special case of cell 14 connected to VC15
	m_v_cell[13] = m_v_cell[14];
}

float bq_last_cell_voltage(int cell) {
	if (cell < 0 || cell >= MAX_CELL_NUM) {
		return -1.0;
	}

	return m_v_cell[cell];
}

float bq_last_pack_voltage(void) {
	return 27.6;//m_v_pack;
}

void read_temp(volatile float *measurement_temp) {
	uint16_t buffer[6];
	float vtsx = 0.0;
	float R_ts = 0.0;

	buffer[0] = read_reg(BQ_TS1_HI);
	buffer[1] = read_reg(BQ_TS1_LO);
	vtsx = (float)((buffer[0]) | (buffer[1] << 8));
	R_ts = (vtsx * 1e4) / (vtsx - 3.3);
	*((measurement_temp) + 0) = (1.0 / ((logf(R_ts / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);

	buffer[2] = read_reg(BQ_TS2_HI);
	buffer[3] = read_reg(BQ_TS2_LO);
	//vtsx = ((buffer[2]) | (buffer[3]<<8));
	//R_ts = (vtsx*1e4)/(vtsx-3.3);
	*((measurement_temp) + 1) = 10;//(1.0 / (((logf(R_ts / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);

	buffer[4] = read_reg(BQ_TS3_HI);
	buffer[5] = read_reg(BQ_TS3_LO);
	//vtsx = ((buffer[4]) | (buffer[5]<<8));
	//R_ts = (vtsx*1e4)/(vtsx-3.3);
	*((measurement_temp) + 2) = 10;//(1.0 / (((logf(20000) / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);

	*((measurement_temp) + 3) = 10;

	*((measurement_temp) + 4) = 10;

}

float get_temp(int sensor){
	if (sensor < 0 || sensor >= 5){
			return -1.0;
	}

	return measurement_temp[sensor];
}

void iin_measure(float *i_in ){
	uint16_t buffer[2] = {0,0};
	uint8_t data = 0;

	//data = read_reg(SYS_CTRL2);
	//data = data | 0x20;
	//write_reg(SYS_CTRL2, data);
	chThdSleepMilliseconds(251);
	buffer[0] = read_reg(BQ_CC_HI);
	buffer[1] = read_reg(BQ_CC_LO);
	*(i_in) = ((float)(buffer[1] | buffer[0] << 8)) * 0.00844 / 0.5;
	//data = read_reg(SYS_CTRL2);
	//data = data & 0xBF;
	//write_reg(SYS_CTRL2, data);

	write_reg(BQ_SYS_STAT,0xFF);

	return;
}

float get_current(void){
	return i_in;
}

void DISCHARGE_ON(void){
	uint8_t	data = 0;

	data = read_reg(BQ_SYS_CTRL2);
	data = data | 0x02;
	write_reg(BQ_SYS_CTRL2, data);

	return;
}

void DISCHARGE_OFF(void){
	uint8_t	data = 0;

	data = read_reg(BQ_SYS_CTRL2);
	data = (data & 0xFD);
	write_reg(BQ_SYS_CTRL2, data);

	return;
}

void CHARGE_ON(void){
	uint8_t	data = 0;

	data = read_reg(BQ_SYS_CTRL2);
	data = data | 0x01;
	write_reg(BQ_SYS_CTRL2, data);

	return;
}

void CHARGE_OFF(void){
	uint8_t	data = 0;

	data = read_reg(BQ_SYS_CTRL2);
	data = data & 0xFE;
	write_reg(BQ_SYS_CTRL2, data);

	return;
}

void bq_set_dsc(int cell, bool set) {
	if (cell < 0 || cell >= MAX_CELL_NUM) {
		return;
	}

	m_discharge_state[cell] = set;
}

bool bq_get_dsc(int cell) {
	if (cell < 0 || cell >= MAX_CELL_NUM) {
		return false;
	}

	return m_discharge_state[cell];
}

void balance(volatile bool *m_discharge_state) {
	uint8_t buffer[3]= {0 ,0 ,0 };

	/*
	 * for(int i=0; i<5; i++){
	 * buffer[*0*] = m_discharge_state[i]? (1 << i) : 0;
	 * }
	 */
	buffer[0] = m_discharge_state[0] ? buffer[0] = 0x01 : buffer[0];
	buffer[0] = m_discharge_state[1] ? buffer[0] = 0x02 : buffer[0];
	buffer[0] = m_discharge_state[2] ? buffer[0] = 0x04 : buffer[0];
	buffer[0] = m_discharge_state[3] ? buffer[0] = 0x08 : buffer[0];
	buffer[0] = m_discharge_state[4] ? buffer[0] = 0x10 : buffer[0];
	chThdSleepMilliseconds(30);
	write_reg(BQ_CELLBAL1, buffer[0]);

	buffer[1] = m_discharge_state[5] ? buffer[1] = 0x01 : buffer[1];
	buffer[1] = m_discharge_state[6] ? buffer[1] = 0x02 : buffer[1];
	buffer[1] = m_discharge_state[7] ? buffer[1] = 0x04 : buffer[1];
	buffer[1] = m_discharge_state[8] ? buffer[1] = 0x08 : buffer[1];
	buffer[1] = m_discharge_state[9] ? buffer[1] = 0x10 : buffer[1];
	chThdSleepMilliseconds(30);
	write_reg(BQ_CELLBAL2, buffer[1]);

	buffer[2] = m_discharge_state[10] ? buffer[2] = 0x01 : buffer[2];
	buffer[2] = m_discharge_state[11] ? buffer[2] = 0x02 : buffer[2];
	buffer[2] = m_discharge_state[12] ? buffer[2] = 0x0C : buffer[2];	// Special case for a 14s setup
	buffer[2] = m_discharge_state[13] ? buffer[2] = 0x10 : buffer[2];
	chThdSleepMilliseconds(30);
	write_reg(BQ_CELLBAL3, buffer[2]);

	return;
}

bool ltc_get_dsc(int cell) {
	if (cell < 0 || cell > 17) {
		return false;
	}

	return m_discharge_state[cell];
}
