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
#include "bms_if.h"
#include "string.h"
#include "stdbool.h"
#include "utils.h"
#include "main.h"
#include "math.h"

#define MAX_CELL_NUM		15
#define BQ_I2C_ADDR			0x08

// Private variables
static i2c_bb_state  m_i2c;
static volatile float m_v_cell[MAX_CELL_NUM];
static volatile float measurement_temp[5];
static volatile float i_in = 0;
static volatile bool m_discharge_state[MAX_CELL_NUM] = {false};
static volatile float hw_shunt_res = 1.0;

typedef struct {
	i2c_bb_state m_i2c;
	stm32_gpio_t *alert_gpio;
	int alert_pin;
	float shunt_res;
	float gain;
	float offset;
} bq76940_t;

static bq76940_t bq76940;

// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
int8_t gainRead(float *gain);
int8_t offsetRead(float *offset);
void read_temp(volatile float *measurement_temp);
void iin_measure(float *value_iin);
uint8_t write_reg(uint8_t reg, uint16_t val);
static void read_cell_voltages(float *m_v_cell);
uint8_t read_reg(uint8_t reg);
uint8_t CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
void balance(volatile bool *m_discharge_state);
uint8_t tripVoltage(float voltage);

//Macros
#define READ_ALERT()	palReadPad(bq76940.alert_gpio, bq76940.alert_pin)

uint8_t bq76940_init(
		stm32_gpio_t *sda_gpio, int sda_pin,
		stm32_gpio_t *scl_gpio, int scl_pin,
		stm32_gpio_t *alert_gpio, int alert_pin,
		float shunt_res) {

	bq76940.alert_gpio = alert_gpio;
	bq76940.alert_pin = alert_pin;
	bq76940.shunt_res = shunt_res;

	palSetPadMode(alert_gpio, alert_pin, PAL_MODE_INPUT);

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
	
	// write 0x19 to CC_CFG according to datasheet page 39
	error |= write_reg(BQ_CC_CFG, 0x19);

	error |= gainRead(&bq76940.gain);
	error |= offsetRead(&bq76940.offset);

	//OverVoltage and UnderVoltage thresholds
	write_reg(BQ_OV_TRIP, tripVoltage(4.25));
	write_reg(BQ_UV_TRIP, tripVoltage(2.80));

	// Short Circuit Protection at 300 A
	error |= write_reg(BQ_PROTECT1, BQ_SCP_70us |  BQ_SCP_155mV);
	
	// Over Current Protection at 200 A
	error |= write_reg(BQ_PROTECT2, BQ_OCP_640ms | BQ_OCP_100mV);
	
	// Overvoltage and UnderVoltage delays
	error |= write_reg(BQ_PROTECT3, BQ_UV_DELAY_1s | BQ_OV_DELAY_1s);

	// clear SYS-STAT for init
	write_reg(BQ_SYS_STAT,0xFF);

	// doublecheck if bq is ready
	if(read_reg(BQ_SYS_STAT) & SYS_STAT_DEVICE_XREADY){
		// DEVICE_XREADY is set
		// write 1 in DEVICE_XREADY to clear it
		error |= write_reg(BQ_SYS_STAT, SYS_STAT_DEVICE_XREADY);
		// check again
		if(read_reg(BQ_SYS_STAT) & SYS_STAT_DEVICE_XREADY) return 1; // ERROR_XREADY;
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

	return error; // 0 if successful
}

static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("BQ76940");

	while (!chThdShouldTerminateX()) {
		m_i2c.has_error = 0;

		chThdSleepMilliseconds(20);

		if (READ_ALERT() ) {
			uint8_t sys_stat = read_reg(BQ_SYS_STAT);
			write_reg(BQ_SYS_STAT,0xFF);
			
			// time to read the cells
			read_cell_voltages(m_v_cell); 	//read cell voltages
			//chThdSleepMilliseconds(250); 	// time to read the thermistors
			//read_temp(measurement_temp);  	//read temperature
			chThdSleepMilliseconds(30);
			balance(m_discharge_state);
			iin_measure(&i_in);	

			// Report fault codes
			if ( sys_stat & SYS_STAT_DEVICE_XREADY ) {
				//handle error
			}
			if ( sys_stat & SYS_STAT_OVRD_ALERT ) {
				//handle error
			}
			if ( sys_stat & SYS_STAT_UV ) {
				bms_if_fault_report(FAULT_CODE_CELL_UNDERVOLTAGE);
			}
			if ( sys_stat & SYS_STAT_OV ) {
				bms_if_fault_report(FAULT_CODE_CELL_OVERVOLTAGE);
			}
			if ( sys_stat & SYS_STAT_SCD ) {
				bms_if_fault_report(FAULT_CODE_DISCHARGE_SHORT_CIRCUIT);
			}
			if ( sys_stat & SYS_STAT_OCD ) {
				bms_if_fault_report(FAULT_CODE_DISCHARGE_OVERCURRENT);
			}
		}
	}
}

uint8_t write_reg(uint8_t reg, uint16_t val) {
	m_i2c.has_error = 0;
	uint8_t txbuf[3];
	uint8_t buff[4];

	buff[0] = BQ_I2C_ADDR << 1;
	buff[1] = reg;
	buff[2] = val;

	txbuf[0] = reg;
	txbuf[1] = val;
	uint8_t key = 0x7;
	txbuf[2] = CRC8(buff, 3, key);
	i2c_bb_tx_rx(&m_i2c, BQ_I2C_ADDR, txbuf, 3, 0, 0);

	return 0;
}

uint8_t read_reg(uint8_t reg){
	uint8_t data;
 	i2c_bb_tx_rx(&m_i2c, BQ_I2C_ADDR, &reg, 1, &data, 2);
 	return data;
}

int8_t gainRead(float *gain){
	int8_t error = 0;
	uint8_t reg1 = read_reg(BQ_ADCGAIN1);
	uint8_t reg2 = read_reg(BQ_ADCGAIN2);

	reg1 &= 0x0C;
	*gain = (365.0 + ((reg1 << 1) | (reg2 >> 5)));

	if ((*gain < 365) | (*gain > 396)) {
		error = BQ76940_FAULT_GAIN;
	}
	return error;
}

// convert a voltage into the format used by the trip registers
uint8_t tripVoltage(float threshold) {
	uint16_t reg_val = (uint16_t)(threshold * 1000.0);
	reg_val -= bq76940.offset;
	reg_val *= 1000;
	reg_val /= bq76940.gain;
	reg_val++;
	reg_val >>= 4;
	return ((uint8_t)reg_val);
}

int8_t offsetRead(float *offset){
	int8_t error = 0;
	*offset = ((float)read_reg(BQ_ADCOFFSET) / 1000.0);

	if( (*offset < 0x00) | (*offset > 0xFF) ) {
		error = BQ76940_FAULT_OFFSET;
	}
	return error;
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

static void read_cell_voltages(float *m_v_cell) {
	float 	cell_voltages[MAX_CELL_NUM];

	for (int i=0; i<MAX_CELL_NUM; i++) {
		uint16_t VCx_lo = read_reg(BQ_VC1_LO + i * 2);
		uint16_t VCx_hi = read_reg(BQ_VC1_HI + i * 2);
		cell_voltages[i] = (((((float)(VCx_lo | (VCx_hi << 8))) * bq76940.gain) / 1e6)) + bq76940.offset;
	}
	
	// For 14s setups, handle the special case of cell 14 connected to VC15
	cell_voltages[13] = cell_voltages[14];
	
	memcpy( m_v_cell, cell_voltages, sizeof(cell_voltages) );
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

void iin_measure(float *i_in ) {
	uint8_t CC_hi = read_reg(BQ_CC_HI);
	uint8_t CC_lo = read_reg(BQ_CC_LO);
	int16_t CC_reg = (int16_t)(CC_lo | CC_hi << 8);
	
	*(i_in) = (float)CC_reg * 0.00844 ;/// bq76940.shunt_res;

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
