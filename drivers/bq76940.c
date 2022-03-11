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
#include "stm32l4xx_hal.h"
#include "sleep.h"

#ifdef HW_HAS_BQ76940

#define MAX_CELL_NUM		15
#define BQ_I2C_ADDR			0x08

// Private variables
static i2c_bb_state  m_i2c;
static volatile float m_v_cell[MAX_CELL_NUM];
static volatile float measurement_temp[5];
static volatile float i_in = 0;
static volatile float v_bat = 0;
static volatile bool m_discharge_state[MAX_CELL_NUM] = {false};
static volatile bool status_pin_discharge = false;
static volatile bool status_pin_charge = false;
static volatile bool status = false;
static volatile float hw_shunt_res = 1.0;

typedef struct {
	i2c_bb_state m_i2c;
	stm32_gpio_t *alert_gpio;
	int alert_pin;
	stm32_gpio_t *lrd_gpio;
    int lrd_pin;
	float shunt_res;
	float gain;
	float offset;
} bq76940_t;
/*
static I2CConfig i2cfg1 = {
	     STM32_TIMINGR_PRESC(5U)  |            // 48MHz/6 = 8MHz I2CCLK.
	     STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
	     STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U),
	                    // Just scale bits 28..31 if not 8MHz clock
	    0,              // CR1
	    0,              // CR2
};
*/
static bq76940_t bq76940;

// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
void   print_state_SOC();
int8_t gainRead(float *gain);
int8_t current_discharge_protect_set(uint8_t time1,
								     uint8_t short_circuit_current,
									 uint8_t time2,
									 uint8_t overcurrent);
void toggle_pincharge();
void status_load_removal_discharge();
int8_t offsetRead(float *offset);
void read_temp(volatile float *measurement_temp);
void iin_measure(float *value_iin);
uint8_t write_reg(uint8_t reg, uint16_t val);
static void read_cell_voltages(float *m_v_cell);
static void read_v_batt(float *v_bat);
uint8_t read_reg(uint8_t reg);
uint8_t CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
void balance(volatile bool *m_discharge_state);
uint8_t tripVoltage(float voltage);

//Macros
#define READ_ALERT()						palReadPad(bq76940.alert_gpio, bq76940.alert_pin)
#define LOAD_REMOVAL_DISCHARGE()			palReadPad(bq76940.lrd_gpio, bq76940.lrd_pin)

uint8_t bq76940_init(
		stm32_gpio_t *sda_gpio, int sda_pin,
		stm32_gpio_t *scl_gpio, int scl_pin,
		stm32_gpio_t *alert_gpio, int alert_pin,
		stm32_gpio_t *lrd_gpio, int lrd_pin,
		float shunt_res) {

	palEnablePadEvent(GPIOA, 2U, PAL_EVENT_MODE_RISING_EDGE);

	bq76940.alert_gpio = alert_gpio;
	bq76940.alert_pin = alert_pin;
	bq76940.lrd_gpio = lrd_gpio;
	bq76940.lrd_pin = lrd_pin;
	bq76940.shunt_res = shunt_res;

	palSetPadMode(alert_gpio, alert_pin, PAL_MODE_INPUT);
	palSetPadMode(lrd_gpio, lrd_pin, PAL_MODE_INPUT_PULLDOWN);

	memset(&m_i2c, 0, sizeof(i2c_bb_state));
	m_i2c.sda_gpio = sda_gpio;
	m_i2c.sda_pin = sda_pin;
	m_i2c.scl_gpio = scl_gpio;
	m_i2c.scl_pin = scl_pin;

	i2c_bb_init(&m_i2c);
/*	i2cStart(&I2CD2, &i2cfg1);
	palSetPadMode(GPIOB, 10, PAL_STM32_MODE_ALTERNATE
							| PAL_MODE_OUTPUT_OPENDRAIN
							| PAL_STM32_ALTERNATE(4) );
	palSetPadMode(GPIOB, 11, PAL_STM32_MODE_ALTERNATE
							| PAL_MODE_OUTPUT_OPENDRAIN
							| PAL_STM32_ALTERNATE(4) );
*/
	uint8_t error = 0;

	// make sure the bq is booted up--->set TS1 to 3.3V and back to VSS
	// maybe set temp-mode for internal or external temp here

	// enable ADC and thermistors
	error |= write_reg(BQ_SYS_CTRL1, (ADC_EN | TS_ON));
	
	// check if ADC and thermistors is active
	error |= read_reg(BQ_SYS_CTRL1) & (ADC_EN | TS_ON);
	
	// write 0x19 to CC_CFG according to datasheet page 39
	error |= write_reg(BQ_CC_CFG, 0x19);

	error |= gainRead(&bq76940.gain);
	error |= offsetRead(&bq76940.offset);

	//OverVoltage and UnderVoltage thresholds
	write_reg(BQ_OV_TRIP, 0xC9);//tripVoltage(4.25));
	write_reg(BQ_UV_TRIP, tripVoltage(2.80));

	// Short Circuit Protection
	current_discharge_protect_set(BQ_SCP_70us, BQ_SCP_22mV,5,5);
	//error |= write_reg(BQ_PROTECT1, BQ_SCP_70us |  BQ_SCP_22mV);
	

	// Over Current Protection at 200 A
	error |= write_reg(BQ_PROTECT2, BQ_OCP_8ms | BQ_OCP_17mV);
	
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

	bq_discharge_enable();
	bq_charge_enable();

	chThdSleepMilliseconds(40);
	read_reg(BQ_SYS_STAT);

	/////////////////////////////////////////////////// provisional
    //if(status_pin_discharge){
	//	uint8_t data = read_reg(BQ_SYS_CTRL2);
	//	data = data | 0x03;
	//	write_reg(BQ_SYS_CTRL2, data);
	//}
	//else{
		//uint8_t data = read_reg(BQ_SYS_CTRL2);
		//data = (data & 0xFD);
		//write_reg(BQ_SYS_CTRL2, data);
	//}
    ////////////////////////////////////////////////////provisional

	chThdCreateStatic(sample_thread_wa, sizeof(sample_thread_wa), HIGHPRIO, sample_thread, NULL);

	return error; // 0 if successful
}

static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("BQ76940");

	while (!chThdShouldTerminateX()) {
		m_i2c.has_error = 0;

		chThdSleepMilliseconds(20);

		//provisional
		//wake up BQ76940
		write_reg(BQ_SYS_CTRL1, (ADC_EN | TS_ON));
		//

		//chThdSleepMilliseconds(250);
		palWaitPadTimeout(GPIOA, 2U, TIME_INFINITE);


		//provisional
//		if ( READ_ALERT() ) {
			//Clear Alert
			uint8_t sys_stat = read_reg(BQ_SYS_STAT);
			write_reg(BQ_SYS_STAT,0xFF);
			
			static uint8_t i = 0;
			if(i++ == 10){
				// time to read the cells
				read_temp(measurement_temp);  	//read temperature
				read_cell_voltages(m_v_cell); 	//read cell voltages
				read_v_batt(&v_bat);
				print_state_SOC();
				i = 0;
			}

			//toggle_pin_charge(); //if only if the status of pin charge toggle
			//status_load_removal_discharge();
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
	//}

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
	//i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, txbuf, 3, NULL, 0);

	return 0;
}

uint8_t read_reg(uint8_t reg){
	uint8_t data;
 	i2c_bb_tx_rx(&m_i2c, BQ_I2C_ADDR, &reg, 1, &data, 2);
	//i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, &reg, 1, &data, 2);
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
	return  v_bat;
}

void read_temp(volatile float *measurement_temp) {
	for(int i = 0 ; i < 3 ; i++){
		uint16_t BQ_TSx_hi = read_reg(BQ_TS1_HI + i * 2 );
		uint16_t BQ_TSx_lo = read_reg(BQ_TS1_LO + i * 2);
		float vtsx = (float)((BQ_TSx_hi << 8) | BQ_TSx_hi) * 0.000382;
		float R_ts = ( vtsx * 1e4 ) / ( 3.3 - vtsx );
		measurement_temp[i] = (1.0 / ((logf(R_ts / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);
	}

	return;

}

float bq_get_temp(int sensor){
	if (sensor < 0 || sensor >= 5){
			return -1.0;
	}

	return measurement_temp[sensor];
}

void iin_measure(float *i_in ) {
	uint16_t CC_hi = read_reg(BQ_CC_HI);
	uint16_t CC_lo = read_reg(BQ_CC_LO);
	int16_t CC_reg = (int16_t)(CC_lo | CC_hi << 8);
	
	*(i_in) = (float)CC_reg * lsb_unit_regCC / bq76940.shunt_res;

	return;
}

float bq_get_current(void){
	return i_in;
}

void bq_discharge_enable(void){
	status_pin_discharge = true;

	uint8_t data = read_reg(BQ_SYS_CTRL2);
	data = data | 0x02;
	write_reg(BQ_SYS_CTRL2, data);

	return;
}

void bq_discharge_disable(void){
	status_pin_discharge = false;

	uint8_t data = read_reg(BQ_SYS_CTRL2);
	data = data & 0xFD;
	write_reg(BQ_SYS_CTRL2, data);

	return;
}

void bq_charge_enable(void){
	status_pin_charge = true;

	uint8_t data = read_reg(BQ_SYS_CTRL2);
	data = data | 0x01;
	write_reg(BQ_SYS_CTRL2, data);

	return;
}

void bq_charge_disable(void){
	status_pin_charge = false;

	uint8_t data = read_reg(BQ_SYS_CTRL2);
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

void balance(volatile bool *m_discharge_state)
{
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
void toggle_pin_charge()
{

	if(!(status == status_pin_charge)){
		status = status_pin_charge;

		if(status_pin_charge){
			uint8_t data = read_reg(BQ_SYS_CTRL2);
			data = data | 0x01;
			write_reg(BQ_SYS_CTRL2, data);
		}
		else{
			uint8_t data = read_reg(BQ_SYS_CTRL2);
			data = data & 0xE2;
			write_reg(BQ_SYS_CTRL2, data);
		}
	}

	return;
}
int8_t current_discharge_protect_set(uint8_t time1,
									 uint8_t current_short_circuit,
									 uint8_t time2,
									 uint8_t overcurrent)
{

	uint8_t RSNS = 0;

	if(current_short_circuit == ( BQ_SCP_22mV | BQ_SCP_33mV | BQ_SCP_44mV | BQ_SCP_56mV |
				    			  BQ_SCP_67mV | BQ_SCP_78mV | BQ_SCP_89mV | BQ_SCP_100mV )){
		RSNS = 0x00;
	}
	if(current_short_circuit == ( BQ_SCP_44mV | BQ_SCP_67mV | BQ_SCP_89mV | BQ_SCP_111mV |
				                  BQ_SCP_133mV | BQ_SCP_155mV | BQ_SCP_178mV | BQ_SCP_200mV )){
		RSNS = 0x80;
	}
	else{
		RSNS = 0x00;
		current_short_circuit = BQ_SCP_22mV;
		time1 = BQ_SCP_70us;
	}

	write_reg(BQ_PROTECT1, ( time1 | (RSNS | current_short_circuit) ) );


	if(overcurrent == (BQ_OCP_8mV | BQ_OCP_11mV | BQ_OCP_14mV | BQ_OCP_17mV | BQ_OCP_19mV |
			          BQ_OCP_22mV | BQ_OCP_25mV | BQ_OCP_28mV | BQ_OCP_31mV | BQ_OCP_33mV |
					  BQ_OCP_36mV | BQ_OCP_39mV | BQ_OCP_39mV | BQ_OCP_42mV | BQ_OCP_44mV |
					  BQ_OCP_47mV | BQ_OCP_50mV)){
		write_reg(BQ_PROTECT2, ( time2 | (RSNS | overcurrent) ) );
	}
	else{
		write_reg(BQ_PROTECT2, ( BQ_OCP_8ms | (RSNS | BQ_OCP_8mV) ) );
	}


	return 0;
}

void status_load_removal_discharge()
{

	if(LOAD_REMOVAL_DISCHARGE()){
		//FAULT!
		bms_if_fault_report(FAULT_CODE_CELL_UNDERVOLTAGE);
		commands_printf("SHORT CIRCUIT! LOAD CONNECT");
		//time to wait (2min)
		chThdSleepMilliseconds(6000);
		//Can turn on the big mosfet?
		if(!(LOAD_REMOVAL_DISCHARGE())) bq_discharge_enable();
		}

}

static void read_v_batt(float *v_bat){
	uint16_t BAT_hi = read_reg(BQ_BAT_HI);
	uint16_t BAT_lo = read_reg(BQ_BAT_LO);

	*v_bat = (float)(((uint16_t)(BAT_lo | BAT_hi << 8)) * lsb_unit_regVbat )-(14 * bq76940.offset);
}

void   print_state_SOC()
{
//Provisional!
 return 0;
}

void sleep_bq76940()
{
	write_reg(BQ_SYS_CTRL1, (ADC_DIS | TS_ON));
	//write_reg(BQ_SYS_CTRL2, CC_DIS);
	//Shutdown everything frontend
	//write_reg(BQ_SYS_CTRL1, 0x01);
	//write_reg(BQ_SYS_CTRL1, 0x02);

}
#endif
