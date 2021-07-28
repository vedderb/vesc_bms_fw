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


// Private variables
static i2c_bb_state  m_i2c;
volatile uint8_t gain_offset = 0;
static volatile float m_v_cell[14];
static volatile float measurement_temp[5];
static volatile float i_in = 0.0;
static volatile bool m_discharge_state[18] = {false};
// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
void read_temp(volatile float *measurement_temp);
void iin_measure(float *value_iin);
static void write_reg(uint8_t reg, uint16_t val);
//static bool read_reg_group(uint16_t cmd, uint8_t *buffer);
static void read_cell_voltages(volatile float *m_v_cell);
uint8_t read_reg(uint8_t reg);//i2c_bb_state *s
uint8_t CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
void balance(volatile bool *m_discharge_state);


void bq76940_init(
		stm32_gpio_t *sda_gpio, int sda_pin,
		stm32_gpio_t *scl_gpio, int scl_pin) {

	memset(&m_i2c, 0, sizeof(i2c_bb_state));
	m_i2c.sda_gpio = sda_gpio;
	m_i2c.sda_pin = sda_pin;
	m_i2c.scl_gpio = scl_gpio;
	m_i2c.scl_pin = scl_pin;

	i2c_bb_init(&m_i2c);

	volatile uint8_t data = 0;

	chThdSleepMilliseconds(30);
	write_reg(SYS_STAT, 0xBF);

	chThdSleepMilliseconds(30);
	write_reg(SYS_CTRL1,0x18); //

	chThdSleepMilliseconds(30);
	write_reg(SYS_CTRL2,0x40);

	chThdSleepMilliseconds(30);
	write_reg(CC_CFG, 0x19);

	chThdSleepMilliseconds(30);
	write_reg(CELLBAL1, 0x00);

	chThdSleepMilliseconds(30);
	write_reg(CELLBAL2, 0x00);

	chThdSleepMilliseconds(30);
	write_reg(CELLBAL3, 0x00);

	chThdSleepMilliseconds(30);
	write_reg(SYS_CTRL2, 0x60);

	chThdSleepMilliseconds(30);
	write_reg(PROTECT1, 0x00);

	chThdSleepMilliseconds(30);
	write_reg(PROTECT2, 0x0F);

	chThdSleepMilliseconds(30);
	write_reg(PROTECT3, 0xF0);

	chThdSleepMilliseconds(30);
	write_reg(OV_TRIP, 0xFF);

	chThdSleepMilliseconds(30);
	write_reg(UV_TRIP, 0x00);

	chThdSleepMilliseconds(30);
	data=read_reg(ADCOFFSET); //read the offset register//&m_i2c,
	gain_offset=data;

	chThdSleepMilliseconds(30);
	read_reg(ADCGAIN1); //read the offset register//(&m_i2c,

	DISCHARGE_ON();

	chThdCreateStatic(sample_thread_wa, sizeof(sample_thread_wa), LOWPRIO, sample_thread, NULL);
}



static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("BQ76940");

	while (!chThdShouldTerminateX()) {   //for(;;) {
		m_i2c.has_error = 0;

		chThdSleepMilliseconds(250); 	// time to read the cells
		read_cell_voltages(m_v_cell); 	//read cell voltages
		chThdSleepMilliseconds(250); 	// time to read the thermistors
		read_temp(measurement_temp);  	//read temperature
		chThdSleepMilliseconds(30);
		//write_reg(CELLBAL1, 0x01);		//Balance
		//m_discharge_state[0] = true;
		balance(m_discharge_state);
		//m_discharge_state[18] = {false};
		//iin_measure(&i_in);		//measure current

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

uint8_t read_reg(uint8_t reg){//i2c_bb_state *s

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

static void read_cell_voltages(volatile float *m_v_cell) {
	uint16_t buffer[28];

	//Cell 1
	buffer[0]=read_reg(VC1_LO); //(&m_i2c,
	buffer[1]=read_reg(VC1_HI); //(&m_i2c,
	*((m_v_cell)+0) = (((float)(buffer[0] | (buffer[1]<<8)))*379)/1e6;

	//Cell 2
	buffer[2]=read_reg(VC2_LO); //(&m_i2c,
	buffer[3]=read_reg(VC2_HI); //(&m_i2c,
	*((m_v_cell)+1) = (((float)(buffer[2] | (buffer[3]<<8)))*379)/1e6;

	//Cell 3
	buffer[4]=read_reg(VC3_LO); //(&m_i2c,
	buffer[5]=read_reg(VC3_HI); //(&m_i2c,
	*((m_v_cell)+2) = (((float)(buffer[4] | (buffer[5]<<8)))*379)/1e6;

	//Cell 4
	buffer[6]=read_reg(VC4_LO); //(&m_i2c,
	buffer[7]=read_reg(VC4_HI); //(&m_i2c,
	*((m_v_cell)+3) = (((float)(buffer[6] | (buffer[7]<<8)))*379)/1e6;

	//Cell 5
	buffer[8]=read_reg(VC5_LO); //(&m_i2c,
	buffer[9]=read_reg(VC5_HI); //(&m_i2c,
	*((m_v_cell)+4) = (((float)(buffer[8] | (buffer[9]<<8)))*379)/1e6;

	//Cell 6
	buffer[10]=read_reg(VC6_LO); //(&m_i2c,
	buffer[11]=read_reg(VC6_HI); //
	*((m_v_cell)+5) = (((float)(buffer[10] | (buffer[11]<<8)))*379)/1e6;

	//Cell 7
	buffer[12]=read_reg(VC7_LO); //
	buffer[13]=read_reg(VC7_HI); //
	*((m_v_cell)+6) = (((float)(buffer[12] | (buffer[13]<<8)))*379)/1e6;

	//Cell 8
	buffer[14]=read_reg(VC8_LO); //
	buffer[15]=read_reg(VC8_HI); //
	*((m_v_cell)+7) = (((float)(buffer[14] | (buffer[15]<<8)))*379)/1e6;

	//Cell 9
	buffer[16]=read_reg(VC9_LO); //
	buffer[17]=read_reg(VC9_HI); //
	*((m_v_cell)+8) = (((float)(buffer[16] | (buffer[17]<<8)))*379)/1e6;

	//Cell 10
	buffer[18]=read_reg(VC10_LO); //
	buffer[19]=read_reg(VC10_HI); //
	*((m_v_cell)+9) = (((float)((buffer[18]) | (buffer[19]<<8)))*379)/1e6;

	//Cell 11
	buffer[20]=read_reg(VC11_LO); //
	buffer[21]=read_reg(VC11_HI); //
	*((m_v_cell)+10) = (((float)((buffer[20]) | (buffer[21]<<8)))*379)/1e6;

	//Cell 12
	buffer[22]=read_reg(VC12_LO); //
	buffer[23]=read_reg(VC12_HI); //
	*((m_v_cell)+11) = (((float)((buffer[22]) | (buffer[23]<<8)))*379)/1e6;

	//Cell 13
	buffer[24]=read_reg(VC13_LO); // warning!!!
	buffer[25]=read_reg(VC13_HI); // warning!!!
	*((m_v_cell)+12) = (((float)((buffer[24]) | (buffer[25]<<8)))*379)/1e6;

	//Cell 14
	buffer[26]=read_reg(VC15_LO); //
	buffer[27]=read_reg(VC15_HI); //
	*((m_v_cell)+13) = (((float)((buffer[26]) | (buffer[27]<<8)))*379)/1e6;
}

float bq_last_cell_voltage(int cell) {
	if (cell < 0 || cell > 17) {
		return -1.0;
	}

	return m_v_cell[cell];
}

float bq_last_pack_voltage(void) {
	return 27.6;//m_v_pack;
}

void read_temp(volatile float *measurement_temp) {
	uint16_t 	buffer[6];
	float		vtsx=0, R_ts=0;

	buffer[0]=read_reg(TS1_HI); //
	buffer[1]=read_reg(TS1_LO); //
	vtsx = (float)((buffer[0]) | (buffer[1]<<8));
	R_ts = (vtsx*1e4)/(vtsx-3.3);
	*((measurement_temp)+0) = (1.0 / ((logf(R_ts / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);

	buffer[2]=read_reg(TS2_HI); //
	buffer[3]=read_reg(TS2_LO); //
	//vtsx = ((buffer[2]) | (buffer[3]<<8));
	//R_ts = (vtsx*1e4)/(vtsx-3.3);
	*((measurement_temp)+1) = 10;//(1.0 / (((logf(R_ts / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);

	buffer[4]=read_reg(TS3_HI); //
	buffer[5]=read_reg(TS3_LO); //
	//vtsx = ((buffer[4]) | (buffer[5]<<8));
	//R_ts = (vtsx*1e4)/(vtsx-3.3);
	*((measurement_temp)+2) = 10;//(1.0 / (((logf(20000) / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);

	*((measurement_temp)+3) = 10;

	*((measurement_temp)+4) = 10;

//#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15)
}

float get_temp(int sensor){
	if (sensor < 0 || sensor >= 6){//HW_ADC_TEMP_SENSORS) {
			return -1.0;
	}

	return measurement_temp[sensor];
}

void charge_on(void){

	write_reg(SYS_CTRL2, 0x01);

	return;
}

void iin_measure(float *i_in ){
	uint8_t buffer[2] = {0,0};


	chThdSleepMilliseconds(30);
	buffer[0]=read_reg(CC_HI);
	chThdSleepMilliseconds(30);
	buffer[1]=read_reg(CC_LO);

	*(i_in)=(float)((buffer[0]) | (buffer[1]<<8));

	return;
}

float get_current(void){
	return i_in ;
}

void DISCHARGE_ON(void){
	uint8_t	data = 0;

	data = read_reg(SYS_CTRL2); //read the offset register
	data |= data << 1;
	chThdSleepMilliseconds(30);
	write_reg(SYS_CTRL2, data);//0x42

	return;
}

void DISCHARGE_OFF(void){
	uint8_t	data = 0;

	data = read_reg(SYS_CTRL2); //read the offset register
	data |= data << 0;
	chThdSleepMilliseconds(30);
	write_reg(SYS_CTRL2, data); // 0x40

	return;
}

void CHARGE_ON(void){

	chThdSleepMilliseconds(30);
	write_reg(SYS_CTRL2, 0x41);

	return;
}

void CHARGE_OFF(void){

	chThdSleepMilliseconds(30);
	write_reg(SYS_CTRL2, 0x40);

	return;
}

void bq_set_dsc(int cell, bool set) {
	if (cell < 0 || cell > 17) {
		return;
	}

	m_discharge_state[cell] = set;
}

bool bq_get_dsc(int cell) {
	if (cell < 0 || cell > 17) {
		return false;
	}

	return m_discharge_state[cell];
}

void balance(volatile bool *m_discharge_state) {
	uint8_t buffer[3]= {0 ,0 ,0 };

	buffer[0] |= m_discharge_state[0] ? 1 << buffer[0]  : 0;
	buffer[0] |= m_discharge_state[1] ? 2 << buffer[0]  : 0;
	buffer[0] |= m_discharge_state[2] ? 3 << buffer[0]  : 0;
	buffer[0] |= m_discharge_state[3] ? 4 << buffer[0]  : 0;
	buffer[0] |= m_discharge_state[4] ? 5 << buffer[0]  : 0;
	buffer[0] |= m_discharge_state[5] ? 6 << buffer[0]  : 0;
	chThdSleepMilliseconds(30);
	write_reg(CELLBAL1, buffer[0]);

	buffer[1] |= m_discharge_state[6] ? 1 << buffer[1]  : 0;
	buffer[1] |= m_discharge_state[7] ? 2 << buffer[1]  : 0;
	buffer[1] |= m_discharge_state[8] ? 3 << buffer[1]  : 0;
	buffer[1] |= m_discharge_state[9] ? 4 << buffer[1]  : 0;
	buffer[1] |= m_discharge_state[10] ? 5 << buffer[1]  : 0;
	buffer[1] |= m_discharge_state[11] ? 6 << buffer[1]  : 0;
	chThdSleepMilliseconds(30);
	write_reg(CELLBAL2, buffer[1]);

	buffer[2] |= m_discharge_state[12] ? 1 << buffer[2]  : 0;
	buffer[2] |= m_discharge_state[13] ? 2 << buffer[2]  : 0;
	buffer[2] |= m_discharge_state[14] ? 3 << buffer[2]  : 0;
	buffer[2] |= m_discharge_state[15] ? 4 << buffer[2]  : 0;
	chThdSleepMilliseconds(30);
	write_reg(CELLBAL3, buffer[2]);

	return;
}
