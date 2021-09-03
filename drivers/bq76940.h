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

#ifndef BQ76940_H_
#define BQ76940_H_


// SYS_STATUS bits
#define NUM_REG			55
#define ADC_EN			0x10
#define CC_EN			0x40
#define DEVICE_XREADY	0x20

// Register address
#define BQ_SYS_STAT		0x00
#define BQ_CELLBAL1		0x01
#define BQ_CELLBAL2		0x02
#define BQ_CELLBAL3		0x03
#define BQ_SYS_CTRL1	0x04
#define BQ_SYS_CTRL2	0x05
#define BQ_PROTECT1		0x06
#define BQ_PROTECT2 	0x07
#define BQ_PROTECT3		0x08
#define BQ_OV_TRIP		0x09
#define BQ_UV_TRIP		0x0A
#define BQ_CC_CFG		0x0B
#define BQ_VC1_HI		0x0C
#define BQ_VC1_LO		0x0D
#define	BQ_VC2_HI		0x0E
#define BQ_VC2_LO		0x0F
#define BQ_VC3_HI		0x10
#define BQ_VC3_LO		0x11
#define BQ_VC4_HI		0x12
#define BQ_VC4_LO		0x13
#define BQ_VC5_HI		0x14
#define BQ_VC5_LO		0x15
#define BQ_VC6_HI		0x16
#define BQ_VC6_LO		0x17
#define BQ_VC7_HI		0x18
#define BQ_VC7_LO		0x19
#define BQ_VC8_HI		0x1A
#define BQ_VC8_LO		0x1B
#define BQ_VC9_HI		0x1C
#define BQ_VC9_LO		0x1D
#define BQ_VC10_HI		0x1E
#define BQ_VC10_LO		0x1F
#define BQ_VC11_HI		0x20
#define BQ_VC11_LO		0x21
#define BQ_VC12_HI		0x22
#define BQ_VC12_LO		0x23
#define BQ_VC13_HI		0x24
#define BQ_VC13_LO		0x25
#define BQ_VC14_HI		0x26
#define BQ_VC14_LO		0x27
#define BQ_VC15_HI		0x28
#define BQ_VC15_LO		0x29
#define BQ_BAT_HI		0x2A
#define BQ_BAT_LO		0x2B
#define BQ_TS1_HI		0x2C
#define BQ_TS1_LO		0x2D
#define BQ_TS2_HI		0x2E
#define BQ_TS2_LO		0x2F
#define BQ_TS3_HI		0x30
#define BQ_TS3_LO		0x31
#define BQ_CC_HI		0x32
#define BQ_CC_LO		0x33
#define BQ_ADCGAIN1		0x50
#define BQ_ADCOFFSET	0x51
#define BQ_ADCGAIN2		0x59

// Functions
uint8_t bq76940_init(stm32_gpio_t *sda_gpio, int sda_pin,
				  	 stm32_gpio_t *scl_gpio, int scl_pin,
					 float shunt_res);
void bq_set_dsc(int cell, bool set);
bool bq_get_dsc(int cell);
float bq_last_pack_voltage(void);
float bq_last_cell_voltage(int cell);
#ifdef AFE
float get_temp(int sensor);
#endif
float get_current(void);
void DISCHARGE_ON(void);
void DISCHARGE_OFF(void);
void CHARGE_ON(void);
void CHARGE_OFF(void);
bool ltc_get_dsc(int cell);
#endif /* BQ76940_H_ */
