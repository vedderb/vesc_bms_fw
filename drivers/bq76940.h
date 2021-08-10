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


//Macros
#define CHARGE 		charge_on()
#define NUM_REG		55

#define SYS_STAT	0x00
#define CELLBAL1	0x01
#define CELLBAL2	0x02
#define CELLBAL3	0x03
#define SYS_CTRL1	0x04
#define SYS_CTRL2	0x05
#define PROTECT1	0x06
#define PROTECT2 	0x07
#define PROTECT3	0x08
#define OV_TRIP		0x09
#define UV_TRIP		0x0A
#define CC_CFG		0x0B
#define VC1_HI		0x0C
#define VC1_LO		0x0D
#define	VC2_HI		0x0E
#define VC2_LO		0x0F
#define VC3_HI		0x10
#define VC3_LO		0x11
#define VC4_HI		0x12
#define VC4_LO		0x13
#define VC5_HI		0x14
#define VC5_LO		0x15
#define VC6_HI		0x16
#define VC6_LO		0x17
#define VC7_HI		0x18
#define VC7_LO		0x19
#define VC8_HI		0x1A
#define VC8_LO		0x1B
#define VC9_HI		0x1C
#define VC9_LO		0x1D
#define VC10_HI		0x1E
#define VC10_LO		0x1F
#define VC11_HI		0x20
#define VC11_LO		0x21
#define VC12_HI		0x22
#define VC12_LO		0x23
#define VC13_HI		0x24
#define VC13_LO		0x25
#define VC14_HI		0x26
#define VC14_LO		0x27
#define VC15_HI		0x28
#define VC15_LO		0x29
#define BAT_HI		0x2A
#define BAT_LO		0x2B
#define TS1_HI		0x2C
#define TS1_LO		0x2D
#define TS2_HI		0x2E
#define TS2_LO		0x2F
#define TS3_HI		0x30
#define TS3_LO		0x31
#define CC_HI		0x32
#define CC_LO		0x33
#define ADCGAIN1	0x50
#define ADCOFFSET	0x51
#define ADCGAIN2	0x59

// Functions
void bq76940_init(stm32_gpio_t *sda_gpio, int sda_pin,
				  stm32_gpio_t *scl_gpio, int scl_pin);
//float hdc1080_get_hum(void);
//float hdc1080_get_temp(void);
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
#endif /* BQ76940_H_ */
