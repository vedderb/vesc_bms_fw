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

#ifndef LTC_H_
#define LTC_H_

#include "conf_general.h"


// Functions
#ifndef AFE
float ltc_last_pack_voltage(void);
float ltc_last_cell_voltage(int cell);
bool ltc_get_dsc(int cell);
#endif
void ltc_init(void);
float ltc_last_pu_diff_voltage(int cell);
float ltc_last_temp(void);

void ltc_set_dsc(int cell, bool set);
float ltc_last_gpio_voltage(int gpio);
void ltc_sleep(void);

// Commands
#define LTC_ADCV					0x0260
#define LTC_ADOW					0x0228
#define LTC_ADSTAT					0x0468
#define LTC_ADCVSC					0x0467
#define LTC_ADAX					0x0460
#define LTC_MUTE					0x0028
#define LTC_UNMUTE					0x0029
#define LTC_PLADC					0x0714

// Read commands
#define LTC_RDCVA					0x0004
#define LTC_RDCVB					0x0006
#define LTC_RDCVC					0x0008
#define LTC_RDCVD					0x000A
#define LTC_RDCVE					0x0009
#define LTC_RDCVF					0x000B
#define LTC_RDAUXA					0x000C
#define LTC_RDAUXB					0x000E
#define LTC_RDAUXC					0x000D
#define LTC_RDAUXD					0x000F
#define LTC_RDSTATA					0x0010
#define LTC_RDSTATB					0x0012

// Write commands
#define LTC_WRCFGA					0x0001
#define LTC_WRCFGB					0x0024

// Command bit options
#define LTC_MD00					0x0000 // 422 Hz / 1 kHz
#define LTC_MD01					0x0080 // 27 kHz (fast) / 14 kHz
#define LTC_MD10					0x0100 // 7 kHz (normal) / 3 kHz
#define LTC_MD11					0x0180 // 26 Hz / 2 kHz
#define LTC_DCP						0x0010
#define LTC_CHG000					0x0000 // GPIO 1-5, 2nd ref, GPIO 6-9
#define LTC_CHG101					0x0005 // GPIO 5
#define LTC_PUP						0x0040

// Configuration bits
#define LTC_REFON					0x04
#define LTC_GPIO1					0x08
#define LTC_GPIO2					0x10
#define LTC_GPIO3					0x20
#define LTC_GPIO4					0x40
#define LTC_GPIO5					0x80


#endif /* LTC_H_ */



