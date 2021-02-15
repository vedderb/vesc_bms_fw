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

#ifndef HWCONF_HW_H_
#define HWCONF_HW_H_

#include <math.h>

#include HW_HEADER

// Default configuration overrides
#ifndef CONF_CELL_NUM
#define CONF_CELL_NUM 			HW_CELLS_SERIES
#endif

#ifndef CONF_CELL_FIRST_INDEX
#define CONF_CELL_FIRST_INDEX 	0
#endif

#ifndef CONF_EXT_SHUNT_RES
#define CONF_EXT_SHUNT_RES		HW_SHUNT_RES
#endif

#ifndef CONF_EXT_SHUNT_GAIN
#define CONF_EXT_SHUNT_GAIN		HW_SHUNT_AMP_GAIN
#endif

// Macros
#ifndef LED_OFF
#define LED_OFF(led)			palClearLine(led)
#endif
#ifndef LED_ON
#define LED_ON(led)				palSetLine(led)
#endif
#define LED_TOGGLE(led)			palToggleLine(led)

#ifndef HW_DEFAULT_ID
#define HW_DEFAULT_ID			(CONF_CONTROLLER_ID >= 0 ? CONF_CONTROLLER_ID : hw_id_from_uuid())
#endif

#ifndef HW_ADC_TEMP_SENSORS
#define HW_ADC_TEMP_SENSORS		6
#endif

#ifndef HW_INIT_HOOK
#define HW_INIT_HOOK()
#endif

#ifndef LINE_CAN_EN
#define HW_CAN_ON()
#define HW_CAN_OFF()
#endif

#ifndef LINE_CURR_MEASURE_EN
#define CURR_MEASURE_ON()
#define CURR_MEASURE_OFF()
#endif

// Functions
uint8_t hw_id_from_uuid(void);

#endif /* HWCONF_HW_H_ */
