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

#ifndef PWR_H_
#define PWR_H_

#include "conf_general.h"
#include "bq76940.h"

#define BQ_CHG_ON()				palSetLine(LINE_BQ_CHG_EN)
#define BQ_CHG_OFF()			palClearLine(LINE_BQ_CHG_EN)

#define BQ_CP_ON()				palSetLine(LINE_BQ_CP_EN)
#define BQ_CP_OFF()				palClearLine(LINE_BQ_CP_EN)

#define BQ_DSG_ON()				palSetLine(LINE_BQ_DSG_EN)
#define BQ_DSG_OFF()			palClearLine(LINE_BQ_DSG_EN)

#define BQ_PMON_ON()			palSetLine(LINE_BQ_PMON_EN)
#define BQ_PMON_OFF()			palClearLine(LINE_BQ_PMON_EN)

#define BQ_PCHG_ON()			palSetLine(LINE_BQ_PCHG_EN)
#define BQ_PCHG_OFF()			palClearLine(LINE_BQ_PCHG_EN)

#define TEMP_MEASURE_ON()		palSetLine(LINE_TEMP_0_EN); palSetLine(LINE_TEMP_1_EN); \
								palSetLine(LINE_TEMP_2_EN); palSetLine(LINE_TEMP_3_EN); \
								palSetLine(LINE_TEMP_4_EN); palSetLine(LINE_TEMP_5_EN); \
								palSetLine(LINE_TEMP_6_EN);
#define TEMP_MEASURE_OFF()		palClearLine(LINE_TEMP_0_EN); palClearLine(LINE_TEMP_1_EN); \
								palClearLine(LINE_TEMP_2_EN); palClearLine(LINE_TEMP_3_EN); \
								palClearLine(LINE_TEMP_4_EN); palClearLine(LINE_TEMP_5_EN);

void pwr_init(void);
float pwr_get_vcharge(void);
float pwr_get_vfuse(void);
float pwr_get_iin(void);
float pwr_get_temp(int sensor);

#endif /* PWR_H_ */
