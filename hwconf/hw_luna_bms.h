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

#ifndef HWCONF_HW_LUNA_BMS_H_
#define HWCONF_HW_LUNA_BMS_H_

#include "bq76940.h"

#define HW_NAME					"luna_bms"

// HW-specific
#define HW_INIT_HOOK()			palSetLineMode(LINE_CURR_MEASURE_EN, PAL_MODE_OUTPUT_PUSHPULL)
#define HW_HAS_BQ76940

#define CURR_MEASURE_ON()		palClearLine(LINE_CURR_MEASURE_EN)
#define CURR_MEASURE_OFF()		palSetLine(LINE_CURR_MEASURE_EN)

// Macros
#define CHARGE_ENABLE()				bq_charge_enable(); bq_discharge_enable()
#define CHARGE_DISABLE()			bq_charge_disable(); bq_discharge_disable()
#define HW_GET_TEMP(sensors)		bq_get_temp(sensors)
#define HW_SET_DSC(cell, set)		bq_set_dsc(cell, set)
#define HW_GET_DSC(cell)			bq_get_dsc(cell)
#define HW_LAST_CELL_VOLTAGE(cell)	bq_last_cell_voltage(cell)
#define HW_GET_V_CHARGE()			bq_last_pack_voltage()	//until we implement charge voltage measurement in hw

// Settings
#define HW_CELLS_SERIES			14
#define HW_SHUNT_RES			(0.0005)
#define HW_SHUNT_AMP_GAIN		(20.0)
#define V_REG					3.3
#define R_CHARGE_TOP			(520e3 + 2.5e3 + 100.0)
#define R_CHARGE_BOTTOM			(10e3)
#define AFE						bq76940

// LEDs
#define LINE_LED_RED			PAL_LINE(GPIOA, 0)
#define LINE_LED_GREEN			PAL_LINE(GPIOA, 1)

// BQ76200
#define LINE_BQ_CHG_EN			PAL_LINE(GPIOB, 15)
#define LINE_BQ_CP_EN			PAL_LINE(GPIOB, 13)
#define LINE_BQ_DSG_EN			PAL_LINE(GPIOB, 14)
#define LINE_BQ_PMON_EN			PAL_LINE(GPIOB, 11)
#define LINE_BQ_PCHG_EN			PAL_LINE(GPIOB, 12)

// LTC6813
#define LINE_LTC_CS				PAL_LINE(GPIOA, 4)
#define LINE_LTC_SCLK			PAL_LINE(GPIOA, 5)
#define LINE_LTC_MISO			PAL_LINE(GPIOA, 6)
#define LINE_LTC_MOSI			PAL_LINE(GPIOA, 7)
#define LTC_GPIO_CURR_MON		5

// CAN
#define LINE_CAN_RX				PAL_LINE(GPIOB, 8)
#define LINE_CAN_TX				PAL_LINE(GPIOB, 9)
#define HW_CAN_DEV				CAND1
#define HW_CAN_AF				9

// BQ76940
#define BQ76940_SDA_GPIO		GPIOB
#define BQ76940_SDA_PIN			11
#define BQ76940_SCL_GPIO		GPIOB
#define BQ76940_SCL_PIN			10
#define BQ76940_ALERT_GPIO		GPIOA
#define BQ76940_ALERT_PIN		2

// Analog
#define LINE_V_CHARGE			PAL_LINE(GPIOC, 2)
#define LINE_CURRENT			PAL_LINE(GPIOC, 3)
#define LINE_TEMP_0				PAL_LINE(GPIOC, 1)
#define LINE_TEMP_1				PAL_LINE(GPIOC, 0)
#define LINE_TEMP_2				PAL_LINE(GPIOC, 4)
#define LINE_TEMP_3				PAL_LINE(GPIOC, 5)
#define LINE_TEMP_4				PAL_LINE(GPIOB, 0)
#define LINE_TEMP_5				PAL_LINE(GPIOB, 1)

#define LINE_TEMP_0_EN			PAL_LINE(GPIOB, 5)
#define LINE_TEMP_1_EN			PAL_LINE(GPIOC, 8)
#define LINE_TEMP_2_EN			PAL_LINE(GPIOC, 9)
#define LINE_TEMP_3_EN			PAL_LINE(GPIOC, 10)
#define LINE_TEMP_4_EN			PAL_LINE(GPIOC, 11)
#define LINE_TEMP_5_EN			PAL_LINE(GPIOB, 2)

#define NTC_RES(adc)			(10000.0 / ((4095.0 / (float)adc) - 1.0))
#define NTC_TEMP(adc)			(1.0 / ((logf(NTC_RES(adc) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

// TODO: Take highest of all temp sensors
#define HW_TEMP_CELLS_MAX()		bms_if_get_temp(2)

// ADC Channels
#define ADC_CH_V_CHARGE			ADC_CHANNEL_IN3
#define ADC_CH_CURRENT			ADC_CHANNEL_IN4
#define ADC_CH_TEMP0			ADC_CHANNEL_IN2 // Next to STM32
#define ADC_CH_TEMP1			ADC_CHANNEL_IN1 // Leftmost edge
#define ADC_CH_TEMP2			ADC_CHANNEL_IN13 // Center of cells
#define ADC_CH_TEMP3			ADC_CHANNEL_IN14 // Lower right center
#define ADC_CH_TEMP4			ADC_CHANNEL_IN15 // On wire
#define ADC_CH_TEMP5			ADC_CHANNEL_IN16 // Top PCB center

// Other
#define LINE_CURR_MEASURE_EN	PAL_LINE(GPIOB, 6)

#endif /* HWCONF_HW_LUNA_BMS_H_ */
