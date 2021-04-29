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
/*
 * TODO V1:
 * * Battery temp sensor CPU connection
 * * Boot0-pin connect to GND
 * * R17 had wrong value populated
 * * Red LED was flipped
 */

/*
 * TODO Software:
 * * Power switch support
 * * External current measurement support
 */

#ifndef HWCONF_HW_LUNA_BMS_H_
#define HWCONF_HW_LUNA_BMS_H_

#define HW_NAME					"hw_luna_bms"

// HW-specific
#define HW_INIT_HOOK()			palSetLineMode(LINE_CAN_EN, PAL_MODE_OUTPUT_PUSHPULL); \
								palSetLineMode(LINE_CURR_MEASURE_EN, PAL_MODE_OUTPUT_PUSHPULL)

#define HW_CAN_ON()				palClearLine(LINE_CAN_EN)
#define HW_CAN_OFF()			palSetLine(LINE_CAN_EN)


// Macros


// Settings
#define HW_CELLS_SERIES			14
#define HW_MAX_BAL_CH			10
#define HW_SHUNT_RES			(1.0e-3)
#define HW_SHUNT_AMP_GAIN		(50.0)
#define V_REG					3.3
#define R_CHARGE_TOP			(520e3 + 2.5e3 + 100.0)
#define R_CHARGE_BOTTOM			(10e3)

// LEDs
#define LINE_LED_RED			PAL_LINE(GPIOA, 0)
#define LINE_LED_GREEN			PAL_LINE(GPIOA, 1)

// CAN
#define LINE_CAN_RX				PAL_LINE(GPIOB, 8)
#define LINE_CAN_TX				PAL_LINE(GPIOB, 9)
//#define LINE_CAN_EN				PAL_LINE(GPIOB, 4)
#define HW_CAN_DEV				CAND1
#define HW_CAN_AF				9

//BQ76940
#define BQ76940_SDA_GPIO		GPIOB
#define BQ76940_SDA_PIN			11
#define BQ76940_SCL_GPIO		GPIOB
#define BQ76940_SCL_PIN			10

// Analog
//#define LINE_V_CHARGE			PAL_LINE(GPIOC, 2)
//#define LINE_CURRENT			PAL_LINE(GPIOC, 3)
//#define LINE_TEMP_0				PAL_LINE(GPIOC, 1)
//#define LINE_TEMP_1				PAL_LINE(GPIOC, 0)
//#define LINE_TEMP_2				PAL_LINE(GPIOC, 0)
//#define LINE_TEMP_3				PAL_LINE(GPIOC, 0)
//#define LINE_TEMP_4				PAL_LINE(GPIOC, 0)
//#define LINE_TEMP_5				PAL_LINE(GPIOC, 0)

//#define LINE_TEMP_0_EN			PAL_LINE(GPIOB, 5)
//#define LINE_TEMP_1_EN			PAL_LINE(GPIOB, 13)
//#define LINE_TEMP_2_EN			PAL_LINE(GPIOB, 13)
//#define LINE_TEMP_3_EN			PAL_LINE(GPIOB, 13)
//#define LINE_TEMP_4_EN			PAL_LINE(GPIOB, 13)
//#define LINE_TEMP_5_EN			PAL_LINE(GPIOB, 13)

#define NTC_RES(adc)			(10000.0 / ((4095.0 / (float)adc) - 1.0))
#define NTC_TEMP(adc)			(1.0 / ((logf(NTC_RES(adc) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

// TODO: Take highest of all temp sensors
#define HW_TEMP_CELLS_MAX()		bms_if_get_temp(1)

// ADC Channels
//#define ADC_CH_V_CHARGE			ADC_CHANNEL_IN3
//#define ADC_CH_CURRENT			ADC_CHANNEL_IN4
//#define ADC_CH_TEMP0			ADC_CHANNEL_IN2 // Next to STM32
//#define ADC_CH_TEMP1			ADC_CHANNEL_IN1 // Ext
//#define ADC_CH_TEMP2			ADC_CHANNEL_IN1 // Ext
//#define ADC_CH_TEMP3			ADC_CHANNEL_IN1 // Ext
//#define ADC_CH_TEMP4			ADC_CHANNEL_IN1 // Ext
//#define ADC_CH_TEMP5			ADC_CHANNEL_IN1 // Ext

// Other
//#define LINE_CURR_MEASURE_EN	PAL_LINE(GPIOB, 6)

// Default configuration
//#ifndef CONF_I_MEASURE_MODE
//#define CONF_I_MEASURE_MODE		I_MEASURE_MODE_VESC
//#endif

#endif /* HWCONF_HW_LUNA_BMS_H_ */
