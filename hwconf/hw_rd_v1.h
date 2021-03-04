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

#ifndef HWCONF_HW_RD_V1_H_
#define HWCONF_HW_RD_V1_H_

#define HW_NAME					"rd_v1"

/**
 * TODO:
 * * Precharge control
 * * Charge enable and disable
 * * Powerkey
 * * UART-comm
 * * [OK] Last temp sensor
 * * Black magic probe
 */

// HW-specific
#define HW_INIT_HOOK()			hw_board_init()

#define HW_CAN_ON()				palClearLine(LINE_CAN_EN)
#define HW_CAN_OFF()			palSetLine(LINE_CAN_EN)
#define CURR_MEASURE_ON()		palClearLine(LINE_CURR_MEASURE_EN)
#define CURR_MEASURE_OFF()		palSetLine(LINE_CURR_MEASURE_EN)

#define HW_RELAY_MAIN_ON()		palSetLine(LINE_RELAY_MAIN)
#define HW_RELAY_MAIN_OFF()		palClearLine(LINE_RELAY_MAIN)
#define HW_RELAY_PCH_ON()		palSetLine(LINE_RELAY_PCH)
#define HW_RELAY_PCH_OFF()		palClearLine(LINE_RELAY_PCH)
#define HW_RALAY_MAIN_IS_ON()	palReadLine(LINE_RELAY_MAIN)

// Macros
#define CHARGE_ENABLE()			hw_board_chg_en(true)
#define CHARGE_DISABLE()		hw_board_chg_en(false)

// Settings
#define HW_CELLS_SERIES			14
#define HW_MAX_BAL_CH			8
#define HW_SHUNT_RES			(0.1e-3)
#define HW_SHUNT_AMP_GAIN		(50.0)
#define V_REG					3.3
#define R_CHARGE_TOP			(110e3)
#define R_CHARGE_BOTTOM			(3e3)

// Enable pins
#define LINE_CURR_MEASURE_EN	PAL_LINE(GPIOB, 6)
#define LINE_5V_HP_EN			PAL_LINE(GPIOA, 15)
#define LINE_3V3P_EN			PAL_LINE(GPIOB, 0)
#define LINE_12V_HP_EN			PAL_LINE(GPIOB, 1)
#define LINE_AFTER_FUSE_EN		PAL_LINE(GPIOD, 2)

// Relays
#define LINE_RELAY_PCH			PAL_LINE(GPIOC, 13)
#define LINE_RELAY_MAIN			PAL_LINE(GPIOC, 1)

// External connectors
#define LINE_PWRKEY_1			PAL_LINE(GPIOH, 0)
#define LINE_PWRKEY_2			PAL_LINE(GPIOH, 1)

// LEDs
#define LINE_LED_RED			PAL_LINE(GPIOB, 10)
#define LINE_LED_GREEN			PAL_LINE(GPIOB, 11)
#define LINE_LED_BLUE			PAL_LINE(GPIOB, 12) // TODO!

// LTC6813
#define LINE_LTC_CS				PAL_LINE(GPIOC, 6)
#define LINE_LTC_SCLK			PAL_LINE(GPIOC, 7)
#define LINE_LTC_MISO			PAL_LINE(GPIOC, 8)
#define LINE_LTC_MOSI			PAL_LINE(GPIOC, 9)
#define LTC_GPIO_CURR_MON		2

// CAN
#define LINE_CAN_RX				PAL_LINE(GPIOB, 8)
#define LINE_CAN_TX				PAL_LINE(GPIOB, 9)
#define HW_CAN_DEV				CAND1
#define HW_CAN_AF				9
#define LINE_CAN_EN				PAL_LINE(GPIOB, 7)

// HDC1080 (temp/humidity)
#define HDC1080_SDA_GPIO		GPIOA
#define HDC1080_SDA_PIN			8
#define HDC1080_SCL_GPIO		GPIOA
#define HDC1080_SCL_PIN			7

// Analog
#define HW_ADC_TEMP_SENSORS		7

#define LINE_V_CHARGE			PAL_LINE(GPIOC, 5)
#define LINE_V_FUSE				PAL_LINE(GPIOC, 2)
#define LINE_CURRENT			PAL_LINE(GPIOC, 3)
#define LINE_TEMP_0				PAL_LINE(GPIOA, 0)
#define LINE_TEMP_1				PAL_LINE(GPIOA, 1)
#define LINE_TEMP_2				PAL_LINE(GPIOA, 2)
#define LINE_TEMP_3				PAL_LINE(GPIOA, 3)
#define LINE_TEMP_4				PAL_LINE(GPIOA, 4)
#define LINE_TEMP_5				PAL_LINE(GPIOA, 5)
#define LINE_TEMP_6				PAL_LINE(GPIOA, 6)

#define LINE_TEMP_0_EN			PAL_LINE(GPIOC, 10)
#define LINE_TEMP_1_EN			PAL_LINE(GPIOC, 11)
#define LINE_TEMP_2_EN			PAL_LINE(GPIOC, 12)
#define LINE_TEMP_3_EN			PAL_LINE(GPIOB, 2)
#define LINE_TEMP_4_EN			PAL_LINE(GPIOB, 3)
#define LINE_TEMP_5_EN			PAL_LINE(GPIOB, 4)
#define LINE_TEMP_6_EN			PAL_LINE(GPIOB, 5)

#define NTC_RES(adc)			((4095.0 / (float)adc) * 10000.0 - 10000.0)
#define NTC_TEMP(adc)			(1.0 / ((logf(NTC_RES(adc) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

// TODO: Take highest of all temp sensors
#define HW_TEMP_CELLS_MAX()		bms_if_get_humidity_sensor_temp()

// ADC Channels
#define ADC_CH_V_CHARGE			ADC_CHANNEL_IN14
#define ADC_CH_V_FUSE			ADC_CHANNEL_IN3
#define ADC_CH_CURRENT			ADC_CHANNEL_IN4
#define ADC_CH_TEMP0			ADC_CHANNEL_IN5
#define ADC_CH_TEMP1			ADC_CHANNEL_IN6
#define ADC_CH_TEMP2			ADC_CHANNEL_IN7
#define ADC_CH_TEMP3			ADC_CHANNEL_IN8
#define ADC_CH_TEMP4			ADC_CHANNEL_IN9
#define ADC_CH_TEMP5			ADC_CHANNEL_IN10
#define ADC_CH_TEMP6			ADC_CHANNEL_IN11

// LEDs are inverted, so override macros here
#define LED_ON(led)				palClearLine(led)
#define LED_OFF(led)			palSetLine(led)

// Functions
void hw_board_init(void);
void hw_board_chg_en(bool enable);
bool hw_psw_switch_on(void);
void hw_psw_switch_off(void);

#endif /* HWCONF_HW_RD_V1_H_ */
