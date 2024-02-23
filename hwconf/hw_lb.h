/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#ifndef HWCONF_HW_LB_H_
#define HWCONF_HW_LB_H_

#define HW_NAME					"lb"
#define HW_DEFAULT_ID			20

// HW-specific
#define HW_INIT_HOOK()			hw_board_init()
#define HW_SLEEP_HOOK()			hw_board_sleep()
#define HW_STAY_AWAKE_HOOK()	hw_stay_awake()
#define HW_GET_TEMP(sensor)		hw_get_temp(sensor)
#define HW_TEST_WAKE_UP()		hw_test_wake_up()
#define HW_TEST_USB_AT_BOOT		false

#define HW_CAN_ON()				palClearLine(LINE_CAN_EN)
#define HW_CAN_OFF()			palSetLine(LINE_CAN_EN)
#define CURR_MEASURE_ON()		palClearLine(LINE_CURR_MEASURE_EN)
#define CURR_MEASURE_OFF()		palSetLine(LINE_CURR_MEASURE_EN)

// Macros
#define CHARGE_ENABLE()			palSetLine(LINE_BATT_OUT_EN);
#define CHARGE_DISABLE()		palClearLine(LINE_BATT_OUT_EN);

// Settings
#define HW_CELLS_SERIES			12
#define HW_SHUNT_RES			(0.2e-3)
#define HW_SHUNT_AMP_GAIN		(20.0)
#define V_REG					3.3
#define R_CHARGE_TOP			(39.0e3)
#define R_CHARGE_BOTTOM			(2.2e3)

// LEDs
#define LINE_LED_RED			PAL_LINE(GPIOA, 0)
#define LINE_LED_GREEN			PAL_LINE(GPIOA, 1)

// LTC6813
#define LINE_LTC_CS				PAL_LINE(GPIOA, 4)
#define LINE_LTC_SCLK			PAL_LINE(GPIOA, 5)
#define LINE_LTC_MISO			PAL_LINE(GPIOA, 6)
#define LINE_LTC_MOSI			PAL_LINE(GPIOA, 7)
#define LTC_GPIO_CURR_MON		1
#define LTC_GPIO_CURR_MON_2		2
#define LTC_INVERT_CURRENT

// CAN
#define LINE_CAN_RX				PAL_LINE(GPIOB, 8)
#define LINE_CAN_TX				PAL_LINE(GPIOB, 9)
#define HW_CAN_DEV				CAND1
#define HW_CAN_AF				9
#define LINE_CAN_EN				PAL_LINE(GPIOB, 10)

// BME280 (temp/humidity/pressure)
#define BME280_SDA_GPIO			GPIOB
#define BME280_SDA_PIN			12
#define BME280_SCL_GPIO			GPIOB
#define BME280_SCL_PIN			11

// Buzzer
#define BUZZER_LINE				PAL_LINE(GPIOA, 3)
#define BUZZER_AF				2
#define BUZZER_PWM				PWMD5
#define BUZZER_FREQ_HZ			4000
#define BUZZER_ON()				pwmEnableChannel(&BUZZER_PWM, 3, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM, 5000))
#define BUZZER_OFF()			pwmEnableChannel(&BUZZER_PWM, 3, PWM_PERCENTAGE_TO_WIDTH(&BUZZER_PWM, 0))

// NRF SWD
#define NRF5x_SWDIO_GPIO		GPIOC
#define NRF5x_SWDIO_PIN			10
#define NRF5x_SWCLK_GPIO		GPIOC
#define NRF5x_SWCLK_PIN			11

// Temp and led shift register
#define LINE_SR_SER				PAL_LINE(GPIOC, 4)
#define LINE_SR_RCLK			PAL_LINE(GPIOC, 5)
#define LINE_SR_SCLK			PAL_LINE(GPIOC, 6)

// Analog
#define LINE_V_CHARGE			PAL_LINE(GPIOB, 1)
#define LINE_TEMP_0				PAL_LINE(GPIOC, 0)
#define LINE_TEMP_1				PAL_LINE(GPIOC, 2)
#define LINE_TEMP_2				PAL_LINE(GPIOC, 1)
#define LINE_TEMP_3				PAL_LINE(GPIOC, 3)
#define LINE_TEMP_4				PAL_LINE(GPIOB, 0)
#define LINE_TEMP_5				PAL_LINE(GPIOA, 2)

#define LINE_TEMP_0_EN			PAL_LINE(GPIOC, 13)
#define LINE_TEMP_1_EN			PAL_LINE(GPIOC, 13)
#define LINE_TEMP_2_EN			PAL_LINE(GPIOC, 13)
#define LINE_TEMP_3_EN			PAL_LINE(GPIOC, 13)
#define LINE_TEMP_4_EN			PAL_LINE(GPIOC, 13)
#define LINE_TEMP_5_EN			PAL_LINE(GPIOC, 13)

#define NTC_RES(adc)					(10000.0 / ((4095.0 / (float)adc) - 1.0))
#define NTC_TEMP_FROM_RES(res)			(1.0 / ((logf(res / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_WITH_IND(adc, ind)		NTC_TEMP_FROM_RES(NTC_RES(adc))

#define HW_TEMP_SENSORS			49
#define HW_TEMP_CELLS_MAX()		hw_temp_cell_max()

// ADC Channels
#define ADC_CH_V_CHARGE			ADC_CHANNEL_IN16
#define ADC_CH_TEMP0			ADC_CHANNEL_IN1
#define ADC_CH_TEMP1			ADC_CHANNEL_IN3
#define ADC_CH_TEMP2			ADC_CHANNEL_IN2
#define ADC_CH_TEMP3			ADC_CHANNEL_IN4
#define ADC_CH_TEMP4			ADC_CHANNEL_IN15
#define ADC_CH_TEMP5			ADC_CHANNEL_IN7 // 12V Out

// Other
#define LINE_CURR_MEASURE_EN	PAL_LINE(GPIOC, 7)
#define LINE_MC_EN				PAL_LINE(GPIOC, 8)
#define LINE_BATT_OUT_EN		PAL_LINE(GPIOC, 9)
#define LINE_12V_EN				PAL_LINE(GPIOA, 8)
#define LINE_12V_SENSE_EN		PAL_LINE(GPIOA, 9)
#define LINE_ESP_EN				PAL_LINE(GPIOC, 12)

// HW Functions
void hw_board_init(void);
void hw_board_sleep(void);
void hw_stay_awake(void);
float hw_temp_cell_max(void);
float hw_get_temp(int sensor);
float hw_get_v_charge(void);
bool hw_test_if_conn(bool print);
void hw_test_wake_up(void);
float hw_get_v_12v(void);

#endif /* HWCONF_HW_LB_H_ */
