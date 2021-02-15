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

#include "timeout.h"
#include "pwr.h"
#include "usbcfg.h"
#include "bms_if.h"
#include "main.h"

// Private variables
static volatile int m_sleep_timer = 300;

// Private functions
static void go_to_sleep(void);

// Threads
static THD_WORKING_AREA(sleep_thread_wa, 512);
static THD_FUNCTION(sleep_thread, arg);

void sleep_init(void) {
	chThdCreateStatic(sleep_thread_wa, sizeof(sleep_thread_wa), NORMALPRIO, sleep_thread, NULL);
}

void sleep_reset(void) {
	m_sleep_timer = backup.config.sleep_timeout_reset_ms;
}

int sleep_time_left(void) {
	return m_sleep_timer;
}

static void go_to_sleep(void) {
	chSysLock();

	CURR_MEASURE_OFF();

#ifdef LINE_BQ_CHG_EN
	BQ_CHG_OFF();
	BQ_CP_OFF();
	BQ_DSG_OFF();
	BQ_PMON_OFF();
	BQ_PCHG_OFF();
#endif

	TEMP_MEASURE_OFF();
	HW_CAN_OFF();

	LED_OFF(LINE_LED_RED);
	LED_OFF(LINE_LED_GREEN);

	bms_if_sleep();

	for (volatile int i = 0;i < 1000;i++) {
		__NOP();
	}

	do {
		RTCD1.rtc->WPR = 0xCA;
		RTCD1.rtc->WPR = 0x53;
	} while(0);

	timeout_configure_IWDT_slowest();

	RTCWakeup wakeupspec;
	wakeupspec.wutr = ((uint32_t)4) << 16; // select 1 Hz clock source
	wakeupspec.wutr |= 9; // Period will be 6+1 seconds.
	rtcSTM32SetPeriodicWakeup(&RTCD1, &wakeupspec);

	PWR->CR1 |= PWR_CR1_LPMS_STANDBY;
	PWR->CR3 |= PWR_CR3_RRS; // Keep ram4 during standby
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__WFI();
}

static THD_FUNCTION(sleep_thread, arg) {
	(void)arg;

	chRegSetThreadName("Sleep");

	bool usb_conf_reset = false;

	for(;;) {
		if (m_sleep_timer > 0) {
			m_sleep_timer--;

			static int blink = 0;
			blink++;
			if (blink > 500) {
				blink = 0;
			}

			if (usb_cdc_configured_cnt() > 0 && blink > 200) {
				blink = 0;
			}

			if (blink < 40) {
				LED_ON(LINE_LED_GREEN);
			} else {
				LED_OFF(LINE_LED_GREEN);
			}
		} else {
			LED_OFF(LINE_LED_GREEN);
			go_to_sleep();
		}

		if (!usb_conf_reset && usb_cdc_configured_cnt() > 0) {
			m_sleep_timer = 240000;
			usb_conf_reset = true;
		}

		timeout_feed_WDT(THREAD_SLEEP);

		chThdSleepMilliseconds(1);
	}
}
