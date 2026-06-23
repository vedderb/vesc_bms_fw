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

#include "sleep.h"
#include "timeout.h"
#include "pwr.h"
#include "usbcfg.h"
#include "bms_if.h"
#include "main.h"

// Private variables
static volatile int m_sleep_timer = 200;
static volatile awake_reason_t m_sleep_last_action = AWAKE_REASON_INIT;
static volatile systime_t m_sleep_last_action_time = 0;

// Private functions
static void go_to_sleep(void);
static void sleep_note_action(awake_reason_t action);
static void sleep_reset_internal(awake_reason_t action);
static void sleep_set_timer_internal(int time, awake_reason_t action);

// Threads
static THD_WORKING_AREA(sleep_thread_wa, 512);
static THD_FUNCTION(sleep_thread, arg);

void sleep_init(void) {
	chThdCreateStatic(sleep_thread_wa, sizeof(sleep_thread_wa), NORMALPRIO, sleep_thread, NULL);
}

static void sleep_note_action(awake_reason_t action) {
	m_sleep_last_action = action;
	m_sleep_last_action_time = chVTGetSystemTimeX();
}

static void sleep_reset_internal(awake_reason_t action) {
	sleep_note_action(action);
	m_sleep_timer = backup.config.sleep_timeout_reset_ms;
	HW_STAY_AWAKE_HOOK();
}

void sleep_reset(void) {
	sleep_reset_internal(AWAKE_REASON_UNKNOWN);
}

void sleep_reset_with_reason(awake_reason_t action) {
	sleep_reset_internal(action);
}

int sleep_time_left(void) {
	return m_sleep_timer;
}

static void sleep_set_timer_internal(int time, awake_reason_t action) {
	sleep_note_action(action);
	m_sleep_timer = time;
}

void sleep_set_timer(int time) {
	sleep_set_timer_internal(time, AWAKE_REASON_UNKNOWN);
}

void sleep_set_timer_with_reason(int time, awake_reason_t reason) {
	sleep_set_timer_internal(time, reason);
}
awake_reason_t sleep_last_action_reason(void) {
	return m_sleep_last_action;
}

const char *sleep_action_to_string(awake_reason_t action) {
	switch (action) {
	case AWAKE_REASON_INIT:
		return "init";
	case AWAKE_REASON_UNKNOWN:
		return "unknown";
	case AWAKE_REASON_USB_CONFIGURED_AT_BOOT:
		return "USB configured at boot";
	case AWAKE_REASON_USER_EXTENSION:
		return "user extension";
	case AWAKE_REASON_HOST_COMMAND:
		return "host command";
	case AWAKE_REASON_COMM_ALIVE:
		return "comm alive";
	case AWAKE_REASON_CAN_PING:
		return "CAN ping";
	case AWAKE_REASON_CAN_STATUS:
		return "CAN status";
	case AWAKE_REASON_REMOTE_BMS_CHARGE_BALANCE:
		return "remote BMS charge/balance";
	case AWAKE_REASON_CHARGING:
		return "charging";
	case AWAKE_REASON_BALANCING_ACTIVE:
		return "balancing active";
	case AWAKE_REASON_CURRENT_ABOVE_SLEEP_THRESHOLD:
		return "pack current above sleep threshold";
	case AWAKE_REASON_CHARGE_OVERCURRENT_HOLD:
		return "charge overcurrent hold";
	case AWAKE_REASON_CONNECTOR:
			return "connector";
	case AWAKE_REASON_IO:
		return "io activity";
	default:
		return "unknown";
	}
}

systime_t sleep_last_action_time(void) {
	return m_sleep_last_action_time;
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

	HW_SLEEP_HOOK();
	CURR_MEASURE_OFF();

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
	wakeupspec.wutr |= 5; // Period will be 5+1 seconds.
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
			sleep_reset_with_reason(AWAKE_REASON_USB_CONFIGURED_AT_BOOT);
			sleep_set_timer(240000);
			usb_conf_reset = true;
			HW_STAY_AWAKE_HOOK();
		}

		timeout_feed_WDT(THREAD_SLEEP);

		chThdSleepMilliseconds(1);
	}
}
