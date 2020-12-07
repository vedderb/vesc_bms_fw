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
#include <string.h>

// Macros
#define IWDG_START()				IWDG->KR = 0xCCCC
#define IWDG_RELOAD()				IWDG->KR = 0xAAAA
#define IWDG_WRITE_EN()				IWDG->KR = 0x5555
#define IWDG_WAIT_SR()				while(((IWDG->SR & IWDG_SR_RVU) != 0) || ((IWDG->SR & IWDG_SR_PVU) != 0)) {IWDG_RELOAD();}

// Private variables
static volatile bool init_done = false;
static volatile systime_t timeout_msec;
static volatile systime_t last_update_time;
static volatile uint32_t feed_counter[MAX_THREADS_MONITOR];

// Threads
static THD_WORKING_AREA(timeout_thread_wa, 512);
static THD_FUNCTION(timeout_thread, arg);

void timeout_init(void) {
	timeout_msec = 1000;
	last_update_time = 0;
	init_done = true;

	IWDG_WRITE_EN();

	IWDG_RELOAD();

	// IWDG counter clock: LSI/4
	IWDG->PR = 0;

	/* Set counter reload value to obtain 120ms IWDG TimeOut.
	 *
	 * LSI timer per datasheet is 32KHz typical, but 17KHz min
	 * and 47KHz max over the complete range of operating conditions,
	 * so reload time must ensure watchdog will work correctly under
	 * all conditions.
	 *
	 * Timeout threads runs every 100ms. Take 20% margin so wdt should
	 * be fed every 120ms. The worst condition occurs when the wdt clock
	 * runs at the max freq (47KHz) due to oscillator tolerances.
	 *
	 * t_IWDG(ms) = t_LSI(ms) * 4 * 2^(IWDG_PR[2:0]) * (IWDG_RLR[11:0] + 1)
	 * t_LSI(ms) [MAX] = 0.021276ms
	 * 12ms = 0.0212765 * 4 * 1 * (140 + 1)
	 *
	 * Counter Reload Value = 140
	 *
	 * When LSI clock runs the slowest, the IWDG will expire every 33.17 s
	*/
	IWDG->RLR = 1400;

	IWDG_RELOAD();

	// Enable IWDG (the LSI oscillator will be enabled by hardware)
	IWDG_START();

	chThdSleepMilliseconds(10);

	chThdCreateStatic(timeout_thread_wa, sizeof(timeout_thread_wa), NORMALPRIO, timeout_thread, NULL);
}

void timeout_feed_WDT(int index) {
	++feed_counter[index];
}

void timeout_configure_IWDT_slowest(void) {
	if (!init_done) {
		return;
	}

	IWDG_WAIT_SR();

	IWDG_WRITE_EN();
	IWDG->RLR = 4000;
	IWDG->PR = 7;

	IWDG_WAIT_SR();
}

void timeout_configure_IWDT(void) {
	if (!init_done) {
		return;
	}

	IWDG_WAIT_SR();

	IWDG_WRITE_EN();
	IWDG->RLR = 1400;
	IWDG->PR = 0;

	IWDG_WAIT_SR();
}

bool timeout_had_IWDG_reset(void) {
	// Check if the system has resumed from IWDG reset
	if (RCC->CSR & RCC_CSR_RMVF) {
		RCC->CSR |= RCC_CSR_RMVF;
		return true;
	}

	return false;
}

static THD_FUNCTION(timeout_thread, arg) {
	(void)arg;

	chRegSetThreadName("Timeout");

	for(;;) {
		bool threads_ok = true;

		// Monitored threads (can, bms) must report at least one iteration,
		// otherwise the watchdog won't be feed and MCU will reset. All threads should
		// be monitored

		if (feed_counter[THREAD_BAL] < MIN_THREAD_ITERATIONS) {
			threads_ok = false;
		}

		if (feed_counter[THREAD_CANBUS] < MIN_THREAD_ITERATIONS) {
			threads_ok = false;
		}

		if (feed_counter[THREAD_SLEEP] < MIN_THREAD_ITERATIONS) {
			threads_ok = false;
		}

		memset((void*)feed_counter, 0, sizeof(feed_counter));

		if (threads_ok == true) {
			// Feed WDT
			IWDG_RELOAD();
		} else {
			// not reloading the watchdog will produce a reset.
			// This can be checked from the GUI logs as
			// "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET"
		}

		chThdSleepMilliseconds(100);
	}
}

