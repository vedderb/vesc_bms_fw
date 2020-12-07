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

#ifndef TIMEOUT_H_
#define TIMEOUT_H_

#include "ch.h"
#include "chtypes.h"
#include "chsystypes.h"

#define MAX_THREADS_MONITOR		10
#define MIN_THREAD_ITERATIONS	1

typedef enum {
	THREAD_BAL = 0,
	THREAD_CANBUS,
	THREAD_SLEEP
} WWDT_THREAD_TYPES;

// Functions
void timeout_init(void);
void timeout_configure_IWDT(void);
void timeout_configure_IWDT_slowest(void);
bool timeout_had_IWDG_reset(void);
void timeout_feed_WDT(int index);

#endif /* TIMEOUT_H_ */
