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

#ifndef SLEEP_H_
#define SLEEP_H_

#include "ch.h"
#include "hal.h"
#include "conf_general.h"

typedef enum {
	AWAKE_REASON_INIT = 0,
	AWAKE_REASON_UNKNOWN,
	AWAKE_REASON_USB_CONFIGURED_AT_BOOT,
	AWAKE_REASON_USER_EXTENSION,
	AWAKE_REASON_HOST_COMMAND,
	AWAKE_REASON_COMM_ALIVE,
	AWAKE_REASON_CAN_PING,
	AWAKE_REASON_CAN_STATUS,
	AWAKE_REASON_REMOTE_BMS_CHARGE_BALANCE,
	AWAKE_REASON_CHARGING,
	AWAKE_REASON_BALANCING_ACTIVE,
	AWAKE_REASON_CURRENT_ABOVE_SLEEP_THRESHOLD,
	AWAKE_REASON_CHARGE_OVERCURRENT_HOLD,
	AWAKE_REASON_CONNECTOR,
	AWAKE_REASON_IO,
} awake_reason_t;

// Functions
void sleep_init(void);
void sleep_reset(void);
void sleep_reset_with_reason(awake_reason_t action);
int sleep_time_left(void);
void sleep_set_timer(int time);
void sleep_set_timer_with_reason(int time, awake_reason_t action);
awake_reason_t sleep_last_action_reason(void);
const char *sleep_action_to_string(awake_reason_t action);
systime_t sleep_last_action_time(void);
int sleep_last_action_target_ms(void);

#endif /* SLEEP_H_ */
