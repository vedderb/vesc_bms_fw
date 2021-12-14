/*
	Copyright 2019 - 2021 Benjamin Vedder	benjamin@vedder.se

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

#include "ch.h"
#include "hal.h"
#include "datatypes.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

// Firmware version
#define FW_VERSION_MAJOR			5
#define FW_VERSION_MINOR			03
// Set to 0 for building a release and iterate during beta test builds
#define FW_TEST_VERSION_NUMBER		18

// Init codes for the persistent storage. Change the config code when updating the config struct
// in a way that is not backwards compatible.
#define VAR_INIT_CODE				59763258

#define HW_NAME_MAX_CHARS			16

#if !defined(HW_SOURCE) && !defined(HW_HEADER)
#define HW_HEADER					"hw_12s7p_v1.h"
#define HW_SOURCE					"hw_12s7p_v1.c"

//#define HW_HEADER					"hw_18s_light.h"
//#define HW_SOURCE					"hw_18s_light.c"

//#define HW_HEADER					"hw_stormcore_bms.h"
//#define HW_SOURCE					"hw_stormcore_bms.c"

//#define HW_HEADER					"hw_rbat.h"
//#define HW_SOURCE					"hw_rbat.c"
#endif

/*
 * Enable blackmagic probe
 */
#ifndef HAS_BLACKMAGIC
#define HAS_BLACKMAGIC				1
#endif

/*
 * MCU
 */
#define SYSTEM_CORE_CLOCK			168000000
#define STM32_UUID					((uint32_t*)0x1FFF7590)
#define STM32_UUID_8				((uint8_t*)0x1FFF7590)
#define STM32_FLASH_SIZE			((uint16_t*)0x1FFF75E0)

#ifndef HW_SOURCE
#error "No hardware source file set"
#endif

#ifndef HW_HEADER
#error "No hardware header file set"
#endif

#include "hw.h"
#include "conf_default.h"

// Functions
void conf_general_apply_hw_limits(main_config_t *config);

#endif /* CONF_GENERAL_H_ */
