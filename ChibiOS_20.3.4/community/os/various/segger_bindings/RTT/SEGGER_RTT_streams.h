/*
    ChibiOS - Copyright (C) 2006..2017 Giovanni Di Sirio
              Copyright (C) 2015..2017 Diego Ismirlian, (dismirlian (at) google's mail)

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef SEGGER_RTT_streams_H_
#define SEGGER_RTT_streams_H_

#include "hal.h"
#include "SEGGER_RTT.h"

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef enum {
	RTT_STATE_UNINIT = 0,
	RTT_STATE_STOP = 1,
	RTT_STATE_ACTIVE = 2,
	RTT_STATE_READY = 3
} rtt_state_t;

typedef enum {
	RTT_MODE_FLAGS_NO_BLOCK_SKIP = SEGGER_RTT_MODE_NO_BLOCK_SKIP,
	RTT_MODE_FLAGS_NO_BLOCK_TRIM = SEGGER_RTT_MODE_NO_BLOCK_TRIM,
	RTT_MODE_FLAGS_BLOCK_IF_FIFO_FULL = SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL,
} rtt_mode_flags_t;

#define _rtt_driver_methods                                          \
	_base_sequential_stream_methods

struct RTTDriverVMT {
	_rtt_driver_methods
};

typedef struct RTTDriver RTTDriver;
typedef struct RTTConfig RTTConfig;
typedef struct RTTBufferConfig RTTBufferConfig;

struct RTTDriver {
	/* inherited from abstract asyncrhonous channel driver */
	const struct RTTDriverVMT *vmt;
	_base_sequential_stream_data

	rtt_state_t state;
	unsigned int up_buffer_index;
	unsigned int down_buffer_index;
};

struct RTTBufferConfig {
	void *buff;
	unsigned int size;
	rtt_mode_flags_t flags;
};

struct RTTConfig {
	const char *name;
	RTTBufferConfig up;
	RTTBufferConfig down;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/
#define rttGetState(rttdp) ((rttdp)->state)
#define rttGetUpBufferIndex(rttdp) ((rttdp)->up_buffer_index)
#define rttGetDownBufferIndex(rttdp) ((rttdp)->down_buffer_index)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
extern RTTDriver RTTD0;

#ifdef __cplusplus
extern "C" {
#endif
	/* RTT device driver */
	void rttInit(void);
	void rttObjectInit(RTTDriver *rttdp, const RTTConfig *cfg);
	void rttStart(RTTDriver *rttdp);
	void rttSetUpFlags(RTTDriver *rttdp, rtt_mode_flags_t flags);
	void rttSetDownFlags(RTTDriver *rttdp, rtt_mode_flags_t flags);
#ifdef __cplusplus
}
#endif

#endif /* SEGGER_RTT_streams_H_ */
