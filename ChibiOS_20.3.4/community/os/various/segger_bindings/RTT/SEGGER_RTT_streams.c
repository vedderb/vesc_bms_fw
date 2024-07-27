/*
    ChibiOS - Copyright (C) 2006..2017 Giovanni Di Sirio
              Copyright (C) 2019 Diego Ismirlian, (dismirlian(at)google's mail)

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

#include "hal.h"
#include "SEGGER_RTT_streams.h"

RTTDriver RTTD0;
static bool rtt_global_init;

static size_t _write(RTTDriver *rttdp, const uint8_t *bp, size_t n) {
	return SEGGER_RTT_Write(rttdp->up_buffer_index, bp, n);
}

static size_t _read(RTTDriver *rttdp, uint8_t *bp, size_t n) {
	(void)rttdp, (void)bp, (void)n;
	/* TODO: implement */
	return 0;
}

static msg_t _put(RTTDriver *rttdp, uint8_t b) {
	if (SEGGER_RTT_PutChar(rttdp->up_buffer_index, b) == 1) {
		return MSG_OK;
	}
	return MSG_TIMEOUT;
}

static msg_t _get(RTTDriver *rttdp) {
	(void)rttdp;
	/* TODO: implement */
	return MSG_TIMEOUT;
}

static const struct RTTDriverVMT vmt = {
	(size_t)0,
	(size_t (*)(void *, const uint8_t *, size_t))_write,
	(size_t (*)(void *, uint8_t *, size_t))_read,
	(msg_t (*)(void *, uint8_t))_put,
	(msg_t (*)(void *))_get,
};

static inline void _object_init(RTTDriver *rttdp) {
	rttdp->state = RTT_STATE_READY;
	rttdp->vmt = &vmt;
}

void rttInit(void) {
	osalDbgAssert(rtt_global_init == false, "double init");
	SEGGER_RTT_LOCK();
	SEGGER_RTT_Init();
	RTTD0.up_buffer_index = 0;
	RTTD0.down_buffer_index = 0;
	_object_init(&RTTD0);
	rtt_global_init = true;
	RTTD0.state = RTT_STATE_READY;
	SEGGER_RTT_UNLOCK();
}

void rttObjectInit(RTTDriver *rttdp, const RTTConfig *cfg) {
	osalDbgCheck(rttdp);
	osalDbgAssert(rtt_global_init, "uninitialized");
	osalDbgAssert(rttdp != &RTTD0, "RTTD0 is automatically initialized on rttInit");

	int idx;

	SEGGER_RTT_LOCK();
	if (cfg->down.size) {
		idx = SEGGER_RTT_AllocDownBuffer(cfg->name, cfg->down.buff, cfg->down.size, cfg->down.flags);
		osalDbgAssert(idx > 0, "can't alloc down buffer");
		rttdp->down_buffer_index = (unsigned)idx;
	} else {
		rttdp->down_buffer_index = 0;
	}

	if (cfg->up.size) {
		idx = SEGGER_RTT_AllocUpBuffer(cfg->name, cfg->up.buff, cfg->up.size, cfg->up.flags);
		osalDbgAssert(idx > 0, "can't alloc up buffer");
		rttdp->up_buffer_index = (unsigned)idx;
	} else {
		rttdp->up_buffer_index = 0;
	}

	_object_init(rttdp);
	SEGGER_RTT_UNLOCK();
}

void rttSetUpFlags(RTTDriver *rttdp, rtt_mode_flags_t flags) {
	int ret = SEGGER_RTT_SetFlagsUpBuffer(rttdp->up_buffer_index, (unsigned)flags);
	osalDbgAssert(ret >= 0, "error");
}

void rttSetDownFlags(RTTDriver *rttdp, rtt_mode_flags_t flags) {
	int ret = SEGGER_RTT_SetFlagsDownBuffer(rttdp->down_buffer_index, (unsigned)flags);
	osalDbgAssert(ret >= 0, "error");
}

void rttStart(RTTDriver *rttdp) {
	osalDbgCheck(rttdp);
	osalDbgAssert(rtt_global_init, "uninitialized");
	osalDbgAssert((rttdp->state == RTT_STATE_ACTIVE)
			|| (rttdp->state == RTT_STATE_READY), "wrong state");
	rttdp->state = RTT_STATE_READY;
}
