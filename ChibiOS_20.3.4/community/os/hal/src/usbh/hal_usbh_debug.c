/*
    ChibiOS - Copyright (C) 2006..2017 Giovanni Di Sirio
              Copyright (C) 2015..2019 Diego Ismirlian, (dismirlian(at)google's mail)

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

#if HAL_USE_USBH && USBH_DEBUG_ENABLE

#include "ch.h"
#include "usbh/debug.h"
#include "chprintf.h"
#include <stdarg.h>

#define TEMP_BUFF_LEN	255

/* ************************ */
/* Circular queue structure */
/* ************************ */
static int dq_append_string(usbh_dq_t *q, const uint8_t *s, int len) {
	if (len <= 0) return 0;
	if (len > TEMP_BUFF_LEN) len = TEMP_BUFF_LEN;
	if (q->rem < len + 1) return -1;
	q->rem -= len + 1;

	uint8_t *d = q->next;
	*d++ = len;
	if (d == q->end) d = q->start;
	while (len--) {
		*d++ = *s++;
		if (d == q->end) d = q->start;
	}
	q->next = d;
	return 0;
}

static void dq_remove_oldest_string(usbh_dq_t *q) {
	int len = *q->first;
	if (len) {
		++len;
		q->rem += len;
		q->first += len;
		if (q->first >= q->end) {
			q->first -= q->sz;
		}
		if (q->rem == q->sz) {
			*q->first = 0;
		}
	}
}

static int dq_read_oldest_string(usbh_dq_t *q, uint8_t *d) {
	uint8_t *s = q->first;
	int len;
	int sz;
	len = sz = *s++;
	while (len--) {
		*d++ = *s++;
		if (d == q->end) d = q->start;
	}
	*d = 0;
	return sz;
}

static void dq_init(usbh_dq_t *q, uint8_t *buff, int len) {
	q->start = q->first = q->next = buff;
	q->end = q->start + len;
	q->sz = q->rem = len;
	*buff = 0;
}


static uint8_t buff[TEMP_BUFF_LEN + 1];

static inline syssts_t _dbg_prologue(struct usbh_debug_helper *debug,
		uint32_t hfnum, uint16_t hfir, const char *s, int *len) {
	syssts_t sts = chSysGetStatusAndLockX();

	debug->last = osalOsGetSystemTimeX();
	if (debug->ena) {
		debug->first = debug->last;
	}

	if (((hfnum & 0x3fff) == 0x3fff) && (hfir == (hfnum >> 16))) {
		*len = chsnprintf((char *)buff, sizeof(buff), "+%08d ", debug->last - debug->first);
		debug->ena = FALSE;
	} else {
		uint32_t f = hfnum & 0xffff;
		uint32_t p = 1000 - ((hfnum >> 16) / (hfir / 1000));
		*len = chsnprintf((char *)buff, sizeof(buff), "%05d.%03d %s", f, p, s);
		debug->ena = TRUE;
	}

	return sts;
}

static inline void dbg_epilogue(struct usbh_debug_helper *debug,
		syssts_t sts, int len) {

	while (dq_append_string(&debug->dq, buff, len) < 0) {
		dq_remove_oldest_string(&debug->dq);
	}

	if (debug->on) {
		chThdResumeI(&debug->tr, MSG_OK);
	}

	chSysRestoreStatusX(sts);
}

#if USBH_DEBUG_MULTI_HOST
void usbDbgPrintf(USBHDriver *host, const char *fmt, ...) {
	if (!host) return;
	struct usbh_debug_helper *const debug = &host->debug;
	uint32_t hfnum = host->otg->HFNUM;
	uint16_t hfir = host->otg->HFIR;
#else
void usbDbgPrintf(const char *fmt, ...) {
	struct usbh_debug_helper *const debug = &usbh_debug;
	uint32_t hfnum = USBH_DEBUG_SINGLE_HOST_SELECTION.otg->HFNUM;
	uint16_t hfir = USBH_DEBUG_SINGLE_HOST_SELECTION.otg->HFIR;
#endif
	int len;

	syssts_t sts = _dbg_prologue(debug, hfnum, hfir, "", &len);

	va_list ap;
	va_start(ap, fmt);
	len += chvsnprintf((char *)buff + len, sizeof(buff) - len, fmt, ap);
	va_end(ap);

	dbg_epilogue(debug, sts, len);
}

#if USBH_DEBUG_MULTI_HOST
void usbDbgPuts(USBHDriver *host, const char *s) {
	if (!host) return;
	struct usbh_debug_helper *const debug = &host->debug;
	uint32_t hfnum = host->otg->HFNUM;
	uint16_t hfir = host->otg->HFIR;
#else
void usbDbgPuts(const char *s) {
	struct usbh_debug_helper *const debug = &usbh_debug;
	uint32_t hfnum = USBH_DEBUG_SINGLE_HOST_SELECTION.otg->HFNUM;
	uint16_t hfir = USBH_DEBUG_SINGLE_HOST_SELECTION.otg->HFIR;
#endif
	int len;
	syssts_t sts = _dbg_prologue(debug, hfnum, hfir, s, &len);
	dbg_epilogue(debug, sts, len);
}

#if USBH_DEBUG_MULTI_HOST
void usbDbgEnable(USBHDriver *host, bool enable) {
	struct usbh_debug_helper *const debug = &host->debug;
#else
void usbDbgEnable(bool enable) {
	struct usbh_debug_helper *const debug = &usbh_debug;
#endif
	debug->on = enable;
}

static void usb_debug_thread(void *arg) {
#if USBH_DEBUG_MULTI_HOST
	USBHDriver *const host = (USBHDriver *)arg;
	struct usbh_debug_helper *const debug = &host->debug;
#else
	(void)arg;
	struct usbh_debug_helper *const debug = &usbh_debug;
#endif

	uint8_t rdbuff[TEMP_BUFF_LEN + 1];

	chRegSetThreadName("USBH_DBG");
	while (true) {
		chSysLock();
		int len = dq_read_oldest_string(&debug->dq, rdbuff);
		if (!len) {
			chThdSuspendS(&debug->tr);
			chSysUnlock();
		} else {
			dq_remove_oldest_string(&debug->dq);
			chSysUnlock();
#if USBH_DEBUG_MULTI_HOST
			USBH_DEBUG_OUTPUT_CALLBACK(host, rdbuff, len);
#else
			USBH_DEBUG_OUTPUT_CALLBACK(rdbuff, len);
#endif
		}
	}
}

#if USBH_DEBUG_MULTI_HOST
void usbDbgInit(USBHDriver *host) {
	struct usbh_debug_helper *const debug = &host->debug;
	void *param = host;
#else
void usbDbgInit(void) {
	struct usbh_debug_helper *const debug = &usbh_debug;
	void *param = NULL;
#endif
	dq_init(&debug->dq, debug->buff, sizeof(debug->buff));
	debug->on = true;
	chThdCreateStatic(debug->thd_wa, sizeof(debug->thd_wa),
			NORMALPRIO, usb_debug_thread, param);
}

#endif
