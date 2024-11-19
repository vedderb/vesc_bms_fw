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


#ifndef USBH_DEBUG_H_
#define USBH_DEBUG_H_

#include "hal_usbh.h"

#if HAL_USE_USBH

#if USBH_DEBUG_ENABLE
#if USBH_DEBUG_MULTI_HOST
	/* output callback */
	void USBH_DEBUG_OUTPUT_CALLBACK(USBHDriver *host, const uint8_t *buff, size_t len);

	/* printing functions */
	void usbDbgPrintf(USBHDriver *host, const char *fmt, ...);
	void usbDbgPuts(USBHDriver *host, const char *s);
	void usbDbgInit(USBHDriver *host);
#else
	/* output callback */
	void USBH_DEBUG_OUTPUT_CALLBACK(const uint8_t *buff, size_t len);

	/* printing functions */
	void usbDbgPrintf(const char *fmt, ...);
	void usbDbgPuts(const char *s);
	void usbDbgInit(void);
#endif

	void usbDbgReset(void);
#else

#if USBH_DEBUG_MULTI_HOST
#	define usbDbgPrintf(host, fmt, ...) do {} while(0)
#	define usbDbgPuts(host, s) do {} while(0)
#	define usbDbgInit(host) do {} while(0)
#else
#	define usbDbgPrintf(fmt, ...) do {} while(0)
#	define usbDbgPuts(s) do {} while(0)
#	define usbDbgInit() do {} while(0)
#endif
#	define usbDbgReset() do {} while(0)
#endif

#if USBH_DEBUG_MULTI_HOST
#define _usbh_dbg(host, s) usbDbgPuts(host, s)
#define _usbh_dbgf(host, f, ...) usbDbgPrintf(host, f, ##__VA_ARGS__)
#define _usbh_ldbg(host, lvl, n, s) do {if (lvl >= n) usbDbgPuts(host, s); } while(0)
#define _usbh_ldbgf(host, lvl, n, f, ...) do {if (lvl >= n) usbDbgPrintf(host, f, ##__VA_ARGS__); } while(0)
#else

#define _usbh_dbg(host, s) usbDbgPuts(s)
#define _usbh_dbgf(host, f, ...) usbDbgPrintf(f, ##__VA_ARGS__)
#define _usbh_ldbg(host, lvl, n, s) do {if (lvl >= n) usbDbgPuts(s); } while(0)
#define _usbh_ldbgf(host, lvl, n, f, ...) do {if (lvl >= n) usbDbgPrintf(f, ##__VA_ARGS__); } while(0)

#endif


#endif

#endif /* USBH_DEBUG_H_ */
