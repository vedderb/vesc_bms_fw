#pragma once

#define printf_init()
#define printf(...)
#define printf_debug_init()
#define printf_debug(...)

// Comment out to enable debug messages by a printf_debug implementation of your
// choice, e.g. os/hal/ports/MIMXRT1062/MIMXRT1062/printf_debug.c:
//
// #include "chprintf.h"
// #undef printf_debug
// extern void printf_debug(const char *format, ...);
