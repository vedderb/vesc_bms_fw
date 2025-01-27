/*
    Copyright (C) 2017 PJRC.COM, LLC.

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the
    "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish,
    distribute, sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so, subject to
    the following conditions:
    
    1. The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.
    
    2. If the Software is incorporated into a build system that allows
    selection among a list of target devices, then similar target
    devices manufactured by PJRC.COM must be included in the list of
    target devices and selectable in the same manner.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
    BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
    ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

// Enable to get startup debug on ChibiOS LPUART3 (== NXP peripheral LPUART4),
// i.e. pin 17 on the Teensy 4.1:
#if 0 /* PRINT_DEBUG_STUFF */

#include <stdarg.h>
#include "fsl_lpuart.h"

void putchar_debug(char c);
static void puint_debug(unsigned int num);


 void printf_debug(const char *format, ...)
{
	va_list args;
	unsigned int val;
	int n;

	va_start(args, format);
	for (; *format != 0; format++) { // no-frills stand-alone printf
		if (*format == '%') {
			++format;
			if (*format == '%') goto out;
			if (*format == '-') format++; // ignore size
			while (*format >= '0' && *format <= '9') format++; // ignore size
			if (*format == 'l') format++; // ignore long
			if (*format == '\0') break;
			if (*format == 's') {
				printf_debug((char *)va_arg(args, int));
			} else if (*format == 'd') {
				n = va_arg(args, int);
				if (n < 0) {
					n = -n;
					putchar_debug('-');
				}
				puint_debug(n);
			} else if (*format == 'u') {
				puint_debug(va_arg(args, unsigned int));
			} else if (*format == 'x' || *format == 'X') {
				val = va_arg(args, unsigned int);
				for (n=0; n < 8; n++) {
					unsigned int d = (val >> 28) & 15;
					putchar_debug((d < 10) ? d + '0' : d - 10 + 'A');
					val <<= 4;
				}
			} else if (*format == 'c' ) {
				putchar_debug((char)va_arg(args, int));
			}
		} else {
			out:
			if (*format == '\n') putchar_debug('\r');
			putchar_debug(*format);
		}
	}
	va_end(args);
}

static void puint_debug(unsigned int num)
{
	char buf[12];
	unsigned int i = sizeof(buf)-2;

	buf[sizeof(buf)-1] = 0;
	while (1) {
		buf[i] = (num % 10) + '0';
		num /= 10;
		if (num == 0) break;
		i--;
	}
	printf_debug(buf + i);
}

void putchar_debug(char c)
{
  while (!(LPUART3->STAT & LPUART_STAT_TDRE(1))) ; // wait
	LPUART3->DATA = c;
}

void printf_debug_init(void)
{
  CCM->CCGR0 |= CCM_CCGR0_CG6(1); // turn on Serial4
  IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06] = 2; // Arduino pin 17
  LPUART3->BAUD = LPUART_BAUD_OSR(25) | LPUART_BAUD_SBR(8); // ~115200 baud
  LPUART3->CTRL = LPUART_CTRL_TE(1);
}

#endif
