/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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

#include "ch.h"
#include "chthreads.h"
#include "hal.h"
#include "ch_test.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "chprintf.h"
#include "shell.h"
#include <string.h>

#include "usbcfg.h"

/* This demo can be customized in scope for easier debugging.
 *
 * Define one of the following: */

/* Run rt and oslib test suite to serial console: */
//#define DEMO_MODE_TESTS_TO_SERIAL

/* Provide an interactive shell on serial console.
 * You can run the tests using the test command: */
//#define DEMO_MODE_SHELL_ON_SERIAL

/* Provide an interactive shell on serial console over USB.
 * This mode does not require any extra serial hardware: */
#define DEMO_MODE_SHELL_ON_USB_SERIAL

/*
 * Serial 1 (LPUART1) corresponds to Pin 24 on the Teensy 4.1, or to the built-in
 * usb-to-serial on the debug probe of the MIMXRT1060-EVK.
 */
#define MYSERIAL &SD1

/*
 * LED blinker thread.
 */
static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    chRegSetThreadName("LEDBlinker");
    while (true) {
        palTogglePad(TEENSY_PIN13_IOPORT, TEENSY_PIN13);
        chThdSleepSeconds(1);
    }
}

#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static const ShellCommand commands[] = {{NULL, NULL}};

static const ShellConfig shell_cfg1 = {
#ifdef DEMO_MODE_SHELL_ON_USB_SERIAL
  (BaseSequentialStream *)&SDU1,
#else
  (BaseSequentialStream *)MYSERIAL,
#endif

  commands
};

char buf[1024];

#include "fsl_lpuart.h"
void printf_debug(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  int n = chvsnprintf(buf, sizeof(buf), format, args);
  // Directly write to serial instead of using SD4 BaseSequentialStream, because
  // the latter does not work from within a locked section (e.g. usbStart grabs
  // a lock).
  buf[n] = '\r';
  buf[n+1] = '\n';
  buf[n+2] = '\0';
  LPUART_WriteBlocking(LPUART1, (unsigned char*)buf, n+2);
  va_end(args);
}

semaphore_t scls;

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  chSemObjectInit(&scls, 0);
  
  /*
   * Activates MYSERIAL with 115200 baud.
   */

  const SerialConfig sc = {
    .sc_speed = 115200,
  };
  sdStart(MYSERIAL, &sc);

  chprintf((BaseSequentialStream*)MYSERIAL, "ChibiOS Teensy 4.1 demo\r\n");

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

#if defined(DEMO_MODE_TESTS_TO_SERIAL)
  test_execute((BaseSequentialStream *)MYSERIAL, &rt_test_suite);
  test_execute((BaseSequentialStream *)MYSERIAL, &oslib_test_suite);
#elif defined(DEMO_MODE_SHELL_ON_SERIAL)
  while (true) {
    chprintf((BaseSequentialStream*)MYSERIAL, "Starting serial shell\r\n");
    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO + 1, shellThread, (void *)&shell_cfg1);
    chThdWait(shelltp); /* Waiting termination.             */
  }
#elif defined(DEMO_MODE_SHELL_ON_USB_SERIAL)
  chprintf((BaseSequentialStream*)MYSERIAL, "Starting USB serial\r\n");
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  while (true) {
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      // Wait until sduRequestsHook CDC_SET_CONTROL_LINE_STATE happens, then
      // sleep for a certain time to give the app a chance to configure flags.
      chSemWait(&scls);
      chThdSleepMilliseconds(100);

      chprintf((BaseSequentialStream*)MYSERIAL, "Starting serial-over-USB CDC Shell\r\n");
      thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO + 1, shellThread, (void *)&shell_cfg1);
      chThdWait(shelltp); /* Waiting termination.             */
    }
    chThdSleepSeconds(1);
  }
#else
#error One of DEMO_MODE_TESTS_TO_SERIAL, DEMO_MODE_SHELL_ON_SERIAL or DEMO_MODE_SHELL_ON_USB_SERIAL must be defined
#endif

  return 0;
}
