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
#include "hal.h"
#include "ch_test.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("RedBlinker");
  while (true) {
    palTogglePad(IOPORT3, 3);
    chThdSleepMilliseconds(300);
  }
}

static THD_WORKING_AREA(waThread2, 64);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  chRegSetThreadName("GreenBlinker");
  while (true) {
    palTogglePad(IOPORT4, 4);
    chThdSleepMilliseconds(600);
  }
}

static THD_WORKING_AREA(waThread3, 64);
static THD_FUNCTION(Thread3, arg) {

  (void)arg;
  chRegSetThreadName("BlueBlinker");
  while (true) {
    palTogglePad(IOPORT1, 2);
    chThdSleepMilliseconds(900);
  }
}

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

  /*
   * Activates serial 1 (UART0) using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /*
   * Creates the blinker threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);

  test_execute((BaseSequentialStream *)&SD1, &rt_test_suite);
  test_execute((BaseSequentialStream *)&SD1, &oslib_test_suite);
  while (1) {
    chThdSleepMilliseconds(500);
  }
}
