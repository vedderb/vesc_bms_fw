/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
#include "rt_test_root.h"
#include "oslib_test_root.h"

/*
 * Green and Blue LEDs blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");

  while (true) {
    palSetLine(LINE_LED_BLUE);
    chThdSleepMilliseconds(50);
    palSetLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(150);
    palClearLine(LINE_LED_BLUE);
    chThdSleepMilliseconds(50);
    palClearLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(150);
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
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

  /*
   * Activates the serial driver 0 using the driver default configuration.
   * P0.1(TX) and P0.2(RX) are routed to UART0.
   */
  sdStart(&SD0, NULL);
  palSetPadMode(GP0, 1, PAL_MODE_MULTIPLEXER(3) | PAL_ADUCM_PUL_PULLUP);
  palSetPadMode(GP0, 2, PAL_MODE_MULTIPLEXER(3) | PAL_ADUCM_PUL_PULLUP);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    if (!palReadLine(LINE_BUTTON)) {
      test_execute((BaseSequentialStream *)&SD0, &rt_test_suite);
      test_execute((BaseSequentialStream *)&SD0, &oslib_test_suite);
    }
    chThdSleepMilliseconds(500);
  }
}
