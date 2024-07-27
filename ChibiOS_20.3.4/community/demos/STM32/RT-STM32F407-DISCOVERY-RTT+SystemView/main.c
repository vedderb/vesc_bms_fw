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
#include "SEGGER_SYSVIEW_ChibiOS.h"
#include "SEGGER_RTT_streams.h"

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palSetPad(GPIOD, GPIOD_LED3);       /* Orange.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOD, GPIOD_LED3);     /* Orange.  */
    chThdSleepMilliseconds(500);
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
   * This is optional. You need to initialize this if you will be using the
   * RTT sequential streams (RTTD0). You *don't* need to use them; you can
   * use RTT directly (call SEGGER_RTT_xxxx) without any initialization.
   *
   *
   * Initialize RTT (channel 0). The output can be seen on Telnet por 19201.
   */
  rttInit();
  rttSetUpFlags(&RTTD0, RTT_MODE_FLAGS_BLOCK_IF_FIFO_FULL);

  /*
   * Start SystemView
   */
  SYSVIEW_ChibiOS_Start(STM32_SYSCLK, STM32_SYSCLK, "I#44=OSTick,I#54=USART2");

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  streamWrite(&SD2, (const uint8_t *)"Hello From SD2!\r\n", 17);
  streamWrite(&RTTD0, (const uint8_t *)"Hello From RTT!\r\n", 17);

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    if (palReadPad(GPIOA, GPIOA_BUTTON)) {
      SEGGER_SYSVIEW_PrintfHost("Test begins with output on SD2\r\n");
      test_execute((BaseSequentialStream *)&SD2, &rt_test_suite);
      test_execute((BaseSequentialStream *)&SD2, &oslib_test_suite);

      SEGGER_SYSVIEW_PrintfHost("Test begins with output on RTT\r\n");
      test_execute((BaseSequentialStream *)&RTTD0, &rt_test_suite);
      test_execute((BaseSequentialStream *)&RTTD0, &oslib_test_suite);
    }
    chThdSleepMilliseconds(500);
  }
}
