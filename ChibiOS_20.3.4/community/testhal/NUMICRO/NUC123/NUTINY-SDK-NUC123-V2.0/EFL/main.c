/*
  Copyright (C) 2021 Alex Lewontin

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
#include "shcfg.h"


const SerialConfig shell_serial_cfg = {
                           .speed  = 57600,
                           .mode   = NUC123_SERIAL_MODE_DEFAULT,
                           .data   = NUC123_SERIAL_DATA_8BITS,
                           .parity = NUC123_SERIAL_PARITY_N,
                           .stop   = NUC123_SERIAL_STOP_1};


/*
 * Onboard LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waBlinkThread, 128);
static THD_FUNCTION(BlinkThread, arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    systime_t time = 500;
    OnboardLED_Toggle();
    chThdSleepMilliseconds(time);
  }
}

/*
 * Application entry point.
 */
int main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  OnboardLED_Init();

  /*
   * Turn off the onboard LED.
   */
  OnboardLED_Off();

  chDbgSuspendTrace(CH_DBG_TRACE_MASK_SWITCH);

  /*
   * Activates the serial driver.
   */
  sdStart(&SHELL_SERIAL_DRIVER, &shell_serial_cfg);

  /*
   * Shell manager initialization.
   */
  shellInit();


  eflStart(&EFLD1, NULL);
  EFLD1.bank = NUC123_EFL_BANK_DATAFLASH;
  mfsObjectInit(&mfsd);
  mfsStart(&mfsd, &mfsd_config);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(
      waBlinkThread, sizeof(waBlinkThread), NORMALPRIO, BlinkThread, NULL);

  while (true) {
    thread_t *shelltp = chThdCreateFromHeap(NULL,
                                            SHELL_WA_SIZE,
                                            "shell",
                                            NORMALPRIO + 1,
                                            shellThread,
                                            (void *)&shell_cfg);
    chThdWait(shelltp); /* Waiting termination.             */
    chThdSleepMilliseconds(1000);
  }
}
