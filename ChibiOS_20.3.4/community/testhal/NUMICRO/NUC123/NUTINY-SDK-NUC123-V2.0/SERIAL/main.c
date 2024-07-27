/*
  ChibiOS - Copyright (C) 2020 Alex Lewontin

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
#include "chprintf.h"

#define SERIAL_DRIVER   SD0

#define toupper(c) __builtin_toupper(c)

char buf;

const SerialConfig scfg = {.speed  = 57600,
                           .mode   = NUC123_SERIAL_MODE_DEFAULT,
                           .data   = NUC123_SERIAL_DATA_8BITS,
                           .parity = NUC123_SERIAL_PARITY_N,
                           .stop   = NUC123_SERIAL_STOP_1};

/*
 * Application entry point.
 */
int main(void)
{

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   */

  halInit();

  /*
   * Enabling interrupts, initialization done.
   */
  osalSysEnable();

  sdStart(&SERIAL_DRIVER, &scfg);

  chprintf((BaseSequentialStream *)&SERIAL_DRIVER, "\nInitialized...\n");
  while (true) {
    buf = streamGet((BaseSequentialStream *)&SERIAL_DRIVER);
    buf = toupper(buf);
    streamPut((BaseSequentialStream *)&SERIAL_DRIVER, buf);
  }
}
