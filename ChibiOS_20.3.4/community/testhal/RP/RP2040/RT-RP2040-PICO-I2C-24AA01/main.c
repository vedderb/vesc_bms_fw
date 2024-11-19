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

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#define LED_GREEN_PIN  25U

#define I2C0_SDA_PIN   12U
#define I2C0_SCL_PIN   13U

// This is I2C0
#define I2CN  I2CD1

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static CH_SYS_CORE0_MEMORY THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    chThdSleepMilliseconds(500);
    palToggleLine(LED_GREEN_PIN);
  }
}

#define EEPROM_ADDRESS 0x50

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
   * Activates the UART0 SIO driver using the default configuration.
   */
  sioStart(&SIOD0, NULL);
  sioStartOperation(&SIOD0, NULL);

  /* LED GPIO */
  palSetLineMode(LED_GREEN_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);

  I2CConfig i2cConfig = {
    400000, // baudrate
  };
  i2cStart(&I2CN, &i2cConfig);

  /* Set up I2C pins. */
  palSetLineMode(I2C0_SDA_PIN, PAL_MODE_ALTERNATE_I2C | PAL_RP_PAD_PUE | PAL_RP_PAD_DRIVE4);
  palSetLineMode(I2C0_SCL_PIN, PAL_MODE_ALTERNATE_I2C | PAL_RP_PAD_PUE | PAL_RP_PAD_DRIVE4);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chThdSleepMilliseconds(100);

  chprintf((BaseSequentialStream *)&SIOD0, "--\r\n");

  const uint8_t target_address = 0x00;
  // Write data
  const uint8_t initData[] = {
    target_address, // word address
    0x80, 0xFF, 0x01, 0x7F, 0xF1, 0x08, 0x0, 0x12,
  };

  msg_t msg;

  msg = i2cMasterTransmitTimeout(&I2CN, EEPROM_ADDRESS, (uint8_t*)&initData, sizeof(initData), NULL, 0, 1000);
  chprintf((BaseSequentialStream *)&SIOD0, "write: %d\r\n", msg);
  if (msg == MSG_OK) {

    uint8_t readData[10];
    memset(readData, 0xAA, 10);

    // read 4 bytes
    msg = i2cMasterTransmitTimeout(&I2CN, EEPROM_ADDRESS, (uint8_t*)&target_address, 1, (uint8_t*)&readData, 4, 1000);
    chprintf((BaseSequentialStream *)&SIOD0, "write and read: %d\r\n", msg);
    chprintf((BaseSequentialStream *)&SIOD0, "data: %d\r\n",
      readData[0] == initData[1] &&
      readData[1] == initData[2] &&
      readData[2] == initData[3] &&
      readData[3] == initData[4]);
    for (int i = 0; i < 8; i++) {
      chprintf((BaseSequentialStream *)&SIOD0, "[%d]: 0x%X\r\n", i, readData[i]);
    }

    // read next 4 bytes
    msg = i2cMasterReceiveTimeout(&I2CN, EEPROM_ADDRESS, (uint8_t*)&readData[4], 4, 1000);
    chprintf((BaseSequentialStream *)&SIOD0, "read: %d\r\n", msg);
    chprintf((BaseSequentialStream *)&SIOD0, "data: %d\r\n",
      readData[4] == initData[5] &&
      readData[5] == initData[6] &&
      readData[6] == initData[7] &&
      readData[7] == initData[8]);
    for (int i = 0; i < 10; i++) {
      chprintf((BaseSequentialStream *)&SIOD0, "[%d]: 0x%X\r\n", i, readData[i]);
    }
    if (msg = MSG_OK) {
      uint32_t error = i2cGetErrors(&I2CN);
      chprintf((BaseSequentialStream *)&SIOD0, "error: %d\r\n", error);
    }
  }

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop.
   */
  while (true) {
    chThdSleepMilliseconds(500);
  }
}
