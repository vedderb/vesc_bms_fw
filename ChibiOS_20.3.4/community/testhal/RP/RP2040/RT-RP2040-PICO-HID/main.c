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

#include "usbcfg.h"

#define LED_GREEN_PIN  25U

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static CH_SYS_CORE0_MEMORY THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    systime_t time = USBD1.state == USB_ACTIVE ? 250 : 1000;
    chThdSleepMilliseconds(time);
    palToggleLine(LED_GREEN_PIN);
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
   * Initializes a USB HID driver.
   */
  hidObjectInit(&UHD1);
  hidStart(&UHD1, &usbhidcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(usbhidcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(usbhidcfg.usbp, &usbcfg);
  usbConnectBus(usbhidcfg.usbp);

  /*
   * Setting up GPIOs.
   */
  palSetLineMode(LED_GREEN_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop.
   */
  while (true) {
    if (usbhidcfg.usbp->state == USB_ACTIVE) {
      uint8_t report;
      size_t n = hidGetReport(0, &report, sizeof(report));
      hidWriteReport(&UHD1, &report, n);
      n = hidReadReportt(&UHD1, &report, sizeof(report), TIME_IMMEDIATE);
      if (n > 0)
        hidSetReport(0, &report, n);
    }
    chThdSleepMilliseconds(500);
  }
}
