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


void comp2_cb(COMPDriver *comp) {

  /* Check if output is high (rising) */
  if (comp->reg->CSR & COMP_CSR_COMPxOUTVALUE) {

    palToggleLine(LINE_LED_RED);
  }

}

static const COMPConfig comp2_conf = {
  COMP_OUTPUT_NORMAL,
  COMP_IRQ_RISING,
  comp2_cb,
  STM32_COMP_InvertingInput_VREFINT // CSR
};

/*
 * Application entry point.
 */
int main(void) {

  halInit();
  chSysInit();

  /*
  * Set PA3 to Analog (COMP2_INP)
  * Callback will trigger when voltage rises above VREFINT
  */
  palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);

  /*
   * Set PA2 to alternate 8 (COMP2_OUT)
   * You can connect this to an oscilloscope along with PA4 to compare input/output.
   */
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(8));

  /*
   * Start peripherals
   */
  compStart(&COMPD2, &comp2_conf);
  compEnable(&COMPD2);

  /*
   * Normal main() thread activity.
   */
  while (true) {

    chThdSleepMilliseconds(500);
    palToggleLine(LINE_LED_GREEN);
  }
  return 0;
}
