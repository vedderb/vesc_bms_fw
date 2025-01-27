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
#include "usbdfu.h"
#include <string.h>

#define CM_RESET_VECTOR_OFFSET    4

static void jump_to_application(void) __attribute__ ((noreturn));

static void jump_to_application(void) {

    /* Use the application's vector table */
    // Copy Vector Table to RAM_START(0x10000000)
    memcpy((void*) 0x10000000, (void*)APP_BASE, 512);

    /* Initialize the application's stack pointer */
    __set_MSP(*((volatile uint32_t*)(APP_BASE)));
    uint32_t target_start = *((volatile uint32_t*)(APP_BASE + CM_RESET_VECTOR_OFFSET));
    uint32_t initial_sp = *((volatile uint32_t*)(APP_BASE));
    /* Jump to the application entry point */
    __ASM volatile ("mov sp, %0\n" "bx %1" : : "r" (initial_sp), "r" (target_start) : );

    while (1) {}
}

/*
 * Application entry point.
 */
int main(void) {
  halInit();
  /*
   * System initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  chSysInit();

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * increasing the minutes counter.
   */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(1500);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

  while(1){
    chThdSleepSeconds(600);
  }
}
