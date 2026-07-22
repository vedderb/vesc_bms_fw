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

#ifdef USB_DEBUG

#include "rp_fifo.h"
#include "chprintf.h"
#include "chmtx.h"

mutex_t mtx;

uint8_t remained_data = 0;

#define BUFFER_LEN 32

uint32_t buffer[BUFFER_LEN] = {
  0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
  0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
uint8_t current_buffer_index = 0;
uint8_t next_buffer_index = 0;
uint8_t remained_len = 0;

void process_command(uint32_t data) {
  //chMtxLock(&mtx);

  buffer[next_buffer_index] = data;
  next_buffer_index += 1;
  if (next_buffer_index >= BUFFER_LEN) {
    next_buffer_index = 0;
  }
  remained_len += 1;

  //chMtxUnlock(&mtx);
}

#define CMD_RESET 0x0F
#define CMD_SETUP 0x01
#define CMD_READ_SETUP 0x02
#define CMD_SET_ADDR 0x03
#define CMD_START_IN 0x04
#define CMD_START_OUT 0x05
#define CMD_BUFF_STATUS 0x06
#define CMD_EP_DONE 0x07
#define CMD_DATA_ERROR 0x09
#define CMD_PREP_IN_EP  0x0A
#define CMD_PREP_OUT_EP 0x0B
#define CMD_SET_ADDR_HW 0x0C
#define CMD_EP_NEXT 0x0D

#endif // USB_DEBUG

/**
 * Core 1 entry point.
 */
void c1_main(void) {

  /*
   * Starting a new OS instance running on this core, we need to wait for
   * system initialization on the other side.
   */
  chSysWaitSystemState(ch_sys_running);
  chInstanceObjectInit(&ch1, &ch_core1_cfg);

  /* It is alive now.*/
  chSysUnlock();

#ifdef USB_DEBUG
  chMtxObjectInit(&mtx);
  //chSemObjectInit(&buff_sem, 0);
  /*
   * Setting up GPIOs.
   */
  palSetLineMode(0U, PAL_MODE_ALTERNATE_UART);
  palSetLineMode(1U, PAL_MODE_ALTERNATE_UART);

  /*
   * Activates the UART0 SIO driver using the default configuration.
   */
  sioStart(&SIOD0, NULL);
  sioStartOperation(&SIOD0, NULL);

  chThdSleepMilliseconds(100);
  chprintf((BaseSequentialStream *)&SIOD0, "-- Started in c1\r\n");
#endif // USB_DEBUG

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop (re)spawning a shell.
   */
  while (true) {
#ifdef USB_DEBUG
    chMtxLock(&mtx);
    if (remained_len > 0) {
      const uint32_t data = buffer[current_buffer_index];
      current_buffer_index += 1;
      if (current_buffer_index >= BUFFER_LEN) {
        current_buffer_index = 0;
      }
      remained_len -= 1;

      if (remained_data > 0) {
        remained_data -= 1;
        chprintf((BaseSequentialStream *)&SIOD0, "0x%X", data);
        if (remained_data) {
          chprintf((BaseSequentialStream *)&SIOD0, ", ");
        } else {
          chprintf((BaseSequentialStream *)&SIOD0, "\r\n");
        }
      } else {
        uint8_t cmd = (data & 0xFF000000) >> 24;
        uint8_t length = (data & 0xFF0000) >> 16;
        if (length > 0) {
          remained_data = length;
        }
        const char *text = NULL;
        switch (cmd) {
          case CMD_RESET: text = "Reset\r\n"; break;
          case CMD_SETUP: text = "Setup req\r\n"; break;
          case CMD_READ_SETUP: text = "Read setup data: "; break;
          case CMD_SET_ADDR: text = "Set address: "; break;
          case CMD_START_IN: text = "Start in: "; break;
          case CMD_START_OUT: text = "Start out: "; break;
          case CMD_EP_DONE: text = "Endpoint done: "; break;
          case CMD_BUFF_STATUS: text = "Buffer status: "; break;
          case CMD_PREP_IN_EP: text = "Prepare IN endpoint: "; break;
          case CMD_PREP_OUT_EP: text = "Prepare OUT endpoint: "; break;
          case CMD_SET_ADDR_HW: text = "Set address to hw: "; break;
          case CMD_DATA_ERROR: text = "Data error\r\n"; break;
          default:
            break;
        }
        if (text) {
          chprintf((BaseSequentialStream *)&SIOD0, text);
        } else {
          chprintf((BaseSequentialStream *)&SIOD0, "%X\r\n", cmd);
        }
      }
    }
    chMtxUnlock(&mtx);
#else
    chThdSleepMilliseconds(50);
#endif // USB_DEBUG
  }
}
