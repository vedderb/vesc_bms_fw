/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "tusb.h"

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint16_t blink_timer = TIME_MS2I(BLINK_NOT_MOUNTED);

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_timer = TIME_MS2I(BLINK_MOUNTED);
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_timer = TIME_MS2I(BLINK_NOT_MOUNTED);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_timer = TIME_MS2I(BLINK_SUSPENDED);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_timer = TIME_MS2I(BLINK_MOUNTED);
}

static THD_WORKING_AREA(waUsbThread, 512);
static THD_FUNCTION(UsbThread, arg) {
  (void) arg;

  tusb_init();

  // RTOS forever loop
  while (1)
  {
    // tinyusb device task
    tud_task();
  }
}

static THD_WORKING_AREA(waCdcThread, 256);
static THD_FUNCTION(CdcThread, arg) {
  (void) arg;

    while (true) {
      // connected() check for DTR bit
      // Most but not all terminal client set this when making connection
      // if ( tud_cdc_connected() )
      {
        // There are data available
        if ( tud_cdc_available() )
        {
          uint8_t buf[64];

          // read and echo back
          uint32_t count = tud_cdc_read(buf, sizeof(buf));

          for(uint32_t i=0; i<count; i++)
          {
            tud_cdc_write_char(buf[i]);

            if ( buf[i] == '\r' ) tud_cdc_write_char('\n');
          }

          tud_cdc_write_flush();
        }
      }
      chThdSleepMilliseconds(10);
    }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;

  // connected
  if ( dtr )
  {
    // print initial message when connected
    tud_cdc_write_str("\r\nTinyUSB CDC MSC device with FreeRTOS example\r\n");
    tud_cdc_write_flush();
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
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

  chThdCreateStatic(waUsbThread, sizeof(waUsbThread), HIGHPRIO-1, UsbThread, NULL);
  chThdCreateStatic(waCdcThread, sizeof(waCdcThread), HIGHPRIO-2, CdcThread, NULL);

  while (1)
  {
    chThdSleepMilliseconds(blink_timer);
    //palToggleLine(LINE_LED);
  }
}
