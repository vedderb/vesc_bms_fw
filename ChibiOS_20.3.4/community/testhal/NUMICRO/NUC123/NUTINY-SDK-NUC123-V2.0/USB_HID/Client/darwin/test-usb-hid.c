/*
  Copyright (c) 2014 Guillaume Duc <guillaume@guiduc.org>
  Modifications copyright (c) 2020 Alex Lewontin <alex.c.lewontin@gmail.com>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

#include "hidapi.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define USB_HID_IN_REPORT_SIZE  1
#define USB_HID_OUT_REPORT_SIZE 1

struct usb_hid_in_report_s {
  uint8_t sequence_number;
};

struct usb_hid_out_report_s {
  uint8_t sequence_number;
};

static uint8_t usb_hid_in_report_buf[USB_HID_IN_REPORT_SIZE];
/* +1 for the report index */
static uint8_t usb_hid_out_report_buf[USB_HID_OUT_REPORT_SIZE + 1];

static struct usb_hid_in_report_s* usb_hid_in_report =
    (struct usb_hid_in_report_s*)usb_hid_in_report_buf;

static struct usb_hid_out_report_s* usb_hid_out_report =
    (struct usb_hid_out_report_s*)(&usb_hid_out_report_buf[1]);

static hid_device* handle;

static void close_hidapi() __attribute__((noreturn));
static void close_client() __attribute__((noreturn));

static void read_in_report()
{
  int res, i;

  printf("read()\n");
  res = hid_read(handle, usb_hid_in_report_buf, USB_HID_IN_REPORT_SIZE);
  if (res < 0) {
    perror("read");
    exit(EXIT_FAILURE);
  } else {
    printf("read() read %d bytes:\t", res);
    for (i = 0; i < res; i++)
      printf("%02hhx ", usb_hid_in_report_buf[i]);
    printf("\n");
  }
}

static void send_out_report()
{
  int res;

  usb_hid_out_report_buf[0] = 0;

  res =
      hid_write(handle, usb_hid_out_report_buf, USB_HID_OUT_REPORT_SIZE + 1);
  if (res < 0) {
    perror("write");
    exit(EXIT_FAILURE);
  }

  usb_hid_out_report->sequence_number++;
}

static void close_hidapi()
{
  int res = hid_exit();
  if (res) {
    perror("Could not close hidapi library");
    exit(EXIT_FAILURE);
  }
  exit(EXIT_SUCCESS);
}

static void close_client()
{
  hid_close(handle);
  close_hidapi();
}

int main(int argc, char** argv)
{
  int                     res;
  unsigned long           vid, pid;
  struct hid_device_info *devs, *cur_dev;

  if (argc < 2) {
    fprintf(stderr, "Usage: %s [VID] [PID]\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  vid = strtoul(argv[1], NULL, 16);
  pid = strtoul(argv[2], NULL, 16);

  devs    = hid_enumerate(0x0, 0x0);
  cur_dev = devs;
  while (cur_dev) {
    printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: "
           "%ls",
           cur_dev->vendor_id,
           cur_dev->product_id,
           cur_dev->path,
           cur_dev->serial_number);
    printf("\n");
    printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
    printf("  Product:      %ls\n", cur_dev->product_string);
    printf("  Release:      %hx\n", cur_dev->release_number);
    printf("  Interface:    %d\n", cur_dev->interface_number);
    printf("  Usage (page): 0x%hx (0x%hx)\n",
           cur_dev->usage,
           cur_dev->usage_page);
    printf("\n");
    cur_dev = cur_dev->next;
  }
  hid_free_enumeration(devs);

  /* Make sure we clean up on CTRL-C interrupt */
  signal(SIGINT, close_client);

  res = hid_init();
  if (res) {
    perror("Could not load hidapi library");
    exit(EXIT_FAILURE);
  }
  handle = hid_open(vid, pid, NULL);
  if (!handle) {
    perror("Unable to open device");
    close_hidapi();
    exit(EXIT_FAILURE);
  }

  usb_hid_out_report->sequence_number = 4;
  send_out_report();

  while (1) {
    read_in_report();

    if (usb_hid_in_report->sequence_number == 40) {
      usb_hid_out_report->sequence_number =
          usb_hid_in_report->sequence_number / 2;
      send_out_report();
    }
  }
}
