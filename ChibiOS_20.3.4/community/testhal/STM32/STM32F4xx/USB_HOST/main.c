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
#include "ff.h"
#include <string.h>
#include "usbh/debug.h"		/* for _usbh_dbg/_usbh_dbgf */


#define UVC_TO_MSD_PHOTOS_CAPTURE	FALSE


#if HAL_USBH_USE_FTDI || HAL_USBH_USE_AOA
static uint8_t buf[] =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
#endif

#if HAL_USBH_USE_FTDI
#include "usbh/dev/ftdi.h"
#include "shell.h"
#include "chprintf.h"

static THD_WORKING_AREA(waTestFTDI, 1024);

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)

static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: write\r\n");
    return;
  }

  while (chnGetTimeout((BaseChannel *)chp, TIME_IMMEDIATE) != Q_TIMEOUT) {
    //flush
  }

  while (chnGetTimeout((BaseChannel *)chp, TIME_IMMEDIATE) == Q_TIMEOUT) {
    streamWrite(&FTDIPD[0], buf, sizeof buf - 1);
  }
  chprintf(chp, "\r\n\nstopped\r\n");
}

static const ShellCommand commands[] = {
    {"write", cmd_write},
    {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream *)&FTDIPD[0],
    commands
};

static void ThreadTestFTDI(void *p) {
    (void)p;
    USBHFTDIPortDriver *const ftdipp = &FTDIPD[0];
    USBHDriver *host = NULL;
    (void)host;

    chRegSetThreadName("FTDI");

    shellInit();

start:
    while (usbhftdipGetState(ftdipp) != USBHFTDIP_STATE_ACTIVE) {
        chThdSleepMilliseconds(100);
    }

    host = usbhftdipGetHost(ftdipp);
	_usbh_dbg(host, "FTDI: Connected");

    USBHFTDIPortConfig config = {
        115200,
        USBHFTDI_FRAMING_DATABITS_8 | USBHFTDI_FRAMING_PARITY_NONE | USBHFTDI_FRAMING_STOP_BITS_1,
        USBHFTDI_HANDSHAKE_NONE,
        0,
        0
    };

    usbhftdipStart(ftdipp, &config);

    //loopback
    if (0) {
        for(;;) {
            msg_t m = streamGet(ftdipp);
            if (m < MSG_OK) {
                _usbh_dbg(host, "FTDI: Disconnected");
                goto start;
            }
            streamPut(ftdipp, (uint8_t)m);
            if (m == 'q')
                break;
        }
    }

    //shell test
    if (1) {
        thread_t *shelltp = NULL;
        for(;;) {
            if (usbhftdipGetState(ftdipp) != USBHFTDIP_STATE_READY)
                goto start;
            if (!shelltp) {
                shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                              "shell", NORMALPRIO,
                                              shellThread, (void *)&shell_cfg1);
            } else if (chThdTerminatedX(shelltp)) {
                chThdRelease(shelltp);
                if (usbhftdipGetState(ftdipp) != USBHFTDIP_STATE_READY)
                    goto start;
                break;
            }
            chThdSleepMilliseconds(100);
        }
    }

    //FTDI uart RX to debug TX bridge
    if (0) {
        for(;;) {
            msg_t m = streamGet(ftdipp);
            if (m < MSG_OK) {
                _usbh_dbg(host, "FTDI: Disconnected");
                goto start;
            }
            sdPut(&SD2, (uint8_t)m);
            if (m == 'q')
                break;
        }
    }

    //write speed test
    if (1) {
        usbhftdipStop(ftdipp);
        config.speed = 3000000;
        usbhftdipStart(ftdipp, &config);

        systime_t st, et;
        int i;
        for (i = 0; i < 5; i++) {
            uint32_t bytes = config.speed / 10;
            uint32_t times = bytes / 1024;
            st = chVTGetSystemTimeX();
            while (times--) {
                if (streamWrite(ftdipp, buf, 1024) < 1024) {
                    _usbh_dbg(host, "FTDI: Disconnected");
                    goto start;
                }
                bytes -= 1024;
            }
            if (bytes) {
                if (streamWrite(ftdipp, buf, bytes) < bytes) {
                    _usbh_dbg(host, "FTDI: Disconnected");
                    goto start;
                }
            }
            et = chVTGetSystemTimeX();
            _usbh_dbgf(host, "\tRate=%uB/s", (config.speed * 100) / (et - st));
        }
    }

    //single character write test (tests the timer)
    if (0) {
        for (;;) {
            if (streamPut(ftdipp, 'A') != MSG_OK) {
                _usbh_dbg(host, "FTDI: Disconnected");
                goto start;
            }
            chThdSleepMilliseconds(100);
        }
    }

    usbhftdipStop(ftdipp);

    _usbh_dbg(host, "FTDI: Tests done, restarting in 3s");
    chThdSleepMilliseconds(3000);

    goto start;
}
#endif

#if HAL_USBH_USE_AOA
#include "usbh/dev/aoa.h"
#include "chprintf.h"

static THD_WORKING_AREA(waTestAOA, 1024);

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)

static void ThreadTestAOA(void *p) {
    (void)p;
    USBHAOADriver *const aoap = &USBHAOAD[0];
    USBHAOAChannel *const aoacp = &aoap->channel;
    USBHDriver *host = NULL;
    (void)host;

    chRegSetThreadName("AOA");

start:
    while (usbhaoaGetState(aoap) != USBHAOA_STATE_READY) {
        chThdSleepMilliseconds(100);
    }

    host = usbhaoaGetHost(aoap);
    _usbh_dbg(host, "AOA: Connected");

    if (usbhaoaGetChannelState(aoap) != USBHAOA_CHANNEL_STATE_READY) {
        usbhaoaChannelStart(aoap);
        _usbh_dbg(host, "AOA: Channel started");
    }

    //loopback
    if (1) {
        for(;;) {
            msg_t m = streamGet(aoacp);
            if (m < MSG_OK) {
                _usbh_dbg(host, "AOA: Disconnected");
                goto start;
            }
            streamPut(aoacp, (uint8_t)m);
            if (m == 'q')
                break;
        }
    }

#define AOA_WRITE_SPEED_TEST_BYTES    3000000UL
    //write speed test
    if (1) {
        systime_t st, et;
        int i;
        for (i = 0; i < 5; i++) {
            uint32_t bytes = AOA_WRITE_SPEED_TEST_BYTES;
            uint32_t times = bytes / 1024;
            st = chVTGetSystemTimeX();
            while (times--) {
                if (streamWrite(aoacp, buf, 1024) < 1024) {
                    _usbh_dbg(host, "AOA: Disconnected");
                    goto start;
                }
                bytes -= 1024;
            }
            if (bytes) {
                if (streamWrite(aoacp, buf, bytes) < bytes) {
                    _usbh_dbg(host, "AOA: Disconnected");
                    goto start;
                }
            }
            et = chVTGetSystemTimeX();
            _usbh_dbgf(host, "\tRate=%uB/s", AOA_WRITE_SPEED_TEST_BYTES / (et - st) * 100);
        }
    }

    //single character write test (tests the timer)
    if (0) {
        for (;;) {
            if (streamPut(aoacp, 'A') != MSG_OK) {
                _usbh_dbg(host, "AOA: Disconnected");
                goto start;
            }
            chThdSleepMilliseconds(100);
        }
    }

    usbhaoaChannelStop(aoap);

    _usbh_dbg(host, "AOA: Tests done, restarting in 3s");
    chThdSleepMilliseconds(3000);

    goto start;
}
#endif

#if HAL_USBH_USE_MSD
#include "usbh/dev/msd.h"
#include "ff.h"
#include "fatfs_devices.h"

static FATFS MSDLUN0FS;

#if !UVC_TO_MSD_PHOTOS_CAPTURE
static uint8_t fbuff[10240];
static FIL file;

static FRESULT scan_files(USBHDriver *host, BaseSequentialStream *chp, char *path) {
  FRESULT res;
  DIR dir;
  UINT i;
  static FILINFO fno;

  res = f_opendir(&dir, path);
  if (res == FR_OK) {
    for (;;) {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0)
        break;
      if (fno.fattrib & AM_DIR) {
		i = strlen(path);
        path[i++] = '/';
        strcpy(&path[i], fno.fname);
        res = scan_files(host, chp, path);
        if (res != FR_OK)
          break;
        path[--i] = 0;
      } else {
          _usbh_dbgf(host, "FS: %s/%s", path, fno.fname);
      }
    }
  }
  return res;
}
#endif

static THD_WORKING_AREA(waTestMSD, 1300);
static void ThreadTestMSD(void *p) {
    (void)p;

    FATFS *fsp;
    DWORD clusters;
    FRESULT res;
    USBHDriver *host = NULL;

    chRegSetThreadName("MSD");

#if !UVC_TO_MSD_PHOTOS_CAPTURE
    BaseSequentialStream * const chp = (BaseSequentialStream *)&SD2;
    systime_t st, et;
    uint32_t j;
#endif

start:
    for(;;) {
        chThdSleepMilliseconds(100);

        if (blkGetDriverState(&MSBLKD[0]) == BLK_ACTIVE) {
        	host = usbhmsdLUNGetHost(&MSBLKD[0]);
        	_usbh_dbg(host, "BLK: Active, connect....");
        	usbhmsdLUNConnect(&MSBLKD[0]);
        }
        if (blkGetDriverState(&MSBLKD[0]) != BLK_READY) {
            continue;
    	}

    	_usbh_dbg(host, "BLK: Ready.");

#if !UVC_TO_MSD_PHOTOS_CAPTURE
        //raw read test
        if (1) {
#define RAW_READ_SZ_MB        1
#define NBLOCKS                (sizeof(fbuff) / 512)
#define NITERATIONS            ((RAW_READ_SZ_MB * 1024UL * 1024UL) / sizeof(fbuff))
            uint32_t start = 0;
            chThdSetPriority(HIGHPRIO);
            _usbh_dbgf(host, "BLK: Raw read test (%dMB, %dB blocks)", RAW_READ_SZ_MB, sizeof(fbuff));
            st = chVTGetSystemTime();
            for (j = 0; j < NITERATIONS; j++) {
                if (blkRead(&MSBLKD[0], start, fbuff, NBLOCKS) != HAL_SUCCESS)
                    goto start;
                start += NBLOCKS;
            }
            et = chVTGetSystemTime();
            _usbh_dbgf(host, "BLK: Raw read in %d ms, %dkB/s",
                    et - st,
                    (RAW_READ_SZ_MB * 1024UL * 1000) / (et - st));
            chThdSetPriority(NORMALPRIO);
        }
#endif

        _usbh_dbg(host, "FS: Block driver ready, try mount...");

        res = f_mount(&MSDLUN0FS, FATFSDEV_MSD_DRIVE, 1);
        if (res != FR_OK) {
            _usbh_dbg(host, "FS: Can't mount. Check file system.");
            continue;
        }
        _usbh_dbg(host, "FS: Mounted.");

        res = f_getfree(FATFSDEV_MSD_DRIVE, &clusters, &fsp);
        if (res != FR_OK) {
            _usbh_dbg(host, "FS: f_getfree() failed");
            continue;
        }

        _usbh_dbgf(host, "FS: %lu free clusters, %lu sectors per cluster, %lu bytes free",
                clusters, (uint32_t)MSDLUN0FS.csize,
                clusters * (uint32_t)MSDLUN0FS.csize * MSBLKD[0].info.blk_size);

        break;
    }

#if !UVC_TO_MSD_PHOTOS_CAPTURE
    //FATFS test
    if (1) {
        UINT bw;
        const uint8_t *src;
        const uint8_t *const start = (uint8_t *)0x08000000;
        const uint8_t *const top = (uint8_t *)0x08020000;

        //write test
        if (1) {
            _usbh_dbg(host, "FS: Write test (create file /test.dat, 1MB)");
            f_open(&file, FATFSDEV_MSD_DRIVE "/test.dat", FA_CREATE_ALWAYS | FA_WRITE);
            src = start;
            st = chVTGetSystemTime();
            for (j = 0; j < 2048; j++) {
                if (f_write(&file, src, 512, &bw) != FR_OK)
                    goto start;
                src += bw;
                if (src >= top)
                    src = start;
            }
            et = chVTGetSystemTime();
            _usbh_dbgf(host, "FS: Written 1MB in %d ms, %dkB/s",
                    et - st,
                    (1024UL*1000) / (et - st));
            f_close(&file);
        }

        //read test
        if (1) {
            _usbh_dbg(host, "FS: Read test (read file /test.dat, 1MB, compare)");
            f_open(&file, FATFSDEV_MSD_DRIVE "/test.dat", FA_READ);
            src = start;
            st = chVTGetSystemTime();
            for (j = 0; j < 2048; j++) {
                if (f_read(&file, fbuff, 512, &bw) != FR_OK)
                    goto start;
                if (memcmp(src, fbuff, bw)) {
                    _usbh_dbgf(host, "Compare error @%08x", (uint32_t)src);
                    goto start;
                }
                src += bw;
                if (src >= top)
                    src = start;
            }
            et = chVTGetSystemTime();
            _usbh_dbgf(host, "FS: Read 1MB in %d ms, %dkB/s",
                    et - st,
                    (1024UL*1000) / (et - st));
            f_close(&file);
        }

        //scan files test
        if (1) {
            _usbh_dbg(host, "FS: Scan files test");
            strcpy((char *)fbuff, FATFSDEV_MSD_DRIVE);
            scan_files(host, chp, (char *)fbuff);
        }
    }
#endif

    _usbh_dbg(host, "FS: Tests done, restarting in 3s");
    chThdSleepMilliseconds(3000);

    goto start;

}
#endif

#if HAL_USBH_USE_HID
#include "usbh/dev/hid.h"
#include "chprintf.h"

static THD_WORKING_AREA(waTestHID, 1024);

static void _hid_report_callback(USBHHIDDriver *hidp, uint16_t len) {
    uint8_t *report = (uint8_t *)hidp->config->report_buffer;

    if (hidp->type == USBHHID_DEVTYPE_BOOT_MOUSE) {
        _usbh_dbgf(hidp->dev->host, "Mouse report: buttons=%02x, Dx=%d, Dy=%d",
                report[0],
                (int8_t)report[1],
                (int8_t)report[2]);
    } else if (hidp->type == USBHHID_DEVTYPE_BOOT_KEYBOARD) {
        _usbh_dbgf(hidp->dev->host, "Keyboard report: modifier=%02x, keys=%02x %02x %02x %02x %02x %02x",
                report[0],
                report[2],
                report[3],
                report[4],
                report[5],
                report[6],
                report[7]);
    } else {
        _usbh_dbgf(hidp->dev->host, "Generic report, %d bytes", len);
    }
}

static USBH_DEFINE_BUFFER(uint8_t report[HAL_USBHHID_MAX_INSTANCES][8]);
static USBHHIDConfig hidcfg[HAL_USBHHID_MAX_INSTANCES];

static void ThreadTestHID(void *p) {
    (void)p;
    uint8_t i;
    static uint8_t kbd_led_states[HAL_USBHHID_MAX_INSTANCES];

    chRegSetThreadName("HID");

    for (i = 0; i < HAL_USBHHID_MAX_INSTANCES; i++) {
        hidcfg[i].cb_report = _hid_report_callback;
        hidcfg[i].protocol = USBHHID_PROTOCOL_BOOT;
        hidcfg[i].report_buffer = report[i];
        hidcfg[i].report_len = 8;
    }

    for (;;) {
        for (i = 0; i < HAL_USBHHID_MAX_INSTANCES; i++) {
        	USBHHIDDriver *const hidp = &USBHHIDD[i];
            if (usbhhidGetState(hidp) == USBHHID_STATE_ACTIVE) {
                _usbh_dbgf(hidp->dev->host, "HID: Connected, HID%d", i);
                usbhhidStart(hidp, &hidcfg[i]);
                if (usbhhidGetType(hidp) != USBHHID_DEVTYPE_GENERIC) {
                    usbhhidSetIdle(hidp, 0, 0);
                }
                kbd_led_states[i] = 1;
            } else if (usbhhidGetState(hidp) == USBHHID_STATE_READY) {
                if (usbhhidGetType(hidp) == USBHHID_DEVTYPE_BOOT_KEYBOARD) {
                    USBH_DEFINE_BUFFER(uint8_t val);
                    val = kbd_led_states[i] << 1;
                    if (val == 0x08) {
                        val = 1;
                    }
                    usbhhidSetReport(hidp, 0, USBHHID_REPORTTYPE_OUTPUT, &val, 1);
                    kbd_led_states[i] = val;
                }
            }
        }
        chThdSleepMilliseconds(200);
    }

}
#endif

#if HAL_USBH_USE_UVC
#include "usbh/dev/uvc.h"

static THD_WORKING_AREA(waTestUVC, 1024);

#if UVC_TO_MSD_PHOTOS_CAPTURE
static const uint8_t jpeg_header_plus_dht[] = {
    0xff, 0xd8,                     // SOI
    0xff, 0xe0,                     // APP0
    0x00, 0x10,                     // APP0 header size (including this field, but excluding preceding)
    0x4a, 0x46, 0x49, 0x46, 0x00,   // ID string 'JFIF\0'
    0x01, 0x01,                     // version
    0x00,                           // bits per type
    0x00, 0x00,                     // X density
    0x00, 0x00,                     // Y density
    0x00,                           // X thumbnail size
    0x00,                           // Y thumbnail size
    0xFF, 0xC4, 0x01, 0xA2, 0x00,
    0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 ,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
    0x0a, 0x0b, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    0x10,
    0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d,
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
    0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
    0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
    0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
    0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
    0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa,
    0x11,
    0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77,
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
    0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
    0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
    0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
    0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
    0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
    0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
    0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa
};
#endif

static void ThreadTestUVC(void *p) {
    (void)p;
    USBHUVCDriver *const uvcdp = &USBHUVCD[0];

    chRegSetThreadName("UVC");

    for(;;) {
        chThdSleepMilliseconds(100);

        //chSysLock();
        //state = usbhuvcGetDriverState(&USBHUVCD[0]);
        //chSysUnlock();
        if (usbhuvcGetState(&USBHUVCD[0]) != USBHUVC_STATE_ACTIVE)
            continue;

        _usbh_dbg(uvcdp->dev->host, "UVC: Webcam connected");

        /* ************************************ */
        /*   Find best configuration            */
        /* ************************************ */
        _usbh_dbg(uvcdp->dev->host, "UVC: Find best configuration");

        generic_iterator_t ics;
        const usbh_uvc_format_mjpeg_t *format;

        uint32_t max_frame_sz = 0;
        uint16_t min_ep_sz;
        uint8_t best_frame_interval_index;
        const usbh_uvc_frame_mjpeg_t *best_frame = NULL;


        //find format MJPEG
        if (usbhuvcFindVSDescriptor(uvcdp, &ics, UVC_VS_FORMAT_MJPEG, TRUE) != HAL_SUCCESS)
            goto failed;

        format = (const usbh_uvc_format_mjpeg_t *)ics.curr;
        _usbh_dbgf(uvcdp->dev->host, "\tSelect bFormatIndex=%d", format->bFormatIndex);

        //find the most suitable frame (largest one within the bandwidth requirements)
        if (usbhuvcFindVSDescriptor(uvcdp, &ics, UVC_VS_FRAME_MJPEG, TRUE) != HAL_SUCCESS)
            goto failed;

        do {
            const usbh_uvc_frame_mjpeg_t *const frame = (usbh_uvc_frame_mjpeg_t *)ics.curr;
            uint32_t frame_sz = frame->wWidth * frame->wHeight;

            _usbh_dbgf(uvcdp->dev->host, "\t\tbFrameIndex=%d", frame->bFrameIndex);
            _usbh_dbgf(uvcdp->dev->host, "\t\t\twWidth=%d, wHeight=%d", frame->wWidth, frame->wHeight);
            _usbh_dbgf(uvcdp->dev->host, "\t\t\tdwMinBitRate=%u, dwMaxBitRate=%u", frame->dwMinBitRate, frame->dwMaxBitRate);
            _usbh_dbgf(uvcdp->dev->host, "\t\t\tdwMaxVideoFrameBufferSize=%u", frame->dwMaxVideoFrameBufferSize);
            _usbh_dbgf(uvcdp->dev->host, "\t\t\tdwDefaultFrameInterval=%u", frame->dwDefaultFrameInterval);

            uint8_t j;
            for (j = 0; j < frame->bFrameIntervalType; j++) {
                uint32_t ep_sz =
                        usbhuvcEstimateRequiredEPSize(uvcdp, (const uint8_t *)format, (const uint8_t *)frame, frame->dwFrameInterval[j]);

                _usbh_dbgf(uvcdp->dev->host, "\t\t\tdwFrameInterval=%u, estimated EP size=%u", frame->dwFrameInterval[j], ep_sz);

                if (ep_sz > 310)
                    continue;

                /* candidate found */

                if (frame_sz >= max_frame_sz) {
                    /* new best frame size */
                    min_ep_sz = 0xffff;
                    max_frame_sz = frame_sz;
                } else {
                    continue;
                }

                if (ep_sz < min_ep_sz) {
                    /* new best bitrate */
                    min_ep_sz = ep_sz;
                    _usbh_dbg(uvcdp->dev->host, "\t\t\tNew best candidate found");
                    best_frame_interval_index = j;
                    best_frame = frame;
                }
            }
        } while (usbhuvcFindVSDescriptor(uvcdp, &ics, UVC_VS_FRAME_MJPEG, FALSE) == HAL_SUCCESS);

failed:
        if (best_frame == NULL) {
            _usbh_dbg(uvcdp->dev->host, "\t\t\tCouldn't find suitable format/frame");
            continue;
        }

        /* ************************************ */
        /*   NEGOTIATION                        */
        /* ************************************ */
        _usbh_dbg(uvcdp->dev->host, "UVC: Start negotiation");

        usbhuvcResetPC(uvcdp);
        usbh_uvc_ctrl_vs_probecommit_data_t *const pc = usbhuvcGetPC(uvcdp);

        pc->bmHint = 0x0001;
        pc->bFormatIndex = format->bFormatIndex;
        pc->bFrameIndex = best_frame->bFrameIndex;
        pc->dwFrameInterval = best_frame->dwFrameInterval[best_frame_interval_index];

        _usbh_dbgf(uvcdp->dev->host, "\tFirst probe, selecting bFormatIndex=%d, bFrameIndex=%d, dwFrameInterval=%u",
                pc->bFormatIndex, pc->bFrameIndex, pc->dwFrameInterval);

        _usbh_dbg(uvcdp->dev->host, "SET_CUR (PROBE):"); usbhuvcPrintProbeCommit(uvcdp, &uvcdp->pc);
        if (usbhuvcProbe(uvcdp) != HAL_SUCCESS) {
            _usbh_dbg(uvcdp->dev->host, "\tFirst probe failed");
            continue;
        }
        _usbh_dbg(uvcdp->dev->host, "GET_CUR (PROBE):"); usbhuvcPrintProbeCommit(uvcdp, &uvcdp->pc);
        _usbh_dbg(uvcdp->dev->host, "GET_MIN (PROBE):"); usbhuvcPrintProbeCommit(uvcdp, &uvcdp->pc_min);
        _usbh_dbg(uvcdp->dev->host, "GET_MAX (PROBE):"); usbhuvcPrintProbeCommit(uvcdp, &uvcdp->pc_max);

        pc->bmHint = 0x0001;
        pc->wCompQuality = uvcdp->pc_min.wCompQuality;

        _usbh_dbg(uvcdp->dev->host, "SET_CUR (PROBE):"); usbhuvcPrintProbeCommit(uvcdp, &uvcdp->pc);
        _usbh_dbgf(uvcdp->dev->host, "\tSecond probe, selecting wCompQuality=%d", pc->wCompQuality);
        if (usbhuvcProbe(uvcdp) != HAL_SUCCESS) {
            _usbh_dbg(uvcdp->dev->host, "\tSecond probe failed");
            continue;
        }
        _usbh_dbg(uvcdp->dev->host, "GET_CUR (PROBE):"); usbhuvcPrintProbeCommit(uvcdp, &uvcdp->pc);
        _usbh_dbg(uvcdp->dev->host, "GET_MIN (PROBE):"); usbhuvcPrintProbeCommit(uvcdp, &uvcdp->pc_min);
        _usbh_dbg(uvcdp->dev->host, "GET_MAX (PROBE):"); usbhuvcPrintProbeCommit(uvcdp, &uvcdp->pc_max);

        /* ************************************ */
        /*   Commit negotiated parameters        */
        /* ************************************ */
        _usbh_dbg(uvcdp->dev->host, "UVC: Commit negotiated parameters");
        _usbh_dbg(uvcdp->dev->host, "SET_CUR (COMMIT):"); usbhuvcPrintProbeCommit(uvcdp, &uvcdp->pc);
        if (usbhuvcCommit(uvcdp) != HAL_SUCCESS) {
            _usbh_dbg(uvcdp->dev->host, "\tCommit failed");
            continue;
        }

        _usbh_dbg(uvcdp->dev->host, "UVC: Ready to start streaming");

        uint32_t npackets = 0;
        uint32_t payload = 0;
        uint32_t total = 0;
        uint32_t frame = 0;
        systime_t last = 0;
        usbhuvcStreamStart(uvcdp, 310);

        uint8_t state = 0;
        static FIL fp;

        for (;;) {
            msg_t msg, ret;
            ret = usbhuvcLockAndFetch(uvcdp, &msg, TIME_INFINITE);
            if (ret == MSG_RESET) {
                _usbh_dbg(uvcdp->dev->host, "UVC: Driver is unloading");
                break;
            } else if (ret == MSG_TIMEOUT) {
                continue;
            }

            if (((usbhuvc_message_base_t *)msg)->type == USBHUVC_MESSAGETYPE_DATA) {
                usbhuvc_message_data_t *const data = (usbhuvc_message_data_t *)msg;

                if (data->length < data->data[0]) {
                    _usbh_dbgf(uvcdp->dev->host, "UVC: Length error!");
                    goto free_data;
                }

                uint32_t message_payload = data->length - data->data[0];

                total += data->length;
                payload += message_payload;
                npackets++;

#if UVC_TO_MSD_PHOTOS_CAPTURE
                char fn[20];
                UINT bw;
                bool with_dht = true;
                uint8_t *message_data = data->data + data->data[0];
                if (frame & 7) goto check_eof;

                if (state == 1) {
                    if (message_payload < 12) goto check_eof;
                    if (strncmp("AVI1", (const char *)message_data + 6, 4)) {
                        with_dht = false;
                    } else {
                        uint16_t skip = (message_data[4] << 8) + message_data[5] + 4;
                        if (skip > message_payload) goto check_eof;
                        message_data += skip;
                        message_payload -= skip;
                    }

                    chsnprintf(fn, sizeof(fn), "/img%d.jpg", frame);
                    if (f_open(&fp, fn, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
                        if (with_dht && f_write(&fp, jpeg_header_plus_dht, sizeof(jpeg_header_plus_dht), &bw) != FR_OK) {
                            _usbh_dbg(uvcdp->dev->host, "UVC->MSD: File write error");
                            f_close(&fp);
                            state = 0;
                        }
                        state = 2;
                    } else {
                        _usbh_dbg(uvcdp->dev->host, "UVC->MSD: File open error");
                        state = 0;
                    }
                }

                if (state == 2) {
                    if (f_write(&fp, message_data, message_payload, &bw) != FR_OK) {
                        _usbh_dbg(uvcdp->dev->host, "UVC->MSD: File write error");
                        f_close(&fp);
                        state = 0;
                    }
                }

check_eof:
#endif
                if (data->data[1] & UVC_HDR_EOF) {
                    _usbh_dbgf(uvcdp->dev->host, "UVC: FRAME #%d, delta=%03dticks, #packets=%d, useful_payload=%dbytes, total=%dbytes",
                            frame, data->timestamp - last , npackets, payload, total);
                    last = data->timestamp;
                    npackets = 0;
                    payload = 0;
                    total = 0;
                    frame++;
                    if (state == 2) {
                        f_close(&fp);
                    }
                    state = 1;
                }
free_data:
                usbhuvcFreeDataMessage(uvcdp, data);
            } else {
                usbhuvc_message_status_t *const status = (usbhuvc_message_status_t *)msg;
                const uint8_t *const stat = status->data;
                switch (stat[0] & 0x0f) {
                case 1:
                    _usbh_dbgf(uvcdp->dev->host, "UVC: STATUS Control event, "
                            "bOriginator=%d, bEvent=%d, bSelector=%d, bAttribute=%d",
                            stat[1], stat[2], stat[3], stat[4]);
                    break;
                case 2:
                    _usbh_dbgf(uvcdp->dev->host, "UVC: STATUS Streaming event, "
                            "bOriginator=%d, bEvent=%d, bValue=%d",
                            stat[1], stat[2], stat[3]);
                    break;
                default:
                    _usbh_dbgf(uvcdp->dev->host, "UVC: STATUS unknown status report = %d", stat[0]);
                    break;
                }
                usbhuvcFreeStatusMessage(uvcdp, status);
            }
            usbhuvcUnlock(uvcdp);
        }

    }

}
#endif

#if USBH_DEBUG_MULTI_HOST
void USBH_DEBUG_OUTPUT_CALLBACK(USBHDriver *host, const uint8_t *buff, size_t len) {
	(void)host;
#else
void USBH_DEBUG_OUTPUT_CALLBACK(const uint8_t *buff, size_t len) {
#endif
	sdWrite(&SD2, buff, len);
	sdWrite(&SD2, (const uint8_t *)"\r\n", 2);
}

int main(void) {

    IWDG->KR = 0x5555;
    IWDG->PR = 7;

    halInit();
    chSysInit();

    //PA2(TX) and PA3(RX) are routed to USART2
    sdStart(&SD2, NULL);
    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

#if STM32_USBH_USE_OTG1
    //VBUS - configured in board.h
    //USB_FS - configured in board.h
#endif

#if STM32_USBH_USE_OTG2
#error "TODO: Initialize USB_HS pads"
#endif

#if HAL_USBH_USE_MSD
    chThdCreateStatic(waTestMSD, sizeof(waTestMSD), NORMALPRIO, ThreadTestMSD, 0);
#endif

#if HAL_USBH_USE_FTDI
    chThdCreateStatic(waTestFTDI, sizeof(waTestFTDI), NORMALPRIO, ThreadTestFTDI, 0);
#endif

#if HAL_USBH_USE_AOA
    chThdCreateStatic(waTestAOA, sizeof(waTestAOA), NORMALPRIO, ThreadTestAOA, 0);
#endif

#if HAL_USBH_USE_HID
    chThdCreateStatic(waTestHID, sizeof(waTestHID), NORMALPRIO, ThreadTestHID, 0);
#endif

#if HAL_USBH_USE_UVC
    chThdCreateStatic(waTestUVC, sizeof(waTestUVC), NORMALPRIO + 1, ThreadTestUVC, 0);
#endif

    //turn on USB power
    palClearPad(GPIOC, GPIOC_OTG_FS_POWER_ON);
    chThdSleepMilliseconds(100);

    //start
#if STM32_USBH_USE_OTG1
    usbhStart(&USBHD1);
    _usbh_dbgf(&USBHD1, "Started");
#endif
#if STM32_USBH_USE_OTG2
    usbhStart(&USBHD2);
    _usbh_dbgf(&USBHD2, "Started");
#endif

    for(;;) {
#if STM32_USBH_USE_OTG1
        usbhMainLoop(&USBHD1);
#endif
#if STM32_USBH_USE_OTG2
        usbhMainLoop(&USBHD2);
#endif
        chThdSleepMilliseconds(100);

        IWDG->KR = 0xAAAA;
    }
}
