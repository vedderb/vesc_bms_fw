/*
  Copyright (C) 2020 Ein Terakawa

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
#include "ssd1306.h"
#include "string.h"

/* timeout value must be increased for i2cclk less than its default, 100kHz. */
#define CMD_TRANSMIT_TIMEOUT TIME_MS2I(10)
#define DATA_TRANSMIT_TIMEOUT TIME_MS2I(100)

#define CTRL_CMD_STREAM 0x00
#define CTRL_DATA_STREAM 0x40
#define CTRL_CMD_SINGLE 0x80

#define I2CD ((ssd1306)->i2cd)
#define ADDR ((ssd1306)->i2caddr)

#define send_cmd(ssd1306, ...) \
  do { \
    uint8_t buf[] = { CTRL_CMD_STREAM, __VA_ARGS__ }; \
    bool _success = _send_cmd(I2CD, ADDR, buf, sizeof(buf)); \
    if (!_success) goto done; \
  } while(0)

#define send_cmd_static(ssd1306, ...) \
  do{ \
    static const uint8_t buf[] = { CTRL_CMD_STREAM, __VA_ARGS__ }; \
    bool _success = _send_cmd(I2CD, ADDR, buf, sizeof(buf)); \
    if (!_success) goto done; \
  } while(0)

#define CMD1(c) c
#define CMD2(c, d) c, d
#define CMD3(c, d, e) c, d, e

static bool _send_cmd(I2CDriver *i2cd, uint8_t addr, const uint8_t *buf, int len) {

    msg_t status = i2cMasterTransmitTimeout(i2cd, addr,
      buf, len, NULL, 0, CMD_TRANSMIT_TIMEOUT);

    if (MSG_OK != status) {
      /* i2cflags_t error_code = i2cGetErrors(i2cd); */
      return false;
    }

    return true;
}

#define send_data(ssd1306, buf, len) \
  do { \
    bool _success = _send_data(I2CD, ADDR, buf, len); \
    if (!_success) goto done; \
  } while(0)

static bool _send_data(I2CDriver *i2cd, uint8_t addr, const uint8_t *buf, int len) {
    bool success = false;

    msg_t status = i2cMasterTransmitTimeout(i2cd, addr,
      buf, len, NULL, 0, DATA_TRANSMIT_TIMEOUT);

    if (MSG_OK != status) {
      /* i2cflags_t error_code = i2cGetErrors(I2CD); */
      goto done;
    }

    success = true;

done:
    return success;
}

bool ssd1306_init(SSD1306_DRIVER *ssd1306) {
    bool success = false;
    uint8_t width = ssd1306->width;
    uint8_t height = ssd1306->height;
    uint8_t num_pages = height / 8;

    i2cStart(I2CD, ssd1306->i2ccfg);

    send_cmd_static(ssd1306,
      CMD1(SetDisplayOff),
      CMD1(DeactivateScroll),
      CMD1(DisableEntireDisplayOn),
      CMD2(SetOscFreqAndClkDiv, 0x80),
      CMD2(SetDisplayOffset, 0x00),
      CMD1(SetDisplayStartLine | 0x00),
      CMD2(SetMemoryAddressMode, 0x00)); /* Horizontal Addressing Mode */

    send_cmd(ssd1306, CMD2(SetMultiplexRatio, height - 1));

    if (ssd1306->rotate180) {
      /* rotate 180 degrees == upside down */
      send_cmd_static(ssd1306,
        CMD1(SetSegmentRemapReverse),
        CMD1(SetComScanOrderReverse));
    } else {
      /* no rotation */
      send_cmd_static(ssd1306,
        CMD1(SetSegmentRemapNormal),
        CMD1(SetComScanOrderNormal));
    }

    if (height == 32) {
      /* 128x32 module uses SequentialComMode */
      send_cmd_static(ssd1306, CMD2(SetComPins, 0x02));
    } else {
      /* 128x64 module uses AlternativeComMode */
      send_cmd_static(ssd1306, CMD2(SetComPins, 0x12));
    }

    /* Clear Graphic Display Data RAM */
    send_cmd(ssd1306,
      CMD3(SetPageAddress, 0, num_pages - 1),
      CMD3(SetColumnAddress, 0, width - 1));

    uint8_t *buf = ssd1306->buf;
    size_t len = SSD1306_PREAMBLE_LENGTH + num_pages * width;
    memset(buf, 0, len);
    buf[0] = CTRL_DATA_STREAM; /* need this byte proceeding the actual data */
    send_data(ssd1306, buf, len);

    send_cmd_static(ssd1306,
      CMD2(SetPreChargePeriod, 0xC4),
      CMD2(SetVcomhLevel, 0x20),
      CMD1(SetNormalDisplay),
      CMD2(SetContrastControl, 0x3F),
      CMD2(ChargePumpSetting, 0x14),
      CMD1(SetDisplayOn));

    success = true;

done:
    i2cStop(I2CD);
    return success;
}

bool ssd1306_data(SSD1306_DRIVER *ssd1306) {
    bool success = false;
    uint8_t width = ssd1306->width;
    uint8_t height = ssd1306->height;
    uint8_t num_pages = height / 8;

    i2cStart(I2CD, ssd1306->i2ccfg);

    /* Transfer to Graphic Display Data RAM */
    send_cmd(ssd1306,
      CMD3(SetPageAddress, 0, num_pages - 1),
      CMD3(SetColumnAddress, 0, width - 1));

    uint8_t *buf = ssd1306->buf;
    size_t len = SSD1306_PREAMBLE_LENGTH + num_pages * width;
    send_data(ssd1306, buf, len);

    success = true;

done:
    i2cStop(I2CD);
    return success;
}

bool ssd1306_display_on(SSD1306_DRIVER *ssd1306) {
    bool success = false;
    i2cStart(I2CD, ssd1306->i2ccfg);
    send_cmd_static(ssd1306, CMD1(SetDisplayOn));
    success = true;
done:
    i2cStop(I2CD);
    return success;
}

bool ssd1306_display_off(SSD1306_DRIVER *ssd1306) {
    bool success = false;
    i2cStart(I2CD, ssd1306->i2ccfg);
    send_cmd_static(ssd1306, CMD1(SetDisplayOff));
    success = true;
done:
    i2cStop(I2CD);
    return success;
}
