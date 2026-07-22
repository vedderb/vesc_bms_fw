#ifndef SSD1306_H
#define SSD1306_H

#include <chtypes.h>

enum ssd1306_cmds {

  /* Fundamental Command */
  SetContrastControl = 0x81,
  DisableEntireDisplayOn = 0xA4,
  EnableEntireDisplayOn = 0xA5,
  SetNormalDisplay = 0xA6,
  SetInvertDisplay = 0xA7,
  SetDisplayOff = 0xAE,
  SetDisplayOn = 0xAF,

  /* Charge Pump Command */
  ChargePumpSetting = 0x8D,

  /* Timing & Driving Scheme Setting Command */
  SetOscFreqAndClkDiv = 0xD5,
  SetPreChargePeriod = 0xD9,
  SetVcomhLevel = 0xDB,

  /* Addressing Setting Command */
  SetMemoryAddressMode = 0x20,
  SetColumnAddress = 0x21,
  SetPageAddress = 0x22,
  // SetLowColumn = 0x00, /* 0x00 - 0x0F */
  // SetHighColumn = 0x10, /* 0x10 - 0x1F */
  // SetPageStartAddress = 0xB0, /* 0xB0 - 0xB7 */

  /* Hardware Configuration Command */
  SetDisplayStartLine = 0x40, /* 0x40 - 0x7F */
  SetSegmentRemapNormal = 0xA0,
  SetSegmentRemapReverse = 0xA1,
  SetMultiplexRatio = 0xA8,
  SetComScanOrderNormal = 0xC0,
  SetComScanOrderReverse = 0xC8,
  SetDisplayOffset = 0xD3,
  SetComPins = 0xDA,

  /* Scrolling Command */
  RightHorizontalScroll = 0x26,
  LeftHorizontalScroll = 0x27,
  VerticalAndRightHorizontalScroll = 0x29,
  VerticalAndLeftHorizontalScroll = 0x2A,
  DeactivateScroll = 0x2E,
  ActivateScroll = 0x2F,
  SetVerticalScrollArea = 0xA3,

  /* Other Command */
  NoOperation = 0xE3,
};

#ifndef SSD1306_ADDRESS
/* for your reference (0x3C << 1) == 0x78 , (0x3D << 1) == 0x7A . */
#define SSD1306_ADDRESS 0x3C
#endif

typedef struct I2CDriver I2CDriver;
typedef struct I2CConfig I2CConfig;
typedef struct SSD1306_DRIVER {
  I2CDriver *i2cd;
  I2CConfig const *i2ccfg;
  uint8_t i2caddr;
  uint8_t width;
  uint8_t height;
  bool rotate180;
  uint8_t *buf;
} SSD1306_DRIVER;

#define SSD1306_PREAMBLE_LENGTH 1
#define DEFINE_SSD1306_DRIVER(name, i2cd, addr, width, height, rotate180) \
  _Static_assert(width == 128, "Only support width of 128 for simplicity."); \
  _Static_assert(height == 32 || height == 64, "Only support height of 32 or 64 for simplicity."); \
  uint8_t name##_buf[SSD1306_PREAMBLE_LENGTH + width * height / 8]; \
  struct SSD1306_DRIVER name = { \
    i2cd, NULL, addr, width, height, rotate180, name##_buf \
  }

#define SSD1306_GET_FRAMEBUFFER(ssd1306) \
  (&((ssd1306)->buf[SSD1306_PREAMBLE_LENGTH]))

bool ssd1306_init(SSD1306_DRIVER *ssd1306);
bool ssd1306_data(SSD1306_DRIVER *ssd1306);
bool ssd1306_display_on(SSD1306_DRIVER *ssd1306);
bool ssd1306_display_off(SSD1306_DRIVER *ssd1306);

#endif
