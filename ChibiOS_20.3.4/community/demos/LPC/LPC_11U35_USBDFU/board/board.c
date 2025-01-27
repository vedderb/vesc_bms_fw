#include "board.h"
#include "hal.h"

#define MATRIX_ROWS 8
#define MATRIX_COLS 9

void boardInit(void) {
  palSetLineMode(LINE_USBVBUS, MODE_FUNC_ALT1 | MODE_MODE_PULL_UP | MODE_AD_DIGITAL);
  palSetLineMode(LINE_USBCONN, MODE_FUNC_ALT1 | MODE_AD_DIGITAL);
}
