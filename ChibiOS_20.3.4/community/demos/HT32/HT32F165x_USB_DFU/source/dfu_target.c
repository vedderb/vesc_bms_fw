#include "usbdfu.h"
#include "ch.h"
#include "hal.h"
#include <string.h>

size_t target_get_max_fw_size(void) {
	return (MAX_FLASH_ADDR - APP_BASE);
}

uint16_t target_get_timeout(void) {
  return 5;
}

void target_flash_unlock(void) {
	chSysLock();
}

bool target_flash_write(uint8_t* dst, uint8_t* src, size_t len) {
  // TODO: Write flash
  return true;
}

bool target_prepare_flash(void) {
  // TODO: Erase All Flash Pages
  return true;
}

void target_flash_lock(void) {
	chSysUnlock();
}

void target_complete_programming(void) {
  // Do nothing
}
