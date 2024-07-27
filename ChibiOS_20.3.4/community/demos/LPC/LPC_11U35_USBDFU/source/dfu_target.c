#include "dfu_target.h"
#include "usbdfu.h"
#include "ch.h"
#include "hal.h"
#include <string.h>

#define IAP_LOCATION  0x1fff1ff1
typedef void (*IAP)(uint32_t [], uint32_t []);
const IAP iap_entry = (IAP)IAP_LOCATION;

#define INTERNAL_CHUNK_SIZE 1024

static uint8_t iap_buffer[INTERNAL_CHUNK_SIZE];
static size_t buffer_size = 0;
static size_t buffer_base = 0;
static size_t last_chunk_size = 0;
static size_t preped_flash = 0;

size_t target_get_max_fw_size(void) {
	return (0x10000 - 0x2000);
}

uint16_t target_get_timeout(void) {
  if (last_chunk_size != 0 && (buffer_size + last_chunk_size) < INTERNAL_CHUNK_SIZE) {
    return 5;
  }
  if (preped_flash == 0) {
    return 50;
  }
	return 0;
}

void target_flash_unlock(void) {
	chSysLock();
}

static inline bool write_buffer(void) {
  uint32_t iap_command[5] = {0,0,0,0,0};
  uint32_t iap_result[4] = {0,0,0,0};

  if (((size_t)buffer_base % INTERNAL_CHUNK_SIZE) != 0) {
    return false;
  }

  uint32_t sector = (size_t)buffer_base / 4096;
  iap_command[0] = 50; // Prep Sector
  iap_command[1] = sector; // Start Sec
  iap_command[2] = sector; // Stop Sec
  iap_entry(iap_command, iap_result);

  // Copy the buffer
  iap_command[0] = 51; // Copy RAM -> Flash
  iap_command[1] = (size_t) buffer_base;
  iap_command[2] = (size_t) iap_buffer;
  iap_command[3] = INTERNAL_CHUNK_SIZE;
  iap_command[4] = 48000;
  iap_entry(iap_command, iap_result);
  return iap_result[0] == 0;
}

bool target_flash_write(uint8_t* dst, uint8_t* src, size_t len) {
  last_chunk_size = len;
  size_t copied = 0;
  while(copied < len) { // Stuff remain in the thing
    if (buffer_size == 0) { // If buffer is empty, then we update base
      buffer_base = (size_t)dst + copied;
    }

    size_t copy_size = len - copied;
    if (copy_size > (INTERNAL_CHUNK_SIZE - buffer_size)) {
      copy_size = INTERNAL_CHUNK_SIZE - buffer_size;
    }
    memcpy(&iap_buffer[buffer_size], (src + copied), copy_size);
    buffer_size += copy_size;
    copied += copy_size;
    if (buffer_size == INTERNAL_CHUNK_SIZE) {
      if (!write_buffer())
        return false;
      buffer_size = 0;
    }
  }
  return true;
}

bool target_prepare_flash(void) {
  uint32_t iap_command[5] = {0,0,0,0,0};
  uint32_t iap_result[4] = {0,0,0,0};
  iap_command[0] = 50; // Prep Sector
  iap_command[1] = APP_BASE / 0x1000; // Start Sec
  iap_command[2] = 15; // Stop Sec
  iap_entry(iap_command, iap_result);
  if (iap_result[0] != 0) {
  	return false;
  }

  iap_command[0] = 52; // Erase Sector
  iap_command[1] = APP_BASE / 0x1000; // Start Sec
  iap_command[2] = 15; // Stop Sec
  iap_command[3] = 48000; // 48MHz
  iap_entry(iap_command, iap_result);
  preped_flash = 1;

  return iap_result[0] == 0;
}

void target_flash_lock(void) {
	chSysUnlock();
}

void target_complete_programming(void) {
  if (buffer_size > 0) {
    memset(&iap_buffer[buffer_size], 0, INTERNAL_CHUNK_SIZE - buffer_size);
    write_buffer();
  }
}
