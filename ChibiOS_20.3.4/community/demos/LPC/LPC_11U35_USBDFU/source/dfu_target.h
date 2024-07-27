#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

size_t target_get_max_fw_size(void);
uint16_t target_get_timeout(void);
void target_flash_unlock(void);
bool target_flash_write(uint8_t* dst, uint8_t* src, size_t len);
bool target_prepare_flash(void);
void target_flash_lock(void);
void target_complete_programming(void);