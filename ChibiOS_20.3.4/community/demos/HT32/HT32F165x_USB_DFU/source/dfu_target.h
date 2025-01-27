#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define MAX_FLASH_ADDR 0x10000
#define FLASH_BASE 0x0
#define BL_SIZE 0x2000
#define APP_BASE (FLASH_BASE + BL_SIZE)
#define FLASH_PAGE_SIZE 1024

size_t target_get_max_fw_size(void);
uint16_t target_get_timeout(void);
void target_flash_unlock(void);
bool target_flash_write(uint8_t* dst, uint8_t* src, size_t len);
bool target_prepare_flash(void);
void target_flash_lock(void);
void target_complete_programming(void);