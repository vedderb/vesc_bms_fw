/*
	Copyright 2019 - 2021 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC BMS firmware.

	The VESC BMS firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC BMS firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "flash_helper.h"
#include "timeout.h"
#include "main.h"
#include <string.h>

#define FLASH_PAGE_MAIN_APP			0
#define FLASH_PAGE_BACKUP			60
#define FLASH_BANK_MAIN_APP			FLASH_BANK_1
#define FLASH_BANK_BACKUP			FLASH_BANK_1
#define FLASH_ADDRESS_MAIN_APP		0x08000000
#define FLASH_ADDRESS_BACKUP		0x0801E000
#define FLASH_ADDRESS_NEW_APP		0x08020000
#define FLASH_ADDRESS_BOOTLOADER	0x0803E000
#define FLASH_PAGES_MAIN_APP		60
#define FLASH_PAGES_BACKUP			4
#define FLASH_PAGES_BOOTLOADER		4
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE				2048
#endif
#define MAX_SIZE_MAIN_APP			(FLASH_PAGES_MAIN_APP * FLASH_PAGE_SIZE)

uint16_t flash_helper_erase_new_app(uint32_t new_app_size) {
	uint32_t bank = FLASH_BANK_2;
	uint32_t page = 256;
	if (*STM32_FLASH_SIZE != 256) {
		bank = FLASH_BANK_1;
		page = 64;
	}

	(void)new_app_size; // TODO: Only erase enough pages to fit the new app

	timeout_configure_IWDT_slowest();

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	FLASH_EraseInitTypeDef eType;
	eType.TypeErase = FLASH_TYPEERASE_PAGES;
	eType.Banks = bank;
	eType.Page = page;
	eType.NbPages = FLASH_PAGES_MAIN_APP;

	uint32_t res = 0;
	uint16_t res2 = HAL_FLASHEx_Erase(&eType, &res);

	HAL_FLASH_Lock();

	timeout_configure_IWDT();

	return res2;
}

uint16_t flash_helper_erase_bootloader(void) {
	uint32_t bank = FLASH_BANK_2;
	uint32_t page = 316;
	if (*STM32_FLASH_SIZE != 256) {
		bank = FLASH_BANK_1;
		page = 124;
	}

	timeout_configure_IWDT_slowest();

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	FLASH_EraseInitTypeDef eType;
	eType.TypeErase = FLASH_TYPEERASE_PAGES;
	eType.Banks = bank;
	eType.Page = page;
	eType.NbPages = FLASH_PAGES_BOOTLOADER;

	uint32_t res = 0;
	uint16_t res2 = HAL_FLASHEx_Erase(&eType, &res);

	HAL_FLASH_Lock();

	timeout_configure_IWDT();

	return res2;
}

uint16_t flash_helper_write_new_app_data(uint32_t offset, uint8_t *data, uint32_t len) {
	return flash_helper_write_data(FLASH_ADDRESS_NEW_APP, offset, data, len);
}

uint16_t flash_helper_write_data(uint32_t base, uint32_t offset, uint8_t *data, uint32_t len) {
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	for (uint32_t i = 0;i < len;i += 8) {
		uint64_t dword =
				((uint64_t) data[i + 7]) << 56 |
				((uint64_t) data[i + 6]) << 48 |
				((uint64_t) data[i + 5]) << 40 |
				((uint64_t) data[i + 4]) << 32 |
				((uint64_t) data[i + 3]) << 24 |
				((uint64_t) data[i + 2]) << 16 |
				((uint64_t) data[i + 1]) << 8 |
				((uint64_t) data[i + 0]) << 0;


		int16_t res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, base + offset + i, dword);
		if (res != HAL_OK) {
			return res;
		}
	}

	HAL_FLASH_Lock();

	return HAL_OK;
}

void flash_helper_jump_to_bootloader(void) {
	backup.usb_cnt = 3; // Check USB directly after fw-upload to reconenct faster.

	flash_helper_store_backup_data();

	// Set magic number to jump to bootloader early in the next reset before
	// setting up clocks and peripherals.
	extern uint32_t __ram4_end__;
#define SYMVAL(sym) (uint32_t)(((uint8_t *)&(sym)) - ((uint8_t *)0))
	*((unsigned long *)(SYMVAL(__ram4_end__) - 4)) = 0xDEADBEEF;

	NVIC_SystemReset();
}

uint16_t flash_helper_erase_backup_data(void) {
	timeout_configure_IWDT_slowest();

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	FLASH_EraseInitTypeDef eType;
	eType.TypeErase = FLASH_TYPEERASE_PAGES;
	eType.Banks = FLASH_BANK_BACKUP;
	eType.Page = FLASH_PAGE_BACKUP;
	eType.NbPages = FLASH_PAGES_BACKUP;

	uint32_t res = 0;
	uint16_t res2 = HAL_FLASHEx_Erase(&eType, &res);

	HAL_FLASH_Lock();

	timeout_configure_IWDT();

	return res2;
}

void flash_helper_store_backup_data(void) {
	backup.conf_flash_write_cnt++;
	flash_helper_erase_backup_data();
	flash_helper_write_data(FLASH_ADDRESS_BACKUP, 0, (void*)&backup, sizeof(backup_data));
}

void flash_helper_load_backup_data(void) {
	memcpy((void*)&backup, (uint8_t*)FLASH_ADDRESS_BACKUP, sizeof(backup_data));
}
