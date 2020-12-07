/*
	Copyright 2019 - 2020 Benjamin Vedder	benjamin@vedder.se

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

#ifndef FLASH_HELPER_H_
#define FLASH_HELPER_H_

#include "conf_general.h"
#include "stm32l4xx_hal_conf.h"

// Functions
uint16_t flash_helper_erase_new_app(uint32_t new_app_size);
uint16_t flash_helper_erase_bootloader(void);
uint16_t flash_helper_write_new_app_data(uint32_t offset, uint8_t *data, uint32_t len);
uint16_t flash_helper_write_data(uint32_t base, uint32_t offset, uint8_t *data, uint32_t len);
void flash_helper_jump_to_bootloader(void);
uint16_t flash_helper_erase_backup_data(void);
void flash_helper_store_backup_data(void);
void flash_helper_load_backup_data(void);

#endif /* FLASH_HELPER_H_ */
