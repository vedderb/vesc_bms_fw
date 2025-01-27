/*
    Copyright (C) 2021 andru

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

/**
 * @file    NRF52XX hal_efl_lld.c
 * @brief   NRF52XX Embedded Flash subsystem low level driver source.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#include "hal.h"

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   EFL1 driver identifier.
 */
EFlashDriver EFLD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Internal NRF52xx descriptor.
 */
static flash_descriptor_t flash_descriptor = {
  .attributes       = FLASH_ATTR_ERASED_IS_ONE | FLASH_ATTR_REWRITABLE |
                      FLASH_ATTR_SUSPEND_ERASE_CAPABLE,
  .page_size        = NRF52_PAGE_SIZE,
  .sectors_count    = NRF52_PAGE_COUNT,
  .sectors          = NULL,
  .sectors_size     = NRF52_PAGE_SIZE,
  .address          = (uint8_t *) NRF52_FLASH_BASE
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void nrf52_flash_lock(void *instance) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  devp->flash->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos) & NVMC_CONFIG_WEN_Msk;
}

static void nrf52_flash_write_unlock(void *instance) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  devp->flash->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos) & NVMC_CONFIG_WEN_Msk;
}

static void nrf52_flash_erase_unlock(void *instance) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  devp->flash->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos) & NVMC_CONFIG_WEN_Msk;
}

static flash_error_t nrf52_flash_wait_busy(EFlashDriver *devp) {
  uint32_t READY;

  do {
	READY = devp->flash->READY;
  } while( (READY & NVMC_READY_READY_Msk) == NVMC_READY_READY_Busy);

  return FLASH_NO_ERROR;
}

static flash_error_t nrf52_flash_program_word(EFlashDriver *devp,
                                                uint32_t address,
                                                uint32_t wdata) {
  nrf52_flash_wait_busy(devp);

  *(volatile uint32_t*)address = wdata;

  nrf52_flash_wait_busy(devp);

  if( *(volatile uint32_t*)address != wdata ) {
    return FLASH_ERROR_PROGRAM;
  }

  return FLASH_NO_ERROR; // FLASH_ERROR_ERASE
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level Embedded Flash driver initialization.
 *
 * @notapi
 */
void efl_lld_init(void) {
  /* Driver initialization.*/
  eflObjectInit(&EFLD1);
  EFLD1.flash = NRF_NVMC;

  uint16_t pagecount = NRF_FICR->CODESIZE;
  uint16_t pagesize = NRF_FICR->CODEPAGESIZE;
  osalDbgAssert((pagecount == NRF52_PAGE_COUNT) && (pagesize == NRF52_PAGE_SIZE),
				"invalid configuration");
}

/**
 * @brief   Configures and activates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_start(EFlashDriver *eflp) {
  (void)eflp;
}

/**
 * @brief   Deactivates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_stop(EFlashDriver *eflp) {
  (void)eflp;
}

/**
 * @brief   Gets the flash descriptor structure.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @return                          A flash device descriptor.
 * @retval                          Pointer to single bank if DBM not enabled.
 * @retval                          Pointer to bank1 if DBM enabled.
 *
 * @notapi
 */
const flash_descriptor_t *efl_lld_get_descriptor(void *instance) {
  (void)instance;

  return &flash_descriptor;
}

/**
 * @brief   Read operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                offset within full flash address space
 * @param[in] n                     number of bytes to be read
 * @param[out] rp                   pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_READ         if the read operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_read(void *instance, flash_offset_t offset,
                           size_t n, uint8_t *rp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  uint32_t address  = *flash_descriptor.address + offset;

  osalDbgCheck((instance != NULL) && (rp != NULL) && (n > 0U));

  osalDbgCheck((size_t)offset + n <= (size_t)flash_descriptor.sectors_count *
                                     (size_t)flash_descriptor.sectors_size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
               "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READY state while the operation is performed.*/
  devp->state = FLASH_READ;

  while(n > 3) {
    *((uint32_t*)rp) = *(volatile uint32_t*)address;

    address += 4;
    rp      += 4;
    n       -= 4;
  }

  if(n > 0) {
	uint32_t mask = (n == 3) ? 0x00FFFFFF : (n == 2) ? 0x0000FFFF : 0x000000FF;
	*((uint32_t*)rp) = (uint32_t)((*(volatile uint32_t*)address) & mask);
  }

  devp->state = FLASH_READY;

  return FLASH_NO_ERROR;
}

/**
 * @brief   Program operation.
 * @note    The device supports ECC, it is only possible to write erased
 *          pages once except when writing all zeroes.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                offset within full flash address space
 * @param[in] n                     number of bytes to be programmed
 * @param[in] pp                    pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_PROGRAM      if the program operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                              size_t n, const uint8_t *pp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  uint32_t address  = *flash_descriptor.address + offset;
  uint32_t wdata    = 0;
  flash_error_t fe  = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  osalDbgCheck((size_t)offset + n <= (size_t)flash_descriptor.sectors_count *
                                     (size_t)flash_descriptor.sectors_size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  devp->state = FLASH_PGM;

  nrf52_flash_write_unlock(devp);

  while(n > 3) {
    wdata = (*(uint32_t*)pp);

    fe = nrf52_flash_program_word(devp, address, wdata);
    if( fe != FLASH_NO_ERROR ) {
      nrf52_flash_lock(devp);
      devp->state = FLASH_READY;
      return FLASH_ERROR_PROGRAM;
    }

    address += 4;
    pp      += 4;
    n       -= 4;
  }

  if(n > 0) {
	uint32_t mask = (n == 3) ? 0xFF000000 : (n == 2) ? 0xFFFF0000 : 0xFFFFFF00;
    wdata = (*(uint32_t*)pp) | mask;

    fe = nrf52_flash_program_word(devp, address, wdata);
    if( fe != FLASH_NO_ERROR ) {
      nrf52_flash_lock(devp);
      devp->state = FLASH_READY;
      return FLASH_ERROR_PROGRAM;
    }
  }

  nrf52_flash_lock(devp);

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return FLASH_NO_ERROR;
}

/**
 * @brief   Starts a whole-device erase operation.
 * @note    This function only erases bank 2 if it is present. Bank 1 is not
 *          allowed since it is normally where the primary program is located.
 *          Pages on bank 1 can be individually erased.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_all(void *instance) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_ERASE state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  nrf52_flash_wait_busy(devp);

  nrf52_flash_erase_unlock(devp);

  nrf52_flash_wait_busy(devp);

  devp->flash->ERASEALL = NVMC_ERASEALL_ERASEALL_Erase;	// Erase ALL FLASH & UICR registers!!!

  /* flash_lld_lock must be done in query_erase */

  return FLASH_NO_ERROR;
}

/**
 * @brief   Starts an sector erase operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be erased
 *                                  this is an index within the total sectors
 *                                  in a flash bank
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_sector(void *instance,
                                         flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  uint32_t address  = *flash_descriptor.address +
                      (sector * flash_descriptor.sectors_size);

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < flash_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_ERASE state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  nrf52_flash_wait_busy(devp);

  nrf52_flash_erase_unlock(devp);

  nrf52_flash_wait_busy(devp);

  devp->flash->ERASEPAGE  = address;

  /* flash_lld_lock must be done in query_erase */

  return FLASH_NO_ERROR;
}

/**
 * @brief   Queries the driver for erase operation progress.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[out] msec                 recommended time, in milliseconds, that
 *                                  should be spent before calling this
 *                                  function again, can be @p NULL
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_ERASE        if the erase operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
flash_error_t efl_lld_query_erase(void *instance, uint32_t *msec) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if( msec ) {
      *msec = NRF52_FLASH_WAIT_TIME_MS;
  }

  /* If there is an erase in progress then the device must be checked.*/
  if (devp->state == FLASH_ERASE) {
    uint32_t READY = devp->flash->READY;

    if( (READY & NVMC_READY_READY_Msk) == NVMC_READY_READY_Busy ) {
        return FLASH_BUSY_ERASING;
    }

    nrf52_flash_lock(devp);

    devp->state = FLASH_READY;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief   Returns the erase state of a sector.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be verified
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if the sector is erased.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_VERIFY       if the verify operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  uint32_t address  = *flash_descriptor.address +
                      (sector * flash_descriptor.sectors_size);

  uint32_t wdata    = 0;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < flash_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Read specified sector. All data must be 0xFFFFFFFFU */
  uint32_t address_max = address + flash_descriptor.sectors_size;
  for(; address < address_max; address += 4) {

    wdata = *(volatile uint32_t*)address;

    if( wdata != 0xFFFFFFFF ) {
      return FLASH_ERROR_VERIFY;
    }
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return FLASH_NO_ERROR;
}

#endif /* HAL_USE_EFL == TRUE */

/** @} */
