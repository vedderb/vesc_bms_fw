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
 * @file    hal_efl_lld.h
 * @brief   NRF52XX Embedded Flash subsystem low level driver header.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#ifndef HAL_EFL_LLD_H
#define HAL_EFL_LLD_H

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

#if !defined(NRF52_FLASH_WAIT_TIME_MS)
#define NRF52_FLASH_WAIT_TIME_MS   90
#endif /* !defined(NRF52_FLASH_WAIT_TIME_MS) */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Flag for the whole NRF52xx family. */
#if (defined(NRF_SERIES) && NRF_SERIES == 52) || defined(__DOXYGEN__)
/*
 * Flash size is organized as 64 or 128 x 4K pages.
 *
 */
#define NRF52_FLASH_BASE 	0
#define NRF52_BLOCK_WRITE	181
#define NRF52_ERASE_PAGE	90
#define NRF52_ERASE_ALL		296
#define NRF52_PAGE_SIZE		4096
#define NRF52_BLOCK_COUNT	8
#else
#error "This EFL driver does not support the selected device"
#endif

#if defined(NRF52832_QFAA) || defined(NRF52832_CIAA) || defined(__DOXYGEN__)
#define NRF52_PAGE_COUNT     128
#elif defined(NRF52832_QFAB)
#define NRF52_PAGE_COUNT     64
#else
#error "Missing NRF52xx flash definition"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/* A flash size declaration. */
typedef struct {
  const flash_descriptor_t* desc;
} efl_lld_size_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the embedded flash driver structure.
 */
#define efl_lld_driver_fields                                               \
  /* Flash registers.*/                                                     \
  NRF_NVMC_Type             *flash;                                         \
  const flash_descriptor_t  *descriptor;

/**
 * @brief   Low level fields of the embedded flash configuration structure.
 */
#define efl_lld_config_fields                                               \
  /* Dummy configuration, it is not needed.*/                               \
  uint32_t                  dummy

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern EFlashDriver EFLD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void efl_lld_init(void);
  void efl_lld_start(EFlashDriver *eflp);
  void efl_lld_stop(EFlashDriver *eflp);
  const flash_descriptor_t* efl_lld_get_descriptor(void *instance);
  flash_error_t efl_lld_read(void *instance, flash_offset_t offset,
                               size_t n, uint8_t *rp);
  flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                                  size_t n, const uint8_t *pp);
  flash_error_t efl_lld_start_erase_all(void *instance);
  flash_error_t efl_lld_start_erase_sector(void *instance,
                                             flash_sector_t sector);
  flash_error_t efl_lld_query_erase(void *instance, uint32_t *wait_time);
  flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector);
#ifdef __cplusplus
	}
#endif

#endif /* HAL_USE_EFL == TRUE */

#endif /* HAL_EFL_LLD_H */

/** @} */
