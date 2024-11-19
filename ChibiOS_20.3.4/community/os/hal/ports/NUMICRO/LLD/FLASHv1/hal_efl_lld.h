/*
  ChibiOS - Copyright (C) 2020 Alex Lewontin

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
 * @brief   NUC123 Embedded Flash subsystem low level driver header.
 * @note    This driver only supports management of APROM. LDROM, config
 *          registers, and data flash (not yet supported by the platform driver)
 *          are not supported.
 * @addtogroup HAL_EFL
 * @{
 */

#ifndef HAL_EFL_LLD_H
#define HAL_EFL_LLD_H

#if HAL_USE_EFL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    NUC123 configuration options
 * @{
 */
/**
 * @brief   EFL1 driver enable switch.
 * @details If set to @p TRUE the support for EFL1 is included.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(NUC123_EFL_USE_EFL1) || defined(__DOXYGEN__)
#define NUC123_EFL_USE_EFL1 TRUE
#endif

/**
 * @brief   APROM access enable switch.
 * @details If set to @p TRUE the support for APROM access is included.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(NUC123_EFL_ACCESS_APROM) || defined(__DOXYGEN__)
#define NUC123_EFL_ACCESS_APROM FALSE
#endif

/**
 * @brief   Data flash access enable switch.
 * @details If set to @p TRUE the support for data flash access is included.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(NUC123_EFL_ACCESS_DATAFLASH) || defined(__DOXYGEN__)
#define NUC123_EFL_ACCESS_DATAFLASH FALSE
#endif

/**
 * @brief   LDROM access enable switch.
 * @details If set to @p FALSE the support for LDROM access is included.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(NUC123_EFL_ACCESS_LDROM) || defined(__DOXYGEN__)
#define NUC123_EFL_ACCESS_LDROM FALSE
#endif

/**
 * @brief   CONFIG0/1 access enable switch.
 * @details If set to @p FALSE the support for CONFIG0/1 access is included.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(NUC123_EFL_ACCESS_CONFIG) || defined(__DOXYGEN__)
#define NUC123_EFL_ACCESS_CONFIG FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef enum {
#if (NUC123_EFL_ACCESS_APROM == TRUE) || defined(__DOXYGEN__)
  NUC123_EFL_BANK_APROM,
#endif
#if (NUC123_EFL_ACCESS_DATAFLASH == TRUE) || defined(__DOXYGEN__)
  NUC123_EFL_BANK_DATAFLASH,
#endif
#if (NUC123_EFL_ACCESS_LDROM == TRUE) || defined(__DOXYGEN__)
  NUC123_EFL_BANK_LDROM,
#endif
#if (NUC123_EFL_ACCESS_CONFIG == TRUE) || defined(__DOXYGEN__)
  NUC123_EFL_BANK_CONFIG,
#endif
  NUC123_EFL_BANK_NONE
} nuc123_eflbank_t;
/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the embedded flash driver structure.
 */
#define efl_lld_driver_fields                                               \
  /* The currently used bank.*/                                       \
  nuc123_eflbank_t bank

/**
 * @brief   Low level fields of the embedded flash configuration structure.
 */
#define efl_lld_config_fields                                               \
  /* Dummy configuration, it is not needed.*/                               \
  uint32_t dummy

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (NUC123_EFL_USE_EFL1 == TRUE) && !defined(__DOXYGEN__)
    extern EFlashDriver EFLD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
void                      efl_lld_init(void);
void                      efl_lld_start(EFlashDriver *eflp);
void                      efl_lld_stop(EFlashDriver *eflp);
const flash_descriptor_t *efl_lld_get_descriptor(void *instance);
flash_error_t efl_lld_read(void *instance, flash_offset_t offset, size_t n,
                           uint8_t *rp);
flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                              size_t n, const uint8_t *pp);
flash_error_t efl_lld_start_erase_all(void *instance);
flash_error_t efl_lld_start_erase_sector(void *instance, flash_sector_t sector);
flash_error_t efl_lld_query_erase(void *instance, uint32_t *msec);
flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EFL == TRUE */

#endif /* HAL_EFL_LLD_H */

/** @} */
