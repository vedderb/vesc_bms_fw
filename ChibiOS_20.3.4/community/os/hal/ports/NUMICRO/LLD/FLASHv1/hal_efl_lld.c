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
 * @file    hal_efl_lld.c
 * @brief   NUC123 Embedded Flash subsystem low level driver source.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#include "hal.h"

#if HAL_USE_EFL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define NUC123_LFOM_UPPERBOUND 25000000UL
#define NUC123_MFOM_UPPERBOUND 50000000UL

#define FMC_FATCON_LFOM_Pos FMC_FATCON_FOMSEL0_Pos
#define FMC_FATCON_LFOM_Msk FMC_FATCON_FOMSEL0_Msk
#define FMC_FATCON_MFOM_Pos FMC_FATCON_FOMSEL1_Pos
#define FMC_FATCON_MFOM_Msk FMC_FATCON_FOMSEL1_Msk

#define NUC123_PAGE_SIZE   4UL
#define NUC123_SECTOR_SIZE 0x200UL

#define NUC123_LDROM_SIZE 0x1000UL

#define NUC123_EFL_CMD_ERASE     0x22UL
#define NUC123_EFL_CMD_PROG      0x21UL
#define NUC123_EFL_CMD_READ      0UL
#define NUC123_EFL_CMD_CHIPERASE 0x26UL /* Undocumented */

#if ((NUC123_CONFIG_ENABLED == FALSE) || (NUC123_EFL_ACCESS_CONFIG == TRUE)) && \
    (NUC123_EFL_ACCESS_APROM == TRUE) || (NUC123_EFL_ACCESS_DATAFLASH == TRUE)
#define NUC123_EFL_DYNAMICALLY_CHECK_CONFIG TRUE
#else
#define NUC123_EFL_DYNAMICALLY_CHECK_CONFIG FALSE
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   EFL1 driver identifier.
 */
#if (NUC123_EFL_USE_EFL1 == TRUE) || defined(__DOXYGEN__)
EFlashDriver EFLD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static flash_descriptor_t efl_lld_descriptor;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/* Starts the ISP function, does not block */
static inline void start_ISP(void)
{
  SystemUnlockReg();
  FMC->ISPTRG |= FMC_ISPTRG_ISPGO_Msk;
  __ISB();
  LOCKREG();
}

/* Starts the ISP function and blocks til conclusion */
static inline flash_error_t do_ISP(void)
{
    start_ISP();
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
      ;
    if (FMC->ISPCON & FMC_ISPCON_ISPFF_Msk) {
      return FLASH_ERROR_HW_FAILURE;
    }

    return FLASH_NO_ERROR;
}

/**
 * @brief   Returns the minimum of two unsigned values. A safer implementation
 *          of the classic macro
 *
 * @param[in] x           An unsigned value
 * @param[in] y           An unsigned value
 *
 * @return                The smaller of x and y
 *
 * @notapi
 */
static inline unsigned min(unsigned x, unsigned y)
{
  return ((x > y) ? y : x);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level Embedded Flash driver initialization.
 *
 * @notapi
 */
void efl_lld_init(void)
{
  efl_lld_descriptor = (flash_descriptor_t){
      .attributes = FLASH_ATTR_ERASED_IS_ONE | FLASH_ATTR_MEMORY_MAPPED |
                    FLASH_ATTR_REWRITABLE,
      .page_size    = NUC123_PAGE_SIZE,
      .sectors      = NULL,
      .sectors_size = NUC123_SECTOR_SIZE,
  };

#if (NUC123_EFL_USE_EFL1 == TRUE)
  eflObjectInit(&EFLD1);
  EFLD1.bank = 0;
#endif

}

/**
 * @brief   Configures and activates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_start(EFlashDriver* eflp)
{
  uint32_t ispcon;
  if (eflp->state == FLASH_STOP) {
    SystemUnlockReg();
    ispcon = FMC->ISPCON;

    ispcon |= (FMC_ISPCON_ISPEN_Msk);
    /* Enables the peripheral.*/
    CLK->APBCLK |= CLK_AHBCLK_ISP_EN_Msk;

#if NUC123_HCLK < NUC123_LFOM_UPPERBOUND
    FMC->FATCON |= FMC_FATCON_LFOM_Msk;
#elif NUC123_HCLK < NUC123_MFOM_UPPERBOUND
    FMC->FATCON |= FMC_FATCON_MFOM_Msk;
#endif

#if (NUC123_EFL_ACCESS_APROM == TRUE)
    ispcon |= FMC_ISPCON_APUEN_Msk;
#endif

#if (NUC123_EFL_ACCESS_LDROM == TRUE)
    ispcon |= FMC_ISPCON_LDUEN_Msk;
#endif

#if (NUC123_EFL_ACCESS_CONFIG == TRUE)
    ispcon |= FMC_ISPCON_CFGUEN_Msk;
#endif

    FMC->ISPCON = ispcon;
    LOCKREG();
  }
}

/**
 * @brief   Deactivates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_stop(EFlashDriver* eflp)
{
  uint32_t ispcon;
  if (eflp->state == FLASH_READY) {
    SystemUnlockReg();
    ispcon = FMC->ISPCON;

    ispcon &= ~FMC_ISPCON_ISPEN_Msk;
      /* Disables the peripheral.*/

#if (NUC123_EFL_ACCESS_APROM == TRUE)
    ispcon &= ~FMC_ISPCON_APUEN_Msk;
#endif
#if (NUC123_EFL_ACCESS_LDROM == TRUE)
    ispcon &= ~FMC_ISPCON_LDUEN_Msk;
#endif
#if (NUC123_EFL_ACCESS_CONFIG == TRUE)
      ispcon &= ~FMC_ISPCON_CFGUEN_Msk;
#endif

    FMC->ISPCON = ispcon;
    CLK->APBCLK &= ~CLK_AHBCLK_ISP_EN_Msk;
    LOCKREG();
  }
}

/**
 * @brief   Gets the flash descriptor structure.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @return                          A flash device descriptor.
 *
 * @notapi
 */
const flash_descriptor_t* efl_lld_get_descriptor(void* instance)
{
  size_t dataflash_size;
  void* dfbaddr;

#if (NUC123_EFL_DYNAMICALLY_CHECK_CONFIG == TRUE)

  uint32_t ispcon = FMC->ISPCON;

  FMC->ISPCON = ispcon | FMC_ISPCON_CFGUEN_Msk;
  FMC->ISPCMD     = NUC123_EFL_CMD_READ;

  FMC->ISPADR = 0x300000UL;
  do_ISP();
  dataflash_size = FMC->ISPDAT;

  if (dataflash_size & 4) {
    /* DFVSEN = 1 */
    dataflash_size = 4096;
    dfbaddr        = (void *)0x1F000UL;
  } else {
    if (dataflash_size & 1) {
      /* DFVSEN = 0 & DFEN = 1 */
      dataflash_size = 0;
      dfbaddr        = (void *)0xFFFFF000UL;
    } else {
      /* DFVSEN = 0 & DFEN = 0 */
      dfbaddr = (void *)FMC->DFBADR;
      dataflash_size = NUC123_FLASH_SIZE - (uint32_t)dfbaddr;
    }
  }

  FMC->ISPCON = ispcon;

#else

  dataflash_size = NUC123_CONFIG_DATAFLASH_SIZE;
  dfbaddr = (void *)NUC123_DFBADDR;

#endif

  switch (((EFlashDriver *)instance)->bank) {
#if (NUC123_EFL_ACCESS_APROM == TRUE)
  case NUC123_EFL_BANK_APROM:
    efl_lld_descriptor.address       = (uint8_t *)0;
    efl_lld_descriptor.sectors_count = (NUC123_FLASH_SIZE - dataflash_size) / NUC123_SECTOR_SIZE;
    efl_lld_descriptor.size          = (NUC123_FLASH_SIZE - dataflash_size);
    break;
#endif
#if (NUC123_EFL_ACCESS_DATAFLASH == TRUE)
  case NUC123_EFL_BANK_DATAFLASH:
    efl_lld_descriptor.address       = (uint8_t *)dfbaddr;
    efl_lld_descriptor.sectors_count = dataflash_size / NUC123_SECTOR_SIZE;
    efl_lld_descriptor.size          = dataflash_size;
    break;
#endif
#if (NUC123_EFL_ACCESS_LDROM == TRUE)
  case NUC123_EFL_BANK_LDROM:
    efl_lld_descriptor.address       = (uint8_t *)0x100000;
    efl_lld_descriptor.sectors_count = NUC123_LDROM_SIZE / NUC123_SECTOR_SIZE;
    efl_lld_descriptor.size          = NUC123_LDROM_SIZE;
    break;
#endif
#if (NUC123_EFL_ACCESS_CONFIG == TRUE)
  case NUC123_EFL_BANK_CONFIG:
    efl_lld_descriptor.address       = (uint8_t *)0x300000;
    efl_lld_descriptor.sectors_count = 1;
    efl_lld_descriptor.size          = 8;
    break;
#endif
  case NUC123_EFL_BANK_NONE:
  default: 
    return NULL;
  }
  return &efl_lld_descriptor;
}

/**
 * @brief   Read operation.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[in] offset                flash offset
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
flash_error_t efl_lld_read(void *instance, flash_offset_t offset, size_t n, uint8_t *rp)
{
  EFlashDriver             *devp = (EFlashDriver *)instance;
  const flash_descriptor_t *desc = efl_lld_get_descriptor(instance);
  flash_error_t             err  = FLASH_NO_ERROR;
  uint32_t                  data;

  osalDbgCheck((instance != NULL) && (rp != NULL) && (n > 0U));
  osalDbgCheck(((size_t)offset + n) <= (size_t)desc->size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE), 
                "invalid state");

  /* No reading while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  FMC->ISPCMD = NUC123_EFL_CMD_READ;

  while (n) {
    FMC->ISPADR = (uint32_t)desc->address + (offset & ~3);
    err         = do_ISP();
    if (err) {
      break;
    }
    data = FMC->ISPDAT;
    /* For most iterations, the switch doesn't do anything, but it doesn't
    hurt, and it allows us to consolidate the leading partial word
    with the rest, which compiles smaller. The compiler is pretty good
    at pulling things out of the loop, if speed is more important. */
    switch (offset % 4) {
    case 0:
      *(rp++) = (uint8_t)((data >> 0) & 0xFF);
      if (!(--n)) { break; }
      /* fallthrough */
    case 1:
      *(rp++) = (uint8_t)((data >> 8) & 0xFF);
      if (!(--n)) { break; }
      /* fallthrough */
    case 2:
      *(rp++) = (uint8_t)((data >> 16) & 0xFF);
      if (!(--n)) { break; }
      /* fallthrough */
    case 3:
      *(rp++) = (uint8_t)((data >> 24) & 0xFF);
      if (!(--n)) { break; }
    }
    offset += (4 - (offset % 4));
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

/**
 * @brief   Program operation.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[in] offset                flash offset
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
flash_error_t efl_lld_program(void* instance, flash_offset_t offset,
                              size_t n, const uint8_t* pp)
{
  EFlashDriver* devp = (EFlashDriver*)instance;
  const flash_descriptor_t *desc = efl_lld_get_descriptor(instance);
  flash_error_t err  = FLASH_NO_ERROR;
  uint32_t                  data;

  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  osalDbgCheck(((size_t)offset + n) <= (size_t)desc->size);

  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No programming while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_PGM;

  if (offset % 4) {
    FMC->ISPCMD = NUC123_EFL_CMD_READ;
    FMC->ISPADR = (uint32_t)desc->address + (offset & ~3);
    err         = do_ISP();
    if (err) {
      /* Ready state again.*/
      devp->state = FLASH_READY;
      return err;
    }
    data = FMC->ISPDAT;
    switch (offset % 4) {
    case 1:
      data &= 0xFFFF00FF;
      data |= (*(pp++) << 8);
      if (!(--n)) {
        break;
      }
      /* fallthrough */
    case 2:
      data &= 0xFF00FFFF;
      data |= (*(pp++) << 16);
      if (!(--n)) {
        break;
      }
      /* fallthrough */
    case 3:
      data &= 0x00FFFFFF;
      data |= (*(pp++) << 24);
      if (!(--n)) {
        break;
      }
    }

    FMC->ISPDAT = data;
    FMC->ISPCMD = NUC123_EFL_CMD_PROG;
    err         = do_ISP();
    if (err) {
      /* Ready state again.*/
      devp->state = FLASH_READY;
      return err;
    }

    offset += 4 - (offset % 4);
  }

  FMC->ISPCMD = NUC123_EFL_CMD_PROG;

  while (n >= 4) {
    FMC->ISPADR = (uint32_t)desc->address + offset;
    FMC->ISPDAT = (*(pp + 0)) | (*(pp + 1) << 8) |
                  (*(pp + 2) << 16) | (*(pp + 3) << 24);
    err = do_ISP();
    if (err) {
      /* Ready state again.*/
      devp->state = FLASH_READY;
      return err;
    }
    n -= 4;
    pp += 4;
    offset += 4;
  }

  if (n) {
    FMC->ISPCMD = NUC123_EFL_CMD_READ;
    FMC->ISPADR = (uint32_t)desc->address + offset;
    err         = do_ISP();
    if (err) {
      /* Ready state again.*/
      devp->state = FLASH_READY;
      return err;
    }
    data = FMC->ISPDAT;

    switch (n) {
    case 3:
      data &= 0xFF00FFFF;
      data |= (pp[2] << 16);
      /* fallthrough */
    case 2:
      data &= 0xFFFF00FF;
      data |= (pp[1] << 8);
      /* fallthrough */
    case 1:
      data &= 0xFFFFFF00;
      data |= (pp[0] << 0);
    }
    FMC->ISPDAT = data;
    FMC->ISPCMD = NUC123_EFL_CMD_PROG;
    err         = do_ISP();
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;
  return err;
}

/**
 * @brief   Starts a whole-device erase operation.
 * @note    This operation erases the entirety of the bank associated with
 *          the driver indicated by @p instance. Calling this function on
 *          the bank that contains the code that is currently executing
 *          will result in undefined behavior.
 * @note    This operation is not supported asynchronously, so this will
 *          not return until all erases have been completed.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_all(void* instance)
{
  EFlashDriver* devp = (EFlashDriver*)instance;
  const flash_descriptor_t *desc = efl_lld_get_descriptor(instance);
  flash_error_t             err  = FLASH_NO_ERROR;

  osalDbgCheck(instance != NULL);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  FMC->ISPCMD = NUC123_EFL_CMD_ERASE;

  for (uint8_t i = 0; i < desc->sectors_count; ++i)
  {
    FMC->ISPADR = (uint32_t)((i * (desc->sectors_size)) +
                              desc->address);
    err = do_ISP();
    if (err) {
      break;
    }
  }

  devp->state = FLASH_READY;
  return err;
}

/**
 * @brief   Starts an sector erase operation.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be erased
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_sector(void*          instance,
                                         flash_sector_t sector)
{
  EFlashDriver* devp = (EFlashDriver*)instance;
  const flash_descriptor_t *desc = efl_lld_get_descriptor(instance);

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < desc->sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_ERASE state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  FMC->ISPCMD = NUC123_EFL_CMD_ERASE;
  FMC->ISPADR = (uint32_t)((sector * (desc->sectors_size)) +
                           desc->address);
  start_ISP();

  return FLASH_NO_ERROR;
}

/**
 * @brief   Queries the driver for erase operation progress.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
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
flash_error_t efl_lld_query_erase(void* instance, uint32_t* msec)
{
  EFlashDriver* devp = (EFlashDriver*)instance;

  /* TODO: figure out an actual amount of time */
  *msec = 0UL;

  /* If there is an erase in progress then the device must be checked.*/
  if (devp->state == FLASH_ERASE) {
    if (FMC->ISPSTA & FMC_ISPSTA_ISPGO_Msk) {
      return FLASH_BUSY_ERASING;
    }

    if (FMC->ISPCON & FMC_ISPCON_ISPFF_Msk) {
      return FLASH_ERROR_HW_FAILURE;
    }

    devp->state = FLASH_READY;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief   Returns the erase state of a sector.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be verified
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if the sector is erased.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_VERIFY       if the verify operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_verify_erase(void* instance, flash_sector_t sector)
{
  EFlashDriver* devp = (EFlashDriver*)instance;
  const flash_descriptor_t *desc = efl_lld_get_descriptor(instance);
  flash_error_t err  = FLASH_NO_ERROR;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < desc->sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");
  /* No verifying while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  devp->state = FLASH_READ;

  FMC->ISPCMD = NUC123_EFL_CMD_READ;

  for (size_t i = 0; i < desc->sectors_size; i = i + 4) {
    FMC->ISPADR = (uint32_t)desc->address + (desc->sectors_size * sector) + i;
    err         = do_ISP();
    if (err) {
      break;
    }
    if (FMC->ISPDAT != 0xFFFFFFFF) {
      err = FLASH_ERROR_VERIFY;
      break;
    }
  }

    /* Ready state again.*/
  devp->state = FLASH_READY;
  return err;
}

#endif /* HAL_USE_EFL == TRUE */

/** @} */
