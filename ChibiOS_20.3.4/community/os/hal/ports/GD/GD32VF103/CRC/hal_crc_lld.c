/*
    ChibiOS - Copyright (C) 2015 Michael D. Spradling
    ChibiOS - Copyright (C) 2021 Stefan Kerkmann

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
 * @file    GD32VF103/CRC/hal_crc_lld.c
 * @brief   GD32 CRC subsystem low level driver source.
 *
 * @addtogroup CRC
 * @{
 */

#include "hal.h"

#if (HAL_USE_CRC == TRUE) || defined(__DOXYGEN__)
/**
 * Allow CRC Software override for ST drivers.  Some ST CRC implimentations
 * have limited capabilities.
 */
#if CRCSW_USE_CRC1 != TRUE
/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   CRC default configuration.
 */
static const CRCConfig default_config = {
  .poly_size         = 32,
  .poly              = 0x04C11DB7,
  .initial_val       = 0xFFFFFFFF,
  .final_val         = 0xFFFFFFFF,
  .reflect_data      = 1,
  .reflect_remainder = 1
};

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief CRC1 driver identifier.*/
#if GD32_CRC_USE_CRC0 || defined(__DOXYGEN__)
CRCDriver CRCD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void _crc_lld_calc_byte(CRCDriver *crcp, uint8_t data) {
  __IO uint8_t *crc8 = (__IO uint8_t*)&(crcp->crc->DATA);
  *crc8 = data;
}

/*
 * @brief   Returns calculated CRC from last reset
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 * @param[in] data      data to be added to crc
 *
 * @notapi
 */
void _crc_lld_calc_halfword(CRCDriver *crcp, uint16_t data) {
  __IO uint16_t *crc16 = (__IO uint16_t*)&(crcp->crc->DATA);
  *crc16 = data;
}

/*
 * @brief   Returns calculated CRC from last reset
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 * @param[in] data      data to be added to crc
 *
 * @notapi
 */
void _crc_lld_calc_word(CRCDriver *crcp, uint32_t data) {
  crcp->crc->DATA = data;
}


/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Shared end-of-rx service routine.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
#if CRC_USE_DMA == TRUE
static void crc_lld_serve_interrupt(CRCDriver *crcp, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(GD32_CRC_DMA_ERROR_HOOK)
  if ((flags & (GD32_DMA_INTF_ERRIF)) != 0) {
    GD32_CRC_DMA_ERROR_HOOK(crcp);
  }
#else
  (void)flags;
#endif

  /* Stop everything.*/
  dmaStreamDisable(crcp->dmastp);

  if (crcp->rem_data_size) {
    /* Start DMA follow up transfer for next data chunk */
    crc_lld_start_calc(crcp, crcp->rem_data_size,
      (const void *)crcp->dmastp->channel->PADDR+0xffff);
  } else {
    /* Portable CRC ISR code defined in the high level driver, note, it is a macro.*/
    _crc_isr_code(crcp, crcp->crc->DATA ^ crcp->config->final_val);
  }
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level CRC driver initialization.
 *
 * @notapi
 */
void crc_lld_init(void) {
  crcObjectInit(&CRCD1);
  CRCD1.crc    = CRC;
}

/**
 * @brief   Configures and activates the CRC peripheral.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 *
 * @notapi
 */
void crc_lld_start(CRCDriver *crcp) {
  if (crcp->config == NULL)
    crcp->config = &default_config;

  rcuEnableCRC( FALSE );

  osalDbgAssert(crcp->config->initial_val == default_config.initial_val,
      "hardware doesn't support programmable initial value");
  osalDbgAssert(crcp->config->poly_size == default_config.poly_size,
      "hardware doesn't support programmable polynomial size");
  osalDbgAssert(crcp->config->poly == default_config.poly,
      "hardware doesn't support programmable polynomial");
  osalDbgAssert(crcp->config->reflect_data == default_config.reflect_data,
      "hardware doesn't support reflect of input data");
  osalDbgAssert(crcp->config->reflect_remainder == default_config.reflect_remainder,
      "hardware doesn't support reflect of output remainder");

#if CRC_USE_DMA == TRUE
  crcp->dmamode = GD32_DMA_CTL_DIR_M2M     | GD32_DMA_CTL_PNAGA |
                  GD32_DMA_CTL_MWIDTH_WORD | GD32_DMA_CTL_PWIDTH_WORD |
                  GD32_DMA_CTL_ERRIE       | GD32_DMA_CTL_FTFIE |
                  GD32_DMA_CTL_PRIO(GD32_CRC_CRC0_DMA_PRIORITY);
  {
    crcp->dmastp = dmaStreamAlloc(GD32_CRC_CRC0_DMA_STREAM,
                                                 GD32_CRC_CRC0_DMA_IRQ_PRIORITY,
                                                 (gd32_dmaisr_t)crc_lld_serve_interrupt,
                                                 (void *)crcp);
    osalDbgAssert(crcp->dmastp != NULL, "unable to allocate stream");
  }
#endif
}


/**
 * @brief   Deactivates the CRC peripheral.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 *
 * @notapi
 */
void crc_lld_stop(CRCDriver *crcp) {
#if CRC_USE_DMA == TRUE
  dmaStreamFree(crcp->dmastp);
#else
  (void)crcp;
#endif
  rcuDisableCRC();
}

/**
 * @brief   Resets current CRC calculation.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 *
 * @notapi
 */
void crc_lld_reset(CRCDriver *crcp) {
  crcp->crc->CTL |= CRC_CTL_RST;
}

/**
 * @brief   Returns calculated CRC from last reset
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 * @param[in] n         size of buf in bytes
 * @param[in] buf       @p buffer location
 *
 * @notapi
 */
uint32_t crc_lld_calc(CRCDriver *crcp, size_t n, const void *buf) {
#if CRC_USE_DMA == TRUE
  crc_lld_start_calc(crcp, n, buf);
  (void) osalThreadSuspendS(&crcp->thread);
#else
    while(n > 3) {
      _crc_lld_calc_word(crcp, *(uint32_t*)buf);
      buf+=4;
      n-=4;
    }
  osalDbgAssert(n == 0, "GD32 CRC Unit only supports WORD accesses");
#endif
  return crcp->crc->DATA ^ crcp->config->final_val;
}

#if CRC_USE_DMA == TRUE
void crc_lld_start_calc(CRCDriver *crcp, size_t n, const void *buf) {
  /* The GD32 DMA can only handle max 65535 bytes per transfer
   * because it's data count register has only 16 bit. */
  size_t sz = (n > 0xffff) ? 0xffff : n;
  crcp->rem_data_size = n-sz;

  dmaStreamSetPeripheral(crcp->dmastp, buf);
  dmaStreamSetMemory0(crcp->dmastp, &crcp->crc->DATA);
  dmaStreamSetTransactionSize(crcp->dmastp, (sz / 4));
  dmaStreamSetMode(crcp->dmastp, crcp->dmamode);

  dmaStreamEnable(crcp->dmastp);
}
#endif

#endif /* CRCSW_USE_CRC1 */

#endif /* HAL_USE_CRC */

/** @} */
