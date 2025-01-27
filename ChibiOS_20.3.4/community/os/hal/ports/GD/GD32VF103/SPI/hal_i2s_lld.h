/*
     ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio
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
 * @file    SPI/hal_i2s_lld.h
 * @brief   GD32 I2S subsystem low level driver header.
 *
 * @addtogroup I2S
 * @{
 */

#ifndef HAL_I2S_LLD_H
#define HAL_I2S_LLD_H

#if HAL_USE_I2S || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Static I2S modes
 * @{
 */
#define GD32_I2S_MODE_SLAVE                0
#define GD32_I2S_MODE_MASTER               1
#define GD32_I2S_MODE_RX                   2
#define GD32_I2S_MODE_TX                   4
#define GD32_I2S_MODE_RXTX                 (GD32_I2S_MODE_RX |            \
                                             GD32_I2S_MODE_TX)
/** @} */

/**
 * @name    Mode checks
 * @{
 */
#define GD32_I2S_IS_MASTER(mode)           ((mode) & GD32_I2S_MODE_MASTER)
#define GD32_I2S_RX_ENABLED(mode)          ((mode) & GD32_I2S_MODE_RX)
#define GD32_I2S_TX_ENABLED(mode)          ((mode) & GD32_I2S_MODE_TX)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   I2S1 driver enable switch.
 * @details If set to @p TRUE the support for I2S1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(GD32_I2S_USE_SPI0) || defined(__DOXYGEN__)
#define GD32_I2S_USE_SPI0                  FALSE
#endif

/**
 * @brief   I2S2 driver enable switch.
 * @details If set to @p TRUE the support for I2S2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(GD32_I2S_USE_SPI1) || defined(__DOXYGEN__)
#define GD32_I2S_USE_SPI1                  FALSE
#endif

/**
 * @brief   I2S3 driver enable switch.
 * @details If set to @p TRUE the support for I2S3 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(GD32_I2S_USE_SPI2) || defined(__DOXYGEN__)
#define GD32_I2S_USE_SPI2                  FALSE
#endif

/**
 * @brief   I2S2 mode.
 */
#if !defined(GD32_I2S_SPI1_MODE) || defined(__DOXYGEN__)
#define GD32_I2S_SPI1_MODE                 (GD32_I2S_MODE_MASTER |        \
                                             GD32_I2S_MODE_RX)
#endif

/**
 * @brief   I2S3 mode.
 */
#if !defined(GD32_I2S_SPI2_MODE) || defined(__DOXYGEN__)
#define GD32_I2S_SPI2_MODE                 (GD32_I2S_MODE_MASTER |        \
                                             GD32_I2S_MODE_RX)
#endif

/**
 * @brief   I2S2 interrupt priority level setting.
 */
#if !defined(GD32_I2S_SPI1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_I2S_SPI1_IRQ_PRIORITY         10
#endif

/**
 * @brief   I2S3 interrupt priority level setting.
 */
#if !defined(GD32_I2S_SPI2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_I2S_SPI2_IRQ_PRIORITY         10
#endif

/**
 * @brief   I2S2 DMA priority (0..3|lowest..highest).
 */
#if !defined(GD32_I2S_SPI1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define GD32_I2S_SPI1_DMA_PRIORITY         1
#endif

/**
 * @brief   I2S3 DMA priority (0..3|lowest..highest).
 */
#if !defined(GD32_I2S_SPI2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define GD32_I2S_SPI2_DMA_PRIORITY         1
#endif

/**
 * @brief   I2S DMA error hook.
 */
#if !defined(GD32_I2S_DMA_ERROR_HOOK) || defined(__DOXYGEN__)
#define GD32_I2S_DMA_ERROR_HOOK(i2sp)      osalSysHalt("DMA failure")
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if GD32_I2S_USE_SPI0
#error "SPI0 does not support I2S mode"
#endif

#if GD32_I2S_USE_SPI1 && !GD32_SPI1_SUPPORTS_I2S
#error "SPI1 does not support I2S mode"
#endif

#if GD32_I2S_USE_SPI2 && !GD32_SPI2_SUPPORTS_I2S
#error "SPI2 does not support I2S mode"
#endif

#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI1_MODE) &&                            \
    GD32_I2S_TX_ENABLED(GD32_I2S_SPI1_MODE)
#error "I2S2 RX and TX mode not supported in this driver implementation"
#endif

#if GD32_I2S_RX_ENABLED(GD32_I2S_SPI2_MODE) &&                            \
    GD32_I2S_TX_ENABLED(GD32_I2S_SPI2_MODE)
#error "I2S3 RX and TX mode not supported in this driver implementation"
#endif

#if GD32_I2S_USE_SPI1 && !GD32_HAS_SPI1
#error "SPI1 not present in the selected device"
#endif

#if GD32_I2S_USE_SPI2 && !GD32_HAS_SPI2
#error "SPI2 not present in the selected device"
#endif

#if !GD32_I2S_USE_SPI1 && !GD32_I2S_USE_SPI2
#error "I2S driver activated but no SPI peripheral assigned"
#endif

#if GD32_I2S_USE_SPI1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_I2S_SPI1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI1"
#endif

#if GD32_I2S_USE_SPI2 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(GD32_I2S_SPI2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPI2"
#endif

#if GD32_I2S_USE_SPI1 &&                                                   \
    !GD32_DMA_IS_VALID_PRIORITY(GD32_I2S_SPI1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI1"
#endif

#if GD32_I2S_USE_SPI2 &&                                                   \
    !GD32_DMA_IS_VALID_PRIORITY(GD32_I2S_SPI2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SPI2"
#endif

#if !defined(GD32_DMA_REQUIRED)
#define GD32_DMA_REQUIRED
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the I2S driver structure.
 */
#define i2s_lld_driver_fields                                               \
  /* Pointer to the SPIx registers block.*/                                 \
  SPI_TypeDef               *spi;                                           \
  /* Calculated part of the I2SCFGR register.*/                             \
  uint16_t                  ctl;                                            \
  /* Receive DMA stream or @p NULL.*/                                       \
  const gd32_dma_stream_t  *dmarx;                                          \
  /* Transmit DMA stream or @p NULL.*/                                      \
  const gd32_dma_stream_t  *dmatx;                                          \
  /* RX DMA mode bit mask.*/                                                \
  uint32_t                  rxdmamode;                                      \
  /* TX DMA mode bit mask.*/                                                \
  uint32_t                  txdmamode

/**
 * @brief   Low level fields of the I2S configuration structure.
 */
#define i2s_lld_config_fields                                               \
  /* Configuration of the I2SCTL register.                                 \
     NOTE: See the GD32 reference manual, this register is used for        \
           the I2S configuration, the following bits must not be            \
           specified because handled directly by the driver:                \
           - I2SMOD                                                         \
           - I2SE                                                           \
           - I2SCFG                                                         \
   */                                                                       \
  int16_t                   i2sctl;                                         \
  /* Configuration of the I2SPSC register.                                   \
     NOTE: See the GD32 reference manual, this register is used for        \
           the I2S clock setup.*/                                           \
  int16_t                   i2spsc

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if GD32_I2S_USE_SPI1 && !defined(__DOXYGEN__)
extern I2SDriver I2SD2;
#endif

#if GD32_I2S_USE_SPI2 && !defined(__DOXYGEN__)
extern I2SDriver I2SD3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void i2s_lld_init(void);
  void i2s_lld_start(I2SDriver *i2sp);
  void i2s_lld_stop(I2SDriver *i2sp);
  void i2s_lld_start_exchange(I2SDriver *i2sp);
  void i2s_lld_stop_exchange(I2SDriver *i2sp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2S */

#endif /* HAL_I2S_LLD_H */

/** @} */
