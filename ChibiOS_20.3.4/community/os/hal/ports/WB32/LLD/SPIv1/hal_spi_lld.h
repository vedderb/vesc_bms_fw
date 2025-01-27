/*
    Copyright (C) 2021 Westberry Technology (ChangZhou) Corp., Ltd

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
 * @file    SPIv1/hal_spi_lld.h
 * @brief   WB32 SPI subsystem low level driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef HAL_SPI_LLD_H
#define HAL_SPI_LLD_H

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Circular mode support flag.
 */
#define SPI_SUPPORTS_CIRCULAR                  FALSE

/** @defgroup SPI_Exported_Constants
  * @{
  */

/** @defgroup SPI_Clock_Polarity
  * @{
  */
#define SPI_CPOL_Low                           ((uint16_t)0x00)
#define SPI_CPOL_High                          ((uint16_t)0x80)
/**
  * @}
  */


/** @defgroup SPI_Clock_Phase
  * @{
  */
#define SPI_CPHA_1Edge                         ((uint16_t)0x00)
#define SPI_CPHA_2Edge                         ((uint16_t)0x40)
/**
  * @}
  */

/** @defgroup SPI_NSS_definition
  * @{
  */
#define SPI_NSS_0                              (0x01)
#define SPI_NSS_1                              (0x02)
#define SPI_NSS_2                              (0x04)
/**
  * @}
  */

/** @defgroup SPI_interrupts_definition
  * @{
  */
#define SPI_IT_TXE                             (0x1 << 0)
#define SPI_IT_TXO                             (0x1 << 1)
#define SPI_IT_RXU                             (0x1 << 2)
#define SPI_IT_RXO                             (0x1 << 3)
#define SPI_IT_RXF                             (0x1 << 4)
#define SPI_IT_MST                             (0x1 << 5)
/**
  * @}
  */

/**
  * @}
  */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   QSPI driver enable switch.
 * @details If set to @p TRUE the support for QSPI is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_SPI_USE_QSPI) || defined(__DOXYGEN__)
#define WB32_SPI_USE_QSPI                      FALSE
#endif

/**
 * @brief   SPIM2 driver enable switch.
 * @details If set to @p TRUE the support for SPIM2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_SPI_USE_SPIM2) || defined(__DOXYGEN__)
#define WB32_SPI_USE_SPIM2                     FALSE
#endif

/**
 * @brief   SPIS1 driver enable switch.
 * @details If set to @p TRUE the support for SPIS1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_SPI_USE_SPIS1) || defined(__DOXYGEN__)
#define WB32_SPI_USE_SPIS1                     FALSE
#endif

/**
 * @brief   SPIS2 driver enable switch.
 * @details If set to @p TRUE the support for SPIS2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(WB32_SPI_USE_SPIS2) || defined(__DOXYGEN__)
#define WB32_SPI_USE_SPIS2                     FALSE
#endif

/**
 * @brief   QSPI interrupt priority level setting.
 */
#if !defined(WB32_SPI_QSPI_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_SPI_QSPI_IRQ_PRIORITY             10
#endif

/**
 * @brief   SPIM2 interrupt priority level setting.
 */
#if !defined(WB32_SPI_SPIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_SPI_SPIM2_IRQ_PRIORITY            10
#endif

/**
 * @brief   SPIS1 interrupt priority level setting.
 */
#if !defined(WB32_SPI_SPIS1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_SPI_SPIS1_IRQ_PRIORITY            10
#endif

/**
 * @brief   SPIS2 interrupt priority level setting.
 */
#if !defined(WB32_SPI_SPIS2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define WB32_SPI_SPIS2_IRQ_PRIORITY            10
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if WB32_SPI_USE_QSPI && !WB32_HAS_QSPI
#error "QSPI not present in the selected device"
#endif

#if WB32_SPI_USE_SPIM2 && !WB32_HAS_SPIM2
#error "SPIM2 not present in the selected device"
#endif

#if WB32_SPI_USE_SPIS1 && !WB32_HAS_SPIS1
#error "SPIS1 not present in the selected device"
#endif

#if WB32_SPI_USE_SPIS2 && !WB32_HAS_SPIS2
#error "SPIS2 not present in the selected device"
#endif

#if !WB32_SPI_USE_QSPI && !WB32_SPI_USE_SPIM2 && !WB32_SPI_USE_SPIS1 &&     \
    !WB32_SPI_USE_SPIS2
#error "SPI driver activated but no SPI peripheral assigned"
#endif

#if WB32_SPI_USE_QSPI &&                                                    \
   !OSAL_IRQ_IS_VALID_PRIORITY(WB32_SPI_QSPI_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to QSPI"
#endif

#if WB32_SPI_USE_SPIM2 &&                                                   \
   !OSAL_IRQ_IS_VALID_PRIORITY(WB32_SPI_SPIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPIM2"
#endif

#if WB32_SPI_USE_SPIS1 &&                                                   \
   !OSAL_IRQ_IS_VALID_PRIORITY(WB32_SPI_SPIS1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPIS1"
#endif

#if WB32_SPI_USE_SPIS2 &&                                                   \
   !OSAL_IRQ_IS_VALID_PRIORITY(WB32_SPI_SPIS2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SPIS2"
#endif

#if SPI_SELECT_MODE == SPI_SELECT_MODE_LLD
#error "SPI_SELECT_MODE_LLD not supported by this driver"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef struct {
  const uint8_t             *tx_buf;
  volatile uint32_t         tx_len;
  uint8_t                   *rx_buf;
  volatile uint32_t         rx_len;
} spi_xfer_info_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the SPI driver structure.
 */
#define spi_lld_driver_fields                                               \
  /* Pointer to the SPIx registers block.*/                                 \
  SPI_TypeDef               *spi ;                                          \
  spi_xfer_info_t           xfer;                                           \
  uint16_t                  fifo_len

/**
 * @brief   Low level fields of the SPI configuration structure.
 */
#define spi_lld_config_fields                                               \
  /* SPI_Clock_Polarity. */                                                 \
  uint16_t                  SPI_CPOL;                                       \
  /* SPI_Clock_Phase. */                                                    \
  uint16_t                  SPI_CPHA;                                       \
  /* SPI In order to avoid TX FIFO underflow or */                          \
  /* RX FIFO overflow, SPI_BaudRatePrescaler needs to bigger. */            \
  uint16_t                  SPI_BaudRatePrescaler;                          \

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if WB32_SPI_USE_QSPI && !defined(__DOXYGEN__)
extern SPIDriver SPIDQ;
#endif

#if WB32_SPI_USE_SPIM2 && !defined(__DOXYGEN__)
extern SPIDriver SPIDM2;
#endif

#if WB32_SPI_USE_SPIS1 && !defined(__DOXYGEN__)
extern SPIDriver SPIDS1;
#endif

#if WB32_SPI_USE_SPIS2 && !defined(__DOXYGEN__)
extern SPIDriver SPIDS2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
    void spi_lld_init(void);
    void spi_lld_start(SPIDriver *spip);
    void spi_lld_stop(SPIDriver *spip);
#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
    void spi_lld_select(SPIDriver *spip);
    void spi_lld_unselect(SPIDriver *spip);
#endif
    void spi_lld_ignore(SPIDriver *spip, size_t n);
    void spi_lld_exchange(SPIDriver *spip, size_t n,
                          const void *txbuf, void *rxbuf);
    void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf);
    void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf);
#if (SPI_SUPPORTS_CIRCULAR == TRUE) || defined(__DOXYGEN__)
    void spi_lld_abort(SPIDriver *spip);
#endif
    uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* HAL_SPI_LLD_H */

/** @} */
