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
 * @file    DMA/gd32_dma.h
 * @brief   DMA helper driver header.

 * @addtogroup GD32_DMA
 * @{
 */

#ifndef GD32_DMA_H
#define GD32_DMA_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Total number of DMA streams.
 * @details This is the total number of streams among all the DMA units.
 */
#define GD32_DMA_STREAMS           (GD32_DMA0_NUM_CHANNELS +              \
                                     GD32_DMA1_NUM_CHANNELS)

/**
 * @brief   Mask of the ISR bits passed to the DMA callback functions.
 */
#define GD32_DMA_INTF_MASK          0x0E

/**
 * @brief   Returns the request line associated to the specified stream.
 * @note    In some GD32 manuals the request line is named confusingly
 *          channel.
 *
 * @param[in] id        the unique numeric stream identifier
 * @param[in] c         a stream/request association word, one request per
 *                      nibble
 * @return              Returns the request associated to the stream.
 */
#define GD32_DMA_GETCHANNEL(id, c)                                         \
  (((uint32_t)(c) >> (((uint32_t)(id) % (uint32_t)GD32_DMA0_NUM_CHANNELS) * 4U)) & 15U)

/**
 * @brief   Checks if a DMA priority is within the valid range.
 * @param[in] prio      DMA priority
 *
 * @retval              The check result.
 * @retval false        invalid DMA priority.
 * @retval true         correct DMA priority.
 */
#define GD32_DMA_IS_VALID_PRIORITY(prio) (((prio) >= 0U) && ((prio) <= 3U))

/**
 * @brief   Checks if a DMA stream id is within the valid range.
 *
 * @param[in] id        DMA stream id
 * @retval              The check result.
 * @retval false        invalid DMA channel.
 * @retval true         correct DMA channel.
 */
#define GD32_DMA_IS_VALID_STREAM(id) (((id) >= 0U) &&                      \
                                       ((id) < GD32_DMA_STREAMS))

/**
 * @brief   Returns an unique numeric identifier for a DMA stream.
 *
 * @param[in] dma       the DMA unit number
 * @param[in] stream    the stream number
 * @return              An unique numeric stream identifier.
 */
#define GD32_DMA_STREAM_ID(dma, stream)                                    \
  ((((dma)) * GD32_DMA0_NUM_CHANNELS) + ((stream)))

/**
 * @brief   Returns a DMA stream identifier mask.
 *
 *
 * @param[in] dma       the DMA unit number
 * @param[in] stream    the stream number
 * @return              A DMA stream identifier mask.
 */
#define GD32_DMA_STREAM_ID_MSK(dma, stream)                                \
  (1U << GD32_DMA_STREAM_ID(dma, stream))

/**
 * @brief   Checks if a DMA stream unique identifier belongs to a mask.
 *
 * @param[in] id        the stream numeric identifier
 * @param[in] mask      the stream numeric identifiers mask
 *
 * @retval              The check result.
 * @retval false        id does not belong to the mask.
 * @retval true         id belongs to the mask.
 */
#define GD32_DMA_IS_VALID_ID(id, mask) (((1U << (id)) & (mask)))

/**
 * @name    DMA streams identifiers
 * @{
 */
/**
 * @brief   Returns a pointer to a gd32_dma_stream_t structure.
 *
 * @param[in] id        the stream numeric identifier
 * @return              A pointer to the gd32_dma_stream_t constant structure
 *                      associated to the DMA stream.
 */
#define GD32_DMA_STREAM(id)        (&_gd32_dma_streams[id])

#define GD32_DMA0_STREAM0          GD32_DMA_STREAM(0)
#define GD32_DMA0_STREAM1          GD32_DMA_STREAM(1)
#define GD32_DMA0_STREAM2          GD32_DMA_STREAM(2)
#define GD32_DMA0_STREAM3          GD32_DMA_STREAM(3)
#define GD32_DMA0_STREAM4          GD32_DMA_STREAM(4)
#define GD32_DMA0_STREAM5          GD32_DMA_STREAM(5)
#define GD32_DMA0_STREAM6          GD32_DMA_STREAM(6)
#define GD32_DMA1_STREAM0          GD32_DMA_STREAM(GD32_DMA0_NUM_CHANNELS + 0)
#define GD32_DMA1_STREAM1          GD32_DMA_STREAM(GD32_DMA0_NUM_CHANNELS + 1)
#define GD32_DMA1_STREAM2          GD32_DMA_STREAM(GD32_DMA0_NUM_CHANNELS + 2)
#define GD32_DMA1_STREAM3          GD32_DMA_STREAM(GD32_DMA0_NUM_CHANNELS + 3)
#define GD32_DMA1_STREAM4          GD32_DMA_STREAM(GD32_DMA0_NUM_CHANNELS + 4)
/** @} */

/**
 * @name    CR register constants common to all DMA types
 * @{
 */
#define GD32_DMA_CTL_RESET_VALUE   0x00000000U
#define GD32_DMA_CTL_EN             DMA_CTL_CHEN
#define GD32_DMA_CTL_ERRIE           DMA_CTL_ERRIE
#define GD32_DMA_CTL_HTFIE           DMA_CTL_HTFIE
#define GD32_DMA_CTL_FTFIE           DMA_CTL_FTFIE
#define GD32_DMA_CTL_DIR_MASK       (DMA_CTL_DIR | DMA_CTL_M2M)
#define GD32_DMA_CTL_DIR_P2M        0U
#define GD32_DMA_CTL_DIR_M2P        DMA_CTL_DIR
#define GD32_DMA_CTL_DIR_M2M        DMA_CTL_M2M
#define GD32_DMA_CTL_CMEN           DMA_CTL_CMEN
#define GD32_DMA_CTL_PNAGA           DMA_CTL_PNAGA
#define GD32_DMA_CTL_MNAGA           DMA_CTL_MNAGA
#define GD32_DMA_CTL_PWIDTH_MASK     DMA_CTL_PWIDTH
#define GD32_DMA_CTL_PWIDTH_BYTE     0U
#define GD32_DMA_CTL_PWIDTH_HWORD    DMA_CTL_PWIDTH_0
#define GD32_DMA_CTL_PWIDTH_WORD     DMA_CTL_PWIDTH_1
#define GD32_DMA_CTL_MWIDTH_MASK     DMA_CTL_MWIDTH
#define GD32_DMA_CTL_MWIDTH_BYTE     0U
#define GD32_DMA_CTL_MWIDTH_HWORD    DMA_CTL_MWIDTH_0
#define GD32_DMA_CTL_MWIDTH_WORD     DMA_CTL_MWIDTH_1
#define GD32_DMA_CTL_SIZE_MASK      (GD32_DMA_CTL_PWIDTH_MASK |              \
                                     GD32_DMA_CTL_MWIDTH_MASK)
#define GD32_DMA_CTL_PRIO_MASK        DMA_CTL_PRIO
#define GD32_DMA_CTL_PRIO(n)          ((n) << 12U)
/** @} */

/**
 * @name    Request line selector macro
 * @{
 */
#define GD32_DMA_CTL_CHSEL_MASK     0U
#define GD32_DMA_CTL_CHSEL(n)       0U
/** @} */

/**
 * @name    Status flags passed to the ISR callbacks
 * @{
 */
#define GD32_DMA_INTF_FEIF          0U
#define GD32_DMA_INTF_ERRIF          DMA_INTF_ERRIF0
#define GD32_DMA_INTF_HTFIF          DMA_INTF_HTFIF0
#define GD32_DMA_INTF_FTFIF          DMA_INTF_FTFIF0
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(GD32_DMA0_NUM_CHANNELS)
#error "GD32_DMA0_NUM_CHANNELS not defined in registry"
#endif

#if !defined(GD32_DMA1_NUM_CHANNELS)
#error "GD32_DMA1_NUM_CHANNELS not defined in registry"
#endif

#if (GD32_DMA0_NUM_CHANNELS < 0) || (GD32_DMA0_NUM_CHANNELS > 7)
#error "unsupported channels configuration"
#endif

#if (GD32_DMA1_NUM_CHANNELS < 0) || (GD32_DMA1_NUM_CHANNELS > 5)
#error "unsupported channels configuration"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a DMA callback.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] flags     pre-shifted content of the ISR register, the bits
 *                      are aligned to bit zero
 */
typedef void (*gd32_dmaisr_t)(void *p, uint32_t flags);

/**
 * @brief   GD32 DMA stream descriptor structure.
 */
typedef struct {
  DMA_TypeDef           *dma;           /**< @brief Associated DMA.         */
  DMA_Channel_TypeDef   *channel;       /**< @brief Associated DMA channel. */
  uint32_t              cmask;          /**< @brief Mask of streams sharing
                                             the same ISR.                  */
  uint8_t               dummy;          /**< @brief Filler.                 */
  uint8_t               shift;          /**< @brief Bit offset in ISR, IFCR
                                             and CSELR registers.           */
  uint8_t               selfindex;      /**< @brief Index to self in array. */
  uint8_t               vector;         /**< @brief Associated IRQ vector.  */
} gd32_dma_stream_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Associates a peripheral data register to a DMA stream.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 * @param[in] addr      value to be written in the PADDR register
 *
 * @special
 */
#define dmaStreamSetPeripheral(dmastp, addr) {                              \
  (dmastp)->channel->PADDR = (uint32_t)(addr);                               \
}

/**
 * @brief   Associates a memory destination to a DMA stream.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 * @param[in] addr      value to be written in the MADDR register
 *
 * @special
 */
#define dmaStreamSetMemory0(dmastp, addr) {                                 \
  (dmastp)->channel->MADDR = (uint32_t)(addr);                               \
}

/**
 * @brief   Sets the number of transfers to be performed.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 * @param[in] size      value to be written in the CNT register
 *
 * @special
 */
#define dmaStreamSetTransactionSize(dmastp, size) {                         \
  (dmastp)->channel->CNT = (uint32_t)(size);                              \
}

/**
 * @brief   Returns the number of transfers to be performed.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 * @return              The number of transfers to be performed.
 *
 * @special
 */
#define dmaStreamGetTransactionSize(dmastp) ((size_t)((dmastp)->channel->CNT))

/**
 * @brief   Programs the stream mode settings.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 * @param[in] mode      value to be written in the CCR register
 *
 * @special
 */
#define dmaStreamSetMode(dmastp, mode) {                                    \
  (dmastp)->channel->CTL  = (uint32_t)(mode);                               \
}

/**
 * @brief   DMA stream enable.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 *
 * @special
 */
#define dmaStreamEnable(dmastp) {                                           \
  (dmastp)->channel->CTL |= GD32_DMA_CTL_EN;                                \
}

/**
 * @brief   DMA stream disable.
 * @details The function disables the specified stream and then clears any
 *          pending interrupt.
 * @note    This function can be invoked in both ISR or thread context.
 * @note    Interrupts enabling flags are set to zero after this call, see
 *          bug 3607518.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 *
 * @special
 */
#define dmaStreamDisable(dmastp) {                                          \
  (dmastp)->channel->CTL &= ~(GD32_DMA_CTL_FTFIE | GD32_DMA_CTL_HTFIE |       \
                              GD32_DMA_CTL_ERRIE | GD32_DMA_CTL_EN);         \
  dmaStreamClearInterrupt(dmastp);                                          \
}

/**
 * @brief   DMA stream interrupt sources clear.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 *
 * @special
 */
#define dmaStreamClearInterrupt(dmastp) {                                   \
  (dmastp)->dma->INTC = GD32_DMA_INTF_MASK << (dmastp)->shift;              \
}

/**
 * @brief   Starts a memory to memory operation using the specified stream.
 * @note    The default transfer data mode is "byte to byte" but it can be
 *          changed by specifying extra options in the @p mode parameter.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 * @param[in] mode      value to be written in the CCR register, this value
 *                      is implicitly ORed with:
 *                      - @p GD32_DMA_CTL_MNAGA
 *                      - @p GD32_DMA_CTL_PNAGA
 *                      - @p GD32_DMA_CTL_DIR_M2M
 *                      - @p GD32_DMA_CTL_EN
 *                      .
 * @param[in] src       source address
 * @param[in] dst       destination address
 * @param[in] n         number of data units to copy
 */
#define dmaStartMemCopy(dmastp, mode, src, dst, n) {                        \
  dmaStreamSetPeripheral(dmastp, src);                                      \
  dmaStreamSetMemory0(dmastp, dst);                                         \
  dmaStreamSetTransactionSize(dmastp, n);                                   \
  dmaStreamSetMode(dmastp, (mode) |                                         \
                           GD32_DMA_CTL_MNAGA | GD32_DMA_CTL_PNAGA |          \
                           GD32_DMA_CTL_DIR_M2M | GD32_DMA_CTL_EN);         \
}

/**
 * @brief   Polled wait for DMA transfer end.
 * @pre     The stream must have been allocated using @p dmaStreamAlloc().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 */
#define dmaWaitCompletion(dmastp) {                                         \
  while ((dmastp)->channel->CNT > 0U)                                     \
    ;                                                                       \
  dmaStreamDisable(dmastp);                                                 \
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern const gd32_dma_stream_t _gd32_dma_streams[GD32_DMA_STREAMS];
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void dmaInit(void);
  const gd32_dma_stream_t *dmaStreamAllocI(uint32_t id,
                                            uint32_t priority,
                                            gd32_dmaisr_t func,
                                            void *param);
  const gd32_dma_stream_t *dmaStreamAlloc(uint32_t id,
                                           uint32_t priority,
                                           gd32_dmaisr_t func,
                                           void *param);
  void dmaStreamFreeI(const gd32_dma_stream_t *dmastp);
  void dmaStreamFree(const gd32_dma_stream_t *dmastp);
  void dmaServeInterrupt(const gd32_dma_stream_t *dmastp);
#ifdef __cplusplus
}
#endif

#endif /* GD32_DMA_H */

/** @} */
