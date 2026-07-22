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
 * @file    DMA/gd32_dma.c
 * @brief   DMA helper driver code.
 *
 * @addtogroup GD32_DMA
 * @details DMA sharing helper driver. In the GD32 the DMA streams are a
 *          shared resource, this driver allows to allocate and free DMA
 *          streams at runtime in order to allow all the other device
 *          drivers to coordinate the access to the resource.
 * @note    The DMA ISR handlers are all declared into this module because
 *          sharing, the various device drivers can associate a callback to
 *          ISRs when allocating streams.
 * @{
 */

#include "hal.h"

/* The following macro is only defined if some driver requiring DMA services
   has been enabled.*/
#if defined(GD32_DMA_REQUIRED) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Mask of the DMA0 streams in @p dma_streams_mask.
 */
#define GD32_DMA0_STREAMS_MASK     ((1U << GD32_DMA0_NUM_CHANNELS) - 1U)

/**
 * @brief   Mask of the DMA1 streams in @p dma_streams_mask.
 */
#define GD32_DMA1_STREAMS_MASK     (((1U << GD32_DMA1_NUM_CHANNELS) -     \
                                      1U) << GD32_DMA0_NUM_CHANNELS)

#define DMA0_CH0_VARIANT            0
#define DMA0_CH1_VARIANT            0
#define DMA0_CH2_VARIANT            0
#define DMA0_CH3_VARIANT            0
#define DMA0_CH4_VARIANT            0
#define DMA0_CH5_VARIANT            0
#define DMA0_CH6_VARIANT            0
#define DMA1_CH0_VARIANT            0
#define DMA1_CH1_VARIANT            0
#define DMA1_CH2_VARIANT            0
#define DMA1_CH3_VARIANT            0
#define DMA1_CH4_VARIANT            0

/*
 * Default ISR collision masks.
 */
#if !defined(GD32_DMA0_CH0_CMASK)
#define GD32_DMA0_CH0_CMASK        (1U << 0U)
#endif

#if !defined(GD32_DMA0_CH1_CMASK)
#define GD32_DMA0_CH1_CMASK        (1U << 1U)
#endif

#if !defined(GD32_DMA0_CH2_CMASK)
#define GD32_DMA0_CH2_CMASK        (1U << 2U)
#endif

#if !defined(GD32_DMA0_CH3_CMASK)
#define GD32_DMA0_CH3_CMASK        (1U << 3U)
#endif

#if !defined(GD32_DMA0_CH4_CMASK)
#define GD32_DMA0_CH4_CMASK        (1U << 4U)
#endif

#if !defined(GD32_DMA0_CH5_CMASK)
#define GD32_DMA0_CH5_CMASK        (1U << 5U)
#endif

#if !defined(GD32_DMA0_CH6_CMASK)
#define GD32_DMA0_CH6_CMASK        (1U << 6U)
#endif

#if !defined(GD32_DMA1_CH0_CMASK)
#define GD32_DMA1_CH0_CMASK        (1U << (GD32_DMA0_NUM_CHANNELS + 0U))
#endif

#if !defined(GD32_DMA1_CH1_CMASK)
#define GD32_DMA1_CH1_CMASK        (1U << (GD32_DMA0_NUM_CHANNELS + 1U))
#endif

#if !defined(GD32_DMA1_CH2_CMASK)
#define GD32_DMA1_CH2_CMASK        (1U << (GD32_DMA0_NUM_CHANNELS + 2U))
#endif

#if !defined(GD32_DMA1_CH3_CMASK)
#define GD32_DMA1_CH3_CMASK        (1U << (GD32_DMA0_NUM_CHANNELS + 3U))
#endif

#if !defined(GD32_DMA1_CH4_CMASK)
#define GD32_DMA1_CH4_CMASK        (1U << (GD32_DMA0_NUM_CHANNELS + 4U))
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   DMA streams descriptors.
 * @details This table keeps the association between an unique stream
 *          identifier and the involved physical registers.
 * @note    Don't use this array directly, use the appropriate wrapper macros
 *          instead: @p GD32_DMA0_STREAM0, @p GD32_DMA0_STREAM1 etc.
 */
const gd32_dma_stream_t _gd32_dma_streams[GD32_DMA_STREAMS] = {
  {DMA0, DMA0_Channel0, GD32_DMA0_CH0_CMASK, DMA0_CH0_VARIANT,  0, 0, GD32_DMA0_CH0_NUMBER},
  {DMA0, DMA0_Channel1, GD32_DMA0_CH1_CMASK, DMA0_CH1_VARIANT,  4, 1, GD32_DMA0_CH1_NUMBER},
  {DMA0, DMA0_Channel2, GD32_DMA0_CH2_CMASK, DMA0_CH2_VARIANT,  8, 2, GD32_DMA0_CH2_NUMBER},
  {DMA0, DMA0_Channel3, GD32_DMA0_CH3_CMASK, DMA0_CH3_VARIANT, 12, 3, GD32_DMA0_CH3_NUMBER},
  {DMA0, DMA0_Channel4, GD32_DMA0_CH4_CMASK, DMA0_CH4_VARIANT, 16, 4, GD32_DMA0_CH4_NUMBER},
  {DMA0, DMA0_Channel5, GD32_DMA0_CH5_CMASK, DMA0_CH5_VARIANT, 20, 5, GD32_DMA0_CH5_NUMBER},
  {DMA0, DMA0_Channel6, GD32_DMA0_CH6_CMASK, DMA0_CH6_VARIANT, 24, 6, GD32_DMA0_CH6_NUMBER},
  {DMA1, DMA1_Channel0, GD32_DMA1_CH0_CMASK, DMA1_CH0_VARIANT,  0, 0 + GD32_DMA0_NUM_CHANNELS, GD32_DMA1_CH0_NUMBER},
  {DMA1, DMA1_Channel1, GD32_DMA1_CH1_CMASK, DMA1_CH1_VARIANT,  4, 1 + GD32_DMA0_NUM_CHANNELS, GD32_DMA1_CH1_NUMBER},
  {DMA1, DMA1_Channel2, GD32_DMA1_CH2_CMASK, DMA1_CH2_VARIANT,  8, 2 + GD32_DMA0_NUM_CHANNELS, GD32_DMA1_CH2_NUMBER},
  {DMA1, DMA1_Channel3, GD32_DMA1_CH3_CMASK, DMA1_CH3_VARIANT, 12, 3 + GD32_DMA0_NUM_CHANNELS, GD32_DMA1_CH3_NUMBER},
  {DMA1, DMA1_Channel4, GD32_DMA1_CH4_CMASK, DMA1_CH4_VARIANT, 16, 4 + GD32_DMA0_NUM_CHANNELS, GD32_DMA1_CH4_NUMBER},
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Global DMA-related data structures.
 */
static struct {
  /**
   * @brief   Mask of the allocated streams.
   */
  uint32_t          allocated_mask;
  /**
   * @brief   Mask of the enabled streams ISRs.
   */
  uint32_t          isr_mask;
  /**
   * @brief   DMA IRQ redirectors.
   */
  struct {
    /**
     * @brief   DMA callback function.
     */
    gd32_dmaisr_t    func;
    /**
     * @brief   DMA callback parameter.
     */
    void              *param;
  } streams[GD32_DMA_STREAMS];
} dma;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(GD32_DMA0_CH0_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA0 stream 0 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA0_CH0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA0_STREAM0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA0_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA0 stream 1 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA0_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA0_STREAM1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA0_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA0 stream 2 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA0_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA0_STREAM2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA0_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA0 stream 3 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA0_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA0_STREAM3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA0_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA0 stream 4 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA0_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA0_STREAM4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA0_CH5_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA0 stream 5 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA0_CH5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA0_STREAM5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA0_CH6_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA0 stream 6 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA0_CH6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA0_STREAM6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA1_CH0_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 0 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA1_CH0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA1_STREAM0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA1_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 1 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA1_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA1_STREAM1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA1_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 2 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA1_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA1_STREAM2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA1_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 3 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA1_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA1_STREAM3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(GD32_DMA1_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 4 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_DMA1_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(GD32_DMA1_STREAM4);

  OSAL_IRQ_EPILOGUE();
}
#endif


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   GD32 DMA helper initialization.
 *
 * @init
 */
void dmaInit(void) {
  int i;

  dma.allocated_mask = 0U;
  dma.isr_mask       = 0U;
  for (i = 0; i < GD32_DMA_STREAMS; i++) {
    _gd32_dma_streams[i].channel->CTL = GD32_DMA_CTL_RESET_VALUE;
    dma.streams[i].func = NULL;
  }
  DMA0->INTC = 0xFFFFFFFFU;
  DMA1->INTC = 0xFFFFFFFFU;
}

/**
 * @brief   Allocates a DMA stream.
 * @details The stream is allocated and, if required, the DMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p GD32_DMA_STREAM_ID_ANY for any stream.
 *                      - @p GD32_DMA_STREAM_ID_ANY_DMA0 for any stream
 *                        on DMA0.
 *                      - @p GD32_DMA_STREAM_ID_ANY_DMA1 for any stream
 *                        on DMA1.
 *                      .
 * @param[in] priority  IRQ priority for the DMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p gd32_dma_stream_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @iclass
 */
const gd32_dma_stream_t *dmaStreamAllocI(uint32_t id,
                                          uint32_t priority,
                                          gd32_dmaisr_t func,
                                          void *param) {
  uint32_t i, startid, endid;

  osalDbgCheckClassI();

  if (id < GD32_DMA_STREAMS) {
    startid = id;
    endid   = id;
  } else {
    osalDbgCheck(false);
    return NULL;
  }

  for (i = startid; i <= endid; i++) {
    uint32_t mask = (1U << i);
    if ((dma.allocated_mask & mask) == 0U) {
      const gd32_dma_stream_t *dmastp = GD32_DMA_STREAM(i);

      /* Installs the DMA handler.*/
      dma.streams[i].func  = func;
      dma.streams[i].param = param;
      dma.allocated_mask  |= mask;

      /* Enabling DMA clocks required by the current streams set.*/
      if ((GD32_DMA0_STREAMS_MASK & mask) != 0U) {
        rcuEnableDMA0(true);
      }

      if ((GD32_DMA1_STREAMS_MASK & mask) != 0U) {
        rcuEnableDMA1(true);
      }

      /* Enables the associated IRQ vector if not already enabled and if a
         callback is defined.*/
      if (func != NULL) {
        if ((dma.isr_mask & dmastp->cmask) == 0U) {
          eclicEnableVector(dmastp->vector, priority, ECLIC_DMA_TRIGGER);
        }
        dma.isr_mask |= mask;
      }

      /* Putting the stream in a known state.*/
      dmaStreamDisable(dmastp);
      dmastp->channel->CTL = GD32_DMA_CTL_RESET_VALUE;

      return dmastp;
    }
  }

  return NULL;
}

/**
 * @brief   Allocates a DMA stream.
 * @details The stream is allocated and, if required, the DMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p GD32_DMA_STREAM_ID_ANY for any stream.
 *                      - @p GD32_DMA_STREAM_ID_ANY_DMA0 for any stream
 *                        on DMA0.
 *                      - @p GD32_DMA_STREAM_ID_ANY_DMA1 for any stream
 *                        on DMA1.
 *                      .
 * @param[in] priority  IRQ priority for the DMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p gd32_dma_stream_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @api
 */
const gd32_dma_stream_t *dmaStreamAlloc(uint32_t id,
                                         uint32_t priority,
                                         gd32_dmaisr_t func,
                                         void *param) {
  const gd32_dma_stream_t *dmastp;

  osalSysLock();
  dmastp = dmaStreamAllocI(id, priority, func, param);
  osalSysUnlock();

  return dmastp;
}

/**
 * @brief   Releases a DMA stream.
 * @details The stream is freed and, if required, the DMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 *
 * @iclass
 */
void dmaStreamFreeI(const gd32_dma_stream_t *dmastp) {
  uint32_t selfindex = (uint32_t)dmastp->selfindex;

  osalDbgCheck(dmastp != NULL);

  /* Check if the streams is not taken.*/
  osalDbgAssert((dma.allocated_mask & (1 << selfindex)) != 0U,
                "not allocated");

  /* Marks the stream as not allocated.*/
  dma.allocated_mask &= ~(1U << selfindex);
  dma.isr_mask &= ~(1U << selfindex);

  /* Disables the associated IRQ vector if it is no more in use.*/
  if ((dma.isr_mask & dmastp->cmask) == 0U) {
    eclicDisableVector(dmastp->vector);
  }

  /* Removes the DMA handler.*/
  dma.streams[selfindex].func  = NULL;
  dma.streams[selfindex].param = NULL;

  /* Shutting down clocks that are no more required, if any.*/
  if ((dma.allocated_mask & GD32_DMA0_STREAMS_MASK) == 0U) {
    rcuDisableDMA0();
  }
  if ((dma.allocated_mask & GD32_DMA1_STREAMS_MASK) == 0U) {
    rcuDisableDMA1();
  }
}

/**
 * @brief   Releases a DMA stream.
 * @details The stream is freed and, if required, the DMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 *
 * @api
 */
void dmaStreamFree(const gd32_dma_stream_t *dmastp) {

  osalSysLock();
  dmaStreamFreeI(dmastp);
  osalSysUnlock();
}

/**
 * @brief   Serves a DMA IRQ.
 *
 * @param[in] dmastp    pointer to a gd32_dma_stream_t structure
 *
 * @special
 */
void dmaServeInterrupt(const gd32_dma_stream_t *dmastp) {
  uint32_t flags;
  uint32_t selfindex = (uint32_t)dmastp->selfindex;

  flags = (dmastp->dma->INTF >> dmastp->shift) & GD32_DMA_INTF_MASK;
  if (flags & dmastp->channel->CTL) {
    dmastp->dma->INTC = flags << dmastp->shift;
    if (dma.streams[selfindex].func) {
      dma.streams[selfindex].func(dma.streams[selfindex].param, flags);
    }
  }
}

#endif /* GD32_DMA_REQUIRED */

/** @} */
