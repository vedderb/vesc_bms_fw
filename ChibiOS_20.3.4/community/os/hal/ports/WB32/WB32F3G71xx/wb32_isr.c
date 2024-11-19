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
 * @file    WB32F3G71xx/wb32_isr.c
 * @brief   WB32F3G71xx ISR handler code.
 *
 * @addtogroup WB32F3G71xx_ISR
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#define exti_serve_irq(pr, channel) {                                       \
    if ((pr) & (1U << (channel))) {                                         \
      _pal_isr_code(channel);                                               \
    }                                                                       \
  }

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (HAL_USE_PAL && (PAL_USE_WAIT || PAL_USE_CALLBACKS)) || defined(__DOXYGEN__)
#if !defined(WB32_DISABLE_EXTI0_HANDLER)
/**
 * @brief   EXTI[0] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_EXTI0_IRQ_VECTOR) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & EXTI_IMR_MR0;
  EXTI->PR = pr;

  exti_serve_irq(pr, 0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(WB32_DISABLE_EXTI1_HANDLER)
/**
 * @brief   EXTI[1] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_EXTI1_IRQ_VECTOR) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & EXTI_IMR_MR1;
  EXTI->PR = pr;

  exti_serve_irq(pr, 1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(WB32_DISABLE_EXTI2_HANDLER)
/**
 * @brief   EXTI[2] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_EXTI2_IRQ_VECTOR) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & EXTI_IMR_MR2;
  EXTI->PR = pr;

  exti_serve_irq(pr, 2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(WB32_DISABLE_EXTI3_HANDLER)
/**
 * @brief   EXTI[3] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_EXTI3_IRQ_VECTOR) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & EXTI_IMR_MR3;
  EXTI->PR = pr;

  exti_serve_irq(pr, 3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(WB32_DISABLE_EXTI4_HANDLER)
/**
 * @brief   EXTI[4] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_EXTI4_IRQ_VECTOR) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & EXTI_IMR_MR4;
  EXTI->PR = pr;

  exti_serve_irq(pr, 4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(WB32_DISABLE_EXTI9_5_HANDLER)
/**
 * @brief   EXTI[5]...EXTI[9] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_EXTI9_5_IRQ_VECTOR) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & (EXTI_IMR_MR5 | EXTI_IMR_MR6 | EXTI_IMR_MR7 | EXTI_IMR_MR8 |
                     EXTI_IMR_MR9);
  EXTI->PR = pr;

  exti_serve_irq(pr, 5);
  exti_serve_irq(pr, 6);
  exti_serve_irq(pr, 7);
  exti_serve_irq(pr, 8);
  exti_serve_irq(pr, 9);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(WB32_DISABLE_EXTI15_10_HANDLER)
/**
 * @brief   EXTI[10]...EXTI[15] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_EXTI15_10_IRQ_VECTOR) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR;
  pr &= EXTI->IMR & (EXTI_IMR_MR10 | EXTI_IMR_MR11 | EXTI_IMR_MR12 |
                     EXTI_IMR_MR13 | EXTI_IMR_MR14 | EXTI_IMR_MR15);
  EXTI->PR = pr;

  exti_serve_irq(pr, 10);
  exti_serve_irq(pr, 11);
  exti_serve_irq(pr, 12);
  exti_serve_irq(pr, 13);
  exti_serve_irq(pr, 14);
  exti_serve_irq(pr, 15);

  OSAL_IRQ_EPILOGUE();
}
#endif

#endif /* HAL_USE_PAL */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables IRQ sources.
 *
 * @notapi
 */
void irqInit(void) {

#if HAL_USE_PAL
  nvicEnableVector(EXTI0_IRQn, WB32_IRQ_EXTI0_PRIORITY);
  nvicEnableVector(EXTI1_IRQn, WB32_IRQ_EXTI1_PRIORITY);
  nvicEnableVector(EXTI2_IRQn, WB32_IRQ_EXTI2_PRIORITY);
  nvicEnableVector(EXTI3_IRQn, WB32_IRQ_EXTI3_PRIORITY);
  nvicEnableVector(EXTI4_IRQn, WB32_IRQ_EXTI4_PRIORITY);
  nvicEnableVector(EXTI9_5_IRQn, WB32_IRQ_EXTI5_9_PRIORITY);
  nvicEnableVector(EXTI15_10_IRQn, WB32_IRQ_EXTI10_15_PRIORITY);
#endif
}

/**
 * @brief   Disables IRQ sources.
 *
 * @notapi
 */
void irqDeinit(void) {

#if HAL_USE_PAL
  nvicDisableVector(EXTI0_IRQn);
  nvicDisableVector(EXTI1_IRQn);
  nvicDisableVector(EXTI2_IRQn);
  nvicDisableVector(EXTI3_IRQn);
  nvicDisableVector(EXTI4_IRQn);
  nvicDisableVector(EXTI9_5_IRQn);
  nvicDisableVector(EXTI15_10_IRQn);
#endif
}

/** @} */
