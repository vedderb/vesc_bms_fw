/*
    Copyright (C) 2020 Yaotian Feng

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
 * @file    hal_gpt_lld.c
 * @brief   PLATFORM GPT subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "hal.h"

#if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   GPTD1 driver identifier.
 */
#if (HT32_GPT_USE_BFTM0 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD_BFTM0;
#endif

#if (HT32_GPT_USE_BFTM1 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD_BFTM1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void gpt_lld_handler(GPTDriver *gptp) {
    if(gptp->BFTM->SR & BFTM_SR_MIF){
        gptp->BFTM->SR = 0;
        if(gptp->config->callback)
            gptp->config->callback(gptp);
    }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (HT32_GPT_USE_BFTM0 == TRUE) || defined(__DOXYGEN__)
#ifndef HT32_BFTM0_IRQ_VECTOR
#error "HT32_BFTM0_IRQ_VECTOR is not defined"
#endif
OSAL_IRQ_HANDLER(HT32_BFTM0_IRQ_VECTOR) {
    OSAL_IRQ_PROLOGUE();
    gpt_lld_handler(&GPTD_BFTM0);
    OSAL_IRQ_EPILOGUE();
}
#endif

#if (HT32_GPT_USE_BFTM1 == TRUE) || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(HT32_BFTM1_IRQ_VECTOR) {
    OSAL_IRQ_PROLOGUE();
    gpt_lld_handler(&GPTD_BFTM1);
    OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {
    /* Driver initialization.*/
#if HT32_GPT_USE_BFTM0 == TRUE
    gptObjectInit(&GPTD_BFTM0);
    GPTD_BFTM0.BFTM = BFTM0;
#endif
#if HT32_GPT_USE_BFTM1 == TRUE
    gptObjectInit(&GPTD_BFTM1);
    GPTD_BFTM0.BFTM = BFTM1;
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {
    if (gptp->state == GPT_STOP) {
        /* Enables the peripheral.*/
#if HT32_GPT_USE_BFTM0 == TRUE
        if (&GPTD_BFTM0 == gptp) {
            CKCU->APBCCR1 |= CKCU_APBCCR1_BFTM0EN;
            nvicEnableVector(BFTM0_IRQn, HT32_GPT_BFTM0_IRQ_PRIORITY);
        }
#endif
#if HT32_GPT_USE_BFTM1 == TRUE
        if (&GPTD_BFTM1 == gptp) {
            CKCU->APBCCR1 |= CKCU_APBCCR1_BFTM1EN;
            nvicEnableVector(BFTM1_IRQn, HT32_GPT_BFTM1_IRQ_PRIORITY);
        }
#endif
    }

    /* Configures the peripheral.*/
    gptp->BFTM->CR = 0;
    // counter frequency depends on the AHB clock, we can't
    // change anything here
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {
    if (gptp->state == GPT_READY) {
        /* Resets the peripheral.*/
        /* Disables the peripheral.*/
#if HT32_GPT_USE_BFTM0 == TRUE
        if (&GPTD_BFTM0 == gptp) {
            RSTCU->APBPRSTR1 = RSTCU_APBPRSTR1_BFTM0RST;
            CKCU->APBCCR1 &= ~CKCU_APBCCR1_BFTM0EN;
            nvicDisableVector(BFTM0_IRQn);
        }
#endif
#if HT32_GPT_USE_BFTM1 == TRUE
        if (&GPTD_BFTM1 == gptp) {
            RSTCU->APBPRSTR1 = RSTCU_APBPRSTR1_BFTM1RST;
            CKCU->APBCCR1 &= ~CKCU_APBCCR1_BFTM1EN;
            nvicDisableVector(BFTM1_IRQn);
        }
#endif
    }
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval) {
    gptp->BFTM->SR = 0;
    gptp->BFTM->CNTR = 0;
    gptp->BFTM->CMP = (HT32_CK_AHB_FREQUENCY / gptp->config->frequency) * interval;
    gptp->BFTM->CR = BFTM_CR_CEN | BFTM_CR_MIEN;
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {
    gptp->BFTM->CR = 0;
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {
    gptp->BFTM->SR = 0;
    gptp->BFTM->CNTR = 0;
    gptp->BFTM->CMP = (HT32_CK_AHB_FREQUENCY / gptp->config->frequency) * interval;
    gptp->BFTM->CR = BFTM_CR_CEN | BFTM_CR_OSM;
    while(!(gptp->BFTM->SR & BFTM_SR_MIF));
}

#endif /* HAL_USE_GPT == TRUE */

/** @} */
