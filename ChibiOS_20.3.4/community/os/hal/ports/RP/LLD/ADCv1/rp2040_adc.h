/*
    ChibiOS - Copyright (C) 2006..2021 Giovanni Di Sirio

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
 * @file    ADCv1/rp2040_adc.h
 * @brief   RP2040 ADC registers layout header.
 * @note    This file requires definitions from the RP2040 header files.
 *
 * @addtogroup ADC
 * @{
 */

#ifndef RP2040_ADC_H
#define RP2040_ADC_H

#include "rp2040_adc.h"

#define ADC_CS_RROBIN_Pos       16U
#define ADC_CS_RROBIN_Msk       (0x1F << ADC_CS_RROBIN_Pos)
#define ADC_CS_AINSEL_Pos       12U
#define ADC_CS_AINSEL_Msk       (0x7 << ADC_CS_AINSEL_Pos)
#define ADC_CS_ERR_STICKY       (1U << 10)
#define ADC_CS_ERR              (1U << 9)
#define ADC_CS_READY            (1U << 8)
#define ADC_CS_START_MANY       (1U << 3)
#define ADC_CS_START_ONCE       (1U << 2)
#define ADC_CS_TS_EN            (1U << 1)
#define ADC_CS_EN               (1U << 0)

#define ADC_RESULT_Pos          0U
#define ADC_RESULT_Msk          (0xFFF << ADC_RESULT_Pos)

#define ADC_FCS_THRESH_Pos      24U
#define ADC_FCS_THRESH_Msk      (0xF << ADC_FCS_THRESH_Pos)
#define ADC_FCS_LEVEL_Pos       16U
#define ADC_FCS_LEVEL_Msk       (0xF << ADC_FCS_LEVEL_Pos)
#define ADC_FCS_OVER            (1U << 11)
#define ADC_FCS_UNDER           (1U << 10)
#define ADC_FCS_FULL            (1U << 9)
#define ADC_FCS_EMPTY           (1U << 8)
#define ADC_FCS_DREQ_EN         (1U << 3)
#define ADC_FCS_ERR             (1U << 2)
#define ADC_FCS_SHIFT           (1U << 1)
#define ADC_FCS_EN              (1U << 0)

#define ADC_FIFO_ERR            (1U << 15U)
#define ADC_FIFO_VAL_Pos        0U
#define ADC_FIFO_VAL_Msk        (0xFFF << ADC_FIFO_VAL_Pos)

#define ADC_DIV_INT_Pos         8U
#define ADC_DIV_INT_Msk         (0xFFFF << ADC_DIV_INT_Pos)
#define ADC_DIV_FRAC_Pos        0U
#define ADC_DIV_FRAC_Msk        (0xFF << ADC_DIV_FRAC_Pos)

#define ADC_INTR_FIFO           (1U << 0)

#define ADC_INTE_FIFO           (1U << 0)

#define ADC_INTF_FIFO           (1U << 0)

#define ADC_INTS_FIFO           (1U << 0)

#endif /* RP2040_ADC_H */

/** @} */
