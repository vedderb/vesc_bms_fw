/*
    ChibiOS - Copyright (C) 2020 Yaotian Feng / Codetector

    This file is part of ChibiOS.
    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.
    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file        LPC11Uxx/cmparams.h
 * @brief       ARM Cortex-M0 parameters for the NXP LPC11Uxx family.
 *
 * @defgroup    ARMCMx_LPC11Uxx NXP Semi. LPC11Uxx.
 * @ingroup     ARMCMx_SPECIFIC
 * @details     This file contains the Cortex-M0 specific parameters for the
 *              NRF51x platform.
 * @{
 */

#ifndef _CMPARAMS_H_
#define _CMPARAMS_H_

/**
 * @brief   Cortex core model.
 */
#define CORTEX_MODEL            0

/**
 * @brief   Systick unit presence.
 */
#define CORTEX_HAS_ST           TRUE


/**
 * @brief   Floating Point unit presence.
 */
#define CORTEX_HAS_FPU          FALSE

/**
 * @brief   Number of bits in priority masks.
 */
#define CORTEX_PRIORITY_BITS    2

/**
 * @brief   Number of interrupt vectors.
 * @note    This number does not include the 16 system vectors and must be
 *          rounded to a multiple of 8.
 */
#define CORTEX_NUM_VECTORS      32

#define LPC_VECTOR_CHECKSUM     TRUE

/* The following code is not processed when the file is included from an
   asm module.*/
#if !defined(_FROM_ASM_)
/* Including the device CMSIS header. Note, we are not using the definitions
   from this header because we need this file to be usable also from
   assembler source files. We verify that the info matches instead.*/
#include "LPC11Uxx.h"

#if CORTEX_MODEL != __CORTEX_M
#error "CMSIS __CORTEX_M mismatch"
#endif

#if CORTEX_PRIORITY_BITS != __NVIC_PRIO_BITS
#error "CMSIS __NVIC_PRIO_BITS mismatch"
#endif

#endif /* !defined(_FROM_ASM_) */

#endif /* _CMPARAMS_H_ */

/** @} */
