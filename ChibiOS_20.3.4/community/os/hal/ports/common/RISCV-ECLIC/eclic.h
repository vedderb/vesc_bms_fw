/*
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
 * @file    common/RISCV-ECLIC/eclic.h
 * @brief   RISC-V ECLIC support macros and structures.
 *
 * @addtogroup COMMON_RISCV_ECLIC
 * @{
 */

#ifndef ECLIC_H
#define ECLIC_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
void eclicInit(void);
void eclicEnableVector(uint32_t n, uint32_t level, uint32_t trigger);
void eclicDisableVector(uint32_t n);
void eclicClearPending(uint32_t n);
void eclicSystemReset(void);
#ifdef __cplusplus
}
#endif

#endif /* ECLIC_H */

/** @} */
