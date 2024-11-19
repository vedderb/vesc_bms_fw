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
 * @file    common/RISCV-ECLIC/eclic.c
 * @brief   RISC-V ECLIC interrupt support code.
 *
 * @addtogroup COMMON_RISCV_ECLIC
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
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Sets the level and trigger of an interrupt handler and enables it.
 *
 * @param[in] n         the interrupt number
 * @param[in] level     the interrupt level
 * @param[in] trigger   the interrupt trigger
 */
void eclicEnableVector(uint32_t n, uint32_t prio, uint32_t trigger) {
  __ECLIC_SetLevelIRQ(n, prio);
  __ECLIC_SetTrigIRQ(n, trigger);
  __ECLIC_EnableIRQ(n);
}

/**
 * @brief   Disables an interrupt handler.
 *
 * @param[in] n         the interrupt number
 */
void eclicDisableVector(uint32_t n) { __ECLIC_DisableIRQ(n); }

/**
 * @brief   Clears a pending interrupt source.
 *
 * @param[in] n         the interrupt number
 */
void eclicClearPending(uint32_t n) { __ECLIC_ClearPendingIRQ(n); }

/**
 * @brief   System Reset
 */
void __attribute__((noreturn)) eclicSystemReset(void) {
  SysTimer_SoftwareReset();
  while(1);
}
/** @} */
