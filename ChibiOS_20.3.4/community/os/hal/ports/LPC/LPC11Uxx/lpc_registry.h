/*
    ChibiOS - Copyright (C) 2020 Yaotian Feng / Codetector

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
 * @file    LPC11Uxx/lpc_registry.h
 * @brief   LPC11U11 capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef LPC_REGISTRY_H_
#define LPC_REGISTRY_H_

/**
 * @brief   Sub-family identifier.
 */
#if defined(LPC11U35) || \
defined(__DOXYGEN__)
    #define LPC11Uxx
#else
#error unknown/unsupported LPC11Uxx microcontroller
#endif

#if defined(LPC11Uxx) || defined(__DOXYGEN__)

/* USB attributes.*/
#define LPC_HAS_USB                 TRUE
#define LPC_USB_IRQ_VECTOR          Vector98

#define LPC_SSP0_IRQ_VECTOR         Vector90
#define LPC_SSP1_IRQ_VECTOR         Vector78

#define LPC_UART_IRQ_VECTOR         Vector94

#endif


/** @} */

#endif /* LPC_REGISTRY_H_ */

/** @} */
