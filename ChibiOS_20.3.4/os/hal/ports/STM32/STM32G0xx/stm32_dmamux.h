/*
    ChibiOS - Copyright (C) 2006..2019 Giovanni Di Sirio

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
 * @file    STM32G0xx/stm32_dmamux.h
 * @brief   STM32G0xx DMAMUX handler header.
 *
 * @addtogroup STM32G0xx_DMAMUX
 * @{
 */

#ifndef STM32_DMAMUX_H
#define STM32_DMAMUX_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    DMAMUX1 request sources
 * @{
 */
#define STM32_DMAMUX1_REQ_GEN0      1
#define STM32_DMAMUX1_REQ_GEN1      2
#define STM32_DMAMUX1_REQ_GEN2      3
#define STM32_DMAMUX1_REQ_GEN3      4
#define STM32_DMAMUX1_ADC1          5
#define STM32_DMAMUX1_AES_IN        6
#define STM32_DMAMUX1_AES_OUT       7
#define STM32_DMAMUX1_DAC1_CH1      8
#define STM32_DMAMUX1_DAC1_CH2      9
#define STM32_DMAMUX1_I2C1_RX       10
#define STM32_DMAMUX1_I2C1_TX       11
#define STM32_DMAMUX1_I2C2_RX       12
#define STM32_DMAMUX1_I2C2_TX       13
#define STM32_DMAMUX1_LPUART1_RX    14
#define STM32_DMAMUX1_LPUART1_TX    15
#define STM32_DMAMUX1_SPI1_RX       16
#define STM32_DMAMUX1_SPI1_TX       17
#define STM32_DMAMUX1_SPI2_RX       18
#define STM32_DMAMUX1_SPI2_TX       19
#define STM32_DMAMUX1_TIM1_CH1      20
#define STM32_DMAMUX1_TIM1_CH2      21
#define STM32_DMAMUX1_TIM1_CH3      22
#define STM32_DMAMUX1_TIM1_CH4      23
#define STM32_DMAMUX1_TIM1_COM      24
#define STM32_DMAMUX1_TIM1_UP       25
#define STM32_DMAMUX1_TIM2_CH1      26
#define STM32_DMAMUX1_TIM2_CH2      27
#define STM32_DMAMUX1_TIM2_CH3      28
#define STM32_DMAMUX1_TIM2_CH4      29
#define STM32_DMAMUX1_TIM2_TRIG     30
#define STM32_DMAMUX1_TIM2_UP       31
#define STM32_DMAMUX1_TIM3_CH1      32
#define STM32_DMAMUX1_TIM3_CH2      33
#define STM32_DMAMUX1_TIM3_CH3      34
#define STM32_DMAMUX1_TIM3_CH4      35
#define STM32_DMAMUX1_TIM3_TRIG     36
#define STM32_DMAMUX1_TIM3_UP       37
#define STM32_DMAMUX1_TIM6_UP       38
#define STM32_DMAMUX1_TIM7_UP       39
#define STM32_DMAMUX1_TIM15_CH1     40
#define STM32_DMAMUX1_TIM15_CH2     41
#define STM32_DMAMUX1_TIM15_COM     42
#define STM32_DMAMUX1_TIM15_UP      43
#define STM32_DMAMUX1_TIM16_CH1     44
#define STM32_DMAMUX1_TIM16_COM     45
#define STM32_DMAMUX1_TIM16_UP      46
#define STM32_DMAMUX1_TIM17_CH1     47
#define STM32_DMAMUX1_TIM17_COM     48
#define STM32_DMAMUX1_TIM17_UP      49
#define STM32_DMAMUX1_USART1_RX     50
#define STM32_DMAMUX1_USART1_TX     51
#define STM32_DMAMUX1_USART2_RX     52
#define STM32_DMAMUX1_USART2_TX     53
#define STM32_DMAMUX1_USART3_RX     54
#define STM32_DMAMUX1_USART3_TX     55
#define STM32_DMAMUX1_USART4_RX     56
#define STM32_DMAMUX1_USART4_TX     57
#define STM32_DMAMUX1_UART4_RX      STM32_DMAMUX1_USART4_RX /* Legacy.      */
#define STM32_DMAMUX1_UART4_TX      STM32_DMAMUX1_USART4_TX /* Legacy.      */
#define STM32_DMAMUX1_UCPD1_RX      58
#define STM32_DMAMUX1_UCPD1_TX      59
#define STM32_DMAMUX1_UCPD2_RX      60
#define STM32_DMAMUX1_UCPD2_TX      61
/** @} */

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

#ifdef __cplusplus
}
#endif

#endif /* STM32_DMAMUX_H */

/** @} */
