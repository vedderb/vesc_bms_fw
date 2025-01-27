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
 * @file    WB32F3G71xx/wb32_registry.h
 * @brief   WB32F3G71xx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef WB32_REGISTRY_H
#define WB32_REGISTRY_H

#if !defined(WB32F3G71xx)
#error "unsupported or unrecognized WB32F3G71xx member"
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/
#if defined(WB32F3G71xx) || defined(__DOXYGEN__)
/**
 * @name    WB32F3G71xx capabilities
 * @{
 */
/* GPIO attributes.*/
#define WB32_HAS_GPIOA                         TRUE
#define WB32_HAS_GPIOB                         TRUE
#define WB32_HAS_GPIOC                         TRUE
#define WB32_HAS_GPIOD                         TRUE

/* WWDG attributes */
#define WB32_HAS_WWDG                          TRUE

/* PVD attributes */
#define WB32_HAS_PVD                           TRUE

/* TAMPER attributes */
#define WB32_HAS_TAMPER                        TRUE

/* RTC attributes */
#define WB32_HAS_RTC                           TRUE

/* FMC attributes */
#define WB32_HAS_FMC                           TRUE

/* EXTI attributes */
#define WB32_HAS_EXTI                          TRUE
#define WB32_HAS_EXTI0                         TRUE
#define WB32_HAS_EXTI1                         TRUE
#define WB32_HAS_EXTI2                         TRUE
#define WB32_HAS_EXTI3                         TRUE
#define WB32_HAS_EXTI4                         TRUE
#define WB32_HAS_EXTI9_5                       TRUE
#define WB32_HAS_EXTI15_10                     TRUE
#define WB32_EXTI_NUM_LINES                    19

/* DMAC1 attributes */
#define WB32_HAS_DMAC                          TRUE
#define WB32_HAS_DMAC1                         TRUE
#define WB32_DMAC1_NUM_CHANNELS                3
#define WB32_HAS_DMAC2                         TRUE
#define WB32_DMAC2_NUM_CHANNELS                3

/* ADC attributes */
#define WB32_HAS_ADC                           TRUE

/* USB attributes */
#define WB32_HAS_USB                           TRUE
#define WB32_HAS_USB1                          TRUE
#define WB32_HAS_USB1_DMA                      TRUE
#define WB32_HAS_USB1_WKUP                     TRUE

/* TIM attributes */
#define WB32_HAS_TIM                           TRUE
#define WB32_HAS_TIM1                          TRUE
#define WB32_HAS_TIM2                          TRUE
#define WB32_HAS_TIM3                          TRUE
#define WB32_HAS_TIM4                          TRUE

#define WB32_TIM1_IS_32BITS                    TRUE
#define WB32_TIM1_CHANNELS                     4
#define WB32_TIM2_IS_32BITS                    TRUE
#define WB32_TIM2_CHANNELS                     4
#define WB32_TIM3_IS_32BITS                    TRUE
#define WB32_TIM3_CHANNELS                     4
#define WB32_TIM4_IS_32BITS                    TRUE
#define WB32_TIM4_CHANNELS                     4

/* I2C attributes */
#define WB32_HAS_I2C                           TRUE
#define WB32_HAS_I2C1                          TRUE
#define WB32_HAS_I2C2                          TRUE

/* SPI attributes */
#define WB32_HAS_SPI                           TRUE
#define WB32_HAS_QSPI                          TRUE
#define WB32_HAS_SPIM2                         TRUE
#define WB32_HAS_SPIS1                         TRUE
#define WB32_HAS_SPIS2                         TRUE

/* UART attributes */
#define WB32_HAS_UART                          TRUE
#define WB32_HAS_UART1                         TRUE
#define WB32_HAS_UART2                         TRUE
#define WB32_HAS_UART3                         TRUE

/* I2S attributes */
#define WB32_HAS_I2S                           TRUE

/* ISO attributes */
#define WB32_HAS_ISO                           TRUE

/* IWDG attributes.*/
#define WB32_HAS_IWDG                          TRUE

/* CRC attributes.*/
#define WB32_HAS_CRC                           TRUE
/** @} */
#endif /* defined(WB32F3G71xx) */

#endif /* WB32_REGISTRY_H */

/** @} */
