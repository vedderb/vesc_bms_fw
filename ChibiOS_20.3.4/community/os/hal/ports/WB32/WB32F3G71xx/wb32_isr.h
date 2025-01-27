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
 * @file    WB32F3G71xx/wb32_isr.h
 * @brief   WB32F3G71xx ISR handler header.
 *
 * @addtogroup WB32F3G71xx_ISR
 * @{
 */

#ifndef WB32_ISR_H
#define WB32_ISR_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    ISR names and numbers remapping
 * @{
 */

/*
 * IWDG units.
 */
#define WB32_WWDG_IRQ_VECTOR                   Vector40
#define WB32_WWDG_NUMBER                       0

/*
 * PVD units.
 */
#define WB32_PVD_IRQ_VECTOR                    Vector44
#define WB32_PVD_NUMBER                        1

/*
 * TAMPER units.
 */
#define WB32_TAMPER_IRQ_VECTOR                 Vector48
#define WB32_TAMPER_NUMBER                     2

/*
 * RTC units.
 */
#define WB32_RTC_IRQ_VECTOR                    Vector4C
#define WB32_RTC_NUMBER                        3

#define WB32_RTCAlarm_IRQ_VECTOR               VectorC8
#define WB32_RTCAlarm_NUMBER                   34

/*
 * FMC units.
 */
#define WB32_FMC_IRQ_VECTOR                    Vector50
#define WB32_FMC_NUMBER                        4
#define WB32_RCC_IRQ_VECTOR                    Vector54
#define WB32_RCC_NUMBER                        5

/*
 * EXTI units.
 */
#define WB32_EXTI0_IRQ_VECTOR                  Vector58
#define WB32_EXTI0_NUMBER                      6

#define WB32_EXTI1_IRQ_VECTOR                  Vector5C
#define WB32_EXTI1_NUMBER                      7

#define WB32_EXTI2_IRQ_VECTOR                  Vector60
#define WB32_EXTI2_NUMBER                      8

#define WB32_EXTI3_IRQ_VECTOR                  Vector64
#define WB32_EXTI3_NUMBER                      9

#define WB32_EXTI4_IRQ_VECTOR                  Vector68
#define WB32_EXTI4_NUMBER                      10

#define WB32_EXTI9_5_IRQ_VECTOR                Vector80
#define WB32_EXTI9_5_NUMBER                    16

#define WB32_EXTI15_10_IRQ_VECTOR              VectorC4
#define WB32_EXTI15_10_NUMBER                  33

/*
 * DMAC units.
 */
#define WB32_DMAC1_IRQ_VECTOR                  Vector6C
#define WB32_DMAC1_NUMBER                      11

#define WB32_DMAC2_IRQ_VECTOR                  Vector70
#define WB32_DMAC2_NUMBER                      12

/*
 * ADC units.
 */
#define WB32_ADC_IRQ_VECTOR                    Vector74
#define WB32_ADC_NUMBER                        13

/*
 * USB units.
 */
#define WB32_USB1_IRQ_VECTOR                   Vector78
#define WB32_USB1_DMA_IRQ_VECTOR               Vector7C
#define WB32_USBP1_WKUP_IRQ_VECTOR             VectorCC

#define WB32_USB1_NUMBER                       14
#define WB32_USB1_DMA_NUMBER                   15
#define WB32_USBP1_WKUP_NUMBER                 35

/*
 * I2C units.
 */
#define WB32_I2C1_IRQ_VECTOR                   VectorA0
#define WB32_I2C1_NUMBER                       24

#define WB32_I2C2_IRQ_VECTOR                   VectorA4
#define WB32_I2C2_NUMBER                       25

/*
 * I2S units.
 */
#define WB32_I2S_IRQ_VECTOR                    VectorD0
#define WB32_I2S_NUMBER                        36

/*
 * SPI units.
 */
#define WB32_QSPI_IRQ_VECTOR                   VectorA8
#define WB32_QSPI_NUMBER                       26

#define WB32_SPIM2_IRQ_VECTOR                  VectorAC
#define WB32_SPIM2_NUMBER                      27

#define WB32_SPIS1_IRQ_VECTOR                  VectorB0
#define WB32_SPIS1_NUMBER                      28

#define WB32_SPIS2_IRQ_VECTOR                  VectorB4
#define WB32_SPIS2_NUMBER                      29

/*
 * TIM units.
 */
#define WB32_TIM1_BRK_IRQ_VECTOR               Vector84
#define WB32_TIM1_UP_IRQ_VECTOR                Vector88
#define WB32_TIM1_TRG_COM_IRQ_VECTOR           Vector8C
#define WB32_TIM1_CC_IRQ_VECTOR                Vector90
#define WB32_TIM1_BRK_NUMBER                   17
#define WB32_TIM1_UP_NUMBER                    18
#define WB32_TIM1_TRG_COM_NUMBER               19
#define WB32_TIM1_CC_NUMBER                    20

#define WB32_TIM2_IRQ_VECTOR                   Vector94
#define WB32_TIM2_NUMBER                       21

#define WB32_TIM3_IRQ_VECTOR                   Vector98
#define WB32_TIM3_NUMBER                       22

#define WB32_TIM4_IRQ_VECTOR                   Vector9C
#define WB32_TIM4_NUMBER                       23

/*
 * USART units.
 */
#define WB32_UART1_IRQ_VECTOR                  VectorB8
#define WB32_UART1_NUMBER                      30

#define WB32_UART2_IRQ_VECTOR                  VectorBC
#define WB32_UART2_NUMBER                      31

#define WB32_UART3_IRQ_VECTOR                  VectorC0
#define WB32_UART3_NUMBER                      32

/*
 * ISO units.
 */
#define WB32_ISO_IRQ_VECTOR                    VectorD4
#define WB32_ISO_NUMBER                        37

/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   EXTI0 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI0_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI0_PRIORITY            6
#endif

/**
 * @brief   EXTI1 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI1_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI1_PRIORITY            6
#endif

/**
 * @brief   EXTI2 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI2_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI2_PRIORITY            6
#endif

/**
 * @brief   EXTI3 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI3_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI3_PRIORITY            6
#endif

/**
 * @brief   EXTI4 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI4_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI4_PRIORITY            6
#endif

/**
 * @brief   EXTI9..5 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI5_9_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI5_9_PRIORITY          6
#endif

/**
 * @brief   EXTI15..10 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI10_15_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI10_15_PRIORITY        6
#endif

/**
 * @brief   EXTI16 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI16_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI16_PRIORITY           6
#endif

/**
 * @brief   EXTI17 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI17_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI17_PRIORITY           6
#endif

/**
 * @brief   EXTI18 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI18_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI18_PRIORITY           6
#endif

/**
 * @brief   EXTI19 interrupt priority level setting.
 */
#if !defined(WB32_IRQ_EXTI19_PRIORITY) || defined(__DOXYGEN__)
#define WB32_IRQ_EXTI19_PRIORITY           6
#endif
/** @} */

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
  void irqInit(void);
  void irqDeinit(void);
#ifdef __cplusplus
}
#endif

#endif /* WB32_ISR_H */

/** @} */
