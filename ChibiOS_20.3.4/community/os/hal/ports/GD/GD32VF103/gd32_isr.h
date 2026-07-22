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
 * @file    GD32VF103/gd32_isr.h
 * @brief   GD32VF103 ISR handler header.
 *
 * @addtogroup GD32VF103_ISR
 * @{
 */

#ifndef GD32_ISR_H
#define GD32_ISR_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    ISR names and numbers remapping
 * @{
 */
/*
 * CAN units.
 */
#define GD32_CAN0_TX_HANDLER       vector38
#define GD32_CAN0_RX0_HANDLER      vector39
#define GD32_CAN0_RX1_HANDLER      vector40
#define GD32_CAN0_EWMC_HANDLER     vector41
#define GD32_CAN1_TX_HANDLER       vector82
#define GD32_CAN1_RX0_HANDLER      vector83
#define GD32_CAN1_RX1_HANDLER      vector84
#define GD32_CAN1_EWMC_HANDLER     vector85

#define GD32_CAN0_TX_NUMBER        38
#define GD32_CAN0_RX0_NUMBER       39
#define GD32_CAN0_RX1_NUMBER       40
#define GD32_CAN0_EWMC_NUMBER      41
#define GD32_CAN1_TX_NUMBER        82
#define GD32_CAN1_RX0_NUMBER       83
#define GD32_CAN1_RX1_NUMBER       84
#define GD32_CAN1_EWMC_NUMBER      85

/*
 * I2C units.
 */
#define GD32_I2C0_EVENT_HANDLER    vector50
#define GD32_I2C0_ERROR_HANDLER    vector51
#define GD32_I2C0_EVENT_NUMBER     50
#define GD32_I2C0_ERROR_NUMBER     51

#define GD32_I2C1_EVENT_HANDLER    vector52
#define GD32_I2C1_ERROR_HANDLER    vector53
#define GD32_I2C1_EVENT_NUMBER     52
#define GD32_I2C1_ERROR_NUMBER     53

/*
 * TIM units.
 */
#define GD32_TIM0_UP_HANDLER       vector44
#define GD32_TIM0_CC_HANDLER       vector46
#define GD32_TIM1_HANDLER          vector47
#define GD32_TIM2_HANDLER          vector48
#define GD32_TIM3_HANDLER          vector49
#define GD32_TIM4_HANDLER          vector69
#define GD32_TIM5_HANDLER          vector73
#define GD32_TIM6_HANDLER          vector74

#define GD32_TIM0_UP_NUMBER        44
#define GD32_TIM0_CC_NUMBER        46
#define GD32_TIM1_NUMBER           47
#define GD32_TIM2_NUMBER           48
#define GD32_TIM3_NUMBER           49
#define GD32_TIM4_NUMBER           69
#define GD32_TIM5_NUMBER           73
#define GD32_TIM6_NUMBER           74

/*
 * USART units.
 */
#define GD32_USART0_HANDLER        vector56
#define GD32_USART1_HANDLER        vector57
#define GD32_USART2_HANDLER        vector58
#define GD32_UART3_HANDLER         vector71
#define GD32_UART4_HANDLER         vector72

#define GD32_USART0_NUMBER         56
#define GD32_USART1_NUMBER         57
#define GD32_USART2_NUMBER         58
#define GD32_UART3_NUMBER          71
#define GD32_UART4_NUMBER          72

/*
 * OTG units.
 */
#define GD32_USBFS_HANDLER          vector86
#define GD32_USBFS_NUMBER           86

/*
 * RTC unit
 */
#define GD32_RTC1_HANDLER          vector22
#define GD32_RTC1_NUMBER           22
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
#if !defined(GD32_IRQ_EXTI0_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI0_PRIORITY            6
#endif

/**
 * @brief   EXTI1 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI1_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI1_PRIORITY            6
#endif

/**
 * @brief   EXTI2 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI2_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI2_PRIORITY            6
#endif

/**
 * @brief   EXTI3 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI3_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI3_PRIORITY            6
#endif

/**
 * @brief   EXTI4 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI4_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI4_PRIORITY            6
#endif

/**
 * @brief   EXTI9..5 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI5_9_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI5_9_PRIORITY          6
#endif

/**
 * @brief   EXTI15..10 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI10_15_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI10_15_PRIORITY        6
#endif

/**
 * @brief   EXTI0 interrupt trigger setting.
 */
#if !defined(GD32_IRQ_EXTI0_TRIGGER) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI0_TRIGGER            ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   EXTI1 interrupt trigger setting.
 */
#if !defined(GD32_IRQ_EXTI1_TRIGGER) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI1_TRIGGER            ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   EXTI2 interrupt trigger setting.
 */
#if !defined(GD32_IRQ_EXTI2_TRIGGER) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI2_TRIGGER            ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   EXTI3 interrupt trigger setting.
 */
#if !defined(GD32_IRQ_EXTI3_TRIGGER) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI3_TRIGGER            ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   EXTI4 interrupt trigger setting.
 */
#if !defined(GD32_IRQ_EXTI4_TRIGGER) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI4_TRIGGER            ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   EXTI9..5 interrupt trigger setting.
 */
#if !defined(GD32_IRQ_EXTI5_9_TRIGGER) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI5_9_TRIGGER          ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   EXTI15..10 interrupt trigger setting.
 */
#if !defined(GD32_IRQ_EXTI10_15_TRIGGER) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI10_15_TRIGGER        ECLIC_TRIGGER_DEFAULT
#endif

/**
 * @brief   Default ELIC interrupt trigger setting, applied to all 
 *          interrupt sources that do not define a different trigger.
 */
#if !defined(ECLIC_TRIGGER_DEFAULT) || defined(__DOXYGEN__)
#define ECLIC_TRIGGER_DEFAULT            ECLIC_POSTIVE_EDGE_TRIGGER
#endif

/**
 * @brief   Default DMA interrupt trigger setting.
 */
#if !defined(ECLIC_DMA_TRIGGER) || defined(__DOXYGEN__)
#define ECLIC_DMA_TRIGGER                ECLIC_TRIGGER_DEFAULT
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

#endif /* GD32_ISR_H */

/** @} */
