/*
    Copyright (C) 2019 /u/KeepItUnder

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
 * @file    NUC123/nuc123_registry.h
 * @brief   NUC123 capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef NUC123_REGISTRY_H
#define NUC123_REGISTRY_H

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    NUC123 capabilities
 * @{
 */

/* RCC attributes. */
#define NUC123_HAS_HSI48                     FALSE
#define NUC123_HAS_HSI_PREDIV                FALSE
#define NUC123_HAS_MCO_PREDIV                TRUE

/* ADC attributes.*/
#define NUC123_HAS_ADC1                      TRUE
#define NUC123_ADC_SUPPORTS_PRESCALER        FALSE
#define NUC123_ADC_SUPPORTS_OVERSAMPLING     FALSE
#define NUC123_ADC1_IRQ_SHARED_WITH_EXTI     FALSE
#define NUC123_ADC1_HANDLER                  Vector70
#define NUC123_ADC1_NUMBER                   Vector70_IRQn
#define NUC123_ADC1_DMA_MSK                  (NUC123_DMA_STREAM_ID_MSK(1, 1) |\
                                             NUC123_DMA_STREAM_ID_MSK(1, 2))
#define NUC123_ADC1_DMA_CHN                  0x00000011

#define NUC123_HAS_ADC2                      FALSE
#define NUC123_HAS_ADC3                      FALSE
#define NUC123_HAS_ADC4                      FALSE

/* CAN attributes.*/
#define NUC123_HAS_CAN1                      FALSE
#define NUC123_HAS_CAN2                      FALSE
#define NUC123_HAS_CAN3                      FALSE

/* DAC attributes.*/
#define NUC123_HAS_DAC1_CH1                  FALSE
#define NUC123_HAS_DAC1_CH2                  FALSE
#define NUC123_HAS_DAC2_CH1                  FALSE
#define NUC123_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define NUC123_ADVANCED_DMA                  FALSE
#define NUC123_DMA_SUPPORTS_CSELR            FALSE
#define NUC123_DMA1_NUM_CHANNELS             6
#define NUC123_DMA2_NUM_CHANNELS             0
#define NUC123_DMA1_CH1_HANDLER              Vector64
#define NUC123_DMA1_CH23_HANDLER             Vector68
#define NUC123_DMA1_CH4567_HANDLER           Vector6C
#define NUC123_DMA1_CH1_NUMBER               Vector64_IRQn
#define NUC123_DMA1_CH23_NUMBER              Vector68_IRQn
#define NUC123_DMA1_CH4567_NUMBER            Vector6C_IRQn

#define NUC123_DMA1_CH2_NUMBER               NUC123_DMA1_CH23_NUMBER
#define NUC123_DMA1_CH3_NUMBER               NUC123_DMA1_CH23_NUMBER
#define DMA1_CH2_CMASK                      0x00000006U
#define DMA1_CH3_CMASK                      0x00000006U

#define NUC123_DMA1_CH4_NUMBER               NUC123_DMA1_CH4567_NUMBER
#define NUC123_DMA1_CH5_NUMBER               NUC123_DMA1_CH4567_NUMBER
#define NUC123_DMA1_CH6_NUMBER               NUC123_DMA1_CH4567_NUMBER
#define NUC123_DMA1_CH7_NUMBER               NUC123_DMA1_CH4567_NUMBER
#define DMA1_CH4_CMASK                      0x00000078U
#define DMA1_CH5_CMASK                      0x00000078U
#define DMA1_CH6_CMASK                      0x00000078U
#define DMA1_CH7_CMASK                      0x00000078U

/* ETH attributes.*/
#define NUC123_HAS_ETH                       FALSE

/* EXTI attributes.*/
/* #define NUC123_EXTI_NUM_LINES                20 */
/* #define NUC123_EXTI_IMR_MASK                 0xFFF50000U */

/* GPIO attributes.*/
#define NUC123_HAS_GPIOA                     TRUE
#define NUC123_HAS_GPIOB                     TRUE
#define NUC123_HAS_GPIOC                     TRUE
#define NUC123_HAS_GPIOD                     TRUE
#define NUC123_HAS_GPIOE                     FALSE
#define NUC123_HAS_GPIOF                     TRUE
#define NUC123_HAS_GPIOG                     FALSE
#define NUC123_HAS_GPIOH                     FALSE
#define NUC123_HAS_GPIOI                     FALSE
#define NUC123_HAS_GPIOJ                     FALSE
#define NUC123_HAS_GPIOK                     FALSE

/* I2C attributes.*/
#define NUC123_HAS_I2C0                      TRUE
#define NUC123_HAS_I2C1                      TRUE

/* QUADSPI attributes.*/
#define NUC123_HAS_QUADSPI1                  FALSE

/* RTC attributes.*/
#define NUC123_HAS_RTC                       FALSE
#define NUC123_RTC_HAS_SUBSECONDS            FALSE
#define NUC123_RTC_HAS_PERIODIC_WAKEUPS      FALSE
#define NUC123_RTC_NUM_ALARMS                0
#define NUC123_RTC_HAS_INTERRUPTS            FALSE

/* SDIO attributes.*/
#define NUC123_HAS_SDIO                      FALSE

/* SPI attributes.*/
#define NUC123_HAS_SPI1                      TRUE
#define NUC123_SPI1_SUPPORTS_I2S             FALSE
#define NUC123_SPI1_RX_DMA_MSK               NUC123_DMA_STREAM_ID_MSK(1, 2)
#define NUC123_SPI1_RX_DMA_CHN               0x00000030
#define NUC123_SPI1_TX_DMA_MSK               NUC123_DMA_STREAM_ID_MSK(1, 3)
#define NUC123_SPI1_TX_DMA_CHN               0x00000300

#define NUC123_HAS_SPI2                      TRUE
#define NUC123_SPI2_SUPPORTS_I2S             FALSE
#define NUC123_SPI2_RX_DMA_MSK               NUC123_DMA_STREAM_ID_MSK(1, 4)
#define NUC123_SPI2_RX_DMA_CHN               0x00003000
#define NUC123_SPI2_TX_DMA_MSK               NUC123_DMA_STREAM_ID_MSK(1, 5)
#define NUC123_SPI2_TX_DMA_CHN               0x00030000

#define NUC123_HAS_SPI3                      FALSE
#define NUC123_HAS_SPI4                      FALSE
#define NUC123_HAS_SPI5                      FALSE
#define NUC123_HAS_SPI6                      FALSE

/* TIM attributes.*/
#define NUC123_TIM_MAX_CHANNELS              4

#define NUC123_HAS_TIM1                      TRUE
#define NUC123_TIM1_IS_32BITS                TRUE
#define NUC123_TIM1_CHANNELS                 1

#define NUC123_HAS_TIM2                      TRUE
#define NUC123_TIM2_IS_32BITS                TRUE
#define NUC123_TIM2_CHANNELS                 1

#define NUC123_HAS_TIM3                      TRUE
#define NUC123_TIM3_IS_32BITS                TRUE
#define NUC123_TIM3_CHANNELS                 1

#define NUC123_HAS_TIM4                      TRUE
#define NUC123_TIM14_IS_32BITS               TRUE
#define NUC123_TIM14_CHANNELS                1

#define NUC123_HAS_TIM5                      FALSE
#define NUC123_HAS_TIM6                      FALSE
#define NUC123_HAS_TIM7                      FALSE
#define NUC123_HAS_TIM8                      FALSE
#define NUC123_HAS_TIM9                      FALSE
#define NUC123_HAS_TIM10                     FALSE
#define NUC123_HAS_TIM11                     FALSE
#define NUC123_HAS_TIM12                     FALSE
#define NUC123_HAS_TIM13                     FALSE
#define NUC123_HAS_TIM14                     FALSE
#define NUC123_HAS_TIM15                     FALSE
#define NUC123_HAS_TIM16                     FALSE
#define NUC123_HAS_TIM17                     FALSE
#define NUC123_HAS_TIM18                     FALSE
#define NUC123_HAS_TIM19                     FALSE
#define NUC123_HAS_TIM20                     FALSE
#define NUC123_HAS_TIM21                     FALSE
#define NUC123_HAS_TIM22                     FALSE

/* USART attributes.*/
/* #define NUC123_HAS_USART1                    TRUE
#define NUC123_USART1_RX_DMA_MSK             (NUC123_DMA_STREAM_ID_MSK(1, 1) |\
                                             NUC123_DMA_STREAM_ID_MSK(1, 3) |\
                                             NUC123_DMA_STREAM_ID_MSK(1, 5))
#define NUC123_USART1_RX_DMA_CHN             0x00080808
#define NUC123_USART1_TX_DMA_MSK             (NUC123_DMA_STREAM_ID_MSK(1, 2) |\
                                             NUC123_DMA_STREAM_ID_MSK(1, 4))
#define NUC123_USART1_TX_DMA_CHN             0x00008080

#define NUC123_HAS_USART2                    TRUE
#define NUC123_USART2_RX_DMA_MSK             (NUC123_DMA_STREAM_ID_MSK(1, 1) |\
                                             NUC123_DMA_STREAM_ID_MSK(1, 3) |\
                                             NUC123_DMA_STREAM_ID_MSK(1, 5))
#define NUC123_USART2_RX_DMA_CHN             0x00090909
#define NUC123_USART2_TX_DMA_MSK             (NUC123_DMA_STREAM_ID_MSK(1, 2) |\
                                             NUC123_DMA_STREAM_ID_MSK(1, 4))
#define NUC123_USART2_TX_DMA_CHN             0x00009090 */

#define NUC123_HAS_USART1                    FALSE
#define NUC123_HAS_USART2                    FALSE
#define NUC123_HAS_USART3                    FALSE
#define NUC123_HAS_UART4                     FALSE
#define NUC123_HAS_UART5                     FALSE
#define NUC123_HAS_USART6                    FALSE
#define NUC123_HAS_UART7                     FALSE
#define NUC123_HAS_UART8                     FALSE
#define NUC123_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define NUC123_HAS_USB                       TRUE
#define NUC123_HAS_OTG1                      FALSE
#define NUC123_HAS_OTG2                      FALSE

/* IWDG attributes.*/
#define NUC123_HAS_IWDG                      TRUE
#define NUC123_IWDG_IS_WINDOWED              TRUE

/* LTDC attributes.*/
#define NUC123_HAS_LTDC                      FALSE

/* DMA2D attributes.*/
#define NUC123_HAS_DMA2D                     FALSE

/* FSMC attributes.*/
#define NUC123_HAS_FSMC                      FALSE

/* CRC attributes.*/
#define NUC123_HAS_CRC                       TRUE
#define NUC123_CRC_PROGRAMMABLE              FALSE

/** @} */

#endif /* NUC123_REGISTRY_H */

/** @} */
