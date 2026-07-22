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
 * @file    GD32VF103/gd32_registry.h
 * @brief   GD32VF103xx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef GD32_REGISTRY_H
#define GD32_REGISTRY_H

#if defined(GD32VF103TB) || defined(GD32VF103T8)

#define GD32_HAS_TIM_12                     TRUE
#define GD32_HAS_TIM_34                     TRUE
#define GD32_HAS_USART_01                   TRUE
#define GD32_HAS_USART_2                    FALSE
#define GD32_HAS_USART_34                   FALSE
#define GD32_HAS_I2C_0                      TRUE
#define GD32_HAS_I2C_1                      FALSE
#define GD32_HAS_SPI_0                      TRUE
#define GD32_HAS_SPI_12                     FALSE
#define GD32_HAS_GPIO_AB                    TRUE
#define GD32_HAS_GPIO_CD                    FALSE
#define GD32_HAS_GPIO_E                     FALSE
#define GD32_HAS_EXMC                       FALSE

#elif defined(GD32VF103T6) || defined(GD32VF103T4)

#define GD32_HAS_TIM_12                     TRUE
#define GD32_HAS_TIM_34                     FALSE
#define GD32_HAS_USART_01                   TRUE
#define GD32_HAS_USART_2                    FALSE
#define GD32_HAS_USART_34                   FALSE
#define GD32_HAS_I2C_0                      TRUE
#define GD32_HAS_I2C_1                      FALSE
#define GD32_HAS_SPI_0                      TRUE
#define GD32_HAS_SPI_12                     FALSE
#define GD32_HAS_GPIO_AB                    TRUE
#define GD32_HAS_GPIO_CD                    FALSE
#define GD32_HAS_GPIO_E                     FALSE
#define GD32_HAS_EXMC                       FALSE

#elif defined(GD32VF103CB) || defined(GD32VF103C8)

#define GD32_HAS_TIM_12                     TRUE
#define GD32_HAS_TIM_34                     TRUE
#define GD32_HAS_USART_01                   TRUE
#define GD32_HAS_USART_2                    TRUE
#define GD32_HAS_USART_34                   FALSE
#define GD32_HAS_I2C_0                      TRUE
#define GD32_HAS_I2C_1                      TRUE
#define GD32_HAS_SPI_0                      TRUE
#define GD32_HAS_SPI_12                     TRUE
#define GD32_HAS_GPIO_AB                    TRUE
#define GD32_HAS_GPIO_CD                    FALSE
#define GD32_HAS_GPIO_E                     FALSE
#define GD32_HAS_EXMC                       FALSE

#elif defined(GD32VF103C6) || defined(GD32VF103C4)

#define GD32_HAS_TIM_12                     TRUE
#define GD32_HAS_TIM_34                     FALSE
#define GD32_HAS_USART_01                   TRUE
#define GD32_HAS_USART_2                    FALSE
#define GD32_HAS_USART_34                   FALSE
#define GD32_HAS_I2C_0                      TRUE
#define GD32_HAS_I2C_1                      FALSE
#define GD32_HAS_SPI_0                      TRUE
#define GD32_HAS_SPI_12                     FALSE
#define GD32_HAS_GPIO_AB                    TRUE
#define GD32_HAS_GPIO_CD                    FALSE
#define GD32_HAS_GPIO_E                     FALSE
#define GD32_HAS_EXMC                       FALSE

#elif defined(GD32VF103RB) || defined(GD32VF103R8)

#define GD32_HAS_TIM_12                     TRUE
#define GD32_HAS_TIM_34                     TRUE
#define GD32_HAS_USART_01                   TRUE
#define GD32_HAS_USART_2                    TRUE
#define GD32_HAS_USART_34                   TRUE
#define GD32_HAS_I2C_0                      TRUE
#define GD32_HAS_I2C_1                      TRUE
#define GD32_HAS_SPI_0                      TRUE
#define GD32_HAS_SPI_12                     TRUE
#define GD32_HAS_GPIO_AB                    TRUE
#define GD32_HAS_GPIO_CD                    TRUE
#define GD32_HAS_GPIO_E                     FALSE
#define GD32_HAS_EXMC                       FALSE

#elif defined(GD32VF103R6) || defined(GD32VF103R4)

#define GD32_HAS_TIM_12                     TRUE
#define GD32_HAS_TIM_34                     FALSE
#define GD32_HAS_USART_01                   TRUE
#define GD32_HAS_USART_2                    FALSE
#define GD32_HAS_USART_34                   FALSE
#define GD32_HAS_I2C_0                      TRUE
#define GD32_HAS_I2C_1                      FALSE
#define GD32_HAS_SPI_0                      TRUE
#define GD32_HAS_SPI_12                     FALSE
#define GD32_HAS_GPIO_AB                    TRUE
#define GD32_HAS_GPIO_CD                    TRUE
#define GD32_HAS_GPIO_E                     FALSE
#define GD32_HAS_EXMC                       FALSE

#elif defined(GD32VF103VB) || defined(GD32VF103V8)

#define GD32_HAS_TIM_12                     TRUE
#define GD32_HAS_TIM_34                     TRUE
#define GD32_HAS_USART_01                   TRUE
#define GD32_HAS_USART_2                    TRUE
#define GD32_HAS_USART_34                   TRUE
#define GD32_HAS_I2C_0                      TRUE
#define GD32_HAS_I2C_1                      TRUE
#define GD32_HAS_SPI_0                      TRUE
#define GD32_HAS_SPI_12                     TRUE
#define GD32_HAS_GPIO_AB                    TRUE
#define GD32_HAS_GPIO_CD                    TRUE
#define GD32_HAS_GPIO_E                     TRUE
#define GD32_HAS_EXMC                       TRUE

#else

#error "unsupported or unrecognized GD32VF103 member"

#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

#if defined(GD32VF103) || defined(__DOXYGEN__)
/**
 * @name    GD32VF103 family capabilities
 * @{
 */
/* ADC attributes.*/
#define GD32_HAS_ADC0                      TRUE
#define GD32_HAS_ADC1                      TRUE

/* CAN attributes.*/
#define GD32_HAS_CAN0                      TRUE
#define GD32_HAS_CAN1                      TRUE
#define GD32_CAN_MAX_FILTERS               28

/* CRC attributes.*/
#define GD32_HAS_CRC                       TRUE
#define GD32_CRC_PROGRAMMABLE              FALSE

/* STM32 compatibility define. */
#if GD32_CRC_USE_CRC0 == TRUE 
#define STM32_CRC_USE_CRC1 TRUE
#else
#define STM32_CRC_USE_CRC1 FALSE
#endif

/* DAC attributes.*/
#define GD32_HAS_DAC_CH1                  TRUE
#define GD32_DAC_CH1_DMA_STREAM           GD32_DMA_STREAM_ID(1, 2)

#define GD32_HAS_DAC_CH2                  TRUE
#define GD32_DAC_CH2_DMA_STREAM           GD32_DMA_STREAM_ID(2, 3)

/* DMA attributes.*/
#define GD32_DMA0_NUM_CHANNELS             7
#define GD32_DMA0_CH0_HANDLER               vector30
#define GD32_DMA0_CH1_HANDLER               vector31
#define GD32_DMA0_CH2_HANDLER               vector32
#define GD32_DMA0_CH3_HANDLER               vector33
#define GD32_DMA0_CH4_HANDLER               vector34
#define GD32_DMA0_CH5_HANDLER               vector35
#define GD32_DMA0_CH6_HANDLER               vector36
#define GD32_DMA0_CH0_NUMBER                30
#define GD32_DMA0_CH1_NUMBER                31
#define GD32_DMA0_CH2_NUMBER                32
#define GD32_DMA0_CH3_NUMBER                33
#define GD32_DMA0_CH4_NUMBER                34
#define GD32_DMA0_CH5_NUMBER                35
#define GD32_DMA0_CH6_NUMBER                36

#define GD32_DMA1_NUM_CHANNELS             5
#define GD32_DMA1_CH0_HANDLER              vector75
#define GD32_DMA1_CH1_HANDLER              vector76
#define GD32_DMA1_CH2_HANDLER              vector77
#define GD32_DMA1_CH3_HANDLER              vector78
#define GD32_DMA1_CH4_HANDLER              vector79
#define GD32_DMA1_CH0_NUMBER                75
#define GD32_DMA1_CH1_NUMBER                76
#define GD32_DMA1_CH2_NUMBER                77
#define GD32_DMA1_CH3_NUMBER                78
#define GD32_DMA1_CH4_NUMBER                79

/* EXTI attributes.*/
#define GD32_EXTI_NUM_LINES                15
#define GD32_EXTI_IMR_MASK                 0x00000000U

/* Flash attributes.*/
#define GD32_FLASH_NUMBER_OF_BANKS         1
#define GD32_FLASH_SECTOR_SIZE             1024U
#if !defined(GD32_FLASH_SECTORS_PER_BANK) || defined(__DOXYGEN__)
#define GD32_FLASH_SECTORS_PER_BANK        128 /* Maximum, can be redefined.*/
#endif

/* GPIO attributes.*/
#if GD32_HAS_GPIO_AB 
    #define GD32_HAS_GPIOA                     TRUE
    #define GD32_HAS_GPIOB                     TRUE
#else
    #define GD32_HAS_GPIOA                     FALSE
    #define GD32_HAS_GPIOB                     FALSE
#endif

#if GD32_HAS_GPIO_CD
    #define GD32_HAS_GPIOC                     TRUE
    #define GD32_HAS_GPIOD                     TRUE
#else
    #define GD32_HAS_GPIOC                     FALSE
    #define GD32_HAS_GPIOD                     FALSE
#endif

#if GD32_HAS_GPIO_E
    #define GD32_HAS_GPIOE                     TRUE
#else
    #define GD32_HAS_GPIOE                     FALSE
#endif

/* I2C attributes.*/
#if GD32_HAS_I2C_0 
    #define GD32_HAS_I2C0                      TRUE
    #define GD32_I2C_I2C0_RX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 6)
    #define GD32_I2C_I2C0_TX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 5)
#else
    #define GD32_HAS_I2C0                      FALSE
#endif

#if GD32_HAS_I2C_1
    #define GD32_HAS_I2C1                      TRUE
    #define GD32_I2C_I2C1_RX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 4)
    #define GD32_I2C_I2C1_TX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 3)
#else
    #define GD32_HAS_I2C1                      FALSE
#endif

/* RTC attributes.*/
#define GD32_HAS_RTC                       TRUE
#define GD32_RTC_HAS_SUBSECONDS            TRUE
#define GD32_RTC_IS_CALENDAR               FALSE

/* SPI attributes.*/
#if GD32_HAS_SPI_0 
    #define GD32_HAS_SPI0                      TRUE
    #define GD32_SPI0_SUPPORTS_I2S             FALSE
    #define GD32_SPI_SPI0_RX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 1)
    #define GD32_SPI_SPI0_TX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 2)
#else
    #define GD32_HAS_SPI0                      FALSE
#endif

#if GD32_HAS_SPI_12
    #define GD32_HAS_SPI1                      TRUE
    #define GD32_SPI1_SUPPORTS_I2S             TRUE
    #define GD32_SPI1_I2S_FULLDUPLEX           FALSE
    #define GD32_SPI_SPI1_RX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 3)
    #define GD32_SPI_SPI1_TX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 4)
    #define GD32_I2S_SPI1_RX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 3)
    #define GD32_I2S_SPI1_TX_DMA_STREAM        GD32_DMA_STREAM_ID(0, 4)
    #define GD32_HAS_SPI2                      TRUE
    #define GD32_SPI2_SUPPORTS_I2S             TRUE
    #define GD32_SPI2_I2S_FULLDUPLEX           FALSE
    #define GD32_SPI_SPI2_RX_DMA_STREAM        GD32_DMA_STREAM_ID(1, 0)
    #define GD32_SPI_SPI2_TX_DMA_STREAM        GD32_DMA_STREAM_ID(1, 1)
    #define GD32_I2S_SPI2_RX_DMA_STREAM        GD32_DMA_STREAM_ID(1, 0)
    #define GD32_I2S_SPI2_TX_DMA_STREAM        GD32_DMA_STREAM_ID(1, 1)
#else
    #define GD32_HAS_SPI1                      FALSE
    #define GD32_HAS_SPI2                      FALSE
#endif

/* TIM attributes.*/
#define GD32_TIM_MAX_CHANNELS              4

#define GD32_HAS_TIM0                      TRUE
#define GD32_TIM0_IS_32BITS                FALSE
#define GD32_TIM0_CHANNELS                 4

#if GD32_HAS_TIM_12 
    #define GD32_HAS_TIM1                      TRUE
    #define GD32_TIM1_IS_32BITS                FALSE
    #define GD32_TIM1_CHANNELS                 4

    #define GD32_HAS_TIM2                      TRUE
    #define GD32_TIM2_IS_32BITS                FALSE
    #define GD32_TIM2_CHANNELS                 4
#else
    #define GD32_HAS_TIM1                      FALSE
    #define GD32_HAS_TIM2                      FALSE
#endif

#if GD32_HAS_TIM_34
    #define GD32_HAS_TIM3                      TRUE
    #define GD32_TIM3_IS_32BITS                FALSE
    #define GD32_TIM3_CHANNELS                 4

    #define GD32_HAS_TIM4                      TRUE
    #define GD32_TIM4_IS_32BITS                FALSE
    #define GD32_TIM4_CHANNELS                 4
#else
    #define GD32_HAS_TIM3                      FALSE
    #define GD32_HAS_TIM4                      FALSE
#endif

#define GD32_HAS_TIM5                      TRUE
#define GD32_TIM5_IS_32BITS                FALSE
#define GD32_TIM5_CHANNELS                 0

#define GD32_HAS_TIM6                      TRUE
#define GD32_TIM6_IS_32BITS                FALSE
#define GD32_TIM6_CHANNELS                 0

/* USART attributes.*/
#if GD32_HAS_USART_01 
    #define GD32_HAS_USART0                    TRUE
    #define GD32_UART_USART0_RX_DMA_STREAM     GD32_DMA_STREAM_ID(0, 4)
    #define GD32_UART_USART0_TX_DMA_STREAM     GD32_DMA_STREAM_ID(0, 3)

    #define GD32_HAS_USART1                    TRUE
    #define GD32_UART_USART1_RX_DMA_STREAM     GD32_DMA_STREAM_ID(0, 5)
    #define GD32_UART_USART1_TX_DMA_STREAM     GD32_DMA_STREAM_ID(0, 6)
#else
    #define GD32_HAS_USART0                    FALSE
    #define GD32_HAS_USART1                    FALSE
#endif

#if GD32_HAS_USART_2 
    #define GD32_HAS_USART2                    TRUE
    #define GD32_UART_USART2_RX_DMA_STREAM     GD32_DMA_STREAM_ID(0, 2)
    #define GD32_UART_USART2_TX_DMA_STREAM     GD32_DMA_STREAM_ID(0, 1)
#else
    #define GD32_HAS_USART2                    FALSE
#endif

#if GD32_HAS_USART_34
    #define GD32_HAS_UART3                     TRUE
    #define GD32_UART_UART3_RX_DMA_STREAM      GD32_DMA_STREAM_ID(1, 2)
    #define GD32_UART_UART3_TX_DMA_STREAM      GD32_DMA_STREAM_ID(1, 4)
    #define GD32_HAS_UART4                     TRUE
#else
    #define GD32_HAS_UART3                     FALSE
    #define GD32_HAS_UART4                     FALSE
#endif

/* USB attributes.*/
#define GD32_HAS_USBFS                      TRUE
#define GD32_USBFS_ENDPOINTS                3

#define GD32_HAS_USB                       TRUE

/* FWDGT attributes.*/
#define GD32_HAS_FWDGT                      TRUE

/* EXMC attributes.*/
#if GD32_HAS_EXMC
    #define GD32_HAS_EXMC                      TRUE
#else
    #define GD32_HAS_EXMC                      FALSE
#endif

/* CRC attributes.*/
#define GD32_HAS_CRC                       TRUE
#define GD32_CRC_PROGRAMMABLE              FALSE

#endif

/** @} */

#endif /* GD32_REGISTRY_H */

/** @} */
