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
 * @file    hal_pal_lld.h
 * @brief   PLATFORM PAL subsystem low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef HAL_PAL_LLD_H
#define HAL_PAL_LLD_H

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

#ifndef PAL_OLD_INIT
#define PAL_NEW_INIT
#endif

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/
/**
 * @name    NUC123 specific I/O mode flags
 * @{
 */

/**
 * @brief   I/O port modes.
 */
#define GPIO_PMD_INPUT                                    0x00ul          /*!< Input Mode */
#define GPIO_PMD_OUTPUT                                   0x01ul          /*!< Output Mode */
#define GPIO_PMD_OPEN_DRAIN                               0x02ul          /*!< Open-Drain Mode */
#define GPIO_PMD_QUASI                                    0x03ul          /*!< Quasi-bidirectional Mode */

/**
 * @brief   NUC123 specific alternate input mode.
 */
#define PAL_MODE_NUC123_ALTERNATE_INPUT                   GPIO_PMD_INPUT

/**
 * @brief   NUC123 specific alternate push-pull output mode.
 */
#define PAL_MODE_NUC123_ALTERNATE_OUTPUT                  GPIO_PMD_OUTPUT
/** @} */

/**
 * @brief   NUC123 specific alternate output mode.
 */
#define PAL_MODE_NUC123_ALTERNATE_OPEN_DRAIN              GPIO_PMD_OPEN_DRAIN
/** @} */

/**
 * @brief   NUC123 specific alternate output mode.
 */
#define PAL_MODE_NUC123_ALTERNATE_QUASI                   GPIO_PMD_QUASI
/** @} */

/**
 * @brief        Alternate GPIO pin defines
 *
 * @description  SYS defines for alternative GPIO pin usage (instead of general I/O)
 *               These are all for two sets of registers - GPx_MFP and ALT_MFP/ALT_MFP1
 *
 */

/**
 * GPIO Port A Alternative Pin Modes
 *
 */

/* Pin 10 */
/* GPIO */
#define SYS_GPA_MFP_PA10_GPIO                             0x0ul
#define SYS_ALT_MFP_PA10_GPIO                             0x0ul
#define SYS_ALT_MFP1_PA10_GPIO                            NULL

/* I2C1 SDA */
#define SYS_GPA_MFP_PA10_I2C1_SDA                         (0x01ul << GPA_MFP10)
#define SYS_ALT_MFP_PA10_I2C1_SDA                         0x0ul
#define SYS_ALT_MFP1_PA10_I2C1_SDA                        NULL

/* SPI1 MISO0 */
#define SYS_GPA_MFP_PA10_SPI1_MISO0                       0x0ul
#define SYS_ALT_MFP_PA10_SPI1_MISO0                       (0x01ul << SYS_ALT_MFP_PA10_MFP1_Pos)
#define SYS_ALT_MFP1_PA10_SPI1_MISO0                      NULL

/* SPI2 MISO0 */
#define SYS_GPA_MFP_PA10_SPI2_MISO0                       (0x01ul << GPA_MFP10)
#define SYS_ALT_MFP_PA10_SPI2_MISO0                       (0x01ul << SYS_ALT_MFP_PA10_MFP1_Pos)
#define SYS_ALT_MFP1_PA10_SPI2_MISO0                      NULL

#define SYS_GPA_MFP_PA10_Msk                              (0x01ul << GPA_MFP10)
#define SYS_ALT_MFP_PA10_Msk                              (0x01ul << SYS_ALT_MFP_PA10_MFP1_Pos)
#define SYS_ALT_MFP1_PA10_Msk                             NULL

/* Pin 11 */
/* GPIO */
#define SYS_GPA_MFP_PA11_GPIO                             0x0ul
#define SYS_ALT_MFP_PA11_GPIO                             0x0ul
#define SYS_ALT_MFP1_PA11_GPIO                            NULL

/* I2C1 SCL */
#define SYS_GPA_MFP_PA11_I2C1_SCL                         (0x01ul << GPA_MFP11)
#define SYS_ALT_MFP_PA11_I2C1_SCL                         0x0ul
#define SYS_ALT_MFP1_PA11_I2C1_SCL                        NULL

/* SPI1 CLK */
#define SYS_GPA_MFP_PA11_SPI1_CLK                         0x0ul
#define SYS_ALT_MFP_PA11_SPI1_CLK                         (0x01ul << GPA_MFP11)
#define SYS_ALT_MFP1_PA11_SPI1_CLK                        NULL

/* SPI2 MOSI0 */
#define SYS_GPA_MFP_PA11_SPI2_MOSI0                       (0x01ul << GPA_MFP11)
#define SYS_ALT_MFP_PA11_SPI2_MOSI0                       (0x01ul << SYS_ALT_MFP_PA11_MFP1_Pos)
#define SYS_ALT_MFP1_PA11_SPI2_MOSI0                      NULL

#define SYS_GPA_MFP_PA11_Msk                              (0x01ul << GPA_MFP11)
#define SYS_ALT_MFP_PA11_Msk                              (0x01ul << SYS_ALT_MFP_PA11_MFP1_Pos)
#define SYS_ALT_MFP1_PA11_Msk                             NULL

/* Pin 12 */
/* GPIO */
#define SYS_GPA_MFP_PA12_GPIO                             0x0ul
#define SYS_ALT_MFP_PA12_GPIO                             NULL
#define SYS_ALT_MFP1_PA12_GPIO                            NULL

/* PWM0 */
#define SYS_GPA_MFP_PA12_PWM0                             (0x01ul << GPA_MFP12)
#define SYS_ALT_MFP_PA12_PWM0                             NULL
#define SYS_ALT_MFP1_PA12_PWM0                            NULL

#define SYS_GPA_MFP_PA12_Msk                              (0x01ul << GPA_MFP12)
#define SYS_ALT_MFP_PA12_Msk                              NULL
#define SYS_ALT_MFP1_PA12_Msk                             NULL

/* Pin 13 */
/* GPIO */
#define SYS_GPA_MFP_PA13_GPIO                             0x0ul
#define SYS_ALT_MFP_PA13_GPIO                             NULL
#define SYS_ALT_MFP1_PA13_GPIO                            NULL

/* PWM1 */
#define SYS_GPA_MFP_PA13_PWM1                             (0x01ul << GPA_MFP13)
#define SYS_ALT_MFP_PA13_PWM1                             NULL
#define SYS_ALT_MFP1_PA13_PWM1                            NULL

#define SYS_GPA_MFP_PA13_Msk                              (0x01ul << GPA_MFP13)
#define SYS_ALT_MFP_PA13_Msk                              NULL
#define SYS_ALT_MFP1_PA13_Msk                             NULL

/* Pin 14 */
/* GPIO */
#define SYS_GPA_MFP_PA14_GPIO                             0x0ul
#define SYS_ALT_MFP_PA14_GPIO                             NULL
#define SYS_ALT_MFP1_PA14_GPIO                            NULL

/* PWM2 */
#define SYS_GPA_MFP_PA14_PWM2                             (0x01ul << GPA_MFP14)
#define SYS_ALT_MFP_PA14_PWM2                             NULL
#define SYS_ALT_MFP1_PA14_PWM2                            NULL

#define SYS_GPA_MFP_PA14_Msk                              (0x01ul << GPA_MFP14)
#define SYS_ALT_MFP_PA14_Msk                              NULL
#define SYS_ALT_MFP1_PA14_Msk                             NULL

/* Pin 15 */
/* GPIO */
#define SYS_GPA_MFP_PA15_GPIO                             0x0ul
#define SYS_ALT_MFP_PA15_GPIO                             0x0ul
#define SYS_ALT_MFP1_PA15_GPIO                            NULL

/* PWM3 */
#define SYS_GPA_MFP_PA15_PWM3                             (0x01ul << GPA_MFP15)
#define SYS_ALT_MFP_PA15_PWM3                             0x0ul
#define SYS_ALT_MFP1_PA15_PWM3                            NULL

/* CLK0 */
#define SYS_GPA_MFP_PA15_CLKO                             0x0ul
#define SYS_ALT_MFP_PA15_CLKO                             (0x01ul << SYS_ALT_MFP_PA15_MFP1_Pos)
#define SYS_ALT_MFP1_PA15_CLKO                            NULL

/* I2S MCLK */
#define SYS_GPA_MFP_PA15_I2S_MCLK                         (0x01ul << GPA_MFP15)
#define SYS_ALT_MFP_PA15_I2S_MCLK                         (0x01ul << SYS_ALT_MFP_PA15_MFP1_Pos)
#define SYS_ALT_MFP1_PA15_I2S_MCLK                        NULL

#define SYS_GPA_MFP_PA15_Msk                              (0x01ul << GPA_MFP15)
#define SYS_ALT_MFP_PA15_Msk                              (0x01ul << SYS_ALT_MFP_PA15_MFP1_Pos)
#define SYS_ALT_MFP1_PA15_Msk                             NULL

/**
 * GPIO Port B Alternative Pin Modes
 *
 */

/* Pin 0 */
/* GPIO */
#define SYS_GPB_MFP_PB0_GPIO                              0x0ul
#define SYS_ALT_MFP_PB0_GPIO                              NULL
#define SYS_ALT_MFP1_PB0_GPIO                             NULL

/* UART0 RXD */
#define SYS_GPB_MFP_PB0_UART0_RXD                         (0x01ul << 0)
#define SYS_ALT_MFP_PB0_UART0_RXD                         NULL
#define SYS_ALT_MFP1_PB0_UART0_RXD                        NULL

#define SYS_GPB_MFP_PB0_Msk                               (0x01ul << 0)
#define SYS_ALT_MFP_PB0_Msk                               NULL
#define SYS_ALT_MFP1_PB0_Msk                              NULL

/* Pin 1 */
/* GPIO */
#define SYS_GPB_MFP_PB1_GPIO                              0x0ul
#define SYS_ALT_MFP_PB1_GPIO                              NULL
#define SYS_ALT_MFP1_PB1_GPIO                             NULL

/* UART0 TXD */
#define SYS_GPB_MFP_PB1_UART0_TXD                         (0x01ul << 1)
#define SYS_ALT_MFP_PB1_UART0_TXD                         NULL
#define SYS_ALT_MFP1_PB1_UART0_TXD                        NULL

#define SYS_GPB_MFP_PB1_Msk                               (0x01ul << 1)
#define SYS_ALT_MFP_PB1_Msk                               NULL
#define SYS_ALT_MFP1_PB1_Msk                              NULL

/* Pin 2 */
/* GPIO */
#define SYS_GPB_MFP_PB2_GPIO                              0x0ul
#define SYS_ALT_MFP_PB2_GPIO                              0x0ul
#define SYS_ALT_MFP1_PB2_GPIO                             NULL

/* UART0 nRTS */
#define SYS_GPB_MFP_PB2_UART0_nRTS                        (0x01ul << 2)
#define SYS_ALT_MFP_PB2_UART0_nRTS                        0x0ul
#define SYS_ALT_MFP1_PB2_UART0_nRTS                       NULL

/* TM2_EXT */
#define SYS_GPB_MFP_PB2_TM2_EXT                           (0x01ul << 2)
#define SYS_ALT_MFP_PB2_TM2_EXT                           (0x01ul << SYS_ALT_MFP_PB2_MFP1_Pos)
#define SYS_ALT_MFP1_PB2_TM2_EXT                          NULL

#define SYS_GPB_MFP_PB2_Msk                               (0x01ul << 2)
#define SYS_ALT_MFP_PB2_Msk                               (0x01ul << SYS_ALT_MFP_PB2_MFP1_Pos)
#define SYS_ALT_MFP1_PB2_Msk                              NULL

/* Pin 3 */
/* GPIO */
#define SYS_GPB_MFP_PB3_GPIO                              0x0ul
#define SYS_ALT_MFP_PB3_GPIO                              0x0ul
#define SYS_ALT_MFP1_PB3_GPIO                             NULL

/* UART0 nCTS */
#define SYS_GPB_MFP_PB3_UART0_nCTS                        (0x01ul << 3)
#define SYS_ALT_MFP_PB3_UART0_nCTS                        0x0ul
#define SYS_ALT_MFP1_PB3_UART0_nCTS                       NULL

/* TM3_EXT */
#define SYS_GPB_MFP_PB3_TM3_EXT                           (0x01ul << 3)
#define SYS_ALT_MFP_PB3_TM3_EXT                           (0x01ul << SYS_ALT_MFP_PB3_MFP1_Pos)
#define SYS_ALT_MFP1_PB3_TM3_EXT                          NULL

#define SYS_GPB_MFP_PB3_Msk                               (0x01ul << 3)
#define SYS_ALT_MFP_PB3_Msk                               (0x01ul << SYS_ALT_MFP_PB3_MFP1_Pos)
#define SYS_ALT_MFP1_PB3_Msk                              NULL

/* Pin 4 */
/* GPIO */
#define SYS_GPB_MFP_PB4_GPIO                              0x0ul
#define SYS_ALT_MFP_PB4_GPIO                              0x0ul
#define SYS_ALT_MFP1_PB4_GPIO                             NULL

/* UART1 RXD */
#define SYS_GPB_MFP_PB4_UART1_RXD                         (0x01ul << 4)
#define SYS_ALT_MFP_PB4_UART1_RXD                         0x0ul
#define SYS_ALT_MFP1_PB4_UART1_RXD                        NULL

/* SPI2 SS0 */
#define SYS_GPB_MFP_PB4_SPI2_SS0                          0x0ul
#define SYS_ALT_MFP_PB4_SPI2_SS0                          (0x01ul << SYS_ALT_MFP_PB4_MFP1_Pos)
#define SYS_ALT_MFP1_PB4_SPI2_SS0                         NULL

/* SPI1 SS1 */
#define SYS_GPB_MFP_PB4_SPI1_SS1                          (0x01ul << 4)
#define SYS_ALT_MFP_PB4_SPI1_SS1                          (0x01ul << SYS_ALT_MFP_PB4_MFP1_Pos)
#define SYS_ALT_MFP1_PB4_SPI1_SS1                         NULL

#define SYS_GPB_MFP_PB4_Msk                               (0x01ul << 4)
#define SYS_ALT_MFP_PB4_Msk                               (0x01ul << SYS_ALT_MFP_PB4_MFP1_Pos)
#define SYS_ALT_MFP1_PB4_Msk                              NULL

/* Pin 5 */
/* GPIO */
#define SYS_GPB_MFP_PB5_GPIO                              0x0ul
#define SYS_ALT_MFP_PB5_GPIO                              0x0ul
#define SYS_ALT_MFP1_PB5_GPIO                             NULL

/* UART1 TXD */
#define SYS_GPB_MFP_PB5_UART1_TXD                         (0x01ul << 5)
#define SYS_ALT_MFP_PB5_UART1_TXD                         0x0ul
#define SYS_ALT_MFP1_PB5_UART1_TXD                        NULL

/* SPI2 CLK */
#define SYS_GPB_MFP_PB5_SPI2_CLK                          (0x01ul << 5)
#define SYS_ALT_MFP_PB5_SPI2_CLK                          (0x01ul << SYS_ALT_MFP_PB5_MFP1_Pos)
#define SYS_ALT_MFP1_PB5_SPI2_CLK                         NULL

#define SYS_GPB_MFP_PB5_Msk                               (0x01ul << 5)
#define SYS_ALT_MFP_PB5_Msk                               (0x01ul << SYS_ALT_MFP_PB5_MFP1_Pos)
#define SYS_ALT_MFP1_PB5_Msk                              NULL

/* Pin 6 */
/* GPIO */
#define SYS_GPB_MFP_PB6_GPIO                              0x0ul
#define SYS_ALT_MFP_PB6_GPIO                              0x0ul
#define SYS_ALT_MFP1_PB6_GPIO                             NULL

/* UART1 nRTS */
#define SYS_GPB_MFP_PB6_UART1_nRTS                        (0x01ul << 6)
#define SYS_ALT_MFP_PB6_UART1_nRTS                        0x0ul
#define SYS_ALT_MFP1_PB6_UART1_nRTS                       NULL

/* SPI2 MOSI0 */
#define SYS_GPB_MFP_PB6_SPI2_MOSI0                        (0x01ul << 6)
#define SYS_ALT_MFP_PB6_SPI2_MOSI0                        (0x01ul << SYS_ALT_MFP_PB6_MFP1_Pos)
#define SYS_ALT_MFP1_PB6_SPI2_MOSI0                       NULL

#define SYS_GPB_MFP_PB6_Msk                               (0x01ul << 6)
#define SYS_ALT_MFP_PB6_Msk                               (0x01ul << SYS_ALT_MFP_PB6_MFP1_Pos)
#define SYS_ALT_MFP1_PB6_Msk                              NULL

/* Pin 7 */
/* GPIO */
#define SYS_GPB_MFP_PB7_GPIO                              0x0ul
#define SYS_ALT_MFP_PB7_GPIO                              0x0ul
#define SYS_ALT_MFP1_PB7_GPIO                             NULL

/* UART1 nCTS */
#define SYS_GPB_MFP_PB7_UART1_nCTS                        (0x01ul << 7)
#define SYS_ALT_MFP_PB7_UART1_nCTS                        0x0ul
#define SYS_ALT_MFP1_PB7_UART1_nCTS                       NULL

/* SPI2 MISO0 */
#define SYS_GPB_MFP_PB7_SPI2_MISO0                        (0x01ul << 7)
#define SYS_ALT_MFP_PB7_SPI2_MISO0                        (0x01ul << SYS_ALT_MFP_PB7_MFP1_Pos)
#define SYS_ALT_MFP1_PB7_SPI2_MISO0                       NULL

#define SYS_GPB_MFP_PB7_Msk                               (0x01ul << 7)
#define SYS_ALT_MFP_PB7_Msk                               (0x01ul << SYS_ALT_MFP_PB7_MFP1_Pos)
#define SYS_ALT_MFP1_PB7_Msk                              NULL

/* Pin 8 */
/* GPIO */
#define SYS_GPB_MFP_PB8_GPIO                              0x0ul
#define SYS_ALT_MFP_PB8_GPIO                              NULL
#define SYS_ALT_MFP1_PB8_GPIO                             NULL

/* TM0 */
#define SYS_GPB_MFP_PB8_TM0                               (0x01ul << 8)
#define SYS_ALT_MFP_PB8_TM0                               NULL
#define SYS_ALT_MFP1_PB8_TM0                              NULL

#define SYS_GPB_MFP_PB8_Msk                               (0x01ul << 8)
#define SYS_ALT_MFP_PB8_Msk                               NULL
#define SYS_ALT_MFP1_PB8_Msk                              NULL

/* Pin 9 */
/* GPIO */
#define SYS_GPB_MFP_PB9_GPIO                              0x0ul
#define SYS_ALT_MFP_PB9_GPIO                              0x0ul
#define SYS_ALT_MFP1_PB9_GPIO                             NULL

/* TM1 */
#define SYS_GPB_MFP_PB9_TM1                               (0x01ul << 9)
#define SYS_ALT_MFP_PB9_TM1                               0x0ul
#define SYS_ALT_MFP1_PB9_TM1                              NULL

/* SPI1 SS1 */
#define SYS_GPB_MFP_PB9_SPI1_SS1                          (0x01ul << 9)
#define SYS_ALT_MFP_PB9_SPI1_SS1                          (0x01ul << SYS_ALT_MFP_PB9_MFP1_Pos)
#define SYS_ALT_MFP1_PB9_SPI1_SS1                         NULL

/* PWM1 */
#define SYS_GPB_MFP_PB9_PWM1                              0x0ul
#define SYS_ALT_MFP_PB9_PWM1                              (0x01ul << SYS_ALT_MFP_PB9_MFP1_Pos)
#define SYS_ALT_MFP1_PB9_PWM1                             NULL

#define SYS_GPB_MFP_PB9_Msk                               (0x01ul << 9)
#define SYS_ALT_MFP_PB9_Msk                               (0x01ul << SYS_ALT_MFP_PB9_MFP1_Pos)
#define SYS_ALT_MFP1_PB9_Msk                              NULL

/* Pin 10 */
/* GPIO */
#define SYS_GPB_MFP_PB10_GPIO                             0x0ul
#define SYS_ALT_MFP_PB10_GPIO                             0x0ul
#define SYS_ALT_MFP1_PB10_GPIO                            NULL

/* TM2 */
#define SYS_GPB_MFP_PB10_TM2                              (0x01ul << 10)
#define SYS_ALT_MFP_PB10_TM2                              0x0ul
#define SYS_ALT_MFP1_PB10_TM2                             NULL

/* SPI0 SS1 */
#define SYS_GPB_MFP_PB10_SPI0_SS1                         (0x01ul << 10)
#define SYS_ALT_MFP_PB10_SPI0_SS1                         (0x01ul << SYS_ALT_MFP_PB10_MFP1_Pos)
#define SYS_ALT_MFP1_PB10_SPI0_SS1                        NULL

#define SYS_GPB_MFP_PB10_Msk                              (0x01ul << 10)
#define SYS_ALT_MFP_PB10_Msk                              (0x01ul << SYS_ALT_MFP_PB10_MFP1_Pos)
#define SYS_ALT_MFP1_PB10_Msk                             NULL

/* Pin 12 */
/* GPIO */
#define SYS_GPB_MFP_PB12_GPIO                             0x0ul
#define SYS_ALT_MFP_PB12_GPIO                             0x0ul
#define SYS_ALT_MFP1_PB12_GPIO                            NULL

/* SPI1 SS0 */
#define SYS_GPB_MFP_PB12_SPI1_SS0                         (0x01ul << 12)
#define SYS_ALT_MFP_PB12_SPI1_SS0                         0x0ul
#define SYS_ALT_MFP1_PB12_SPI1_SS0                        NULL

/* CLK0 */
#define SYS_GPB_MFP_PB12_CLKO                             (0x01ul << 12)
#define SYS_ALT_MFP_PB12_CLKO                             (0x01ul << SYS_ALT_MFP_PB12_MFP1_Pos)
#define SYS_ALT_MFP1_PB12_CLKO                            NULL

#define SYS_GPB_MFP_PB12_Msk                              (0x01ul << 12)
#define SYS_ALT_MFP_PB12_Msk                              (0x01ul << SYS_ALT_MFP_PB12_MFP1_Pos)
#define SYS_ALT_MFP1_PB12_Msk                             NULL

/* Pin 13 */
/* GPIO */
#define SYS_GPB_MFP_PB13_GPIO                             0x0ul
#define SYS_ALT_MFP_PB13_GPIO                             NULL
#define SYS_ALT_MFP1_PB13_GPIO                            NULL

#define SYS_GPB_MFP_PB13_Msk                              (0x01ul << 13)
#define SYS_ALT_MFP_PB13_Msk                              NULL
#define SYS_ALT_MFP1_PB13_Msk                             NULL

/* Pin 14 */
/* GPIO */
#define SYS_GPB_MFP_PB14_GPIO                             0x0ul
#define SYS_ALT_MFP_PB14_GPIO                             NULL
#define SYS_ALT_MFP1_PB14_GPIO                            NULL

/* INT0 */
#define SYS_GPB_MFP_PB14_INT0                             (0x01ul << 14)
#define SYS_ALT_MFP_PB14_INT0                             NULL
#define SYS_ALT_MFP1_PB14_INT0                            NULL

#define SYS_GPB_MFP_PB14_Msk                              (0x01ul << 14)
#define SYS_ALT_MFP_PB14_Msk                              NULL
#define SYS_ALT_MFP1_PB14_Msk                             NULL

/* Pin 15 */
/* GPIO */
#define SYS_GPB_MFP_PB15_GPIO                             0x0ul
#define SYS_ALT_MFP_PB15_GPIO                             0x0ul
#define SYS_ALT_MFP1_PB15_GPIO                            NULL

/* INT1 */
#define SYS_GPB_MFP_PB15_INT1                             (0x01ul << 15)
#define SYS_ALT_MFP_PB15_INT1                             0x0ul
#define SYS_ALT_MFP1_PB15_INT1                            NULL

/* TM0_EXT */
#define SYS_GPB_MFP_PB15_TM0_EXT                          (0x01ul << 15)
#define SYS_ALT_MFP_PB15_TM0_EXT                          (0x01ul << SYS_ALT_MFP_PB15_MFP1_Pos)
#define SYS_ALT_MFP1_PB15_TM0_EXT                         NULL

#define SYS_GPB_MFP_PB15_Msk                              (0x01ul << 15)
#define SYS_ALT_MFP_PB15_Msk                              (0x01ul << SYS_ALT_MFP_PB15_MFP1_Pos)
#define SYS_ALT_MFP1_PB15_Msk                             NULL

/**
 * GPIO Port C Alternative Pin Modes
 *
 */

/* Pin 0 */
/* GPIO */
#define SYS_GPC_MFP_PC0_GPIO                              0x0ul
#define SYS_ALT_MFP_PC0_GPIO                              0x0ul
#define SYS_ALT_MFP1_PC0_GPIO                             NULL

/* SPI0 SS0 */
#define SYS_GPC_MFP_PC0_SPI0_SS0                          (0x01ul << GPC_MFP0)
#define SYS_ALT_MFP_PC0_SPI0_SS0                          0x0ul
#define SYS_ALT_MFP1_PC0_SPI0_SS0                         NULL

/* I2S LRCLK */
#define SYS_GPC_MFP_PC0_I2S_LRCLK                         (0x01ul << GPC_MFP0)
#define SYS_ALT_MFP_PC0_I2S_LRCLK                         (0x01ul << SYS_ALT_MFP_PC0_MFP1_Pos)
#define SYS_ALT_MFP1_PC0_I2S_LRCLK                        NULL

#define SYS_GPC_MFP_PC0_Msk                               (0x01ul << GPC_MFP0)
#define SYS_ALT_MFP_PC0_Msk                               (0x01ul << SYS_ALT_MFP_PC0_MFP1_Pos)
#define SYS_ALT_MFP1_PC0_Msk                              NULL

/* Pin 1 */
/* GPIO */
#define SYS_GPC_MFP_PC1_GPIO                              0x0ul
#define SYS_ALT_MFP_PC1_GPIO                              0x0ul
#define SYS_ALT_MFP1_PC1_GPIO                             NULL

/* SPI0 CLK */
#define SYS_GPC_MFP_PC1_SPI0_CLK                          (0x01ul << GPC_MFP1)
#define SYS_ALT_MFP_PC1_SPI0_CLK                          0x0ul
#define SYS_ALT_MFP1_PC1_SPI0_CLK                         NULL

/* I2S BCLK */
#define SYS_GPC_MFP_PC1_I2S_BCLK                          (0x01ul << GPC_MFP1)
#define SYS_ALT_MFP_PC1_I2S_BCLK                          (0x01ul << SYS_ALT_MFP_PC1_MFP1_Pos)
#define SYS_ALT_MFP1_PC1_I2S_BCLK                         NULL

#define SYS_GPC_MFP_PC1_Msk                               (0x01ul << GPC_MFP1)
#define SYS_ALT_MFP_PC1_Msk                               (0x01ul << SYS_ALT_MFP_PC1_MFP1_Pos)
#define SYS_ALT_MFP1_PC1_Msk                              NULL

/* Pin 2 */
/* GPIO */
#define SYS_GPC_MFP_PC2_GPIO                              0x0ul
#define SYS_ALT_MFP_PC2_GPIO                              0x0ul
#define SYS_ALT_MFP1_PC2_GPIO                             NULL

/* SPI0 MISO0 */
#define SYS_GPC_MFP_PC2_SPI0_MISO0                        (0x01ul << GPC_MFP2)
#define SYS_ALT_MFP_PC2_SPI0_MISO0                        0x0ul
#define SYS_ALT_MFP1_PC2_SPI0_MISO0                       NULL

/* I2S DI */
#define SYS_GPC_MFP_PC2_I2S_DI                            (0x01ul << GPC_MFP2)
#define SYS_ALT_MFP_PC2_I2S_DI                            (0x01ul << SYS_ALT_MFP_PC2_MFP1_Pos)
#define SYS_ALT_MFP1_PC2_I2S_DI                           NULL

#define SYS_GPC_MFP_PC2_Msk                               (0x01ul << GPC_MFP2)
#define SYS_ALT_MFP_PC2_Msk                               (0x01ul << SYS_ALT_MFP_PC2_MFP1_Pos)
#define SYS_ALT_MFP1_PC2_Msk                              NULL

/* Pin 3 */
/* GPIO */
#define SYS_GPC_MFP_PC3_GPIO                              0x0ul
#define SYS_ALT_MFP_PC3_GPIO                              0x0ul
#define SYS_ALT_MFP1_PC3_GPIO                             NULL

/* SPI0 MOSI0 */
#define SYS_GPC_MFP_PC3_SPI0_MOSI0                        (0x01ul << GPC_MFP3)
#define SYS_ALT_MFP_PC3_SPI0_MOSI0                        0x0ul
#define SYS_ALT_MFP1_PC3_SPI0_MOSI0                       NULL

/* I2S DO */
#define SYS_GPC_MFP_PC3_I2S_DO                            (0x01ul << GPC_MFP3)
#define SYS_ALT_MFP_PC3_I2S_DO                            (0x01ul << SYS_ALT_MFP_PC3_MFP1_Pos)
#define SYS_ALT_MFP1_PC3_I2S_DO                           NULL

#define SYS_GPC_MFP_PC3_Msk                               (0x01ul << GPC_MFP3)
#define SYS_ALT_MFP_PC3_Msk                               (0x01ul << SYS_ALT_MFP_PC3_MFP1_Pos)
#define SYS_ALT_MFP1_PC3_Msk                              NULL

/* Pin 4 */
/* GPIO */
#define SYS_GPC_MFP_PC4_GPIO                              0x0ul
#define SYS_ALT_MFP_PC4_GPIO                              0x0ul
#define SYS_ALT_MFP1_PC4_GPIO                             NULL

/* SPI0 MISO1 */
#define SYS_GPC_MFP_PC4_SPI0_MISO1                        (0x01ul << GPC_MFP4)
#define SYS_ALT_MFP_PC4_SPI0_MISO1                        0x0ul
#define SYS_ALT_MFP1_PC4_SPI0_MISO1                       NULL

/* UART0 RXD */
#define SYS_GPC_MFP_PC4_UART0_RXD                         (0x01ul << GPC_MFP4)
#define SYS_ALT_MFP_PC4_UART0_RXD                         (0x01ul << SYS_ALT_MFP_PC4_MFP1_Pos)
#define SYS_ALT_MFP1_PC4_UART0_RXD                        NULL

#define SYS_GPC_MFP_PC4_Msk                               (0x01ul << GPC_MFP4)
#define SYS_ALT_MFP_PC4_Msk                               (0x01ul << SYS_ALT_MFP_PC4_MFP1_Pos)
#define SYS_ALT_MFP1_PC4_Msk                              NULL

/* Pin 5 */
/* GPIO */
#define SYS_GPC_MFP_PC5_GPIO                              0x0ul
#define SYS_ALT_MFP_PC5_GPIO                              0x0ul
#define SYS_ALT_MFP1_PC5_GPIO                             NULL

/* SPI0 MOSI1 */
#define SYS_GPC_MFP_PC5_SPI0_MOSI1                        (0x01ul << GPC_MFP5)
#define SYS_ALT_MFP_PC5_SPI0_MOSI1                        0x0ul
#define SYS_ALT_MFP1_PC5_SPI0_MOSI1                       NULL

/* UART0 TXD */
#define SYS_GPC_MFP_PC5_UART0_TXD                         (0x01ul << GPC_MFP5)
#define SYS_ALT_MFP_PC5_UART0_TXD                         (0x01ul << SYS_ALT_MFP_PC5_MFP1_Pos)
#define SYS_ALT_MFP1_PC5_UART0_TXD                        NULL

#define SYS_GPC_MFP_PC5_Msk                               (0x01ul << GPC_MFP5)
#define SYS_ALT_MFP_PC5_Msk                               (0x01ul << SYS_ALT_MFP_PC5_MFP1_Pos)
#define SYS_ALT_MFP1_PC5_Msk                              NULL

/* Pin 8 */
/* GPIO */
#define SYS_GPC_MFP_PC8_GPIO                              0x0ul
#define SYS_ALT_MFP_PC8_GPIO                              NULL
#define SYS_ALT_MFP1_PC8_GPIO                             0x0ul

/* SPI1 SS0 */
#define SYS_GPC_MFP_PC8_SPI1_SS0                          (0x01ul << GPC_MFP8)
#define SYS_ALT_MFP_PC8_SPI1_SS0                          NULL
#define SYS_ALT_MFP1_PC8_SPI1_SS0                         0x0ul

/* PWM0 */
#define SYS_GPC_MFP_PC8_PWM0                              (0x01ul << GPC_MFP8)
#define SYS_ALT_MFP_PC8_PWM0                              NULL
#define SYS_ALT_MFP1_PC8_PWM0                             (0x01ul << SYS_ALT_MFP1_PC8_MFP1_Pos)

#define SYS_GPC_MFP_PC8_Msk                               (0x01ul << GPC_MFP8)
#define SYS_ALT_MFP_PC8_Msk                               NULL
#define SYS_ALT_MFP1_PC8_Msk                              (0x01ul << SYS_ALT_MFP1_PC8_MFP1_Pos)

/* Pin 9 */
/* GPIO */
#define SYS_GPC_MFP_PC9_GPIO                              0x0ul
#define SYS_ALT_MFP_PC9_GPIO                              NULL
#define SYS_ALT_MFP1_PC9_GPIO                             NULL

/* SPI1 CLK */
#define SYS_GPC_MFP_PC9_SPI1_CLK                          (0x01ul << GPC_MFP9)
#define SYS_ALT_MFP_PC9_SPI1_CLK                          NULL
#define SYS_ALT_MFP1_PC9_SPI1_CLK                         NULL

#define SYS_GPC_MFP_PC9_Msk                               (0x01ul << GPC_MFP9)
#define SYS_ALT_MFP_PC9_Msk                               NULL
#define SYS_ALT_MFP1_PC9_Msk                              NULL

/* Pin 10 */
/* GPIO */
#define SYS_GPC_MFP_PC10_GPIO                             0x0ul
#define SYS_ALT_MFP_PC10_GPIO                             NULL
#define SYS_ALT_MFP1_PC10_GPIO                            NULL

/* SPI1 MISO0 */
#define SYS_GPC_MFP_PC10_SPI1_MISO0                       (0x01ul << GPC_MFP10)
#define SYS_ALT_MFP_PC10_SPI1_MISO0                       NULL
#define SYS_ALT_MFP1_PC10_SPI1_MISO0                      NULL

#define SYS_GPC_MFP_PC10_Msk                              (0x01ul << GPC_MFP10)
#define SYS_ALT_MFP_PC10_Msk                              NULL
#define SYS_ALT_MFP1_PC10_Msk                             NULL

/* Pin 11 */
/* GPIO */
#define SYS_GPC_MFP_PC11_GPIO                             0x0ul
#define SYS_ALT_MFP_PC11_GPIO                             NULL
#define SYS_ALT_MFP1_PC11_GPIO                            NULL

/* SPI1 MOSI0 */
#define SYS_GPC_MFP_PC11_SPI1_MOSI0                       (0x01ul << GPC_MFP11)
#define SYS_ALT_MFP_PC11_SPI1_MOSI0                       NULL
#define SYS_ALT_MFP1_PC11_SPI1_MOSI0                      NULL

#define SYS_GPC_MFP_PC11_Msk                              (0x01ul << GPC_MFP11)
#define SYS_ALT_MFP_PC11_Msk                              NULL
#define SYS_ALT_MFP1_PC11_Msk                             NULL

/* Pin 12 */
/* GPIO */
#define SYS_GPC_MFP_PC12_GPIO                             0x0ul
#define SYS_ALT_MFP_PC12_GPIO                             0x0ul
#define SYS_ALT_MFP1_PC12_GPIO                            NULL

/* SPI1 MISO1 */
#define SYS_GPC_MFP_PC12_SPI1_MISO1                       (0x01ul << GPC_MFP12)
#define SYS_ALT_MFP_PC12_SPI1_MISO1                       0x0ul
#define SYS_ALT_MFP1_PC12_SPI1_MISO1                      NULL

/* I2S MCLK */
#define SYS_GPC_MFP_PC12_I2S_MCLK                         0x0ul
#define SYS_ALT_MFP_PC12_I2S_MCLK                         (0x01ul << SYS_ALT_MFP_PC12_MFP1_Pos)
#define SYS_ALT_MFP1_PC12_I2S_MCLK                        NULL

/* PWM2 */
#define SYS_GPC_MFP_PC12_PWM2                             (0x01ul << GPC_MFP12)
#define SYS_ALT_MFP_PC12_PWM2                             (0x01ul << SYS_ALT_MFP_PC12_MFP1_Pos)
#define SYS_ALT_MFP1_PC12_PWM2                            NULL

#define SYS_GPC_MFP_PC12_Msk                              (0x01ul << GPC_MFP12)
#define SYS_ALT_MFP_PC12_Msk                              (0x01ul << SYS_ALT_MFP_PC12_MFP1_Pos)
#define SYS_ALT_MFP1_PC12_Msk                             NULL

/* Pin 12 */
/* GPIO */
#define SYS_GPC_MFP_PC13_GPIO                             0x0ul
#define SYS_ALT_MFP_PC13_GPIO                             0x0ul
#define SYS_ALT_MFP1_PC13_GPIO                            NULL

/* SPI1 MOSI1 */
#define SYS_GPC_MFP_PC13_SPI1_MOSI1                       (0x01ul << GPC_MFP13)
#define SYS_ALT_MFP_PC13_SPI1_MOSI1                       0x0ul
#define SYS_ALT_MFP1_PC13_SPI1_MOSI1                      NULL

/* CLK0 */
#define SYS_GPC_MFP_PC13_CLKO                             0x0ul
#define SYS_ALT_MFP_PC13_CLKO                             (0x01ul << SYS_ALT_MFP_PC13_MFP1_Pos)
#define SYS_ALT_MFP1_PC13_CLKO                            NULL

/* PWM3 */
#define SYS_GPC_MFP_PC13_PWM3                             (0x01ul << GPC_MFP13)
#define SYS_ALT_MFP_PC13_PWM3                             (0x01ul << SYS_ALT_MFP_PC13_MFP1_Pos)
#define SYS_ALT_MFP1_PC13_PWM3                            NULL

#define SYS_GPC_MFP_PC13_Msk                              (0x01ul << GPC_MFP13)
#define SYS_ALT_MFP_PC13_Msk                              (0x01ul << SYS_ALT_MFP_PC13_MFP1_Pos)
#define SYS_ALT_MFP1_PC13_Msk                             NULL

/**
 * GPIO Port D Alternative Pin Modes
 *
 */

/* Pin 0 */
/* GPIO */
#define SYS_GPD_MFP_PD0_GPIO                              0x0ul
#define SYS_ALT_MFP_PD0_GPIO                              NULL
#define SYS_ALT_MFP1_PD0_GPIO                             0x0ul

/* SPI2 SS0 */
#define SYS_GPD_MFP_PD0_SPI2_SS0                          0x0ul
#define SYS_ALT_MFP_PD0_SPI2_SS0                          NULL
#define SYS_ALT_MFP1_PD0_SPI2_SS0                         (0x01ul << SYS_ALT_MFP1_PD0_MFP1_Pos)

/* ADC0 */
#define SYS_GPD_MFP_PD0_ADC0                              (0x01ul << GPD_MFP0)
#define SYS_ALT_MFP_PD0_ADC0                              NULL
#define SYS_ALT_MFP1_PD0_ADC0                             (0x01ul << SYS_ALT_MFP1_PD0_MFP1_Pos)

#define SYS_GPD_MFP_PD0_Msk                               (0x01ul << GPD_MFP0)
#define SYS_ALT_MFP_PD0_Msk                               NULL
#define SYS_ALT_MFP1_PD0_Msk                              (0x01ul << SYS_ALT_MFP1_PD0_MFP1_Pos)

/* Pin 1 */
/* GPIO */
#define SYS_GPD_MFP_PD1_GPIO                              0x0ul
#define SYS_ALT_MFP_PD1_GPIO                              NULL
#define SYS_ALT_MFP1_PD1_GPIO                             0x0ul

/* SPI0 SS1 */
#define SYS_GPD_MFP_PD1_SPI0_SS1                          (0x01ul << GPD_MFP1)
#define SYS_ALT_MFP_PD1_SPI0_SS1                          NULL
#define SYS_ALT_MFP1_PD1_SPI0_SS1                         0x0ul

/* SPI2 CLK */
#define SYS_GPD_MFP_PD1_SPI2_CLK                          0x0ul
#define SYS_ALT_MFP_PD1_SPI2_CLK                          NULL
#define SYS_ALT_MFP1_PD1_SPI2_CLK                         (0x01ul << SYS_ALT_MFP1_PD1_MFP1_Pos)

/* ADC1 */
#define SYS_GPD_MFP_PD1_ADC1                              (0x01ul << GPD_MFP1)
#define SYS_ALT_MFP_PD1_ADC1                              NULL
#define SYS_ALT_MFP1_PD1_ADC1                             (0x01ul << SYS_ALT_MFP1_PD1_MFP1_Pos)

#define SYS_GPD_MFP_PD1_Msk                               (0x01ul << GPD_MFP1)
#define SYS_ALT_MFP_PD1_Msk                               NULL
#define SYS_ALT_MFP1_PD1_Msk                              (0x01ul << SYS_ALT_MFP1_PD1_MFP1_Pos)

/* Pin 2 */
/* GPIO */
#define SYS_GPD_MFP_PD2_GPIO                              0x0ul
#define SYS_ALT_MFP_PD2_GPIO                              NULL
#define SYS_ALT_MFP1_PD2_GPIO                             0x0ul

/* SPI0 MISO1 */
#define SYS_GPD_MFP_PD2_SPI0_MISO1                        (0x01ul << GPD_MFP2)
#define SYS_ALT_MFP_PD2_SPI0_MISO1                        NULL
#define SYS_ALT_MFP1_PD2_SPI0_MISO1                       0x0ul

/* SPI2 MISO0 */
#define SYS_GPD_MFP_PD2_SPI2_MISO0                        0x0ul
#define SYS_ALT_MFP_PD2_SPI2_MISO0                        NULL
#define SYS_ALT_MFP1_PD2_SPI2_MISO0                       (0x01ul << SYS_ALT_MFP1_PD2_MFP1_Pos)

/* ADC2 */
#define SYS_GPD_MFP_PD2_ADC2                              (0x01ul << GPD_MFP2)
#define SYS_ALT_MFP_PD2_ADC2                              NULL
#define SYS_ALT_MFP1_PD2_ADC2                             (0x01ul << SYS_ALT_MFP1_PD2_MFP1_Pos)

#define SYS_GPD_MFP_PD2_Msk                               (0x01ul << GPD_MFP2)
#define SYS_ALT_MFP_PD2_Msk                               NULL
#define SYS_ALT_MFP1_PD2_Msk                              (0x01ul << SYS_ALT_MFP1_PD2_MFP1_Pos)

/* Pin 3 */
/* GPIO */
#define SYS_GPD_MFP_PD3_GPIO                              0x0ul
#define SYS_ALT_MFP_PD3_GPIO                              NULL
#define SYS_ALT_MFP1_PD3_GPIO                             0x0ul

/* SPI0 MOSI1 */
#define SYS_GPD_MFP_PD3_SPI0_MOSI1                        (0x01ul << GPD_MFP3)
#define SYS_ALT_MFP_PD3_SPI0_MOSI1                        NULL
#define SYS_ALT_MFP1_PD3_SPI0_MOSI1                       0x0ul

/* SPI2 MOSI0 */
#define SYS_GPD_MFP_PD3_SPI2_MOSI0                        0x0ul
#define SYS_ALT_MFP_PD3_SPI2_MOSI0                        NULL
#define SYS_ALT_MFP1_PD3_SPI2_MOSI0                       (0x01ul << SYS_ALT_MFP1_PD3_MFP1_Pos)

/* ADC3 */
#define SYS_GPD_MFP_PD3_ADC3                              (0x01ul << GPD_MFP3)
#define SYS_ALT_MFP_PD3_ADC3                              NULL
#define SYS_ALT_MFP1_PD3_ADC3                             (0x01ul << SYS_ALT_MFP1_PD3_MFP1_Pos)

#define SYS_GPD_MFP_PD3_Msk                               (0x01ul << GPD_MFP3)
#define SYS_ALT_MFP_PD3_Msk                               NULL
#define SYS_ALT_MFP1_PD3_Msk                              (0x01ul << SYS_ALT_MFP1_PD3_MFP1_Pos)

/* Pin 4 */
/* GPIO */
#define SYS_GPD_MFP_PD4_GPIO                              0x0ul
#define SYS_ALT_MFP_PD4_GPIO                              NULL
#define SYS_ALT_MFP1_PD4_GPIO                             0x0ul

/* SPI2 MISO1 */
#define SYS_GPD_MFP_PD4_SPI2_MISO1                        0x0ul
#define SYS_ALT_MFP_PD4_SPI2_MISO1                        NULL
#define SYS_ALT_MFP1_PD4_SPI2_MISO1                       (0x01ul << SYS_ALT_MFP1_PD4_MFP1_Pos)

/* ADC4 */
#define SYS_GPD_MFP_PD4_ADC4                              (0x01ul << GPD_MFP4)
#define SYS_ALT_MFP_PD4_ADC4                              NULL
#define SYS_ALT_MFP1_PD4_ADC4                             (0x01ul << SYS_ALT_MFP1_PD4_MFP1_Pos)

#define SYS_GPD_MFP_PD4_Msk                               (0x01ul << GPD_MFP4)
#define SYS_ALT_MFP_PD4_Msk                               NULL
#define SYS_ALT_MFP1_PD4_Msk                              (0x01ul << SYS_ALT_MFP1_PD4_MFP1_Pos)

/* Pin 5 */
/* GPIO */
#define SYS_GPD_MFP_PD5_GPIO                              0x0ul
#define SYS_ALT_MFP_PD5_GPIO                              NULL
#define SYS_ALT_MFP1_PD5_GPIO                             0x0ul

/* SPI2 MOSI1 */
#define SYS_GPD_MFP_PD5_SPI2_MOSI1                        0x0ul
#define SYS_ALT_MFP_PD5_SPI2_MOSI1                        NULL
#define SYS_ALT_MFP1_PD5_SPI2_MOSI1                       (0x01ul << SYS_ALT_MFP1_PD5_MFP1_Pos)

/* ADC5 */
#define SYS_GPD_MFP_PD5_ADC5                              (0x01ul << GPD_MFP5)
#define SYS_ALT_MFP_PD5_ADC5                              NULL
#define SYS_ALT_MFP1_PD5_ADC5                             (0x01ul << SYS_ALT_MFP1_PD5_MFP1_Pos)

#define SYS_GPD_MFP_PD5_Msk                               (0x01ul << GPD_MFP5)
#define SYS_ALT_MFP_PD5_Msk                               NULL
#define SYS_ALT_MFP1_PD5_Msk                              (0x01ul << SYS_ALT_MFP1_PD5_MFP1_Pos)

/* Pin 8 */
/* GPIO */
#define SYS_GPD_MFP_PD8_GPIO                              0x0ul
#define SYS_ALT_MFP_PD8_GPIO                              NULL
#define SYS_ALT_MFP1_PD8_GPIO                             NULL

/* SPI1 MOSI0 */
#define SYS_GPD_MFP_PD8_SPI1_MOSI0                        (0x01ul << GPD_MFP8)
#define SYS_ALT_MFP_PD8_SPI1_MOSI0                        NULL
#define SYS_ALT_MFP1_PD8_SPI1_MOSI0                       NULL

#define SYS_GPD_MFP_PD8_Msk                               (0x01ul << GPD_MFP8)
#define SYS_ALT_MFP_PD8_Msk                               NULL
#define SYS_ALT_MFP1_PD8_Msk                              NULL

/* Pin 9 */
/* GPIO */
#define SYS_GPD_MFP_PD9_GPIO                              0x0ul
#define SYS_ALT_MFP_PD9_GPIO                              NULL
#define SYS_ALT_MFP1_PD9_GPIO                             NULL

#define SYS_GPD_MFP_PD9_Msk                               (0x01ul << GPD_MFP9)
#define SYS_ALT_MFP_PD9_Msk                               NULL
#define SYS_ALT_MFP1_PD9_Msk                              NULL

/* Pin 10 */
/* GPIO */
#define SYS_GPD_MFP_PD10_GPIO                             0x0ul
#define SYS_ALT_MFP_PD10_GPIO                             NULL
#define SYS_ALT_MFP1_PD10_GPIO                            NULL

/* CLK0 */
#define SYS_GPD_MFP_PD10_CLKO                             (0x01ul << GPD_MFP10)
#define SYS_ALT_MFP_PD10_CLKO                             NULL
#define SYS_ALT_MFP1_PD10_CLKO                            NULL

#define SYS_GPD_MFP_PD10_Msk                              (0x01ul << GPD_MFP10)
#define SYS_ALT_MFP_PD10_Msk                              NULL
#define SYS_ALT_MFP1_PD10_Msk                             NULL

/* Pin 11 */
/* GPIO */
#define SYS_GPD_MFP_PD11_GPIO                             0x0ul
#define SYS_ALT_MFP_PD11_GPIO                             NULL
#define SYS_ALT_MFP1_PD11_GPIO                            NULL

/* INT1 */
#define SYS_GPD_MFP_PD11_INT1                             (0x01ul << GPD_MFP11)
#define SYS_ALT_MFP_PD11_INT1                             NULL
#define SYS_ALT_MFP1_PD11_INT1                            NULL

#define SYS_GPD_MFP_PD11_Msk                              (0x01ul << GPD_MFP11)
#define SYS_ALT_MFP_PD11_Msk                              NULL
#define SYS_ALT_MFP1_PD11_Msk                             NULL

/**
 * GPIO Port F Alternative Pin Modes
 *
 */

/* Pin 0 */
/* GPIO */
#define SYS_GPF_MFP_PF0_GPIO                              0x0ul
#define SYS_ALT_MFP_PF0_GPIO                              NULL
#define SYS_ALT_MFP1_PF0_GPIO                             NULL

/* XT1 OUT */
#define SYS_GPF_MFP_PF0_XT1_OUT                           SYS_GPF_MFP_GPF_MFP0_Msk
#define SYS_ALT_MFP_PF0_XT1_OUT                           NULL
#define SYS_ALT_MFP1_PF0_XT1_OUT                          NULL

#define SYS_GPF_MFP_PF0_Msk                               SYS_GPF_MFP_GPF_MFP0_Msk
#define SYS_ALT_MFP_PF0_Msk                               NULL
#define SYS_ALT_MFP1_PF0_Msk                              NULL

/* Pin 1 */
/* GPIO */
#define SYS_GPF_MFP_PF1_GPIO                              0x0ul
#define SYS_ALT_MFP_PF1_GPIO                              NULL
#define SYS_ALT_MFP1_PF1_GPIO                             NULL

/* XT1 IN */
#define SYS_GPF_MFP_PF1_XT1_IN                            SYS_GPF_MFP_GPF_MFP1_Msk
#define SYS_ALT_MFP_PF1_XT1_IN                            NULL
#define SYS_ALT_MFP1_PF1_XT1_IN                           NULL

#define SYS_GPF_MFP_PF1_Msk                               SYS_GPF_MFP_GPF_MFP1_Msk
#define SYS_ALT_MFP_PF1_Msk                               NULL
#define SYS_ALT_MFP1_PF1_Msk                              NULL

/* Pin 2 */
/* GPIO */
#define SYS_GPF_MFP_PF2_GPIO                              0x0ul
#define SYS_ALT_MFP_PF2_GPIO                              NULL
#define SYS_ALT_MFP1_PF2_GPIO                             0x0ul

/* PS2 DAT */
#define SYS_GPF_MFP_PF2_PS2_DAT                           SYS_GPF_MFP_GPF_MFP2_Msk
#define SYS_ALT_MFP_PF2_PS2_DAT                           NULL
#define SYS_ALT_MFP1_PF2_PS2_DAT                          0x0ul

/* I2C0 SDA */
#define SYS_GPF_MFP_PF2_I2C0_SDA                          SYS_GPF_MFP_GPF_MFP2_Msk
#define SYS_ALT_MFP_PF2_I2C0_SDA                          NULL
#define SYS_ALT_MFP1_PF2_I2C0_SDA                         (0x02ul << SYS_ALT_MFP1_PF2_MFP1_Pos)

/* ADC6 */
#define SYS_GPF_MFP_PF2_ADC6                              SYS_GPF_MFP_GPF_MFP2_Msk
#define SYS_ALT_MFP_PF2_ADC6                              NULL
#define SYS_ALT_MFP1_PF2_ADC6                             (0x03ul << SYS_ALT_MFP1_PF2_MFP1_Pos)

#define SYS_GPF_MFP_PF2_Msk                               SYS_GPF_MFP_GPF_MFP2_Msk
#define SYS_ALT_MFP_PF2_Msk                               NULL
#define SYS_ALT_MFP1_PF2_Msk                              (0x03ul << SYS_ALT_MFP1_PF2_MFP1_Pos)

/* Pin 3 */
/* GPIO */
#define SYS_GPF_MFP_PF3_GPIO                              0x0ul
#define SYS_ALT_MFP_PF3_GPIO                              NULL
#define SYS_ALT_MFP1_PF3_GPIO                             0x0ul

/* PS2 CLK */
#define SYS_GPF_MFP_PF3_PS2_CLK                           SYS_GPF_MFP_GPF_MFP3_Msk
#define SYS_ALT_MFP_PF3_PS2_CLK                           NULL
#define SYS_ALT_MFP1_PF3_PS2_CLK                          0x0ul

/* I2C0 SCL */
#define SYS_GPF_MFP_PF3_I2C0_SCL                          SYS_GPF_MFP_GPF_MFP3_Msk
#define SYS_ALT_MFP_PF3_I2C0_SCL                          NULL
#define SYS_ALT_MFP1_PF3_I2C0_SCL                         (0x2UL << SYS_ALT_MFP1_PF3_MFP1_Pos)

/* ADC7 */
#define SYS_GPF_MFP_PF3_ADC7                              (SYS_GPF_MFP_GPF_MFP3_Msk
#define SYS_ALT_MFP_PF3_ADC7                              NULL
#define SYS_ALT_MFP1_PF3_ADC7                             (0x03ul << SYS_ALT_MFP1_PF3_MFP1_Pos)

#define SYS_GPF_MFP_PF3_Msk                               SYS_GPF_MFP_GPF_MFP3_Msk
#define SYS_ALT_MFP_PF3_Msk                               NULL
#define SYS_ALT_MFP1_PF3_Msk                              (0x03ul << SYS_ALT_MFP1_PF3_MFP1_Pos)

/*
#undef PAL_MODE_RESET
#undef PAL_MODE_UNCONNECTED
#undef PAL_MODE_INPUT
#undef PAL_MODE_INPUT_PULLUP
#undef PAL_MODE_INPUT_PULLDOWN
#undef PAL_MODE_INPUT_ANALOG
#undef PAL_MODE_OUTPUT_PUSHPULL
#undef PAL_MODE_OUTPUT_OPENDRAIN

#define PAL_MODE_RESET 3U
#define PAL_MODE_INPUT 0U
#define PAL_MODE_OUTPUT_PUSHPULL 1U
#define PAL_MODE_OUTPUT_OPENDRAIN 2U
#define PAL_MODE_INPUT_PULLUP 3U
*/

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

/**
 * @name    Port related definitions
 * @{
 */
/* Maximum pins per GPIO port */
#define GPIO_PINSPERPORT_MAX        16

/* GPIO PORT/PIN TO BASE ADDRESS MACRO */
#define GPIO_PIN_DATA(port, pin)    \
  (*((volatile uint32_t *)((GPIO_PIN_DATA_BASE + (0x40 * (port))) + ((pin) << 2))))

/* GPIO Port A (10~15) */
#define PA10                        GPIO_PIN_DATA(0, 10)
#define PA11                        GPIO_PIN_DATA(0, 11)
#define PA12                        GPIO_PIN_DATA(0, 12)
#define PA13                        GPIO_PIN_DATA(0, 13)
#define PA14                        GPIO_PIN_DATA(0, 14)
#define PA15                        GPIO_PIN_DATA(0, 15)

/* GPIO Port B (0~10 & 12~15) */
#define PB0                         GPIO_PIN_DATA(1, 0)
#define PB1                         GPIO_PIN_DATA(1, 1)
#define PB2                         GPIO_PIN_DATA(1, 2)
#define PB3                         GPIO_PIN_DATA(1, 3)
#define PB4                         GPIO_PIN_DATA(1, 4)
#define PB5                         GPIO_PIN_DATA(1, 5)
#define PB6                         GPIO_PIN_DATA(1, 6)
#define PB7                         GPIO_PIN_DATA(1, 7)
#define PB8                         GPIO_PIN_DATA(1, 8)
#define PB9                         GPIO_PIN_DATA(1, 9)
#define PB10                        GPIO_PIN_DATA(1, 10)
#define PB12                        GPIO_PIN_DATA(1, 12)
#define PB13                        GPIO_PIN_DATA(1, 13)
#define PB14                        GPIO_PIN_DATA(1, 14)
#define PB15                        GPIO_PIN_DATA(1, 15)

/* GPIO Port C (0~5 & 8~13) */
#define PC0                         GPIO_PIN_DATA(2, 0)
#define PC1                         GPIO_PIN_DATA(2, 1)
#define PC2                         GPIO_PIN_DATA(2, 2)
#define PC3                         GPIO_PIN_DATA(2, 3)
#define PC4                         GPIO_PIN_DATA(2, 4)
#define PC5                         GPIO_PIN_DATA(2, 5)
#define PC8                         GPIO_PIN_DATA(2, 8)
#define PC9                         GPIO_PIN_DATA(2, 9)
#define PC10                        GPIO_PIN_DATA(2, 10)
#define PC11                        GPIO_PIN_DATA(2, 11)
#define PC12                        GPIO_PIN_DATA(2, 12)
#define PC13                        GPIO_PIN_DATA(2, 13)

/* GPIO Port D (0~5 & 8~11) */
#define PD0                         GPIO_PIN_DATA(3, 0)
#define PD1                         GPIO_PIN_DATA(3, 1)
#define PD2                         GPIO_PIN_DATA(3, 2)
#define PD3                         GPIO_PIN_DATA(3, 3)
#define PD4                         GPIO_PIN_DATA(3, 4)
#define PD5                         GPIO_PIN_DATA(3, 5)
#define PD8                         GPIO_PIN_DATA(3, 8)
#define PD9                         GPIO_PIN_DATA(3, 9)
#define PD10                        GPIO_PIN_DATA(3, 10)
#define PD11                        GPIO_PIN_DATA(3, 11)

/* GPIO Port D (0~3) */
#define PF0                         GPIO_PIN_DATA(5, 0)
#define PF1                         GPIO_PIN_DATA(5, 1)
#define PF2                         GPIO_PIN_DATA(5, 2)
#define PF3                         GPIO_PIN_DATA(5, 3)

/**
 * @name    Port Abstraction Layer related definitions
 * @{
 */
/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH           GPIO_PINSPERPORT_MAX

/**
 * @brief   Whole port mask.
 * @details This macro specifies all the valid bits into a port.
 */
/* #define PAL_WHOLE_PORT              ((ioportmask_t)0xFFFFU) */
#define PAL_WHOLE_PORT ((ioportmask_t)(2^PAL_IOPORTS_WIDTH) - 1)
/** @} */

/**
 * @name    Line handling macros
 * @{
 */
/**
 * @brief   Forms a line identifier.
 * @details A port/pad pair are encoded into an @p ioline_t type. The encoding
 *          of this type is platform-dependent.
 */
#define PAL_LINE(port, pad)                                                 \
  ((ioline_t)((uint32_t)(port)) | ((uint32_t)(pad)))

/**
 * @brief   Decodes a port identifier from a line identifier.
 */
#define PAL_PORT(line)                                                      \
  ((GPIO_T *)(((uint32_t)(line)) & 0xFFFFFFF0U))

/**
 * @brief   Decodes a pad identifier from a line identifier.
 */
#define PAL_PAD(line)                                                       \
  ((uint32_t)((uint32_t)(line) & 0x0000000FU))

/**
 * @brief   Value identifying an invalid line.
 */
#define PAL_NOLINE                      0U
/** @} */

/**
 * @brief   Generic I/O ports static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialized the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {
  /** @brief Port A setup data.*/
  GPIO_T    PAData;
  /** @brief Port B setup data.*/
  GPIO_T    PBData;
  /** @brief Port C setup data.*/
  GPIO_T    PCData;
  /** @brief Port D setup data.*/
  GPIO_T    PDData;
  /** @brief Port F setup data.*/
  GPIO_T    PFData;
} PALConfig;

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint32_t iomode_t;

/**
 * @brief   Type of an I/O line.
 */
typedef uint32_t ioline_t;

/**
 * @brief   Port Identifier.
 * @details This type can be a scalar or some kind of pointer, do not make
 *          any assumption about it, use the provided macros when populating
 *          variables of this type.
 */
typedef GPIO_T * ioportid_t;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/*===========================================================================*/

/**
 * @brief   First I/O port identifier.
 * @details Low level drivers can define multiple ports, it is suggested to
 *          use this naming convention.
 */
#define IOPORT1         0
#define GPIOA           PA
#define GPIOB           PB
#define GPIOC           PC
#define GPIOD           PD
#define GPIOF           PF

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @param[in] config    architecture-dependent ports configuration
 *
 * @notapi
 */
#if defined(PAL_NEW_INIT)
#define pal_lld_init() _pal_lld_init()
#else
#define pal_lld_init(config) _pal_lld_init(config)
#endif

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port) ((port)->PIN)

/**
 * @brief   Reads the output latch.
 * @details The purpose of this function is to read back the latched output
 *          value.
 *
 * @param[in] port      port identifier
 * @return              The latched logical states.
 *
 * @notapi
 */
#define pal_lld_readlatch(port) ((port)->DOUT)

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits) ((port)->DOUT = (bits))

/**
 * @brief   Sets a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 *
 * @notapi
 */
#define pal_lld_setport(port, bits) ((port)->DOUT |= (bits))

/**
 * @brief   Clears a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 *
 * @notapi
 */
#define pal_lld_clearport(port, bits) ((port)->DOUT &= ~(bits))

/**
 * @brief   Toggles a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be XORed on the specified port
 *
 * @notapi
 */
#define pal_lld_toggleport(port, bits) ((port)->DOUT ^= (bits))

/**
 * @brief   Reads a group of bits.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @return              The group logical states.
 *
 * @notapi
 */
/* #define pal_lld_readgroup(port, mask, offset) 0U */

/**
 * @brief   Writes a group of bits.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] bits      bits to be written. Values exceeding the group width
 *                      are masked.
 *
 * @notapi
 */
#define pal_lld_writegroup(port, mask, offset, bits)                        \
  do {                                                                      \
    uint32_t oldmask = (port)->DMASK;                                       \
    (port)->DMASK = ~((mask) << (offset));                                  \
    (port)->DOUT = ((bits) & (mask)) << (offset);                           \
    (port)->DMASK = oldmask;                                                \
  } while (false)

/**
 * @brief   Pads group mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] mode      group mode
 *
 * @notapi
 */
#define pal_lld_setgroupmode(port, mask, offset, mode)                      \
  _pal_lld_setgroupmode(port, (mask) << (offset), mode)

/**
 * @brief   Reads a logical state from an I/O pad.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @return              The logical state.
 * @retval PAL_LOW      low logical state.
 * @retval PAL_HIGH     high logical state.
 *
 * @notapi
 */
/**
 * @brief   Reads a logical state from an I/O pad.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @return              The logical state.
 * @retval PAL_LOW      low logical state.
 * @retval PAL_HIGH     high logical state.
 *
 * @notapi
 */
/* #define pal_lld_readpad(port, pad) PAL_LOW */
#define pal_lld_readpad(port, pad) (((port)->PIN & PAL_PORT_BIT(pad)) >> (pad))

/**
 * @brief   Writes a logical state on an output pad.
 * @note    This function is not meant to be invoked directly by the
 *          application  code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] bit       logical value, the value must be @p PAL_LOW or
 *                      @p PAL_HIGH
 *
 * @notapi
 */
/*
#define pal_lld_writepad(port, pad, bit)                                    \
  do {                                                                      \
    (void)port;                                                             \
    (void)pad;                                                              \
    (void)bit;                                                              \
  } while (false)
*/
/**
 * @brief   Sets a pad logical state to @p PAL_HIGH.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_setpad(port, pad)                                           \
  ((port)->DOUT |= PAL_PORT_BIT(pad))

/**
 * @brief   Clears a pad logical state to @p PAL_LOW.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_clearpad(port, pad)                                         \
  ((port)->DOUT &= ~(0xFFFF0000U | PAL_PORT_BIT(pad)))

/**
 * @brief   Toggles a pad logical state.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_togglepad(port, pad)                                        \
  ((port)->DOUT ^= PAL_PORT_BIT(pad))

/**
 * @brief   Pad mode setup.
 * @details This function programs a pad with the specified mode.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad mode
 *
 * @notapi
 */
#define pal_lld_setpadmode(port, pad, mode)                                 \
  _pal_lld_setgroupmode(port, PAL_PORT_BIT(pad), mode)
  /* GPIO_SetMode(port, PAL_PORT_BIT(pad), mode) */

#if !defined(PAL_NEW_INIT) && !defined(__DOXYGEN__)
extern const PALConfig pal_default_config;
#endif

#ifdef __cplusplus
extern "C" {
#endif
#if defined(PAL_NEW_INIT)
  void _pal_lld_init(void);
#else
  void _pal_lld_init(const PALConfig *config);
#endif
  void _pal_lld_setgroupmode(ioportid_t port,
                             ioportmask_t mask,
                             iomode_t mode);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL == TRUE */

#endif /* HAL_PAL_LLD_H */

/** @} */
