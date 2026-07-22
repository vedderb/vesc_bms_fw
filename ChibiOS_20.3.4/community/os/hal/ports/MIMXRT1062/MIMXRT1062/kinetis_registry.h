/*
    ChibiOS - Copyright (C) 2014 Derek Mulcahy
                        (C) 2016 flabbergast <s3+flabbergast@sdfeu.org>

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
 * @file    MK66F18/kinetis_registry.h
 * @brief   MK66F18 capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef MIMXRT1062_REGISTRY_H_
#define MIMXRT1062_REGISTRY_H_

#if !defined(MIMXRT1062) || defined(__DOXYGEN__)
#define MIMXRT1062 
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/* See IMXRT1060RM, page 44, table 4-2: CM7 domain interrupt summary */

/* ADC attributes.*/
#define MIMXRT1062_HAS_ADC0            TRUE
#define MIMXRT1062_ADC0_IRQ_VECTOR     Vector14C
#define MIMXRT1062_HAS_ADC1            TRUE
#define MIMXRT1062_ADC1_IRQ_VECTOR     Vector150

/* DAC attributes.*/
/* IMXRT1060RM, page 3324, 65.11 DAC interrupts: This module has no interrupts. */
#define MIMXRT1062_HAS_DAC0            TRUE
#define MIMXRT1062_HAS_DAC1            TRUE
#define MIMXRT1062_HAS_DAC2            TRUE
#define MIMXRT1062_HAS_DAC3            TRUE

/* DMA attributes.*/
#define MIMXRT1062_DMA0_IRQ_VECTOR     Vector40
#define MIMXRT1062_DMA1_IRQ_VECTOR     Vector44 
#define MIMXRT1062_DMA2_IRQ_VECTOR     Vector48
#define MIMXRT1062_DMA3_IRQ_VECTOR     Vector4C
#define MIMXRT1062_DMA4_IRQ_VECTOR     Vector50
#define MIMXRT1062_DMA5_IRQ_VECTOR     Vector54
#define MIMXRT1062_DMA6_IRQ_VECTOR     Vector58
#define MIMXRT1062_DMA7_IRQ_VECTOR     Vector5C
#define MIMXRT1062_DMA8_IRQ_VECTOR     Vector60
#define MIMXRT1062_DMA9_IRQ_VECTOR     Vector64
#define MIMXRT1062_DMA10_IRQ_VECTOR     Vector68
#define MIMXRT1062_DMA11_IRQ_VECTOR     Vector6C
#define MIMXRT1062_DMA12_IRQ_VECTOR     Vector70
#define MIMXRT1062_DMA13_IRQ_VECTOR     Vector74
#define MIMXRT1062_DMA14_IRQ_VECTOR     Vector78
#define MIMXRT1062_DMA15_IRQ_VECTOR     Vector7C
#define MIMXRT1062_HAS_DMA_ERROR_IRQ   TRUE
#define MIMXRT1062_DMA_ERROR_IRQ_VECTOR Vector80

/* I2C attributes.*/
#define MIMXRT1062_HAS_I2C0            TRUE
#define MIMXRT1062_I2C0_IRQ_VECTOR     VectorB0
#define MIMXRT1062_I2C0_IS_LPI2C       TRUE
#define MIMXRT1062_HAS_I2C1            TRUE
#define MIMXRT1062_I2C1_IRQ_VECTOR     VectorB4
#define MIMXRT1062_I2C1_IS_LPI2C       TRUE
#define MIMXRT1062_HAS_I2C2            TRUE
#define MIMXRT1062_I2C2_IRQ_VECTOR     VectorB8
#define MIMXRT1062_I2C2_IS_LPI2C       TRUE
#define MIMXRT1062_HAS_I2C3            TRUE
#define MIMXRT1062_I2C3_IRQ_VECTOR     VectorBC
#define MIMXRT1062_I2C3_IS_LPI2C       TRUE

/* Serial attributes.*/
#define MIMXRT1062_HAS_SERIAL0         TRUE
#define MIMXRT1062_SERIAL0_IRQ_VECTOR  Vector90
#define MIMXRT1062_HAS_SERIAL1         TRUE
#define MIMXRT1062_SERIAL1_IRQ_VECTOR  Vector94
#define MIMXRT1062_HAS_SERIAL2         TRUE
#define MIMXRT1062_SERIAL2_IRQ_VECTOR  Vector98
#define MIMXRT1062_HAS_SERIAL3         TRUE
#define MIMXRT1062_SERIAL3_IRQ_VECTOR  Vector9C
#define MIMXRT1062_HAS_SERIAL4         TRUE
#define MIMXRT1062_SERIAL4_IRQ_VECTOR  VectorA0
#define MIMXRT1062_HAS_SERIAL5         TRUE
#define MIMXRT1062_SERIAL5_IRQ_VECTOR  VectorA4
#define MIMXRT1062_HAS_SERIAL6         TRUE
#define MIMXRT1062_SERIAL6_IRQ_VECTOR  VectorA8
#define MIMXRT1062_HAS_SERIAL7         TRUE
#define MIMXRT1062_SERIAL7_IRQ_VECTOR  VectorAC
#define MIMXRT1062_HAS_SERIAL_ERROR_IRQ FALSE
#define MIMXRT1062_SERIAL0_IS_LPUART   TRUE
#define MIMXRT1062_SERIAL0_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL1_IS_LPUART   TRUE
#define MIMXRT1062_SERIAL1_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL2_IS_LPUART   TRUE
#define MIMXRT1062_SERIAL2_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL3_IS_LPUART   TRUE
#define MIMXRT1062_SERIAL3_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL4_IS_LPUART   TRUE
#define MIMXRT1062_SERIAL4_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL5_IS_LPUART   TRUE
#define MIMXRT1062_SERIAL5_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL6_IS_LPUART   TRUE
#define MIMXRT1062_SERIAL6_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL7_IS_LPUART   TRUE
#define MIMXRT1062_SERIAL7_IS_UARTLP   FALSE

/* SPI attributes.*/
#define MIMXRT1062_HAS_SPI0            TRUE
#define MIMXRT1062_SPI0_IRQ_VECTOR     VectorC0
#define MIMXRT1062_SPI0_IS_LPSPI       TRUE
#define MIMXRT1062_HAS_SPI1            TRUE
#define MIMXRT1062_SPI1_IRQ_VECTOR     VectorC4
#define MIMXRT1062_SPI1_IS_LPSPI       TRUE
#define MIMXRT1062_HAS_SPI2            TRUE
#define MIMXRT1062_SPI2_IRQ_VECTOR     VectorC8
#define MIMXRT1062_SPI2_IS_LPSPI       TRUE
#define MIMXRT1062_HAS_SPI3            TRUE
#define MIMXRT1062_SPI3_IRQ_VECTOR     VectorCC
#define MIMXRT1062_SPI3_IS_LPSPI       TRUE

/* GPT attributes.*/
#define MIMXRT1062_HAS_PIT0            TRUE
#define MIMXRT1062_HAS_PIT1            TRUE
#define MIMXRT1062_HAS_PIT2            TRUE
#define MIMXRT1062_HAS_PIT3            TRUE
#define MIMXRT1062_HAS_PIT_COMMON_IRQ  TRUE
#define MIMXRT1062_PIT_IRQ_VECTOR      Vector228

/* USB attributes.*/
#define MIMXRT1062_HAS_USB                TRUE
#define MIMXRT1062_USB_OTG1_IRQ_VECTOR    Vector204
#define MIMXRT1062_USB_OTG2_IRQ_VECTOR    Vector200
#define MIMXRT1062_USB0_IS_USBOTG         TRUE
#define MIMXRT1062_HAS_USB_CLOCK_RECOVERY TRUE

/* SDHC (SDC, MMC, SDIO) attributes */
#define MIMXRT1062_HAS_SDHC0            TRUE
#define MIMXRT1062_SDHC0_IRQ_VECTOR     Vector1F8
#define MIMXRT1062_HAS_SDHC1            TRUE
#define MIMXRT1062_SDHC1_IRQ_VECTOR     Vector1FC

/** @} */

#endif /* MIMXRT1062_REGISTRY_H_ */

/** @} */
