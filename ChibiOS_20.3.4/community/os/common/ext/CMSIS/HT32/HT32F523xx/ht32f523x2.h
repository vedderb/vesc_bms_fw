/*
 * Copyright (C) 2020 Codetector <codetector@codetector.cn>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#if defined(HT32F52342) || defined(HT32F52352)
    #define HT32F523x2
#else
    #error "Unknown HT32 device"
#endif

#if defined(HT32F523x2)
    #define HT32
#endif

/*
 * ==============================================================
 * ---------- Interrupt Number Definition -----------------------
 * ==============================================================
 */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ****************/
    InitialSP_IRQn                = -16,
    InitialPC_IRQn                = -15,
    NonMaskableInt_IRQn           = -14,
    HardFault_IRQn                = -13,
    SVCall_IRQn                   = -5,
    PendSV_IRQn                   = -2,
    SysTick_IRQn                  = -1,

/******  HT32F165x Specific Interrupt Numbers ***********************/
    LVD_IRQn                      = 0,
    RTC_IRQn                      = 1,
    FMC_IRQn                      = 2,
    WKUP_IRQn                     = 3,
    EXTI0_1_IRQn                  = 4,
    EXTI2_3_IRQn                  = 5,
    EXTI4_15_IRQn                 = 6,
    CMP_IRQn                      = 7,
    ADC_IRQn                      = 8,

    MCTM_IRQn                     = 10,
    GPTM1_IRQn                    = 11,
    GPTM0_IRQn                    = 12,
    SCTM0_IRQn                    = 13,
    SCTM1_IRQn                    = 14,

    BFTM0_IRQn                    = 17,
    BFTM1_IRQn                    = 18,
    I2C0_IRQn                     = 19,
    I2C1_IRQn                     = 20,
    SPI0_IRQn                     = 21,
    SPI1_IRQn                     = 22,
    USART0_IRQn                   = 23,
    USART1_IRQn                   = 24,
    UART0_IRQn                    = 25,
    UART1_IRQn                    = 26,
    SCI_IRQn                      = 27,
    I2S_IRQn                      = 28,
    USB_IRQn                      = 29,
    PDMA_CH0_1_IRQn               = 30,
    PDMA_CH2_5_IRQn               = 31
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/**
 * @brief HT32F165x Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
#define __FPU_PRESENT           0
#define __MPU_PRESENT           0
#define __NVIC_PRIO_BITS        8
#define __Vendor_SysTickConfig  0

#include "core_cm0plus.h"           /* Cortex-M0+ processor and core peripherals */

/****************************************************************/
/*                  Peripheral memory map                       */
/****************************************************************/
#define USART0_BASE             ((uint32_t)0x40000000)
#define UART0_BASE              ((uint32_t)0x40001000)
#define SPI0_BASE               ((uint32_t)0x40004000)
#define ADC_BASE                ((uint32_t)0x40010000)

#define AFIO_BASE               ((uint32_t)0x40022000)
#define EXTI_BASE               ((uint32_t)0x40024000)
#define I2S_BASE                ((uint32_t)0x40026000)
#define MCTM0_BASE              ((uint32_t)0x4002C000)
#define MCTM1_BASE              ((uint32_t)0x4002D000)

#define USART1_BASE             ((uint32_t)0x40040000)
#define UART1_BASE              ((uint32_t)0x40041000)
#define SCI_BASE                ((uint32_t)0x40043000)
#define SPI1_BASE               ((uint32_t)0x40044000)
#define I2C0_BASE               ((uint32_t)0x40048000)
#define I2C1_BASE               ((uint32_t)0x40049000)

#define CMP0_BASE               ((uint32_t)0x40058000)
#define CMP1_BASE               ((uint32_t)0x40058100)

#define WDT_BASE                ((uint32_t)0x40068000)
#define RTC_BASE                ((uint32_t)0x4006A000)
#define PWRCU_BASE              ((uint32_t)0x4006A000)
#define GPTM0_BASE              ((uint32_t)0x4006E000)
#define GPTM1_BASE              ((uint32_t)0x4006F000)
#define BFTM0_BASE              ((uint32_t)0x40076000)
#define BFTM1_BASE              ((uint32_t)0x40077000)

#define FMC_BASE                ((uint32_t)0x40080000)
#define CKCU_BASE               ((uint32_t)0x40088000)
#define RSTCU_BASE              ((uint32_t)0x40088000)
#define CRC_BASE                ((uint32_t)0x4008A000)
#define PDMA_BASE               ((uint32_t)0x40090000)
#define EBI_BASE                ((uint32_t)0x40098000)
#define USB_BASE                ((uint32_t)0x400A8000)
#define USB_SRAM_BASE           ((uint32_t)0x400AA000)
#define GPIO_A_BASE             ((uint32_t)0x400B0000)
#define GPIO_B_BASE             ((uint32_t)0x400B2000)
#define GPIO_C_BASE             ((uint32_t)0x400B4000)
#define GPIO_D_BASE             ((uint32_t)0x400B6000)


// Registers Headers
#include "ht32f523x2_reg.h"

/****************************************************************/
/*                 Peripheral declaration                       */
/****************************************************************/
#define USART0                  ((USART_TypeDef *)  USART0_BASE)
#define UART0                   ((USART_TypeDef *)  UART0_BASE)
#define SPI0                    ((SPI_TypeDef *)    SPI0_BASE)
#define ADC                     ((ADC_TypeDef *)    ADC_BASE)

#define AFIO                    ((AFIO_TypeDef *)   AFIO_BASE)
#define EXTI                    ((EXTI_TypeDef *)   EXTI_BASE)
#define I2S                     ((I2S_TypeDef *)    I2S_BASE)
#define MCTM0                   ((TM_TypeDef *)     MCTM0_BASE)
#define MCTM1                   ((TM_TypeDef *)     MCTM1_BASE)

#define USART1                  ((USART_TypeDef *)  USART1_BASE)
#define UART1                   ((USART_TypeDef *)  UART1_BASE)
#define SCI                     ((SCI_TypeDef *)    SCI_BASE)
#define SPI1                    ((SPI_TypeDef *)    SPI1_BASE)
#define I2C0                    ((I2C_TypeDef *)    I2C0_BASE)
#define I2C1                    ((I2C_TypeDef *)    I2C1_BASE)

#define CMP0                    ((CMP_TypeDef *)    CMP0_BASE)
#define CMP1                    ((CMP_TypeDef *)    CMP1_BASE)

#define WDT                     ((WDT_TypeDef *)    WDT_BASE)
#define RTC                     ((RTC_TypeDef *)    RTC_BASE)
#define PWRCU                   ((PWRCU_TypeDef *)  PWRCU_BASE)
#define GPTM0                   ((TM_TypeDef *)     GPTM0_BASE)
#define GPTM1                   ((TM_TypeDef *)     GPTM1_BASE)
#define BFTM0                   ((BFTM_TypeDef *)   BFTM0_BASE)
#define BFTM1                   ((BFTM_TypeDef *)   BFTM1_BASE)

#define FMC                     ((FMC_TypeDef *)    FMC_BASE)
#define CKCU                    ((CKCU_TypeDef *)   CKCU_BASE)
#define RSTCU                   ((RSTCU_TypeDef *)  RSTCU_BASE)
#define CRC                     ((CRC_TypeDef *)    CRC_BASE)
#define PDMA                    ((PDMA_TypeDef *)   PDMA_BASE)
#define EBI                     ((EBI_TypeDef *)    EBI_BASE)
#define USB                     ((USB_TypeDef *)    USB_BASE)

#define GPIOA                   ((GPIO_TypeDef *)   GPIO_A_BASE)
#define GPIO_A GPIOA
#define GPIOB                   ((GPIO_TypeDef *)   GPIO_B_BASE)
#define GPIO_B GPIOB
#define GPIOC                   ((GPIO_TypeDef *)   GPIO_C_BASE)
#define GPIO_C GPIOC
#define GPIOD                   ((GPIO_TypeDef *)   GPIO_D_BASE)
#define GPIO_D GPIOD

