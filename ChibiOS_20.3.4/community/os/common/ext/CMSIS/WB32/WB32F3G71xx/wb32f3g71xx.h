/**************************************************************************//**
 * @file     wb32f3g71xx.h
 * @brief    CMSIS Core Peripheral Access Layer Header File for
 *           wb32f3g71xx Device Series
 * @version  V0.1.5
 * @date     18-February-2021
 ******************************************************************************/
/*
    Copyright (c) 2020 - 2021 Westberry Technology (ChangZhou) Corp., Ltd

    All rights reserved.
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

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup wb32f3g71xx
  * @{
  */

#ifndef __WB32F3G71xx_H__
#define __WB32F3G71xx_H__

#if !defined(WB32F3G71xx)
  #define WB32F3G71xx
#endif

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

/** @addtogroup Library_configuration_section
  * @{
  */

#if !defined  USE_STDPERIPH_DRIVER
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
 * In this case, these drivers will not be included and the application code will
 * be based on direct access to peripherals registers
 */
  /*#define USE_STDPERIPH_DRIVER*/
#endif

/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup
 *        Timeout value
 */
#define HSE_STARTUP_TIMEOUT (48000)   /*!< Time out for HSE start up */


/**
 * @brief WB32F3G71xx Standard Peripheral Library version number
 */
#define __WB32F3G71xx_STDPERIPH_VERSION_MAIN   (0x00) /*!< [31:24] main version */
#define __WB32F3G71xx_STDPERIPH_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __WB32F3G71xx_STDPERIPH_VERSION_SUB2   (0x05) /*!< [15:8]  sub2 version */
#define __WB32F3G71xx_STDPERIPH_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __WB32F3G71xx_STDPERIPH_VERSION        ( (__WB32F3G71xx_STDPERIPH_VERSION_MAIN << 24)\
                                             |(__WB32F3G71xx_STDPERIPH_VERSION_SUB1 << 16)\
                                             |(__WB32F3G71xx_STDPERIPH_VERSION_SUB2 << 8)\
                                             |(__WB32F3G71xx_STDPERIPH_VERSION_RC))

/**
  * @}
  */

/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum IRQn
{
/* -------------------  Cortex-M3 Processor Exceptions Numbers  ------------------- */
  NonMaskableInt_IRQn           = -14,      /*  2 Non Maskable Interrupt */
  HardFault_IRQn                = -13,      /*  3 HardFault Interrupt */
  MemoryManagement_IRQn         = -12,      /*  4 Memory Management Interrupt */
  BusFault_IRQn                 = -11,      /*  5 Bus Fault Interrupt */
  UsageFault_IRQn               = -10,      /*  6 Usage Fault Interrupt */
  SVCall_IRQn                   =  -5,      /* 11 SV Call Interrupt */
  DebugMonitor_IRQn             =  -4,      /* 12 Debug Monitor Interrupt */
  PendSV_IRQn                   =  -2,      /* 14 Pend SV Interrupt */
  SysTick_IRQn                  =  -1,      /* 15 System Tick Interrupt */

/* ----------------------  WB32F3G71xx Specific Interrupt Numbers  --------------------- */
  WWDG_IRQn                     =   0,      /* Window WatchDog Interrupt */
  PVD_IRQn                      =   1,      /* PVD through EXTI Line detection Interrupt */
  TAMPER_IRQn                   =   2,      /* Tamper Interrupt */
  RTC_IRQn                      =   3,      /* RTC global Interrupt */
  FMC_IRQn                      =   4,      /* FMC global Interrupt */
  RCC_IRQn                      =   5,      /* RCC global Interrupt */
  EXTI0_IRQn                    =   6,      /* EXTI Line0 Interrupt */
  EXTI1_IRQn                    =   7,      /* EXTI Line1 Interrupt */
  EXTI2_IRQn                    =   8,      /* EXTI Line2 Interrupt */
  EXTI3_IRQn                    =   9,      /* EXTI Line3 Interrupt */
  EXTI4_IRQn                    =  10,      /* EXTI Line4 Interrupt */
  DMAC1_IRQn                    =  11,      /* DMAC1 Interrupt */
  DMAC2_IRQn                    =  12,      /* DMAC2 Interrupt */
  ADC_IRQn                      =  13,      /* ADC global Interrupt */
  USB_IRQn                      =  14,      /* USB Interrupt */
  USB_DMA_IRQn                  =  15,      /* USB DMA Interrupt */
  EXTI9_5_IRQn                  =  16,      /* External Line[9:5] Interrupts */
  TIM1_BRK_IRQn                 =  17,      /* TIM1 Break Interrupt */
  TIM1_UP_IRQn                  =  18,      /* TIM1 Update Interrupt */
  TIM1_TRG_COM_IRQn             =  19,      /* TIM1 Trigger and Commutation Interrupt */
  TIM1_CC_IRQn                  =  20,      /* TIM1 Capture Compare Interrupt */
  TIM2_IRQn                     =  21,      /* TIM2 global Interrupt */
  TIM3_IRQn                     =  22,      /* TIM3 global Interrupt */
  TIM4_IRQn                     =  23,      /* TIM4 global Interrupt */
  I2C1_IRQn                     =  24,      /* I2C1 global Interrupt */
  I2C2_IRQn                     =  25,      /* I2C2 global Interrupt */
  QSPI_IRQn                     =  26,      /* QSPI global Interrupt */
  SPIM2_IRQn                    =  27,      /* SPIM2 global Interrupt */
  SPIS1_IRQn                    =  28,      /* SPIS1 global Interrupt */
  SPIS2_IRQn                    =  29,      /* SPIS2 global Interrupt */
  UART1_IRQn                    =  30,      /* UART1 global Interrupt */
  UART2_IRQn                    =  31,      /* UART2 global Interrupt */
  UART3_IRQn                    =  32,      /* UART3 global Interrupt */
  EXTI15_10_IRQn                =  33,      /* External Line[15:10] Interrupts */
  RTCAlarm_IRQn                 =  34,      /* RTC Alarm through EXTI Line Interrupt */
  USBP_WKUP_IRQn                =  35,      /* USB PIN global interrupt */
  I2S_IRQn                      =  36,      /* I2S global Interrupt */
  ISO_IRQn                      =  37,      /* ISO7816 global Interrupt */
} IRQn_Type;


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* -------  Start of section using anonymous unions and disabling warnings  ------- */
#if   defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif


/* --------  Configuration of the Cortex-M3 Processor and Core Peripherals  ------- */
#define __CM3_REV                 0x0200U   /* Core revision r2p0 */
#define __MPU_PRESENT             1         /* WB32F3G71xx devices provide an MPU */
#define __VTOR_PRESENT            1         /* VTOR present or not */
#define __NVIC_PRIO_BITS          4         /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0         /* Set to 1 if different SysTick Config is used */

#include "core_cm3.h"                       /* Processor and core peripherals */

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {INACTIVE = 0, ACTIVE = !INACTIVE} SignalState;

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/* ================================================================================ */
/* ================           General-purpose I/Os (GPIO)          ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t MODER;           /*!< GPIO port mode register,                             Address offset: 0x000 */
  __IOM uint32_t OTYPER;          /*!< GPIO port output type register,                      Address offset: 0x004 */
  __IOM uint32_t OSPEEDR;         /*!< GPIO port output speed register,                     Address offset: 0x008 */
  __IOM uint32_t PUPDR;           /*!< GPIO port pull-up/pull-down register,                Address offset: 0x00C */
  __IM  uint32_t IDR;             /*!< GPIO port input data register,                       Address offset: 0x010 */
  __IOM uint32_t ODR;             /*!< GPIO port output data register,                      Address offset: 0x014 */
  __OM  uint32_t BSRR;            /*!< GPIO port bit set/reset register,                    Address offset: 0x018 */
  __IOM uint32_t LCKR;            /*!< GPIO port configuration lock register,               Address offset: 0x01C */
  __IOM uint32_t AFRL;            /*!< GPIO alternate function low register,                Address offset: 0x020 */
  __IOM uint32_t AFRH;            /*!< GPIO alternate function high register,               Address offset: 0x024 */
  __IOM uint32_t SMIT;            /*!< No Description,                                      Address offset: 0x028 */
  __IOM uint32_t CURRENT;         /*!< No Description,                                      Address offset: 0x02C */
  __IOM uint32_t CFGMSK;          /*!< No Description,                                      Address offset: 0x030 */
} GPIO_TypeDef;


/* ================================================================================ */
/* ================                   Timer (TIM)                  ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t CR1;             /*!< Control register 1,                                  Address offset: 0x000 */
  __IOM uint32_t CR2;             /*!< Control register 2,                                  Address offset: 0x004 */
  __IOM uint32_t SMCR;            /*!< Slave mode control register,                         Address offset: 0x008 */
  __IOM uint32_t DIER;            /*!< DMA/interrupt enable register,                       Address offset: 0x00C */
  __IOM uint32_t SR;              /*!< Status register,                                     Address offset: 0x010 */
  __OM  uint32_t EGR;             /*!< Event generation register,                           Address offset: 0x014 */
  __IOM uint32_t CCMR1;           /*!< Capture/compare mode register 1,                     Address offset: 0x018 */
  __IOM uint32_t CCMR2;           /*!< Capture/compare mode register 2,                     Address offset: 0x01C */
  __IOM uint32_t CCER;            /*!< Capture/compare enable register,                     Address offset: 0x020 */
  __IOM uint32_t CNT;             /*!< Counter,                                             Address offset: 0x024 */
  __IOM uint32_t PSC;             /*!< Prescaler,                                           Address offset: 0x028 */
  __IOM uint32_t ARR;             /*!< Auto-reload register,                                Address offset: 0x02C */
  __IOM uint32_t RCR;             /*!< Repetition counter register,                         Address offset: 0x030 */
  __IOM uint32_t CCR1;            /*!< Capture/compare register 1,                          Address offset: 0x034 */
  __IOM uint32_t CCR2;            /*!< Capture/compare register 2,                          Address offset: 0x038 */
  __IOM uint32_t CCR3;            /*!< Capture/compare register 3,                          Address offset: 0x03C */
  __IOM uint32_t CCR4;            /*!< Capture/compare register 4,                          Address offset: 0x040 */
  __IOM uint32_t BDTR;            /*!< Break and dead-time register,                        Address offset: 0x044 */
} TIM_TypeDef;


/* ================================================================================ */
/* ================   External Interrupt/Event Controller (EXTI)   ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t IMR;             /*!< Interrupt mask register,                             Address offset: 0x000 */
  __IOM uint32_t EMR;             /*!< Event mask register,                                 Address offset: 0x004 */
  __IOM uint32_t RTSR;            /*!< Rising trigger selection register,                   Address offset: 0x008 */
  __IOM uint32_t FTSR;            /*!< Falling trigger selection register,                  Address offset: 0x00C */
  __IOM uint32_t SWIER;           /*!< Software interrupt event register,                   Address offset: 0x010 */
  __IOM uint32_t PR;              /*!< Pending register,                                    Address offset: 0x014 */
} EXTI_TypeDef;


/* ================================================================================ */
/* ================          Alternate Function I/O (AFIO)         ================ */
/* ================================================================================ */
typedef struct
{
        uint32_t RESERVED0[2];    /*!< Reserved,                                                            0x000 - 0x004 */
  __IOM uint32_t EXTICR[4];       /*!< External interrupt configuration register 1 to 4,    Address offset: 0x008 - 0x014 */
} AFIO_TypeDef;


/* ================================================================================ */
/* ================             Window watchdog (WWDG)             ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t CR;              /*!< Control register,                                    Address offset: 0x000 */
  __IOM uint32_t CFR;             /*!< Configuration register,                              Address offset: 0x004 */
  __IOM uint32_t SR;              /*!< Status register,                                     Address offset: 0x008 */
} WWDG_TypeDef;


/* ================================================================================ */
/* ================           Independent watchdog (IWDG)          ================ */
/* ================================================================================ */
typedef struct
{
  __OM  uint32_t KR;              /*!< Key register,                                        Address offset: 0x000 */
  __IOM uint32_t PR;              /*!< Prescaler register,                                  Address offset: 0x004 */
  __IOM uint32_t RLR;             /*!< Reload register,                                     Address offset: 0x008 */
  __IM  uint32_t SR;              /*!< Status register,                                     Address offset: 0x00C */
} IWDG_TypeDef;


/* ================================================================================ */
/* ================             Backup Registers (BKP)             ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t RTCCR;           /*!< RTC clock calibration register,                      Address offset: 0x000 */
  __IOM uint32_t CR;              /*!< Backup control register,                             Address offset: 0x004 */
  __IOM uint32_t CSR;             /*!< Backup control/status register,                      Address offset: 0x008 */
        uint32_t RESERVED0;       /*!< Reserved,                                                            0x00C */
  __IOM uint32_t DR1;             /*!< Backup data register 1,                              Address offset: 0x010 */
  __IOM uint32_t DR2;             /*!< Backup data register 2,                              Address offset: 0x014 */
  __IOM uint32_t DR3;             /*!< Backup data register 3,                              Address offset: 0x018 */
  __IOM uint32_t DR4;             /*!< Backup data register 4,                              Address offset: 0x01C */
  __IOM uint32_t DR5;             /*!< Backup data register 5,                              Address offset: 0x020 */
  __IOM uint32_t DR6;             /*!< Backup data register 6,                              Address offset: 0x024 */
  __IOM uint32_t DR7;             /*!< Backup data register 7,                              Address offset: 0x028 */
  __IOM uint32_t DR8;             /*!< Backup data register 8,                              Address offset: 0x02C */
  __IOM uint32_t DR9;             /*!< Backup data register 9,                              Address offset: 0x030 */
  __IOM uint32_t DR10;            /*!< Backup data register 10,                             Address offset: 0x034 */
  __IOM uint32_t DR11;            /*!< Backup data register 11,                             Address offset: 0x038 */
  __IOM uint32_t DR12;            /*!< Backup data register 12,                             Address offset: 0x03C */
  __IOM uint32_t DR13;            /*!< Backup data register 13,                             Address offset: 0x040 */
  __IOM uint32_t DR14;            /*!< Backup data register 14,                             Address offset: 0x044 */
  __IOM uint32_t DR15;            /*!< Backup data register 15,                             Address offset: 0x048 */
  __IOM uint32_t DR16;            /*!< Backup data register 16,                             Address offset: 0x04C */
  __IOM uint32_t DR17;            /*!< Backup data register 17,                             Address offset: 0x050 */
  __IOM uint32_t DR18;            /*!< Backup data register 18,                             Address offset: 0x054 */
  __IOM uint32_t DR19;            /*!< Backup data register 19,                             Address offset: 0x058 */
  __IOM uint32_t DR20;            /*!< Backup data register 20,                             Address offset: 0x05C */
  __IOM uint32_t DR21;            /*!< Backup data register 21,                             Address offset: 0x060 */
        uint32_t RESERVED1[39];   /*!< Reserved,                                                    0x064 - 0x0FC */
  __IOM uint32_t BDCR;            /*!< Backup domain control register,                      Address offset: 0x100 */
} BKP_TypeDef;


/* ================================================================================ */
/* ================              Real-Time Clock (RTC)             ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint16_t CRH;             /*!< RTC control register high,                           Address offset: 0x000 */    uint16_t RESERVED0;
  __IOM uint16_t CRL;             /*!< RTC control register low,                            Address offset: 0x004 */    uint16_t RESERVED1;
  __OM  uint16_t PRLH;            /*!< RTC prescaler load register high,                    Address offset: 0x008 */    uint16_t RESERVED2;
  union {
  __IM  uint32_t PRL;             /*!< RTC prescaler load register (read only),             Address offset: 0x00C */
  __OM  uint16_t PRLL;            /*!< RTC prescaler load register low,                     Address offset: 0x00C */
  };
  __IM  uint16_t DIVH;            /*!< RTC prescaler divider register high,                 Address offset: 0x010 */    uint16_t RESERVED4;
  union {
  __IM  uint32_t DIV;             /*!< RTC prescaler divider register,                      Address offset: 0x014 */
  __IM  uint16_t DIVL;            /*!< RTC prescaler divider register low,                  Address offset: 0x014 */
  };
  __IOM uint16_t CNTH;            /*!< RTC counter register high,                           Address offset: 0x018 */    uint16_t RESERVED5;
  union {
  __IM  uint32_t CNT;             /*!< RTC counter register (read only),                    Address offset: 0x01C */
  __IOM uint16_t CNTL;            /*!< RTC counter register low,                            Address offset: 0x01C */
  };
  __OM  uint16_t ALRH;            /*!< RTC alarm register high,                             Address offset: 0x020 */    uint16_t RESERVED6;
  union {
  __IM  uint32_t ALR;             /*!< RTC alarm register (read only),                      Address offset: 0x024 */
  __OM  uint16_t ALRL;            /*!< RTC alarm register low,                              Address offset: 0x024 */
  };
} RTC_TypeDef;


/* ================================================================================ */
/* ================                      LED                       ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t CON;             /*!< LED control register,                                Address offset: 0x000 */
  __IOM uint32_t CYC;             /*!< LED cycle register,                                  Address offset: 0x004 */
  __IOM uint32_t ECO;             /*!< LED register,                                        Address offset: 0x008 */
  __IOM uint32_t SEGH;            /*!< LED segment control high register,                   Address offset: 0x00C */
  __IOM uint32_t SEGL;            /*!< LED segment control low register,                    Address offset: 0x010 */
} LED_TypeDef;


/* ================================================================================ */
/* ================          Special Function Macro (SFM)          ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t CTRL;            /*!< SFM Control register,                                Address offset: 0x000 */
  __IOM uint32_t DATA;            /*!< SFM Input Data register,                             Address offset: 0x004 */
  __IM  uint32_t DOUT[8];         /*!< SFM Result register,                         Address offset: 0x008 - 0x024 */
        uint32_t RESERVED0[6];    /*!< Reserved,                                                    0x028 - 0x03C */
  __IOM uint32_t USBPCON;         /*!< USB Port Control register,                           Address offset: 0x040 */
  __IOM uint32_t USBPSDCSR;       /*!< USB Port State Detect Control/Status register,       Address offset: 0x044 */
  __IM  uint32_t USBPSTAT;        /*!< USB Port Status register,                            Address offset: 0x048 */
} SFM_TypeDef;


/* ================================================================================ */
/* ================                 ISO7816 (ISO)                  ================ */
/* ================================================================================ */
typedef struct
{
  __OM  uint32_t TXBUF;         /*!< ISO7816 register,                                      Address offset: 0x000 */
  __IM  uint32_t RXBUF;         /*!< ISO7816 register,                                      Address offset: 0x004 */
  __IOM uint32_t TXDONE;        /*!< ISO7816 register,                                      Address offset: 0x008 */
  __IOM uint32_t RXDONE;        /*!< ISO7816 register,                                      Address offset: 0x00C */
  __IOM uint32_t STARTDET;      /*!< ISO7816 register,                                      Address offset: 0x010 */
  __IM  uint32_t CLRTXDONE;     /*!< ISO7816 register,                                      Address offset: 0x014 */
  __IM  uint32_t CLRRXDONE;     /*!< ISO7816 register,                                      Address offset: 0x018 */
  __IM  uint32_t CLRSTART;      /*!< ISO7816 register,                                      Address offset: 0x01C */
  __IOM uint32_t TRANSR;        /*!< ISO7816 register,                                      Address offset: 0x020 */
  __IOM uint32_t FIFOSR;        /*!< ISO7816 register,                                      Address offset: 0x024 */
  __IOM uint32_t IER;           /*!< ISO7816 register,                                      Address offset: 0x028 */
  __IOM uint32_t MODE;          /*!< ISO7816 register,                                      Address offset: 0x02C */
  __IOM uint32_t ETU;           /*!< ISO7816 register,                                      Address offset: 0x030 */
  __IM  uint32_t RISR;          /*!< ISO7816 register,                                      Address offset: 0x034 */
  __IM  uint32_t ISR;           /*!< ISO7816 register,                                      Address offset: 0x038 */
} ISO_TypeDef;


/* ================================================================================ */
/* ================            Cache Registers (CACHE)             ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t CR;            /*!< CACHE Control register,                                Address offset: 0x000 */
} CACHE_TypeDef;


/* ================================================================================ */
/* ================                      FMC                       ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t CON;             /*!< FMC control register,                                Address offset: 0x000 */
  __IOM uint32_t CRCON;           /*!< FMC CRC control register,                            Address offset: 0x004 */
  __IOM uint32_t STAT;            /*!< FMC status register,                                 Address offset: 0x008 */
  __OM  uint32_t KEY;             /*!< FMC key register,                                    Address offset: 0x00C */
  __IOM uint32_t ADDR;            /*!< FMC address register,                                Address offset: 0x010 */
        uint32_t RESERVED2[2];    /*!< Reserved,                                                    0x014 - 0x018 */
  __IM  uint32_t DATA1;           /*!< FMC data register 1,                                 Address offset: 0x01C */
        uint32_t RESERVED3[56];   /*!< Reserved,                                                    0x020 - 0x0FC */
  __OM  uint32_t BUF[64];         /*!< FMC buffer,                                  Address offset: 0x100 - 0x1FC */
} FMC_TypeDef;


/* ================================================================================ */
/* ================                        SYS                     ================ */
/* ================================================================================ */
typedef struct
{
  __IM  uint32_t ID;              /*!< SYS ID register,                                     Address offset: 0x000 */
  __IM  uint32_t MEMSZ;           /*!< SYS Memory Size register,                            Address offset: 0x004 */
        uint32_t RESERVED0;       /*!< Reserved,                                                            0x008 */
  __IM  uint32_t BTCR;            /*!< SYS BOOT Control register,                           Address offset: 0x00C */
  __IM  uint32_t MEMWEN;          /*!< SYS Main Memory Write Enable register,               Address offset: 0x010 */
  __IM  uint32_t SENDEV;          /*!< SYS Second Development Control register,             Address offset: 0x014 */
  __IM  uint32_t RSTCR;           /*!< SYS Reset Control register,                          Address offset: 0x018 */
  __IM  uint32_t IF4LCK;          /*!< SYS Info4 Write Enable register,                     Address offset: 0x01C */
  __IM  uint32_t IF5LCK;          /*!< SYS Info5 Write Enable register,                     Address offset: 0x020 */
  __IM  uint32_t IF6LCK;          /*!< SYS Info6 Write Enable register,                     Address offset: 0x024 */
  __IM  uint32_t IF7LCK;          /*!< SYS Info7 Write Enable register,                     Address offset: 0x028 */
        uint32_t RESERVED1[2];    /*!< Reserved,                                                    0x02C - 0x030 */
  __IM  uint32_t BTSR;            /*!< SYS Boot Status register,                            Address offset: 0x034 */
} SYS_TypeDef;


/* ================================================================================ */
/* ================          Cyclic Redundancy Check (CRC)         ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t MODE;            /*!< CRC Mode register,                                   Address offset: 0x000 */
  __IOM uint32_t SEED;            /*!< CRC Seed register,                                   Address offset: 0x004 */
  union {
  __IM uint32_t SUM;              /*!< CRC Sum register,                                    Address offset: 0x008 */
  __OM uint32_t WR_DATA_WORD;     /*!< CRC Data register for word,                          Address offset: 0x008 */
  __OM uint16_t WR_DATA_HALF_WORD;/*!< CRC Data register for half word,                     Address offset: 0x008 */
  __OM uint8_t  WR_DATA_BYTE;     /*!< CRC Data register for byte,                          Address offset: 0x008 */
  };
} CRC_TypeDef;



/* ================================================================================ */
/* ============== Universal Asyncronous Receiver / Transmitter (UART) ============= */
/* ================================================================================ */
typedef struct
{
  union {
  __IM  uint32_t RBR;             /*!< Receive Buffer Register,                             Address offset: 0x000 */
  __OM  uint32_t THR;             /*!< Transmit Holding Register,                           Address offset: 0x000 */
  __IOM uint32_t DLL;             /*!< Divisor Latch(Low),                                  Address offset: 0x000 */
  };
  union {
  __IOM uint32_t DLH;             /*!< Divisor Latch(High),                                 Address offset: 0x004 */
  __IOM uint32_t IER;             /*!< Interrupt Enable Register,                           Address offset: 0x004 */
  };
  union {
  __IM  uint32_t IIR;             /*!< Interrupt Identification Register,                   Address offset: 0x008 */
  __OM  uint32_t FCR;             /*!< FIFO Control Register,                               Address offset: 0x008 */
  };
  __IOM uint32_t LCR;             /*!< Line Control Register,                               Address offset: 0x00C */
  __IOM uint32_t MCR;             /*!< Modem Control Register,                              Address offset: 0x010 */
  __IM  uint32_t LSR;             /*!< Line Status Register,                                Address offset: 0x014 */
  __IM  uint32_t MSR;             /*!< Modem Status Register,                               Address offset: 0x018 */
  __IOM uint32_t SCR;             /*!< Scratchpad Register,                                 Address offset: 0x01C */
        uint32_t RESERVED0[23];   /*!< Reserved,                                                    0x020 - 0x078 */
  __IM  uint32_t USR;             /*!< UART Status Register,                                Address offset: 0x07C */
  __IM  uint32_t TFL;             /*!< Transmit FIFO Level,                                 Address offset: 0x080 */
  __IM  uint32_t RFL;             /*!< Receive FIFO Level,                                  Address offset: 0x084 */
  __OM  uint32_t SRR;             /*!< Software Reset Register,                             Address offset: 0x088 */
  __IOM uint32_t SRTS;            /*!< Shadow Request to Send,                              Address offset: 0x08C */
  __IOM uint32_t SBCR;            /*!< Shadow Break Control Register,                       Address offset: 0x090 */
        uint32_t RESERVED1;       /*!< Reserved,                                                            0x094 */
  __IOM uint32_t SFE;             /*!< Shadow FIFO Enable,                                  Address offset: 0x098 */
  __IOM uint32_t SRT;             /*!< Shadow RCVR Trigger,                                 Address offset: 0x09C */
  __IOM uint32_t STET;            /*!< Shadow TX Empty Trigger,                             Address offset: 0x0A0 */
  __IOM uint32_t HTX;             /*!< Halt TX,                                             Address offset: 0x0A4 */
  __OM  uint32_t DMASA;           /*!< DMA Software Acknowledge,                            Address offset: 0x0A8 */
        uint32_t RESERVED2[5];    /*!< Reserved,                                                    0x0AC - 0x0BC */
  __IOM uint32_t DLF;             /*!< Divisor Latch Fractional Value,                      Address offset: 0x0C0 */
  __IOM uint32_t RAR;             /*!< Receive Address Register,                            Address offset: 0x0C4 */
  __IOM uint32_t TAR;             /*!< Transmit Address Register,                           Address offset: 0x0C8 */
  __IOM uint32_t EXTLCR;          /*!< Line Extended Control Register,                      Address offset: 0x0CC */
} UART_TypeDef;


/* ================================================================================ */
/* ==============        Direct Memory Access Controller (DMAC)       ============= */
/* ================================================================================ */
typedef struct
{
  struct {
    __IOM uint32_t SAR;           /*!< Channel x Source Address Register,                   Address offset: 0x000, 0x058, 0x0B0 */    uint32_t Undefined_SAR;
    __IOM uint32_t DAR;           /*!< Channel x Destination Address Register,              Address offset: 0x008, 0x060, 0x0B8 */    uint32_t Undefined_DAR;
          uint32_t RESERVED0[2];  /*!< Reserved */
    __IOM uint32_t CTLL;          /*!< Channel x Control Low Register,                      Address offset: 0x018, 0x070, 0x0C8 */
    __IOM uint32_t CTLH;          /*!< Channel x Control High Register,                     Address offset: 0x01C, 0x074, 0x0CC */
          uint32_t RESERVED1[8];  /*!< Reserved */
    __IOM uint32_t CFGL;          /*!< Channel x Configuration Low Register,                Address offset: 0x040, 0x098, 0x0F0 */
    __IOM uint32_t CFGH;          /*!< Channel x Configuration High Register,               Address offset: 0x044, 0x09C, 0x0F4 */
    __IOM uint32_t SGR;           /*!< Channel x Source Gather Register,                    Address offset: 0x048, 0x0A0, 0x0F8 */    uint32_t Undefined_SGR;
    __IOM uint32_t DSR;           /*!< Channel x Destination Scatter Register,              Address offset: 0x050, 0x0A8, 0x100 */    uint32_t Undefined_DSR;
  } Ch[3];
        uint32_t RESERVED2[110];  /*!< Reserved,                                                    0x108 - 0x2BC */
  __IM  uint32_t RawTfr;          /*!< Raw Status for IntTfr Interrupt,                     Address offset: 0x2C0 */    uint32_t Undefined_RawTfr;
  __IM  uint32_t RawBlock;        /*!< Raw Status for IntBlock Interrupt,                   Address offset: 0x2C8 */    uint32_t Undefined_RawBlock;
  __IM  uint32_t RawSrcTran;      /*!< Raw Status for IntSrcTran Interrupt,                 Address offset: 0x2D0 */    uint32_t Undefined_RawSrcTran;
  __IM  uint32_t RawDstTran;      /*!< Raw Status for IntDstTran Interrupt,                 Address offset: 0x2D8 */    uint32_t Undefined_RawDstTran;
  __IM  uint32_t RawErr;          /*!< Raw Status for IntErr Interrupt,                     Address offset: 0x2E0 */    uint32_t Undefined_RawErr;
  __IM  uint32_t StatusTfr;       /*!< Status for IntTfr Interrupt,                         Address offset: 0x2E8 */    uint32_t Undefined_StatusTfr;
  __IM  uint32_t StatusBlock;     /*!< Status for IntBlock Interrupt,                       Address offset: 0x2F0 */    uint32_t Undefined_StatusBlock;
  __IM  uint32_t StatusSrcTran;   /*!< Status for IntSrcTran Interrupt,                     Address offset: 0x2F8 */    uint32_t Undefined_StatusSrcTran;
  __IM  uint32_t StatusDstTran;   /*!< Status for IntDstTran Interrupt,                     Address offset: 0x300 */    uint32_t Undefined_StatusDstTran;
  __IM  uint32_t StatusErr;       /*!< Status for IntErr Interrupt,                         Address offset: 0x308 */    uint32_t Undefined_StatusErr;
  __IOM uint32_t MaskTfr;         /*!< Mask for IntTfr Interrupt,                           Address offset: 0x310 */    uint32_t Undefined_MaskTfr;
  __IOM uint32_t MaskBlock;       /*!< Mask for IntBlock Interrupt,                         Address offset: 0x318 */    uint32_t Undefined_MaskBlock;
  __IOM uint32_t MaskSrcTran;     /*!< Mask for IntSrcTran Interrupt,                       Address offset: 0x320 */    uint32_t Undefined_MaskSrcTran;
  __IOM uint32_t MaskDstTran;     /*!< Mask for IntDstTran Interrupt,                       Address offset: 0x328 */    uint32_t Undefined_MaskDstTran;
  __IOM uint32_t MaskErr;         /*!< Mask for IntErr Interrupt,                           Address offset: 0x330 */    uint32_t Undefined_MaskErr;
  __OM  uint32_t ClearTfr;        /*!< Clear for IntTfr Interrupt,                          Address offset: 0x338 */    uint32_t Undefined_ClearTfr;
  __OM  uint32_t ClearBlock;      /*!< Clear for IntBlock Interrupt,                        Address offset: 0x340 */    uint32_t Undefined_ClearBlock;
  __OM  uint32_t ClearSrcTran;    /*!< Clear for IntSrcTran Interrupt,                      Address offset: 0x348 */    uint32_t Undefined_ClearSrcTran;
  __OM  uint32_t ClearDstTran;    /*!< Clear for IntDstTran Interrupt,                      Address offset: 0x350 */    uint32_t Undefined_ClearDstTran;
  __OM  uint32_t ClearErr;        /*!< Clear for IntErr Interrupt,                          Address offset: 0x358 */    uint32_t Undefined_ClearErr;
  __IM  uint32_t StatusInt;       /*!< Status for each interrupt type,                      Address offset: 0x360 */    uint32_t Undefined_StatusInt;
  __IOM uint32_t ReqSrcReg;       /*!< Source Software Transaction Request Register,        Address offset: 0x368 */    uint32_t Undefined_ReqSrcReg;
  __IOM uint32_t ReqDstReg;       /*!< Destination Software Transaction Request Register,   Address offset: 0x370 */    uint32_t Undefined_ReqDstReg;
  __IOM uint32_t SglReqSrcReg;    /*!< Single Source Transaction Request Register,          Address offset: 0x378 */    uint32_t Undefined_SglReqSrcReg;
  __IOM uint32_t SglReqDstReg;    /*!< Single Destination Transaction Request Register,     Address offset: 0x380 */    uint32_t Undefined_SglReqDstReg;
  __IOM uint32_t LstSrcReg;       /*!< Last Source Transaction Request Register,            Address offset: 0x388 */    uint32_t Undefined_LstSrcReg;
  __IOM uint32_t LstDstReg;       /*!< Last Destination Transaction Request Register,       Address offset: 0x390 */    uint32_t Undefined_LstDstReg;
  __IOM uint32_t DmaCfgReg;       /*!< DMA Configuration Register,                          Address offset: 0x398 */    uint32_t Undefined_DmaCfgReg;
  __IOM uint32_t ChEnReg;         /*!< DMA Channel Enable Register,                         Address offset: 0x3A0 */    uint32_t Undefined_ChEnReg;
} DMAC_TypeDef;


/* ================================================================================ */
/* ==============          Serial Peripheral Interface (SPI)          ============= */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t CR0;             /*!< Control Register 0,                                  Address offset: 0x000 */
  __IOM uint32_t CR1;             /*!< Control Register 1,                                  Address offset: 0x004 */
  __IOM uint32_t SPIENR;          /*!< SPI Enable Register,                                 Address offset: 0x008 */
  __IOM uint32_t MWCR;            /*!< Microwire Control Register,                          Address offset: 0x00C */
  __IOM uint32_t SER;             /*!< Slave Enable Register,                               Address offset: 0x010 */
  __IOM uint32_t BAUDR;           /*!< Baud Rate Select,                                    Address offset: 0x014 */
  __IOM uint32_t TXFTLR;          /*!< Transmit FIFO Threshold Level,                       Address offset: 0x018 */
  __IOM uint32_t RXFTLR;          /*!< Receive FIFO Threshold Level,                        Address offset: 0x01C */
  __IM  uint32_t TXFLR;           /*!< Transmit FIFO Level Register,                        Address offset: 0x020 */
  __IM  uint32_t RXFLR;           /*!< Receive FIFO Level Register,                         Address offset: 0x024 */
  __IM  uint32_t SR;              /*!< Status Register,                                     Address offset: 0x028 */
  __IOM uint32_t IER;             /*!< Interrupt Enable Register,                           Address offset: 0x02C */
  __IM  uint32_t ISR;             /*!< Interrupt Status Register,                           Address offset: 0x030 */
  __IM  uint32_t RISR;            /*!< Raw Interrupt Status Register,                       Address offset: 0x034 */
  __IM  uint32_t TXOICR;          /*!< Transmit FIFO Overflow Interrupt Clear Register,     Address offset: 0x038 */
  __IM  uint32_t RXOICR;          /*!< Receive FIFO Overflow Interrupt Clear Register,      Address offset: 0x03C */
  __IM  uint32_t RXUICR;          /*!< Receive FIFO Underflow Interrupt Clear Register,     Address offset: 0x040 */
  __IM  uint32_t MSTICR;          /*!< Multi-Master Interrupt Clear Register,               Address offset: 0x044 */
  __IM  uint32_t ICR;             /*!< Interrupt Clear Register,                            Address offset: 0x048 */
  __IOM uint32_t DMACR;           /*!< DMA Control Register,                                Address offset: 0x04C */
  __IOM uint32_t DMATDLR;         /*!< DMA Transmit Data Level,                             Address offset: 0x050 */
  __IOM uint32_t DMARDLR;         /*!< DMA Receive Data Level,                              Address offset: 0x054 */
        uint32_t RESERVED0[2];    /*!< Reserved,                                                    0x058 - 0x05C */
  __IOM uint32_t DR;              /*!< Data Register,                                       Address offset: 0x060 */
        uint32_t RESERVED1[35];   /*!< Reserved,                                                    0x064 - 0x0EC */
  __IOM uint32_t RX_SAMPLE_DLY;   /*!< RX Sample Delay Register,                            Address offset: 0x0F0 */
  __IOM uint32_t ESPICR;          /*!< Enhanced SPI Control Register,                       Address offset: 0x0F4 */
} SPI_TypeDef;


/* ================================================================================ */
/* ==============            Inter-Integrated Circuit (I2C)           ============= */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t CON;                         /*!< I2C Control Register,                                          Address offset: 0x000 */
  __IOM uint32_t TAR;                         /*!< I2C Target Address Register,                                   Address offset: 0x004 */
  __IOM uint32_t SAR;                         /*!< I2C Slave Address Register,                                    Address offset: 0x008 */
  __IOM uint32_t HS_MADDR;                    /*!< I2C High Speed Master Mode Code Address Register,              Address offset: 0x00C */
  __IOM uint32_t DATA_CMD;                    /*!< I2C Rx/Tx Data Buffer and Command Register,                    Address offset: 0x010 */
  __IOM uint32_t SS_SCL_HCNT;                 /*!< Standard Speed I2C Clock SCL High Count Register,              Address offset: 0x014 */
  __IOM uint32_t SS_SCL_LCNT;                 /*!< Standard Speed I2C Clock SCL Low Count Register,               Address offset: 0x018 */
  __IOM uint32_t FS_SCL_HCNT;                 /*!< Fast Mode or Fast Mode Plus I2C Clock SCL High Count Register, Address offset: 0x01C */
  __IOM uint32_t FS_SCL_LCNT;                 /*!< Fast Mode or Fast Mode Plus I2C Clock SCL Low Count Register,  Address offset: 0x020 */
  __IOM uint32_t HS_SCL_HCNT;                 /*!< High Speed I2C Clock SCL High Count Register,                  Address offset: 0x024 */
  __IOM uint32_t HS_SCL_LCNT;                 /*!< High Speed I2C Clock SCL Low Count Register,                   Address offset: 0x028 */
  __IM  uint32_t INTR_STAT;                   /*!< I2C Interrupt Status Register,                                 Address offset: 0x02C */
  __IOM uint32_t INTR_MASK;                   /*!< I2C Interrupt Mask Register,                                   Address offset: 0x030 */
  __IM  uint32_t RAW_INTR_STAT;               /*!< I2C Raw Interrupt Status Register,                             Address offset: 0x034 */
  __IOM uint32_t RX_TL;                       /*!< I2C Receive FIFO Threshold Register,                           Address offset: 0x038 */
  __IOM uint32_t TX_TL;                       /*!< I2C Transmit FIFO Threshold Register,                          Address offset: 0x03C */
  __IM  uint32_t CLR_INTR;                    /*!< Clear Combined and Individual Interrupt Register,              Address offset: 0x040 */
  __IM  uint32_t CLR_RX_UNDER;                /*!< Clear RX_UNDER Interrupt Register,                             Address offset: 0x044 */
  __IM  uint32_t CLR_RX_OVER;                 /*!< Clear RX_OVER Interrupt Register,                              Address offset: 0x048 */
  __IM  uint32_t CLR_TX_OVER;                 /*!< Clear TX_OVER Interrupt Register,                              Address offset: 0x04C */
  __IM  uint32_t CLR_RD_REQ;                  /*!< Clear RD_REQ Interrupt Register,                               Address offset: 0x050 */
  __IM  uint32_t CLR_TX_ABRT;                 /*!< Clear TX_ABRT Interrupt Register,                              Address offset: 0x054 */
  __IM  uint32_t CLR_RX_DONE;                 /*!< Clear RX_DONE Interrupt Register,                              Address offset: 0x058 */
  __IM  uint32_t CLR_ACTIVITY;                /*!< Clear ACTIVITY Interrupt Register,                             Address offset: 0x05C */
  __IM  uint32_t CLR_STOP_DET;                /*!< Clear STOP_DET Interrupt Register,                             Address offset: 0x060 */
  __IM  uint32_t CLR_START_DET;               /*!< Clear START_DET Interrupt Register,                            Address offset: 0x064 */
  __IM  uint32_t CLR_GEN_CALL;                /*!< Clear GEN_CALL Interrupt Register,                             Address offset: 0x068 */
  __IOM uint32_t ENABLE;                      /*!< I2C Enable Register,                                           Address offset: 0x06C */
  __IM  uint32_t STATUS;                      /*!< I2C Status Register,                                           Address offset: 0x070 */
  __IM  uint32_t TXFLR;                       /*!< I2C Transmit FIFO Level Register,                              Address offset: 0x074 */
  __IM  uint32_t RXFLR;                       /*!< I2C Receive FIFO Level Register,                               Address offset: 0x078 */
  __IOM uint32_t SDA_HOLD;                    /*!< I2C SDA Hold Time Length Register,                             Address offset: 0x07C */
  __IM  uint32_t TX_ABRT_SOURCE;              /*!< I2C Transmit Abort Source Register,                            Address offset: 0x080 */
  __IOM uint32_t SLV_DATA_NACK_ONLY;          /*!< Generate Slave Data NACK Register,                             Address offset: 0x084 */
  __IOM uint32_t DMA_CR;                      /*!< DMA Control Register,                                          Address offset: 0x088 */
  __IOM uint32_t DMA_TDLR;                    /*!< DMA Transmit Data Level Register,                              Address offset: 0x08C */
  __IOM uint32_t DMA_RDLR;                    /*!< DMA Receive Data Level Register,                               Address offset: 0x090 */
  __IOM uint32_t SDA_SETUP;                   /*!< I2C SDA Setup Register,                                        Address offset: 0x094 */
  __IOM uint32_t ACK_GENERAL_CALL;            /*!< I2C ACK General Call Register,                                 Address offset: 0x098 */
  __IM  uint32_t ENABLE_STATUS;               /*!< I2C Enable Status Register,                                    Address offset: 0x09C */
  __IOM uint32_t FS_SPKLEN;                   /*!< I2C SS, FS or FM+ spike suppression limit,                     Address offset: 0x0A0 */
  __IOM uint32_t HS_SPKLEN;                   /*!< I2C HS spike suppression limit,                                Address offset: 0x0A4 */
  __IM  uint32_t CLR_RESTART_DET;             /*!< Clear RESTART_DET Interrupt Register,                          Address offset: 0x0A8 */
  __IOM uint32_t SCL_STUCK_AT_LOW_TIMEOUT;    /*!< I2C SCL Stuck at Low Timeout,                                  Address offset: 0x0AC */
  __IOM uint32_t SDA_STUCK_AT_LOW_TIMEOUT;    /*!< I2C SDA Stuck at Low Timeout,                                  Address offset: 0x0B0 */
  __IM  uint32_t CLR_SCL_STUCK_DET;           /*!< Clear SCL Stuck at Low Detect Interrupt Register,              Address offset: 0x0B4 */
        uint32_t RESERVED0;                   /*!< Reserved,                                                                      0x0B8 */
  __IOM uint32_t SMBUS_CLK_LOW_SEXT;          /*!< SMBus Slave Clock Extend Timeout Register,                     Address offset: 0x0BC */
  __IOM uint32_t SMBUS_CLK_LOW_MEXT;          /*!< SMBus Master Clock Extend Timeout Register,                    Address offset: 0x0C0 */
  __IOM uint32_t SMBUS_THIGH_MAX_IDLE_COUNT;  /*!< SMBus Master THigh MAX Bus-idle count Register,                Address offset: 0x0C4 */
  __IM  uint32_t SMBUS_INTR_STAT;             /*!< SMBUS Interrupt Status Register,                               Address offset: 0x0C8 */
  __IOM uint32_t SMBUS_INTR_MASK;             /*!< SMBus Interrupt Mask Register,                                 Address offset: 0x0CC */
  __IM  uint32_t SMBUS_RAW_INTR_STAT;         /*!< SMBus Raw Interrupt Status Register,                           Address offset: 0x0D0 */
  __OM  uint32_t CLR_SMBUS_INTR;              /*!< SMBus Clear Interrupt Register,                                Address offset: 0x0D4 */
  __IOM uint32_t OPTIONAL_SAR;                /*!< I2C Optional Slave Address Register,                           Address offset: 0x0D8 */
  __IOM uint32_t SMBUS_UDID_LSB;              /*!< SMBUS ARP UDID LSB Register,                                   Address offset: 0x0DC */
} I2C_TypeDef;


/* ================================================================================ */
/* ================              Inter-IC Sound (I2S)              ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t IER;             /*!< I2S Enable Register,                                 Address offset: 0x000 */
  __IOM uint32_t IRER;            /*!< I2S Receiver Block Enable Register,                  Address offset: 0x004 */
  __IOM uint32_t ITER;            /*!< I2S Transmitter Block Enable Register,               Address offset: 0x008 */
  __IOM uint32_t CER;             /*!< Clock Enable Register,                               Address offset: 0x00C */
  __IOM uint32_t CCR;             /*!< Clock Configuration Register,                        Address offset: 0x010 */
  __OM  uint32_t RXFFR;           /*!< Receiver Block FIFO Reset Register,                  Address offset: 0x014 */
  __OM  uint32_t TXFFR;           /*!< Transmitter Block FIFO Reset Register,               Address offset: 0x018 */
        uint32_t RESERVED0;       /*!< Reserved,                                                            0x01C */
  struct {
    union {
    __IM  uint32_t LRBR;          /*!< Left Receive Buffer Register,                        Address offset: 0x020, 0x060 */
    __OM  uint32_t LTHR;          /*!< Left Transmit Holding Register,                      Address offset: 0x020, 0x060 */
    };
    union {
    __IM  uint32_t RRBR;          /*!< Right Receive Buffer Register,                       Address offset: 0x024, 0x064 */
    __OM  uint32_t RTHR;          /*!< Right Transmit Holding Register,                     Address offset: 0x024, 0x064 */
    };
    __IOM uint32_t RER;           /*!< Receive Enable Register,                             Address offset: 0x028, 0x068 */
    __IOM uint32_t TER;           /*!< Transmit Enable Register,                            Address offset: 0x02C, 0x06C */
    __IOM uint32_t RCR;           /*!< Receive Configuration Register,                      Address offset: 0x030, 0x070 */
    __IOM uint32_t TCR;           /*!< Transmit Configuration Register,                     Address offset: 0x034, 0x074 */
    __IM  uint32_t ISR;           /*!< Interrupt Status Register,                           Address offset: 0x038, 0x078 */
    __IOM uint32_t IMR;           /*!< Interrupt Mask Register,                             Address offset: 0x03C, 0x07C */
    __IM  uint32_t ROR;           /*!< Receive Overrun Register,                            Address offset: 0x040, 0x080 */
    __IM  uint32_t TOR;           /*!< Transmit Overrun Register,                           Address offset: 0x044, 0x084 */
    __IOM uint32_t RFCR;          /*!< Receive FIFO Configuration Register,                 Address offset: 0x048, 0x088 */
    __IOM uint32_t TFCR;          /*!< Transmit FIFO Configuration Register,                Address offset: 0x04C, 0x08C */
    __OM  uint32_t RFF;           /*!< Receive FIFO Flush Register,                         Address offset: 0x050, 0x090 */
    __OM  uint32_t TFF;           /*!< Transmit FIFO Flush Register,                        Address offset: 0x054, 0x094 */
          uint32_t RESERVED0;
          uint32_t RESERVED1;
  } Ch[2];
        uint32_t RESERVED1[72];   /*!< Reserved,                                                    0x0A0 - 0x1BC */
  __IM  uint32_t RXDMA;           /*!< Receiver Block DMA Register,                         Address offset: 0x1C0 */
  __OM  uint32_t RRXDMA;          /*!< Reset Receiver Block DMA Register,                   Address offset: 0x1C4 */
  __OM  uint32_t TXDMA;           /*!< Transmitter Block DMA Register,                      Address offset: 0x1C8 */
  __OM  uint32_t RTXDMA;          /*!< Reset Transmitter Block DMA Register,                Address offset: 0x1CC */
} I2S_TypeDef;


/* ================================================================================ */
/* ==============            Random Number Generator (RNG)            ============= */
/* ================================================================================ */
typedef struct
{
  __IM  uint32_t RAND;      /*!< RNG generated 8bit random number register,                 Address offset: 0x000 */
  __IOM uint32_t STOP;      /*!< RNG stop register,                                         Address offset: 0x004 */
} RNG_TypeDef;


/* ================================================================================ */
/* ==============             Universal Serial Bus (USB)              ============= */
/* ================================================================================ */
typedef struct
{
  __IOM uint8_t FADDR;         /*!< Function address register,                                     Address offset: 0x000 */
  __IOM uint8_t POWER;         /*!< Power management register,                                     Address offset: 0x001 */
  __IM  uint8_t INTRIN;        /*!< Interrupt register for Endpoint 0 plus IN Endpoints 1 to 3,    Address offset: 0x002 */
        uint8_t RESERVED0;     /*!< Reserved,                                                                      0x003 */
  __IM  uint8_t INTROUT;       /*!< Interrupt register for OUT Endpoints 1 to 3,                   Address offset: 0x004 */
        uint8_t RESERVED1;     /*!< Reserved,                                                                      0x005 */
  __IM  uint8_t INTRUSB;       /*!< Interrupt register for common USB interrupts,                  Address offset: 0x006 */
  __IOM uint8_t INTRINE;       /*!< Interrupt enable register for IntrIn,                          Address offset: 0x007 */
        uint8_t RESERVED2;     /*!< Reserved,                                                                      0x008 */
  __IOM uint8_t INTROUTE;      /*!< Interrupt enable register for IntrOut,                         Address offset: 0x009 */
        uint8_t RESERVED3;     /*!< Reserved,                                                                      0x00A */
  __IOM uint8_t INTRUSBE;      /*!< Interrupt enable register for IntrUSB,                         Address offset: 0x00B */
  __IM  uint8_t FRAMEL;        /*!< Frame number bits 0 to 7,                                      Address offset: 0x00C */
  __IM  uint8_t FRAMEH;        /*!< Frame number bits 8 to 10,                                     Address offset: 0x00D */
  __IOM uint8_t INDEX;         /*!< Index register,                                                Address offset: 0x00E */
        uint8_t RESERVED4;     /*!< Reserved,                                                                      0x00F */

  __IOM uint8_t INMAXP;        /*!< Maximum packet size for IN endpoint,                           Address offset: 0x010 */
  union {
  __IOM uint8_t CSR0;          /*!< Control Status register for Endpoint 0,                        Address offset: 0x011 */
  __IOM uint8_t INCSR1;        /*!< Control Status register 1 for IN endpoint,                     Address offset: 0x011 */
  };
  __IOM uint8_t INCSR2;        /*!< Control Status register 2 for IN endpoint,                     Address offset: 0x012 */
  __IOM uint8_t OUTMAXP;       /*!< Maximum packet size for OUT endpoint,                          Address offset: 0x013 */
  __IOM uint8_t OUTCSR1;       /*!< Control Status register 1 for OUT endpoint,                    Address offset: 0x014 */
  __IOM uint8_t OUTCSR2;       /*!< Control Status register 2 for OUT endpoint,                    Address offset: 0x015 */
  union {
  __IM  uint8_t COUNT0;        /*!< Number of received bytes in Endpoint 0 FIFO,                   Address offset: 0x016 */
  __IM  uint8_t OUTCOUNTL;     /*!< Number of bytes in OUT endpoint FIFO (lower byte),             Address offset: 0x016 */
  };
  __IM  uint8_t OUTCOUNTH;     /*!< Number of bytes in OUT endpoint FIFO (upper byte),             Address offset: 0x017 */
        uint8_t RESERVED5[8];  /*!< Reserved,                                                              0x018 - 0x01F */

  __IOM uint32_t FIFO[16];        /*!< FIFOs for Endpoints 0 to 15 (must accessed by bytes),       Address offset: 0x020 - 0x05F */
        uint32_t RESERVED6[104];  /*!< Reserved,                                                                   0x060 - 0x1FC */

  __IOM uint32_t DMAINTR;         /*!< DMA Interrupt Register,                                Address offset: 0x200 */
  struct {
    __IOM uint32_t CNTL;          /*!< DMA Control Register for DMA channel x,                Address offset: 0x204, 0x214, 0x224 */
    __IOM uint32_t ADDR;          /*!< DMA Address Register for DMA channel x,                Address offset: 0x208, 0x218, 0x228 */
    __IOM uint32_t COUNT;         /*!< DMA Count Register for DMA channel x,                  Address offset: 0x20C, 0x21C, 0x22C */
          uint32_t RESERVED;
  } DMACH[8];
} USB_TypeDef;


/* ================================================================================ */
/* ================       Analog to Digital Converter (ADC)        ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t SR;              /*!< ADC status register,                             Address offset: 0x000 */
  __IOM uint32_t CR1;             /*!< ADC control register 1,                          Address offset: 0x004 */
  __IOM uint32_t CR2;             /*!< ADC control register 2,                          Address offset: 0x008 */
  __IOM uint32_t SMPR1;           /*!< ADC sample time register 1,                      Address offset: 0x00C */
  __IOM uint32_t SMPR2;           /*!< ADC sample time register 2,                      Address offset: 0x010 */
  __IOM uint32_t JOFR1;           /*!< ADC injected channel data offset register 1,     Address offset: 0x014 */
  __IOM uint32_t JOFR2;           /*!< ADC injected channel data offset register 2,     Address offset: 0x018 */
  __IOM uint32_t JOFR3;           /*!< ADC injected channel data offset register 3,     Address offset: 0x01C */
  __IOM uint32_t JOFR4;           /*!< ADC injected channel data offset register 4,     Address offset: 0x020 */
  __IOM uint32_t HTR;             /*!< ADC watchdog high threshold register,            Address offset: 0x024 */
  __IOM uint32_t LTR;             /*!< ADC watchdog low threshold register,             Address offset: 0x028 */
  __IOM uint32_t SQR1;            /*!< ADC regular sequence register 1,                 Address offset: 0x02C */
  __IOM uint32_t SQR2;            /*!< ADC regular sequence register 2,                 Address offset: 0x030 */
  __IOM uint32_t SQR3;            /*!< ADC regular sequence register 3,                 Address offset: 0x034 */
  __IOM uint32_t JSQR;            /*!< ADC injected sequence register,                  Address offset: 0x038 */
  __IOM uint32_t JDR1;            /*!< ADC injected data register 1,                    Address offset: 0x03C */
  __IOM uint32_t JDR2;            /*!< ADC injected data register 2,                    Address offset: 0x040 */
  __IOM uint32_t JDR3;            /*!< ADC injected data register 3,                    Address offset: 0x044 */
  __IOM uint32_t JDR4;            /*!< ADC injected data register 4,                    Address offset: 0x048 */
  __IOM uint32_t DR;              /*!< ADC regular data register,                       Address offset: 0x04C */
  __IOM uint32_t HS;              /*!< ADC hold&sample control register,                Address offset: 0x050 */
  __IOM uint32_t CR3;             /*!< ADC control register 3,                          Address offset: 0x054 */
  __IOM uint32_t JDMAR;           /*!< Used for Injection channel DMA transfer,         Address offset: 0x058 */
} ADC_TypeDef;


/* ================================================================================ */
/* ================                      ANCTL                     ================ */
/* ================================================================================ */
typedef struct
{
        uint32_t RESERVED0[7];    /*!< Reserved,                                                0x000 - 0x018 */
  __IOM uint32_t BGCR2;           /*!< BandGap control register 2,                      Address offset: 0x01C */
        uint32_t RESERVED1[3];    /*!< Reserved,                                                0x020 - 0x028 */
  __IOM uint32_t MHSIENR;         /*!< MHSI enable register,                            Address offset: 0x02C */
  __IOM uint32_t MHSISR;          /*!< MHSI status register,                            Address offset: 0x030 */
        uint32_t RESERVED2;       /*!< Reserved,                                                        0x034 */
  __IOM uint32_t FHSIENR;         /*!< FHSI enable register,                            Address offset: 0x038 */
  __IOM uint32_t FHSISR;          /*!< FHSI status register,                            Address offset: 0x03C */
        uint32_t RESERVED3;       /*!< Reserved,                                                        0x040 */
  __IOM uint32_t LSIENR;          /*!< LSI enable register,                             Address offset: 0x044 */
  __IOM uint32_t LSISR;           /*!< LSI status register,                             Address offset: 0x048 */
  __IOM uint32_t HSECR0;          /*!< HSE control register 0,                          Address offset: 0x04C */
  __IOM uint32_t HSECR1;          /*!< HSE control register 1,                          Address offset: 0x050 */
        uint32_t RESERVED4;       /*!< Reserved,                                                        0x054 */
  __IOM uint32_t HSESR;           /*!< HSE status register,                             Address offset: 0x058 */
        uint32_t RESERVED5[6];    /*!< Reserved,                                                0x05C - 0x070 */
  __IOM uint32_t PLLCR;           /*!< PLL control register,                            Address offset: 0x074 */
  __IOM uint32_t PLLENR;          /*!< PLl enable register,                             Address offset: 0x078 */
  __IOM uint32_t PLLSR;           /*!< PLL status register,                             Address offset: 0x07C */
  __IOM uint32_t PVDCR;           /*!< PVD control register,                            Address offset: 0x080 */
  __IOM uint32_t PVDENR;          /*!< PVD enable register,                             Address offset: 0x084 */
        uint32_t RESERVED6;       /*!< Reserved,                                                        0x088 */
  __IOM uint32_t SARENR;          /*!< SAR ADC enable register,                         Address offset: 0x08C */
  __IOM uint32_t USBPCR;          /*!< USB PHY control register,                        Address offset: 0x090 */
  __IOM uint32_t PORCR;           /*!< POR control register,                            Address offset: 0x094 */
  __IOM uint32_t CMPACR;          /*!< CMPA control register,                           Address offset: 0x098 */
  __IOM uint32_t CMPBCR;          /*!< CMPB control register,                           Address offset: 0x09C */
  __IOM uint32_t ISR;             /*!< Interrupt status register,                       Address offset: 0x0A0 */
  __IOM uint32_t IER;             /*!< Interrupt enable register,                       Address offset: 0x0A4 */
  __IOM uint32_t ICR;             /*!< Interrupt clear register,                        Address offset: 0x0A8 */
  __IOM uint32_t CMPASR;          /*!< CMPA status register,                            Address offset: 0x0AC */
  __IOM uint32_t CMPBSR;          /*!< CMPB status register,                            Address offset: 0x0B0 */
  __IOM uint32_t DCSSENR;         /*!< DCSS enable register,                            Address offset: 0x0B4 */
  __IOM uint32_t DCSSCR;          /*!< DCSS control register,                           Address offset: 0x0B8 */
} ANCTL_TypeDef;


/* ================================================================================ */
/* ================         Reset and Clock Control (RCC)          ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t PLLPRE;          /*!< RCC PLL prescaler register,                      Address offset: 0x000 */
  __IOM uint32_t PLLSRC;          /*!< RCC PLL source register,                         Address offset: 0x004 */
  __IOM uint32_t MAINCLKSRC;      /*!< RCC main clock source register,                  Address offset: 0x008 */
  __IOM uint32_t MAINCLKUEN;      /*!< RCC main clock update enable register,           Address offset: 0x00C */
        uint32_t RESERVED0;       /*!< Reserved,                                                        0x010 */
  __IOM uint32_t USBPRE;          /*!< RCC USB prescaler register,                      Address offset: 0x014 */
  __IOM uint32_t AHBPRE;          /*!< RCC AHB prescaler register,                      Address offset: 0x018 */
  __IOM uint32_t APB1PRE;         /*!< RCC APB1 prescaler register,                     Address offset: 0x01C */
  __IOM uint32_t APB2PRE;         /*!< RCC APB2 prescaler register,                     Address offset: 0x020 */
  __IOM uint32_t MCLKPRE;         /*!< RCC MCLK prescaler register,                     Address offset: 0x024 */
  __IOM uint32_t I2SPRE;          /*!< RCC I2S prescaler register,                      Address offset: 0x028 */
  __IOM uint32_t MCLKSRC;         /*!< RCC MCLK source register,                        Address offset: 0x02C */
        uint32_t RESERVED1;       /*!< Reserved,                                                        0x030 */
  __IOM uint32_t USBFIFOCLKSRC;   /*!< RCC USB FIFO clock source register,              Address offset: 0x034 */
  __IOM uint32_t MCOSEL;          /*!< RCC MCO select register,                         Address offset: 0x038 */
  __IOM uint32_t AHBENR0;         /*!< RCC AHB peripheral clock enable register 0,      Address offset: 0x03C */
  __IOM uint32_t AHBENR1;         /*!< RCC AHB peripheral clock enable register 1,      Address offset: 0x040 */
  __IOM uint32_t AHBENR2;         /*!< RCC AHB peripheral clock enable register 2,      Address offset: 0x044 */
  __IOM uint32_t APB1ENR;         /*!< RCC APB1 peripheral clock enable register,       Address offset: 0x048 */
  __IOM uint32_t APB2ENR;         /*!< RCC APB2 peripheral clock enable register,       Address offset: 0x04C */
        uint32_t RESERVED2[3];    /*!< Reserved,                                                0x050 - 0x058 */
  __IOM uint32_t RNGCLKENR;       /*!< RCC RNG clock enable register,                   Address offset: 0x05C */
  __IOM uint32_t PCLKENR;         /*!< RCC panel PCLK clock enable register,            Address offset: 0x060 */
  __IOM uint32_t IWDGCLKENR;      /*!< RCC IWDG clock enable register,                  Address offset: 0x064 */
        uint32_t RESERVED4;       /*!< Reserved,                                                        0x068 */
  __IOM uint32_t USBCLKENR;       /*!< RCC USB clock enable register,                   Address offset: 0x06C */
  __IOM uint32_t I2SCLKENR;       /*!< RCC I2S SCLK enable register,                    Address offset: 0x070 */
  __IOM uint32_t SPIS1CLKENR;     /*!< RCC SPIS1 clock enable register,                 Address offset: 0x074 */
  __IOM uint32_t SPIS2CLKENR;     /*!< RCC SPIS2 clock enable register,                 Address offset: 0x078 */
  __IOM uint32_t USBFIFOCLKENR;   /*!< RCC USB FIFO clock enable register,              Address offset: 0x07C */
        uint32_t RESERVED5[2];    /*!< Reserved,                                                0x080 - 0x084 */
  __IOM uint32_t AHBRSTR1;        /*!< RCC AHB peripheral reset register 1,             Address offset: 0x088 */
        uint32_t RESERVED6;       /*!< Reserved,                                                        0x08C */
  __IOM uint32_t APB1RSTR;        /*!< RCC APB1 peripheral reset register,              Address offset: 0x090 */
  __IOM uint32_t APB2RSTR;        /*!< RCC APB2 peripheral reset register,              Address offset: 0x094 */
        uint32_t RESERVED7[8];    /*!< Reserved,                                                0x098 - 0x0B4 */
  __IOM uint32_t I2SCLKRSTR;      /*!< RCC I2S SCLK reset register,                     Address offset: 0x0B8 */
        uint32_t RESERVED8[3];    /*!< Reserved,                                                0x0BC - 0x0C4 */
  __IOM uint32_t CLRRSTSTAT;      /*!< RCC clear reset status register,                 Address offset: 0x0C8 */
        uint32_t RESERVED9[2];    /*!< Reserved,                                                0x0CC - 0x0D0 */
  __IOM uint32_t BDRSTR;          /*!< RCC Batt domain reset register,                  Address offset: 0x0D4 */
  __IOM uint32_t LSI2RTCENR;      /*!< RCC LSI to RTC clock enable register,            Address offset: 0x0D8 */
  __IOM uint32_t HSE2RTCENR;      /*!< RCC HSE to RTC clock enable register,            Address offset: 0x0DC */
        uint32_t RESERVED10[8];   /*!< Reserved,                                                0x0E0 - 0x0FC */
  __IOM uint32_t RSTSTAT;         /*!< RCC reset status register,                       Address offset: 0x100 */
} RCC_TypeDef;


/* ================================================================================ */
/* ================              Power Control (PWR)               ================ */
/* ================================================================================ */
typedef struct
{
  __IOM uint32_t CR0;         /*!< Control register 0,                                  Address offset: 0x000 */
  __OM  uint32_t CR1;         /*!< Control register 1,                                  Address offset: 0x004 */
  __IOM uint32_t CR2;         /*!< Control register 2,                                  Address offset: 0x008 */
        uint32_t RESERVED0;   /*!< Reserved,                                                            0x00C */
  __IM  uint32_t SR0;         /*!< Status register 0,                                   Address offset: 0x010 */
  __IM  uint32_t SR1;         /*!< Status register 1,                                   Address offset: 0x014 */
  __IOM uint32_t GPREG0;      /*!< General purpose register 0,                          Address offset: 0x018 */
  __IOM uint32_t GPREG1;      /*!< General purpose register 1,                          Address offset: 0x01C */
  __IOM uint32_t CFGR;        /*!< PWR configuration register,                          Address offset: 0x020 */
        uint32_t RESERVED1;   /*!< Reserved,                                                            0x024 */
  __OM  uint32_t ANAKEY1;     /*!< ANCTL write enable key register 0,                   Address offset: 0x028 */
  __OM  uint32_t ANAKEY2;     /*!< ANCTL write enable key register 1,                   Address offset: 0x02C */
} PWR_TypeDef;


/* ================================================================================ */
/* ================               Debug MCU (DBGMCU)               ================ */
/* ================================================================================ */
typedef struct
{
        uint32_t RESERVED0;     /*!< Reserved,                                                  0x000 */
  __IOM uint32_t CR;            /*!< Debug MCU configuration register,          Address offset: 0x004 */
} DBGMCU_TypeDef;


/* --------  End of section using anonymous unions and disabling warnings  -------- */
#if   defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */
#define FLASH_BASE                ((uint32_t)0x08000000UL)          /*!< FLASH base address in the alias region */
#define SRAM_BASE                 ((uint32_t)0x20000000UL)          /*!< SRAM base address in the alias region */
#define PERIPH_BASE               ((uint32_t)0x40000000UL)          /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE              ((uint32_t)0x22000000UL)          /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE            ((uint32_t)0x42000000UL)          /*!< Peripheral base address in the bit-band region */

#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x08000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x10000)


#define GPIOA_BASE            (APB1PERIPH_BASE + 0x0000)    // 4000_0000
#define GPIOB_BASE            (APB1PERIPH_BASE + 0x0400)    // 4000_0400
#define GPIOC_BASE            (APB1PERIPH_BASE + 0x0800)    // 4000_0800
#define GPIOD_BASE            (APB1PERIPH_BASE + 0x0C00)    // 4000_0C00
#define AFIO_BASE             (APB1PERIPH_BASE + 0x1400)    // 4000_1400
#define EXTI_BASE             (APB1PERIPH_BASE + 0x1800)    // 4000_1800
#define TIM1_BASE             (APB1PERIPH_BASE + 0x1C00)    // 4000_1C00
#define TIM2_BASE             (APB1PERIPH_BASE + 0x2000)    // 4000_2000
#define TIM3_BASE             (APB1PERIPH_BASE + 0x2400)    // 4000_2400
#define TIM4_BASE             (APB1PERIPH_BASE + 0x2800)    // 4000_2800
#define QSPI_BASE             (APB1PERIPH_BASE + 0x3000)    // 4000_3000
#define SPIS1_BASE            (APB1PERIPH_BASE + 0x3400)    // 4000_3400
#define UART1_BASE            (APB1PERIPH_BASE + 0x3800)    // 4000_3800
#define ADC_BASE              (APB1PERIPH_BASE + 0x3C00)    // 4000_3C00
#define DMAC1_BASE            (APB1PERIPH_BASE + 0x7C00)    // 4000_7C00

#define UART2_BASE            (APB2PERIPH_BASE + 0x0000)    // 4000_8000
#define UART3_BASE            (APB2PERIPH_BASE + 0x0400)    // 4000_8400
#define I2C1_BASE             (APB2PERIPH_BASE + 0x0800)    // 4000_8800
#define I2C2_BASE             (APB2PERIPH_BASE + 0x0C00)    // 4000_8C00
#define SPIM2_BASE            (APB2PERIPH_BASE + 0x1000)    // 4000_9000
#define SPIS2_BASE            (APB2PERIPH_BASE + 0x1400)    // 4000_9400
#define WWDG_BASE             (APB2PERIPH_BASE + 0x1800)    // 4000_9800
#define I2S_BASE              (APB2PERIPH_BASE + 0x3400)    // 4000_B400
#define RNG_BASE              (APB2PERIPH_BASE + 0x3800)    // 4000_B800
#define LED_BASE              (APB2PERIPH_BASE + 0x3C00)    // 4000_BC00
#define DMAC2_BASE            (APB2PERIPH_BASE + 0x7C00)    // 4000_FC00

#define PWR_BASE              (AHBPERIPH_BASE + 0x0000)  // 4001_0000
#define ANCTL_BASE            (AHBPERIPH_BASE + 0x0400)  // 4001_0400
#define IWDG_BASE             (AHBPERIPH_BASE + 0x0800)  // 4001_0800
#define RCC_BASE              (AHBPERIPH_BASE + 0x0C00)  // 4001_0C00
#define USB_BASE              (AHBPERIPH_BASE + 0x4000)  // 4001_4000
#define CRC_BASE              (AHBPERIPH_BASE + 0x4800)  // 4001_4800
#define SFM_BASE              (AHBPERIPH_BASE + 0x4C00)  // 4001_4C00
#define CACHE_BASE            (AHBPERIPH_BASE + 0x5400)  // 4001_5400
#define RTC_BASE              (AHBPERIPH_BASE + 0x5800)  // 4001_5800
#define BKP_BASE              (AHBPERIPH_BASE + 0x5C00)  // 4001_5C00
#define ISO_BASE              (AHBPERIPH_BASE + 0x6000)  // 4001_6000
#define SYS_BASE              (AHBPERIPH_BASE + 0x6400)  // 4001_6400
#define FMC_BASE              (AHBPERIPH_BASE + 0x7800)  // 4001_7800

#define DBGMCU_BASE           (0xE0042000UL)  // E004_2000





/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */
#define GPIOA                     ((       GPIO_TypeDef *)        GPIOA_BASE)
#define GPIOB                     ((       GPIO_TypeDef *)        GPIOB_BASE)
#define GPIOC                     ((       GPIO_TypeDef *)        GPIOC_BASE)
#define GPIOD                     ((       GPIO_TypeDef *)        GPIOD_BASE)
#define AFIO                      ((       AFIO_TypeDef *)         AFIO_BASE)
#define EXTI                      ((       EXTI_TypeDef *)         EXTI_BASE)
#define TIM1                      ((        TIM_TypeDef *)         TIM1_BASE)
#define TIM2                      ((        TIM_TypeDef *)         TIM2_BASE)
#define TIM3                      ((        TIM_TypeDef *)         TIM3_BASE)
#define TIM4                      ((        TIM_TypeDef *)         TIM4_BASE)
#define QSPI                      ((        SPI_TypeDef *)         QSPI_BASE)
#define SPIS1                     ((        SPI_TypeDef *)        SPIS1_BASE)
#define UART1                     ((       UART_TypeDef *)        UART1_BASE)
#define ADC                       ((        ADC_TypeDef *)          ADC_BASE)
#define DMAC1                     ((       DMAC_TypeDef *)        DMAC1_BASE)

#define UART2                     ((       UART_TypeDef *)        UART2_BASE)
#define UART3                     ((       UART_TypeDef *)        UART3_BASE)
#define I2C1                      ((        I2C_TypeDef *)         I2C1_BASE)
#define I2C2                      ((        I2C_TypeDef *)         I2C2_BASE)
#define SPIM2                     ((        SPI_TypeDef *)        SPIM2_BASE)
#define SPIS2                     ((        SPI_TypeDef *)        SPIS2_BASE)
#define WWDG                      ((       WWDG_TypeDef *)         WWDG_BASE)
#define I2S                       ((        I2S_TypeDef *)          I2S_BASE)
#define RNG                       ((        RNG_TypeDef *)          RNG_BASE)
#define LED                       ((        LED_TypeDef *)          LED_BASE)
#define DMAC2                     ((       DMAC_TypeDef *)        DMAC2_BASE)

#define PWR                       ((       PWR_TypeDef *)           PWR_BASE)
#define ANCTL                     ((     ANCTL_TypeDef *)         ANCTL_BASE)
#define IWDG                      ((      IWDG_TypeDef *)          IWDG_BASE)
#define RCC                       ((       RCC_TypeDef *)           RCC_BASE)
#define USB                       ((        USB_TypeDef *)          USB_BASE)
#define CRC                       ((        CRC_TypeDef *)          CRC_BASE)
#define SFM                       ((        SFM_TypeDef *)          SFM_BASE)
#define CACHE                     ((      CACHE_TypeDef *)        CACHE_BASE)
#define RTC                       ((        RTC_TypeDef *)          RTC_BASE)
#define BKP                       ((        BKP_TypeDef *)          BKP_BASE)
#define ISO                       ((        ISO_TypeDef *)          ISO_BASE)
#define SYS                       ((        SYS_TypeDef *)          SYS_BASE)
#define FMC                       ((        FMC_TypeDef *)          FMC_BASE)

#define DBGMCU                    ((     DBGMCU_TypeDef *)       DBGMCU_BASE)







/*------------------------------------------------------------------------------------------------------*/
/*---                                  General-purpose I/Os (GPIO)                                   ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for GPIO_MODER register  ******************************/
#define GPIO_MODER_MODER0_Pos         (0U)
#define GPIO_MODER_MODER0_Msk         (0x3U << GPIO_MODER_MODER0_Pos)
#define GPIO_MODER_MODER1_Pos         (2U)
#define GPIO_MODER_MODER1_Msk         (0x3U << GPIO_MODER_MODER1_Pos)
#define GPIO_MODER_MODER2_Pos         (4U)
#define GPIO_MODER_MODER2_Msk         (0x3U << GPIO_MODER_MODER2_Pos)
#define GPIO_MODER_MODER3_Pos         (6U)
#define GPIO_MODER_MODER3_Msk         (0x3U << GPIO_MODER_MODER3_Pos)
#define GPIO_MODER_MODER4_Pos         (8U)
#define GPIO_MODER_MODER4_Msk         (0x3U << GPIO_MODER_MODER4_Pos)
#define GPIO_MODER_MODER5_Pos         (10U)
#define GPIO_MODER_MODER5_Msk         (0x3U << GPIO_MODER_MODER5_Pos)
#define GPIO_MODER_MODER6_Pos         (12U)
#define GPIO_MODER_MODER6_Msk         (0x3U << GPIO_MODER_MODER6_Pos)
#define GPIO_MODER_MODER7_Pos         (14U)
#define GPIO_MODER_MODER7_Msk         (0x3U << GPIO_MODER_MODER7_Pos)
#define GPIO_MODER_MODER8_Pos         (16U)
#define GPIO_MODER_MODER8_Msk         (0x3U << GPIO_MODER_MODER8_Pos)
#define GPIO_MODER_MODER9_Pos         (18U)
#define GPIO_MODER_MODER9_Msk         (0x3U << GPIO_MODER_MODER9_Pos)
#define GPIO_MODER_MODER10_Pos        (20U)
#define GPIO_MODER_MODER10_Msk        (0x3U << GPIO_MODER_MODER10_Pos)
#define GPIO_MODER_MODER11_Pos        (22U)
#define GPIO_MODER_MODER11_Msk        (0x3U << GPIO_MODER_MODER11_Pos)
#define GPIO_MODER_MODER12_Pos        (24U)
#define GPIO_MODER_MODER12_Msk        (0x3U << GPIO_MODER_MODER12_Pos)
#define GPIO_MODER_MODER13_Pos        (26U)
#define GPIO_MODER_MODER13_Msk        (0x3U << GPIO_MODER_MODER13_Pos)
#define GPIO_MODER_MODER14_Pos        (28U)
#define GPIO_MODER_MODER14_Msk        (0x3U << GPIO_MODER_MODER14_Pos)
#define GPIO_MODER_MODER15_Pos        (30U)
#define GPIO_MODER_MODER15_Msk        (0x3U << GPIO_MODER_MODER15_Pos)

/********************************  Bit definition for GPIO_OTYPER register  ******************************/
#define GPIO_OTYPER_OT0               (0x1U << 0)
#define GPIO_OTYPER_OT1               (0x1U << 1)
#define GPIO_OTYPER_OT2               (0x1U << 2)
#define GPIO_OTYPER_OT3               (0x1U << 3)
#define GPIO_OTYPER_OT4               (0x1U << 4)
#define GPIO_OTYPER_OT5               (0x1U << 5)
#define GPIO_OTYPER_OT6               (0x1U << 6)
#define GPIO_OTYPER_OT7               (0x1U << 7)
#define GPIO_OTYPER_OT8               (0x1U << 8)
#define GPIO_OTYPER_OT9               (0x1U << 9)
#define GPIO_OTYPER_OT10              (0x1U << 10)
#define GPIO_OTYPER_OT11              (0x1U << 11)
#define GPIO_OTYPER_OT12              (0x1U << 12)
#define GPIO_OTYPER_OT13              (0x1U << 13)
#define GPIO_OTYPER_OT14              (0x1U << 14)
#define GPIO_OTYPER_OT15              (0x1U << 15)

/********************************  Bit definition for GPIO_OSPEEDR register  ******************************/
#define GPIO_OSPEEDER_OSPEEDR0_Pos        (0U)
#define GPIO_OSPEEDER_OSPEEDR0_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR0_Pos)
#define GPIO_OSPEEDER_OSPEEDR1_Pos        (2U)
#define GPIO_OSPEEDER_OSPEEDR1_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR1_Pos)
#define GPIO_OSPEEDER_OSPEEDR2_Pos        (4U)
#define GPIO_OSPEEDER_OSPEEDR2_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR2_Pos)
#define GPIO_OSPEEDER_OSPEEDR3_Pos        (6U)
#define GPIO_OSPEEDER_OSPEEDR3_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR3_Pos)
#define GPIO_OSPEEDER_OSPEEDR4_Pos        (8U)
#define GPIO_OSPEEDER_OSPEEDR4_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR4_Pos)
#define GPIO_OSPEEDER_OSPEEDR5_Pos        (10U)
#define GPIO_OSPEEDER_OSPEEDR5_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR5_Pos)
#define GPIO_OSPEEDER_OSPEEDR6_Pos        (12U)
#define GPIO_OSPEEDER_OSPEEDR6_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR6_Pos)
#define GPIO_OSPEEDER_OSPEEDR7_Pos        (14U)
#define GPIO_OSPEEDER_OSPEEDR7_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR7_Pos)
#define GPIO_OSPEEDER_OSPEEDR8_Pos        (16U)
#define GPIO_OSPEEDER_OSPEEDR8_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR8_Pos)
#define GPIO_OSPEEDER_OSPEEDR9_Pos        (18U)
#define GPIO_OSPEEDER_OSPEEDR9_Msk        (0x3U << GPIO_OSPEEDER_OSPEEDR9_Pos)
#define GPIO_OSPEEDER_OSPEEDR10_Pos       (20U)
#define GPIO_OSPEEDER_OSPEEDR10_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR10_Pos)
#define GPIO_OSPEEDER_OSPEEDR11_Pos       (22U)
#define GPIO_OSPEEDER_OSPEEDR11_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR11_Pos)
#define GPIO_OSPEEDER_OSPEEDR12_Pos       (24U)
#define GPIO_OSPEEDER_OSPEEDR12_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR12_Pos)
#define GPIO_OSPEEDER_OSPEEDR13_Pos       (26U)
#define GPIO_OSPEEDER_OSPEEDR13_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR13_Pos)
#define GPIO_OSPEEDER_OSPEEDR14_Pos       (28U)
#define GPIO_OSPEEDER_OSPEEDR14_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR14_Pos)
#define GPIO_OSPEEDER_OSPEEDR15_Pos       (30U)
#define GPIO_OSPEEDER_OSPEEDR15_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR15_Pos)

/********************************  Bit definition for GPIO_PUPDR register  ******************************/
#define GPIO_PUPDR_PUPDR0_Pos         (0U)
#define GPIO_PUPDR_PUPDR0_Msk         (0x3U << GPIO_PUPDR_PUPDR0_Pos)
#define GPIO_PUPDR_PUPDR1_Pos         (2U)
#define GPIO_PUPDR_PUPDR1_Msk         (0x3U << GPIO_PUPDR_PUPDR1_Pos)
#define GPIO_PUPDR_PUPDR2_Pos         (4U)
#define GPIO_PUPDR_PUPDR2_Msk         (0x3U << GPIO_PUPDR_PUPDR2_Pos)
#define GPIO_PUPDR_PUPDR3_Pos         (6U)
#define GPIO_PUPDR_PUPDR3_Msk         (0x3U << GPIO_PUPDR_PUPDR3_Pos)
#define GPIO_PUPDR_PUPDR4_Pos         (8U)
#define GPIO_PUPDR_PUPDR4_Msk         (0x3U << GPIO_PUPDR_PUPDR4_Pos)
#define GPIO_PUPDR_PUPDR5_Pos         (10U)
#define GPIO_PUPDR_PUPDR5_Msk         (0x3U << GPIO_PUPDR_PUPDR5_Pos)
#define GPIO_PUPDR_PUPDR6_Pos         (12U)
#define GPIO_PUPDR_PUPDR6_Msk         (0x3U << GPIO_PUPDR_PUPDR6_Pos)
#define GPIO_PUPDR_PUPDR7_Pos         (14U)
#define GPIO_PUPDR_PUPDR7_Msk         (0x3U << GPIO_PUPDR_PUPDR7_Pos)
#define GPIO_PUPDR_PUPDR8_Pos         (16U)
#define GPIO_PUPDR_PUPDR8_Msk         (0x3U << GPIO_PUPDR_PUPDR8_Pos)
#define GPIO_PUPDR_PUPDR9_Pos         (18U)
#define GPIO_PUPDR_PUPDR9_Msk         (0x3U << GPIO_PUPDR_PUPDR9_Pos)
#define GPIO_PUPDR_PUPDR10_Pos        (20U)
#define GPIO_PUPDR_PUPDR10_Msk        (0x3U << GPIO_PUPDR_PUPDR10_Pos)
#define GPIO_PUPDR_PUPDR11_Pos        (22U)
#define GPIO_PUPDR_PUPDR11_Msk        (0x3U << GPIO_PUPDR_PUPDR11_Pos)
#define GPIO_PUPDR_PUPDR12_Pos        (24U)
#define GPIO_PUPDR_PUPDR12_Msk        (0x3U << GPIO_PUPDR_PUPDR12_Pos)
#define GPIO_PUPDR_PUPDR13_Pos        (26U)
#define GPIO_PUPDR_PUPDR13_Msk        (0x3U << GPIO_PUPDR_PUPDR13_Pos)
#define GPIO_PUPDR_PUPDR14_Pos        (28U)
#define GPIO_PUPDR_PUPDR14_Msk        (0x3U << GPIO_PUPDR_PUPDR14_Pos)
#define GPIO_PUPDR_PUPDR15_Pos        (30U)
#define GPIO_PUPDR_PUPDR15_Msk        (0x3U << GPIO_PUPDR_PUPDR15_Pos)

/********************************  Bit definition for GPIO_IDR register  ******************************/
#define GPIO_IDR_IDR0                 (0x1U << 0)        /*!< Port input data, bit 0 */
#define GPIO_IDR_IDR1                 (0x1U << 1)        /*!< Port input data, bit 1 */
#define GPIO_IDR_IDR2                 (0x1U << 2)        /*!< Port input data, bit 2 */
#define GPIO_IDR_IDR3                 (0x1U << 3)        /*!< Port input data, bit 3 */
#define GPIO_IDR_IDR4                 (0x1U << 4)        /*!< Port input data, bit 4 */
#define GPIO_IDR_IDR5                 (0x1U << 5)        /*!< Port input data, bit 5 */
#define GPIO_IDR_IDR6                 (0x1U << 6)        /*!< Port input data, bit 6 */
#define GPIO_IDR_IDR7                 (0x1U << 7)        /*!< Port input data, bit 7 */
#define GPIO_IDR_IDR8                 (0x1U << 8)        /*!< Port input data, bit 8 */
#define GPIO_IDR_IDR9                 (0x1U << 9)        /*!< Port input data, bit 9 */
#define GPIO_IDR_IDR10                (0x1U << 10)       /*!< Port input data, bit 10 */
#define GPIO_IDR_IDR11                (0x1U << 11)       /*!< Port input data, bit 11 */
#define GPIO_IDR_IDR12                (0x1U << 12)       /*!< Port input data, bit 12 */
#define GPIO_IDR_IDR13                (0x1U << 13)       /*!< Port input data, bit 13 */
#define GPIO_IDR_IDR14                (0x1U << 14)       /*!< Port input data, bit 14 */
#define GPIO_IDR_IDR15                (0x1U << 15)       /*!< Port input data, bit 15 */

/********************************  Bit definition for GPIO_ODR register  ******************************/
#define GPIO_ODR_ODR0                 (0x1U << 0)        /*!< Port output data, bit 0 */
#define GPIO_ODR_ODR1                 (0x1U << 1)        /*!< Port output data, bit 1 */
#define GPIO_ODR_ODR2                 (0x1U << 2)        /*!< Port output data, bit 2 */
#define GPIO_ODR_ODR3                 (0x1U << 3)        /*!< Port output data, bit 3 */
#define GPIO_ODR_ODR4                 (0x1U << 4)        /*!< Port output data, bit 4 */
#define GPIO_ODR_ODR5                 (0x1U << 5)        /*!< Port output data, bit 5 */
#define GPIO_ODR_ODR6                 (0x1U << 6)        /*!< Port output data, bit 6 */
#define GPIO_ODR_ODR7                 (0x1U << 7)        /*!< Port output data, bit 7 */
#define GPIO_ODR_ODR8                 (0x1U << 8)        /*!< Port output data, bit 8 */
#define GPIO_ODR_ODR9                 (0x1U << 9)        /*!< Port output data, bit 9 */
#define GPIO_ODR_ODR10                (0x1U << 10)       /*!< Port output data, bit 10 */
#define GPIO_ODR_ODR11                (0x1U << 11)       /*!< Port output data, bit 11 */
#define GPIO_ODR_ODR12                (0x1U << 12)       /*!< Port output data, bit 12 */
#define GPIO_ODR_ODR13                (0x1U << 13)       /*!< Port output data, bit 13 */
#define GPIO_ODR_ODR14                (0x1U << 14)       /*!< Port output data, bit 14 */
#define GPIO_ODR_ODR15                (0x1U << 15)       /*!< Port output data, bit 15 */

/********************************  Bit definition for GPIO_BSRR register  ******************************/
#define GPIO_BSRR_BS0                 (0x1U << 0)        /*!< Port x Set bit 0 */
#define GPIO_BSRR_BS1                 (0x1U << 1)        /*!< Port x Set bit 1 */
#define GPIO_BSRR_BS2                 (0x1U << 2)        /*!< Port x Set bit 2 */
#define GPIO_BSRR_BS3                 (0x1U << 3)        /*!< Port x Set bit 3 */
#define GPIO_BSRR_BS4                 (0x1U << 4)        /*!< Port x Set bit 4 */
#define GPIO_BSRR_BS5                 (0x1U << 5)        /*!< Port x Set bit 5 */
#define GPIO_BSRR_BS6                 (0x1U << 6)        /*!< Port x Set bit 6 */
#define GPIO_BSRR_BS7                 (0x1U << 7)        /*!< Port x Set bit 7 */
#define GPIO_BSRR_BS8                 (0x1U << 8)        /*!< Port x Set bit 8 */
#define GPIO_BSRR_BS9                 (0x1U << 9)        /*!< Port x Set bit 9 */
#define GPIO_BSRR_BS10                (0x1U << 10)       /*!< Port x Set bit 10 */
#define GPIO_BSRR_BS11                (0x1U << 11)       /*!< Port x Set bit 11 */
#define GPIO_BSRR_BS12                (0x1U << 12)       /*!< Port x Set bit 12 */
#define GPIO_BSRR_BS13                (0x1U << 13)       /*!< Port x Set bit 13 */
#define GPIO_BSRR_BS14                (0x1U << 14)       /*!< Port x Set bit 14 */
#define GPIO_BSRR_BS15                (0x1U << 15)       /*!< Port x Set bit 15 */

#define GPIO_BSRR_BR0                 (0x1U << 16)       /*!< Port x Reset bit 0 */
#define GPIO_BSRR_BR1                 (0x1U << 17)       /*!< Port x Reset bit 1 */
#define GPIO_BSRR_BR2                 (0x1U << 18)       /*!< Port x Reset bit 2 */
#define GPIO_BSRR_BR3                 (0x1U << 19)       /*!< Port x Reset bit 3 */
#define GPIO_BSRR_BR4                 (0x1U << 20)       /*!< Port x Reset bit 4 */
#define GPIO_BSRR_BR5                 (0x1U << 21)       /*!< Port x Reset bit 5 */
#define GPIO_BSRR_BR6                 (0x1U << 22)       /*!< Port x Reset bit 6 */
#define GPIO_BSRR_BR7                 (0x1U << 23)       /*!< Port x Reset bit 7 */
#define GPIO_BSRR_BR8                 (0x1U << 24)       /*!< Port x Reset bit 8 */
#define GPIO_BSRR_BR9                 (0x1U << 25)       /*!< Port x Reset bit 9 */
#define GPIO_BSRR_BR10                (0x1U << 26)       /*!< Port x Reset bit 10 */
#define GPIO_BSRR_BR11                (0x1U << 27)       /*!< Port x Reset bit 11 */
#define GPIO_BSRR_BR12                (0x1U << 28)       /*!< Port x Reset bit 12 */
#define GPIO_BSRR_BR13                (0x1U << 29)       /*!< Port x Reset bit 13 */
#define GPIO_BSRR_BR14                (0x1U << 30)       /*!< Port x Reset bit 14 */
#define GPIO_BSRR_BR15                (0x1U << 31)       /*!< Port x Reset bit 15 */

/********************************  Bit definition for GPIO_LCKR register  *******************************/
#define GPIO_LCKR_LCK0                (0x1U << 0)        /*!< Port x Lock bit 0 */
#define GPIO_LCKR_LCK1                (0x1U << 1)        /*!< Port x Lock bit 1 */
#define GPIO_LCKR_LCK2                (0x1U << 2)        /*!< Port x Lock bit 2 */
#define GPIO_LCKR_LCK3                (0x1U << 3)        /*!< Port x Lock bit 3 */
#define GPIO_LCKR_LCK4                (0x1U << 4)        /*!< Port x Lock bit 4 */
#define GPIO_LCKR_LCK5                (0x1U << 5)        /*!< Port x Lock bit 5 */
#define GPIO_LCKR_LCK6                (0x1U << 6)        /*!< Port x Lock bit 6 */
#define GPIO_LCKR_LCK7                (0x1U << 7)        /*!< Port x Lock bit 7 */
#define GPIO_LCKR_LCK8                (0x1U << 8)        /*!< Port x Lock bit 8 */
#define GPIO_LCKR_LCK9                (0x1U << 9)        /*!< Port x Lock bit 9 */
#define GPIO_LCKR_LCK10               (0x1U << 10)       /*!< Port x Lock bit 10 */
#define GPIO_LCKR_LCK11               (0x1U << 11)       /*!< Port x Lock bit 11 */
#define GPIO_LCKR_LCK12               (0x1U << 12)       /*!< Port x Lock bit 12 */
#define GPIO_LCKR_LCK13               (0x1U << 13)       /*!< Port x Lock bit 13 */
#define GPIO_LCKR_LCK14               (0x1U << 14)       /*!< Port x Lock bit 14 */
#define GPIO_LCKR_LCK15               (0x1U << 15)       /*!< Port x Lock bit 15 */
#define GPIO_LCKR_LCKK                (0x1U << 16)       /*!< Lock key */

/********************************  Bit definition for GPIO_AFRL register  *******************************/
#define GPIO_AFRL_AFR0_Pos            (0U)
#define GPIO_AFRL_AFR0_Msk            (0xFU << GPIO_AFRL_AFR0_Pos)
#define GPIO_AFRL_AFR1_Pos            (4U)
#define GPIO_AFRL_AFR1_Msk            (0xFU << GPIO_AFRL_AFR1_Pos)
#define GPIO_AFRL_AFR2_Pos            (8U)
#define GPIO_AFRL_AFR2_Msk            (0xFU << GPIO_AFRL_AFR2_Pos)
#define GPIO_AFRL_AFR3_Pos            (12U)
#define GPIO_AFRL_AFR3_Msk            (0xFU << GPIO_AFRL_AFR3_Pos)
#define GPIO_AFRL_AFR4_Pos            (16U)
#define GPIO_AFRL_AFR4_Msk            (0xFU << GPIO_AFRL_AFR4_Pos)
#define GPIO_AFRL_AFR5_Pos            (20U)
#define GPIO_AFRL_AFR5_Msk            (0xFU << GPIO_AFRL_AFR5_Pos)
#define GPIO_AFRL_AFR6_Pos            (24U)
#define GPIO_AFRL_AFR6_Msk            (0xFU << GPIO_AFRL_AFR6_Pos)
#define GPIO_AFRL_AFR7_Pos            (28U)
#define GPIO_AFRL_AFR7_Msk            (0xFU << GPIO_AFRL_AFR7_Pos)

/********************************  Bit definition for GPIO_AFRH register  *******************************/
#define GPIO_AFRH_AFR8_Pos            (0U)
#define GPIO_AFRH_AFR8_Msk            (0xFU << GPIO_AFRH_AFR8_Pos)
#define GPIO_AFRH_AFR9_Pos            (4U)
#define GPIO_AFRH_AFR9_Msk            (0xFU << GPIO_AFRH_AFR9_Pos)
#define GPIO_AFRH_AFR10_Pos           (8U)
#define GPIO_AFRH_AFR10_Msk           (0xFU << GPIO_AFRH_AFR10_Pos)
#define GPIO_AFRH_AFR11_Pos           (12U)
#define GPIO_AFRH_AFR11_Msk           (0xFU << GPIO_AFRH_AFR11_Pos)
#define GPIO_AFRH_AFR12_Pos           (16U)
#define GPIO_AFRH_AFR12_Msk           (0xFU << GPIO_AFRH_AFR12_Pos)
#define GPIO_AFRH_AFR13_Pos           (20U)
#define GPIO_AFRH_AFR13_Msk           (0xFU << GPIO_AFRH_AFR13_Pos)
#define GPIO_AFRH_AFR14_Pos           (24U)
#define GPIO_AFRH_AFR14_Msk           (0xFU << GPIO_AFRH_AFR14_Pos)
#define GPIO_AFRH_AFR15_Pos           (28U)
#define GPIO_AFRH_AFR15_Msk           (0xFU << GPIO_AFRH_AFR15_Pos)

/********************************  Bit definition for GPIO_SMIT register  *******************************/
#define GPIO_SMIT_SMIT0               (0x1U << 0)
#define GPIO_SMIT_SMIT1               (0x1U << 1)
#define GPIO_SMIT_SMIT2               (0x1U << 2)
#define GPIO_SMIT_SMIT3               (0x1U << 3)
#define GPIO_SMIT_SMIT4               (0x1U << 4)
#define GPIO_SMIT_SMIT5               (0x1U << 5)
#define GPIO_SMIT_SMIT6               (0x1U << 6)
#define GPIO_SMIT_SMIT7               (0x1U << 7)
#define GPIO_SMIT_SMIT8               (0x1U << 8)
#define GPIO_SMIT_SMIT9               (0x1U << 9)
#define GPIO_SMIT_SMIT10              (0x1U << 10)
#define GPIO_SMIT_SMIT11              (0x1U << 11)
#define GPIO_SMIT_SMIT12              (0x1U << 12)
#define GPIO_SMIT_SMIT13              (0x1U << 13)
#define GPIO_SMIT_SMIT14              (0x1U << 14)
#define GPIO_SMIT_SMIT15              (0x1U << 15)

/********************************  Bit definition for GPIO_CURRENT register  *******************************/
#define GPIO_CURRENT_CURRENT0_Pos     (0U)
#define GPIO_CURRENT_CURRENT0_Msk     (0x3U << GPIO_CURRENT_CURRENT0_Pos)
#define GPIO_CURRENT_CURRENT1_Pos     (2U)
#define GPIO_CURRENT_CURRENT1_Msk     (0x3U << GPIO_CURRENT_CURRENT1_Pos)
#define GPIO_CURRENT_CURRENT2_Pos     (4U)
#define GPIO_CURRENT_CURRENT2_Msk     (0x3U << GPIO_CURRENT_CURRENT2_Pos)
#define GPIO_CURRENT_CURRENT3_Pos     (6U)
#define GPIO_CURRENT_CURRENT3_Msk     (0x3U << GPIO_CURRENT_CURRENT3_Pos)
#define GPIO_CURRENT_CURRENT4_Pos     (8U)
#define GPIO_CURRENT_CURRENT4_Msk     (0x3U << GPIO_CURRENT_CURRENT4_Pos)
#define GPIO_CURRENT_CURRENT5_Pos     (10U)
#define GPIO_CURRENT_CURRENT5_Msk     (0x3U << GPIO_CURRENT_CURRENT5_Pos)
#define GPIO_CURRENT_CURRENT6_Pos     (12U)
#define GPIO_CURRENT_CURRENT6_Msk     (0x3U << GPIO_CURRENT_CURRENT6_Pos)
#define GPIO_CURRENT_CURRENT7_Pos     (14U)
#define GPIO_CURRENT_CURRENT7_Msk     (0x3U << GPIO_CURRENT_CURRENT7_Pos)
#define GPIO_CURRENT_CURRENT8_Pos     (16U)
#define GPIO_CURRENT_CURRENT8_Msk     (0x3U << GPIO_CURRENT_CURRENT8_Pos)
#define GPIO_CURRENT_CURRENT9_Pos     (18U)
#define GPIO_CURRENT_CURRENT9_Msk     (0x3U << GPIO_CURRENT_CURRENT9_Pos)
#define GPIO_CURRENT_CURRENT10_Pos    (20U)
#define GPIO_CURRENT_CURRENT10_Msk    (0x3U << GPIO_CURRENT_CURRENT10_Pos)
#define GPIO_CURRENT_CURRENT11_Pos    (22U)
#define GPIO_CURRENT_CURRENT11_Msk    (0x3U << GPIO_CURRENT_CURRENT11_Pos)
#define GPIO_CURRENT_CURRENT12_Pos    (24U)
#define GPIO_CURRENT_CURRENT12_Msk    (0x3U << GPIO_CURRENT_CURRENT12_Pos)
#define GPIO_CURRENT_CURRENT13_Pos    (26U)
#define GPIO_CURRENT_CURRENT13_Msk    (0x3U << GPIO_CURRENT_CURRENT13_Pos)
#define GPIO_CURRENT_CURRENT14_Pos    (28U)
#define GPIO_CURRENT_CURRENT14_Msk    (0x3U << GPIO_CURRENT_CURRENT14_Pos)
#define GPIO_CURRENT_CURRENT15_Pos    (30U)
#define GPIO_CURRENT_CURRENT15_Msk    (0x3U << GPIO_CURRENT_CURRENT15_Pos)

/********************************  Bit definition for GPIO_CFGMSK register  *******************************/
#define GPIO_CFGMSK_CFGMSK0           (0x1U << 0)
#define GPIO_CFGMSK_CFGMSK1           (0x1U << 1)
#define GPIO_CFGMSK_CFGMSK2           (0x1U << 2)
#define GPIO_CFGMSK_CFGMSK3           (0x1U << 3)
#define GPIO_CFGMSK_CFGMSK4           (0x1U << 4)
#define GPIO_CFGMSK_CFGMSK5           (0x1U << 5)
#define GPIO_CFGMSK_CFGMSK6           (0x1U << 6)
#define GPIO_CFGMSK_CFGMSK7           (0x1U << 7)
#define GPIO_CFGMSK_CFGMSK8           (0x1U << 8)
#define GPIO_CFGMSK_CFGMSK9           (0x1U << 9)
#define GPIO_CFGMSK_CFGMSK10          (0x1U << 10)
#define GPIO_CFGMSK_CFGMSK11          (0x1U << 11)
#define GPIO_CFGMSK_CFGMSK12          (0x1U << 12)
#define GPIO_CFGMSK_CFGMSK13          (0x1U << 13)
#define GPIO_CFGMSK_CFGMSK14          (0x1U << 14)
#define GPIO_CFGMSK_CFGMSK15          (0x1U << 15)




/*------------------------------------------------------------------------------------------------------*/
/*---                       Universal Asyncronous Receiver / Transmitter (UART)                      ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for UART_IER register  ********************************/
#define UART_IER_RDAIE              (0x1U << 0)          /*!< Received Data Available Interrupt Enable */
#define UART_IER_THREIE             (0x1U << 1)          /*!< Transmit Holding Register Empty Interrupt Enable */
#define UART_IER_RLSIE              (0x1U << 2)          /*!< Receiver Line Status Interrupt Enable */
#define UART_IER_MSIE               (0x1U << 3)          /*!< Modem Status Interrupt Enable */
#define UART_IER_LSRCLRMD           (0x1U << 4)          /*!< Controls the method for clearing the status in the LSR register */
#define UART_IER_PTIME              (0x1U << 7)          /*!< Programmable THRE Interrupt Mode Enable */

/********************************  Bit definition for UART_IIR register  ********************************/
#define UART_IIR_INTID_Msk          (0xFU)               /*!< Interrupt ID bit mask */
#define UART_IIR_INTID_MSI          (0x0U)               /*!< Modem status interrupt */
#define UART_IIR_INTID_NONE         (0x1U)               /*!< No interrupt pending */
#define UART_IIR_INTID_THRE         (0x2U)               /*!< Transmitter holding register empty */
#define UART_IIR_INTID_RDA          (0x4U)               /*!< Received data available interrupt */
#define UART_IIR_INTID_RLS          (0x6U)               /*!< Receiver line status interrupt */
#define UART_IIR_INTID_BUSY         (0x7U)               /*!< Busy detect */
#define UART_IIR_INTID_CTI          (0xCU)               /*!< character timeout indicator */

#define UART_IIR_FIFOSE_Pos         (6U)
#define UART_IIR_FIFOSE_Msk         (0x3U << UART_IIR_FIFOSE_Pos)

/********************************  Bit definition for UART_FCR register  ********************************/
#define UART_FCR_FIFOE              (0x1U << 0)          /*!< FIFO Enable */
#define UART_FCR_RFIFOR             (0x1U << 1)          /*!< RCVR FIFO Reset */
#define UART_FCR_XFIFOR             (0x1U << 2)          /*!< XMIT FIFO Reset */

/* This is used to select the empty threshold level at which the THRE Interrupts are generated when the mode is active. */
#define UART_FCR_TET_0              (0x0U << 4)          /*!< FIFO empty */
#define UART_FCR_TET_2              (0x1U << 4)          /*!< 2 characters in the FIFO */
#define UART_FCR_TET_4              (0x2U << 4)          /*!< FIFO 1/4 full */
#define UART_FCR_TET_8              (0x3U << 4)          /*!< FIFO 1/2 full */

/* This is used to select the trigger level in the receiver FIFO at which the Received Data Available Interrupt is generated. */
#define UART_FCR_RT_1               (0x0U << 6)          /*!< 1 character in the FIFO */
#define UART_FCR_RT_4               (0x1U << 6)          /*!< FIFO 1/4 full */
#define UART_FCR_RT_8               (0x2U << 6)          /*!< FIFO 1/2 full */
#define UART_FCR_RT_14              (0x3U << 6)          /*!< FIFO 2 less than full */


/********************************  Bit definition for UART_LCR register  ********************************/
#define UART_LCR_WLS_Msk            (0x3U << 0)          /*!< Word lenghth field mask bit */
#define UART_LCR_WLS_5BIT           (0x0U << 0)          /*!< Word lenghth is 5 bits */
#define UART_LCR_WLS_6BIT           (0x1U << 0)          /*!< Word lenghth is 6 bits */
#define UART_LCR_WLS_7BIT           (0x2U << 0)          /*!< Word lenghth is 7 bits */
#define UART_LCR_WLS_8BIT           (0x3U << 0)          /*!< Word lenghth is 8 bits */

#define UART_LCR_SBS_Msk            (0x1U << 2)          /*!< Stop bit select field mask bit */
#define UART_LCR_SBS_1BIT           (0x0U << 2)          /*!< 1 stop bit */
#define UART_LCR_SBS_2BIT           (0x1U << 2)          /*!< 2 stop bits (1.5 stop bits when data length is 5 bits) */

// #define UART_LCR_PE                 (0x1U << 3)          /*!< Parity enable */

// #define UART_LCR_PS_Msk             (0x3U << 4)          /*!< Parity select field mask bit */
// #define UART_LCR_PS_ODD             (0x0U << 4)          /*!< Odd parity (Sets the parity bit so that the count of bits set is an odd number) */
// #define UART_LCR_PS_EVEN            (0x1U << 4)          /*!< Even parity (Sets the parity bit so that the count of bits set is an even number) */
// #define UART_LCR_PS_MARK            (0x2U << 4)          /*!< Mark parity (Leaves the parity bit set to 1) */
// #define UART_LCR_PS_SPACE           (0x3U << 4)          /*!< Space parity (Leaves the parity bit set to 0) */

#define UART_LCR_PARITY_Msk         (0x7U << 3)          /*!< Parity field mask bit */
#define UART_LCR_PARITY_NONE        (0x0U << 3)          /*!< No parity */
#define UART_LCR_PARITY_ODD         (0x1U << 3)          /*!< Odd parity (Sets the parity bit so that the count of bits set is an odd number) */
#define UART_LCR_PARITY_EVEN        (0x3U << 3)          /*!< Even parity (Sets the parity bit so that the count of bits set is an even number) */
#define UART_LCR_PARITY_MARK        (0x5U << 3)          /*!< Mark parity (Leaves the parity bit set to 1) */
#define UART_LCR_PARITY_SPACE       (0x7U << 3)          /*!< Space parity (Leaves the parity bit set to 0) */


#define UART_LCR_BC                 (0x1U << 6)          /*!< Break Control Bit */
#define UART_LCR_DLAB               (0x1U << 7)          /*!< Divisor Latch Access Bit */


/********************************  Bit definition for UART_MCR register  ********************************/
#define UART_MCR_RTS                (0x1U << 1)          /*!< Request to Send */
#define UART_MCR_LB                 (0x1U << 4)          /*!< LoopBack Bit */
#define UART_MCR_AFCE               (0x1U << 5)          /*!< Auto Flow Control Enable */
#define UART_MCR_SIRE               (0x1U << 6)          /*!< SIR Mode Enable */

/********************************  Bit definition for UART_LSR register  ********************************/
#define UART_LSR_DR                 (0x1U << 0)          /*!< Data Ready bit */
#define UART_LSR_OE                 (0x1U << 1)          /*!< Overrun error bit */
#define UART_LSR_PE                 (0x1U << 2)          /*!< Parity error bit */
#define UART_LSR_FE                 (0x1U << 3)          /*!< Framing error bit */
#define UART_LSR_BI                 (0x1U << 4)          /*!< Break Interrupt bit */
#define UART_LSR_THRE               (0x1U << 5)          /*!< Transmit Holding Register Empty bit */
#define UART_LSR_TEMT               (0x1U << 6)          /*!< Transmitter Empty bit */
#define UART_LSR_RFE                (0x1U << 7)          /*!< Receiver FIFO Error bit */
#define UART_LSR_ADDR_RCVD          (0x1U << 8)          /*!< Address Received bit */

/********************************  Bit definition for UART_MSR register  ********************************/
#define UART_MSR_DCTS               (0x1U << 0)          /*!< Delta Clear to Send */
#define UART_MSR_CTS                (0x1U << 4)          /*!< Clear to Send */

/********************************  Bit definition for UART_USR register  ********************************/
#define UART_USR_BUSY               (0x1U << 0)          /*!< UART Busy */
#define UART_USR_TFNF               (0x1U << 1)          /*!< Transmit FIFO Not Full */
#define UART_USR_TFE                (0x1U << 2)          /*!< Transmit FIFO Empty */
#define UART_USR_RFNE               (0x1U << 3)          /*!< Receive FIFO Not Empty */
#define UART_USR_RFF                (0x1U << 4)          /*!< Receive FIFO Full */

/********************************  Bit definition for UART_TFL register  ********************************/
/********************************  Bit definition for UART_RFL register  ********************************/

/********************************  Bit definition for UART_SRR register  ********************************/
#define UART_SRR_UR                 (0x1U << 0)          /*!< UART Reset */
#define UART_SRR_RFR                (0x1U << 1)          /*!< RCVR FIFO Reset. This is a shadow register for the RCVR FIFO Reset bit (FCR[1]). */
#define UART_SRR_XFR                (0x1U << 2)          /*!< XMIT FIFO Reset. This is a shadow register for the XMIT FIFO Reset bit (FCR[2]). */

/********************************  Bit definition for UART_SRTS register  ********************************/
#define UART_SRTS_SRTS              (0x1U << 0)          /*!< Shadow Request to Send. This is a shadow register for the RTS bit (MCR[1]). */

/********************************  Bit definition for UART_SBCR register  *******************************/
#define UART_SBCR_SBCB              (0x1U << 0)          /*!< Shadow Break Control Bit. This is a shadow register for the Break bit (LCR[6]). */

/********************************  Bit definition for UART_SDMAM register  ******************************/
#define UART_SDMAM_SDMAM            (0x1U << 0)          /*!< Shadow DMA Mode. This is a shadow register for the DMA mode bit (FCR[3]). */

/********************************  Bit definition for UART_SFE register  ********************************/
#define UART_SFE_SFE                (0x1U << 0)          /*!< Shadow FIFO Enable. This is a shadow register for the FIFO enable bit (FCR[0]). */

/********************************  Bit definition for UART_SRT register  ********************************/
/* Shadow RCVR Trigger. This is a shadow register for the RCVR trigger bits (FCR[7:6]). */
#define UART_SRT_LEV0               (0x0U)               /*!< 1 character in the FIFO */
#define UART_SRT_LEV1               (0x1U)               /*!< FIFO 1/4 full */
#define UART_SRT_LEV2               (0x2U)               /*!< FIFO 1/2 full */
#define UART_SRT_LEV3               (0x3U)               /*!< FIFO 2 less than full */

/********************************  Bit definition for UART_STET register  *******************************/
/* Shadow TX Empty Trigger. This is a shadow register for the TX empty trigger bits (FCR[5:4]). */
#define UART_STET_LEV0              (0x0U)               /*!< FIFO empty */
#define UART_STET_LEV1              (0x1U)               /*!< 2 characters in the FIFO */
#define UART_STET_LEV2              (0x2U)               /*!< FIFO 1/4 full */
#define UART_STET_LEV3              (0x3U)               /*!< FIFO 1/2 full */

/********************************  Bit definition for UART_HTX register  ********************************/
#define UART_HTX_HTX                (0x1U << 0)          /*!< Halt TX */

/********************************  Bit definition for UART_DMASA register  ******************************/
#define UART_DMASA                  (0x1U << 0)          /*!< DMA Software Acknowledge */

/********************************  Bit definition for UART_EXTLCR register  ********************************/
#define UART_EXTLCR_WLS_E           (0x1U << 0)          /*!< This bit is used to enable 9-bit data for transmit and receive transfers */
#define UART_EXTLCR_ADDR_MATCH      (0x1U << 1)          /*!< Address Match Mode */
#define UART_EXTLCR_SEND_ADDR       (0x1U << 2)          /*!< Send address control bit */
#define UART_EXTLCR_TRANSMIT_MODE   (0x1U << 3)          /*!< Transmit mode control bit */




/*------------------------------------------------------------------------------------------------------*/
/*---                                 Cyclic Redundancy Check (CRC)                                  ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for CRC_MODE register  ********************************/
#define CRC_MODE_CRC_POLY_Msk     (0x3U << 0)          /*!< CRC polynomial field mask bit */
#define CRC_MODE_CRC_POLY_CRC8    (0x0U << 0)          /*!< CRC-8 polynomial */
#define CRC_MODE_CRC_POLY_CCITT   (0x1U << 0)          /*!< CRC-CCITT polynomial */
#define CRC_MODE_CRC_POLY_CRC16   (0x2U << 0)          /*!< CRC-CRC16 polynomial */
#define CRC_MODE_CRC_POLY_CRC32   (0x3U << 0)          /*!< CRC-CRC32 polynomial */

#define CRC_MODE_BIT_RVS_WR       (0x1U << 2)          /*!< Select bit order for CRC_WR_DATA */
#define CRC_MODE_CMPL_WR          (0x1U << 3)          /*!< Select one's complement for CRC_WR_DATA */
#define CRC_MODE_BIT_RVS_SUM      (0x1U << 4)          /*!< Select bit order revers for CRC_SUM */
#define CRC_MODE_CMPL_SUM         (0x1U << 5)          /*!< Select one's complement for CRC_SUM */
#define CRC_MODE_SEED_OP          (0x1U << 6)          /*!< CRC seed option set */
#define CRC_MODE_SEED_SET         (0x1U << 7)          /*!< Load seed to CRC generator */



/*------------------------------------------------------------------------------------------------------*/
/*---                                 Special Function Macro (SFM)                                   ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for SFM_CTRL register  ********************************/
#define SFM_CTRL_EXP_RATE_Msk     (0x7U << 0)          /*!< Bit Expand Rate field mask bit */
#define SFM_CTRL_EXP_RATE_1       (0x0U << 0)          /*!< Bit Expand Rate 1 */
#define SFM_CTRL_EXP_RATE_2       (0x1U << 0)          /*!< Bit Expand Rate 2 */
#define SFM_CTRL_EXP_RATE_3       (0x2U << 0)          /*!< Bit Expand Rate 3 */
#define SFM_CTRL_EXP_RATE_4       (0x3U << 0)          /*!< Bit Expand Rate 4 */
#define SFM_CTRL_EXP_RATE_5       (0x4U << 0)          /*!< Bit Expand Rate 5 */
#define SFM_CTRL_EXP_RATE_6       (0x5U << 0)          /*!< Bit Expand Rate 6 */
#define SFM_CTRL_EXP_RATE_7       (0x6U << 0)          /*!< Bit Expand Rate 7 */
#define SFM_CTRL_EXP_RATE_8       (0x7U << 0)          /*!< Bit Expand Rate 8 */

#define SFM_CTRL_EXP_EN           (0x1U << 3)          /*!< Bit Expand Function Enable Bit */

/********************************  Bit definition for SFM_USBPSDCSR register  ********************************/
#define SFM_USBPSDCSR_SE0F        (0x1U << 0)          /*!< No description */
#define SFM_USBPSDCSR_JSTATF      (0x1U << 1)          /*!< No description */
#define SFM_USBPSDCSR_KSTATF      (0x1U << 2)          /*!< No description */
#define SFM_USBPSDCSR_SE1F        (0x1U << 3)          /*!< No description */

#define SFM_USBPSDCSR_SE0EN       (0x1U << 8)          /*!< No description */
#define SFM_USBPSDCSR_JSTATEN     (0x1U << 9)          /*!< No description */
#define SFM_USBPSDCSR_KSTATEN     (0x1U << 10)         /*!< No description */
#define SFM_USBPSDCSR_SE1EN       (0x1U << 11)         /*!< No description */



/*------------------------------------------------------------------------------------------------------*/
/*---                             Direct Memory Access Controller (DMAC)                             ---*/
/*------------------------------------------------------------------------------------------------------*/
/*******************************  Bit definition for DMAC_CTLLx register  *******************************/
#define DMAC_CTLL_INT_EN               (0x1U << 0)               /*!< Interrupt Enable Bit */

#define DMAC_CTLL_DST_TR_WIDTH_Msk     (0x7U << 1)               /*!< Destination Transfer Width field mask bit */
#define DMAC_CTLL_DST_TR_WIDTH_8       (0x0U << 1)               /*!< Destination Transfer Width is 8 bits */
#define DMAC_CTLL_DST_TR_WIDTH_16      (0x1U << 1)               /*!< Destination Transfer Width is 16 bits */
#define DMAC_CTLL_DST_TR_WIDTH_32      (0x2U << 1)               /*!< Destination Transfer Width is 32 bits */

#define DMAC_CTLL_SRC_TR_WIDTH_Msk     (0x7U << 4)               /*!< Source Transfer Width field mask bit*/
#define DMAC_CTLL_SRC_TR_WIDTH_8       (0x0U << 4)               /*!< Source Transfer Width is 8 bits */
#define DMAC_CTLL_SRC_TR_WIDTH_16      (0x1U << 4)               /*!< Source Transfer Width is 16 bits */
#define DMAC_CTLL_SRC_TR_WIDTH_32      (0x2U << 4)               /*!< Source Transfer Width is 32 bits */

#define DMAC_CTLL_DINC_Msk             (0x3U << 7)               /*!< Destination Address Increment field mask bit */
#define DMAC_CTLL_DINC_INC             (0x0U << 7)               /*!< Destination Address Increment is increment */
#define DMAC_CTLL_DINC_DEC             (0x1U << 7)               /*!< Destination Address Increment is decrement */
#define DMAC_CTLL_DINC_NO              (0x2U << 7)               /*!< Destination Address Increment is no change */

#define DMAC_CTLL_SINC_Msk             (0x3U << 9)               /*!< Source Address Increment field mask bit */
#define DMAC_CTLL_SINC_INC             (0x0U << 9)               /*!< Source Address Increment is increment */
#define DMAC_CTLL_SINC_DEC             (0x1U << 9)               /*!< Source Address Increment is decrement */
#define DMAC_CTLL_SINC_NO              (0x2U << 9)               /*!< Source Address Increment is no change */

#define DMAC_CTLL_DEST_MSIZE_Msk       (0x7U << 11)              /*!< Destination Burst Transaction Length field mask bit */
#define DMAC_CTLL_DEST_MSIZE_1         (0x0U << 11)              /*!< Destination Burst Transaction Length */
#define DMAC_CTLL_DEST_MSIZE_4         (0x1U << 11)              /*!< Destination Burst Transaction Length */
#define DMAC_CTLL_DEST_MSIZE_8         (0x2U << 11)              /*!< Destination Burst Transaction Length */
#define DMAC_CTLL_DEST_MSIZE_16        (0x3U << 11)              /*!< Destination Burst Transaction Length */
#define DMAC_CTLL_DEST_MSIZE_32        (0x4U << 11)              /*!< Destination Burst Transaction Length */
#define DMAC_CTLL_DEST_MSIZE_64        (0x5U << 11)              /*!< Destination Burst Transaction Length */
#define DMAC_CTLL_DEST_MSIZE_128       (0x6U << 11)              /*!< Destination Burst Transaction Length */
#define DMAC_CTLL_DEST_MSIZE_256       (0x7U << 11)              /*!< Destination Burst Transaction Length */

#define DMAC_CTLL_SRC_MSIZE_Msk        (0x7U << 14)              /*!< Source Burst Transaction Length field mask bit */
#define DMAC_CTLL_SRC_MSIZE_1          (0x0U << 14)              /*!< Source Burst Transaction Length */
#define DMAC_CTLL_SRC_MSIZE_4          (0x1U << 14)              /*!< Source Burst Transaction Length */
#define DMAC_CTLL_SRC_MSIZE_8          (0x2U << 14)              /*!< Source Burst Transaction Length */
#define DMAC_CTLL_SRC_MSIZE_16         (0x3U << 14)              /*!< Source Burst Transaction Length */
#define DMAC_CTLL_SRC_MSIZE_32         (0x4U << 14)              /*!< Source Burst Transaction Length */
#define DMAC_CTLL_SRC_MSIZE_64         (0x5U << 14)              /*!< Source Burst Transaction Length */
#define DMAC_CTLL_SRC_MSIZE_128        (0x6U << 14)              /*!< Source Burst Transaction Length */
#define DMAC_CTLL_SRC_MSIZE_256        (0x7U << 14)              /*!< Source Burst Transaction Length */

#define DMAC_CTLL_SRC_GATHER_EN        (0x1U << 17)              /*!< Source gather enable bit */
#define DMAC_CTLL_DST_SCATTER_EN       (0x1U << 18)              /*!< Destination scatter enable bit */

#define DMAC_CTLL_TT_FC_Msk              (0x7U << 20)              /*!< Transfer Type and Flow Control field mask bit */
#define DMAC_CTLL_TT_FC_M2M_DMAC         (0x0U << 20)              /*!< Transfer Type is Memory to Memory, Flow Control is DMAC */
#define DMAC_CTLL_TT_FC_M2P_DMAC         (0x1U << 20)              /*!< Transfer Type is Memory to Peripheral, Flow Control is DMAC */
#define DMAC_CTLL_TT_FC_P2M_DMAC         (0x2U << 20)              /*!< Transfer Type is Peripheral to Memory, Flow Control is DMAC */
#define DMAC_CTLL_TT_FC_P2P_DMAC         (0x3U << 20)              /*!< Transfer Type is Peripheral to Peripheral, Flow Control is DMAC */
#define DMAC_CTLL_TT_FC_P2M_PERIPH       (0x4U << 20)              /*!< Transfer Type is Peripheral to Memory, Flow Control is Peripheral */
#define DMAC_CTLL_TT_FC_P2P_SRC_PERIPH   (0x5U << 20)              /*!< Transfer Type is Peripheral to Peripheral, Flow Control is Source Peripheral */
#define DMAC_CTLL_TT_FC_M2P_PERIPH       (0x6U << 20)              /*!< Transfer Type is Memory to Peripheral, Flow Control is Peripheral */
#define DMAC_CTLL_TT_FC_P2P_DST_PERIPH   (0x7U << 20)              /*!< Transfer Type is Peripheral to Peripheral, Flow Control is Destination Peripheral */

#define DMAC_CTLL_LLP_DST_EN           (0x1U << 27)              /*!< Source Burst Transaction Length */
#define DMAC_CTLL_LLP_SRC_EN           (0x1U << 28)              /*!< Block chaining is enabled on the source side */

/*******************************  Bit definition for DMAC_CTLHx register  *******************************/
#define DMAC_CTLH_BLOCK_TS_Msk         (0xFFFU)                  /*!< Block Transfer Size field mask bit */
#define DMAC_CTLH_DONE                 (0x1U << 12)              /*!< Done bit */




/*******************************  Bit definition for DMAC_CFGLx register  *******************************/
#define DMAC_CFGL_CH_PRIOR_Msk        (0x7U << 5)                /*!< Channel priority field mask */
#define DMAC_CFGL_CH_SUSP             (0x1U << 8)                /*!< Channel Suspend */
#define DMAC_CFGL_FIFO_EMPTY          (0x1U << 9)                /*!< Indicates if there is data left in the channel FIFO */
#define DMAC_CFGL_HS_SEL_DST          (0x1U << 10)               /*!< Destination Software or Hardware Handshaking Select */
#define DMAC_CFGL_HS_SEL_SRC          (0x1U << 11)               /*!< Source Software or Hardware Handshaking Select */

#define DMAC_CFGL_DST_HS_POL          (0x1U << 18)               /*!< Destination Handshaking Interface Polarity */
#define DMAC_CFGL_SRC_HS_POL          (0x1U << 19)               /*!< Source Handshaking Interface Polarity */

#define DMAC_CFGL_RELOAD_SRC          (0x1U << 30)               /*!< Automatic Source Reload */
#define DMAC_CFGL_RELOAD_DST          (0x1U << 31)               /*!< Automatic Destination Reload */

/*******************************  Bit definition for DMAC_CFGHx register  *******************************/

#define DMAC_CFGH_FIFO_MODE           (0x1U << 1)                /*!< FIFO Mode Select */
#define DMAC_CFGH_DS_UPD_EN           (0x1U << 5)                /*!< Destination Status Update Enable */
#define DMAC_CFGH_SS_UPD_EN           (0x1U << 6)                /*!< Source Status Update Enable */

#define DMAC_CFGH_SRC_PER_Msk         (0xFU << 7)                /*!< No description */
#define DMAC_CFGH_DEST_PER_Msk        (0xFU << 11)               /*!< No description */



/*------------------------------------------------------------------------------------------------------*/
/*---                                  Inter-Integrated Circuit (I2C)                               ---*/
/*------------------------------------------------------------------------------------------------------*/
/******************************    Bit definition for I2C_CON register *******************************/
#define I2C_CON_MASTER_MODE                   (0x1U << 0)                /*!< This bit controls whether the i2c master is enable */

#define I2C_CON_SPEED_Msk                     (0x3U << 1)                /*!< SPEED field mask bit */
#define I2C_CON_SPEED_STANDARD                (0x1U << 1)                /*!< standard mode (0 to 100 Kb/s) */
#define I2C_CON_SPEED_FAST                    (0x2U << 1)                /*!< fast mode (≤ 400 Kb/s) or fast mode plus (≤ 1000 Kb/s) */
#define I2C_CON_SPEED_HIGH                    (0x3U << 1)                /*!< high speed mode (≤ 3.4 Mb/s) */

#define I2C_CON_10BITADDR_SLAVE               (0x1U << 3)                /*!< When acting as a slave, this bit controls i2c responds to 10-bit addresses. */
#define I2C_CON_10BITADDR_MASTER              (0x1U << 4)                /*!< XXX */
#define I2C_CON_RESTART_EN                    (0x1U << 5)                /*!< Determines whether RESTART conditions may be sent when acting as a master. */
#define I2C_CON_SLAVE_DISABLE                 (0x1U << 6)                /*!< This bit controls whether I2C has its slave disabled */
#define I2C_CON_STOP_DET_IFADDRESSED          (0x1U << 7)                /*!< XXX */
#define I2C_CON_TX_EMPTY_CTRL                 (0x1U << 8)                /*!< This bit controls the generation of the TX_EMPTY interrupt */
#define I2C_CON_RX_FIFO_FULL_HLD_CTRL         (0x1U << 9)                /*!< This bit controls whether i2c should hold the bus when the Rx FIFO is physically full to its RX_BUFFER_DEPTH */
#define I2C_CON_STOP_DET_IF_MASTER_ACTIVE     (0x1U << 10)               /*!< XXX */
#define I2C_CON_BUS_CLEAR_FEATURE_CTRL        (0x1U << 11)               /*!< XXX */
#define I2C_CON_OPTIONAL_SAR_CTRL             (0x1U << 16)               /*!< Enables the usage of OPTIONAL_SAR register */
#define I2C_CON_SMBUS_SLAVE_QUICK_EN          (0x1U << 17)               /*!< XXX */
#define I2C_CON_SMBUS_ARP_EN                  (0x1U << 18)               /*!< This bit controls whether i2c should enable Address Resolution Logic in SMBus Mode  */
#define I2C_CON_SMBUS_PERSISTANT_SLV_ADDR_EN  (0x1U << 19)               /*!< This bit controls to enable i2c slave as persistent or non-persistent slave */

/******************************    Bit definition for I2C_TAR register   *******************************/
#define I2C_TAR_TAR_Msk                       (0x3FFU)                   /*!< target address field mask bit */
#define I2C_TAR_GC_OR_START                   (0x1U << 10)               /*!< this bit indicates whether a General Call or START byte command is to be performed by the i2c. */
#define I2C_TAR_SPECIAL                       (0x1U << 11)               /*!< This bit indicates whether software performs a Device-ID, General Call or START BYTE command. */
#define I2C_TAR_10BITADDR_MASTER              (0x1U << 12)               /*!< This bit controls whether the i2c starts its transfers in 7-or 10-bit addressing mode when acting as a master. */
#define I2C_TAR_DEVICE_ID                     (0x1U << 13)               /*!< If bit 11 (SPECIAL) is set to 1, then this bit indicates whether a Device-ID of a particular slave mentioned in TAR[6:0] is to be performed by the i2c Master. */
#define I2C_TAR_SMBUS_QUICK_CMD               (0x1U << 16)               /*!< If bit 11 (SPECIAL) is set to 1, then this bit indicates whether a Quick command is to be performed by the i2c. */

/******************************    Bit definition for I2C_SAR register   *******************************/
/******************************    Bit definition for I2C_HS_MADDR register   *******************************/

/******************************    Bit definition for I2C_DATA_CMD register   *******************************/
#define I2C_DATA_CMD_DAT_Msk                  (0xFFU)                    /*!< DAT field mask bit */

#define I2C_DATA_CMD_READ                     (0x1U << 8)                /*!< This bit controls whether a read or a write is performed */
#define I2C_DATA_CMD_STOP                     (0x1U << 9)                /*!< This bit controls whether a STOP is issued after the byte is sent or received */
#define I2C_DATA_CMD_RESTART                  (0x1U << 10)               /*!< This bit controls whether a RESTART is issued before the byte is sent or received */
#define I2C_DATA_CMD_FIRST_DATA_BYTE          (0x1U << 11)               /*!< Indicates the first data byte received after the address phase for receive transfer in Master receiver or Slave receiver mode */

/******************************    Bit definition for I2C_SS_SCL_HCNT register   *******************************/
/******************************    Bit definition for I2C_SS_SCL_LCNT register   *******************************/
/******************************    Bit definition for I2C_FS_SCL_HCNT register   *******************************/
/******************************    Bit definition for I2C_FS_SCL_LCNT register   *******************************/
/******************************    Bit definition for I2C_HS_SCL_HCNT register   *******************************/
/******************************    Bit definition for I2C_HS_SCL_LCNT register   *******************************/

/******************************    Bit definition for I2C_INTR_STAT register   *******************************/
/******************************    Bit definition for I2C_INTR_MASK register   *******************************/
/******************************    Bit definition for I2C_RAW_INTR_STAT register   *******************************/
#define I2C_INTR_RX_UNDER                     (0x1U << 0)                /*!< Set if the processor attempts to read the receive buffer when it is empty by reading from the DATA_CMD register */
#define I2C_INTR_RX_OVER                      (0x1U << 1)                /*!< Set if the receive buffer is completely filled to RX_BUFFER_DEPTH and an additional byte is received from an external I2C device */
#define I2C_INTR_RX_FULL                      (0x1U << 2)                /*!< Set when the receive buffer reaches or goes above the RX_TL threshold in the RX_TL register */
#define I2C_INTR_TX_OVER                      (0x1U << 3)                /*!< Set during transmit if the transmit buffer is filled to TX_BUFFER_DEPTH and the processor attempts to issue another I2C command by writing to the DATA_CMD register */
#define I2C_INTR_TX_EMPTY                     (0x1U << 4)                /*!< The behavior of the TX_EMPTY interrupt status differs based on the TX_EMPTY_CTRL selection in the CON register */
#define I2C_INTR_RD_REQ                       (0x1U << 5)                /*!< This bit is set to 1 when DW_apb_i2c is acting as a slave and another I2C master is attempting to read data from DW_apb_i2c */
#define I2C_INTR_TX_ABRT                      (0x1U << 6)                /*!< This bit indicates if DW_apb_i2c, as an I2C transmitter, is unable to complete the intended actions on the contents of the transmit FIFO */
#define I2C_INTR_RX_DONE                      (0x1U << 7)                /*!< When the DW_apb_i2c is acting as a slave-transmitter, this bit is set to 1 if the master does not acknowledge a transmitted byte */
#define I2C_INTR_ACTIVITY                     (0x1U << 8)                /*!< This bit captures DW_apb_i2c activity and stays set until it is cleared */
#define I2C_INTR_STOP_DET                     (0x1U << 9)                /*!< Indicates whether a STOP condition has occurred on the I2C interface regardless of whether DW_apb_i2c is operating in slave or master mode */
#define I2C_INTR_START_DET                    (0x1U << 10)               /*!< Indicates whether a STOP condition has occurred on the I2C interface regardless of whether DW_apb_i2c is operating in slave or master mode */
#define I2C_INTR_GEN_CALL                     (0x1U << 11)               /*!< Indicates whether a START or RESTART condition has occurred on the I2C interface regardless of whether DW_apb_i2c is operating in slave or master mode */
#define I2C_INTR_RESTART_DET                  (0x1U << 12)               /*!< Indicates whether a RESTART condition has occurred on the I2C interface when DW_apb_i2c is operating in slave mode and the slave is the addressed slave */
#define I2C_INTR_SCL_STUCK_AT_LOW             (0x1U << 14)               /*!< Indicates whether the SCL Line is stuck at low for the SCL_STUCK_LOW_TIMOUT number of clk periods */


/******************************    Bit definition for I2C_RX_TL register   *******************************/
/******************************    Bit definition for I2C_TX_TL register   *******************************/
/******************************    Bit definition for I2C_CLR_INTR register   *******************************/
/******************************    Bit definition for I2C_CLR_RX_UNDERT register   *******************************/
/******************************    Bit definition for I2C_CLR_RX_OVER register   *******************************/
/******************************    Bit definition for I2C_CLR_TX_OVER register   *******************************/
/******************************    Bit definition for I2C_CLR_RD_REQ register   *******************************/
/******************************    Bit definition for I2C_CLR_TX_ABRT register   *******************************/
/******************************    Bit definition for I2C_CLR_RX_DONE register   *******************************/
/******************************    Bit definition for I2C_CLR_ACTIVITY register   *******************************/
/******************************    Bit definition for I2C_CLR_STOP_DET register   *******************************/
/******************************    Bit definition for I2C_CLR_START_DET register   *******************************/
/******************************    Bit definition for I2C_CLR_GEN_CALL register   *******************************/

/******************************    Bit definition for I2C_ENABLE register   *******************************/
#define I2C_ENABLE_ENABLE                            (0x1U << 0)         /*!< Controls whether the DW_apb_i2c is enabled */
#define I2C_ENABLE_ABORT                             (0x1U << 1)         /*!< XXX */
#define I2C_ENABLE_TX_CMD_BLOCK                      (0x1U << 2)         /*!< XXX */
#define I2C_ENABLE_SDA_STUCK_RECOVERY_ENA            (0x1U << 3)         /*!< XXX */
#define I2C_ENABLE_SMBUS_CLK_RESET                   (0x1U << 16)        /*!< This bit is used in SMBus Host mode to initiate the SMBus Master Clock Reset */
#define I2C_ENABLE_SMBUS_SUSPEND_EN                  (0x1U << 17)        /*!< The SMBUS_SUSPEND_EN register bit is used to control assertion and de-assertion of SMBSUS signal */
#define I2C_ENABLE_SMBUS_ALERT_EN                    (0x1U << 18)        /*!< The SMBUS_ALERT_CTRL register bit is used to control assertion of SMBALERT signal */

/******************************    Bit definition for I2C_STATUS register   *******************************/
#define I2C_STATUS_ACTIVITY                          (0x1U << 0)         /*!< I2C Activity Status */
#define I2C_STATUS_TFNF                              (0x1U << 1)         /*!< Transmit FIFO Not Full */
#define I2C_STATUS_TFE                               (0x1U << 2)         /*!< Transmit FIFO Completely Empty */
#define I2C_STATUS_RFNE                              (0x1U << 3)         /*!< Receive FIFO Not Empty */
#define I2C_STATUS_RFF                               (0x1U << 4)         /*!< Receive FIFO Completely Full */
#define I2C_STATUS_MST_ACTIVITY                      (0x1U << 5)         /*!< Master FSM Activity Status */
#define I2C_STATUS_SLV_ACTIVITY                      (0x1U << 6)         /*!< Slave FSM Activity Status */
#define I2C_STATUS_MST_HOLD_TX_FIFO_EMPTY            (0x1U << 7)         /*!< XXX */
#define I2C_STATUS_MST_HOLD_RX_FIFO_FULL             (0x1U << 8)         /*!< XXX */
#define I2C_STATUS_SLV_HOLD_TX_FIFO_EMPTY            (0x1U << 9)         /*!< XXX */
#define I2C_STATUS_SLV_HOLD_RX_FIFO_FULL             (0x1U << 10)        /*!< XXX */
#define I2C_STATUS_SDA_STUCK_NOT_RECOVERED           (0x1U << 11)        /*!< XXX */
#define I2C_STATUS_SMBUS_QUICK_CMD_BIT               (0x1U << 16)        /*!< XXX */
#define I2C_STATUS_SMBUS_SLAVE_ADDR_VALID            (0x1U << 17)        /*!< This bit indicates whether the SMBus Slave address (sar[6:0]) is valid or not */
#define I2C_STATUS_SMBUS_SLAVE_ADDR_RESOLVED         (0x1U << 18)        /*!< This bit indicates whether the SMBus Slave address (sar[6:0]) is Resolved by ARP Master */
#define I2C_STATUS_SMBUS_SUSPEND_STATUS              (0x1U << 19)        /*!< This bit indicates whether the status of the input signal is smbus_sus_in_n */
#define I2C_STATUS_SMBUS_ALERT_STATUS                (0x1U << 20)        /*!< This bit indicates whether the status of the input signal is smbus_alert_in_n */

/******************************    Bit definition for I2C_TXFLR register   *******************************/
/******************************    Bit definition for I2C_RXFLR register   *******************************/
/******************************    Bit definition for I2C_SDA_HOLD register   *******************************/

/******************************    Bit definition for I2C_TX_ABRT_SOURCE   *******************************/
#define I2C_TX_ABRT_SOURCE_7B_ADDR_NOACK             (0x1U << 0)         /*!< Master is in 7-bit addressing mode and the address sent was not acknowledged by any slave */
#define I2C_TX_ABRT_SOURCE_10ADDR1_NOACK             (0x1U << 1)         /*!< Master is in 10-bit address mode and the first 10-bit address byte was not acknowledged by any slave. */
#define I2C_TX_ABRT_SOURCE_10ADDR2_NOACK             (0x1U << 2)         /*!< Master is in 10-bit address mode and the second address byte of the 10-bit address was not acknowledged by any slave */
#define I2C_TX_ABRT_SOURCE_TXDATA_NOACK              (0x1U << 3)         /*!< XXX */
#define I2C_TX_ABRT_SOURCE_GCALL_NOACK               (0x1U << 4)         /*!< XXX */
#define I2C_TX_ABRT_SOURCE_GCALL_READ                (0x1U << 5)         /*!< XXX */
#define I2C_TX_ABRT_SOURCE_HS_ACKDET                 (0x1U << 6)         /*!< XXX */
#define I2C_TX_ABRT_SOURCE_SBYTE_ACKDET              (0x1U << 7)         /*!< XXX */
#define I2C_TX_ABRT_SOURCE_HS_NORSTRT                (0x1U << 8)         /*!< XXX */
#define I2C_TX_ABRT_SOURCE_SBYTE_NORSTRT             (0x1U << 9)         /*!< XXX */
#define I2C_TX_ABRT_SOURCE_10B_RD_NORSTRT            (0x1U << 10)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_MASTER_DIS                (0x1U << 11)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_LOST                      (0x1U << 12)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_SLVFLUSH_TXFIFO           (0x1U << 13)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_SLV_ARBLOST               (0x1U << 14)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_SLVRD_INTX                (0x1U << 15)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_USER_ABRT                 (0x1U << 16)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_SDA_STUCK_AT_LOW          (0x1U << 17)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_DEVICE_NOACK              (0x1U << 18)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_DEVICE_SLVADDR_NOACK      (0x1U << 19)        /*!< XXX */
#define I2C_TX_ABRT_SOURCE_DEVICE_WRITE              (0x1U << 20)        /*!< XXX */

#define I2C_TX_ABRT_SOURCE_TX_FLUSH_CNT_Msk          (0xFF800000U)       /*!< XXX */

/******************************    Bit definition for I2C_SLV_DATA_NACK_ONLY register   *******************************/
#define I2C_SLV_DATA_NACK_ONLY_NACK                  (0x1U << 0)         /*!< XXX */

/******************************    Bit definition for I2C_DMA_CR register   *******************************/
#define I2C_DMA_CR_RDMAE                             (0x1U << 0)         /*!< XXX */
#define I2C_DMA_CR_TDMAE                             (0x1U << 1)         /*!< XXX */

/******************************    Bit definition for I2C_DMA_TDLR register   *******************************/
/******************************    Bit definition for I2C_DMA_RDLR register   *******************************/
/******************************    Bit definition for I2C_SDA_SETUP register   *******************************/
/******************************    Bit definition for I2C_ACK_GENERAL_CALL register   *******************************/

/******************************    Bit definition for I2C_ENABLE_STATUS register   *******************************/
#define I2C_ENABLE_STATUS_IC_EN                      (0x1U << 0)         /*!< XXX */
#define I2C_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY    (0x1U << 1)         /*!< XXX */
#define I2C_ENABLE_STATUS_SLV_RX_DATA_LOST           (0x1U << 2)         /*!< XXX */

/******************************    Bit definition for FS_SPKLEN register   *******************************/
/******************************    Bit definition for HS_SPKLEN register   *******************************/
/******************************    Bit definition for CLR_RESTART_DET register   *******************************/
/******************************    Bit definition for SCL_STUCK_AT_LOW_TIMEOUT register   *******************************/
/******************************    Bit definition for SDA_STUCK_AT_LOW_TIMEOUT register   *******************************/
/******************************    Bit definition for CLR_SCL_STUCK_DET register   *******************************/
/******************************    Bit definition for SMBUS_CLK_LOW_SEXT register   *******************************/
/******************************    Bit definition for SMBUS_CLK_LOW_MEXT register   *******************************/
/******************************    Bit definition for SMBUS_THIGH_MAX_IDLE_COUNT register   *******************************/
/******************************    Bit definition for SMBUS_INTR_STAT register   *******************************/
/******************************    Bit definition for SMBUS_INTR_MASK register   *******************************/
/******************************    Bit definition for SMBUS_RAW_INTR_STAT register   *******************************/
#define I2C_SMBUS_INTR_SLV_CLOCK_EXTND_TIMEOUT       (0x1U << 0)         /*!< XXX */
#define I2C_SMBUS_INTR_MST_CLOCK_EXTND_TIMEOUT       (0x1U << 1)         /*!< XXX */
#define I2C_SMBUS_INTR_QUICK_CMD_DET                 (0x1U << 2)         /*!< XXX */
#define I2C_SMBUS_INTR_HOST_NTFY_MST_DET             (0x1U << 3)         /*!< XXX */
#define I2C_SMBUS_INTR_ARP_PREPARE_CMD_DET           (0x1U << 4)         /*!< XXX */
#define I2C_SMBUS_INTR_ARP_RST_CMD_DET               (0x1U << 5)         /*!< XXX */
#define I2C_SMBUS_INTR_ARP_GET_UDID_CMD_DET          (0x1U << 6)         /*!< XXX */
#define I2C_SMBUS_INTR_ARP_ASSGN_ADDR_CMD_DET        (0x1U << 7)         /*!< XXX */
#define I2C_SMBUS_INTR_SLV_RX_PEC_NACK               (0x1U << 8)         /*!< XXX */
#define I2C_SMBUS_INTR_SMBUS_SUSPEND_DET             (0x1U << 9)         /*!< XXX */
#define I2C_SMBUS_INTR_SMBUS_ALERT_DET               (0x1U << 10)        /*!< XXX */

/******************************    Bit definition for CLR_SMBUS_INTR register   *******************************/
/******************************    Bit definition for OPTIONAL_SAR register   *******************************/
/******************************    Bit definition for SMBUS_UDID_LSB register   *******************************/





/*------------------------------------------------------------------------------------------------------*/
/*---                                Serial Peripheral Interface (SPI)                               ---*/
/*------------------------------------------------------------------------------------------------------*/
/******************************    Bit definition for SPI_CR0 register *******************************/
#define SPI_CR0_DFS_Msk               (0xFU << 0)         /*!< Data Frame Size field mask */
#define SPI_CR0_DFS_4BITS             (0x3U << 0)         /*!< 4-bit serial data transfer */
#define SPI_CR0_DFS_5BITS             (0x4U << 0)         /*!< 5-bit serial data transfer */
#define SPI_CR0_DFS_6BITS             (0x5U << 0)         /*!< 6-bit serial data transfer */
#define SPI_CR0_DFS_7BITS             (0x6U << 0)         /*!< 7-bit serial data transfer */
#define SPI_CR0_DFS_8BITS             (0x7U << 0)         /*!< 8-bit serial data transfer */
#define SPI_CR0_DFS_9BITS             (0x8U << 0)         /*!< 9-bit serial data transfer */
#define SPI_CR0_DFS_10BITS            (0x9U << 0)         /*!< 10-bit serial data transfer */
#define SPI_CR0_DFS_11BITS            (0xAU << 0)         /*!< 11-bit serial data transfer */
#define SPI_CR0_DFS_12BITS            (0xBU << 0)         /*!< 12-bit serial data transfer */
#define SPI_CR0_DFS_13BITS            (0xCU << 0)         /*!< 13-bit serial data transfer */
#define SPI_CR0_DFS_14BITS            (0xDU << 0)         /*!< 14-bit serial data transfer */
#define SPI_CR0_DFS_15BITS            (0xEU << 0)         /*!< 15-bit serial data transfer */
#define SPI_CR0_DFS_16BITS            (0xFU << 0)         /*!< 16-bit serial data transfer */

#define SPI_CR0_FRF_Msk               (0x3U << 4)         /*!< Frame Format field mask */
#define SPI_CR0_FRF_SPI               (0x0U << 4)         /*!< Motorolla SPI Frame Format */
#define SPI_CR0_FRF_SSP               (0x1U << 4)         /*!< Texas Instruments SSP Frame Format */
#define SPI_CR0_FRF_NS                (0x2U << 4)         /*!< National Microwire Frame Format */

#define SPI_CR0_CPHA                  (0x1U << 6)         /*!< Serial Clock Phase */
#define SPI_CR0_CPOL                  (0x1U << 7)         /*!< Serial Clock Polarity */

#define SPI_CR0_TMOD_Msk              (0x3U << 8)         /*!< Transfer Mode field mask */
#define SPI_CR0_TMOD_TX_AND_RX        (0x0U << 8)         /*!< Transfer & receive */
#define SPI_CR0_TMOD_TX_ONLY          (0x1U << 8)         /*!< Transmit only mode */
#define SPI_CR0_TMOD_RX_ONLY          (0x2U << 8)         /*!< Receive only mode */
#define SPI_CR0_TMOD_EEPROM_READ      (0x3U << 8)         /*!< EEPROM Read mode */

#define SPI_CR0_SLV_OE                (0x1U << 10)        /*!< Slave Output Enable */
#define SPI_CR0_SRL                   (0x1U << 11)        /*!< Shift Register Loop */

#define SPI_CR0_CFS_Msk               (0xFU << 12)        /*!< Control Frame Size field mask */
#define SPI_CR0_CFS_01_BIT            (0x0U << 12)        /*!< 1-bit Control Word */
#define SPI_CR0_CFS_02_BIT            (0x1U << 12)        /*!< 2-bit Control Word */
#define SPI_CR0_CFS_03_BIT            (0x2U << 12)        /*!< 3-bit Control Word */
#define SPI_CR0_CFS_04_BIT            (0x3U << 12)        /*!< 4-bit Control Word */
#define SPI_CR0_CFS_05_BIT            (0x4U << 12)        /*!< 5-bit Control Word */
#define SPI_CR0_CFS_06_BIT            (0x5U << 12)        /*!< 6-bit Control Word */
#define SPI_CR0_CFS_07_BIT            (0x6U << 12)        /*!< 7-bit Control Word */
#define SPI_CR0_CFS_08_BIT            (0x7U << 12)        /*!< 8-bit Control Word */
#define SPI_CR0_CFS_09_BIT            (0x8U << 12)        /*!< 9-bit Control Word */
#define SPI_CR0_CFS_10_BIT            (0x9U << 12)        /*!< 10-bit Control Word */
#define SPI_CR0_CFS_11_BIT            (0xAU << 12)        /*!< 11-bit Control Word */
#define SPI_CR0_CFS_12_BIT            (0xBU << 12)        /*!< 12-bit Control Word */
#define SPI_CR0_CFS_13_BIT            (0xCU << 12)        /*!< 13-bit Control Word */
#define SPI_CR0_CFS_14_BIT            (0xDU << 12)        /*!< 14-bit Control Word */
#define SPI_CR0_CFS_15_BIT            (0xEU << 12)        /*!< 15-bit Control Word */
#define SPI_CR0_CFS_16_BIT            (0xFU << 12)        /*!< 16-bit Control Word */

#define SPI_CR0_SPI_MODE_Msk          (0x3U << 21)        /*!< SPI Mode field mask */
#define SPI_CR0_SPI_MODE_STD          (0x0U << 21)        /*!< Standard SPI Mode */
#define SPI_CR0_SPI_MODE_DUAL         (0x1U << 21)        /*!< Dual SPI Mode */
#define SPI_CR0_SPI_MODE_QUAD         (0x2U << 21)        /*!< Quad SPI Mode */
#define SPI_CR0_SPI_MODE_OCTAL        (0x3U << 21)        /*!< Octal SPI Mode */

#define SPI_CR0_SSTE                  (0x1U << 24)        /*!< Slave Select Toggle Enable */

/******************************    Bit definition for SPI_CR1 register *******************************/
#define SPI_CR1_NDF_Msk               (0xFFFFU)           /*!< Number of Data Frames field mask */

/******************************    Bit definition for SPI_SPIENR register *******************************/
#define SPI_SPIENR_SPI_EN             (0x1U << 0)         /*!< SPI Enable */

/******************************    Bit definition for SPI_MWCR register   *******************************/
#define SPI_MWCR_MWMOD                (0x1U << 0)         /*!< Microwire Transfer Mode */
#define SPI_MWCR_MDD                  (0x1U << 1)         /*!< Microwire Control */
#define SPI_MWCR_MHS                  (0x1U << 2)         /*!< Microwire Handshaking */

/******************************    Bit definition for SPI_SER register    *******************************/
#define SPI_SER_Msk                   (0x7U << 0)         /*!< Slave Select Enable Flag field mask */
#define SPI_SER_SE0                   (0x1U << 0)         /*!< Slave 0 Select Enable Flag */
#define SPI_SER_SE1                   (0x1U << 1)         /*!< Slave 1 Select Enable Flag */
#define SPI_SER_SE2                   (0x1U << 2)         /*!< Slave 2 Select Enable Flag */

/******************************    Bit definition for SPI_BAUDR register  *******************************/
#define SPI_BAUDR_SCKDV_Msk           (0xFFFFU)           /*!< SPI Clock Divider field mask */

/******************************   Bit definition for SPI_TXFTLR register  *******************************/

/******************************   Bit definition for SPI_RXFTLR register  *******************************/

/******************************    Bit definition for SPI_TXFLR register  *******************************/

/******************************    Bit definition for SPI_RXFLR register  *******************************/

/******************************     Bit definition for SPI_SR register    *******************************/
#define SPI_SR_BUSY                   (0x1U << 0)         /*!< SPI Busy Flag */
#define SPI_SR_TFNF                   (0x1U << 1)         /*!< Transmit FIFO Not Full */
#define SPI_SR_TFE                    (0x1U << 2)         /*!< Transmit FIFO Empty */
#define SPI_SR_RFNE                   (0x1U << 3)         /*!< Receive FIFO Not Empty */
#define SPI_SR_RFF                    (0x1U << 4)         /*!< Receive FIFO Full */
#define SPI_SR_TXERR                  (0x1U << 5)         /*!< Transmission Error */
#define SPI_SR_DCOL                   (0x1U << 6)         /*!< Data Collision Error */

/******************************     Bit definition for SPI_IER register   *******************************/
#define SPI_IER_TXEIE                 (0x1U << 0)         /*!< Transmit FIFO Empty Interrupt Enable */
#define SPI_IER_TXOIE                 (0x1U << 1)         /*!< Transmit FIFO Overflow Interrupt Enable */
#define SPI_IER_RXUIE                 (0x1U << 2)         /*!< Receive FIFO Underflow Interrupt Enable */
#define SPI_IER_RXOIE                 (0x1U << 3)         /*!< Receive FIFO Overflow Interrupt Enable */
#define SPI_IER_RXFIE                 (0x1U << 4)         /*!< Receive FIFO Full Interrupt Enable */
#define SPI_IER_MSTIE                 (0x1U << 5)         /*!< Multi-Master Contention Interrupt Enable */

/******************************     Bit definition for SPI_ISR register   *******************************/
#define SPI_ISR_TXEIS                 (0x1U << 0)         /*!< Transmit FIFO Empty Interrupt Status */
#define SPI_ISR_TXOIS                 (0x1U << 1)         /*!< Transmit FIFO Overflow Interrupt Status */
#define SPI_ISR_RXUIS                 (0x1U << 2)         /*!< Receive FIFO Underflow Interrupt Status */
#define SPI_ISR_RXOIS                 (0x1U << 3)         /*!< Receive FIFO Overflow Interrupt Status */
#define SPI_ISR_RXFIS                 (0x1U << 4)         /*!< Receive FIFO Full Interrupt Status */
#define SPI_ISR_MSTIS                 (0x1U << 5)         /*!< Multi-Master Contention Interrupt Status */

/******************************    Bit definition for SPI_RISR register   *******************************/
#define SPI_RISR_TXEIR                (0x1U << 0)         /*!< Transmit FIFO Empty Raw Interrupt Status */
#define SPI_RISR_TXOIR                (0x1U << 1)         /*!< Transmit FIFO Overflow Raw Interrupt Status */
#define SPI_RISR_RXUIR                (0x1U << 2)         /*!< Receive FIFO Underflow Raw Interrupt Status */
#define SPI_RISR_RXOIR                (0x1U << 3)         /*!< Receive FIFO Overflow Raw Interrupt Status */
#define SPI_RISR_RXFIR                (0x1U << 4)         /*!< Receive FIFO Full Raw Interrupt Status */
#define SPI_RISR_MSTIR                (0x1U << 5)         /*!< Multi-Master Contention Raw Interrupt Status */


/******************************   Bit definition for SPI_DMACR register   *******************************/
#define SPI_DMACR_RDMAE               (0x1U << 0)         /*!< Receive DMA Enable */
#define SPI_DMACR_TDMAE               (0x1U << 1)         /*!< Transmit DMA Enable */

/******************************  Bit definition for SPI_DMATDLR register  *******************************/

/******************************  Bit definition for SPI_DMARDLR register  *******************************/


/******************************  Bit definition for SPI_ESPICR register  ****************************/
#define SPI_ESPICR_TRANST_Msk         (0x3U << 0)         /*!< Address and instruction transfer format field mask */

#define SPI_ESPICR_ADDRL_Msk          (0xFU << 2)         /*!< Address Length field mask */
#define SPI_ESPICR_ADDRL_0BIT         (0x0U << 2)         /*!< 0-bit Address Width */
#define SPI_ESPICR_ADDRL_4BIT         (0x1U << 2)         /*!< 4-bit Address Width */
#define SPI_ESPICR_ADDRL_8BIT         (0x2U << 2)         /*!< 8-bit Address Width */
#define SPI_ESPICR_ADDRL_12BIT        (0x3U << 2)         /*!< 12-bit Address Width */
#define SPI_ESPICR_ADDRL_16BIT        (0x4U << 2)         /*!< 16-bit Address Width */
#define SPI_ESPICR_ADDRL_20BIT        (0x5U << 2)         /*!< 20-bit Address Width */
#define SPI_ESPICR_ADDRL_24BIT        (0x6U << 2)         /*!< 24-bit Address Width */
#define SPI_ESPICR_ADDRL_28BIT        (0x7U << 2)         /*!< 28-bit Address Width */
#define SPI_ESPICR_ADDRL_32BIT        (0x8U << 2)         /*!< 32-bit Address Width */
#define SPI_ESPICR_ADDRL_36BIT        (0x9U << 2)         /*!< 36-bit Address Width */
#define SPI_ESPICR_ADDRL_40BIT        (0xAU << 2)         /*!< 40-bit Address Width */
#define SPI_ESPICR_ADDRL_44BIT        (0xBU << 2)         /*!< 44-bit Address Width */
#define SPI_ESPICR_ADDRL_48BIT        (0xCU << 2)         /*!< 48-bit Address Width */
#define SPI_ESPICR_ADDRL_52BIT        (0xDU << 2)         /*!< 52-bit Address Width */
#define SPI_ESPICR_ADDRL_56BIT        (0xEU << 2)         /*!< 56-bit Address Width */
#define SPI_ESPICR_ADDRL_60BIT        (0xFU << 2)         /*!< 60-bit Address Width */

#define SPI_ESPICR_INSTL_Msk          (0x3U << 8)         /*!< Instruction Length field mask */
#define SPI_ESPICR_INSTL_0BIT         (0x0U << 8)         /*!< 0-bit (No Instruction) */
#define SPI_ESPICR_INSTL_4BIT         (0x1U << 8)         /*!< 4-bit Instruction */
#define SPI_ESPICR_INSTL_8BIT         (0x2U << 8)         /*!< 8-bit Instruction */
#define SPI_ESPICR_INSTL_16BIT        (0x3U << 8)         /*!< 16-bit Instruction */

#define SPI_ESPICR_WCYC_Msk           (0x1FU << 11)       /*!< Wait cycles field mask */






/*------------------------------------------------------------------------------------------------------*/
/*---                                      Inter-IC Sound (I2S)                                      ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for I2S_IER register  *********************************/
#define I2S_IER_IEN             (0x1U << 0)          /*!< I2S Enable */

/********************************  Bit definition for I2S_IRER register  ********************************/
#define I2S_IRER_RXEN           (0x1U << 0)          /*!< Receiver block enable */

/********************************  Bit definition for I2S_ITER register  ********************************/
#define I2S_ITER_TXEN           (0x1U << 0)          /*!< Transmitter block enable */

/********************************  Bit definition for I2S_CER register  *********************************/
#define I2S_CER_CLKEN           (0x1U << 0)          /*!< Clock generation enable/disable */

/********************************  Bit definition for I2S_CCR register  *********************************/
#define I2S_CCR_SCLKG_Msk       (0x7U << 0)          /*!< Gating of sclk field mask */
#define I2S_CCR_SCLKG_NONE      (0x0U << 0)          /*!< Clock gating is disabled */
#define I2S_CCR_SCLKG_12        (0x1U << 0)          /*!< Gating after 12 sclk cycles */
#define I2S_CCR_SCLKG_16        (0x2U << 0)          /*!< Gating after 16 sclk cycles */
#define I2S_CCR_SCLKG_20        (0x3U << 0)          /*!< Gating after 20 sclk cycles */
#define I2S_CCR_SCLKG_24        (0x4U << 0)          /*!< Gating after 24 sclk cycles */

#define I2S_CCR_WSS_Msk         (0x3U << 3)          /*!< No description */
#define I2S_CCR_WSS_16          (0x0U << 3)          /*!< 16 sclk cycles */
#define I2S_CCR_WSS_24          (0x1U << 3)          /*!< 24 sclk cycles */
#define I2S_CCR_WSS_32          (0x2U << 3)          /*!< 32 sclk cycles */


/********************************  Bit definition for I2S_RXFFR register  *******************************/
#define I2S_RXFFR_RXFFR         (0x1U << 0)          /*!< Receiver FIFO Reset */

/********************************  Bit definition for I2S_TXFFR register  *******************************/
#define I2S_TXFFR_TXFFR         (0x1U << 0)          /*!< Transmitter FIFO Reset */





/*------------------------------------------------------------------------------------------------------*/
/*---                                   Universal Serial Bus (USB)                                   ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for USB_FADDR register  ********************************/
#define USB_FADDR_FADDR_Msk       (0x7FU)              /*!< The function address field mask */
#define USB_FADDR_UPDATE          (0x1U << 7)          /*!< Set when FAddr is written */

/********************************  Bit definition for USB_POWER register  ********************************/
#define USB_POWER_SUSEN           (0x1U << 0)          /*!< No description */
#define USB_POWER_SUSMD           (0x1U << 1)          /*!< No description */
#define USB_POWER_RESUME          (0x1U << 2)          /*!< No description */
#define USB_POWER_RSTFLG          (0x1U << 3)          /*!< No description */
#define USB_POWER_ISOUD           (0x1U << 7)          /*!< No description */

/********************************  Bit definition for USB_INTRIN register  ********************************/
#define USB_INTRIN_EP0            (0x1U << 0)          /*!< No description */
#define USB_INTRIN_IN1            (0x1U << 1)          /*!< No description */
#define USB_INTRIN_IN2            (0x1U << 2)          /*!< No description */
#define USB_INTRIN_IN3            (0x1U << 3)          /*!< No description */

/********************************  Bit definition for USB_INTROUT register  ********************************/
#define USB_INTROUT_OUT1          (0x1U << 1)          /*!< No description */
#define USB_INTROUT_OUT2          (0x1U << 2)          /*!< No description */
#define USB_INTROUT_OUT3          (0x1U << 3)          /*!< No description */

/********************************  Bit definition for USB_INTRUSB register  ********************************/
#define USB_INTRUSB_SUSIS         (0x1U << 0)          /*!< No description */
#define USB_INTRUSB_RSUIS         (0x1U << 1)          /*!< No description */
#define USB_INTRUSB_RSTIS         (0x1U << 2)          /*!< No description */
#define USB_INTRUSB_SOFIS         (0x1U << 3)          /*!< No description */

/********************************  Bit definition for USB_INTRINE register  ********************************/
#define USB_INTRINE_EP0E          (0x1U << 0)          /*!< No description */
#define USB_INTRINE_IN1E          (0x1U << 1)          /*!< No description */
#define USB_INTRINE_IN2E          (0x1U << 2)          /*!< No description */
#define USB_INTRINE_IN3E          (0x1U << 3)          /*!< No description */

/********************************  Bit definition for USB_INTROUTE register  ********************************/
#define USB_INTROUTE_OUT1E        (0x1U << 1)          /*!< No description */
#define USB_INTROUTE_OUT2E        (0x1U << 2)          /*!< No description */
#define USB_INTROUTE_OUT3E        (0x1U << 3)          /*!< No description */

/********************************  Bit definition for USB_INTRUSBE register  ********************************/
#define USB_INTRUSBE_SUSIE        (0x1U << 0)          /*!< No description */
#define USB_INTRUSBE_RSUIE        (0x1U << 1)          /*!< No description */
#define USB_INTRUSBE_RSTIE        (0x1U << 2)          /*!< No description */
#define USB_INTRUSBE_SOFIE        (0x1U << 3)          /*!< No description */

/********************************  Bit definition for USB_CSR0 register  ********************************/
#define USB_CSR0_OUTPKTRDY        (0x1U << 0)          /*!< No description */
#define USB_CSR0_INPKTRDY         (0x1U << 1)          /*!< No description */
#define USB_CSR0_SENTSTALL        (0x1U << 2)          /*!< No description */
#define USB_CSR0_DATAEND          (0x1U << 3)          /*!< No description */
#define USB_CSR0_SETUPEND         (0x1U << 4)          /*!< No description */
#define USB_CSR0_SENDSTALL        (0x1U << 5)          /*!< No description */
#define USB_CSR0_SVDOUTPKTRDY     (0x1U << 6)          /*!< No description */
#define USB_CSR0_SVDSETUPEND      (0x1U << 7)          /*!< No description */

/********************************  Bit definition for USB_INCSR1 register  ********************************/
#define USB_INCSR1_INPKTRDY       (0x1U << 0)          /*!< No description */
#define USB_INCSR1_FIFONE         (0x1U << 1)          /*!< No description */
#define USB_INCSR1_UNDERRUN       (0x1U << 2)          /*!< No description */
#define USB_INCSR1_FLUSHFIFO      (0x1U << 3)          /*!< No description */
#define USB_INCSR1_SENDSTALL      (0x1U << 4)          /*!< No description */
#define USB_INCSR1_SENTSTALL      (0x1U << 5)          /*!< No description */
#define USB_INCSR1_CLRDATATOG     (0x1U << 6)          /*!< No description */

/********************************  Bit definition for USB_INCSR2 register  ********************************/
#define USB_INCSR2_FRCDATATOG     (0x1U << 3)          /*!< No description */
#define USB_INCSR2_DMAEN          (0x1U << 4)          /*!< No description */
#define USB_INCSR2_DIRSEL         (0x1U << 5)          /*!< No description */
#define USB_INCSR2_ISO            (0x1U << 6)          /*!< No description */
#define USB_INCSR2_AUTOSET        (0x1U << 7)          /*!< No description */

/********************************  Bit definition for USB_OUTCSR1 register  ********************************/
#define USB_OUTCSR1_OUTPKTRDY     (0x1U << 0)          /*!< No description */
#define USB_OUTCSR1_FIFOFULL      (0x1U << 1)          /*!< No description */
#define USB_OUTCSR1_OVERRUN       (0x1U << 2)          /*!< No description */
#define USB_OUTCSR1_DATAERROR     (0x1U << 3)          /*!< No description */
#define USB_OUTCSR1_FLUSHFIFO     (0x1U << 4)          /*!< No description */
#define USB_OUTCSR1_SENDSTALL     (0x1U << 5)          /*!< No description */
#define USB_OUTCSR1_SENTSTALL     (0x1U << 6)          /*!< No description */
#define USB_OUTCSR1_CLRDATATOG    (0x1U << 7)          /*!< No description */

/********************************  Bit definition for USB_OUTCSR2 register  ********************************/
#define USB_OUTCSR2_DMAMODE       (0x1U << 4)          /*!< No description */
#define USB_OUTCSR2_DMAEN         (0x1U << 5)          /*!< No description */
#define USB_OUTCSR2_ISO           (0x1U << 6)          /*!< No description */
#define USB_OUTCSR2_AUTOCLR       (0x1U << 7)          /*!< No description */





/*------------------------------------------------------------------------------------------------------*/
/*---                                           Timer (TIM)                                          ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for TIM_CR1 register  ********************************/
#define TIM_CR1_CEN             (0x1U << 0)          /*!< Counter enable */
#define TIM_CR1_UDIS            (0x1U << 1)          /*!< Update disable */
#define TIM_CR1_URS             (0x1U << 2)          /*!< Update request source */
#define TIM_CR1_OPM             (0x1U << 3)          /*!< One pulse mode */
#define TIM_CR1_DIR             (0x1U << 4)          /*!< Direction */

#define TIM_CR1_CMS             (0x3U << 5)          /*!< CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0           (0x1U << 5)          /*!< Bit 0 */
#define TIM_CR1_CMS_1           (0x1U << 6)          /*!< Bit 1 */

#define TIM_CR1_ARPE            (0x1U << 7)          /*!< Auto-reload preload enable */

#define TIM_CR1_CKD             (0x3U << 8)          /*!< CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0           (0x1U << 8)          /*!< Bit 0 */
#define TIM_CR1_CKD_1           (0x1U << 9)          /*!< Bit 1 */

/********************************  Bit definition for TIM_CR2 register  ********************************/
#define TIM_CR2_CCPC            (0x1U << 0)          /*!< Capture/Compare Preloaded Control */
#define TIM_CR2_CCUS            (0x1U << 2)          /*!< Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS            (0x1U << 3)          /*!< Capture/Compare DMA Selection */

#define TIM_CR2_MMS             (0x7U << 4)          /*!< MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0           (0x1U << 4)          /*!< Bit 0 */
#define TIM_CR2_MMS_1           (0x1U << 5)          /*!< Bit 1 */
#define TIM_CR2_MMS_2           (0x1U << 6)          /*!< Bit 2 */

#define TIM_CR2_TI1S            (0x1U << 7)          /*!< TI1 Selection */
#define TIM_CR2_OIS1            (0x1U << 8)          /*!< Output Idle state 1 (OC1 output) */
#define TIM_CR2_OIS1N           (0x1U << 9)          /*!< Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2            (0x1U << 10)         /*!< Output Idle state 2 (OC2 output) */
#define TIM_CR2_OIS2N           (0x1U << 11)         /*!< Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3            (0x1U << 12)         /*!< Output Idle state 3 (OC3 output) */
#define TIM_CR2_OIS3N           (0x1U << 13)         /*!< Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4            (0x1U << 14)         /*!< Output Idle state 4 (OC4 output) */

/********************************  Bit definition for TIM_SMCR register  ********************************/
#define TIM_SMCR_SMS            (0x7U << 0)          /*!< SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_SMS_0          (0x1U << 0)          /*!< Bit 0 */
#define TIM_SMCR_SMS_1          (0x1U << 1)          /*!< Bit 1 */
#define TIM_SMCR_SMS_2          (0x1U << 2)          /*!< Bit 2 */

#define TIM_SMCR_TS             (0x7U << 4)          /*!< TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_TS_0           (0x1U << 4)          /*!< Bit 0 */
#define TIM_SMCR_TS_1           (0x1U << 5)          /*!< Bit 1 */
#define TIM_SMCR_TS_2           (0x1U << 6)          /*!< Bit 2 */

#define TIM_SMCR_MSM            (0x1U << 7)          /*!< Master/slave mode */

#define TIM_SMCR_ETF            (0xFU << 8)          /*!< ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0          (0x1U << 8)          /*!< Bit 0 */
#define TIM_SMCR_ETF_1          (0x1U << 9)          /*!< Bit 1 */
#define TIM_SMCR_ETF_2          (0x1U << 10)         /*!< Bit 2 */
#define TIM_SMCR_ETF_3          (0x1U << 11)         /*!< Bit 3 */

#define TIM_SMCR_ETPS           (0x3U << 12)         /*!< ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0         (0x1U << 12)         /*!< Bit 0 */
#define TIM_SMCR_ETPS_1         (0x1U << 13)         /*!< Bit 1 */

#define TIM_SMCR_ECE            (0x1U << 14)         /*!< External clock enable */
#define TIM_SMCR_ETP            (0x1U << 15)         /*!< External trigger polarity */

/********************************  Bit definition for TIM_DIER register  ********************************/
#define TIM_DIER_UIE            (0x1U << 0)          /*!< Update interrupt enable */
#define TIM_DIER_CC1IE          (0x1U << 1)          /*!< Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE          (0x1U << 2)          /*!< Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE          (0x1U << 3)          /*!< Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE          (0x1U << 4)          /*!< Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE          (0x1U << 5)          /*!< COM interrupt enable */
#define TIM_DIER_TIE            (0x1U << 6)          /*!< Trigger interrupt enable */
#define TIM_DIER_BIE            (0x1U << 7)          /*!< Break interrupt enable */
#define TIM_DIER_UDE            (0x1U << 8)          /*!< Update DMA request enable */
#define TIM_DIER_CC1DE          (0x1U << 9)          /*!< Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE          (0x1U << 10)         /*!< Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE          (0x1U << 11)         /*!< Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE          (0x1U << 12)         /*!< Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE          (0x1U << 13)         /*!< COM DMA request enable */
#define TIM_DIER_TDE            (0x1U << 14)         /*!< Trigger DMA request enable */

/********************************  Bit definition for TIM_SR register  ********************************/
#define TIM_SR_UIF              (0x1U << 0)          /*!< Update interrupt Flag */
#define TIM_SR_CC1IF            (0x1U << 1)          /*!< Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF            (0x1U << 2)          /*!< Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF            (0x1U << 3)          /*!< Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF            (0x1U << 4)          /*!< Capture/Compare 4 interrupt Flag */
#define TIM_SR_COMIF            (0x1U << 5)          /*!< COM interrupt Flag */
#define TIM_SR_TIF              (0x1U << 6)          /*!< Trigger interrupt Flag */
#define TIM_SR_BIF              (0x1U << 7)          /*!< Break interrupt Flag */
#define TIM_SR_CC1OF            (0x1U << 9)          /*!< Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF            (0x1U << 10)         /*!< Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF            (0x1U << 11)         /*!< Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF            (0x1U << 12)         /*!< Capture/Compare 4 Overcapture Flag */

/********************************  Bit definition for TIM_EGR register  ********************************/
#define TIM_EGR_UG              (0x1U << 0)          /*!< Update Generation */
#define TIM_EGR_CC1G            (0x1U << 1)          /*!< Capture/Compare 1 Generation */
#define TIM_EGR_CC2G            (0x1U << 2)          /*!< Capture/Compare 2 Generation */
#define TIM_EGR_CC3G            (0x1U << 3)          /*!< Capture/Compare 3 Generation */
#define TIM_EGR_CC4G            (0x1U << 4)          /*!< Capture/Compare 4 Generation */
#define TIM_EGR_COMG            (0x1U << 5)          /*!< Capture/Compare Control Update Generation */
#define TIM_EGR_TG              (0x1U << 6)          /*!< Trigger Generation */
#define TIM_EGR_BG              (0x1U << 7)          /*!< Break Generation */

/********************************  Bit definition for TIM_CCMR1 register  ********************************/
#define TIM_CCMR1_CC1S          (0x3U << 0)          /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0        (0x1U << 0)          /*!< Bit 0 */
#define TIM_CCMR1_CC1S_1        (0x1U << 1)          /*!< Bit 1 */

#define TIM_CCMR1_OC1FE         (0x1U << 2)          /*!< Output Compare 1 Fast enable */
#define TIM_CCMR1_OC1PE         (0x1U << 3)          /*!< Output Compare 1 Preload enable */

#define TIM_CCMR1_OC1M          (0x7U << 4)          /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0        (0x1U << 4)          /*!< Bit 0 */
#define TIM_CCMR1_OC1M_1        (0x1U << 5)          /*!< Bit 1 */
#define TIM_CCMR1_OC1M_2        (0x1U << 6)          /*!< Bit 2 */

#define TIM_CCMR1_OC1CE         (0x1U << 7)          /*!< Output Compare 1 Clear Enable */

#define TIM_CCMR1_CC2S          (0x3U << 8)          /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0        (0x1U << 8)          /*!< Bit 0 */
#define TIM_CCMR1_CC2S_1        (0x1U << 9)          /*!< Bit 1 */

#define TIM_CCMR1_OC2FE         (0x1U << 10)         /*!< Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE         (0x1U << 11)         /*!< Output Compare 2 Preload enable */

#define TIM_CCMR1_OC2M          (0x7U << 12)         /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0        (0x1U << 12)         /*!< Bit 0 */
#define TIM_CCMR1_OC2M_1        (0x1U << 13)         /*!< Bit 1 */
#define TIM_CCMR1_OC2M_2        (0x1U << 14)         /*!< Bit 2 */

#define TIM_CCMR1_OC2CE         (0x1U << 15)         /*!< Output Compare 2 Clear Enable */

/*------------------------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC        (0x3U << 2)          /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0      (0x1U << 2)          /*!< Bit 0 */
#define TIM_CCMR1_IC1PSC_1      (0x1U << 3)          /*!< Bit 1 */

#define TIM_CCMR1_IC1F          (0xFU << 4)          /*!< IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC1F_0        (0x1U << 4)          /*!< Bit 0 */
#define TIM_CCMR1_IC1F_1        (0x1U << 5)          /*!< Bit 1 */
#define TIM_CCMR1_IC1F_2        (0x1U << 6)          /*!< Bit 2 */
#define TIM_CCMR1_IC1F_3        (0x1U << 7)          /*!< Bit 3 */

#define TIM_CCMR1_IC2PSC        (0x3U << 10)         /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2PSC_0      (0x1U << 10)         /*!< Bit 0 */
#define TIM_CCMR1_IC2PSC_1      (0x1U << 11)         /*!< Bit 1 */

#define TIM_CCMR1_IC2F          (0xFU << 12)         /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMR1_IC2F_0        (0x1U << 12)         /*!< Bit 0 */
#define TIM_CCMR1_IC2F_1        (0x1U << 13)         /*!< Bit 1 */
#define TIM_CCMR1_IC2F_2        (0x1U << 14)         /*!< Bit 2 */
#define TIM_CCMR1_IC2F_3        (0x1U << 15)         /*!< Bit 3 */

/********************************  Bit definition for TIM_CCMR2 register  ********************************/
#define TIM_CCMR2_CC3S          (0x3U << 0)          /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_CC3S_0        (0x1U << 0)          /*!< Bit 0 */
#define TIM_CCMR2_CC3S_1        (0x1U << 1)          /*!< Bit 1 */

#define TIM_CCMR2_OC3FE         (0x1U << 2)          /*!< Output Compare 3 Fast enable */
#define TIM_CCMR2_OC3PE         (0x1U << 3)          /*!< Output Compare 3 Preload enable */

#define TIM_CCMR2_OC3M          (0x7U << 4)          /*!< OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0        (0x1U << 4)          /*!< Bit 0 */
#define TIM_CCMR2_OC3M_1        (0x1U << 5)          /*!< Bit 1 */
#define TIM_CCMR2_OC3M_2        (0x1U << 6)          /*!< Bit 2 */

#define TIM_CCMR2_OC3CE         (0x1U << 7)          /*!< Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S          (0x3U << 8)          /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0        (0x1U << 8)          /*!< Bit 0 */
#define TIM_CCMR2_CC4S_1        (0x1U << 9)          /*!< Bit 1 */

#define TIM_CCMR2_OC4FE         (0x1U << 10)         /*!< Output Compare 4 Fast enable */
#define TIM_CCMR2_OC4PE         (0x1U << 11)         /*!< Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M          (0x7U << 12)         /*!< OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0        (0x1U << 12)         /*!< Bit 0 */
#define TIM_CCMR2_OC4M_1        (0x1U << 13)         /*!< Bit 1 */
#define TIM_CCMR2_OC4M_2        (0x1U << 14)         /*!< Bit 2 */

#define TIM_CCMR2_OC4CE         (0x1U << 15)         /*!< Output Compare 4 Clear Enable */

/*------------------------------------------------------------------------------------------*/

#define TIM_CCMR2_IC3PSC        (0x3U << 2)          /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0      (0x1U << 2)          /*!< Bit 0 */
#define TIM_CCMR2_IC3PSC_1      (0x1U << 3)          /*!< Bit 1 */

#define TIM_CCMR2_IC3F          (0xFU << 4)          /*!< IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0        (0x1U << 4)          /*!< Bit 0 */
#define TIM_CCMR2_IC3F_1        (0x1U << 5)          /*!< Bit 1 */
#define TIM_CCMR2_IC3F_2        (0x1U << 6)          /*!< Bit 2 */
#define TIM_CCMR2_IC3F_3        (0x1U << 7)          /*!< Bit 3 */

#define TIM_CCMR2_IC4PSC        (0x3U << 10)         /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0      (0x1U << 10)         /*!< Bit 0 */
#define TIM_CCMR2_IC4PSC_1      (0x1U << 11)         /*!< Bit 1 */

#define TIM_CCMR2_IC4F          (0xFU << 12)         /*!< IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0        (0x1U << 12)         /*!< Bit 0 */
#define TIM_CCMR2_IC4F_1        (0x1U << 13)         /*!< Bit 1 */
#define TIM_CCMR2_IC4F_2        (0x1U << 14)         /*!< Bit 2 */
#define TIM_CCMR2_IC4F_3        (0x1U << 15)         /*!< Bit 3 */

/********************************  Bit definition for TIM_CCER register  ********************************/
#define TIM_CCER_CC1E           (0x1U << 0)          /*!< Capture/Compare 1 output enable */
#define TIM_CCER_CC1P           (0x1U << 1)          /*!< Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE          (0x1U << 2)          /*!< Capture/Compare 1 Complementary output enable */
#define TIM_CCER_CC1NP          (0x1U << 3)          /*!< Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E           (0x1U << 4)          /*!< Capture/Compare 2 output enable */
#define TIM_CCER_CC2P           (0x1U << 5)          /*!< Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE          (0x1U << 6)          /*!< Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP          (0x1U << 7)          /*!< Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E           (0x1U << 8)          /*!< Capture/Compare 3 output enable */
#define TIM_CCER_CC3P           (0x1U << 9)          /*!< Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE          (0x1U << 10)         /*!< Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP          (0x1U << 11)         /*!< Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E           (0x1U << 12)         /*!< Capture/Compare 4 output enable */
#define TIM_CCER_CC4P           (0x1U << 13)         /*!< Capture/Compare 4 output Polarity */

/********************************  Bit definition for TIM_CNT register  ********************************/
#define TIM_CNT_CNT             (0xFFFFFU)           /*!< Counter Value */

/********************************  Bit definition for TIM_PSC register  ********************************/
#define TIM_PSC_PSC             (0xFFFFU)            /*!< Prescaler Value */

/********************************  Bit definition for TIM_ARR register  ********************************/
#define TIM_ARR_ARR             (0xFFFFFU)           /*!< actual auto-reload Value */

/********************************  Bit definition for TIM_RCR register  ********************************/
#define TIM_RCR_REP             (0xFFU)              /*!< Repetition Counter Value */

/********************************  Bit definition for TIM_CCR1 register  ********************************/
#define TIM_CCR1_CCR1           (0xFFFFFU)           /*!< Capture/Compare 1 Value */

/********************************  Bit definition for TIM_CCR2 register  ********************************/
#define TIM_CCR2_CCR2           (0xFFFFFU)           /*!< Capture/Compare 2 Value */

/********************************  Bit definition for TIM_CCR3 register  ********************************/
#define TIM_CCR3_CCR3           (0xFFFFFU)           /*!< Capture/Compare 3 Value */

/********************************  Bit definition for TIM_CCR4 register  ********************************/
#define TIM_CCR4_CCR4           (0xFFFFFU)           /*!< Capture/Compare 4 Value */

/********************************  Bit definition for TIM_BDTR register  ********************************/
#define TIM_BDTR_DTG            (0xFFU << 0)         /*!< DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0          (0x1U << 0)          /*!< Bit 0 */
#define TIM_BDTR_DTG_1          (0x1U << 1)          /*!< Bit 1 */
#define TIM_BDTR_DTG_2          (0x1U << 2)          /*!< Bit 2 */
#define TIM_BDTR_DTG_3          (0x1U << 3)          /*!< Bit 3 */
#define TIM_BDTR_DTG_4          (0x1U << 4)          /*!< Bit 4 */
#define TIM_BDTR_DTG_5          (0x1U << 5)          /*!< Bit 5 */
#define TIM_BDTR_DTG_6          (0x1U << 6)          /*!< Bit 6 */
#define TIM_BDTR_DTG_7          (0x1U << 7)          /*!< Bit 7 */

#define TIM_BDTR_LOCK           (0x3U << 8)          /*!< LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0         (0x1U << 8)          /*!< Bit 0 */
#define TIM_BDTR_LOCK_1         (0x1U << 9)          /*!< Bit 1 */

#define TIM_BDTR_OSSI           (0x1U << 10)         /*!< Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR           (0x1U << 11)         /*!< Off-State Selection for Run mode */
#define TIM_BDTR_BKE            (0x1U << 12)         /*!< Break enable */
#define TIM_BDTR_BKP            (0x1U << 13)         /*!< Break Polarity */
#define TIM_BDTR_AOE            (0x1U << 14)         /*!< Automatic Output enable */
#define TIM_BDTR_MOE            (0x1U << 15)         /*!< Main Output enable */


/*------------------------------------------------------------------------------------------------------*/
/*---                           External Interrupt/Event Controller (EXTI)                           ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for EXTI_IMR register  ********************************/
#define  EXTI_IMR_MR0               ((uint32_t)0x00000001)        /*!< Interrupt Mask on line 0 */
#define  EXTI_IMR_MR1               ((uint32_t)0x00000002)        /*!< Interrupt Mask on line 1 */
#define  EXTI_IMR_MR2               ((uint32_t)0x00000004)        /*!< Interrupt Mask on line 2 */
#define  EXTI_IMR_MR3               ((uint32_t)0x00000008)        /*!< Interrupt Mask on line 3 */
#define  EXTI_IMR_MR4               ((uint32_t)0x00000010)        /*!< Interrupt Mask on line 4 */
#define  EXTI_IMR_MR5               ((uint32_t)0x00000020)        /*!< Interrupt Mask on line 5 */
#define  EXTI_IMR_MR6               ((uint32_t)0x00000040)        /*!< Interrupt Mask on line 6 */
#define  EXTI_IMR_MR7               ((uint32_t)0x00000080)        /*!< Interrupt Mask on line 7 */
#define  EXTI_IMR_MR8               ((uint32_t)0x00000100)        /*!< Interrupt Mask on line 8 */
#define  EXTI_IMR_MR9               ((uint32_t)0x00000200)        /*!< Interrupt Mask on line 9 */
#define  EXTI_IMR_MR10              ((uint32_t)0x00000400)        /*!< Interrupt Mask on line 10 */
#define  EXTI_IMR_MR11              ((uint32_t)0x00000800)        /*!< Interrupt Mask on line 11 */
#define  EXTI_IMR_MR12              ((uint32_t)0x00001000)        /*!< Interrupt Mask on line 12 */
#define  EXTI_IMR_MR13              ((uint32_t)0x00002000)        /*!< Interrupt Mask on line 13 */
#define  EXTI_IMR_MR14              ((uint32_t)0x00004000)        /*!< Interrupt Mask on line 14 */
#define  EXTI_IMR_MR15              ((uint32_t)0x00008000)        /*!< Interrupt Mask on line 15 */
#define  EXTI_IMR_MR16              ((uint32_t)0x00010000)        /*!< Interrupt Mask on line 16 */
#define  EXTI_IMR_MR17              ((uint32_t)0x00020000)        /*!< Interrupt Mask on line 17 */
#define  EXTI_IMR_MR18              ((uint32_t)0x00040000)        /*!< Interrupt Mask on line 18 */

/********************************  Bit definition for EXTI_EMR register  ********************************/
#define  EXTI_EMR_MR0               ((uint32_t)0x00000001)        /*!< Event Mask on line 0 */
#define  EXTI_EMR_MR1               ((uint32_t)0x00000002)        /*!< Event Mask on line 1 */
#define  EXTI_EMR_MR2               ((uint32_t)0x00000004)        /*!< Event Mask on line 2 */
#define  EXTI_EMR_MR3               ((uint32_t)0x00000008)        /*!< Event Mask on line 3 */
#define  EXTI_EMR_MR4               ((uint32_t)0x00000010)        /*!< Event Mask on line 4 */
#define  EXTI_EMR_MR5               ((uint32_t)0x00000020)        /*!< Event Mask on line 5 */
#define  EXTI_EMR_MR6               ((uint32_t)0x00000040)        /*!< Event Mask on line 6 */
#define  EXTI_EMR_MR7               ((uint32_t)0x00000080)        /*!< Event Mask on line 7 */
#define  EXTI_EMR_MR8               ((uint32_t)0x00000100)        /*!< Event Mask on line 8 */
#define  EXTI_EMR_MR9               ((uint32_t)0x00000200)        /*!< Event Mask on line 9 */
#define  EXTI_EMR_MR10              ((uint32_t)0x00000400)        /*!< Event Mask on line 10 */
#define  EXTI_EMR_MR11              ((uint32_t)0x00000800)        /*!< Event Mask on line 11 */
#define  EXTI_EMR_MR12              ((uint32_t)0x00001000)        /*!< Event Mask on line 12 */
#define  EXTI_EMR_MR13              ((uint32_t)0x00002000)        /*!< Event Mask on line 13 */
#define  EXTI_EMR_MR14              ((uint32_t)0x00004000)        /*!< Event Mask on line 14 */
#define  EXTI_EMR_MR15              ((uint32_t)0x00008000)        /*!< Event Mask on line 15 */
#define  EXTI_EMR_MR16              ((uint32_t)0x00010000)        /*!< Event Mask on line 16 */
#define  EXTI_EMR_MR17              ((uint32_t)0x00020000)        /*!< Event Mask on line 17 */
#define  EXTI_EMR_MR18              ((uint32_t)0x00040000)        /*!< Event Mask on line 18 */

/********************************  Bit definition for EXTI_RTSR register  ********************************/
#define  EXTI_RTSR_TR0              ((uint32_t)0x00000001)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR_TR1              ((uint32_t)0x00000002)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR_TR2              ((uint32_t)0x00000004)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR_TR3              ((uint32_t)0x00000008)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR_TR4              ((uint32_t)0x00000010)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR_TR5              ((uint32_t)0x00000020)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR_TR6              ((uint32_t)0x00000040)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR_TR7              ((uint32_t)0x00000080)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR_TR8              ((uint32_t)0x00000100)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR_TR9              ((uint32_t)0x00000200)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR_TR10             ((uint32_t)0x00000400)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR_TR11             ((uint32_t)0x00000800)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR_TR12             ((uint32_t)0x00001000)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR_TR13             ((uint32_t)0x00002000)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR_TR14             ((uint32_t)0x00004000)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR_TR15             ((uint32_t)0x00008000)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR_TR16             ((uint32_t)0x00010000)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR_TR17             ((uint32_t)0x00020000)        /*!< Rising trigger event configuration bit of line 17 */
#define  EXTI_RTSR_TR18             ((uint32_t)0x00040000)        /*!< Rising trigger event configuration bit of line 18 */

/********************************  Bit definition for EXTI_FTSR register  ********************************/
#define  EXTI_FTSR_TR0              ((uint32_t)0x00000001)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR_TR1              ((uint32_t)0x00000002)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR_TR2              ((uint32_t)0x00000004)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR_TR3              ((uint32_t)0x00000008)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR_TR4              ((uint32_t)0x00000010)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR_TR5              ((uint32_t)0x00000020)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR_TR6              ((uint32_t)0x00000040)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR_TR7              ((uint32_t)0x00000080)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR_TR8              ((uint32_t)0x00000100)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR_TR9              ((uint32_t)0x00000200)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR_TR10             ((uint32_t)0x00000400)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR_TR11             ((uint32_t)0x00000800)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR_TR12             ((uint32_t)0x00001000)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR_TR13             ((uint32_t)0x00002000)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR_TR14             ((uint32_t)0x00004000)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR_TR15             ((uint32_t)0x00008000)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR_TR16             ((uint32_t)0x00010000)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR_TR17             ((uint32_t)0x00020000)        /*!< Falling trigger event configuration bit of line 17 */
#define  EXTI_FTSR_TR18             ((uint32_t)0x00040000)        /*!< Falling trigger event configuration bit of line 18 */

/********************************  Bit definition for EXTI_SWIER register  ********************************/
#define  EXTI_SWIER_SWIER0          ((uint32_t)0x00000001)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIER_SWIER1          ((uint32_t)0x00000002)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIER_SWIER2          ((uint32_t)0x00000004)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIER_SWIER3          ((uint32_t)0x00000008)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIER_SWIER4          ((uint32_t)0x00000010)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIER_SWIER5          ((uint32_t)0x00000020)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIER_SWIER6          ((uint32_t)0x00000040)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIER_SWIER7          ((uint32_t)0x00000080)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIER_SWIER8          ((uint32_t)0x00000100)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIER_SWIER9          ((uint32_t)0x00000200)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIER_SWIER10         ((uint32_t)0x00000400)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIER_SWIER11         ((uint32_t)0x00000800)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIER_SWIER12         ((uint32_t)0x00001000)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIER_SWIER13         ((uint32_t)0x00002000)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIER_SWIER14         ((uint32_t)0x00004000)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIER_SWIER15         ((uint32_t)0x00008000)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIER_SWIER16         ((uint32_t)0x00010000)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIER_SWIER17         ((uint32_t)0x00020000)        /*!< Software Interrupt on line 17 */
#define  EXTI_SWIER_SWIER18         ((uint32_t)0x00040000)        /*!< Software Interrupt on line 18 */

/********************************  Bit definition for EXTI_PR register  ********************************/
#define  EXTI_PR_PR0                ((uint32_t)0x00000001)        /*!< Pending bit for line 0 */
#define  EXTI_PR_PR1                ((uint32_t)0x00000002)        /*!< Pending bit for line 1 */
#define  EXTI_PR_PR2                ((uint32_t)0x00000004)        /*!< Pending bit for line 2 */
#define  EXTI_PR_PR3                ((uint32_t)0x00000008)        /*!< Pending bit for line 3 */
#define  EXTI_PR_PR4                ((uint32_t)0x00000010)        /*!< Pending bit for line 4 */
#define  EXTI_PR_PR5                ((uint32_t)0x00000020)        /*!< Pending bit for line 5 */
#define  EXTI_PR_PR6                ((uint32_t)0x00000040)        /*!< Pending bit for line 6 */
#define  EXTI_PR_PR7                ((uint32_t)0x00000080)        /*!< Pending bit for line 7 */
#define  EXTI_PR_PR8                ((uint32_t)0x00000100)        /*!< Pending bit for line 8 */
#define  EXTI_PR_PR9                ((uint32_t)0x00000200)        /*!< Pending bit for line 9 */
#define  EXTI_PR_PR10               ((uint32_t)0x00000400)        /*!< Pending bit for line 10 */
#define  EXTI_PR_PR11               ((uint32_t)0x00000800)        /*!< Pending bit for line 11 */
#define  EXTI_PR_PR12               ((uint32_t)0x00001000)        /*!< Pending bit for line 12 */
#define  EXTI_PR_PR13               ((uint32_t)0x00002000)        /*!< Pending bit for line 13 */
#define  EXTI_PR_PR14               ((uint32_t)0x00004000)        /*!< Pending bit for line 14 */
#define  EXTI_PR_PR15               ((uint32_t)0x00008000)        /*!< Pending bit for line 15 */
#define  EXTI_PR_PR16               ((uint32_t)0x00010000)        /*!< Pending bit for line 16 */
#define  EXTI_PR_PR17               ((uint32_t)0x00020000)        /*!< Pending bit for line 17 */
#define  EXTI_PR_PR18               ((uint32_t)0x00040000)        /*!< Pending bit for line 18 */


/*------------------------------------------------------------------------------------------------------*/
/*---                                 Alternate Function I/O (AFIO)                                  ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for AFIO_EXTICR1 register  ********************************/
#define AFIO_EXTICR1_EXTI0_Msk               ((uint16_t)0x000F)            /*!< EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI1_Msk               ((uint16_t)0x00F0)            /*!< EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI2_Msk               ((uint16_t)0x0F00)            /*!< EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI3_Msk               ((uint16_t)0xF000)            /*!< EXTI 3 configuration */

/*!< EXTI0 configuration */
#define AFIO_EXTICR1_EXTI0_PA                ((uint16_t)0x0000)            /*!< PA[0] pin */
#define AFIO_EXTICR1_EXTI0_PB                ((uint16_t)0x0001)            /*!< PB[0] pin */
#define AFIO_EXTICR1_EXTI0_PC                ((uint16_t)0x0002)            /*!< PC[0] pin */
#define AFIO_EXTICR1_EXTI0_PD                ((uint16_t)0x0003)            /*!< PD[0] pin */

/*!< EXTI1 configuration */
#define AFIO_EXTICR1_EXTI1_PA                ((uint16_t)0x0000)            /*!< PA[1] pin */
#define AFIO_EXTICR1_EXTI1_PB                ((uint16_t)0x0010)            /*!< PB[1] pin */
#define AFIO_EXTICR1_EXTI1_PC                ((uint16_t)0x0020)            /*!< PC[1] pin */
#define AFIO_EXTICR1_EXTI1_PD                ((uint16_t)0x0030)            /*!< PD[1] pin */

/*!< EXTI2 configuration */
#define AFIO_EXTICR1_EXTI2_PA                ((uint16_t)0x0000)            /*!< PA[2] pin */
#define AFIO_EXTICR1_EXTI2_PB                ((uint16_t)0x0100)            /*!< PB[2] pin */
#define AFIO_EXTICR1_EXTI2_PC                ((uint16_t)0x0200)            /*!< PC[2] pin */
#define AFIO_EXTICR1_EXTI2_PD                ((uint16_t)0x0300)            /*!< PD[2] pin */

/*!< EXTI3 configuration */
#define AFIO_EXTICR1_EXTI3_PA                ((uint16_t)0x0000)            /*!< PA[3] pin */
#define AFIO_EXTICR1_EXTI3_PB                ((uint16_t)0x1000)            /*!< PB[3] pin */
#define AFIO_EXTICR1_EXTI3_PC                ((uint16_t)0x2000)            /*!< PC[3] pin */

/********************************  Bit definition for AFIO_EXTICR2 register  ********************************/
#define AFIO_EXTICR2_EXTI4_Msk               ((uint16_t)0x000F)            /*!< EXTI 4 configuration */
#define AFIO_EXTICR2_EXTI5_Msk               ((uint16_t)0x00F0)            /*!< EXTI 5 configuration */
#define AFIO_EXTICR2_EXTI6_Msk               ((uint16_t)0x0F00)            /*!< EXTI 6 configuration */
#define AFIO_EXTICR2_EXTI7_Msk               ((uint16_t)0xF000)            /*!< EXTI 7 configuration */

/*!< EXTI4 configuration */
#define AFIO_EXTICR2_EXTI4_PA                ((uint16_t)0x0000)            /*!< PA[4] pin */
#define AFIO_EXTICR2_EXTI4_PB                ((uint16_t)0x0001)            /*!< PB[4] pin */
#define AFIO_EXTICR2_EXTI4_PC                ((uint16_t)0x0002)            /*!< PC[4] pin */

/*!< EXTI5 configuration */
#define AFIO_EXTICR2_EXTI5_PA                ((uint16_t)0x0000)            /*!< PA[5] pin */
#define AFIO_EXTICR2_EXTI5_PB                ((uint16_t)0x0010)            /*!< PB[5] pin */
#define AFIO_EXTICR2_EXTI5_PC                ((uint16_t)0x0020)            /*!< PC[5] pin */

/*!< EXTI6 configuration */
#define AFIO_EXTICR2_EXTI6_PA                ((uint16_t)0x0000)            /*!< PA[6] pin */
#define AFIO_EXTICR2_EXTI6_PB                ((uint16_t)0x0100)            /*!< PB[6] pin */
#define AFIO_EXTICR2_EXTI6_PC                ((uint16_t)0x0200)            /*!< PC[6] pin */

/*!< EXTI7 configuration */
#define AFIO_EXTICR2_EXTI7_PA                ((uint16_t)0x0000)            /*!< PA[7] pin */
#define AFIO_EXTICR2_EXTI7_PB                ((uint16_t)0x1000)            /*!< PB[7] pin */
#define AFIO_EXTICR2_EXTI7_PC                ((uint16_t)0x2000)            /*!< PC[7] pin */

/********************************  Bit definition for AFIO_EXTICR3 register  ********************************/
#define AFIO_EXTICR3_EXTI8_Msk               ((uint16_t)0x000F)            /*!< EXTI 8 configuration */
#define AFIO_EXTICR3_EXTI9_Msk               ((uint16_t)0x00F0)            /*!< EXTI 9 configuration */
#define AFIO_EXTICR3_EXTI10_Msk              ((uint16_t)0x0F00)            /*!< EXTI 10 configuration */
#define AFIO_EXTICR3_EXTI11_Msk              ((uint16_t)0xF000)            /*!< EXTI 11 configuration */

/*!< EXTI8 configuration */
#define AFIO_EXTICR3_EXTI8_PA                ((uint16_t)0x0000)            /*!< PA[8] pin */
#define AFIO_EXTICR3_EXTI8_PB                ((uint16_t)0x0001)            /*!< PB[8] pin */
#define AFIO_EXTICR3_EXTI8_PC                ((uint16_t)0x0002)            /*!< PC[8] pin */

/*!< EXTI9 configuration */
#define AFIO_EXTICR3_EXTI9_PA                ((uint16_t)0x0000)            /*!< PA[9] pin */
#define AFIO_EXTICR3_EXTI9_PB                ((uint16_t)0x0010)            /*!< PB[9] pin */
#define AFIO_EXTICR3_EXTI9_PC                ((uint16_t)0x0020)            /*!< PC[9] pin */

/*!< EXTI10 configuration */
#define AFIO_EXTICR3_EXTI10_PA               ((uint16_t)0x0000)            /*!< PA[10] pin */
#define AFIO_EXTICR3_EXTI10_PB               ((uint16_t)0x0100)            /*!< PB[10] pin */
#define AFIO_EXTICR3_EXTI10_PC               ((uint16_t)0x0200)            /*!< PC[10] pin */

/*!< EXTI11 configuration */
#define AFIO_EXTICR3_EXTI11_PA               ((uint16_t)0x0000)            /*!< PA[11] pin */
#define AFIO_EXTICR3_EXTI11_PB               ((uint16_t)0x1000)            /*!< PB[11] pin */
#define AFIO_EXTICR3_EXTI11_PC               ((uint16_t)0x2000)            /*!< PC[11] pin */

/********************************  Bit definition for AFIO_EXTICR4 register  ********************************/
#define AFIO_EXTICR4_EXTI12_Msk              ((uint16_t)0x000F)            /*!< EXTI 12 configuration */
#define AFIO_EXTICR4_EXTI13_Msk              ((uint16_t)0x00F0)            /*!< EXTI 13 configuration */
#define AFIO_EXTICR4_EXTI14_Msk              ((uint16_t)0x0F00)            /*!< EXTI 14 configuration */
#define AFIO_EXTICR4_EXTI15_Msk              ((uint16_t)0xF000)            /*!< EXTI 15 configuration */

/* EXTI12 configuration */
#define AFIO_EXTICR4_EXTI12_PA               ((uint16_t)0x0000)            /*!< PA[12] pin */
#define AFIO_EXTICR4_EXTI12_PB               ((uint16_t)0x0001)            /*!< PB[12] pin */
#define AFIO_EXTICR4_EXTI12_PC               ((uint16_t)0x0002)            /*!< PC[12] pin */

/* EXTI13 configuration */
#define AFIO_EXTICR4_EXTI13_PA               ((uint16_t)0x0000)            /*!< PA[13] pin */
#define AFIO_EXTICR4_EXTI13_PB               ((uint16_t)0x0010)            /*!< PB[13] pin */
#define AFIO_EXTICR4_EXTI13_PC               ((uint16_t)0x0020)            /*!< PC[13] pin */

/*!< EXTI14 configuration */
#define AFIO_EXTICR4_EXTI14_PA               ((uint16_t)0x0000)            /*!< PA[14] pin */
#define AFIO_EXTICR4_EXTI14_PB               ((uint16_t)0x0100)            /*!< PB[14] pin */
#define AFIO_EXTICR4_EXTI14_PC               ((uint16_t)0x0200)            /*!< PC[14] pin */

/*!< EXTI15 configuration */
#define AFIO_EXTICR4_EXTI15_PA               ((uint16_t)0x0000)            /*!< PA[15] pin */
#define AFIO_EXTICR4_EXTI15_PB               ((uint16_t)0x1000)            /*!< PB[15] pin */
#define AFIO_EXTICR4_EXTI15_PC               ((uint16_t)0x2000)            /*!< PC[15] pin */



/*------------------------------------------------------------------------------------------------------*/
/*---                                      Real-Time Clock (RTC)                                     ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for RTC_CRH register  ********************************/
#define  RTC_CRH_SECIE                       ((uint8_t)0x01)               /*!< Second Interrupt Enable */
#define  RTC_CRH_ALRIE                       ((uint8_t)0x02)               /*!< Alarm Interrupt Enable */
#define  RTC_CRH_OWIE                        ((uint8_t)0x04)               /*!< OverfloW Interrupt Enable */

/********************************  Bit definition for RTC_CRL register  ********************************/
#define  RTC_CRL_SECF                        ((uint8_t)0x01)               /*!< Second Flag */
#define  RTC_CRL_ALRF                        ((uint8_t)0x02)               /*!< Alarm Flag */
#define  RTC_CRL_OWF                         ((uint8_t)0x04)               /*!< OverfloW Flag */
#define  RTC_CRL_RSF                         ((uint8_t)0x08)               /*!< Registers Synchronized Flag */
#define  RTC_CRL_CNF                         ((uint8_t)0x10)               /*!< Configuration Flag */
#define  RTC_CRL_RTOFF                       ((uint8_t)0x20)               /*!< RTC operation OFF */

/********************************  Bit definition for RTC_PRLH register  ********************************/
#define  RTC_PRLH_PRL                        ((uint16_t)0x000F)            /*!< RTC Prescaler Reload Value High */

/********************************  Bit definition for RTC_PRLL register  ********************************/
#define  RTC_PRLL_PRL                        ((uint16_t)0xFFFF)            /*!< RTC Prescaler Reload Value Low */

/********************************  Bit definition for RTC_DIVH register  ********************************/
#define  RTC_DIVH_RTC_DIV                    ((uint16_t)0x000F)            /*!< RTC Clock Divider High */

/********************************  Bit definition for RTC_DIVL register  ********************************/
#define  RTC_DIVL_RTC_DIV                    ((uint16_t)0xFFFF)            /*!< RTC Clock Divider Low */

/********************************  Bit definition for RTC_CNTH register  ********************************/
#define  RTC_CNTH_RTC_CNT                    ((uint16_t)0xFFFF)            /*!< RTC Counter High */

/********************************  Bit definition for RTC_CNTL register  ********************************/
#define  RTC_CNTL_RTC_CNT                    ((uint16_t)0xFFFF)            /*!< RTC Counter Low */

/********************************  Bit definition for RTC_ALRH register  ********************************/
#define  RTC_ALRH_RTC_ALR                    ((uint16_t)0xFFFF)            /*!< RTC Alarm High */

/********************************  Bit definition for RTC_ALRL register  ********************************/
#define  RTC_ALRL_RTC_ALR                    ((uint16_t)0xFFFF)            /*!< RTC Alarm Low */






/*------------------------------------------------------------------------------------------------------*/
/*---                                     Backup Registers (BKP)                                     ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for BKP_RTCCR register  *********************************/
#define  BKP_RTCCR_CAL                       ((uint16_t)0x007F)     /*!< Calibration value */
#define  BKP_RTCCR_CCO                       ((uint16_t)0x0080)     /*!< Calibration Clock Output */
#define  BKP_RTCCR_ASOE                      ((uint16_t)0x0100)     /*!< Alarm or Second Output Enable */
#define  BKP_RTCCR_ASOS                      ((uint16_t)0x0200)     /*!< Alarm or Second Output Selection */

/********************************  Bit definition for BKP_CR register  *********************************/
#define  BKP_CR_TPE                          ((uint8_t)0x01)        /*!< TAMPER pin enable */
#define  BKP_CR_TPAL                         ((uint8_t)0x02)        /*!< TAMPER pin active level */

/********************************  Bit definition for BKP_CSR register  *********************************/
#define  BKP_CSR_CTE                         ((uint16_t)0x0001)     /*!< Clear Tamper event */
#define  BKP_CSR_CTI                         ((uint16_t)0x0002)     /*!< Clear Tamper Interrupt */
#define  BKP_CSR_TPIE                        ((uint16_t)0x0004)     /*!< TAMPER Pin interrupt enable */
#define  BKP_CSR_TEF                         ((uint16_t)0x0100)     /*!< Tamper Event Flag */
#define  BKP_CSR_TIF                         ((uint16_t)0x0200)     /*!< Tamper Interrupt Flag */





/*------------------------------------------------------------------------------------------------------*/
/*---                                     Window watchdog (WWDG)                                     ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for WWDG_CR register  ********************************/
#define  WWDG_CR_T                           ((uint8_t)0x7F)               /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T0                          ((uint8_t)0x01)               /*!< Bit 0 */
#define  WWDG_CR_T1                          ((uint8_t)0x02)               /*!< Bit 1 */
#define  WWDG_CR_T2                          ((uint8_t)0x04)               /*!< Bit 2 */
#define  WWDG_CR_T3                          ((uint8_t)0x08)               /*!< Bit 3 */
#define  WWDG_CR_T4                          ((uint8_t)0x10)               /*!< Bit 4 */
#define  WWDG_CR_T5                          ((uint8_t)0x20)               /*!< Bit 5 */
#define  WWDG_CR_T6                          ((uint8_t)0x40)               /*!< Bit 6 */

#define  WWDG_CR_WDGA                        ((uint8_t)0x80)               /*!< Activation bit */

/********************************  Bit definition for WWDG_CFR register  ********************************/
#define  WWDG_CFR_W                          ((uint16_t)0x007F)            /*!< W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W0                         ((uint16_t)0x0001)            /*!< Bit 0 */
#define  WWDG_CFR_W1                         ((uint16_t)0x0002)            /*!< Bit 1 */
#define  WWDG_CFR_W2                         ((uint16_t)0x0004)            /*!< Bit 2 */
#define  WWDG_CFR_W3                         ((uint16_t)0x0008)            /*!< Bit 3 */
#define  WWDG_CFR_W4                         ((uint16_t)0x0010)            /*!< Bit 4 */
#define  WWDG_CFR_W5                         ((uint16_t)0x0020)            /*!< Bit 5 */
#define  WWDG_CFR_W6                         ((uint16_t)0x0040)            /*!< Bit 6 */

#define  WWDG_CFR_WDGTB                      ((uint16_t)0x0180)            /*!< WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB0                     ((uint16_t)0x0080)            /*!< Bit 0 */
#define  WWDG_CFR_WDGTB1                     ((uint16_t)0x0100)            /*!< Bit 1 */

#define  WWDG_CFR_EWI                        ((uint16_t)0x0200)            /*!< Early Wakeup Interrupt */

/********************************  Bit definition for WWDG_SR register  ********************************/
#define  WWDG_SR_EWIF                        ((uint8_t)0x01)               /*!< Early Wakeup Interrupt Flag */



/*------------------------------------------------------------------------------------------------------*/
/*---                                  Independent watchdog (IWDG)                                   ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for IWDG_KR register  ********************************/
#define  IWDG_KR_KEY                         ((uint16_t)0xFFFF)            /*!< Key value (write only, read 0000h) */

/********************************  Bit definition for IWDG_PR register  ********************************/
#define  IWDG_PR_PR                          ((uint8_t)0x07)               /*!< PR[2:0] (Prescaler divider) */
#define  IWDG_PR_PR_0                        ((uint8_t)0x01)               /*!< Bit 0 */
#define  IWDG_PR_PR_1                        ((uint8_t)0x02)               /*!< Bit 1 */
#define  IWDG_PR_PR_2                        ((uint8_t)0x04)               /*!< Bit 2 */

/********************************  Bit definition for IWDG_RLR register  ********************************/
#define  IWDG_RLR_RL                         ((uint16_t)0x0FFF)            /*!< Watchdog counter reload value */

/********************************  Bit definition for IWDG_SR register  ********************************/
#define  IWDG_SR_PVU                         ((uint8_t)0x01)               /*!< Watchdog prescaler value update */
#define  IWDG_SR_RVU                         ((uint8_t)0x02)               /*!< Watchdog counter reload value update */


/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_SR register  ********************/
#define  ADC_SR_AWD                          ((uint8_t)0x01)               /*!< Analog watchdog flag */
#define  ADC_SR_EOC                          ((uint8_t)0x02)               /*!< End of conversion */
#define  ADC_SR_JEOC                         ((uint8_t)0x04)               /*!< Injected channel end of conversion */
#define  ADC_SR_JSTRT                        ((uint8_t)0x08)               /*!< Injected channel Start flag */
#define  ADC_SR_STRT                         ((uint8_t)0x10)               /*!< Regular channel Start flag */
#define  ADC_SR_EMP                          ((uint8_t)0x20)              /*!< XXXXX flag */
#define  ADC_SR_OVF                          ((uint8_t)0x40)              /*!< XXXXX flag */


/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       ((uint32_t)0x0000001F)        /*!< AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_CR1_AWDCH_1                     ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_CR1_AWDCH_2                     ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_CR1_AWDCH_3                     ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_CR1_AWDCH_4                     ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_CR1_EOCIE                       ((uint32_t)0x00000020)        /*!< Interrupt enable for EOC */
#define  ADC_CR1_AWDIE                       ((uint32_t)0x00000040)        /*!< Analog Watchdog interrupt enable */
#define  ADC_CR1_JEOCIE                      ((uint32_t)0x00000080)        /*!< Interrupt enable for injected channels */
#define  ADC_CR1_SCAN                        ((uint32_t)0x00000100)        /*!< Scan mode */
#define  ADC_CR1_AWDSGL                      ((uint32_t)0x00000200)        /*!< Enable the watchdog on a single channel in scan mode */
#define  ADC_CR1_JAUTO                       ((uint32_t)0x00000400)        /*!< Automatic injected group conversion */
#define  ADC_CR1_DISCEN                      ((uint32_t)0x00000800)        /*!< Discontinuous mode on regular channels */
#define  ADC_CR1_JDISCEN                     ((uint32_t)0x00001000)        /*!< Discontinuous mode on injected channels */

#define  ADC_CR1_DISCNUM                     ((uint32_t)0x0000E000)        /*!< DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CR1_DISCNUM_0                   ((uint32_t)0x00002000)        /*!< Bit 0 */
#define  ADC_CR1_DISCNUM_1                   ((uint32_t)0x00004000)        /*!< Bit 1 */
#define  ADC_CR1_DISCNUM_2                   ((uint32_t)0x00008000)        /*!< Bit 2 */

#define  ADC_CR1_JAWDEN                      ((uint32_t)0x00400000)        /*!< Analog watchdog enable on injected channels */
#define  ADC_CR1_AWDEN                       ((uint32_t)0x00800000)        /*!< Analog watchdog enable on regular channels */


/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        ((uint32_t)0x00000001)        /*!< A/D Converter ON / OFF */
#define  ADC_CR2_CONT                        ((uint32_t)0x00000002)        /*!< Continuous Conversion */
#define  ADC_CR2_CAL                         ((uint32_t)0x00000004)        /*!< A/D Calibration */
#define  ADC_CR2_RSTCAL                      ((uint32_t)0x00000008)        /*!< Reset Calibration */
#define  ADC_CR2_DMAEN                       ((uint32_t)0x00000100)        /*!< XXXXX */
#define  ADC_CR2_JDMAEN                      ((uint32_t)0x00000200)        /*!< XXXXX */
#define  ADC_CR2_JEXTSYNC                    ((uint32_t)0x00000400)        /*!< XXXXX */
#define  ADC_CR2_ALIGN                       ((uint32_t)0x00000800)        /*!< Data Alignment */

#define  ADC_CR2_JEXTSEL                     ((uint32_t)0x00007000)        /*!< JEXTSEL[2:0] bits (External event select for injected group) */
#define  ADC_CR2_JEXTSEL_0                   ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  ADC_CR2_JEXTSEL_1                   ((uint32_t)0x00002000)        /*!< Bit 1 */
#define  ADC_CR2_JEXTSEL_2                   ((uint32_t)0x00004000)        /*!< Bit 2 */

#define  ADC_CR2_JEXTTRIG                    ((uint32_t)0x00008000)        /*!< External Trigger Conversion mode for injected channels */
#define  ADC_CR2_EXTSYNC                     ((uint32_t)0x00010000)        /*!< XXXXX */

#define  ADC_CR2_EXTSEL                      ((uint32_t)0x000E0000)        /*!< EXTSEL[2:0] bits (External Event Select for regular group) */
#define  ADC_CR2_EXTSEL_0                    ((uint32_t)0x00020000)        /*!< Bit 0 */
#define  ADC_CR2_EXTSEL_1                    ((uint32_t)0x00040000)        /*!< Bit 1 */
#define  ADC_CR2_EXTSEL_2                    ((uint32_t)0x00080000)        /*!< Bit 2 */

#define  ADC_CR2_EXTTRIG                     ((uint32_t)0x00100000)        /*!< External Trigger Conversion mode for regular channels */
#define  ADC_CR2_JSWSTART                    ((uint32_t)0x00200000)        /*!< Start Conversion of injected channels */
#define  ADC_CR2_SWSTART                     ((uint32_t)0x00400000)        /*!< Start Conversion of regular channels */
#define  ADC_CR2_TSVREFE                     ((uint32_t)0x00800000)        /*!< Temperature Sensor and VREFINT Enable */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  ADC_SMPR1_SMP10                     ((uint32_t)0x00000007)        /*!< SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPR1_SMP10_0                   ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP10_1                   ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP10_2                   ((uint32_t)0x00000004)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP11                     ((uint32_t)0x00000038)        /*!< SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPR1_SMP11_0                   ((uint32_t)0x00000008)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP11_1                   ((uint32_t)0x00000010)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP11_2                   ((uint32_t)0x00000020)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP12                     ((uint32_t)0x000001C0)        /*!< SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPR1_SMP12_0                   ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP12_1                   ((uint32_t)0x00000080)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP12_2                   ((uint32_t)0x00000100)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP13                     ((uint32_t)0x00000E00)        /*!< SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPR1_SMP13_0                   ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP13_1                   ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP13_2                   ((uint32_t)0x00000800)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP14                     ((uint32_t)0x00007000)        /*!< SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPR1_SMP14_0                   ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP14_1                   ((uint32_t)0x00002000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP14_2                   ((uint32_t)0x00004000)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP15                     ((uint32_t)0x00038000)        /*!< SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPR1_SMP15_0                   ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP15_1                   ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP15_2                   ((uint32_t)0x00020000)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP16                     ((uint32_t)0x001C0000)        /*!< SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPR1_SMP16_0                   ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP16_1                   ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP16_2                   ((uint32_t)0x00100000)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP17                     ((uint32_t)0x00E00000)        /*!< SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPR1_SMP17_0                   ((uint32_t)0x00200000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP17_1                   ((uint32_t)0x00400000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP17_2                   ((uint32_t)0x00800000)        /*!< Bit 2 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  ADC_SMPR2_SMP0                      ((uint32_t)0x00000007)        /*!< SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPR2_SMP0_0                    ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP0_1                    ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP0_2                    ((uint32_t)0x00000004)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP1                      ((uint32_t)0x00000038)        /*!< SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPR2_SMP1_0                    ((uint32_t)0x00000008)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP1_1                    ((uint32_t)0x00000010)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP1_2                    ((uint32_t)0x00000020)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP2                      ((uint32_t)0x000001C0)        /*!< SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMPR2_SMP2_0                    ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP2_1                    ((uint32_t)0x00000080)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP2_2                    ((uint32_t)0x00000100)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP3                      ((uint32_t)0x00000E00)        /*!< SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMPR2_SMP3_0                    ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP3_1                    ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP3_2                    ((uint32_t)0x00000800)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP4                      ((uint32_t)0x00007000)        /*!< SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMPR2_SMP4_0                    ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP4_1                    ((uint32_t)0x00002000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP4_2                    ((uint32_t)0x00004000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP5                      ((uint32_t)0x00038000)        /*!< SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMPR2_SMP5_0                    ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP5_1                    ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP5_2                    ((uint32_t)0x00020000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP6                      ((uint32_t)0x001C0000)        /*!< SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMPR2_SMP6_0                    ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP6_1                    ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP6_2                    ((uint32_t)0x00100000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP7                      ((uint32_t)0x00E00000)        /*!< SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMPR2_SMP7_0                    ((uint32_t)0x00200000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP7_1                    ((uint32_t)0x00400000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP7_2                    ((uint32_t)0x00800000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP8                      ((uint32_t)0x07000000)        /*!< SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMPR2_SMP8_0                    ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP8_1                    ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP8_2                    ((uint32_t)0x04000000)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP9                      ((uint32_t)0x38000000)        /*!< SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMPR2_SMP9_0                    ((uint32_t)0x08000000)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP9_1                    ((uint32_t)0x10000000)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP9_2                    ((uint32_t)0x20000000)        /*!< Bit 2 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define  ADC_JOFR1_JOFFSET1                  ((uint16_t)0x0FFF)            /*!< Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  ((uint16_t)0x0FFF)            /*!< Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  ((uint16_t)0x0FFF)            /*!< Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  ((uint16_t)0x0FFF)            /*!< Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          ((uint16_t)0x0FFF)            /*!< Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          ((uint16_t)0x0FFF)            /*!< Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define  ADC_SQR1_SQ13                       ((uint32_t)0x0000001F)        /*!< SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_SQR1_SQ13_0                     ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_SQR1_SQ13_1                     ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_SQR1_SQ13_2                     ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_SQR1_SQ13_3                     ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_SQR1_SQ13_4                     ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_SQR1_SQ14                       ((uint32_t)0x000003E0)        /*!< SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_SQR1_SQ14_0                     ((uint32_t)0x00000020)        /*!< Bit 0 */
#define  ADC_SQR1_SQ14_1                     ((uint32_t)0x00000040)        /*!< Bit 1 */
#define  ADC_SQR1_SQ14_2                     ((uint32_t)0x00000080)        /*!< Bit 2 */
#define  ADC_SQR1_SQ14_3                     ((uint32_t)0x00000100)        /*!< Bit 3 */
#define  ADC_SQR1_SQ14_4                     ((uint32_t)0x00000200)        /*!< Bit 4 */

#define  ADC_SQR1_SQ15                       ((uint32_t)0x00007C00)        /*!< SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_SQR1_SQ15_0                     ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  ADC_SQR1_SQ15_1                     ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  ADC_SQR1_SQ15_2                     ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  ADC_SQR1_SQ15_3                     ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  ADC_SQR1_SQ15_4                     ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  ADC_SQR1_SQ16                       ((uint32_t)0x000F8000)        /*!< SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_SQR1_SQ16_0                     ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_SQR1_SQ16_1                     ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_SQR1_SQ16_2                     ((uint32_t)0x00020000)        /*!< Bit 2 */
#define  ADC_SQR1_SQ16_3                     ((uint32_t)0x00040000)        /*!< Bit 3 */
#define  ADC_SQR1_SQ16_4                     ((uint32_t)0x00080000)        /*!< Bit 4 */

#define  ADC_SQR1_L                          ((uint32_t)0x00F00000)        /*!< L[3:0] bits (Regular channel sequence length) */
#define  ADC_SQR1_L_0                        ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  ADC_SQR1_L_1                        ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  ADC_SQR1_L_2                        ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  ADC_SQR1_L_3                        ((uint32_t)0x00800000)        /*!< Bit 3 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define  ADC_SQR2_SQ7                        ((uint32_t)0x0000001F)        /*!< SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_SQR2_SQ7_0                      ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_SQR2_SQ7_1                      ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_SQR2_SQ7_2                      ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_SQR2_SQ7_3                      ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_SQR2_SQ7_4                      ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_SQR2_SQ8                        ((uint32_t)0x000003E0)        /*!< SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_SQR2_SQ8_0                      ((uint32_t)0x00000020)        /*!< Bit 0 */
#define  ADC_SQR2_SQ8_1                      ((uint32_t)0x00000040)        /*!< Bit 1 */
#define  ADC_SQR2_SQ8_2                      ((uint32_t)0x00000080)        /*!< Bit 2 */
#define  ADC_SQR2_SQ8_3                      ((uint32_t)0x00000100)        /*!< Bit 3 */
#define  ADC_SQR2_SQ8_4                      ((uint32_t)0x00000200)        /*!< Bit 4 */

#define  ADC_SQR2_SQ9                        ((uint32_t)0x00007C00)        /*!< SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_SQR2_SQ9_0                      ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  ADC_SQR2_SQ9_1                      ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  ADC_SQR2_SQ9_2                      ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  ADC_SQR2_SQ9_3                      ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  ADC_SQR2_SQ9_4                      ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  ADC_SQR2_SQ10                       ((uint32_t)0x000F8000)        /*!< SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_SQR2_SQ10_0                     ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_SQR2_SQ10_1                     ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_SQR2_SQ10_2                     ((uint32_t)0x00020000)        /*!< Bit 2 */
#define  ADC_SQR2_SQ10_3                     ((uint32_t)0x00040000)        /*!< Bit 3 */
#define  ADC_SQR2_SQ10_4                     ((uint32_t)0x00080000)        /*!< Bit 4 */

#define  ADC_SQR2_SQ11                       ((uint32_t)0x01F00000)        /*!< SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_SQR2_SQ11_0                     ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  ADC_SQR2_SQ11_1                     ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  ADC_SQR2_SQ11_2                     ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  ADC_SQR2_SQ11_3                     ((uint32_t)0x00800000)        /*!< Bit 3 */
#define  ADC_SQR2_SQ11_4                     ((uint32_t)0x01000000)        /*!< Bit 4 */

#define  ADC_SQR2_SQ12                       ((uint32_t)0x3E000000)        /*!< SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_SQR2_SQ12_0                     ((uint32_t)0x02000000)        /*!< Bit 0 */
#define  ADC_SQR2_SQ12_1                     ((uint32_t)0x04000000)        /*!< Bit 1 */
#define  ADC_SQR2_SQ12_2                     ((uint32_t)0x08000000)        /*!< Bit 2 */
#define  ADC_SQR2_SQ12_3                     ((uint32_t)0x10000000)        /*!< Bit 3 */
#define  ADC_SQR2_SQ12_4                     ((uint32_t)0x20000000)        /*!< Bit 4 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define  ADC_SQR3_SQ1                        ((uint32_t)0x0000001F)        /*!< SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_SQR3_SQ1_0                      ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_SQR3_SQ1_1                      ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_SQR3_SQ1_2                      ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_SQR3_SQ1_3                      ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_SQR3_SQ1_4                      ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_SQR3_SQ2                        ((uint32_t)0x000003E0)        /*!< SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_SQR3_SQ2_0                      ((uint32_t)0x00000020)        /*!< Bit 0 */
#define  ADC_SQR3_SQ2_1                      ((uint32_t)0x00000040)        /*!< Bit 1 */
#define  ADC_SQR3_SQ2_2                      ((uint32_t)0x00000080)        /*!< Bit 2 */
#define  ADC_SQR3_SQ2_3                      ((uint32_t)0x00000100)        /*!< Bit 3 */
#define  ADC_SQR3_SQ2_4                      ((uint32_t)0x00000200)        /*!< Bit 4 */

#define  ADC_SQR3_SQ3                        ((uint32_t)0x00007C00)        /*!< SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_SQR3_SQ3_0                      ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  ADC_SQR3_SQ3_1                      ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  ADC_SQR3_SQ3_2                      ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  ADC_SQR3_SQ3_3                      ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  ADC_SQR3_SQ3_4                      ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  ADC_SQR3_SQ4                        ((uint32_t)0x000F8000)        /*!< SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_SQR3_SQ4_0                      ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_SQR3_SQ4_1                      ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_SQR3_SQ4_2                      ((uint32_t)0x00020000)        /*!< Bit 2 */
#define  ADC_SQR3_SQ4_3                      ((uint32_t)0x00040000)        /*!< Bit 3 */
#define  ADC_SQR3_SQ4_4                      ((uint32_t)0x00080000)        /*!< Bit 4 */

#define  ADC_SQR3_SQ5                        ((uint32_t)0x01F00000)        /*!< SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_SQR3_SQ5_0                      ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  ADC_SQR3_SQ5_1                      ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  ADC_SQR3_SQ5_2                      ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  ADC_SQR3_SQ5_3                      ((uint32_t)0x00800000)        /*!< Bit 3 */
#define  ADC_SQR3_SQ5_4                      ((uint32_t)0x01000000)        /*!< Bit 4 */

#define  ADC_SQR3_SQ6                        ((uint32_t)0x3E000000)        /*!< SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_SQR3_SQ6_0                      ((uint32_t)0x02000000)        /*!< Bit 0 */
#define  ADC_SQR3_SQ6_1                      ((uint32_t)0x04000000)        /*!< Bit 1 */
#define  ADC_SQR3_SQ6_2                      ((uint32_t)0x08000000)        /*!< Bit 2 */
#define  ADC_SQR3_SQ6_3                      ((uint32_t)0x10000000)        /*!< Bit 3 */
#define  ADC_SQR3_SQ6_4                      ((uint32_t)0x20000000)        /*!< Bit 4 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define  ADC_JSQR_JSQ1                       ((uint32_t)0x0000001F)        /*!< JSQ1[4:0] bits (1st conversion in injected sequence) */
#define  ADC_JSQR_JSQ1_0                     ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_JSQR_JSQ1_1                     ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_JSQR_JSQ1_2                     ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_JSQR_JSQ1_3                     ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_JSQR_JSQ1_4                     ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_JSQR_JSQ2                       ((uint32_t)0x000003E0)        /*!< JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQR_JSQ2_0                     ((uint32_t)0x00000020)        /*!< Bit 0 */
#define  ADC_JSQR_JSQ2_1                     ((uint32_t)0x00000040)        /*!< Bit 1 */
#define  ADC_JSQR_JSQ2_2                     ((uint32_t)0x00000080)        /*!< Bit 2 */
#define  ADC_JSQR_JSQ2_3                     ((uint32_t)0x00000100)        /*!< Bit 3 */
#define  ADC_JSQR_JSQ2_4                     ((uint32_t)0x00000200)        /*!< Bit 4 */

#define  ADC_JSQR_JSQ3                       ((uint32_t)0x00007C00)        /*!< JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQR_JSQ3_0                     ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  ADC_JSQR_JSQ3_1                     ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  ADC_JSQR_JSQ3_2                     ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  ADC_JSQR_JSQ3_3                     ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  ADC_JSQR_JSQ3_4                     ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  ADC_JSQR_JSQ4                       ((uint32_t)0x000F8000)        /*!< JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQR_JSQ4_0                     ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_JSQR_JSQ4_1                     ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_JSQR_JSQ4_2                     ((uint32_t)0x00020000)        /*!< Bit 2 */
#define  ADC_JSQR_JSQ4_3                     ((uint32_t)0x00040000)        /*!< Bit 3 */
#define  ADC_JSQR_JSQ4_4                     ((uint32_t)0x00080000)        /*!< Bit 4 */

#define  ADC_JSQR_JL                         ((uint32_t)0x00300000)        /*!< JL[1:0] bits (Injected Sequence length) */
#define  ADC_JSQR_JL_0                       ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  ADC_JSQR_JL_1                       ((uint32_t)0x00200000)        /*!< Bit 1 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define  ADC_JDR1_JDATA                      ((uint16_t)0xFFFF)            /*!< Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      ((uint16_t)0xFFFF)            /*!< Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      ((uint16_t)0xFFFF)            /*!< Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      ((uint16_t)0xFFFF)            /*!< Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         ((uint32_t)0x0000FFFF)        /*!< Regular data */


/********************  Bit definition for ADC_CR3 register  ********************/
#define  ADC_CR3_ADVMODE                     ((uint32_t)0x00000003)        /*!< ADVMODE[1:0] bits */
#define  ADC_CR3_SAMCHN                      ((uint32_t)0x0000000C)        /*!< SAMCHN[1:0] bits */
#define  ADC_CR3_VREFCFG                     ((uint32_t)0x00000030)        /*!< VREFCFG[1:0] bits */
#define  ADC_CR3_12BIT                       ((uint32_t)0x00000040)        /*!< 12-bit enable */
#define  ADC_CR3_PRS                         ((uint32_t)0x0000FF00)        /*!< PRS[7:0] bits (Prescaler to the apb clock input) */
#define  ADC_CR3_OVFIE                       ((uint32_t)0x00010000)        /*!< ADC fifo overflow interrupt enable */
#define  ADC_CR3_EMPIE                       ((uint32_t)0x00020000)        /*!< ADC fifo empty interrupt enable */

/*******************  Bit definition for ADC_JDMAR register  *******************/
#define  ADC_JDMAR_JDATA                     ((uint16_t)0xFFFF)            /*!< Injected data */





/*------------------------------------------------------------------------------------------------------*/
/*---                                         ISO7816 (ISO)                                          ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for ISO_FIFOSR register  *********************/
#define ISO_FIFOSR_FULL           (0x1U << 0)
#define ISO_FIFOSR_EMPTY          (0x1U << 1)


/*------------------------------------------------------------------------------------------------------*/
/*---                                             CACHE                                              ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for CACHE_CR register  ****************************/
#define CACHE_CR_LATENCY_Msk      (0xFU)
#define CACHE_CR_LATENCY_0WS      (0x0U)
#define CACHE_CR_LATENCY_1WS      (0x1U)
#define CACHE_CR_LATENCY_2WS      (0x2U)
#define CACHE_CR_LATENCY_3WS      (0x3U)
#define CACHE_CR_LATENCY_4WS      (0x4U)
#define CACHE_CR_LATENCY_5WS      (0x5U)
#define CACHE_CR_LATENCY_6WS      (0x6U)
#define CACHE_CR_LATENCY_7WS      (0x7U)
#define CACHE_CR_LATENCY_8WS      (0x8U)
#define CACHE_CR_LATENCY_9WS      (0x9U)
#define CACHE_CR_LATENCY_10WS     (0xAU)
#define CACHE_CR_LATENCY_11WS     (0xBU)
#define CACHE_CR_LATENCY_12WS     (0xCU)
#define CACHE_CR_LATENCY_13WS     (0xDU)
#define CACHE_CR_LATENCY_14WS     (0xEU)
#define CACHE_CR_LATENCY_15WS     (0xFU)

#define CACHE_CR_PREFEN_Msk       (0x3U << 4)
#define CACHE_CR_PREFEN_OFF       (0x0U << 4)
#define CACHE_CR_PREFEN_ON        (0x1U << 4)

#define CACHE_CR_HIFREQ           (0x1U << 8)

#define CACHE_CR_CHEEN            (0x3U << 24)


/*------------------------------------------------------------------------------------------------------*/
/*---                                              FMC                                               ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for FMC_CON register  ********************************/
#define FMC_CON_OP_Msk             (0x1FU)        /*!< OP field mask bit */

#define FMC_CON_WREN               (0x1U << 6)
#define FMC_CON_WR                 (0x1U << 7)

#define FMC_CON_SETHLDCNT_Msk      (0x7FU << 8)     /*!< SETHLDCNT field mask bit */

/********************************  Bit definition for FMC_CRCON register  ********************************/
#define FMC_CRCON_CRCEN            (0x1U << 0)
#define FMC_CRCON_CRCF             (0x1U << 1)
#define FMC_CRCON_PAUSE            (0x1U << 2)
#define FMC_CRCON_SLOWRD           (0x1U << 3)
#define FMC_CRCON_CRCFIE           (0x1U << 8)

#define FMC_CRCON_PERIOD_Pos       (12U)
#define FMC_CRCON_PERIOD_Msk       (0xFU << FMC_CRCON_PERIOD_Pos)

#define FMC_CRCON_CRCLEN_Pos       (16U)
#define FMC_CRCON_CRCLEN_Msk       (0x3FFU << FMC_CRCON_CRCLEN_Pos)

/********************************  Bit definition for FMC_STAT register  *******************************/
#define FMC_STAT_ERR               (0x1U << 2)





/*------------------------------------------------------------------------------------------------------*/
/*---                                             ANCTL                                              ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for ANCTL_BGCR2 register  ***************************/
#define ANCTL_BGCR2_TEMPOUTEN           (0x1U << 1)

/********************************  Bit definition for ANCTL_MHSIENR register  ***************************/
#define ANCTL_MHSIENR_MHSION            (0x1U << 0)    /*!< Internal 8M high-speed clock enable */

/********************************  Bit definition for ANCTL_MHSISR register  ***************************/
#define ANCTL_MHSISR_MHSIRDY            (0x1U << 0)    /*!< Internal 8M high-speed clock ready flag */

/********************************  Bit definition for ANCTL_FHSIENR register  ***************************/
#define ANCTL_FHSIENR_FHSION            (0x1U << 0)    /*!< Internal 48M high-speed clock enable */

/********************************  Bit definition for ANCTL_FHSISR register  ***************************/
#define ANCTL_FHSISR_FHSIRDY            (0x1U << 0)    /*!< Internal 48M high-speed clock ready flag */

/********************************  Bit definition for ANCTL_LSIENR register  ***************************/
#define ANCTL_LSIENR_LSION              (0x1U << 0)   /*!< Internal low-speed oscillator enable */

/********************************  Bit definition for ANCTL_LSISR register  ***************************/
#define ANCTL_LSISR_LSIRDY              (0x1U << 0)   /*!< Internal low-speed oscillator ready flag */

/********************************  Bit definition for ANCTL_HSECR0 register  ***************************/
#define ANCTL_HSECR0_HSEON              (0x1U << 0)
#define ANCTL_HSECR0_BYPASS             (0x1U << 1)

/********************************  Bit definition for ANCTL_HSECR1 register  ***************************/
#define ANCTL_HSECR1_PADOEN             (0x1U << 1)

/********************************  Bit definition for ANCTL_HSESR register  ***************************/
#define ANCTL_HSESR_HSERDY              (0x1U << 0)   /*!< External high-speed clock ready flag */

/********************************  Bit definition for ANCTL_PLLCR register  ***************************/
#define ANCTL_PLLCR_PLLMUL_Msk          (0x3U << 6)
#define ANCTL_PLLCR_PLLMUL_24           (0x0U << 6)
#define ANCTL_PLLCR_PLLMUL_20           (0x1U << 6)
#define ANCTL_PLLCR_PLLMUL_16           (0x2U << 6)
#define ANCTL_PLLCR_PLLMUL_12           (0x3U << 6)

/********************************  Bit definition for ANCTL_PLLENR register  ***************************/
#define ANCTL_PLLENR_PLLON              (0x1U << 0)    /*!< PLL enable */

/********************************  Bit definition for ANCTL_PLLSR register  ***************************/
#define ANCTL_PLLSR_PLLRDY_Msk          (0x3U)

/********************************  Bit definition for ANCTL_PVDCR register  ***************************/
#define ANCTL_PVDCR_PLS_Msk             (0x7U)
#define ANCTL_PVDCR_PLS_LEV0            (0x0U)
#define ANCTL_PVDCR_PLS_LEV1            (0x1U)
#define ANCTL_PVDCR_PLS_LEV2            (0x2U)
#define ANCTL_PVDCR_PLS_LEV3            (0x3U)
#define ANCTL_PVDCR_PLS_LEV4            (0x4U)
#define ANCTL_PVDCR_PLS_LEV5            (0x5U)
#define ANCTL_PVDCR_PLS_LEV6            (0x6U)
#define ANCTL_PVDCR_PLS_LEV7            (0x7U)

/********************************  Bit definition for ANCTL_PVDENR register  ***************************/
#define ANCTL_PVDENR_PVDE               (0x1U << 0)    /*!< PVD enable */

/********************************  Bit definition for ANCTL_SARENR register  ***************************/
#define ANCTL_SARENR_SAREN              (0x1U << 0)    /*!< SAR ADC enable */

/********************************  Bit definition for ANCTL_USBPCR register  ***************************/
#define ANCTL_USBPCR_USBPEN             (0x1U << 0)    /*!< USB PHY enable */
#define ANCTL_USBPCR_DPPUEN             (0x1U << 1)
#define ANCTL_USBPCR_HIGHRESEN          (0x1U << 2)
#define ANCTL_USBPCR_DMSTEN             (0x1U << 3)
#define ANCTL_USBPCR_DPSTEN             (0x1U << 4)

/********************************  Bit definition for ANCTL_CMPACR register  ***************************/
#define ANCTL_CMPACR_PSEL_Msk           (0xFU << 0)
#define ANCTL_CMPACR_NSEL_Msk           (0xFU << 4)
#define ANCTL_CMPACR_CMPAEN             (0x1U << 8)

/********************************  Bit definition for ANCTL_CMPBCR register  ***************************/
#define ANCTL_CMPBCR_PSEL_Msk           (0xFU << 0)
#define ANCTL_CMPBCR_NSEL_Msk           (0xFU << 4)
#define ANCTL_CMPBCR_CMPBEN             (0x1U << 8)

/********************************  Bit definition for ANCTL_ISR register  ***************************/
#define ANCTL_ISR_MHSIIS                (0x1U << 0)
#define ANCTL_ISR_FHSIIS                (0x1U << 1)
#define ANCTL_ISR_LSIIS                 (0x1U << 2)
#define ANCTL_ISR_HSEIS                 (0x1U << 3)
#define ANCTL_ISR_LSEIS                 (0x1U << 4)
#define ANCTL_ISR_PLLIS                 (0x1U << 5)
#define ANCTL_ISR_DCSSIS                (0x1U << 7)

/********************************  Bit definition for ANCTL_IER register  ***************************/
#define ANCTL_IER_MHSIIE                (0x1U << 0)
#define ANCTL_IER_FHSIIE                (0x1U << 1)
#define ANCTL_IER_LSIIE                 (0x1U << 2)
#define ANCTL_IER_HSEIE                 (0x1U << 3)
#define ANCTL_IER_LSEIE                 (0x1U << 4)
#define ANCTL_IER_PLLIE                 (0x1U << 5)

/********************************  Bit definition for ANCTL_ICR register  ***************************/
#define ANCTL_ICR_MHSIIC                (0x1U << 0)
#define ANCTL_ICR_FHSIIC                (0x1U << 1)
#define ANCTL_ICR_LSIIC                 (0x1U << 2)
#define ANCTL_ICR_HSEIC                 (0x1U << 3)
#define ANCTL_ICR_LSEIC                 (0x1U << 4)
#define ANCTL_ICR_PLLIC                 (0x1U << 5)
#define ANCTL_ICR_DCSSIC                (0x1U << 7)

/********************************  Bit definition for ANCTL_DCSSENR register  ***************************/
#define ANCTL_DCSSENR_DCSSON            (0x1U << 0)

/********************************  Bit definition for ANCTL_DCSSCR register  ***************************/
#define ANCTL_DCSSCR_FREQCNT_Msk        (0xFFFU)



/*------------------------------------------------------------------------------------------------------*/
/*---                                 Reset and Clock Control (RCC)                                  ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for RCC_PLLPRE register  ***************************/
#define RCC_PLLPRE_DIVEN          (0x1U << 0)

#define RCC_PLLPRE_RATIO_Msk      (0xFU << 1)
#define RCC_PLLPRE_RATIO_2        (0x0U << 1)
#define RCC_PLLPRE_RATIO_3        (0x1U << 1)
#define RCC_PLLPRE_RATIO_4        (0x2U << 1)
#define RCC_PLLPRE_RATIO_5        (0x3U << 1)
#define RCC_PLLPRE_RATIO_6        (0x4U << 1)
#define RCC_PLLPRE_RATIO_7        (0x5U << 1)
#define RCC_PLLPRE_RATIO_8        (0x6U << 1)
#define RCC_PLLPRE_RATIO_9        (0x7U << 1)
#define RCC_PLLPRE_RATIO_10       (0x8U << 1)
#define RCC_PLLPRE_RATIO_11       (0x9U << 1)
#define RCC_PLLPRE_RATIO_12       (0xAU << 1)
#define RCC_PLLPRE_RATIO_13       (0xBU << 1)
#define RCC_PLLPRE_RATIO_14       (0xCU << 1)
#define RCC_PLLPRE_RATIO_15       (0xDU << 1)
#define RCC_PLLPRE_RATIO_16       (0xEU << 1)

#define RCC_PLLPRE_SRCEN          (0x1U << 5)

/********************************  Bit definition for RCC_PLLSRC register  ***************************/
#define RCC_PLLSRC_MHSI           (0x0U)
#define RCC_PLLSRC_HSE            (0x1U)

/********************************  Bit definition for RCC_MAINCLKSRC register  ***************************/
#define RCC_MAINCLKSRC_MHSI       (0x0U)
#define RCC_MAINCLKSRC_FHSI       (0x1U)
#define RCC_MAINCLKSRC_PLLCLK     (0x2U)
#define RCC_MAINCLKSRC_HSE        (0x3U)

/********************************  Bit definition for RCC_MAINCLKUEN register  ***************************/
#define RCC_MAINCLKUEN_ENA        (0x1U)

/********************************  Bit definition for RCC_USBPRE register  ***************************/
#define RCC_USBPRE_DIVEN          (0x1U << 0)

#define RCC_USBPRE_RATIO_Msk      (0x3U << 1)
#define RCC_USBPRE_RATIO_1_5      (0x2U << 1)
#define RCC_USBPRE_RATIO_2        (0x0U << 1)
#define RCC_USBPRE_RATIO_3        (0x1U << 1)

#define RCC_USBPRE_SRCEN          (0x1U << 3)

/********************************  Bit definition for RCC_AHBPRE register  ***************************/
#define RCC_AHBPRE_DIVEN          (0x1U << 0)

#define RCC_AHBPRE_RATIO_Msk      (0x3FU << 1)
#define RCC_AHBPRE_RATIO_2        (0x0U << 1)
#define RCC_AHBPRE_RATIO_3        (0x1U << 1)
#define RCC_AHBPRE_RATIO_4        (0x2U << 1)
#define RCC_AHBPRE_RATIO_5        (0x3U << 1)
#define RCC_AHBPRE_RATIO_6        (0x4U << 1)
#define RCC_AHBPRE_RATIO_7        (0x5U << 1)
#define RCC_AHBPRE_RATIO_8        (0x6U << 1)
#define RCC_AHBPRE_RATIO_9        (0x7U << 1)
#define RCC_AHBPRE_RATIO_10       (0x8U << 1)
#define RCC_AHBPRE_RATIO_11       (0x9U << 1)
#define RCC_AHBPRE_RATIO_12       (0xAU << 1)
#define RCC_AHBPRE_RATIO_13       (0xBU << 1)
#define RCC_AHBPRE_RATIO_14       (0xCU << 1)
#define RCC_AHBPRE_RATIO_15       (0xDU << 1)
#define RCC_AHBPRE_RATIO_16       (0xEU << 1)
#define RCC_AHBPRE_RATIO_17       (0xFU << 1)
#define RCC_AHBPRE_RATIO_18       (0x10U << 1)
#define RCC_AHBPRE_RATIO_19       (0x11U << 1)
#define RCC_AHBPRE_RATIO_20       (0x12U << 1)
#define RCC_AHBPRE_RATIO_21       (0x13U << 1)
#define RCC_AHBPRE_RATIO_22       (0x14U << 1)
#define RCC_AHBPRE_RATIO_23       (0x15U << 1)
#define RCC_AHBPRE_RATIO_24       (0x16U << 1)
#define RCC_AHBPRE_RATIO_25       (0x17U << 1)
#define RCC_AHBPRE_RATIO_26       (0x18U << 1)
#define RCC_AHBPRE_RATIO_27       (0x19U << 1)
#define RCC_AHBPRE_RATIO_28       (0x1AU << 1)
#define RCC_AHBPRE_RATIO_29       (0x1BU << 1)
#define RCC_AHBPRE_RATIO_30       (0x1CU << 1)
#define RCC_AHBPRE_RATIO_31       (0x1DU << 1)
#define RCC_AHBPRE_RATIO_32       (0x1EU << 1)
#define RCC_AHBPRE_RATIO_33       (0x1FU << 1)
#define RCC_AHBPRE_RATIO_34       (0x20U << 1)
#define RCC_AHBPRE_RATIO_35       (0x21U << 1)
#define RCC_AHBPRE_RATIO_36       (0x22U << 1)
#define RCC_AHBPRE_RATIO_37       (0x23U << 1)
#define RCC_AHBPRE_RATIO_38       (0x24U << 1)
#define RCC_AHBPRE_RATIO_39       (0x25U << 1)
#define RCC_AHBPRE_RATIO_40       (0x26U << 1)
#define RCC_AHBPRE_RATIO_41       (0x27U << 1)
#define RCC_AHBPRE_RATIO_42       (0x28U << 1)
#define RCC_AHBPRE_RATIO_43       (0x29U << 1)
#define RCC_AHBPRE_RATIO_44       (0x2AU << 1)
#define RCC_AHBPRE_RATIO_45       (0x2BU << 1)
#define RCC_AHBPRE_RATIO_46       (0x2CU << 1)
#define RCC_AHBPRE_RATIO_47       (0x2DU << 1)
#define RCC_AHBPRE_RATIO_48       (0x2EU << 1)
#define RCC_AHBPRE_RATIO_49       (0x2FU << 1)
#define RCC_AHBPRE_RATIO_50       (0x30U << 1)
#define RCC_AHBPRE_RATIO_51       (0x31U << 1)
#define RCC_AHBPRE_RATIO_52       (0x32U << 1)
#define RCC_AHBPRE_RATIO_53       (0x33U << 1)
#define RCC_AHBPRE_RATIO_54       (0x34U << 1)
#define RCC_AHBPRE_RATIO_55       (0x35U << 1)
#define RCC_AHBPRE_RATIO_56       (0x36U << 1)
#define RCC_AHBPRE_RATIO_57       (0x37U << 1)
#define RCC_AHBPRE_RATIO_58       (0x38U << 1)
#define RCC_AHBPRE_RATIO_59       (0x39U << 1)
#define RCC_AHBPRE_RATIO_60       (0x3AU << 1)
#define RCC_AHBPRE_RATIO_61       (0x3BU << 1)
#define RCC_AHBPRE_RATIO_62       (0x3CU << 1)
#define RCC_AHBPRE_RATIO_63       (0x3DU << 1)
#define RCC_AHBPRE_RATIO_64       (0x3EU << 1)

/********************************  Bit definition for RCC_APB1PRE register  ***************************/
#define RCC_APB1PRE_DIVEN          (0x1U << 0)

#define RCC_APB1PRE_RATIO_Msk      (0x3FU << 1)
#define RCC_APB1PRE_RATIO_2        (0x0U << 1)
#define RCC_APB1PRE_RATIO_3        (0x1U << 1)
#define RCC_APB1PRE_RATIO_4        (0x2U << 1)
#define RCC_APB1PRE_RATIO_5        (0x3U << 1)
#define RCC_APB1PRE_RATIO_6        (0x4U << 1)
#define RCC_APB1PRE_RATIO_7        (0x5U << 1)
#define RCC_APB1PRE_RATIO_8        (0x6U << 1)
#define RCC_APB1PRE_RATIO_9        (0x7U << 1)
#define RCC_APB1PRE_RATIO_10       (0x8U << 1)
#define RCC_APB1PRE_RATIO_11       (0x9U << 1)
#define RCC_APB1PRE_RATIO_12       (0xAU << 1)
#define RCC_APB1PRE_RATIO_13       (0xBU << 1)
#define RCC_APB1PRE_RATIO_14       (0xCU << 1)
#define RCC_APB1PRE_RATIO_15       (0xDU << 1)
#define RCC_APB1PRE_RATIO_16       (0xEU << 1)
#define RCC_APB1PRE_RATIO_17       (0xFU << 1)
#define RCC_APB1PRE_RATIO_18       (0x10U << 1)
#define RCC_APB1PRE_RATIO_19       (0x11U << 1)
#define RCC_APB1PRE_RATIO_20       (0x12U << 1)
#define RCC_APB1PRE_RATIO_21       (0x13U << 1)
#define RCC_APB1PRE_RATIO_22       (0x14U << 1)
#define RCC_APB1PRE_RATIO_23       (0x15U << 1)
#define RCC_APB1PRE_RATIO_24       (0x16U << 1)
#define RCC_APB1PRE_RATIO_25       (0x17U << 1)
#define RCC_APB1PRE_RATIO_26       (0x18U << 1)
#define RCC_APB1PRE_RATIO_27       (0x19U << 1)
#define RCC_APB1PRE_RATIO_28       (0x1AU << 1)
#define RCC_APB1PRE_RATIO_29       (0x1BU << 1)
#define RCC_APB1PRE_RATIO_30       (0x1CU << 1)
#define RCC_APB1PRE_RATIO_31       (0x1DU << 1)
#define RCC_APB1PRE_RATIO_32       (0x1EU << 1)
#define RCC_APB1PRE_RATIO_33       (0x1FU << 1)
#define RCC_APB1PRE_RATIO_34       (0x20U << 1)
#define RCC_APB1PRE_RATIO_35       (0x21U << 1)
#define RCC_APB1PRE_RATIO_36       (0x22U << 1)
#define RCC_APB1PRE_RATIO_37       (0x23U << 1)
#define RCC_APB1PRE_RATIO_38       (0x24U << 1)
#define RCC_APB1PRE_RATIO_39       (0x25U << 1)
#define RCC_APB1PRE_RATIO_40       (0x26U << 1)
#define RCC_APB1PRE_RATIO_41       (0x27U << 1)
#define RCC_APB1PRE_RATIO_42       (0x28U << 1)
#define RCC_APB1PRE_RATIO_43       (0x29U << 1)
#define RCC_APB1PRE_RATIO_44       (0x2AU << 1)
#define RCC_APB1PRE_RATIO_45       (0x2BU << 1)
#define RCC_APB1PRE_RATIO_46       (0x2CU << 1)
#define RCC_APB1PRE_RATIO_47       (0x2DU << 1)
#define RCC_APB1PRE_RATIO_48       (0x2EU << 1)
#define RCC_APB1PRE_RATIO_49       (0x2FU << 1)
#define RCC_APB1PRE_RATIO_50       (0x30U << 1)
#define RCC_APB1PRE_RATIO_51       (0x31U << 1)
#define RCC_APB1PRE_RATIO_52       (0x32U << 1)
#define RCC_APB1PRE_RATIO_53       (0x33U << 1)
#define RCC_APB1PRE_RATIO_54       (0x34U << 1)
#define RCC_APB1PRE_RATIO_55       (0x35U << 1)
#define RCC_APB1PRE_RATIO_56       (0x36U << 1)
#define RCC_APB1PRE_RATIO_57       (0x37U << 1)
#define RCC_APB1PRE_RATIO_58       (0x38U << 1)
#define RCC_APB1PRE_RATIO_59       (0x39U << 1)
#define RCC_APB1PRE_RATIO_60       (0x3AU << 1)
#define RCC_APB1PRE_RATIO_61       (0x3BU << 1)
#define RCC_APB1PRE_RATIO_62       (0x3CU << 1)
#define RCC_APB1PRE_RATIO_63       (0x3DU << 1)
#define RCC_APB1PRE_RATIO_64       (0x3EU << 1)

#define RCC_APB1PRE_SRCEN          (0x1U << 7)

/********************************  Bit definition for RCC_APB2PRE register  ***************************/
#define RCC_APB2PRE_DIVEN          (0x1U << 0)

#define RCC_APB2PRE_RATIO_Msk      (0x3FU << 1)
#define RCC_APB2PRE_RATIO_2        (0x0U << 1)
#define RCC_APB2PRE_RATIO_3        (0x1U << 1)
#define RCC_APB2PRE_RATIO_4        (0x2U << 1)
#define RCC_APB2PRE_RATIO_5        (0x3U << 1)
#define RCC_APB2PRE_RATIO_6        (0x4U << 1)
#define RCC_APB2PRE_RATIO_7        (0x5U << 1)
#define RCC_APB2PRE_RATIO_8        (0x6U << 1)
#define RCC_APB2PRE_RATIO_9        (0x7U << 1)
#define RCC_APB2PRE_RATIO_10       (0x8U << 1)
#define RCC_APB2PRE_RATIO_11       (0x9U << 1)
#define RCC_APB2PRE_RATIO_12       (0xAU << 1)
#define RCC_APB2PRE_RATIO_13       (0xBU << 1)
#define RCC_APB2PRE_RATIO_14       (0xCU << 1)
#define RCC_APB2PRE_RATIO_15       (0xDU << 1)
#define RCC_APB2PRE_RATIO_16       (0xEU << 1)
#define RCC_APB2PRE_RATIO_17       (0xFU << 1)
#define RCC_APB2PRE_RATIO_18       (0x10U << 1)
#define RCC_APB2PRE_RATIO_19       (0x11U << 1)
#define RCC_APB2PRE_RATIO_20       (0x12U << 1)
#define RCC_APB2PRE_RATIO_21       (0x13U << 1)
#define RCC_APB2PRE_RATIO_22       (0x14U << 1)
#define RCC_APB2PRE_RATIO_23       (0x15U << 1)
#define RCC_APB2PRE_RATIO_24       (0x16U << 1)
#define RCC_APB2PRE_RATIO_25       (0x17U << 1)
#define RCC_APB2PRE_RATIO_26       (0x18U << 1)
#define RCC_APB2PRE_RATIO_27       (0x19U << 1)
#define RCC_APB2PRE_RATIO_28       (0x1AU << 1)
#define RCC_APB2PRE_RATIO_29       (0x1BU << 1)
#define RCC_APB2PRE_RATIO_30       (0x1CU << 1)
#define RCC_APB2PRE_RATIO_31       (0x1DU << 1)
#define RCC_APB2PRE_RATIO_32       (0x1EU << 1)
#define RCC_APB2PRE_RATIO_33       (0x1FU << 1)
#define RCC_APB2PRE_RATIO_34       (0x20U << 1)
#define RCC_APB2PRE_RATIO_35       (0x21U << 1)
#define RCC_APB2PRE_RATIO_36       (0x22U << 1)
#define RCC_APB2PRE_RATIO_37       (0x23U << 1)
#define RCC_APB2PRE_RATIO_38       (0x24U << 1)
#define RCC_APB2PRE_RATIO_39       (0x25U << 1)
#define RCC_APB2PRE_RATIO_40       (0x26U << 1)
#define RCC_APB2PRE_RATIO_41       (0x27U << 1)
#define RCC_APB2PRE_RATIO_42       (0x28U << 1)
#define RCC_APB2PRE_RATIO_43       (0x29U << 1)
#define RCC_APB2PRE_RATIO_44       (0x2AU << 1)
#define RCC_APB2PRE_RATIO_45       (0x2BU << 1)
#define RCC_APB2PRE_RATIO_46       (0x2CU << 1)
#define RCC_APB2PRE_RATIO_47       (0x2DU << 1)
#define RCC_APB2PRE_RATIO_48       (0x2EU << 1)
#define RCC_APB2PRE_RATIO_49       (0x2FU << 1)
#define RCC_APB2PRE_RATIO_50       (0x30U << 1)
#define RCC_APB2PRE_RATIO_51       (0x31U << 1)
#define RCC_APB2PRE_RATIO_52       (0x32U << 1)
#define RCC_APB2PRE_RATIO_53       (0x33U << 1)
#define RCC_APB2PRE_RATIO_54       (0x34U << 1)
#define RCC_APB2PRE_RATIO_55       (0x35U << 1)
#define RCC_APB2PRE_RATIO_56       (0x36U << 1)
#define RCC_APB2PRE_RATIO_57       (0x37U << 1)
#define RCC_APB2PRE_RATIO_58       (0x38U << 1)
#define RCC_APB2PRE_RATIO_59       (0x39U << 1)
#define RCC_APB2PRE_RATIO_60       (0x3AU << 1)
#define RCC_APB2PRE_RATIO_61       (0x3BU << 1)
#define RCC_APB2PRE_RATIO_62       (0x3CU << 1)
#define RCC_APB2PRE_RATIO_63       (0x3DU << 1)
#define RCC_APB2PRE_RATIO_64       (0x3EU << 1)

#define RCC_APB2PRE_SRCEN          (0x1U << 7)

/********************************  Bit definition for RCC_MCLKPRE register  ***************************/
#define RCC_MCLKPRE_DIVEN          (0x1U << 0)

#define RCC_MCLKPRE_RATIO_Msk      (0x3FU << 1)
#define RCC_MCLKPRE_RATIO_2        (0x0U << 1)
#define RCC_MCLKPRE_RATIO_3        (0x1U << 1)
#define RCC_MCLKPRE_RATIO_4        (0x2U << 1)
#define RCC_MCLKPRE_RATIO_5        (0x3U << 1)
#define RCC_MCLKPRE_RATIO_6        (0x4U << 1)
#define RCC_MCLKPRE_RATIO_7        (0x5U << 1)
#define RCC_MCLKPRE_RATIO_8        (0x6U << 1)
#define RCC_MCLKPRE_RATIO_9        (0x7U << 1)
#define RCC_MCLKPRE_RATIO_10       (0x8U << 1)
#define RCC_MCLKPRE_RATIO_11       (0x9U << 1)
#define RCC_MCLKPRE_RATIO_12       (0xAU << 1)
#define RCC_MCLKPRE_RATIO_13       (0xBU << 1)
#define RCC_MCLKPRE_RATIO_14       (0xCU << 1)
#define RCC_MCLKPRE_RATIO_15       (0xDU << 1)
#define RCC_MCLKPRE_RATIO_16       (0xEU << 1)
#define RCC_MCLKPRE_RATIO_17       (0xFU << 1)
#define RCC_MCLKPRE_RATIO_18       (0x10U << 1)
#define RCC_MCLKPRE_RATIO_19       (0x11U << 1)
#define RCC_MCLKPRE_RATIO_20       (0x12U << 1)
#define RCC_MCLKPRE_RATIO_21       (0x13U << 1)
#define RCC_MCLKPRE_RATIO_22       (0x14U << 1)
#define RCC_MCLKPRE_RATIO_23       (0x15U << 1)
#define RCC_MCLKPRE_RATIO_24       (0x16U << 1)
#define RCC_MCLKPRE_RATIO_25       (0x17U << 1)
#define RCC_MCLKPRE_RATIO_26       (0x18U << 1)
#define RCC_MCLKPRE_RATIO_27       (0x19U << 1)
#define RCC_MCLKPRE_RATIO_28       (0x1AU << 1)
#define RCC_MCLKPRE_RATIO_29       (0x1BU << 1)
#define RCC_MCLKPRE_RATIO_30       (0x1CU << 1)
#define RCC_MCLKPRE_RATIO_31       (0x1DU << 1)
#define RCC_MCLKPRE_RATIO_32       (0x1EU << 1)
#define RCC_MCLKPRE_RATIO_33       (0x1FU << 1)
#define RCC_MCLKPRE_RATIO_34       (0x20U << 1)
#define RCC_MCLKPRE_RATIO_35       (0x21U << 1)
#define RCC_MCLKPRE_RATIO_36       (0x22U << 1)
#define RCC_MCLKPRE_RATIO_37       (0x23U << 1)
#define RCC_MCLKPRE_RATIO_38       (0x24U << 1)
#define RCC_MCLKPRE_RATIO_39       (0x25U << 1)
#define RCC_MCLKPRE_RATIO_40       (0x26U << 1)
#define RCC_MCLKPRE_RATIO_41       (0x27U << 1)
#define RCC_MCLKPRE_RATIO_42       (0x28U << 1)
#define RCC_MCLKPRE_RATIO_43       (0x29U << 1)
#define RCC_MCLKPRE_RATIO_44       (0x2AU << 1)
#define RCC_MCLKPRE_RATIO_45       (0x2BU << 1)
#define RCC_MCLKPRE_RATIO_46       (0x2CU << 1)
#define RCC_MCLKPRE_RATIO_47       (0x2DU << 1)
#define RCC_MCLKPRE_RATIO_48       (0x2EU << 1)
#define RCC_MCLKPRE_RATIO_49       (0x2FU << 1)
#define RCC_MCLKPRE_RATIO_50       (0x30U << 1)
#define RCC_MCLKPRE_RATIO_51       (0x31U << 1)
#define RCC_MCLKPRE_RATIO_52       (0x32U << 1)
#define RCC_MCLKPRE_RATIO_53       (0x33U << 1)
#define RCC_MCLKPRE_RATIO_54       (0x34U << 1)
#define RCC_MCLKPRE_RATIO_55       (0x35U << 1)
#define RCC_MCLKPRE_RATIO_56       (0x36U << 1)
#define RCC_MCLKPRE_RATIO_57       (0x37U << 1)
#define RCC_MCLKPRE_RATIO_58       (0x38U << 1)
#define RCC_MCLKPRE_RATIO_59       (0x39U << 1)
#define RCC_MCLKPRE_RATIO_60       (0x3AU << 1)
#define RCC_MCLKPRE_RATIO_61       (0x3BU << 1)
#define RCC_MCLKPRE_RATIO_62       (0x3CU << 1)
#define RCC_MCLKPRE_RATIO_63       (0x3DU << 1)
#define RCC_MCLKPRE_RATIO_64       (0x3EU << 1)

#define RCC_MCLKPRE_SRCEN          (0x1U << 7)

/********************************  Bit definition for RCC_I2SPRE register  ***************************/
#define RCC_I2SPRE_DIVEN           (0x1U << 0)

#define RCC_I2SPRE_RATIO_Msk       (0x1FFU << 1)
#define RCC_I2SPRE_RATIO_2         (0x0U << 1)
#define RCC_I2SPRE_RATIO_3         (0x1U << 1)
#define RCC_I2SPRE_RATIO_4         (0x2U << 1)
#define RCC_I2SPRE_RATIO_5         (0x3U << 1)
#define RCC_I2SPRE_RATIO_6         (0x4U << 1)
#define RCC_I2SPRE_RATIO_7         (0x5U << 1)
#define RCC_I2SPRE_RATIO_8         (0x6U << 1)
#define RCC_I2SPRE_RATIO_9         (0x7U << 1)
#define RCC_I2SPRE_RATIO_10        (0x8U << 1)
#define RCC_I2SPRE_RATIO_11        (0x9U << 1)
#define RCC_I2SPRE_RATIO_12        (0xAU << 1)
#define RCC_I2SPRE_RATIO_13        (0xBU << 1)
#define RCC_I2SPRE_RATIO_14        (0xCU << 1)
#define RCC_I2SPRE_RATIO_15        (0xDU << 1)
#define RCC_I2SPRE_RATIO_16        (0xEU << 1)
#define RCC_I2SPRE_RATIO_17        (0xFU << 1)
#define RCC_I2SPRE_RATIO_18        (0x10U << 1)
#define RCC_I2SPRE_RATIO_19        (0x11U << 1)
#define RCC_I2SPRE_RATIO_20        (0x12U << 1)
#define RCC_I2SPRE_RATIO_21        (0x13U << 1)
#define RCC_I2SPRE_RATIO_22        (0x14U << 1)
#define RCC_I2SPRE_RATIO_23        (0x15U << 1)
#define RCC_I2SPRE_RATIO_24        (0x16U << 1)
#define RCC_I2SPRE_RATIO_25        (0x17U << 1)
#define RCC_I2SPRE_RATIO_26        (0x18U << 1)
#define RCC_I2SPRE_RATIO_27        (0x19U << 1)
#define RCC_I2SPRE_RATIO_28        (0x1AU << 1)
#define RCC_I2SPRE_RATIO_29        (0x1BU << 1)
#define RCC_I2SPRE_RATIO_30        (0x1CU << 1)
#define RCC_I2SPRE_RATIO_31        (0x1DU << 1)
#define RCC_I2SPRE_RATIO_32        (0x1EU << 1)
#define RCC_I2SPRE_RATIO_33        (0x1FU << 1)
#define RCC_I2SPRE_RATIO_34        (0x20U << 1)
#define RCC_I2SPRE_RATIO_35        (0x21U << 1)
#define RCC_I2SPRE_RATIO_36        (0x22U << 1)
#define RCC_I2SPRE_RATIO_37        (0x23U << 1)
#define RCC_I2SPRE_RATIO_38        (0x24U << 1)
#define RCC_I2SPRE_RATIO_39        (0x25U << 1)
#define RCC_I2SPRE_RATIO_40        (0x26U << 1)
#define RCC_I2SPRE_RATIO_41        (0x27U << 1)
#define RCC_I2SPRE_RATIO_42        (0x28U << 1)
#define RCC_I2SPRE_RATIO_43        (0x29U << 1)
#define RCC_I2SPRE_RATIO_44        (0x2AU << 1)
#define RCC_I2SPRE_RATIO_45        (0x2BU << 1)
#define RCC_I2SPRE_RATIO_46        (0x2CU << 1)
#define RCC_I2SPRE_RATIO_47        (0x2DU << 1)
#define RCC_I2SPRE_RATIO_48        (0x2EU << 1)
#define RCC_I2SPRE_RATIO_49        (0x2FU << 1)
#define RCC_I2SPRE_RATIO_50        (0x30U << 1)
#define RCC_I2SPRE_RATIO_51        (0x31U << 1)
#define RCC_I2SPRE_RATIO_52        (0x32U << 1)
#define RCC_I2SPRE_RATIO_53        (0x33U << 1)
#define RCC_I2SPRE_RATIO_54        (0x34U << 1)
#define RCC_I2SPRE_RATIO_55        (0x35U << 1)
#define RCC_I2SPRE_RATIO_56        (0x36U << 1)
#define RCC_I2SPRE_RATIO_57        (0x37U << 1)
#define RCC_I2SPRE_RATIO_58        (0x38U << 1)
#define RCC_I2SPRE_RATIO_59        (0x39U << 1)
#define RCC_I2SPRE_RATIO_60        (0x3AU << 1)
#define RCC_I2SPRE_RATIO_61        (0x3BU << 1)
#define RCC_I2SPRE_RATIO_62        (0x3CU << 1)
#define RCC_I2SPRE_RATIO_63        (0x3DU << 1)
#define RCC_I2SPRE_RATIO_64        (0x3EU << 1)
#define RCC_I2SPRE_RATIO_65        (0x3FU << 1)
#define RCC_I2SPRE_RATIO_66        (0x40U << 1)
#define RCC_I2SPRE_RATIO_67        (0x41U << 1)
#define RCC_I2SPRE_RATIO_68        (0x42U << 1)
#define RCC_I2SPRE_RATIO_69        (0x43U << 1)
#define RCC_I2SPRE_RATIO_70        (0x44U << 1)
#define RCC_I2SPRE_RATIO_71        (0x45U << 1)
#define RCC_I2SPRE_RATIO_72        (0x46U << 1)
#define RCC_I2SPRE_RATIO_73        (0x47U << 1)
#define RCC_I2SPRE_RATIO_74        (0x48U << 1)
#define RCC_I2SPRE_RATIO_75        (0x49U << 1)
#define RCC_I2SPRE_RATIO_76        (0x4AU << 1)
#define RCC_I2SPRE_RATIO_77        (0x4BU << 1)
#define RCC_I2SPRE_RATIO_78        (0x4CU << 1)
#define RCC_I2SPRE_RATIO_79        (0x4DU << 1)
#define RCC_I2SPRE_RATIO_80        (0x4EU << 1)
#define RCC_I2SPRE_RATIO_81        (0x4FU << 1)
#define RCC_I2SPRE_RATIO_82        (0x50U << 1)
#define RCC_I2SPRE_RATIO_83        (0x51U << 1)
#define RCC_I2SPRE_RATIO_84        (0x52U << 1)
#define RCC_I2SPRE_RATIO_85        (0x53U << 1)
#define RCC_I2SPRE_RATIO_86        (0x54U << 1)
#define RCC_I2SPRE_RATIO_87        (0x55U << 1)
#define RCC_I2SPRE_RATIO_88        (0x56U << 1)
#define RCC_I2SPRE_RATIO_89        (0x57U << 1)
#define RCC_I2SPRE_RATIO_90        (0x58U << 1)
#define RCC_I2SPRE_RATIO_91        (0x59U << 1)
#define RCC_I2SPRE_RATIO_92        (0x5AU << 1)
#define RCC_I2SPRE_RATIO_93        (0x5BU << 1)
#define RCC_I2SPRE_RATIO_94        (0x5CU << 1)
#define RCC_I2SPRE_RATIO_95        (0x5DU << 1)
#define RCC_I2SPRE_RATIO_96        (0x5EU << 1)
#define RCC_I2SPRE_RATIO_97        (0x5FU << 1)
#define RCC_I2SPRE_RATIO_98        (0x60U << 1)
#define RCC_I2SPRE_RATIO_99        (0x61U << 1)
#define RCC_I2SPRE_RATIO_100       (0x62U << 1)
#define RCC_I2SPRE_RATIO_101       (0x63U << 1)
#define RCC_I2SPRE_RATIO_102       (0x64U << 1)
#define RCC_I2SPRE_RATIO_103       (0x65U << 1)
#define RCC_I2SPRE_RATIO_104       (0x66U << 1)
#define RCC_I2SPRE_RATIO_105       (0x67U << 1)
#define RCC_I2SPRE_RATIO_106       (0x68U << 1)
#define RCC_I2SPRE_RATIO_107       (0x69U << 1)
#define RCC_I2SPRE_RATIO_108       (0x6AU << 1)
#define RCC_I2SPRE_RATIO_109       (0x6BU << 1)
#define RCC_I2SPRE_RATIO_110       (0x6CU << 1)
#define RCC_I2SPRE_RATIO_111       (0x6DU << 1)
#define RCC_I2SPRE_RATIO_112       (0x6EU << 1)
#define RCC_I2SPRE_RATIO_113       (0x6FU << 1)
#define RCC_I2SPRE_RATIO_114       (0x70U << 1)
#define RCC_I2SPRE_RATIO_115       (0x71U << 1)
#define RCC_I2SPRE_RATIO_116       (0x72U << 1)
#define RCC_I2SPRE_RATIO_117       (0x73U << 1)
#define RCC_I2SPRE_RATIO_118       (0x74U << 1)
#define RCC_I2SPRE_RATIO_119       (0x75U << 1)
#define RCC_I2SPRE_RATIO_120       (0x76U << 1)
#define RCC_I2SPRE_RATIO_121       (0x77U << 1)
#define RCC_I2SPRE_RATIO_122       (0x78U << 1)
#define RCC_I2SPRE_RATIO_123       (0x79U << 1)
#define RCC_I2SPRE_RATIO_124       (0x7AU << 1)
#define RCC_I2SPRE_RATIO_125       (0x7BU << 1)
#define RCC_I2SPRE_RATIO_126       (0x7CU << 1)
#define RCC_I2SPRE_RATIO_127       (0x7DU << 1)
#define RCC_I2SPRE_RATIO_128       (0x7EU << 1)
#define RCC_I2SPRE_RATIO_129       (0x7FU << 1)
#define RCC_I2SPRE_RATIO_130       (0x80U << 1)
#define RCC_I2SPRE_RATIO_131       (0x81U << 1)
#define RCC_I2SPRE_RATIO_132       (0x82U << 1)
#define RCC_I2SPRE_RATIO_133       (0x83U << 1)
#define RCC_I2SPRE_RATIO_134       (0x84U << 1)
#define RCC_I2SPRE_RATIO_135       (0x85U << 1)
#define RCC_I2SPRE_RATIO_136       (0x86U << 1)
#define RCC_I2SPRE_RATIO_137       (0x87U << 1)
#define RCC_I2SPRE_RATIO_138       (0x88U << 1)
#define RCC_I2SPRE_RATIO_139       (0x89U << 1)
#define RCC_I2SPRE_RATIO_140       (0x8AU << 1)
#define RCC_I2SPRE_RATIO_141       (0x8BU << 1)
#define RCC_I2SPRE_RATIO_142       (0x8CU << 1)
#define RCC_I2SPRE_RATIO_143       (0x8DU << 1)
#define RCC_I2SPRE_RATIO_144       (0x8EU << 1)
#define RCC_I2SPRE_RATIO_145       (0x8FU << 1)
#define RCC_I2SPRE_RATIO_146       (0x90U << 1)
#define RCC_I2SPRE_RATIO_147       (0x91U << 1)
#define RCC_I2SPRE_RATIO_148       (0x92U << 1)
#define RCC_I2SPRE_RATIO_149       (0x93U << 1)
#define RCC_I2SPRE_RATIO_150       (0x94U << 1)
#define RCC_I2SPRE_RATIO_151       (0x95U << 1)
#define RCC_I2SPRE_RATIO_152       (0x96U << 1)
#define RCC_I2SPRE_RATIO_153       (0x97U << 1)
#define RCC_I2SPRE_RATIO_154       (0x98U << 1)
#define RCC_I2SPRE_RATIO_155       (0x99U << 1)
#define RCC_I2SPRE_RATIO_156       (0x9AU << 1)
#define RCC_I2SPRE_RATIO_157       (0x9BU << 1)
#define RCC_I2SPRE_RATIO_158       (0x9CU << 1)
#define RCC_I2SPRE_RATIO_159       (0x9DU << 1)
#define RCC_I2SPRE_RATIO_160       (0x9EU << 1)
#define RCC_I2SPRE_RATIO_161       (0x9FU << 1)
#define RCC_I2SPRE_RATIO_162       (0xA0U << 1)
#define RCC_I2SPRE_RATIO_163       (0xA1U << 1)
#define RCC_I2SPRE_RATIO_164       (0xA2U << 1)
#define RCC_I2SPRE_RATIO_165       (0xA3U << 1)
#define RCC_I2SPRE_RATIO_166       (0xA4U << 1)
#define RCC_I2SPRE_RATIO_167       (0xA5U << 1)
#define RCC_I2SPRE_RATIO_168       (0xA6U << 1)
#define RCC_I2SPRE_RATIO_169       (0xA7U << 1)
#define RCC_I2SPRE_RATIO_170       (0xA8U << 1)
#define RCC_I2SPRE_RATIO_171       (0xA9U << 1)
#define RCC_I2SPRE_RATIO_172       (0xAAU << 1)
#define RCC_I2SPRE_RATIO_173       (0xABU << 1)
#define RCC_I2SPRE_RATIO_174       (0xACU << 1)
#define RCC_I2SPRE_RATIO_175       (0xADU << 1)
#define RCC_I2SPRE_RATIO_176       (0xAEU << 1)
#define RCC_I2SPRE_RATIO_177       (0xAFU << 1)
#define RCC_I2SPRE_RATIO_178       (0xB0U << 1)
#define RCC_I2SPRE_RATIO_179       (0xB1U << 1)
#define RCC_I2SPRE_RATIO_180       (0xB2U << 1)
#define RCC_I2SPRE_RATIO_181       (0xB3U << 1)
#define RCC_I2SPRE_RATIO_182       (0xB4U << 1)
#define RCC_I2SPRE_RATIO_183       (0xB5U << 1)
#define RCC_I2SPRE_RATIO_184       (0xB6U << 1)
#define RCC_I2SPRE_RATIO_185       (0xB7U << 1)
#define RCC_I2SPRE_RATIO_186       (0xB8U << 1)
#define RCC_I2SPRE_RATIO_187       (0xB9U << 1)
#define RCC_I2SPRE_RATIO_188       (0xBAU << 1)
#define RCC_I2SPRE_RATIO_189       (0xBBU << 1)
#define RCC_I2SPRE_RATIO_190       (0xBCU << 1)
#define RCC_I2SPRE_RATIO_191       (0xBDU << 1)
#define RCC_I2SPRE_RATIO_192       (0xBEU << 1)
#define RCC_I2SPRE_RATIO_193       (0xBFU << 1)
#define RCC_I2SPRE_RATIO_194       (0xC0U << 1)
#define RCC_I2SPRE_RATIO_195       (0xC1U << 1)
#define RCC_I2SPRE_RATIO_196       (0xC2U << 1)
#define RCC_I2SPRE_RATIO_197       (0xC3U << 1)
#define RCC_I2SPRE_RATIO_198       (0xC4U << 1)
#define RCC_I2SPRE_RATIO_199       (0xC5U << 1)
#define RCC_I2SPRE_RATIO_200       (0xC6U << 1)
#define RCC_I2SPRE_RATIO_201       (0xC7U << 1)
#define RCC_I2SPRE_RATIO_202       (0xC8U << 1)
#define RCC_I2SPRE_RATIO_203       (0xC9U << 1)
#define RCC_I2SPRE_RATIO_204       (0xCAU << 1)
#define RCC_I2SPRE_RATIO_205       (0xCBU << 1)
#define RCC_I2SPRE_RATIO_206       (0xCCU << 1)
#define RCC_I2SPRE_RATIO_207       (0xCDU << 1)
#define RCC_I2SPRE_RATIO_208       (0xCEU << 1)
#define RCC_I2SPRE_RATIO_209       (0xCFU << 1)
#define RCC_I2SPRE_RATIO_210       (0xD0U << 1)
#define RCC_I2SPRE_RATIO_211       (0xD1U << 1)
#define RCC_I2SPRE_RATIO_212       (0xD2U << 1)
#define RCC_I2SPRE_RATIO_213       (0xD3U << 1)
#define RCC_I2SPRE_RATIO_214       (0xD4U << 1)
#define RCC_I2SPRE_RATIO_215       (0xD5U << 1)
#define RCC_I2SPRE_RATIO_216       (0xD6U << 1)
#define RCC_I2SPRE_RATIO_217       (0xD7U << 1)
#define RCC_I2SPRE_RATIO_218       (0xD8U << 1)
#define RCC_I2SPRE_RATIO_219       (0xD9U << 1)
#define RCC_I2SPRE_RATIO_220       (0xDAU << 1)
#define RCC_I2SPRE_RATIO_221       (0xDBU << 1)
#define RCC_I2SPRE_RATIO_222       (0xDCU << 1)
#define RCC_I2SPRE_RATIO_223       (0xDDU << 1)
#define RCC_I2SPRE_RATIO_224       (0xDEU << 1)
#define RCC_I2SPRE_RATIO_225       (0xDFU << 1)
#define RCC_I2SPRE_RATIO_226       (0xE0U << 1)
#define RCC_I2SPRE_RATIO_227       (0xE1U << 1)
#define RCC_I2SPRE_RATIO_228       (0xE2U << 1)
#define RCC_I2SPRE_RATIO_229       (0xE3U << 1)
#define RCC_I2SPRE_RATIO_230       (0xE4U << 1)
#define RCC_I2SPRE_RATIO_231       (0xE5U << 1)
#define RCC_I2SPRE_RATIO_232       (0xE6U << 1)
#define RCC_I2SPRE_RATIO_233       (0xE7U << 1)
#define RCC_I2SPRE_RATIO_234       (0xE8U << 1)
#define RCC_I2SPRE_RATIO_235       (0xE9U << 1)
#define RCC_I2SPRE_RATIO_236       (0xEAU << 1)
#define RCC_I2SPRE_RATIO_237       (0xEBU << 1)
#define RCC_I2SPRE_RATIO_238       (0xECU << 1)
#define RCC_I2SPRE_RATIO_239       (0xEDU << 1)
#define RCC_I2SPRE_RATIO_240       (0xEEU << 1)
#define RCC_I2SPRE_RATIO_241       (0xEFU << 1)
#define RCC_I2SPRE_RATIO_242       (0xF0U << 1)
#define RCC_I2SPRE_RATIO_243       (0xF1U << 1)
#define RCC_I2SPRE_RATIO_244       (0xF2U << 1)
#define RCC_I2SPRE_RATIO_245       (0xF3U << 1)
#define RCC_I2SPRE_RATIO_246       (0xF4U << 1)
#define RCC_I2SPRE_RATIO_247       (0xF5U << 1)
#define RCC_I2SPRE_RATIO_248       (0xF6U << 1)
#define RCC_I2SPRE_RATIO_249       (0xF7U << 1)
#define RCC_I2SPRE_RATIO_250       (0xF8U << 1)
#define RCC_I2SPRE_RATIO_251       (0xF9U << 1)
#define RCC_I2SPRE_RATIO_252       (0xFAU << 1)
#define RCC_I2SPRE_RATIO_253       (0xFBU << 1)
#define RCC_I2SPRE_RATIO_254       (0xFCU << 1)
#define RCC_I2SPRE_RATIO_255       (0xFDU << 1)
#define RCC_I2SPRE_RATIO_256       (0xFEU << 1)
#define RCC_I2SPRE_RATIO_257       (0xFFU << 1)
#define RCC_I2SPRE_RATIO_258       (0x100U << 1)
#define RCC_I2SPRE_RATIO_259       (0x101U << 1)
#define RCC_I2SPRE_RATIO_260       (0x102U << 1)
#define RCC_I2SPRE_RATIO_261       (0x103U << 1)
#define RCC_I2SPRE_RATIO_262       (0x104U << 1)
#define RCC_I2SPRE_RATIO_263       (0x105U << 1)
#define RCC_I2SPRE_RATIO_264       (0x106U << 1)
#define RCC_I2SPRE_RATIO_265       (0x107U << 1)
#define RCC_I2SPRE_RATIO_266       (0x108U << 1)
#define RCC_I2SPRE_RATIO_267       (0x109U << 1)
#define RCC_I2SPRE_RATIO_268       (0x10AU << 1)
#define RCC_I2SPRE_RATIO_269       (0x10BU << 1)
#define RCC_I2SPRE_RATIO_270       (0x10CU << 1)
#define RCC_I2SPRE_RATIO_271       (0x10DU << 1)
#define RCC_I2SPRE_RATIO_272       (0x10EU << 1)
#define RCC_I2SPRE_RATIO_273       (0x10FU << 1)
#define RCC_I2SPRE_RATIO_274       (0x110U << 1)
#define RCC_I2SPRE_RATIO_275       (0x111U << 1)
#define RCC_I2SPRE_RATIO_276       (0x112U << 1)
#define RCC_I2SPRE_RATIO_277       (0x113U << 1)
#define RCC_I2SPRE_RATIO_278       (0x114U << 1)
#define RCC_I2SPRE_RATIO_279       (0x115U << 1)
#define RCC_I2SPRE_RATIO_280       (0x116U << 1)
#define RCC_I2SPRE_RATIO_281       (0x117U << 1)
#define RCC_I2SPRE_RATIO_282       (0x118U << 1)
#define RCC_I2SPRE_RATIO_283       (0x119U << 1)
#define RCC_I2SPRE_RATIO_284       (0x11AU << 1)
#define RCC_I2SPRE_RATIO_285       (0x11BU << 1)
#define RCC_I2SPRE_RATIO_286       (0x11CU << 1)
#define RCC_I2SPRE_RATIO_287       (0x11DU << 1)
#define RCC_I2SPRE_RATIO_288       (0x11EU << 1)
#define RCC_I2SPRE_RATIO_289       (0x11FU << 1)
#define RCC_I2SPRE_RATIO_290       (0x120U << 1)
#define RCC_I2SPRE_RATIO_291       (0x121U << 1)
#define RCC_I2SPRE_RATIO_292       (0x122U << 1)
#define RCC_I2SPRE_RATIO_293       (0x123U << 1)
#define RCC_I2SPRE_RATIO_294       (0x124U << 1)
#define RCC_I2SPRE_RATIO_295       (0x125U << 1)
#define RCC_I2SPRE_RATIO_296       (0x126U << 1)
#define RCC_I2SPRE_RATIO_297       (0x127U << 1)
#define RCC_I2SPRE_RATIO_298       (0x128U << 1)
#define RCC_I2SPRE_RATIO_299       (0x129U << 1)
#define RCC_I2SPRE_RATIO_300       (0x12AU << 1)
#define RCC_I2SPRE_RATIO_301       (0x12BU << 1)
#define RCC_I2SPRE_RATIO_302       (0x12CU << 1)
#define RCC_I2SPRE_RATIO_303       (0x12DU << 1)
#define RCC_I2SPRE_RATIO_304       (0x12EU << 1)
#define RCC_I2SPRE_RATIO_305       (0x12FU << 1)
#define RCC_I2SPRE_RATIO_306       (0x130U << 1)
#define RCC_I2SPRE_RATIO_307       (0x131U << 1)
#define RCC_I2SPRE_RATIO_308       (0x132U << 1)
#define RCC_I2SPRE_RATIO_309       (0x133U << 1)
#define RCC_I2SPRE_RATIO_310       (0x134U << 1)
#define RCC_I2SPRE_RATIO_311       (0x135U << 1)
#define RCC_I2SPRE_RATIO_312       (0x136U << 1)
#define RCC_I2SPRE_RATIO_313       (0x137U << 1)
#define RCC_I2SPRE_RATIO_314       (0x138U << 1)
#define RCC_I2SPRE_RATIO_315       (0x139U << 1)
#define RCC_I2SPRE_RATIO_316       (0x13AU << 1)
#define RCC_I2SPRE_RATIO_317       (0x13BU << 1)
#define RCC_I2SPRE_RATIO_318       (0x13CU << 1)
#define RCC_I2SPRE_RATIO_319       (0x13DU << 1)
#define RCC_I2SPRE_RATIO_320       (0x13EU << 1)
#define RCC_I2SPRE_RATIO_321       (0x13FU << 1)
#define RCC_I2SPRE_RATIO_322       (0x140U << 1)
#define RCC_I2SPRE_RATIO_323       (0x141U << 1)
#define RCC_I2SPRE_RATIO_324       (0x142U << 1)
#define RCC_I2SPRE_RATIO_325       (0x143U << 1)
#define RCC_I2SPRE_RATIO_326       (0x144U << 1)
#define RCC_I2SPRE_RATIO_327       (0x145U << 1)
#define RCC_I2SPRE_RATIO_328       (0x146U << 1)
#define RCC_I2SPRE_RATIO_329       (0x147U << 1)
#define RCC_I2SPRE_RATIO_330       (0x148U << 1)
#define RCC_I2SPRE_RATIO_331       (0x149U << 1)
#define RCC_I2SPRE_RATIO_332       (0x14AU << 1)
#define RCC_I2SPRE_RATIO_333       (0x14BU << 1)
#define RCC_I2SPRE_RATIO_334       (0x14CU << 1)
#define RCC_I2SPRE_RATIO_335       (0x14DU << 1)
#define RCC_I2SPRE_RATIO_336       (0x14EU << 1)
#define RCC_I2SPRE_RATIO_337       (0x14FU << 1)
#define RCC_I2SPRE_RATIO_338       (0x150U << 1)
#define RCC_I2SPRE_RATIO_339       (0x151U << 1)
#define RCC_I2SPRE_RATIO_340       (0x152U << 1)
#define RCC_I2SPRE_RATIO_341       (0x153U << 1)
#define RCC_I2SPRE_RATIO_342       (0x154U << 1)
#define RCC_I2SPRE_RATIO_343       (0x155U << 1)
#define RCC_I2SPRE_RATIO_344       (0x156U << 1)
#define RCC_I2SPRE_RATIO_345       (0x157U << 1)
#define RCC_I2SPRE_RATIO_346       (0x158U << 1)
#define RCC_I2SPRE_RATIO_347       (0x159U << 1)
#define RCC_I2SPRE_RATIO_348       (0x15AU << 1)
#define RCC_I2SPRE_RATIO_349       (0x15BU << 1)
#define RCC_I2SPRE_RATIO_350       (0x15CU << 1)
#define RCC_I2SPRE_RATIO_351       (0x15DU << 1)
#define RCC_I2SPRE_RATIO_352       (0x15EU << 1)
#define RCC_I2SPRE_RATIO_353       (0x15FU << 1)
#define RCC_I2SPRE_RATIO_354       (0x160U << 1)
#define RCC_I2SPRE_RATIO_355       (0x161U << 1)
#define RCC_I2SPRE_RATIO_356       (0x162U << 1)
#define RCC_I2SPRE_RATIO_357       (0x163U << 1)
#define RCC_I2SPRE_RATIO_358       (0x164U << 1)
#define RCC_I2SPRE_RATIO_359       (0x165U << 1)
#define RCC_I2SPRE_RATIO_360       (0x166U << 1)
#define RCC_I2SPRE_RATIO_361       (0x167U << 1)
#define RCC_I2SPRE_RATIO_362       (0x168U << 1)
#define RCC_I2SPRE_RATIO_363       (0x169U << 1)
#define RCC_I2SPRE_RATIO_364       (0x16AU << 1)
#define RCC_I2SPRE_RATIO_365       (0x16BU << 1)
#define RCC_I2SPRE_RATIO_366       (0x16CU << 1)
#define RCC_I2SPRE_RATIO_367       (0x16DU << 1)
#define RCC_I2SPRE_RATIO_368       (0x16EU << 1)
#define RCC_I2SPRE_RATIO_369       (0x16FU << 1)
#define RCC_I2SPRE_RATIO_370       (0x170U << 1)
#define RCC_I2SPRE_RATIO_371       (0x171U << 1)
#define RCC_I2SPRE_RATIO_372       (0x172U << 1)
#define RCC_I2SPRE_RATIO_373       (0x173U << 1)
#define RCC_I2SPRE_RATIO_374       (0x174U << 1)
#define RCC_I2SPRE_RATIO_375       (0x175U << 1)
#define RCC_I2SPRE_RATIO_376       (0x176U << 1)
#define RCC_I2SPRE_RATIO_377       (0x177U << 1)
#define RCC_I2SPRE_RATIO_378       (0x178U << 1)
#define RCC_I2SPRE_RATIO_379       (0x179U << 1)
#define RCC_I2SPRE_RATIO_380       (0x17AU << 1)
#define RCC_I2SPRE_RATIO_381       (0x17BU << 1)
#define RCC_I2SPRE_RATIO_382       (0x17CU << 1)
#define RCC_I2SPRE_RATIO_383       (0x17DU << 1)
#define RCC_I2SPRE_RATIO_384       (0x17EU << 1)
#define RCC_I2SPRE_RATIO_385       (0x17FU << 1)
#define RCC_I2SPRE_RATIO_386       (0x180U << 1)
#define RCC_I2SPRE_RATIO_387       (0x181U << 1)
#define RCC_I2SPRE_RATIO_388       (0x182U << 1)
#define RCC_I2SPRE_RATIO_389       (0x183U << 1)
#define RCC_I2SPRE_RATIO_390       (0x184U << 1)
#define RCC_I2SPRE_RATIO_391       (0x185U << 1)
#define RCC_I2SPRE_RATIO_392       (0x186U << 1)
#define RCC_I2SPRE_RATIO_393       (0x187U << 1)
#define RCC_I2SPRE_RATIO_394       (0x188U << 1)
#define RCC_I2SPRE_RATIO_395       (0x189U << 1)
#define RCC_I2SPRE_RATIO_396       (0x18AU << 1)
#define RCC_I2SPRE_RATIO_397       (0x18BU << 1)
#define RCC_I2SPRE_RATIO_398       (0x18CU << 1)
#define RCC_I2SPRE_RATIO_399       (0x18DU << 1)
#define RCC_I2SPRE_RATIO_400       (0x18EU << 1)
#define RCC_I2SPRE_RATIO_401       (0x18FU << 1)
#define RCC_I2SPRE_RATIO_402       (0x190U << 1)
#define RCC_I2SPRE_RATIO_403       (0x191U << 1)
#define RCC_I2SPRE_RATIO_404       (0x192U << 1)
#define RCC_I2SPRE_RATIO_405       (0x193U << 1)
#define RCC_I2SPRE_RATIO_406       (0x194U << 1)
#define RCC_I2SPRE_RATIO_407       (0x195U << 1)
#define RCC_I2SPRE_RATIO_408       (0x196U << 1)
#define RCC_I2SPRE_RATIO_409       (0x197U << 1)
#define RCC_I2SPRE_RATIO_410       (0x198U << 1)
#define RCC_I2SPRE_RATIO_411       (0x199U << 1)
#define RCC_I2SPRE_RATIO_412       (0x19AU << 1)
#define RCC_I2SPRE_RATIO_413       (0x19BU << 1)
#define RCC_I2SPRE_RATIO_414       (0x19CU << 1)
#define RCC_I2SPRE_RATIO_415       (0x19DU << 1)
#define RCC_I2SPRE_RATIO_416       (0x19EU << 1)
#define RCC_I2SPRE_RATIO_417       (0x19FU << 1)
#define RCC_I2SPRE_RATIO_418       (0x1A0U << 1)
#define RCC_I2SPRE_RATIO_419       (0x1A1U << 1)
#define RCC_I2SPRE_RATIO_420       (0x1A2U << 1)
#define RCC_I2SPRE_RATIO_421       (0x1A3U << 1)
#define RCC_I2SPRE_RATIO_422       (0x1A4U << 1)
#define RCC_I2SPRE_RATIO_423       (0x1A5U << 1)
#define RCC_I2SPRE_RATIO_424       (0x1A6U << 1)
#define RCC_I2SPRE_RATIO_425       (0x1A7U << 1)
#define RCC_I2SPRE_RATIO_426       (0x1A8U << 1)
#define RCC_I2SPRE_RATIO_427       (0x1A9U << 1)
#define RCC_I2SPRE_RATIO_428       (0x1AAU << 1)
#define RCC_I2SPRE_RATIO_429       (0x1ABU << 1)
#define RCC_I2SPRE_RATIO_430       (0x1ACU << 1)
#define RCC_I2SPRE_RATIO_431       (0x1ADU << 1)
#define RCC_I2SPRE_RATIO_432       (0x1AEU << 1)
#define RCC_I2SPRE_RATIO_433       (0x1AFU << 1)
#define RCC_I2SPRE_RATIO_434       (0x1B0U << 1)
#define RCC_I2SPRE_RATIO_435       (0x1B1U << 1)
#define RCC_I2SPRE_RATIO_436       (0x1B2U << 1)
#define RCC_I2SPRE_RATIO_437       (0x1B3U << 1)
#define RCC_I2SPRE_RATIO_438       (0x1B4U << 1)
#define RCC_I2SPRE_RATIO_439       (0x1B5U << 1)
#define RCC_I2SPRE_RATIO_440       (0x1B6U << 1)
#define RCC_I2SPRE_RATIO_441       (0x1B7U << 1)
#define RCC_I2SPRE_RATIO_442       (0x1B8U << 1)
#define RCC_I2SPRE_RATIO_443       (0x1B9U << 1)
#define RCC_I2SPRE_RATIO_444       (0x1BAU << 1)
#define RCC_I2SPRE_RATIO_445       (0x1BBU << 1)
#define RCC_I2SPRE_RATIO_446       (0x1BCU << 1)
#define RCC_I2SPRE_RATIO_447       (0x1BDU << 1)
#define RCC_I2SPRE_RATIO_448       (0x1BEU << 1)
#define RCC_I2SPRE_RATIO_449       (0x1BFU << 1)
#define RCC_I2SPRE_RATIO_450       (0x1C0U << 1)
#define RCC_I2SPRE_RATIO_451       (0x1C1U << 1)
#define RCC_I2SPRE_RATIO_452       (0x1C2U << 1)
#define RCC_I2SPRE_RATIO_453       (0x1C3U << 1)
#define RCC_I2SPRE_RATIO_454       (0x1C4U << 1)
#define RCC_I2SPRE_RATIO_455       (0x1C5U << 1)
#define RCC_I2SPRE_RATIO_456       (0x1C6U << 1)
#define RCC_I2SPRE_RATIO_457       (0x1C7U << 1)
#define RCC_I2SPRE_RATIO_458       (0x1C8U << 1)
#define RCC_I2SPRE_RATIO_459       (0x1C9U << 1)
#define RCC_I2SPRE_RATIO_460       (0x1CAU << 1)
#define RCC_I2SPRE_RATIO_461       (0x1CBU << 1)
#define RCC_I2SPRE_RATIO_462       (0x1CCU << 1)
#define RCC_I2SPRE_RATIO_463       (0x1CDU << 1)
#define RCC_I2SPRE_RATIO_464       (0x1CEU << 1)
#define RCC_I2SPRE_RATIO_465       (0x1CFU << 1)
#define RCC_I2SPRE_RATIO_466       (0x1D0U << 1)
#define RCC_I2SPRE_RATIO_467       (0x1D1U << 1)
#define RCC_I2SPRE_RATIO_468       (0x1D2U << 1)
#define RCC_I2SPRE_RATIO_469       (0x1D3U << 1)
#define RCC_I2SPRE_RATIO_470       (0x1D4U << 1)
#define RCC_I2SPRE_RATIO_471       (0x1D5U << 1)
#define RCC_I2SPRE_RATIO_472       (0x1D6U << 1)
#define RCC_I2SPRE_RATIO_473       (0x1D7U << 1)
#define RCC_I2SPRE_RATIO_474       (0x1D8U << 1)
#define RCC_I2SPRE_RATIO_475       (0x1D9U << 1)
#define RCC_I2SPRE_RATIO_476       (0x1DAU << 1)
#define RCC_I2SPRE_RATIO_477       (0x1DBU << 1)
#define RCC_I2SPRE_RATIO_478       (0x1DCU << 1)
#define RCC_I2SPRE_RATIO_479       (0x1DDU << 1)
#define RCC_I2SPRE_RATIO_480       (0x1DEU << 1)
#define RCC_I2SPRE_RATIO_481       (0x1DFU << 1)
#define RCC_I2SPRE_RATIO_482       (0x1E0U << 1)
#define RCC_I2SPRE_RATIO_483       (0x1E1U << 1)
#define RCC_I2SPRE_RATIO_484       (0x1E2U << 1)
#define RCC_I2SPRE_RATIO_485       (0x1E3U << 1)
#define RCC_I2SPRE_RATIO_486       (0x1E4U << 1)
#define RCC_I2SPRE_RATIO_487       (0x1E5U << 1)
#define RCC_I2SPRE_RATIO_488       (0x1E6U << 1)
#define RCC_I2SPRE_RATIO_489       (0x1E7U << 1)
#define RCC_I2SPRE_RATIO_490       (0x1E8U << 1)
#define RCC_I2SPRE_RATIO_491       (0x1E9U << 1)
#define RCC_I2SPRE_RATIO_492       (0x1EAU << 1)
#define RCC_I2SPRE_RATIO_493       (0x1EBU << 1)
#define RCC_I2SPRE_RATIO_494       (0x1ECU << 1)
#define RCC_I2SPRE_RATIO_495       (0x1EDU << 1)
#define RCC_I2SPRE_RATIO_496       (0x1EEU << 1)
#define RCC_I2SPRE_RATIO_497       (0x1EFU << 1)
#define RCC_I2SPRE_RATIO_498       (0x1F0U << 1)
#define RCC_I2SPRE_RATIO_499       (0x1F1U << 1)
#define RCC_I2SPRE_RATIO_500       (0x1F2U << 1)
#define RCC_I2SPRE_RATIO_501       (0x1F3U << 1)
#define RCC_I2SPRE_RATIO_502       (0x1F4U << 1)
#define RCC_I2SPRE_RATIO_503       (0x1F5U << 1)
#define RCC_I2SPRE_RATIO_504       (0x1F6U << 1)
#define RCC_I2SPRE_RATIO_505       (0x1F7U << 1)
#define RCC_I2SPRE_RATIO_506       (0x1F8U << 1)
#define RCC_I2SPRE_RATIO_507       (0x1F9U << 1)
#define RCC_I2SPRE_RATIO_508       (0x1FAU << 1)
#define RCC_I2SPRE_RATIO_509       (0x1FBU << 1)
#define RCC_I2SPRE_RATIO_510       (0x1FCU << 1)
#define RCC_I2SPRE_RATIO_511       (0x1FDU << 1)
#define RCC_I2SPRE_RATIO_512       (0x1FEU << 1)

#define RCC_I2SPRE_SRCEN           (0x1U << 10)

/********************************  Bit definition for RCC_MCLKSRC register  ***************************/
#define RCC_MCLKSRC_MAINCLK        (0x0U)
#define RCC_MCLKSRC_FHSI           (0x1U)

/********************************  Bit definition for RCC_USBFIFOCLKSRC register  ***************************/
#define RCC_USBFIFOCLKSRC_AHBCLK   (0x0U)
#define RCC_USBFIFOCLKSRC_USBCLK   (0x1U)

/********************************  Bit definition for RCC_MCOSEL register  ***************************/
#define RCC_MCOSEL_NOCLOCK         (0x0U)
#define RCC_MCOSEL_AHBCLK          (0x1U << 0)
#define RCC_MCOSEL_HSE             (0x1U << 1)
#define RCC_MCOSEL_MHSI            (0x1U << 2)
#define RCC_MCOSEL_PLLDIV2         (0x1U << 3)
#define RCC_MCOSEL_MCLK            (0x1U << 4)

/********************************  Bit definition for RCC_AHBENR0 register  ***************************/
#define RCC_AHBENR0_IWDGEN         (0x1U << 2)

/********************************  Bit definition for RCC_AHBENR1 register  ***************************/
#define RCC_AHBENR1_USBEN          (0x1U << 1)
#define RCC_AHBENR1_ISOEN          (0x1U << 2)
#define RCC_AHBENR1_FLASHEN        (0x1U << 3)
#define RCC_AHBENR1_CACHEEN        (0x1U << 4)
#define RCC_AHBENR1_SYSEN          (0x1U << 5)
#define RCC_AHBENR1_DMAC1BREN      (0x1U << 6)
#define RCC_AHBENR1_DMAC2BREN      (0x1U << 7)
#define RCC_AHBENR1_CRCSFMEN       (0x1U << 8)

/********************************  Bit definition for RCC_AHBENR2 register  ***************************/
#define RCC_AHBENR2_BDIEN          (0x1U << 2)

/********************************  Bit definition for RCC_APB1ENR register  ***************************/
#define RCC_APB1ENR_DMAC1EN        (0x1U << 0)
#define RCC_APB1ENR_TIM1EN         (0x1U << 1)
#define RCC_APB1ENR_TIM2EN         (0x1U << 2)
#define RCC_APB1ENR_TIM3EN         (0x1U << 3)
#define RCC_APB1ENR_TIM4EN         (0x1U << 4)
#define RCC_APB1ENR_GPIOAEN        (0x1U << 5)
#define RCC_APB1ENR_GPIOBEN        (0x1U << 6)
#define RCC_APB1ENR_GPIOCEN        (0x1U << 7)
#define RCC_APB1ENR_GPIODEN        (0x1U << 8)
#define RCC_APB1ENR_EXTIEN         (0x1U << 9)
#define RCC_APB1ENR_AFIOEN         (0x1U << 10)
#define RCC_APB1ENR_ADCEN          (0x1U << 11)
#define RCC_APB1ENR_QSPIEN         (0x1U << 12)
#define RCC_APB1ENR_SPIS1EN        (0x1U << 13)
#define RCC_APB1ENR_UART1EN        (0x1U << 14)
#define RCC_APB1ENR_BMX1EN         (0x1U << 15)

/********************************  Bit definition for RCC_APB2ENR register  ***************************/
#define RCC_APB2ENR_DMAC2EN        (0x1U << 0)
#define RCC_APB2ENR_WWDGEN         (0x1U << 1)
#define RCC_APB2ENR_UART2EN        (0x1U << 2)
#define RCC_APB2ENR_UART3EN        (0x1U << 3)
#define RCC_APB2ENR_SPIM2EN        (0x1U << 4)
#define RCC_APB2ENR_SPIS2EN        (0x1U << 5)
#define RCC_APB2ENR_I2SEN          (0x1U << 6)
#define RCC_APB2ENR_I2C1EN         (0x1U << 7)
#define RCC_APB2ENR_I2C2EN         (0x1U << 8)
#define RCC_APB2ENR_RNGEN          (0x1U << 9)
#define RCC_APB2ENR_LEDEN          (0x1U << 10)
#define RCC_APB2ENR_BMX2EN         (0x1U << 11)


/********************************  Bit definition for RCC_RNGCLKENR register  ***************************/
#define RCC_RNGCLKENR_CLKEN        (0x1U)

/********************************  Bit definition for RCC_IWDGCLKENR register  ***************************/
#define RCC_IWDGCLKENR_IWDGCLKEN   (0x1U)
#define RCC_IWDGCLKENR_DCSSCLKEN   (0x1U << 2)

/********************************  Bit definition for RCC_USBCLKENR register  ***************************/
#define RCC_USBCLKENR_CLKEN        (0x1U)

/********************************  Bit definition for RCC_I2SCLKENR register  ***************************/
#define RCC_I2SCLKENR_CLKEN        (0x1U)

/********************************  Bit definition for RCC_SPIS1CLKENR register  ***************************/
#define RCC_SPIS1CLKENR_CLKEN      (0x1U)

/********************************  Bit definition for RCC_SPIS2CLKENR register  ***************************/
#define RCC_SPIS2CLKENR_CLKEN      (0x1U)

/********************************  Bit definition for RCC_USBFIFOCLKENR register  ***************************/
#define RCC_USBFIFOCLKENR_CLKEN    (0x1U)

/********************************  Bit definition for RCC_AHBRSTR1 register  ***************************/
#define RCC_AHBRSTR1_USBRST        (0x1U << 1)
#define RCC_AHBRSTR1_ISORST        (0x1U << 2)
#define RCC_AHBRSTR1_FLASHRST      (0x1U << 3)
#define RCC_AHBRSTR1_CACHERST      (0x1U << 4)
#define RCC_AHBRSTR1_SYSRST        (0x1U << 5)
#define RCC_AHBRSTR1_CRCSFMRST     (0x1U << 8)

/********************************  Bit definition for RCC_APB1RSTR register  ***************************/
#define RCC_APB1RSTR_DMAC1RST      (0x1U << 0)
#define RCC_APB1RSTR_TIM1RST       (0x1U << 1)
#define RCC_APB1RSTR_TIM2RST       (0x1U << 2)
#define RCC_APB1RSTR_TIM3RST       (0x1U << 3)
#define RCC_APB1RSTR_TIM4RST       (0x1U << 4)
#define RCC_APB1RSTR_GPIOARST      (0x1U << 5)
#define RCC_APB1RSTR_GPIOBRST      (0x1U << 6)
#define RCC_APB1RSTR_GPIOCRST      (0x1U << 7)
#define RCC_APB1RSTR_GPIODRST      (0x1U << 8)
#define RCC_APB1RSTR_EXTIRST       (0x1U << 9)
#define RCC_APB1RSTR_AFIORST       (0x1U << 10)
#define RCC_APB1RSTR_ADCRST        (0x1U << 11)
#define RCC_APB1RSTR_QSPIRST       (0x1U << 12)
#define RCC_APB1RSTR_SPIS1RST      (0x1U << 13)
#define RCC_APB1RSTR_UART1RST      (0x1U << 14)
#define RCC_APB1RSTR_BMX1RST       (0x1U << 15)

/********************************  Bit definition for RCC_APB2RSTR register  ***************************/
#define RCC_APB2RSTR_DMAC2RST      (0x1U << 0)
#define RCC_APB2RSTR_WWDGRST       (0x1U << 1)
#define RCC_APB2RSTR_UART2RST      (0x1U << 2)
#define RCC_APB2RSTR_UART3RST      (0x1U << 3)
#define RCC_APB2RSTR_SPIM2RST      (0x1U << 4)
#define RCC_APB2RSTR_SPIS2RST      (0x1U << 5)
#define RCC_APB2RSTR_I2SRST        (0x1U << 6)
#define RCC_APB2RSTR_I2C1RST       (0x1U << 7)
#define RCC_APB2RSTR_I2C2RST       (0x1U << 8)
#define RCC_APB2RSTR_RNGRST        (0x1U << 9)
#define RCC_APB2RSTR_LEDRST        (0x1U << 10)
#define RCC_APB2RSTR_BMX2RST       (0x1U << 11)

/********************************  Bit definition for RCC_I2SCLKRSTR register  ***************************/
#define RCC_I2SCLKRSTR_SCLKRST     (0x1U)

/********************************  Bit definition for RCC_CLRRSTSTAT register  ***************************/
#define RCC_CLRRSTSTAT_CLR         (0x1U)

/********************************  Bit definition for RCC_BDRSTR register  ***************************/
#define RCC_BDRSTR_BDRST           (0x1U)

/********************************  Bit definition for RCC_LSI2RTCENR register  ***************************/
#define RCC_LSI2RTCENR_CLKEN       (0x1U)

/********************************  Bit definition for RCC_HSE2RTCENR register  ***************************/
#define RCC_HSE2RTCENR_DIVEN       (0x1U)

/********************************  Bit definition for RCC_RSTSTAT register  ***************************/
#define RCC_RSTSTAT_LPWRRSTF       (0x1U << 0)
#define RCC_RSTSTAT_WWDGRSTF       (0x1U << 1)
#define RCC_RSTSTAT_IWDGRSTF       (0x1U << 2)
#define RCC_RSTSTAT_SFTRSTF        (0x1U << 3)
#define RCC_RSTSTAT_PORRSTF        (0x1U << 4)
#define RCC_RSTSTAT_PINRSTF        (0x1U << 5)



/*------------------------------------------------------------------------------------------------------*/
/*---                                      Power Control (PWR)                                       ---*/
/*------------------------------------------------------------------------------------------------------*/
/********************************  Bit definition for PWR_CR0 register  *********************************/
#define PWR_CR0_DBP                 (0x1U << 0)          /*!< Disable Backup Domain write protection */

#define PWR_CR0_FCLKSD              (0x1U << 3)

#define PWR_CR0_PDDS_Pos            (5U)
#define PWR_CR0_PDDS_Msk            (0x3U << PWR_CR0_PDDS_Pos)

#define PWR_CR0_S32KMODE            (0x1U << 18)
#define PWR_CR0_S4KMODE             (0x1U << 19)

/********************************  Bit definition for PWR_CR1 register  *********************************/
#define PWR_CR1_CWUF                (0x1U << 0)
#define PWR_CR1_CSBF                (0x1U << 1)
#define PWR_CR1_CSPF                (0x1U << 2)
#define PWR_CR1_CCKF                (0x1U << 3)

/********************************  Bit definition for PWR_CR2 register  *********************************/
#define PWR_CR2_EWUP                (0x1U << 0)

/********************************  Bit definition for PWR_SR0 register  *********************************/
#define PWR_SR0_PVDO                (0x1U << 0)

/********************************  Bit definition for PWR_SR1 register  *********************************/
#define PWR_SR1_WUF                 (0x1U << 0)
#define PWR_SR1_SBF                 (0x1U << 1)
#define PWR_SR1_SPF                 (0x1U << 2)
#define PWR_SR1_CKF                 (0x1U << 3)

/********************************  Bit definition for DBGMCU_CR register  *********************************/
#define DBGMCU_CR_DBG_IWDG_STOP_Pos         (8U)
#define DBGMCU_CR_DBG_IWDG_STOP_Msk         (0x1U << DBGMCU_CR_DBG_IWDG_STOP_Pos) /*!< 0x00000100 */
#define DBGMCU_CR_DBG_IWDG_STOP             DBGMCU_CR_DBG_IWDG_STOP_Msk           /*!< Debug Independent Watchdog stopped when Core is halted */
#define DBGMCU_CR_DBG_WWDG_STOP_Pos         (9U)
#define DBGMCU_CR_DBG_WWDG_STOP_Msk         (0x1U << DBGMCU_CR_DBG_WWDG_STOP_Pos) /*!< 0x00000200 */
#define DBGMCU_CR_DBG_WWDG_STOP             DBGMCU_CR_DBG_WWDG_STOP_Msk           /*!< Debug Window Watchdog stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM1_STOP_Pos         (10U)
#define DBGMCU_CR_DBG_TIM1_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM1_STOP_Pos) /*!< 0x00000400 */
#define DBGMCU_CR_DBG_TIM1_STOP             DBGMCU_CR_DBG_TIM1_STOP_Msk           /*!< TIM1 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM2_STOP_Pos         (11U)
#define DBGMCU_CR_DBG_TIM2_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM2_STOP_Pos) /*!< 0x00000800 */
#define DBGMCU_CR_DBG_TIM2_STOP             DBGMCU_CR_DBG_TIM2_STOP_Msk           /*!< TIM2 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM3_STOP_Pos         (12U)
#define DBGMCU_CR_DBG_TIM3_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM3_STOP_Pos) /*!< 0x00001000 */
#define DBGMCU_CR_DBG_TIM3_STOP             DBGMCU_CR_DBG_TIM3_STOP_Msk           /*!< TIM3 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM4_STOP_Pos         (13U)
#define DBGMCU_CR_DBG_TIM4_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM4_STOP_Pos) /*!< 0x00002000 */
#define DBGMCU_CR_DBG_TIM4_STOP             DBGMCU_CR_DBG_TIM4_STOP_Msk           /*!< TIM4 counter stopped when core is halted */



#define BIT_BAND_ADDR(addr, bitnum) ((((uint32_t)(addr)) & 0xF0000000) + 0x2000000 + ((((uint32_t)(addr)) & 0xFFFFF) << 5) + ((bitnum) << 2))


/** @addtogroup Exported_macro
  * @{
  */

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __WB32F3G71xx_H__ */

/**
  * @}
  */

/**
  * @}
  */
