/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio
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
 * @file    GD32VF103/hal_lld.h
 * @brief   GD32VF103 HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - GD32_LXTALCLK.
 *          - GD32_LXTAL_BYPASS (optionally).
 *          - GD32_HXTALCLK.
 *          - GD32_HXTAL_BYPASS (optionally).
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification
 * @{
 */
#if defined(__DOXYGEN__) || \
    defined(GD32VF103TB) || defined(GD32VF103T8) || defined(GD32VF103T6) || defined(GD32VF103T4) || \
    defined(GD32VF103CB) || defined(GD32VF103C8) || defined(GD32VF103C6) || defined(GD32VF103C4) || \
    defined(GD32VF103RB) || defined(GD32VF103R8) || defined(GD32VF103R6) || defined(GD32VF103R4) || \
    defined(GD32VF103VB) || defined(GD32VF103V8)
  #define PLATFORM_NAME           "GigaDevice GD32VF103 RISC-V"
  #define GD32VF103
#else
  #error "unsupported or unrecognized GD32VF103 member"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(GD32VF103) || defined(__DOXYGEN__)
#define GD32VF103
#endif
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define GD32_IRC8MCLK            8000000     /**< High speed internal clock. */
#define GD32_IRC40KCLK             40000     /**< Low speed internal clock.  */
/** @} */

/**
 * @name    PMU_CTL register bits definitions
 * @{
 */
#define GD32_LVDT_MASK          (7 << 5)    /**< LVDT bits mask.            */
#define GD32_LVDT_LEV0          (0 << 5)    /**< LVDT level 0.              */
#define GD32_LVDT_LEV1          (1 << 5)    /**< LVDT level 1.              */
#define GD32_LVDT_LEV2          (2 << 5)    /**< LVDT level 2.              */
#define GD32_LVDT_LEV3          (3 << 5)    /**< LVDT level 3.              */
#define GD32_LVDT_LEV4          (4 << 5)    /**< LVDT level 4.              */
#define GD32_LVDT_LEV5          (5 << 5)    /**< LVDT level 5.              */
#define GD32_LVDT_LEV6          (6 << 5)    /**< LVDT level 6.              */
#define GD32_LVDT_LEV7          (7 << 5)    /**< LVDT level 7.              */
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Maximum system clock frequency.
 */
#if defined(GD32_ALLOW_120MHZ_SYSCLK)
#define GD32_SYSCLK_MAX        120000000
#else
#define GD32_SYSCLK_MAX        108000000
#endif

/**
 * @brief   Maximum HXTAL clock frequency.
 */
#define GD32_HXTALCLK_MAX        25000000

/**
 * @brief   Minimum HXTAL clock frequency.
 */
#define GD32_HXTALCLK_MIN         3000000

/**
 * @brief   Maximum LXTAL clock frequency.
 */
#define GD32_LXTALCLK_MAX        1000000

/**
 * @brief   Minimum LXTAL clock frequency.
 */
#define GD32_LXTALCLK_MIN        32768

/**
 * @brief   Maximum PLL input clock frequency.
 */
#define GD32_PLLIN_MAX        12000000

/**
 * @brief   Minimum PLL input clock frequency.
 */
#define GD32_PLLIN_MIN        3000000

/**
 * @brief   Maximum PLL1 input clock frequency.
 */
#define GD32_PLL12IN_MAX       5000000

/**
 * @brief   Minimum PLL1 and PLL2 input clock frequency.
 */
#define GD32_PLL12IN_MIN       3000000

/**
 * @brief   Maximum PLL output clock frequency.
 */
#if defined(GD32_ALLOW_120MHZ_SYSCLK)
#define GD32_PLLOUT_MAX        120000000
#else
#define GD32_PLLOUT_MAX        108000000
#endif

/**
 * @brief   Minimum PLL output clock frequency.
 */
#define GD32_PLLOUT_MIN        16000000

/**
 * @brief   Maximum PLL1 and PLL2 VCO clock frequency.
 */
#define GD32_PLL12VCO_MAX      148000000

/**
 * @brief   Minimum PLL1 and PLL2 VCO clock frequency.
 */
#define GD32_PLL12VCO_MIN      80000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#if defined(GD32_ALLOW_120MHZ_SYSCLK)
#define GD32_PCLK1_MAX         60000000
#else
#define GD32_PCLK1_MAX         54000000
#endif

/**
 * @brief   Maximum APB2 clock frequency.
 */
#if defined(GD32_ALLOW_120MHZ_SYSCLK)
#define GD32_PCLK2_MAX        120000000
#else
#define GD32_PCLK2_MAX        108000000
#endif

/**
 * @brief   Maximum ADC clock frequency.
 */
#define GD32_ADCCLK_MAX        14000000

/**
 * @brief   Maximum SPI/I2S clock frequency.
 */
#define GD32_SPII2S_MAX        18000000
/** @} */

/**
 * @name    RCU_CFG0 register bits definitions
 * @{
 */
#define GD32_SCS_IRC8M            (0 << 0)    /**< SYSCLK source is IRC8M.      */
#define GD32_SCS_HXTAL            (1 << 0)    /**< SYSCLK source is HXTAL.      */
#define GD32_SCS_PLL              (2 << 0)    /**< SYSCLK source is PLL.      */

#define GD32_AHBPSC_DIV1         (0 << 4)    /**< SYSCLK divided by 1.       */
#define GD32_AHBPSC_DIV2         (8 << 4)    /**< SYSCLK divided by 2.       */
#define GD32_AHBPSC_DIV4         (9 << 4)    /**< SYSCLK divided by 4.       */
#define GD32_AHBPSC_DIV8         (10 << 4)   /**< SYSCLK divided by 8.       */
#define GD32_AHBPSC_DIV16        (11 << 4)   /**< SYSCLK divided by 16.      */
#define GD32_AHBPSC_DIV64        (12 << 4)   /**< SYSCLK divided by 64.      */
#define GD32_AHBPSC_DIV128       (13 << 4)   /**< SYSCLK divided by 128.     */
#define GD32_AHBPSC_DIV256       (14 << 4)   /**< SYSCLK divided by 256.     */
#define GD32_AHBPSC_DIV512       (15 << 4)   /**< SYSCLK divided by 512.     */

#define GD32_APB1PSC_DIV1        (0 << 8)    /**< HCLK divided by 1.         */
#define GD32_APB1PSC_DIV2        (4 << 8)    /**< HCLK divided by 2.         */
#define GD32_APB1PSC_DIV4        (5 << 8)    /**< HCLK divided by 4.         */
#define GD32_APB1PSC_DIV8        (6 << 8)    /**< HCLK divided by 8.         */
#define GD32_APB1PSC_DIV16       (7 << 8)    /**< HCLK divided by 16.        */

#define GD32_APB2PSC_DIV1        (0 << 11)   /**< HCLK divided by 1.         */
#define GD32_APB2PSC_DIV2        (4 << 11)   /**< HCLK divided by 2.         */
#define GD32_APB2PSC_DIV4        (5 << 11)   /**< HCLK divided by 4.         */
#define GD32_APB2PSC_DIV8        (6 << 11)   /**< HCLK divided by 8.         */
#define GD32_APB2PSC_DIV16       (7 << 11)   /**< HCLK divided by 16.        */

#define GD32_ADCPSC_DIV2       (0 << 14)   /**< PPRE2 divided by 2.        */
#define GD32_ADCPSC_DIV4       (1 << 14)   /**< PPRE2 divided by 4.        */
#define GD32_ADCPSC_DIV6       (2 << 14)   /**< PPRE2 divided by 6.        */
#define GD32_ADCPSC_DIV8       (3 << 14)   /**< PPRE2 divided by 8.        */
#define GD32_ADCPSC_DIV12      ((1 << 28) | (1 << 14))  /**< PPRE2 divided by 12.        */
#define GD32_ADCPSC_DIV16      ((1 << 28) | (3 << 14))  /**< PPRE2 divided by 16.        */

#define GD32_PLLSEL_IRC8M     (0 << 16)   /**< PLL clock source is IRC8M.   */
#define GD32_PLLSEL_PREDV0    (1 << 16)   /**< PLL clock source is
                                                 PREDV0.                   */

#define GD32_USBFSPSC_DIV1       (1 << 22)     /**< PLLOUT divided by 1.       */
#define GD32_USBFSPSC_DIV1P5     (0 << 22)     /**< PLLOUT divided by 1.5.       */
#define GD32_USBFSPSC_DIV2       (3 << 22)     /**< PLLOUT divided by 2.       */
#define GD32_USBFSPSC_DIV2P5     (2 << 22)     /**< PLLOUT divided by 2.5.     */

#define GD32_CKOUT0SEL_NOCLOCK    (0 << 24)   /**< No clock on MCO pin.       */
#define GD32_CKOUT0SEL_SYSCLK     (4 << 24)   /**< SYSCLK on MCO pin.         */
#define GD32_CKOUT0SEL_IRC8M        (5 << 24)   /**< IRC8M clock on MCO pin.      */
#define GD32_CKOUT0SEL_HXTAL        (6 << 24)   /**< HXTAL clock on MCO pin.      */
#define GD32_CKOUT0SEL_PLLDIV2    (7 << 24)   /**< PLL/2 clock on MCO pin.    */
#define GD32_CKOUT0SEL_PLL1       (8 << 24)   /**< PLL1 clock on MCO pin.     */
#define GD32_CKOUT0SEL_PLL2DIV2   (9 << 24)   /**< PLL2/2 clock on MCO pin.   */
#define GD32_CKOUT0SEL_XT1        (10 << 24)  /**< XT1 clock on MCO pin.      */
#define GD32_CKOUT0SEL_PLL2       (11 << 24)  /**< PLL2 clock on MCO pin.     */
/** @} */

/**
 * @name    RCU_BDCTL register bits definitions
 * @{
 */
#define GD32_RTCSRC_MASK       (3 << 8)    /**< RTC clock source mask.     */
#define GD32_RTCSRC_NOCLOCK    (0 << 8)    /**< No clock.                  */
#define GD32_RTCSRC_LXTAL        (1 << 8)    /**< LXTAL used as RTC clock.     */
#define GD32_RTCSRC_IRC40K        (2 << 8)    /**< IRC40K used as RTC clock.     */
#define GD32_RTCSRC_HXTALDIV     (3 << 8)    /**< HXTAL divided by 128 used as
                                                 RTC clock.                 */
/** @} */

/**
 * @name    RCU_CFG02 register bits definitions
 * @{
 */
#define GD32_PREDV0SEL_HXTAL    (0 << 16)   /**< PREDV0 source is HXTAL.     */
#define GD32_PREDV0SEL_PLL1   (1 << 16)   /**< PREDV0 source is PLL1.    */
/** @} */

#define GD32_PLLMF_VALUE_6P5 65

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the PMU/RCU initialization in the HAL.
 */
#if !defined(GD32_NO_INIT) || defined(__DOXYGEN__)
#define GD32_NO_INIT               FALSE
#endif

/**
 * @brief   Enables or disables the programmable voltage detector.
 */
#if !defined(GD32_PVD_ENABLE) || defined(__DOXYGEN__)
#define GD32_PVD_ENABLE            FALSE
#endif

/**
 * @brief   Sets voltage level for programmable voltage detector.
 */
#if !defined(GD32_LVDT) || defined(__DOXYGEN__)
#define GD32_LVDT                   GD32_LVDT_LEV0
#endif

/**
 * @brief   Enables or disables the IRC8M clock source.
 */
#if !defined(GD32_IRC8M_ENABLED) || defined(__DOXYGEN__)
#define GD32_IRC8M_ENABLED           TRUE
#endif

/**
 * @brief   Enables or disables the IRC40K clock source.
 */
#if !defined(GD32_IRC40K_ENABLED) || defined(__DOXYGEN__)
#define GD32_IRC40K_ENABLED           FALSE
#endif

/**
 * @brief   Enables or disables the HXTAL clock source.
 */
#if !defined(GD32_HXTAL_ENABLED) || defined(__DOXYGEN__)
#define GD32_HXTAL_ENABLED           TRUE
#endif

/**
 * @brief   Enables or disables the LXTAL clock source.
 */
#if !defined(GD32_LXTAL_ENABLED) || defined(__DOXYGEN__)
#define GD32_LXTAL_ENABLED           FALSE
#endif
/** @} */

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Main clock source selection.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using both PLL and PLL2.
 */
#if !defined(GD32_SCS) || defined(__DOXYGEN__)
#define GD32_SCS                    GD32_SCS_PLL
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using both PLL and PLL2.
 */
#if !defined(GD32_PLLSEL) || defined(__DOXYGEN__)
#define GD32_PLLSEL                GD32_PLLSEL_PREDV0
#endif

/**
 * @brief   PREDV0 clock source.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using both PLL and PLL2.
 */
#if !defined(GD32_PREDV0SEL) || defined(__DOXYGEN__)
#define GD32_PREDV0SEL            GD32_PREDV0SEL_HXTAL
#endif

/**
 * @brief   PREDV0 division factor.
 * @note    The allowed range is 1...16.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using both PLL and PLL2.
 */
#if !defined(GD32_PREDV0_VALUE) || defined(__DOXYGEN__)
#define GD32_PREDV0_VALUE         2
#endif

/**
 * @brief   PLL multiplier value.
 * @note    The allowed range is 4...32.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using both PLL and PLL2.
 */
#if !defined(GD32_PLLMF_VALUE) || defined(__DOXYGEN__)
#define GD32_PLLMF_VALUE          18
#endif

/**
 * @brief   PREDV1 division factor.
 * @note    The allowed range is 1...16.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using both PLL and PLL2.
 */
#if !defined(GD32_PREDV1_VALUE) || defined(__DOXYGEN__)
#define GD32_PREDV1_VALUE         2
#endif

/**
 * @brief   PLL1 multiplier value.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using both PLL and PLL2.
 */
#if !defined(GD32_PLL1MF_VALUE) || defined(__DOXYGEN__)
#define GD32_PLL1MF_VALUE         14
#endif

/**
 * @brief   PLL2 multiplier value.
 * @note    The default value is calculated for a 72MHz clock from
 *          a 8MHz crystal.
 */
#if !defined(GD32_PLL2MF_VALUE) || defined(__DOXYGEN__)
#define GD32_PLL2MF_VALUE         13
#endif

/**
 * @brief   AHB prescaler value.
 * @note    The default value is calculated for a 72MHz system clock from
 *          a 8MHz crystal using both PLL and PLL2.
 */
#if !defined(GD32_AHBPSC) || defined(__DOXYGEN__)
#define GD32_AHBPSC                  GD32_AHBPSC_DIV1
#endif

/**
 * @brief   APB1 prescaler value.
 */
#if !defined(GD32_APB1PSC) || defined(__DOXYGEN__)
#define GD32_APB1PSC                 GD32_APB1PSC_DIV2
#endif

/**
 * @brief   APB2 prescaler value.
 */
#if !defined(GD32_APB2PSC) || defined(__DOXYGEN__)
#define GD32_APB2PSC                 GD32_APB2PSC_DIV1
#endif

/**
 * @brief   ADC prescaler value.
 */
#if !defined(GD32_ADCPSC) || defined(__DOXYGEN__)
#define GD32_ADCPSC                GD32_ADCPSC_DIV6
#endif

/**
 * @brief   USB clock setting.
 */
#if !defined(GD32_USBFS_CLOCK_REQUIRED) || defined(__DOXYGEN__)
#define GD32_USBFS_CLOCK_REQUIRED    TRUE
#endif

/**
 * @brief   USBFS prescaler initialization.
 */
#if !defined(GD32_USBFSPSC) || defined(__DOXYGEN__)
#define GD32_USBFSPSC              GD32_USBFSPSC_DIV1P5
#endif

/**
 * @brief   Dedicated I2S clock setting.
 */
#if !defined(GD32_I2S_CLOCK_REQUIRED) || defined(__DOXYGEN__)
#define GD32_I2S_CLOCK_REQUIRED    FALSE
#endif

/**
 * @brief   MCO pin setting.
 */
#if !defined(GD32_CKOUT0SEL) || defined(__DOXYGEN__)
#define GD32_CKOUT0SEL              GD32_CKOUT0SEL_NOCLOCK
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(GD32_RTCSRC) || defined(__DOXYGEN__)
#define GD32_RTCSRC                   GD32_RTCSRC_NOCLOCK
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(GD32VF103_MCUCONF)
#error "Using a wrong mcuconf.h file, GD32VF103_MCUCONF not defined"
#endif

/*
 * IRC8M related checks.
 */
#if GD32_IRC8M_ENABLED
#else /* !GD32_IRC8M_ENABLED */

#if GD32_SCS == GD32_SCS_IRC8M
#error "IRC8M not enabled, required by GD32_SCS"
#endif

#if (GD32_SCS == GD32_SCS_PLL) && (GD32_PLLSEL == GD32_PLLSEL_IRC8M)
#error "IRC8M not enabled, required by GD32_SCS and GD32_PLLSEL"
#endif

#if (GD32_CKOUT0SEL == GD32_CKOUT0SEL_IRC8M) ||                                   \
    ((GD32_CKOUT0SEL == GD32_CKOUT0SEL_PLLDIV2) &&                              \
     (GD32_PLLSEL == GD32_PLLSEL_IRC8M))
#error "IRC8M not enabled, required by GD32_CKOUT0SEL"
#endif

#endif /* !GD32_IRC8M_ENABLED */

/*
 * HXTAL related checks.
 */
#if GD32_HXTAL_ENABLED

#if GD32_HXTALCLK == 0
#error "HXTAL frequency not defined"
#elif (GD32_HXTALCLK < GD32_HXTALCLK_MIN) || (GD32_HXTALCLK > GD32_HXTALCLK_MAX)
#error "GD32_HXTALCLK outside acceptable range (GD32_HXTALCLK_MIN...GD32_HXTALCLK_MAX)"
#endif

#else /* !GD32_HXTAL_ENABLED */

#if GD32_SCS == GD32_SCS_HXTAL
#error "HXTAL not enabled, required by GD32_SCS"
#endif

#if (GD32_SCS == GD32_SCS_PLL) && (GD32_PLLSEL == GD32_PLLSEL_PREDV0)
#error "HXTAL not enabled, required by GD32_SCS and GD32_PLLSEL"
#endif

#if (GD32_CKOUT0SEL == GD32_CKOUT0SEL_HXTAL) ||                                   \
    (((GD32_CKOUT0SEL == GD32_CKOUT0SEL_PLLDIV2) ||                             \
      (GD32_CKOUT0SEL == GD32_CKOUT0SEL_PLL1) ||                                \
      (GD32_CKOUT0SEL == GD32_CKOUT0SEL_PLL2) ||                                \
      (GD32_CKOUT0SEL == GD32_CKOUT0SEL_PLL2DIV2)) &&                           \
     (GD32_PLLSEL == GD32_PLLSEL_HXTAL)) ||                                 \
    (GD32_CKOUT0SEL == GD32_CKOUT0SEL_XT1)
#error "HXTAL not enabled, required by GD32_CKOUT0SEL"
#endif

#if GD32_RTCSRC == GD32_RTCSRC_HXTALDIV
#error "HXTAL not enabled, required by GD32_RTCSRC"
#endif

#endif /* !GD32_HXTAL_ENABLED */

/*
 * IRC40K related checks.
 */
#if GD32_IRC40K_ENABLED
#else /* !GD32_IRC40K_ENABLED */

#if GD32_RTCSRC == GD32_RTCSRC_IRC40K
#error "IRC40K not enabled, required by GD32_RTCSRC"
#endif

#endif /* !GD32_IRC40K_ENABLED */

/*
 * LXTAL related checks.
 */
#if GD32_LXTAL_ENABLED

#if (GD32_LXTALCLK == 0)
#error "LXTAL frequency not defined"
#endif

#if (GD32_LXTALCLK < GD32_LXTALCLK_MIN) || (GD32_LXTALCLK > GD32_LXTALCLK_MAX)
#error "GD32_LXTALCLK outside acceptable range (GD32_LXTALCLK_MIN...GD32_LXTALCLK_MAX)"
#endif

#else /* !GD32_LXTAL_ENABLED */

#if GD32_RTCSRC == GD32_RTCSRC_LXTAL
#error "LXTAL not enabled, required by GD32_RTCSRC"
#endif

#endif /* !GD32_LXTAL_ENABLED */

/* PLL activation conditions.*/
#if GD32_USBFS_CLOCK_REQUIRED ||                                             \
    (GD32_SCS == GD32_SCS_PLL) ||                                           \
    (GD32_CKOUT0SEL == GD32_CKOUT0SEL_PLLDIV2) ||                               \
    defined(__DOXYGEN__)
/**
 * @brief   PLL activation flag.
 */
#define GD32_ACTIVATE_PLL         TRUE
#else
#define GD32_ACTIVATE_PLL         FALSE
#endif

/* PLL1 activation conditions.*/
#if ((GD32_PREDV0SEL == GD32_PREDV0SEL_PLL1) && GD32_ACTIVATE_PLL) || \
    (GD32_CKOUT0SEL == GD32_CKOUT0SEL_PLL1) || defined(__DOXYGEN__)
/**
 * @brief   PLL1 activation flag.
 */
#define GD32_ACTIVATE_PLL1         TRUE
#else
#define GD32_ACTIVATE_PLL1         FALSE
#endif

/* PLL2 activation conditions.*/
#if GD32_I2S_CLOCK_REQUIRED ||                                             \
    (GD32_CKOUT0SEL == GD32_CKOUT0SEL_PLL2DIV2) ||                              \
    (GD32_CKOUT0SEL == GD32_CKOUT0SEL_PLL2) ||                                  \
    defined(__DOXYGEN__)
/**
 * @brief   PLL2 activation flag.
 */
#define GD32_ACTIVATE_PLL2         TRUE
#else
#define GD32_ACTIVATE_PLL2         FALSE
#endif

/**
 * @brief   PREDV0 field.
 */
#if (GD32_PREDV0_VALUE >= 1) && (GD32_PREDV0_VALUE <= 16) ||            \
    defined(__DOXYGEN__)
#define GD32_PREDV0               ((GD32_PREDV0_VALUE - 1) << 0)
#else
#error "invalid GD32_PREDV0_VALUE value specified"
#endif

/**
 * @brief   PREDV1 field.
 */
#if (GD32_PREDV1_VALUE >= 1) && (GD32_PREDV1_VALUE <= 16) ||            \
    defined(__DOXYGEN__)
#define GD32_PREDV1               ((GD32_PREDV1_VALUE - 1) << 4)
#else
#error "invalid GD32_PREDV1_VALUE value specified"
#endif

/**
 * @brief   PLLMUL field.
 */
#if GD32_PLLMF_VALUE == GD32_PLLMF_VALUE_6P5 || defined(__DOXYGEN__)
    #define GD32_PLLMF 13 << 18
#elif ((GD32_PLLMF_VALUE >= 2) && (GD32_PLLMF_VALUE <= 16))
    #define GD32_PLLMF                ((GD32_PLLMF_VALUE - 2) << 18)
#elif ((GD32_PLLMF_VALUE >= 17) && (GD32_PLLMF_VALUE <= 32))
    #define GD32_PLLMF                ((1 << 29) | ((GD32_PLLMF_VALUE - 17) << 18))
#else
#error "invalid GD32_PLLMF_VALUE value specified"
#endif

/**
 * @brief   PLL1MF field.
 */
#if ((GD32_PLL1MF_VALUE >= 8) && (GD32_PLL1MF_VALUE <= 14)) ||          \
    defined(__DOXYGEN__)
#define GD32_PLL1MF               ((GD32_PLL1MF_VALUE - 2) << 8)
#elif (GD32_PLL1MF_VALUE == 16)
#define GD32_PLL1MF               (14 << 8)
#elif (GD32_PLL1MF_VALUE == 20)
#define GD32_PLL1MF               (15 << 8)
#else
#error "invalid GD32_PLL1MF_VALUE value specified"
#endif

/**
 * @brief   PLL2MF field.
 */
#if ((GD32_PLL2MF_VALUE >= 8) && (GD32_PLL2MF_VALUE <= 14)) ||          \
    defined(__DOXYGEN__)
#define GD32_PLL2MF               ((GD32_PLL2MF_VALUE - 2) << 12)
#elif (GD32_PLL2MF_VALUE == 16)
#define GD32_PLL2MF               (14 << 12)
#elif (GD32_PLL2MF_VALUE == 20)
#define GD32_PLL2MF               (15 << 12)
#else
#error "invalid GD32_PLL2MF_VALUE value specified"
#endif

/**
 * @brief   PLL1 input frequency.
 */
#define GD32_PLL1CLKIN             (GD32_HXTALCLK / GD32_PREDV1_VALUE)

/* PLL1 input frequency range check.*/
#if (GD32_PLL1CLKIN < GD32_PLL12IN_MIN) ||                                \
    (GD32_PLL1CLKIN > GD32_PLL12IN_MAX)
#error "GD32_PLL1CLKIN outside acceptable range (GD32_PLL12IN_MIN...GD32_PLL12IN_MAX)"
#endif


/**
 * @brief   PLL1 output clock frequency.
 */
#define GD32_PLL1CLKOUT            (GD32_PLL1CLKIN * GD32_PLL1MF_VALUE)

/**
 * @brief   PLL1 VCO clock frequency.
 */
#define GD32_PLL1VCO               (GD32_PLL1CLKOUT * 2)

/* PLL1 output frequency range check.*/
#if (GD32_PLL1VCO < GD32_PLL12VCO_MIN) ||                                 \
    (GD32_PLL1VCO > GD32_PLL12VCO_MAX)
#error "GD32_PLL1VCO outside acceptable range (GD32_PLL12VCO_MIN...GD32_PLL12VCO_MAX)"
#endif


/**
 * @brief   PLL2 input frequency.
 */
#define GD32_PLL2CLKIN             (GD32_HXTALCLK / GD32_PREDV1_VALUE)

/* PLL2 input frequency range check.*/
#if (GD32_PLL2CLKIN < GD32_PLL12IN_MIN) ||                                \
    (GD32_PLL2CLKIN > GD32_PLL12IN_MAX)
#error "GD32_PLL2CLKIN outside acceptable range (GD32_PLL12IN_MIN...GD32_PLL12IN_MAX)"
#endif

/**
 * @brief   PLL2 output clock frequency.
 */
#define GD32_PLL2CLKOUT            (GD32_PLL2CLKIN * GD32_PLL2MF_VALUE)

/**
 * @brief   PLL2 VCO clock frequency.
 */
#define GD32_PLL2VCO               (GD32_PLL2CLKOUT * 2)

/* PLL2 output frequency range check.*/
#if (GD32_PLL2VCO < GD32_PLL12VCO_MIN) ||                                 \
    (GD32_PLL2VCO > GD32_PLL12VCO_MAX)
#error "GD32_PLL2CLKOUT outside acceptable range (GD32_PLL12VCO_MIN...GD32_PLL12VCO_MAX)"
#endif

/**
 * @brief   PREDV0 input frequency.
 */
#if (GD32_PREDV0SEL == GD32_PREDV0SEL_HXTAL) || defined(__DOXYGEN__)
#define GD32_PREDV0CLK            GD32_HXTALCLK
#elif GD32_PREDV0SEL == GD32_PREDV0SEL_PLL1
#define GD32_PREDV0CLK            GD32_PLL1CLKOUT
#else
#error "invalid GD32_PREDV0SEL value specified"
#endif

/**
 * @brief   PLL input clock frequency.
 */
#if (GD32_PLLSEL == GD32_PLLSEL_PREDV0) || defined(__DOXYGEN__)
#define GD32_PLLCLKIN              (GD32_PREDV0CLK / GD32_PREDV0_VALUE)
#elif GD32_PLLSEL == GD32_PLLSEL_IRC8M
#define GD32_PLLCLKIN              (GD32_IRC8MCLK / 2)
#else
#error "invalid GD32_PLLSEL value specified"
#endif


/* PLL input frequency range check.*/
#if (GD32_PLLCLKIN < GD32_PLLIN_MIN) || (GD32_PLLCLKIN > GD32_PLLIN_MAX)
#error "GD32_PLLCLKIN outside acceptable range (GD32_PLLIN_MIN...GD32_PLLIN_MAX)"
#endif

/**
 * @brief   PLL output clock frequency.
 */
#define GD32_PLLCLKOUT             (GD32_PLLCLKIN * GD32_PLLMF_VALUE)


/* PLL output frequency range check.*/
#if (GD32_PLLCLKOUT < GD32_PLLOUT_MIN) || (GD32_PLLCLKOUT > GD32_PLLOUT_MAX)
#error "GD32_PLLCLKOUT outside acceptable range (GD32_PLLOUT_MIN...GD32_PLLOUT_MAX)"
#endif

/**
 * @brief   System clock source.
 */
#if (GD32_SCS == GD32_SCS_PLL) || defined(__DOXYGEN__)
#define GD32_SYSCLK                GD32_PLLCLKOUT
#elif (GD32_SCS == GD32_SCS_IRC8M)
#define GD32_SYSCLK                GD32_IRC8MCLK
#elif (GD32_SCS == GD32_SCS_HXTAL)
#define GD32_SYSCLK                GD32_HXTALCLK
#else
#error "invalid GD32_SCS value specified"
#endif

/* Check on the system clock.*/
#if GD32_SYSCLK > GD32_SYSCLK_MAX
#error "GD32_SYSCLK above maximum rated frequency (GD32_SYSCLK_MAX)"
#endif

/**
 * @brief   AHB frequency.
 */
#if (GD32_AHBPSC == GD32_AHBPSC_DIV1) || defined(__DOXYGEN__)
#define GD32_HCLK                  (GD32_SYSCLK / 1)
#elif GD32_AHBPSC == GD32_AHBPSC_DIV2
#define GD32_HCLK                  (GD32_SYSCLK / 2)
#elif GD32_AHBPSC == GD32_AHBPSC_DIV4
#define GD32_HCLK                  (GD32_SYSCLK / 4)
#elif GD32_AHBPSC == GD32_AHBPSC_DIV8
#define GD32_HCLK                  (GD32_SYSCLK / 8)
#elif GD32_AHBPSC == GD32_AHBPSC_DIV16
#define GD32_HCLK                  (GD32_SYSCLK / 16)
#elif GD32_AHBPSC == GD32_AHBPSC_DIV64
#define GD32_HCLK                  (GD32_SYSCLK / 64)
#elif GD32_AHBPSC == GD32_AHBPSC_DIV128
#define GD32_HCLK                  (GD32_SYSCLK / 128)
#elif GD32_AHBPSC == GD32_AHBPSC_DIV256
#define GD32_HCLK                  (GD32_SYSCLK / 256)
#elif GD32_AHBPSC == GD32_AHBPSC_DIV512
#define GD32_HCLK                  (GD32_SYSCLK / 512)
#else
#error "invalid GD32_AHBPSC value specified"
#endif

/* AHB frequency check.*/
#if GD32_HCLK > GD32_SYSCLK_MAX
#error "GD32_HCLK exceeding maximum frequency (GD32_SYSCLK_MAX)"
#endif

/**
 * @brief   APB1 frequency.
 */
#if (GD32_APB1PSC == GD32_APB1PSC_DIV1) || defined(__DOXYGEN__)
#define GD32_PCLK1                 (GD32_HCLK / 1)
#elif GD32_APB1PSC == GD32_APB1PSC_DIV2
#define GD32_PCLK1                 (GD32_HCLK / 2)
#elif GD32_APB1PSC == GD32_APB1PSC_DIV4
#define GD32_PCLK1                 (GD32_HCLK / 4)
#elif GD32_APB1PSC == GD32_APB1PSC_DIV8
#define GD32_PCLK1                 (GD32_HCLK / 8)
#elif GD32_APB1PSC == GD32_APB1PSC_DIV16
#define GD32_PCLK1                 (GD32_HCLK / 16)
#else
#error "invalid GD32_APB1PSC value specified"
#endif

/* APB1 frequency check.*/
#if GD32_PCLK1 > GD32_PCLK1_MAX
#error "GD32_PCLK1 exceeding maximum frequency (GD32_PCLK1_MAX)"
#endif

/**
 * @brief   APB2 frequency.
 */
#if (GD32_APB2PSC == GD32_APB2PSC_DIV1) || defined(__DOXYGEN__)
#define GD32_PCLK2                 (GD32_HCLK / 1)
#elif GD32_APB2PSC == GD32_APB2PSC_DIV2
#define GD32_PCLK2                 (GD32_HCLK / 2)
#elif GD32_APB2PSC == GD32_APB2PSC_DIV4
#define GD32_PCLK2                 (GD32_HCLK / 4)
#elif GD32_APB2PSC == GD32_APB2PSC_DIV8
#define GD32_PCLK2                 (GD32_HCLK / 8)
#elif GD32_APB2PSC == GD32_APB2PSC_DIV16
#define GD32_PCLK2                 (GD32_HCLK / 16)
#else
#error "invalid GD32_APB2PSC value specified"
#endif

/* APB2 frequency check.*/
#if GD32_PCLK2 > GD32_PCLK2_MAX
#error "GD32_PCLK2 exceeding maximum frequency (GD32_PCLK2_MAX)"
#endif

/**
 * @brief   RTC clock.
 */
#if (GD32_RTCSRC == GD32_RTCSRC_LXTAL) || defined(__DOXYGEN__)
#define GD32_RTCCLK                GD32_LXTALCLK
#elif GD32_RTCSRC == GD32_RTCSRC_IRC40K
#define GD32_RTCCLK                GD32_IRC40KCLK
#elif GD32_RTCSRC == GD32_RTCSRC_HXTALDIV
#define GD32_RTCCLK                (GD32_HXTALCLK / 128)
#elif GD32_RTCSRC == GD32_RTCSRC_NOCLOCK
#define GD32_RTCCLK                0
#else
#error "invalid source selected for RTC clock"
#endif

/**
 * @brief   ADC frequency.
 */
#if (GD32_ADCPSC == GD32_ADCPSC_DIV2) || defined(__DOXYGEN__)
#define GD32_ADCCLK                (GD32_PCLK2 / 2)
#elif GD32_ADCPSC == GD32_ADCPSC_DIV4
#define GD32_ADCCLK                (GD32_PCLK2 / 4)
#elif GD32_ADCPSC == GD32_ADCPSC_DIV6
#define GD32_ADCCLK                (GD32_PCLK2 / 6)
#elif GD32_ADCPSC == GD32_ADCPSC_DIV8
#define GD32_ADCCLK                (GD32_PCLK2 / 8)
#elif GD32_ADCPSC == GD32_ADCPSC_DIV12
#define GD32_ADCCLK                (GD32_PCLK2 / 12)
#elif GD32_ADCPSC == GD32_ADCPSC_DIV16
#define GD32_ADCCLK                (GD32_PCLK2 / 16)
#else
#error "invalid GD32_ADCPSC value specified"
#endif

/* ADC frequency check.*/
#if GD32_ADCCLK > GD32_ADCCLK_MAX
#error "GD32_ADCCLK exceeding maximum frequency (GD32_ADCCLK_MAX)"
#endif

/**
 * @brief   USB frequency.
 */
#if (GD32_USBFSPSC == GD32_USBFSPSC_DIV1P5) || defined(__DOXYGEN__)
#define GD32_USBFSCLK                ((GD32_PLLCLKOUT * 2) / 3)
#elif (GD32_USBFSPSC == GD32_USBFSPSC_DIV1)
#define GD32_USBFSCLK                GD32_PLLCLKOUT
#elif (GD32_USBFSPSC == GD32_USBFSPSC_DIV2)
#define GD32_USBFSCLK                GD32_PLLCLKOUT / 2
#elif (GD32_USBFSPSC == GD32_USBFSPSC_DIV2P5)
#define GD32_USBFSCLK                ((GD32_PLLCLKOUT * 2) / 5)
#else
#error "invalid GD32_USBFSPSC value specified"
#endif

/**
 * @brief   Timers 2, 3, 4, 5, 6, 7 clock.
 */
#if (GD32_APB1PSC == GD32_APB1PSC_DIV1) || defined(__DOXYGEN__)
#define GD32_TIMCLK1               (GD32_PCLK1 * 1)
#else
#define GD32_TIMCLK1               (GD32_PCLK1 * 2)
#endif

/**
 * @brief   Timers 1 clock.
 */
#if (GD32_APB2PSC == GD32_APB2PSC_DIV1) || defined(__DOXYGEN__)
#define GD32_TIMCLK2               (GD32_PCLK2 * 1)
#else
#define GD32_TIMCLK2               (GD32_PCLK2 * 2)
#endif

/**
 * @brief   Flash settings.
 */
#if (GD32_HCLK <= 24000000) || defined(__DOXYGEN__)
#define GD32_FLASHBITS             0x00000010
#elif GD32_HCLK <= 48000000
#define GD32_FLASHBITS             0x00000011
#else
#define GD32_FLASHBITS             0x00000012
#endif
/** @} */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#include "gd32_registry.h"
#include "gd32vf103.h"

/* Various helpers.*/
#include "eclic.h"
#include "gd32_isr.h"
#include "gd32_dma.h"
#include "gd32_rcu.h"
#include "gd32_tim.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void gd32_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
