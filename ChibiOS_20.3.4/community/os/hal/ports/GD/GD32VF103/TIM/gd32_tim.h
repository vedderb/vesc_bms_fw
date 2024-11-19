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
 * @file    TIM/gd32_tim.h
 * @brief   GD32 TIM units common header.
 * @note    This file requires definitions from the GD32 header file.
 *
 * @addtogroup GD32_TIM
 * @{
 */

#ifndef GD32_TIM_H
#define GD32_TIM_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    TIM_CR1 register
 * @{
 */
#define GD32_TIM_CR1_CEN                   (1U << 0)
#define GD32_TIM_CR1_UDIS                  (1U << 1)
#define GD32_TIM_CR1_URS                   (1U << 2)
#define GD32_TIM_CR1_OPM                   (1U << 3)
#define GD32_TIM_CR1_DIR                   (1U << 4)

#define GD32_TIM_CR1_CMS_MASK              (3U << 5)
#define GD32_TIM_CR1_CMS(n)                ((n) << 5)

#define GD32_TIM_CR1_ARPE                  (1U << 7)

#define GD32_TIM_CR1_CKD_MASK              (3U << 8)
#define GD32_TIM_CR1_CKD(n)                ((n) << 8)

#define GD32_TIM_CR1_UIFREMAP              (1U << 11)
/** @} */

/**
 * @name    TIM_CR2 register
 * @{
 */
#define GD32_TIM_CR2_CCPC                  (1U << 0)
#define GD32_TIM_CR2_CCUS                  (1U << 2)
#define GD32_TIM_CR2_CCDS                  (1U << 3)

#define GD32_TIM_CR2_MMS_MASK              (7U << 4)
#define GD32_TIM_CR2_MMS(n)                ((n) << 4)

#define GD32_TIM_CR2_TI1S                  (1U << 7)
#define GD32_TIM_CR2_OIS1                  (1U << 8)
#define GD32_TIM_CR2_OIS1N                 (1U << 9)
#define GD32_TIM_CR2_OIS2                  (1U << 10)
#define GD32_TIM_CR2_OIS2N                 (1U << 11)
#define GD32_TIM_CR2_OIS3                  (1U << 12)
#define GD32_TIM_CR2_OIS3N                 (1U << 13)
#define GD32_TIM_CR2_OIS4                  (1U << 14)
#define GD32_TIM_CR2_OIS5                  (1U << 16)
#define GD32_TIM_CR2_OIS6                  (1U << 18)

#define GD32_TIM_CR2_MMS2_MASK             (15U << 20)
#define GD32_TIM_CR2_MMS2(n)               ((n) << 20)
/** @} */

/**
 * @name    TIM_SMCR register
 * @{
 */
#define GD32_TIM_SMCR_SMS_MASK             ((7U << 0) | (1U << 16))
#define GD32_TIM_SMCR_SMS(n)               ((((n) & 7) << 0) |             \
                                             (((n) >> 3) << 16))

#define GD32_TIM_SMCR_OCCS                 (1U << 3)

#define GD32_TIM_SMCR_TS_MASK              (7U << 4)
#define GD32_TIM_SMCR_TS(n)                ((n) << 4)

#define GD32_TIM_SMCR_MSM                  (1U << 7)

#define GD32_TIM_SMCR_ETF_MASK             (15U << 8)
#define GD32_TIM_SMCR_ETF(n)               ((n) << 8)

#define GD32_TIM_SMCR_ETPS_MASK            (3U << 12)
#define GD32_TIM_SMCR_ETPS(n)              ((n) << 12)

#define GD32_TIM_SMCR_ECE                  (1U << 14)
#define GD32_TIM_SMCR_ETP                  (1U << 15)
/** @} */

/**
 * @name    TIM_DIER register
 * @{
 */
#define GD32_TIM_DIER_UIE                  (1U << 0)
#define GD32_TIM_DIER_CC1IE                (1U << 1)
#define GD32_TIM_DIER_CC2IE                (1U << 2)
#define GD32_TIM_DIER_CC3IE                (1U << 3)
#define GD32_TIM_DIER_CC4IE                (1U << 4)
#define GD32_TIM_DIER_COMIE                (1U << 5)
#define GD32_TIM_DIER_TIE                  (1U << 6)
#define GD32_TIM_DIER_BIE                  (1U << 7)
#define GD32_TIM_DIER_UDE                  (1U << 8)
#define GD32_TIM_DIER_CC1DE                (1U << 9)
#define GD32_TIM_DIER_CC2DE                (1U << 10)
#define GD32_TIM_DIER_CC3DE                (1U << 11)
#define GD32_TIM_DIER_CC4DE                (1U << 12)
#define GD32_TIM_DIER_COMDE                (1U << 13)
#define GD32_TIM_DIER_TDE                  (1U << 14)

#define GD32_TIM_DIER_IRQ_MASK             (GD32_TIM_DIER_UIE   |         \
                                             GD32_TIM_DIER_CC1IE |         \
                                             GD32_TIM_DIER_CC2IE |         \
                                             GD32_TIM_DIER_CC3IE |         \
                                             GD32_TIM_DIER_CC4IE |         \
                                             GD32_TIM_DIER_COMIE |         \
                                             GD32_TIM_DIER_TIE   |         \
                                             GD32_TIM_DIER_BIE)

/** @} */

/**
 * @name    TIM_SR register
 * @{
 */
#define GD32_TIM_SR_UIF                    (1U << 0)
#define GD32_TIM_SR_CC1IF                  (1U << 1)
#define GD32_TIM_SR_CC2IF                  (1U << 2)
#define GD32_TIM_SR_CC3IF                  (1U << 3)
#define GD32_TIM_SR_CC4IF                  (1U << 4)
#define GD32_TIM_SR_COMIF                  (1U << 5)
#define GD32_TIM_SR_TIF                    (1U << 6)
#define GD32_TIM_SR_BIF                    (1U << 7)
#define GD32_TIM_SR_B2IF                   (1U << 8)
#define GD32_TIM_SR_CC1OF                  (1U << 9)
#define GD32_TIM_SR_CC2OF                  (1U << 10)
#define GD32_TIM_SR_CC3OF                  (1U << 11)
#define GD32_TIM_SR_CC4OF                  (1U << 12)
#define GD32_TIM_SR_CC5IF                  (1U << 16)
#define GD32_TIM_SR_CC6IF                  (1U << 17)
/** @} */

/**
 * @name    TIM_EGR register
 * @{
 */
#define GD32_TIM_EGR_UG                    (1U << 0)
#define GD32_TIM_EGR_CC1G                  (1U << 1)
#define GD32_TIM_EGR_CC2G                  (1U << 2)
#define GD32_TIM_EGR_CC3G                  (1U << 3)
#define GD32_TIM_EGR_CC4G                  (1U << 4)
#define GD32_TIM_EGR_COMG                  (1U << 5)
#define GD32_TIM_EGR_TG                    (1U << 6)
#define GD32_TIM_EGR_BG                    (1U << 7)
#define GD32_TIM_EGR_B2G                   (1U << 8)
/** @} */

/**
 * @name    TIM_CCMR1 register (output)
 * @{
 */
#define GD32_TIM_CCMR1_CC1S_MASK           (3U << 0)
#define GD32_TIM_CCMR1_CC1S(n)             ((n) << 0)

#define GD32_TIM_CCMR1_OC1FE               (1U << 2)
#define GD32_TIM_CCMR1_OC1PE               (1U << 3)

#define GD32_TIM_CCMR1_OC1M_MASK           ((7U << 4) | (1U << 16))
#define GD32_TIM_CCMR1_OC1M(n)             ((((n) & 7) << 4) |             \
                                             (((n) >> 3) << 16))

#define GD32_TIM_CCMR1_OC1CE               (1U << 7)

#define GD32_TIM_CCMR1_CC2S_MASK           (3U << 8)
#define GD32_TIM_CCMR1_CC2S(n)             ((n) << 8)

#define GD32_TIM_CCMR1_OC2FE               (1U << 10)
#define GD32_TIM_CCMR1_OC2PE               (1U << 11)

#define GD32_TIM_CCMR1_OC2M_MASK           ((7U << 12) | (1U << 24))
#define GD32_TIM_CCMR1_OC2M(n)             ((((n) & 7) << 12) |            \
                                             (((n) >> 3) << 24))

#define GD32_TIM_CCMR1_OC2CE               (1U << 15)
/** @} */

/**
 * @name    CCMR1 register (input)
 * @{
 */
#define GD32_TIM_CCMR1_IC1PSC_MASK         (3U << 2)
#define GD32_TIM_CCMR1_IC1PSC(n)           ((n) << 2)

#define GD32_TIM_CCMR1_IC1F_MASK           (15U << 4)
#define GD32_TIM_CCMR1_IC1F(n)             ((n) << 4)

#define GD32_TIM_CCMR1_IC2PSC_MASK         (3U << 10)
#define GD32_TIM_CCMR1_IC2PSC(n)           ((n) << 10)

#define GD32_TIM_CCMR1_IC2F_MASK           (15U << 12)
#define GD32_TIM_CCMR1_IC2F(n)             ((n) << 12)
/** @} */

/**
 * @name    TIM_CCMR2 register (output)
 * @{
 */
#define GD32_TIM_CCMR2_CC3S_MASK           (3U << 0)
#define GD32_TIM_CCMR2_CC3S(n)             ((n) << 0)

#define GD32_TIM_CCMR2_OC3FE               (1U << 2)
#define GD32_TIM_CCMR2_OC3PE               (1U << 3)

#define GD32_TIM_CCMR2_OC3M_MASK           ((7U << 4) | (1U << 16))
#define GD32_TIM_CCMR2_OC3M(n)             ((((n) & 7) << 4) |             \
                                             (((n) >> 3) << 16))

#define GD32_TIM_CCMR2_OC3CE               (1U << 7)

#define GD32_TIM_CCMR2_CC4S_MASK           (3U << 8)
#define GD32_TIM_CCMR2_CC4S(n)             ((n) << 8)

#define GD32_TIM_CCMR2_OC4FE               (1U << 10)
#define GD32_TIM_CCMR2_OC4PE               (1U << 11)

#define GD32_TIM_CCMR2_OC4M_MASK           ((7U << 12) | (1U << 24))
#define GD32_TIM_CCMR2_OC4M(n)             ((((n) & 7) << 12) |            \
                                             (((n) >> 3) << 24))

#define GD32_TIM_CCMR2_OC4CE               (1U << 15)
/** @} */

/**
 * @name    TIM_CCMR2 register (input)
 * @{
 */
#define GD32_TIM_CCMR2_IC3PSC_MASK         (3U << 2)
#define GD32_TIM_CCMR2_IC3PSC(n)           ((n) << 2)

#define GD32_TIM_CCMR2_IC3F_MASK           (15U << 4)
#define GD32_TIM_CCMR2_IC3F(n)             ((n) << 4)

#define GD32_TIM_CCMR2_IC4PSC_MASK         (3U << 10)
#define GD32_TIM_CCMR2_IC4PSC(n)           ((n) << 10)

#define GD32_TIM_CCMR2_IC4F_MASK           (15U << 12)
#define GD32_TIM_CCMR2_IC4F(n)             ((n) << 12)
/** @} */

/**
 * @name    TIM_CCER register
 * @{
 */
#define GD32_TIM_CCER_CC1E                 (1U << 0)
#define GD32_TIM_CCER_CC1P                 (1U << 1)
#define GD32_TIM_CCER_CC1NE                (1U << 2)
#define GD32_TIM_CCER_CC1NP                (1U << 3)
#define GD32_TIM_CCER_CC2E                 (1U << 4)
#define GD32_TIM_CCER_CC2P                 (1U << 5)
#define GD32_TIM_CCER_CC2NE                (1U << 6)
#define GD32_TIM_CCER_CC2NP                (1U << 7)
#define GD32_TIM_CCER_CC3E                 (1U << 8)
#define GD32_TIM_CCER_CC3P                 (1U << 9)
#define GD32_TIM_CCER_CC3NE                (1U << 10)
#define GD32_TIM_CCER_CC3NP                (1U << 11)
#define GD32_TIM_CCER_CC4E                 (1U << 12)
#define GD32_TIM_CCER_CC4P                 (1U << 13)
#define GD32_TIM_CCER_CC4NE                (1U << 14)
#define GD32_TIM_CCER_CC4NP                (1U << 15)
#define GD32_TIM_CCER_CC5E                 (1U << 16)
#define GD32_TIM_CCER_CC5P                 (1U << 17)
#define GD32_TIM_CCER_CC6E                 (1U << 20)
#define GD32_TIM_CCER_CC6P                 (1U << 21)
/** @} */

/**
 * @name    TIM_CNT register
 * @{
 */
#define GD32_TIM_CNT_UIFCPY                (1U << 31)
/** @} */

/**
 * @name    TIM_BDTR register
 * @{
 */
#define GD32_TIM_BDTR_DTG_MASK             (255U << 0)
#define GD32_TIM_BDTR_DTG(n)               ((n) << 0)

#define GD32_TIM_BDTR_LOCK_MASK            (3U << 8)
#define GD32_TIM_BDTR_LOCK(n)              ((n) << 8)

#define GD32_TIM_BDTR_OSSI                 (1U << 10)
#define GD32_TIM_BDTR_OSSR                 (1U << 11)
#define GD32_TIM_BDTR_BKE                  (1U << 12)
#define GD32_TIM_BDTR_BKP                  (1U << 13)
#define GD32_TIM_BDTR_AOE                  (1U << 14)
#define GD32_TIM_BDTR_MOE                  (1U << 15)

#define GD32_TIM_BDTR_BKF_MASK             (15U << 16)
#define GD32_TIM_BDTR_BKF(n)               ((n) << 16)
#define GD32_TIM_BDTR_BK2F_MASK            (15U << 20)
#define GD32_TIM_BDTR_BK2F(n)              ((n) << 20)

#define GD32_TIM_BDTR_BK2E                 (1U << 24)
#define GD32_TIM_BDTR_BK2P                 (1U << 25)
/** @} */

/**
 * @name    TIM_DCR register
 * @{
 */
#define GD32_TIM_DCR_DBA_MASK              (31U << 0)
#define GD32_TIM_DCR_DBA(n)                ((n) << 0)

#define GD32_TIM_DCR_DBL_MASK              (31U << 8)
#define GD32_TIM_DCR_DBL(n)                ((n) << 8)
/** @} */



/**
 * @name    TIM units references
 * @{
 */
#define GD32_TIM0      ((gd32_tim_t *)TIM0_BASE)
#define GD32_TIM1      ((gd32_tim_t *)TIM1_BASE)
#define GD32_TIM2      ((gd32_tim_t *)TIM2_BASE)
#define GD32_TIM3      ((gd32_tim_t *)TIM3_BASE)
#define GD32_TIM4      ((gd32_tim_t *)TIM4_BASE)
#define GD32_TIM5      ((gd32_tim_t *)TIM5_BASE)
#define GD32_TIM6      ((gd32_tim_t *)TIM6_BASE)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   GD32 TIM registers block.
 * @note    This is the most general known form, not all timers have
 *          necessarily all registers and bits.
 */
typedef struct {
  volatile uint32_t     CTL0;
  volatile uint32_t     CTL1;
  volatile uint32_t     SMCFG;
  volatile uint32_t     DMAINTEN;
  volatile uint32_t     INTF;
  volatile uint32_t     SWEVG;
  volatile uint32_t     CHCTL0;
  volatile uint32_t     CHCTL1;
  volatile uint32_t     CHCTL2;
  volatile uint32_t     CNT;
  volatile uint32_t     PSC;
  volatile uint32_t     CAR;
  volatile uint32_t     CREP;
  volatile uint32_t     CHCV[4];
  volatile uint32_t     CCHP;
  volatile uint32_t     DMACFG;
  volatile uint32_t     DMATB;
} gd32_tim_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* GD32_TIM_H */

/** @} */
