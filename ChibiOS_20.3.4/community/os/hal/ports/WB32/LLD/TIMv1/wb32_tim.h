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
 * @file    TIMv1/wb32_tim.h
 * @brief   WB32 TIM units common header.
 * @note    This file requires definitions from the ST WB32 header file.
 *
 * @addtogroup WB32_TIMv1
 * @{
 */

#ifndef WB32_TIM_H
#define WB32_TIM_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    TIM_CR1 register
 * @{
 */
#define WB32_TIM_CR1_CEN                       (1U << 0)
#define WB32_TIM_CR1_UDIS                      (1U << 1)
#define WB32_TIM_CR1_URS                       (1U << 2)
#define WB32_TIM_CR1_OPM                       (1U << 3)
#define WB32_TIM_CR1_DIR                       (1U << 4)

#define WB32_TIM_CR1_CMS_MASK                  (3U << 5)
#define WB32_TIM_CR1_CMS(n)                    ((n) << 5)

#define WB32_TIM_CR1_ARPE                      (1U << 7)

#define WB32_TIM_CR1_CKD_MASK                  (3U << 8)
#define WB32_TIM_CR1_CKD(n)                    ((n) << 8)

#define WB32_TIM_CR1_UIFREMAP                  (1U << 11)
/** @} */

/**
 * @name    TIM_CR2 register
 * @{
 */
#define WB32_TIM_CR2_CCPC                      (1U << 0)
#define WB32_TIM_CR2_CCUS                      (1U << 2)
#define WB32_TIM_CR2_CCDS                      (1U << 3)

#define WB32_TIM_CR2_MMS_MASK                  (7U << 4)
#define WB32_TIM_CR2_MMS(n)                    ((n) << 4)

#define WB32_TIM_CR2_TI1S                      (1U << 7)
#define WB32_TIM_CR2_OIS1                      (1U << 8)
#define WB32_TIM_CR2_OIS1N                     (1U << 9)
#define WB32_TIM_CR2_OIS2                      (1U << 10)
#define WB32_TIM_CR2_OIS2N                     (1U << 11)
#define WB32_TIM_CR2_OIS3                      (1U << 12)
#define WB32_TIM_CR2_OIS3N                     (1U << 13)
#define WB32_TIM_CR2_OIS4                      (1U << 14)
#define WB32_TIM_CR2_OIS5                      (1U << 16)
#define WB32_TIM_CR2_OIS6                      (1U << 18)

#define WB32_TIM_CR2_MMS2_MASK                 (15U << 20)
#define WB32_TIM_CR2_MMS2(n)                   ((n) << 20)
/** @} */

/**
 * @name    TIM_SMCR register
 * @{
 */
#define WB32_TIM_SMCR_SMS_MASK                 ((7U << 0) | (1U << 16))
#define WB32_TIM_SMCR_SMS(n)                   ((((n) & 7) << 0) |          \
                                                (((n) >> 3) << 16))

#define WB32_TIM_SMCR_OCCS                     (1U << 3)

#define WB32_TIM_SMCR_TS_MASK                  (7U << 4)
#define WB32_TIM_SMCR_TS(n)                    ((n) << 4)

#define WB32_TIM_SMCR_MSM                      (1U << 7)

#define WB32_TIM_SMCR_ETF_MASK                 (15U << 8)
#define WB32_TIM_SMCR_ETF(n)                   ((n) << 8)

#define WB32_TIM_SMCR_ETPS_MASK                (3U << 12)
#define WB32_TIM_SMCR_ETPS(n)                  ((n) << 12)

#define WB32_TIM_SMCR_ECE                      (1U << 14)
#define WB32_TIM_SMCR_ETP                      (1U << 15)
/** @} */

/**
 * @name    TIM_DIER register
 * @{
 */
#define WB32_TIM_DIER_UIE                      (1U << 0)
#define WB32_TIM_DIER_CC1IE                    (1U << 1)
#define WB32_TIM_DIER_CC2IE                    (1U << 2)
#define WB32_TIM_DIER_CC3IE                    (1U << 3)
#define WB32_TIM_DIER_CC4IE                    (1U << 4)
#define WB32_TIM_DIER_COMIE                    (1U << 5)
#define WB32_TIM_DIER_TIE                      (1U << 6)
#define WB32_TIM_DIER_BIE                      (1U << 7)
#define WB32_TIM_DIER_UDE                      (1U << 8)
#define WB32_TIM_DIER_CC1DE                    (1U << 9)
#define WB32_TIM_DIER_CC2DE                    (1U << 10)
#define WB32_TIM_DIER_CC3DE                    (1U << 11)
#define WB32_TIM_DIER_CC4DE                    (1U << 12)
#define WB32_TIM_DIER_COMDE                    (1U << 13)
#define WB32_TIM_DIER_TDE                      (1U << 14)

#define WB32_TIM_DIER_IRQ_MASK                 (WB32_TIM_DIER_UIE   |       \
                                                WB32_TIM_DIER_CC1IE |       \
                                                WB32_TIM_DIER_CC2IE |       \
                                                WB32_TIM_DIER_CC3IE |       \
                                                WB32_TIM_DIER_CC4IE |       \
                                                WB32_TIM_DIER_COMIE |       \
                                                WB32_TIM_DIER_TIE   |       \
                                                WB32_TIM_DIER_BIE)

/** @} */

/**
 * @name    TIM_SR register
 * @{
 */
#define WB32_TIM_SR_UIF                        (1U << 0)
#define WB32_TIM_SR_CC1IF                      (1U << 1)
#define WB32_TIM_SR_CC2IF                      (1U << 2)
#define WB32_TIM_SR_CC3IF                      (1U << 3)
#define WB32_TIM_SR_CC4IF                      (1U << 4)
#define WB32_TIM_SR_COMIF                      (1U << 5)
#define WB32_TIM_SR_TIF                        (1U << 6)
#define WB32_TIM_SR_BIF                        (1U << 7)
#define WB32_TIM_SR_B2IF                       (1U << 8)
#define WB32_TIM_SR_CC1OF                      (1U << 9)
#define WB32_TIM_SR_CC2OF                      (1U << 10)
#define WB32_TIM_SR_CC3OF                      (1U << 11)
#define WB32_TIM_SR_CC4OF                      (1U << 12)
#define WB32_TIM_SR_CC5IF                      (1U << 16)
#define WB32_TIM_SR_CC6IF                      (1U << 17)
/** @} */

/**
 * @name    TIM_EGR register
 * @{
 */
#define WB32_TIM_EGR_UG                        (1U << 0)
#define WB32_TIM_EGR_CC1G                      (1U << 1)
#define WB32_TIM_EGR_CC2G                      (1U << 2)
#define WB32_TIM_EGR_CC3G                      (1U << 3)
#define WB32_TIM_EGR_CC4G                      (1U << 4)
#define WB32_TIM_EGR_COMG                      (1U << 5)
#define WB32_TIM_EGR_TG                        (1U << 6)
#define WB32_TIM_EGR_BG                        (1U << 7)
#define WB32_TIM_EGR_B2G                       (1U << 8)
/** @} */

/**
 * @name    TIM_CCMR1 register (output)
 * @{
 */
#define WB32_TIM_CCMR1_CC1S_MASK               (3U << 0)
#define WB32_TIM_CCMR1_CC1S(n)                 ((n) << 0)

#define WB32_TIM_CCMR1_OC1FE                   (1U << 2)
#define WB32_TIM_CCMR1_OC1PE                   (1U << 3)

#define WB32_TIM_CCMR1_OC1M_MASK               ((7U << 4) | (1U << 16))
#define WB32_TIM_CCMR1_OC1M(n)                 ((((n) & 7) << 4) |          \
                                                (((n) >> 3) << 16))

#define WB32_TIM_CCMR1_OC1CE                   (1U << 7)

#define WB32_TIM_CCMR1_CC2S_MASK               (3U << 8)
#define WB32_TIM_CCMR1_CC2S(n)                 ((n) << 8)

#define WB32_TIM_CCMR1_OC2FE                   (1U << 10)
#define WB32_TIM_CCMR1_OC2PE                   (1U << 11)

#define WB32_TIM_CCMR1_OC2M_MASK               ((7U << 12) | (1U << 24))
#define WB32_TIM_CCMR1_OC2M(n)                 ((((n) & 7) << 12) |         \
                                                (((n) >> 3) << 24))

#define WB32_TIM_CCMR1_OC2CE                   (1U << 15)
/** @} */

/**
 * @name    CCMR1 register (input)
 * @{
 */
#define WB32_TIM_CCMR1_IC1PSC_MASK             (3U << 2)
#define WB32_TIM_CCMR1_IC1PSC(n)               ((n) << 2)

#define WB32_TIM_CCMR1_IC1F_MASK               (15U << 4)
#define WB32_TIM_CCMR1_IC1F(n)                 ((n) << 4)

#define WB32_TIM_CCMR1_IC2PSC_MASK             (3U << 10)
#define WB32_TIM_CCMR1_IC2PSC(n)               ((n) << 10)

#define WB32_TIM_CCMR1_IC2F_MASK               (15U << 12)
#define WB32_TIM_CCMR1_IC2F(n)                 ((n) << 12)
/** @} */

/**
 * @name    TIM_CCMR2 register (output)
 * @{
 */
#define WB32_TIM_CCMR2_CC3S_MASK               (3U << 0)
#define WB32_TIM_CCMR2_CC3S(n)                 ((n) << 0)

#define WB32_TIM_CCMR2_OC3FE                   (1U << 2)
#define WB32_TIM_CCMR2_OC3PE                   (1U << 3)

#define WB32_TIM_CCMR2_OC3M_MASK               ((7U << 4) | (1U << 16))
#define WB32_TIM_CCMR2_OC3M(n)                 ((((n) & 7) << 4) |          \
                                                (((n) >> 3) << 16))

#define WB32_TIM_CCMR2_OC3CE                   (1U << 7)

#define WB32_TIM_CCMR2_CC4S_MASK               (3U << 8)
#define WB32_TIM_CCMR2_CC4S(n)                 ((n) << 8)

#define WB32_TIM_CCMR2_OC4FE                   (1U << 10)
#define WB32_TIM_CCMR2_OC4PE                   (1U << 11)

#define WB32_TIM_CCMR2_OC4M_MASK               ((7U << 12) | (1U << 24))
#define WB32_TIM_CCMR2_OC4M(n)                 ((((n) & 7) << 12) |         \
                                                (((n) >> 3) << 24))

#define WB32_TIM_CCMR2_OC4CE                   (1U << 15)
/** @} */

/**
 * @name    TIM_CCMR2 register (input)
 * @{
 */
#define WB32_TIM_CCMR2_IC3PSC_MASK             (3U << 2)
#define WB32_TIM_CCMR2_IC3PSC(n)               ((n) << 2)

#define WB32_TIM_CCMR2_IC3F_MASK               (15U << 4)
#define WB32_TIM_CCMR2_IC3F(n)                 ((n) << 4)

#define WB32_TIM_CCMR2_IC4PSC_MASK             (3U << 10)
#define WB32_TIM_CCMR2_IC4PSC(n)               ((n) << 10)

#define WB32_TIM_CCMR2_IC4F_MASK               (15U << 12)
#define WB32_TIM_CCMR2_IC4F(n)                 ((n) << 12)
/** @} */

/**
 * @name    TIM_CCER register
 * @{
 */
#define WB32_TIM_CCER_CC1E                     (1U << 0)
#define WB32_TIM_CCER_CC1P                     (1U << 1)
#define WB32_TIM_CCER_CC1NE                    (1U << 2)
#define WB32_TIM_CCER_CC1NP                    (1U << 3)
#define WB32_TIM_CCER_CC2E                     (1U << 4)
#define WB32_TIM_CCER_CC2P                     (1U << 5)
#define WB32_TIM_CCER_CC2NE                    (1U << 6)
#define WB32_TIM_CCER_CC2NP                    (1U << 7)
#define WB32_TIM_CCER_CC3E                     (1U << 8)
#define WB32_TIM_CCER_CC3P                     (1U << 9)
#define WB32_TIM_CCER_CC3NE                    (1U << 10)
#define WB32_TIM_CCER_CC3NP                    (1U << 11)
#define WB32_TIM_CCER_CC4E                     (1U << 12)
#define WB32_TIM_CCER_CC4P                     (1U << 13)
#define WB32_TIM_CCER_CC4NE                    (1U << 14)
#define WB32_TIM_CCER_CC4NP                    (1U << 15)
#define WB32_TIM_CCER_CC5E                     (1U << 16)
#define WB32_TIM_CCER_CC5P                     (1U << 17)
#define WB32_TIM_CCER_CC6E                     (1U << 20)
#define WB32_TIM_CCER_CC6P                     (1U << 21)
/** @} */

/**
 * @name    TIM_CNT register
 * @{
 */
#define WB32_TIM_CNT_UIFCPY                    (1U << 31)
/** @} */

/**
 * @name    TIM_BDTR register
 * @{
 */
#define WB32_TIM_BDTR_DTG_MASK                 (255U << 0)
#define WB32_TIM_BDTR_DTG(n)                   ((n) << 0)

#define WB32_TIM_BDTR_LOCK_MASK                (3U << 8)
#define WB32_TIM_BDTR_LOCK(n)                  ((n) << 8)

#define WB32_TIM_BDTR_OSSI                     (1U << 10)
#define WB32_TIM_BDTR_OSSR                     (1U << 11)
#define WB32_TIM_BDTR_BKE                      (1U << 12)
#define WB32_TIM_BDTR_BKP                      (1U << 13)
#define WB32_TIM_BDTR_AOE                      (1U << 14)
#define WB32_TIM_BDTR_MOE                      (1U << 15)

#define WB32_TIM_BDTR_BKF_MASK                 (15U << 16)
#define WB32_TIM_BDTR_BKF(n)                   ((n) << 16)
#define WB32_TIM_BDTR_BK2F_MASK                (15U << 20)
#define WB32_TIM_BDTR_BK2F(n)                  ((n) << 20)

#define WB32_TIM_BDTR_BK2E                     (1U << 24)
#define WB32_TIM_BDTR_BK2P                     (1U << 25)
/** @} */

/**
 * @name    TIM_DCR register
 * @{
 */
#define WB32_TIM_DCR_DBA_MASK                  (31U << 0)
#define WB32_TIM_DCR_DBA(n)                    ((n) << 0)

#define WB32_TIM_DCR_DBL_MASK                  (31U << 8)
#define WB32_TIM_DCR_DBL(n)                    ((n) << 8)
/** @} */

/**
 * @name    TIM16_OR register
 * @{
 */
#define WB32_TIM16_OR_TI1_RMP_MASK             (3U << 6)
#define WB32_TIM16_OR_TI1_RMP(n)               ((n) << 6)
/** @} */

/**
 * @name    TIM_OR register
 * @{
 */
#define WB32_TIM_OR_ETR_RMP_MASK               (15U << 0)
#define WB32_TIM_OR_ETR_RMP(n)                 ((n) << 0)
/** @} */

/**
 * @name    TIM_CCMR3 register
 * @{
 */
#define WB32_TIM_CCMR3_OC5FE                   (1U << 2)
#define WB32_TIM_CCMR3_OC5PE                   (1U << 3)

#define WB32_TIM_CCMR3_OC5M_MASK               ((7U << 4) | (1U << 16))
#define WB32_TIM_CCMR3_OC5M(n)                 ((((n) & 7) << 4) |          \
                                                (((n) >> 2) << 16))

#define WB32_TIM_CCMR3_OC5CE                   (1U << 7)

#define WB32_TIM_CCMR3_OC6FE                   (1U << 10)
#define WB32_TIM_CCMR3_OC6PE                   (1U << 11)

#define WB32_TIM_CCMR3_OC6M_MASK               ((7U << 12) | (1U << 24))
#define WB32_TIM_CCMR3_OC6M(n)                 ((((n) & 7) << 12) |         \
                                                (((n) >> 2) << 24))

#define WB32_TIM_CCMR3_OC6CE                   (1U << 15)
/** @} */

/**
 * @name    LPTIM_ISR register
 * @{
 */
#define WB32_LPTIM_ISR_CMPM                    (1U << 0)
#define WB32_LPTIM_ISR_ARRM                    (1U << 1)
#define WB32_LPTIM_ISR_EXTTRIG                 (1U << 2)
#define WB32_LPTIM_ISR_CMPOK                   (1U << 3)
#define WB32_LPTIM_ISR_ARROK                   (1U << 4)
#define WB32_LPTIM_ISR_UP                      (1U << 5)
#define WB32_LPTIM_ISR_DOWN                    (1U << 6)
/** @} */

/**
 * @name    LPTIM_ICR register
 * @{
 */
#define WB32_LPTIM_ICR_CMPMCF                  (1U << 0)
#define WB32_LPTIM_ICR_ARRMCF                  (1U << 1)
#define WB32_LPTIM_ICR_EXTTRIGCF               (1U << 2)
#define WB32_LPTIM_ICR_CMPOKCF                 (1U << 3)
#define WB32_LPTIM_ICR_ARROKCF                 (1U << 4)
#define WB32_LPTIM_ICR_UPCF                    (1U << 5)
#define WB32_LPTIM_ICR_DOWNCF                  (1U << 6)
/** @} */

/**
 * @name    LPTIM_IER register
 * @{
 */
#define WB32_LPTIM_IER_CMPMIE                  (1U << 0)
#define WB32_LPTIM_IER_ARRMIE                  (1U << 1)
#define WB32_LPTIM_IER_EXTTRIGIE               (1U << 2)
#define WB32_LPTIM_IER_CMPOKIE                 (1U << 3)
#define WB32_LPTIM_IER_ARROKIE                 (1U << 4)
#define WB32_LPTIM_IER_UPIE                    (1U << 5)
#define WB32_LPTIM_IER_DOWNIE                  (1U << 6)
/** @} */

/**
 * @name    LPTIM_CFGR register
 * @{
 */
#define WB32_LPTIM_CFGR_CKSEL                  (1U << 0)
#define WB32_LPTIM_CFGR_CKPOL_MASK             (3U << 1)
#define WB32_LPTIM_CFGR_CKPOL(n)               ((n) << 1)
#define WB32_LPTIM_CFGR_CKFLT_MASK             (3U << 3)
#define WB32_LPTIM_CFGR_CKFLT(n)               ((n) << 3)
#define WB32_LPTIM_CFGR_TRGFLT_MASK            (3U << 6)
#define WB32_LPTIM_CFGR_TRGFLT(n)              ((n) << 6)
#define WB32_LPTIM_CFGR_PRESC_MASK             (7U << 9)
#define WB32_LPTIM_CFGR_PRESC(n)               ((n) << 9)
#define WB32_LPTIM_CFGR_TRIGSEL_MASK           (7U << 13)
#define WB32_LPTIM_CFGR_TRIGSEL(n)             ((n) << 13)
#define WB32_LPTIM_CFGR_TRIGEN_MASK            (3U << 17)
#define WB32_LPTIM_CFGR_TRIGEN(n)              ((n) << 17)
#define WB32_LPTIM_CFGR_TIMOUT                 (1U << 19)
#define WB32_LPTIM_CFGR_WAVE                   (1U << 20)
#define WB32_LPTIM_CFGR_WAVPOL                 (1U << 21)
#define WB32_LPTIM_CFGR_PRELOAD                (1U << 22)
#define WB32_LPTIM_CFGR_COUNTMODE              (1U << 23)
#define WB32_LPTIM_CFGR_ENC                    (1U << 24)
/** @} */

/**
 * @name    LPTIM_CR register
 * @{
 */
#define WB32_LPTIM_CR_ENABLE                   (1U << 0)
#define WB32_LPTIM_CR_SNGSTRT                  (1U << 1)
#define WB32_LPTIM_CR_CNTSTRT                  (1U << 2)
/** @} */

/**
 * @name    LPTIM_OR register
 * @{
 */
#define WB32_LPTIM_OR_0                        (1U << 0)
#define WB32_LPTIM_OR_1                        (1U << 1)
/** @} */

/**
 * @name    TIM units references
 * @{
 */
#define WB32_TIM1                              ((wb32_tim_t *)TIM1_BASE)
#define WB32_TIM2                              ((wb32_tim_t *)TIM2_BASE)
#define WB32_TIM3                              ((wb32_tim_t *)TIM3_BASE)
#define WB32_TIM4                              ((wb32_tim_t *)TIM4_BASE)

#define WB32_LPTIM1                            ((wb32_lptim_t *)LPTIM1_BASE)
#define WB32_LPTIM2                            ((wb32_lptim_t *)LPTIM2_BASE)
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
 * @brief   WB32 TIM registers block.
 * @note    This is the most general known form, not all timers have
 *          necessarily all registers and bits.
 */
typedef struct {
  volatile uint32_t     CR1;
  volatile uint32_t     CR2;
  volatile uint32_t     SMCR;
  volatile uint32_t     DIER;
  volatile uint32_t     SR;
  volatile uint32_t     EGR;
  volatile uint32_t     CCMR1;
  volatile uint32_t     CCMR2;
  volatile uint32_t     CCER;
  volatile uint32_t     CNT;
  volatile uint32_t     PSC;
  volatile uint32_t     ARR;
  volatile uint32_t     RCR;
  volatile uint32_t     CCR[4];
  volatile uint32_t     BDTR;
  volatile uint32_t     DCR;
  volatile uint32_t     DMAR;
  volatile uint32_t     OR;
  volatile uint32_t     CCMR3;
  volatile uint32_t     CCXR[2];
} wb32_tim_t;

/**
 * @brief   WB32 LPTIM registers block.
 * @note    This is the most general known form, not all timers have
 *          necessarily all registers and bits.
 */
typedef struct {
  volatile uint32_t     ISR;
  volatile uint32_t     ICR;
  volatile uint32_t     IER;
  volatile uint32_t     CFGR;
  volatile uint32_t     CR;
  volatile uint32_t     CMP;
  volatile uint32_t     ARR;
  volatile uint32_t     CNT;
  volatile uint32_t     OR;
} wb32_lptim_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* WB32_TIM_H */

/** @} */
