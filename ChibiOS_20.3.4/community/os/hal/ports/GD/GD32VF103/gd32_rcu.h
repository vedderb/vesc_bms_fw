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
 * @file    GD32VF103/gd32_rcu.h
 * @brief   RCU helper driver header.
 * @note    This file requires definitions from the ST header file
 *          @p gd32vf103.h.
 *
 * @addtogroup GD32VF103_RCU
 * @{
 */

#ifndef GD32_RCU_H
#define GD32_RCU_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Generic RCU operations
 * @{
 */
/**
 * @brief   Enables the clock of one or more peripheral on the APB1 bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      APB1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableAPB1(mask, lp) {                                           \
  RCU->APB1EN |= (mask);                                                   \
  (void)RCU->APB1EN;                                                       \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      APB1 peripherals mask
 *
 * @api
 */
#define rcuDisableAPB1(mask) {                                              \
  RCU->APB1EN &= ~(mask);                                                  \
  (void)RCU->APB1EN;                                                       \
}

/**
 * @brief   Resets one or more peripheral on the APB1 bus.
 *
 * @param[in] mask      APB1 peripherals mask
 *
 * @api
 */
#define rcuResetAPB1(mask) {                                                \
  RCU->APB1RST |= (mask);                                                  \
  RCU->APB1RST &= ~(mask);                                                 \
  (void)RCU->APB1RST;                                                      \
}

/**
 * @brief   Enables the clock of one or more peripheral on the APB2 bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      APB2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableAPB2(mask, lp) {                                           \
  RCU->APB2EN |= (mask);                                                   \
  (void)RCU->APB2EN;                                                       \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB2 bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      APB2 peripherals mask
 *
 * @api
 */
#define rcuDisableAPB2(mask) {                                              \
  RCU->APB2EN &= ~(mask);                                                  \
  (void)RCU->APB2EN;                                                       \
}

/**
 * @brief   Resets one or more peripheral on the APB2 bus.
 *
 * @param[in] mask      APB2 peripherals mask
 *
 * @api
 */
#define rcuResetAPB2(mask) {                                                \
  RCU->APB2RST |= (mask);                                                  \
  RCU->APB2RST &= ~(mask);                                                 \
  (void)RCU->APB2RST;                                                      \
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      AHB peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableAHB(mask, lp) {                                            \
  RCU->AHBEN |= (mask);                                                    \
  (void)RCU->AHBEN;                                                        \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB bus.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] mask      AHB peripherals mask
 *
 * @api
 */
#define rcuDisableAHB(mask) {                                               \
  RCU->AHBEN &= ~(mask);                                                   \
  (void)RCU->AHBEN;                                                        \
}

/**
 * @brief   Resets one or more peripheral on the AHB bus.
 *
 * @param[in] mask      AHB peripherals mask
 *
 * @api
 */
#define rcuResetAHB(mask) {                                                 \
  RCU->AHBRST |= (mask);                                                   \
  RCU->AHBRST &= ~(mask);                                                  \
  (void)RCU->AHBRST;                                                       \
}
/** @} */

/**
 * @name    ADC peripherals specific RCU operations
 * @{
 */
/**
 * @brief   Enables the ADC0 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableADC0(lp) rcuEnableAPB2(RCU_APB2EN_ADC0EN, lp)

/**
 * @brief   Disables the ADC0 peripheral clock.
 *
 * @api
 */
#define rcuDisableADC0() rcuDisableAPB2(RCU_APB2EN_ADC0EN)

/**
 * @brief   Resets the ADC0 peripheral.
 *
 * @api
 */
#define rcuResetADC0() rcuResetAPB2(RCU_APB2RST_ADC0RST)
/** @} */

/**
 * @name    DAC peripheral specific RCU operations
 * @{
 */
/**
 * @brief   Enables the DAC peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableDAC(lp) rcuEnableAPB1(RCU_APB1EN_DACEN, lp)

/**
 * @brief   Disables the DAC peripheral clock.
 *
 * @api
 */
#define rcuDisableDAC() rcuDisableAPB1(RCU_APB1EN_DACEN)

/**
 * @brief   Resets the DAC peripheral.
 *
 * @api
 */
#define rcuResetDAC() rcuResetAPB1(RCU_APB1RST_DACRST)
/** @} */

/**
 * @name    Backup domain interface specific RCU operations
 * @{
 */
/**
 * @brief   Enables the BKP interface clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableBKPInterface(lp) rcuEnableAPB1((RCU_APB1EN_BKPIEN), lp)

/**
 * @brief   Disables BKP interface clock.
 *
 * @api
 */
#define rcuDisableBKPInterface() rcuDisableAPB1(RCU_APB1EN_BKPIEN)

/**
 * @brief   Resets the Backup Domain interface.
 *
 * @api
 */
#define rcuResetBKPInterface() rcuResetAPB1(RCU_APB1EN_BKPRST)

/**
 * @brief   Resets the entire Backup Domain.
 *
 * @api
 */
#define rcuResetBKP() (RCU->BDCTL |= RCU_BDCTL_BKPRST)
/** @} */

/**
 * @name    PMU interface specific RCU operations
 * @{
 */
/**
 * @brief   Enables the PMU interface clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnablePMUInterface(lp) rcuEnableAPB1(RCU_APB1EN_PMUEN, lp)

/**
 * @brief   Disables PMU interface clock.
 *
 * @api
 */
#define rcuDisablePMUInterface() rcuDisableAPB1(RCU_APB1EN_PMUEN)

/**
 * @brief   Resets the PMU interface.
 *
 * @api
 */
#define rcuResetPMUInterface() rcuResetAPB1(RCU_APB1RST_PMURST)
/** @} */

/**
 * @name    CAN peripherals specific RCU operations
 * @{
 */
/**
 * @brief   Enables the CAN0 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableCAN0(lp) rcuEnableAPB1(RCU_APB1EN_CAN0EN, lp)

/**
 * @brief   Disables the CAN0 peripheral clock.
 *
 * @api
 */
#define rcuDisableCAN0() rcuDisableAPB1(RCU_APB1EN_CAN0EN)

/**
 * @brief   Resets the CAN0 peripheral.
 *
 * @api
 */
#define rcuResetCAN0() rcuResetAPB1(RCU_APB1RST_CAN0RST)

/**
 * @brief   Enables the CAN1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableCAN1(lp) rcuEnableAPB1(RCU_APB1EN_CAN1EN, lp)

/**
 * @brief   Disables the CAN1 peripheral clock.
 *
 * @api
 */
#define rcuDisableCAN1() rcuDisableAPB1(RCU_APB1EN_CAN1EN)

/**
 * @brief   Resets the CAN1 peripheral.
 *
 * @api
 */
#define rcuResetCAN1() rcuResetAPB1(RCU_APB1RST_CAN1RST)
/** @} */

/**
 * @name    CRC peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the CRC peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableCRC(lp) rcuEnableAHB(RCU_AHBEN_CRCEN, lp)

/**
 * @brief   Disables the CRC peripheral clock.
 *
 * @api
 */
#define rcuDisableCRC() rcuDisableAHB(RCU_AHBEN_CRCEN)

/** @} */

/**
 * @name    DMA peripherals specific RCU operations
 * @{
 */
/**
 * @brief   Enables the DMA0 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableDMA0(lp) rcuEnableAHB(RCU_AHBEN_DMA0EN, lp)

/**
 * @brief   Disables the DMA0 peripheral clock.
 *
 * @api
 */
#define rcuDisableDMA0() rcuDisableAHB(RCU_AHBEN_DMA0EN)

/**
 * @brief   Resets the DMA0 peripheral.
 * @note    Not supported in this family, does nothing.
 *
 * @api
 */
#define rcuResetDMA0()

/**
 * @brief   Enables the DMA1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableDMA1(lp) rcuEnableAHB(RCU_AHBEN_DMA1EN, lp)

/**
 * @brief   Disables the DMA1 peripheral clock.
 *
 * @api
 */
#define rcuDisableDMA1() rcuDisableAHB(RCU_AHBEN_DMA1EN)

/**
 * @brief   Resets the DMA0 peripheral.
 * @note    Not supported in this family, does nothing.
 *
 * @api
 */
#define rcuResetDMA1()
/** @} */

/**
 * @name    I2C peripherals specific RCU operations
 * @{
 */
/**
 * @brief   Enables the I2C0 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableI2C0(lp) rcuEnableAPB1(RCU_APB1EN_I2C0EN, lp)

/**
 * @brief   Disables the I2C0 peripheral clock.
 *
 * @api
 */
#define rcuDisableI2C0() rcuDisableAPB1(RCU_APB1EN_I2C0EN)

/**
 * @brief   Resets the I2C0 peripheral.
 *
 * @api
 */
#define rcuResetI2C0() rcuResetAPB1(RCU_APB1RST_I2C0RST)

/**
 * @brief   Enables the I2C1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableI2C1(lp) rcuEnableAPB1(RCU_APB1EN_I2C1EN, lp)

/**
 * @brief   Disables the I2C1 peripheral clock.
 *
 * @api
 */
#define rcuDisableI2C1() rcuDisableAPB1(RCU_APB1EN_I2C1EN)

/**
 * @brief   Resets the I2C1 peripheral.
 *
 * @api
 */
#define rcuResetI2C1() rcuResetAPB1(RCU_APB1RST_I2C1RST)
/** @} */

/**
 * @name    OTG peripherals specific RCU operations
 * @{
 */
/**
 * @brief   Enables the USBFS peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableUSBFS(lp) rcuEnableAHB(RCU_AHBEN_USBFSEN, lp)

/**
 * @brief   Disables the USBFS peripheral clock.
 *
 * @api
 */
#define rcuDisableUSBFS() rcuDisableAHB(RCU_AHBEN_USBFSEN)

/**
 * @brief   Resets the USBFS peripheral.
 *
 * @api
 */
#define rcuResetUSBFS() rcuResetAHB(RCU_AHBRST_USBFSRST)
/** @} */

/**
 * @name    SPI peripherals specific RCU operations
 * @{
 */
/**
 * @brief   Enables the SPI0 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableSPI0(lp) rcuEnableAPB2(RCU_APB2EN_SPI0EN, lp)

/**
 * @brief   Disables the SPI0 peripheral clock.
 *
 * @api
 */
#define rcuDisableSPI0() rcuDisableAPB2(RCU_APB2EN_SPI0EN)

/**
 * @brief   Resets the SPI0 peripheral.
 *
 * @api
 */
#define rcuResetSPI0() rcuResetAPB2(RCU_APB2RST_SPI0RST)

/**
 * @brief   Enables the SPI1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableSPI1(lp) rcuEnableAPB1(RCU_APB1EN_SPI1EN, lp)

/**
 * @brief   Disables the SPI1 peripheral clock.
 *
 * @api
 */
#define rcuDisableSPI1() rcuDisableAPB1(RCU_APB1EN_SPI1EN)

/**
 * @brief   Resets the SPI1 peripheral.
 *
 * @api
 */
#define rcuResetSPI1() rcuResetAPB1(RCU_APB1RST_SPI1RST)

/**
 * @brief   Enables the SPI2 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableSPI2(lp) rcuEnableAPB1(RCU_APB1EN_SPI2EN, lp)

/**
 * @brief   Disables the SPI2 peripheral clock.
 *
 * @api
 */
#define rcuDisableSPI2() rcuDisableAPB1(RCU_APB1EN_SPI2EN)

/**
 * @brief   Resets the SPI2 peripheral.
 *
 * @api
 */
#define rcuResetSPI2() rcuResetAPB1(RCU_APB1RST_SPI2RST)
/** @} */

/**
 * @name    TIM peripherals specific RCU operations
 * @{
 */
/**
 * @brief   Enables the TIM0 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableTIM0(lp) rcuEnableAPB2(RCU_APB2EN_TIMER0EN, lp)

/**
 * @brief   Disables the TIM0 peripheral clock.
 *
 * @api
 */
#define rcuDisableTIM0() rcuDisableAPB2(RCU_APB2EN_TIMER0EN)

/**
 * @brief   Resets the TIM0 peripheral.
 *
 * @api
 */
#define rcuResetTIM0() rcuResetAPB2(RCU_APB2RST_TIMER0RST)

/**
 * @brief   Enables the TIM1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableTIM1(lp) rcuEnableAPB1(RCU_APB1EN_TIMER1EN, lp)

/**
 * @brief   Disables the TIM1 peripheral clock.
 *
 * @api
 */
#define rcuDisableTIM1() rcuDisableAPB1(RCU_APB1EN_TIMER1EN)

/**
 * @brief   Resets the TIM1 peripheral.
 *
 * @api
 */
#define rcuResetTIM1() rcuResetAPB1(RCU_APB1RST_TIMER1RST)

/**
 * @brief   Enables the TIM2 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableTIM2(lp) rcuEnableAPB1(RCU_APB1EN_TIMER2EN, lp)

/**
 * @brief   Disables the TIM2 peripheral clock.
 *
 * @api
 */
#define rcuDisableTIM2() rcuDisableAPB1(RCU_APB1EN_TIMER2EN)

/**
 * @brief   Resets the TIM2 peripheral.
 *
 * @api
 */
#define rcuResetTIM2() rcuResetAPB1(RCU_APB1RST_TIMER2RST)

/**
 * @brief   Enables the TIM3 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableTIM3(lp) rcuEnableAPB1(RCU_APB1EN_TIMER3EN, lp)

/**
 * @brief   Disables the TIM3 peripheral clock.
 *
 * @api
 */
#define rcuDisableTIM3() rcuDisableAPB1(RCU_APB1EN_TIMER3EN)

/**
 * @brief   Resets the TIM3 peripheral.
 *
 * @api
 */
#define rcuResetTIM3() rcuResetAPB1(RCU_APB1RST_TIMER3RST)

/**
 * @brief   Enables the TIM4 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableTIM4(lp) rcuEnableAPB1(RCU_APB1EN_TIMER4EN, lp)

/**
 * @brief   Disables the TIM4 peripheral clock.
 *
 * @api
 */
#define rcuDisableTIM4() rcuDisableAPB1(RCU_APB1EN_TIMER4EN)

/**
 * @brief   Resets the TIM4 peripheral.
 *
 * @api
 */
#define rcuResetTIM4() rcuResetAPB1(RCU_APB1RST_TIMER4RST)

/**
 * @brief   Enables the TIM5 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableTIM5(lp) rcuEnableAPB1(RCU_APB1EN_TIMER5EN, lp)

/**
 * @brief   Disables the TIM5 peripheral clock.
 *
 * @api
 */
#define rcuDisableTIM5() rcuDisableAPB1(RCU_APB1EN_TIMER5EN)

/**
 * @brief   Resets the TIM5 peripheral.
 *
 * @api
 */
#define rcuResetTIM5() rcuResetAPB1(RCU_APB1RST_TIMER5RST)

/**
 * @brief   Enables the TIM6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableTIM6(lp) rcuEnableAPB1(RCU_APB1EN_TIMER6EN, lp)

/**
 * @brief   Disables the TIM6 peripheral clock.
 *
 * @api
 */
#define rcuDisableTIM6() rcuDisableAPB1(RCU_APB1EN_TIMER6EN)

/**
 * @brief   Resets the TIM6 peripheral.
 *
 * @api
 */
#define rcuResetTIM6() rcuResetAPB1(RCU_APB1RST_TIMER6RST)


/** @} */

/**
 * @name    USART/UART peripherals specific RCU operations
 * @{
 */
/**
 * @brief   Enables the USART0 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableUSART0(lp) rcuEnableAPB2(RCU_APB2EN_USART0EN, lp)

/**
 * @brief   Disables the USART0 peripheral clock.
 *
 * @api
 */
#define rcuDisableUSART0() rcuDisableAPB2(RCU_APB2EN_USART0EN)

/**
 * @brief   Resets the USART0 peripheral.
 *
 * @api
 */
#define rcuResetUSART0() rcuResetAPB2(RCU_APB2RST_USART0RST)

/**
 * @brief   Enables the USART1 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableUSART1(lp) rcuEnableAPB1(RCU_APB1EN_USART1EN, lp)

/**
 * @brief   Disables the USART1 peripheral clock.
 *
 * @api
 */
#define rcuDisableUSART1() rcuDisableAPB1(RCU_APB1EN_USART1EN)

/**
 * @brief   Resets the USART1 peripheral.
 *
 * @api
 */
#define rcuResetUSART1() rcuResetAPB1(RCU_APB1RST_USART1RST)

/**
 * @brief   Enables the USART2 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableUSART2(lp) rcuEnableAPB1(RCU_APB1EN_USART2EN, lp)

/**
 * @brief   Disables the USART2 peripheral clock.
 *
 * @api
 */
#define rcuDisableUSART2() rcuDisableAPB1(RCU_APB1EN_USART2EN)

/**
 * @brief   Resets the USART2 peripheral.
 *
 * @api
 */
#define rcuResetUSART2() rcuResetAPB1(RCU_APB1RST_USART2RST)

/**
 * @brief   Enables the UART3 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableUART3(lp) rcuEnableAPB1(RCU_APB1EN_UART3EN, lp)

/**
 * @brief   Disables the UART3 peripheral clock.
 *
 * @api
 */
#define rcuDisableUART3() rcuDisableAPB1(RCU_APB1EN_UART3EN)

/**
 * @brief   Resets the UART3 peripheral.
 *
 * @api
 */
#define rcuResetUART3() rcuResetAPB1(RCU_APB1RST_UART3RST)

/**
 * @brief   Enables the UART4 peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableUART4(lp) rcuEnableAPB1(RCU_APB1EN_UART4EN, lp)

/**
 * @brief   Disables the UART4 peripheral clock.
 *
 * @api
 */
#define rcuDisableUART4() rcuDisableAPB1(RCU_APB1EN_UART4EN)

/**
 * @brief   Resets the UART4 peripheral.
 *
 * @api
 */
#define rcuResetUART4() rcuResetAPB1(RCU_APB1RST_UART4RST)
/** @} */

/**
 * @name    USB peripheral specific RCU operations
 * @{
 */
/**
 * @brief   Enables the USB peripheral clock.
 * @note    The @p lp parameter is ignored in this family.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableUSB(lp) rcuEnableAPB1(RCU_APB1EN_USBEN, lp)

/**
 * @brief   Disables the USB peripheral clock
 *
 * @api
 */
#define rcuDisableUSB() rcuDisableAPB1(RCU_APB1EN_USBEN)

/**
 * @brief   Resets the USB peripheral.
 *
 * @api
 */
#define rcuResetUSB() rcuResetAPB1(RCU_APB1RST_USBRST)
/** @} */

/**
 * @name    EXMC peripherals specific RCU operations
 * @{
 */
/**
 * @brief   Enables the EXMC peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rcuEnableEXMC(lp) rcuEnableAHB(RCU_AHBEN_EXMCEN, lp)

/**
 * @brief   Disables the EXMC peripheral clock.
 *
 * @api
 */
#define rcuDisableEXMC() rcuDisableAHB(RCU_AHBEN_EXMCEN)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#endif /* GD32_RCU_H */

/** @} */
