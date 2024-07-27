/**************************************************************************//**
 * @file     system_NUC123.h
 * @version  V3.0
 * $Revision: 5 $
 * $Date: 15/07/02 11:21a $
 * @brief    NUC123 Series CMSIS System Header File
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __SYSTEM_NUC123_H
#define __SYSTEM_NUC123_H

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Macro Definition                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef DEBUG_PORT
# define DEBUG_PORT      UART0       /*!< Select Debug Port which is used for retarget.c to output debug message to UART */
#endif

/*----------------------------------------------------------------------------
  Define SYSCLK
 *----------------------------------------------------------------------------*/
#define __HXT       (12000000UL)    /*!< External Crystal Clock Frequency     */
#define __LXT       (32768UL)       /*!< External Crystal Clock Frequency 32.768KHz */
#define __HIRC      (22118400UL)    /*!< Internal 22M RC Oscillator Frequency */
#define __LIRC      (10000UL)       /*!< Internal 10K RC Oscillator Frequency */
#define __HSI       (50000000UL)    /*!< PLL default output is 50MHz */

extern uint32_t SystemCoreClock;    /*!< System Clock Frequency (Core Clock) */
extern uint32_t CyclesPerUs;        /*!< Cycles per micro second              */
extern uint32_t PllClock;           /*!< PLL Output Clock Frequency           */

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system
 *         Initialize GPIO directions and values
 */
extern void SystemInit(void);


/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from CPU registers.
 */
extern void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif