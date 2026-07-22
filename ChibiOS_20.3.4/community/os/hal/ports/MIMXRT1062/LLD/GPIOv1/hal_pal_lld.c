/*
    ChibiOS - Copyright (C) 2014-2015 Fabio Utzig

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
 * @file    GPIOv1/hal_pal_lld.c
 * @brief   PAL subsystem low level driver.
 *
 * @addtogroup PAL
 * @{
 */

#include "osal.h"
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

uint8_t bit_by_index[] =
  {
   TEENSY_PIN0_BIT,
   TEENSY_PIN1_BIT,
   TEENSY_PIN2_BIT,
   TEENSY_PIN3_BIT,
   TEENSY_PIN4_BIT,
   TEENSY_PIN5_BIT,
   TEENSY_PIN6_BIT,
   TEENSY_PIN7_BIT,
   TEENSY_PIN8_BIT,
   TEENSY_PIN9_BIT,
   TEENSY_PIN10_BIT,
   TEENSY_PIN11_BIT,
   TEENSY_PIN12_BIT,
   TEENSY_PIN13_BIT,
   TEENSY_PIN14_BIT,
   TEENSY_PIN15_BIT,
   TEENSY_PIN16_BIT,
   TEENSY_PIN17_BIT,
   TEENSY_PIN18_BIT,
   TEENSY_PIN19_BIT,
   TEENSY_PIN20_BIT,
   TEENSY_PIN21_BIT,
   TEENSY_PIN22_BIT,
   TEENSY_PIN23_BIT,
   TEENSY_PIN24_BIT,
   TEENSY_PIN25_BIT,
   TEENSY_PIN26_BIT,
   TEENSY_PIN27_BIT,
   TEENSY_PIN28_BIT,
   TEENSY_PIN29_BIT,
   TEENSY_PIN30_BIT,
   TEENSY_PIN31_BIT,
   TEENSY_PIN32_BIT,
   TEENSY_PIN33_BIT,
   TEENSY_PIN34_BIT,
   TEENSY_PIN35_BIT,
   TEENSY_PIN36_BIT,
   TEENSY_PIN37_BIT,
   TEENSY_PIN38_BIT,
   TEENSY_PIN39_BIT,
   TEENSY_PIN40_BIT,
   TEENSY_PIN41_BIT,
   TEENSY_PIN42_BIT,
   TEENSY_PIN43_BIT,
   TEENSY_PIN44_BIT,
   TEENSY_PIN45_BIT,
   TEENSY_PIN46_BIT,
   TEENSY_PIN47_BIT,
   TEENSY_PIN48_BIT,
   TEENSY_PIN49_BIT,
   TEENSY_PIN50_BIT,
   TEENSY_PIN51_BIT,
   TEENSY_PIN52_BIT,
   TEENSY_PIN53_BIT,
   TEENSY_PIN54_BIT,
  };

uint8_t SW_MUX_CTL_PAD_by_index[] =
  {
   TEENSY_PIN0_MUX,
   TEENSY_PIN1_MUX,
   TEENSY_PIN2_MUX,
   TEENSY_PIN3_MUX,
   TEENSY_PIN4_MUX,
   TEENSY_PIN5_MUX,
   TEENSY_PIN6_MUX,
   TEENSY_PIN7_MUX,
   TEENSY_PIN8_MUX,
   TEENSY_PIN9_MUX,
   TEENSY_PIN10_MUX,
   TEENSY_PIN11_MUX,
   TEENSY_PIN12_MUX,
   TEENSY_PIN13_MUX,
   TEENSY_PIN14_MUX,
   TEENSY_PIN15_MUX,
   TEENSY_PIN16_MUX,
   TEENSY_PIN17_MUX,
   TEENSY_PIN18_MUX,
   TEENSY_PIN19_MUX,
   TEENSY_PIN20_MUX,
   TEENSY_PIN21_MUX,
   TEENSY_PIN22_MUX,
   TEENSY_PIN23_MUX,
   TEENSY_PIN24_MUX,
   TEENSY_PIN25_MUX,
   TEENSY_PIN26_MUX,
   TEENSY_PIN27_MUX,
   TEENSY_PIN28_MUX,
   TEENSY_PIN29_MUX,
   TEENSY_PIN30_MUX,
   TEENSY_PIN31_MUX,
   TEENSY_PIN32_MUX,
   TEENSY_PIN33_MUX,
   TEENSY_PIN34_MUX,
   TEENSY_PIN35_MUX,
   TEENSY_PIN36_MUX,
   TEENSY_PIN37_MUX,
   TEENSY_PIN38_MUX,
   TEENSY_PIN39_MUX,
   TEENSY_PIN40_MUX,
   TEENSY_PIN41_MUX,
   TEENSY_PIN42_MUX,
   TEENSY_PIN43_MUX,
   TEENSY_PIN44_MUX,
   TEENSY_PIN45_MUX,
   TEENSY_PIN46_MUX,
   TEENSY_PIN47_MUX,
   TEENSY_PIN48_MUX,
   TEENSY_PIN49_MUX,
   TEENSY_PIN50_MUX,
   TEENSY_PIN51_MUX,
   TEENSY_PIN52_MUX,
   TEENSY_PIN53_MUX,
   TEENSY_PIN54_MUX,
  };

uint8_t SW_PAD_CTL_PAD_by_index[] =
  {
   TEENSY_PIN0_PAD,
   TEENSY_PIN1_PAD,
   TEENSY_PIN2_PAD,
   TEENSY_PIN3_PAD,
   TEENSY_PIN4_PAD,
   TEENSY_PIN5_PAD,
   TEENSY_PIN6_PAD,
   TEENSY_PIN7_PAD,
   TEENSY_PIN8_PAD,
   TEENSY_PIN9_PAD,
   TEENSY_PIN10_PAD,
   TEENSY_PIN11_PAD,
   TEENSY_PIN12_PAD,
   TEENSY_PIN13_PAD,
   TEENSY_PIN14_PAD,
   TEENSY_PIN15_PAD,
   TEENSY_PIN16_PAD,
   TEENSY_PIN17_PAD,
   TEENSY_PIN18_PAD,
   TEENSY_PIN19_PAD,
   TEENSY_PIN20_PAD,
   TEENSY_PIN21_PAD,
   TEENSY_PIN22_PAD,
   TEENSY_PIN23_PAD,
   TEENSY_PIN24_PAD,
   TEENSY_PIN25_PAD,
   TEENSY_PIN26_PAD,
   TEENSY_PIN27_PAD,
   TEENSY_PIN28_PAD,
   TEENSY_PIN29_PAD,
   TEENSY_PIN30_PAD,
   TEENSY_PIN31_PAD,
   TEENSY_PIN32_PAD,
   TEENSY_PIN33_PAD,
   TEENSY_PIN34_PAD,
   TEENSY_PIN35_PAD,
   TEENSY_PIN36_PAD,
   TEENSY_PIN37_PAD,
   TEENSY_PIN38_PAD,
   TEENSY_PIN39_PAD,
   TEENSY_PIN40_PAD,
   TEENSY_PIN41_PAD,
   TEENSY_PIN42_PAD,
   TEENSY_PIN43_PAD,
   TEENSY_PIN44_PAD,
   TEENSY_PIN45_PAD,
   TEENSY_PIN46_PAD,
   TEENSY_PIN47_PAD,
   TEENSY_PIN48_PAD,
   TEENSY_PIN49_PAD,
   TEENSY_PIN50_PAD,
   TEENSY_PIN51_PAD,
   TEENSY_PIN52_PAD,
   TEENSY_PIN53_PAD,
   TEENSY_PIN54_PAD,
  };

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief Reads a logical state from an I/O pad.
 * @note The @ref PAL provides a default software implementation of this
 * functionality, implement this function if can optimize it by using
 * special hardware functionalities or special coding.
 *
 * @param[in] port port identifier
 * @param[in] pad pad number within the port
 * @return The logical state.
 * @retval PAL_LOW low logical state.
 * @retval PAL_HIGH high logical state.
 *
 * @notapi
 */
uint8_t _pal_lld_readpad(ioportid_t port,
                         uint8_t pad) {
  return (port->DR & ((uint32_t) 1 << bit_by_index[pad])) ? PAL_HIGH : PAL_LOW;
}

/**
 * @brief Writes a logical state on an output pad.
 * @note This function is not meant to be invoked directly by the
 * application code.
 * @note The @ref PAL provides a default software implementation of this
 * functionality, implement this function if can optimize it by using
 * special hardware functionalities or special coding.
 *
 * @param[in] port port identifier
 * @param[in] pad pad number within the port
 * @param[in] bit logical value, the value must be @p PAL_LOW or
 * @p PAL_HIGH
 *
 * @notapi
 */
void _pal_lld_writepad(ioportid_t port,
                       uint8_t pad,
                       uint8_t bit) {
  if (bit == PAL_HIGH)
    port->DR_SET = ((uint32_t) 1 << bit_by_index[pad]);
  else
    port->DR_CLEAR = ((uint32_t) 1 << bit_by_index[pad]);
}

void _pal_lld_togglepad(ioportid_t port,
                        uint8_t pad) {
  port->DR_TOGGLE = ((uint32_t) 1 << bit_by_index[pad]);
}

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
void _pal_lld_setpadmode(ioportid_t port,
                         uint8_t pad,
                         iomode_t mode) {
  osalDbgAssert(pad < PADS_PER_PORT, "pal_lld_setpadmode() #1, invalid pad");

  // see IMXRT1060RM Section 12.4.3 GPIO programming

  // Interrupt Mask Register (IMR)
  port->IMR &= ~((uint32_t) 1 << bit_by_index[pad]);

  // All GPIOs are on mode ALT5 as per Chapter 10, External Signals and Pin Multiplexing, Table 10-1
  const int altMode = 5;

  switch (mode) {
  case PAL_MODE_RESET:
  case PAL_MODE_INPUT:
  case PAL_MODE_OUTPUT_PUSHPULL:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(altMode);
    // need to always set PAD_CTL in case the pin was configured as input before
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_by_index[pad]] =
      IOMUXC_SW_PAD_CTL_PAD_DSE(6);
    //IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_by_index[pad]] = 0;
    break;

  case PAL_MODE_OUTPUT_OPENDRAIN:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(altMode);
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_by_index[pad]] =
      IOMUXC_SW_PAD_CTL_PAD_ODE(1); /* Open Drain Enable */
    break;

  case PAL_MODE_INPUT_PULLUP:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(altMode);

    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_by_index[pad]] =
      IOMUXC_SW_PAD_CTL_PAD_PKE(1) | /* Pull/Keep Enable */
      IOMUXC_SW_PAD_CTL_PAD_PUS(1) | /* Pull Up/Down Config: 47k pull up */
      IOMUXC_SW_PAD_CTL_PAD_PUE(1); /* Pull/Keep Select: pull */

    break;

  case PAL_MODE_INPUT_PULLDOWN:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(altMode);

    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_by_index[pad]] =
      IOMUXC_SW_PAD_CTL_PAD_PKE(1) | /* Pull/Keep Enable */
      IOMUXC_SW_PAD_CTL_PAD_PUS(0) | /* Pull Up/Down Config: 100k pull dn */
      IOMUXC_SW_PAD_CTL_PAD_PUE(1); /* Pull/Keep Select: pull */
    break;

  case PAL_MODE_UNCONNECTED:
  case PAL_MODE_INPUT_ANALOG:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(altMode);
    break;

  case PAL_MODE_ALTERNATIVE_1:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(1);
    break;

  case PAL_MODE_ALTERNATIVE_2:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(2);
    break;

  case PAL_MODE_ALTERNATIVE_3:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(3);
    break;

  case PAL_MODE_ALTERNATIVE_4:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(4);
    break;

  case PAL_MODE_ALTERNATIVE_5:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(5);
    break;

  case PAL_MODE_ALTERNATIVE_6:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(6);
    break;

  case PAL_MODE_ALTERNATIVE_7:
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_by_index[pad]] =
      PIN_MUX_ALTERNATIVE(7);
    break;
  }

  // GPIO direction register (GDIR)
  if (mode == PAL_MODE_OUTPUT_PUSHPULL) {
    port->GDIR |= ((uint32_t) 1 << bit_by_index[pad]);
  } else {
    port->GDIR &= ~((uint32_t) 1 << bit_by_index[pad]);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   MIMXRT1062 I/O ports configuration.
 *
 * @param[in] config    the MIMXRT1062 ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config) {
  (void)config;
  /* Clocks are already enabled because MIMXRT1062_clock_init() enables GPIO
     earlier in the boot. */
}

#endif /* HAL_USE_PAL */

/** @} */
