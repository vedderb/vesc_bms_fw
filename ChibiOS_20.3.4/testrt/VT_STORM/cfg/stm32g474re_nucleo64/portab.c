/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @file    portab.c
 * @brief   Application portability module code.
 *
 * @addtogroup application_portability
 * @{
 */

#include "hal.h"
#include "vt_storm.h"

#include "portab.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

#if VT_STORM_CFG_HAMMERS || defined(__DOXYGEN__)
/*
 * GPT4 configuration.
 */
static const GPTConfig gpt3cfg = {
  1000000,              /* 1MHz timer clock.*/
  vt_storm_gpt1_cb,     /* Timer callback.*/
  0,
  0
};

/*
 * GPT3 configuration.
 */
static const GPTConfig gpt4cfg = {
  1000000,              /* 1MHz timer clock.*/
  vt_storm_gpt2_cb,     /* Timer callback.*/
  0,
  0
};
#endif

/*
 * VT Storm configuration.
 */
const vt_storm_config_t portab_vt_storm_config = {
  (BaseSequentialStream  *)&PORTAB_SD1,
  PORTAB_LINE_LED1,
#if VT_STORM_CFG_HAMMERS
  &GPTD3,
  &GPTD4,
  &gpt3cfg,
  &gpt4cfg,
#endif
  STM32_SYSCLK
};

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void portab_setup(void) {

}

/** @} */
