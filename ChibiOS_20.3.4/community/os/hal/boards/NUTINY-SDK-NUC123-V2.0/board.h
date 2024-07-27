/*
    ChibiOS - Copyright (C) 2020 Alex Lewontin

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

#ifndef BOARD_H
#define BOARD_H

/*
 * Setup for a generic board.
 */

/*
 * Board identifier.
 */
#define NUC123SD4AN0
#define BOARD_NAME "NUTINY SDK NUC123 V2.0"

/*
 * Board specific settings.
 */
/*
 * External XTAL speed.
 */
#define NUC123_HSECLK 12000000UL

/*
 * LED macros.
 */
#define ONBOARD_LED_LINE PAL_LINE(GPIOB, 4)

#define OnboardLED_Init()                                                   \
  palSetGroupMode(GPIOB, 1, 4, PAL_MODE_OUTPUT_PUSHPULL)
#define OnboardLED_On()     palClearLine(ONBOARD_LED_LINE)
#define OnboardLED_Off()    palSetLine(ONBOARD_LED_LINE)
#define OnboardLED_Toggle() palToggleLine(ONBOARD_LED_LINE)

#define ONBOARD_SERIAL_DRIVER SD0

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
