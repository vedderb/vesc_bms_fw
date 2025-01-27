/*
  Adapted from ChibiOS-Contrib/testhal/KINETIS/FRDM-KL25Z/PWM
  (c) 2015 flabbergast <s3+flabbergast@sdfeu.org>
  Modifications copyright (C) 2020 Alex Lewontin

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

#include "hal.h"

const PWMConfig pwmcfg = {
    750000,
    1000,
    NULL,
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL, NUC123_PWM_CH0_PIN_PA12},
        {PWM_OUTPUT_ACTIVE_LOW, NULL, NUC123_PWM_CH1_PIN_PA13},
        {PWM_OUTPUT_DISABLED, NULL, NUC123_PWM_CH2_PIN_NONE},
        {PWM_OUTPUT_DISABLED, NULL, NUC123_PWM_CH3_PIN_NONE},
    },
};

#define BREATHE_STEP 16 /* ms; = 4000ms/TABLE_SIZE */

/* Breathing Sleep LED brighness(PWM On period) table
 *
 * http://www.wolframalpha.com/input/?i=%28sin%28+x%2F64*pi%29**8+*+255%2C+x%3D0+to+63
 * (0..63).each {|x| p ((sin(x/64.0*PI)**8)*255).to_i }
 */
/* ruby -e "a = ((0..255).map{|x| Math.exp(Math.cos(Math::PI+(2*x*(Math::PI)/255)))-Math.exp(-1) }); m = a.max; a.map\!{|x| (10000*x/m).to_i}; p a" */
#define TABLE_SIZE 256
static const uint16_t breathing_table[TABLE_SIZE] = {
    0,    0,    1,    4,    7,    11,   17,   23,   30,    38,   47,   58,
    69,   81,   94,   109,  124,  141,  159,  177,  197,   218,  241,  264,
    289,  315,  343,  372,  402,  433,  466,  501,  537,   574,  613,  654,
    696,  741,  786,  834,  883,  935,  988,  1043, 1100,  1159, 1220, 1283,
    1349, 1416, 1486, 1558, 1632, 1709, 1788, 1870, 1954,  2040, 2129, 2220,
    2314, 2411, 2510, 2611, 2715, 2822, 2932, 3044, 3158,  3275, 3395, 3517,
    3641, 3768, 3897, 4028, 4162, 4298, 4436, 4576, 4717,  4861, 5006, 5152,
    5300, 5449, 5600, 5751, 5903, 6055, 6208, 6361, 6513,  6666, 6818, 6970,
    7120, 7269, 7417, 7563, 7708, 7850, 7990, 8127, 8261,  8391, 8519, 8643,
    8762, 8878, 8989, 9095, 9196, 9293, 9383, 9469, 9548,  9622, 9689, 9750,
    9805, 9853, 9895, 9930, 9957, 9978, 9992, 9999, 10000, 9992, 9978, 9957,
    9930, 9895, 9853, 9805, 9750, 9689, 9622, 9548, 9469,  9383, 9293, 9196,
    9095, 8989, 8878, 8762, 8643, 8519, 8391, 8261, 8127,  7990, 7850, 7708,
    7563, 7417, 7269, 7120, 6970, 6818, 6666, 6513, 6361,  6208, 6055, 5903,
    5751, 5600, 5449, 5300, 5152, 5006, 4861, 4717, 4576,  4436, 4298, 4162,
    4028, 3897, 3768, 3641, 3517, 3395, 3275, 3158, 3044,  2932, 2822, 2715,
    2611, 2510, 2411, 2314, 2220, 2129, 2040, 1954, 1870,  1788, 1709, 1632,
    1558, 1486, 1416, 1349, 1283, 1220, 1159, 1100, 1043,  988,  935,  883,
    834,  786,  741,  696,  654,  613,  574,  537,  501,   466,  433,  402,
    372,  343,  315,  289,  264,  241,  218,  197,  177,   159,  141,  124,
    109,  94,   81,   69,   58,   47,   38,   30,   23,    17,   11,   7,
    4,    1,    0,    0};

uint16_t table_pos = 0;


/*
 * Application entry point.
 */
int main(void)
{

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   */

  halInit();

  /*
   * Enabling interrupts, initialization done.
   */
  osalSysEnable();

  pwmStart(&PWMD1, &pwmcfg);
  pwmEnableChannel(&PWMD1, 0, 0);
  pwmEnableChannel(&PWMD1, 1, 0);
  while (true) {
    osalThreadSleepMilliseconds(BREATHE_STEP);
    pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, breathing_table[table_pos]));
    pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, breathing_table[table_pos]));
    table_pos = (table_pos + 1) % TABLE_SIZE;
  }
}