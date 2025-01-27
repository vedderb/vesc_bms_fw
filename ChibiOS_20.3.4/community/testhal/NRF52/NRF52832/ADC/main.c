/*
    Copyright (C) 2020 Konstantin Oblaukhov

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

#include "ch.h"
#include "hal.h"

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      8

#define ADC_GRP2_NUM_CHANNELS   1
#define ADC_GRP2_BUF_DEPTH      16

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static adcsample_t samples2[ADC_GRP2_NUM_CHANNELS * ADC_GRP2_BUF_DEPTH];

/*
 * ADC streaming callback.
 */
size_t nx = 0;
static void adccallback(ADCDriver *adcp) {

  (void)adcp;
  nx++;
}

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    AIN0 (prescaled by 1/6).
 */
static const ADCConversionGroup adcgrpcfg1 = {
  false,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  NULL,
  false,
  SAADC_RESOLUTION_VAL_14bit,
  0,
  0,
  {
    {SAADC_CH_PSELP_PSELP_AnalogInput0, 
     SAADC_CH_PSELN_PSELN_NC,
     SAADC_CH_CONFIG_TACQ_40us << SAADC_CH_CONFIG_TACQ_Pos}
  }
};

/*
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 1 channel, SW triggered.
 * Channels:    VDD (prescaled by 1/6).
 */
static const ADCConversionGroup adcgrpcfg2 = {
  true,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  NULL,
  false,
  SAADC_RESOLUTION_VAL_14bit,
  0,
  0,
  {
    {SAADC_CH_PSELP_PSELP_VDD, 
     SAADC_CH_PSELN_PSELN_NC,
     SAADC_CH_CONFIG_TACQ_40us << SAADC_CH_CONFIG_TACQ_Pos}
  }
};

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palTogglePad(IOPORT1, LED1);
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Activates the ADC1 driver and the temperature sensor.
   */
  adcStart(&ADCD1, NULL);

  /*
   * Linear conversion.
   */
  adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
  chThdSleepMilliseconds(1000);

  /*
   * Starts an ADC continuous conversion.
   */
  adcStartConversion(&ADCD1, &adcgrpcfg2, samples2, ADC_GRP2_BUF_DEPTH);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {
    if (palReadPad(IOPORT1, BTN1) == 0)
      adcStopConversion(&ADCD1);
    chThdSleepMilliseconds(500);
  }
}
