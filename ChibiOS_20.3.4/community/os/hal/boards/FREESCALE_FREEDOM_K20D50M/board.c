/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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

#if HAL_USE_PAL || defined(__DOXYGEN__)
/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
const PALConfig pal_default_config =
{
  .ports = {
    {
      .port = IOPORT1,  // PORTA
      .pads = {
        /* PTA0*/ PAL_MODE_ALTERNATIVE_7,   /* PTA1*/ PAL_MODE_UNCONNECTED,     /* PTA2*/ PAL_MODE_OUTPUT_PUSHPULL,
        /* PTA3*/ PAL_MODE_ALTERNATIVE_7,   /* PTA4*/ PAL_MODE_UNCONNECTED,     /* PTA5*/ PAL_MODE_UNCONNECTED,
        /* PTA6*/ PAL_MODE_UNCONNECTED,     /* PTA7*/ PAL_MODE_UNCONNECTED,     /* PTA8*/ PAL_MODE_UNCONNECTED,
        /* PTA9*/ PAL_MODE_UNCONNECTED,     /*PTA10*/ PAL_MODE_UNCONNECTED,     /*PTA11*/ PAL_MODE_UNCONNECTED,
        /*PTA12*/ PAL_MODE_UNCONNECTED,     /*PTA13*/ PAL_MODE_UNCONNECTED,     /*PTA14*/ PAL_MODE_UNCONNECTED,
        /*PTA15*/ PAL_MODE_UNCONNECTED,     /*PTA16*/ PAL_MODE_UNCONNECTED,     /*PTA17*/ PAL_MODE_UNCONNECTED,
        /*PTA18*/ PAL_MODE_INPUT_ANALOG,    /*PTA19*/ PAL_MODE_INPUT_ANALOG,    /*PTA20*/ PAL_MODE_UNCONNECTED,
        /*PTA21*/ PAL_MODE_UNCONNECTED,     /*PTA22*/ PAL_MODE_UNCONNECTED,     /*PTA23*/ PAL_MODE_UNCONNECTED,
        /*PTA24*/ PAL_MODE_UNCONNECTED,     /*PTA25*/ PAL_MODE_UNCONNECTED,     /*PTA26*/ PAL_MODE_UNCONNECTED,
        /*PTA27*/ PAL_MODE_UNCONNECTED,     /*PTA28*/ PAL_MODE_UNCONNECTED,     /*PTA29*/ PAL_MODE_UNCONNECTED,
        /*PTA30*/ PAL_MODE_UNCONNECTED,     /*PTA31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT2,  // PORTB
      .pads = {
        /* PTB0*/ PAL_MODE_ALTERNATIVE_2,   /* PTB1*/ PAL_MODE_ALTERNATIVE_2,   /* PTB2*/ PAL_MODE_UNCONNECTED,
        /* PTB3*/ PAL_MODE_UNCONNECTED,     /* PTB4*/ PAL_MODE_UNCONNECTED,     /* PTB5*/ PAL_MODE_UNCONNECTED,
        /* PTB6*/ PAL_MODE_UNCONNECTED,     /* PTB7*/ PAL_MODE_UNCONNECTED,     /* PTB8*/ PAL_MODE_UNCONNECTED,
        /* PTB9*/ PAL_MODE_UNCONNECTED,     /*PTB10*/ PAL_MODE_UNCONNECTED,     /*PTB11*/ PAL_MODE_UNCONNECTED,
        /*PTB12*/ PAL_MODE_UNCONNECTED,     /*PTB13*/ PAL_MODE_UNCONNECTED,     /*PTB14*/ PAL_MODE_UNCONNECTED,
        /*PTB15*/ PAL_MODE_UNCONNECTED,     /*PTB16*/ PAL_MODE_ALTERNATIVE_3,   /*PTB17*/ PAL_MODE_ALTERNATIVE_3,
        /*PTB18*/ PAL_MODE_UNCONNECTED,     /*PTB19*/ PAL_MODE_UNCONNECTED,     /*PTB20*/ PAL_MODE_UNCONNECTED,
        /*PTB21*/ PAL_MODE_UNCONNECTED,     /*PTB22*/ PAL_MODE_UNCONNECTED,     /*PTB23*/ PAL_MODE_UNCONNECTED,
        /*PTB24*/ PAL_MODE_UNCONNECTED,     /*PTB25*/ PAL_MODE_UNCONNECTED,     /*PTB26*/ PAL_MODE_UNCONNECTED,
        /*PTB27*/ PAL_MODE_UNCONNECTED,     /*PTB28*/ PAL_MODE_UNCONNECTED,     /*PTB29*/ PAL_MODE_UNCONNECTED,
        /*PTB30*/ PAL_MODE_UNCONNECTED,     /*PTB31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT3,  // PORTC
      .pads = {
        /* PTC0*/ PAL_MODE_UNCONNECTED,     /* PTC1*/ PAL_MODE_UNCONNECTED,     /* PTC2*/ PAL_MODE_UNCONNECTED,
        /* PTC3*/ PAL_MODE_OUTPUT_PUSHPULL, /* PTC4*/ PAL_MODE_UNCONNECTED,     /* PTC5*/ PAL_MODE_UNCONNECTED,
        /* PTC6*/ PAL_MODE_UNCONNECTED,     /* PTC7*/ PAL_MODE_UNCONNECTED,     /* PTC8*/ PAL_MODE_UNCONNECTED,
        /* PTC9*/ PAL_MODE_UNCONNECTED,     /*PTC10*/ PAL_MODE_UNCONNECTED,     /*PTC11*/ PAL_MODE_UNCONNECTED,
        /*PTC12*/ PAL_MODE_UNCONNECTED,     /*PTC13*/ PAL_MODE_UNCONNECTED,     /*PTC14*/ PAL_MODE_UNCONNECTED,
        /*PTC15*/ PAL_MODE_UNCONNECTED,     /*PTC16*/ PAL_MODE_UNCONNECTED,     /*PTC17*/ PAL_MODE_UNCONNECTED,
        /*PTC18*/ PAL_MODE_UNCONNECTED,     /*PTC19*/ PAL_MODE_UNCONNECTED,     /*PTC20*/ PAL_MODE_UNCONNECTED,
        /*PTC21*/ PAL_MODE_UNCONNECTED,     /*PTC22*/ PAL_MODE_UNCONNECTED,     /*PTC23*/ PAL_MODE_UNCONNECTED,
        /*PTC24*/ PAL_MODE_UNCONNECTED,     /*PTC25*/ PAL_MODE_UNCONNECTED,     /*PTC26*/ PAL_MODE_UNCONNECTED,
        /*PTC27*/ PAL_MODE_UNCONNECTED,     /*PTC28*/ PAL_MODE_UNCONNECTED,     /*PTC29*/ PAL_MODE_UNCONNECTED,
        /*PTC30*/ PAL_MODE_UNCONNECTED,     /*PTC31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT4,  // PORTD
      .pads = {
        /* PTD0*/ PAL_MODE_UNCONNECTED,     /* PTD1*/ PAL_MODE_UNCONNECTED,     /* PTD2*/ PAL_MODE_UNCONNECTED,
        /* PTD3*/ PAL_MODE_UNCONNECTED,     /* PTD4*/ PAL_MODE_OUTPUT_PUSHPULL, /* PTD5*/ PAL_MODE_UNCONNECTED,
        /* PTD6*/ PAL_MODE_UNCONNECTED,     /* PTD7*/ PAL_MODE_UNCONNECTED,     /* PTD8*/ PAL_MODE_UNCONNECTED,
        /* PTD9*/ PAL_MODE_UNCONNECTED,     /*PTD10*/ PAL_MODE_UNCONNECTED,     /*PTD11*/ PAL_MODE_UNCONNECTED,
        /*PTD12*/ PAL_MODE_UNCONNECTED,     /*PTD13*/ PAL_MODE_UNCONNECTED,     /*PTD14*/ PAL_MODE_UNCONNECTED,
        /*PTD15*/ PAL_MODE_UNCONNECTED,     /*PTD16*/ PAL_MODE_UNCONNECTED,     /*PTD17*/ PAL_MODE_UNCONNECTED,
        /*PTD18*/ PAL_MODE_UNCONNECTED,     /*PTD19*/ PAL_MODE_UNCONNECTED,     /*PTD20*/ PAL_MODE_UNCONNECTED,
        /*PTD21*/ PAL_MODE_UNCONNECTED,     /*PTD22*/ PAL_MODE_UNCONNECTED,     /*PTD23*/ PAL_MODE_UNCONNECTED,
        /*PTD24*/ PAL_MODE_UNCONNECTED,     /*PTD25*/ PAL_MODE_UNCONNECTED,     /*PTD26*/ PAL_MODE_UNCONNECTED,
        /*PTD27*/ PAL_MODE_UNCONNECTED,     /*PTD28*/ PAL_MODE_UNCONNECTED,     /*PTD29*/ PAL_MODE_UNCONNECTED,
        /*PTD30*/ PAL_MODE_UNCONNECTED,     /*PTD31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT5,  // PORTE
      .pads = {
        /* PTE0*/ PAL_MODE_UNCONNECTED,     /* PTE1*/ PAL_MODE_UNCONNECTED,     /* PTE2*/ PAL_MODE_UNCONNECTED,
        /* PTE3*/ PAL_MODE_UNCONNECTED,     /* PTE4*/ PAL_MODE_UNCONNECTED,     /* PTE5*/ PAL_MODE_UNCONNECTED,
        /* PTE6*/ PAL_MODE_UNCONNECTED,     /* PTE7*/ PAL_MODE_UNCONNECTED,     /* PTE8*/ PAL_MODE_UNCONNECTED,
        /* PTE9*/ PAL_MODE_UNCONNECTED,     /*PTE10*/ PAL_MODE_UNCONNECTED,     /*PTE11*/ PAL_MODE_UNCONNECTED,
        /*PTE12*/ PAL_MODE_UNCONNECTED,     /*PTE13*/ PAL_MODE_UNCONNECTED,     /*PTE14*/ PAL_MODE_UNCONNECTED,
        /*PTE15*/ PAL_MODE_UNCONNECTED,     /*PTE16*/ PAL_MODE_UNCONNECTED,     /*PTE17*/ PAL_MODE_UNCONNECTED,
        /*PTE18*/ PAL_MODE_UNCONNECTED,     /*PTE19*/ PAL_MODE_UNCONNECTED,     /*PTE20*/ PAL_MODE_UNCONNECTED,
        /*PTE21*/ PAL_MODE_UNCONNECTED,     /*PTE22*/ PAL_MODE_UNCONNECTED,     /*PTE23*/ PAL_MODE_UNCONNECTED,
        /*PTE24*/ PAL_MODE_UNCONNECTED,     /*PTE25*/ PAL_MODE_UNCONNECTED,     /*PTE26*/ PAL_MODE_UNCONNECTED,
        /*PTE27*/ PAL_MODE_UNCONNECTED,     /*PTE28*/ PAL_MODE_UNCONNECTED,     /*PTE29*/ PAL_MODE_UNCONNECTED,
        /*PTE30*/ PAL_MODE_UNCONNECTED,     /*PTE31*/ PAL_MODE_UNCONNECTED,
      },
    },
  },
};
#endif

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {

  k20x_clock_init();
}

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void) {
}
