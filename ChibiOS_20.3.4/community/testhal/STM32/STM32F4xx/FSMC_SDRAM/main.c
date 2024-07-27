/*
    ChibiOS/RT - Copyright (C) 2006-2014 Giovanni Di Sirio

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
/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

#include "ch.h"
#include "hal.h"

#include "string.h"

#include "membench.h"
#include "memtest.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */


#define SDRAM_SIZE       (8 * 1024 * 1024)
#define SDRAM_START      ((void *)FSMC_Bank6_MAP_BASE)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

static void mem_error_cb(memtest_t *memp, testtype type, size_t index,
                         size_t width, uint32_t got, uint32_t expect);

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 * SDRAM driver configuration structure.
 */
static const SDRAMConfig sdram_cfg = {
  .sdcr = (uint32_t) FMC_ColumnBits_Number_9b |
                     FMC_RowBits_Number_13b |
                     FMC_SDMemory_Width_16b |
                     FMC_InternalBank_Number_4 |
                     FMC_CAS_Latency_3 |
                     FMC_Write_Protection_Disable |
                     FMC_SDClock_Period_3 |
                     FMC_Read_Burst_Enable |
                     FMC_ReadPipe_Delay_1,
  .sdtr = (uint32_t) (2   - 1) | // FMC_LoadToActiveDelay = 2 (TMRD: 2 Clock cycles)
                     (7 <<  4) | // FMC_ExitSelfRefreshDelay = 7 (TXSR: min=70ns (7x11.11ns))
                     (4 <<  8) | // FMC_SelfRefreshTime = 4 (TRAS: min=42ns (4x11.11ns) max=120k (ns))
                     (7 << 12) | // FMC_RowCycleDelay = 7 (TRC:  min=70 (7x11.11ns))
                     (2 << 16) | // FMC_WriteRecoveryTime = 2 (TWR:  min=1+ 7ns (1+1x11.11ns))
                     (2 << 20) | // FMC_RPDelay = 2 (TRP:  20ns => 2x11.11ns)
                     (2 << 24),  // FMC_RCDDelay = 2 (TRCD: 20ns => 2x11.11ns)
  /* NRFS = 4-1*/
  .sdcmr = (3 << 5) | (FMC_SDCMR_MRD_BURST_LENGTH_2 |
                       FMC_SDCMR_MRD_BURST_TYPE_SEQUENTIAL |
                       FMC_SDCMR_MRD_CAS_LATENCY_3 |
                       FMC_SDCMR_MRD_OPERATING_MODE_STANDARD |
                       FMC_SDCMR_MRD_WRITEBURST_MODE_SINGLE) << 9,

  .sdrtr = (uint32_t)(683 << 1),
};

/*
 *
 */
static uint8_t int_buf[64*1024];

/*
 *
 */
static memtest_t memtest_struct = {
    SDRAM_START,
    SDRAM_SIZE,
    MEMTEST_WIDTH_32,
    mem_error_cb
};

/*
 *
 */
static membench_t membench_ext = {
    SDRAM_START,
    SDRAM_SIZE,
};

/*
 *
 */
static membench_t membench_int = {
    int_buf,
    sizeof(int_buf),
};

/*
 *
 */
static membench_result_t membench_result_ext2int;
static membench_result_t membench_result_int2ext;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

static void mem_error_cb(memtest_t *memp, testtype type, size_t index,
                         size_t width, uint32_t got, uint32_t expect) {
  (void)memp;
  (void)type;
  (void)index;
  (void)width;
  (void)got;
  (void)expect;

  osalSysHalt("Memory broken");
}

/*
 *
 */
static void memtest(void) {

  while (true) {
    memtest_run(&memtest_struct, MEMTEST_RUN_ALL);
  }
}

/*
 *
 */
static void membench(void) {
  membench_run(&membench_ext, &membench_int, &membench_result_int2ext);
  membench_run(&membench_int, &membench_ext, &membench_result_ext2int);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

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

  sdramInit();
  sdramStart(&SDRAMD1, &sdram_cfg);

  membench();
  memtest();

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (TRUE) {
    chThdSleepMilliseconds(500);
  }
}


