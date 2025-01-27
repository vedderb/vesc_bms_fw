/*
    ChibiOS/RT - Copyright (C) 2014 Uladzimir Pylinsky aka barthess

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
 * FSMC driver system settings.
 */
#define STM32_FSMC_USE_FSMC1                FALSE
#define STM32_FSMC_FSMC1_IRQ_PRIORITY       10

/*
 * FSMC SDRAM driver system settings.
 */
#define STM32_USE_FSMC_SDRAM                FALSE
#define STM32_SDRAM_USE_FSMC_SDRAM1         FALSE
#define STM32_SDRAM_USE_FSMC_SDRAM2         FALSE

/*
 * FSMC SRAM driver system settings.
 */
#define STM32_USE_FSMC_SRAM                 FALSE
