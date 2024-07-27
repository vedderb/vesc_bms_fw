/*
    Copyright (C) 2020 Alex Lewontin

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
 * NUC123xD4xx0 memory setup.
 * 64k ROM, 20k ram
 */
MEMORY
{
    flash0 (rx) : org = 0x00000000, len = 0x11000       /* APROM */
    flash1 (rx) : org = 0x00000000, len = 0             /* Data flash placeholder */
    flash2 (rx) : org = 0x00100000, len = 0x1000        /* LDROM */
    flash3 (rx) : org = 0x00300000, len = 4             /* Config0 */
    flash4 (rx) : org = 0x00300004, len = 4             /* Config1 */
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    ram0   (wx) : org = 0x20000000, len = 0x5000
    ram1   (wx) : org = 0x00000000, len = 0
    ram2   (wx) : org = 0x00000000, len = 0
    ram3   (wx) : org = 0x00000000, len = 0
    ram4   (wx) : org = 0x00000000, len = 0
    ram5   (wx) : org = 0x00000000, len = 0
    ram6   (wx) : org = 0x00000000, len = 0
    ram7   (wx) : org = 0x00000000, len = 0
}

REGION_ALIAS("CONFIG0", flash3);
REGION_ALIAS("CONFIG1", flash4);

SECTIONS
{
    .nuc123_config0 : ALIGN(4)
    {
        KEEP(*(.nuc123_config0))
    } > CONFIG0

    .nuc123_config1 : ALIGN(4)
    {
        KEEP(*(.nuc123_config1))
    } > CONFIG1
}

/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", flash0);
REGION_ALIAS("VECTORS_FLASH_LMA", flash0);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", flash0);
REGION_ALIAS("XTORS_FLASH_LMA", flash0);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", flash0);
REGION_ALIAS("TEXT_FLASH_LMA", flash0);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", flash0);
REGION_ALIAS("RODATA_FLASH_LMA", flash0);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", flash0);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash0);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash0);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts.*/
REGION_ALIAS("MAIN_STACK_RAM", ram0);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram0);

/* Generic rules inclusion.*/
INCLUDE rules.ld
