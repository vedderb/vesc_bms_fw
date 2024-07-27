/*
 * Copyright (C) 2013-2016 Fabio Utzig, http://fabioutzig.com
 *           (C) 2016 flabbergast <s3+flabbergast@sdfeu.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * MIMXRT1062 memory setup.
 */
MEMORY
{
  /*
   * The top 64K of flash are unavailable:
   *
   * 60K are reserved for EEPROM emulation,
   *  4K are special read-only memory containing a known-good blink program
   *
   * See also https://www.pjrc.com/store/teensy40.html#memory_layout
   */
  flash0   : org = 0x60000000, len = 1984K  /* FLASH */
  flash1   : org = 0x00000000, len = 0
  flash2   : org = 0x00000000, len = 0
  flash3   : org = 0x00000000, len = 0
  flash4   : org = 0x00000000, len = 0
  flash5   : org = 0x00000000, len = 0
  flash6   : org = 0x00000000, len = 0
  flash7   : org = 0x00000000, len = 0

  /* NOTE: ram0, ram1 and ram2 can be configured to use various
   * percentages of the available 512K of FlexRAM.
   *
   * The teensy linker script defines each with a 512k size.
   *
   * That is not safe to do with ChibiOS, where the full size
   * of an individual entry (e.g. 512K for ram1) will be used
   * in full, e.g. for the heap.
   *
   * We get around this limitation by allocating all 512K of
   * FlexRAM as DTCM (ram1), and not using ram0 and ram2 at all.
   *
   * See also:
   * IMXRT1060RM: Page 36 Table 3-1 System memory map (CM7)
   */

  /* Our startup code configures all of the flexram as DTCM. */
  /* ram0: DTCM, general purpose only
     2000_0000, up to 512KB */
  ram0     : org = 0x20000000, len = 512k

  /* Our startup code does not configure any ram1. */
  /* Possible optimization: copy .text code into ram0 */
  /* ram1: ITCM, can be TCM or general purpose
     0000_0000, up to 512KB */
  /* ram1     : org = 0x00000000, len = 512k */
  ram1     : org = 0x00000000, len = 0

  /* Our startup code does not configure any ram2. */
  /* ram2: OCRAM (On-Chip RAM), general purpose but not TCM (slower)
     2020_0000 Size: 512KB OCRAM2  */
  /* ram2     : org = 0x20200000, len = 512k */
  ram2     : org = 0x00000000, len = 0

  ram3     : org = 0x00000000, len = 0
  ram4     : org = 0x00000000, len = 0
  ram5     : org = 0x00000000, len = 0
  ram6     : org = 0x00000000, len = 0
  ram7     : org = 0x00000000, len = 0
}

/* Flash region for the configuration bytes.*/
SECTIONS
{
  .text.progmem : ALIGN(4)
  {
    . = 0;
    KEEP(*(.flashconfig))
    FILL(0xFF)
    /* IVT offset is 0x1000 from the bootable image as per section 2.5
       of https://www.nxp.com/docs/en/application-note/AN12107.pdf */
    . = ORIGIN(flash0) + 0x1000;
    KEEP(*(.ivt))
    KEEP(*(.bootdata))

    /* .vectors get placed in rules_code.ld */
  } > flash0

  .cfmprotect : ALIGN(4) SUBALIGN(4)
  {
    KEEP(*(.cfmconfig))
  } > flash0
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
/* .data in teensy-arduino-core/imxrt1062.ld */
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash0);

/* RAM region to be used for BSS segment.*/
/* .bss in imxrt1062.ld */
REGION_ALIAS("BSS_RAM", ram0);

/* RAM region to be used for the default heap.*/
/* _heap_start in imxrt1062.ld */
REGION_ALIAS("HEAP_RAM", ram0);

/* Generic rules inclusion.*/
INCLUDE rules.ld
