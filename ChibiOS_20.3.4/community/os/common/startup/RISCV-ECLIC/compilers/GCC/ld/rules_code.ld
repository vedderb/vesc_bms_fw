/*
    ChibiOS - Copyright (C) 2020 Patrick Seidel

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

ENTRY(_start)

SECTIONS
{
    .vectors : ALIGN(4)
    {
        KEEP (*(SORT_NONE(.crt0)))
        /* For 65 to 128 interrupts, the vector table must be 512 byte aligned.
         * See: https://doc.nucleisys.com/nuclei_spec/isa/core_csr.html#mtvt */
        . = ALIGN(512);
        KEEP (*(SORT_NONE(.vectors)))
    } > VECTORS_FLASH AT > VECTORS_FLASH_LMA

    .xtors : ALIGN(4)
    {
        __init_array_base__ = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end__ = .;
        __fini_array_base__ = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        __fini_array_end__ = .;
    } > XTORS_FLASH AT > XTORS_FLASH_LMA

    .text : ALIGN_WITH_INPUT
    {
        __text_base__ = .;
        *(.text)
        *(.text.*)
        *(.gcc*)
        __text_end__ = .;
    } > TEXT_FLASH AT > TEXT_FLASH_LMA

    .rodata : ALIGN(4)
    {
        __rodata_base__ = .;
        *(.rodata)
        *(.rodata.*)
        *(.srodata)
        *(.srodata.*)
        . = ALIGN(4);
        __rodata_end__ = .;
    } > RODATA_FLASH AT > RODATA_FLASH_LMA

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA
}
