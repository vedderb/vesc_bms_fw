/*
    ChibiOS - Copyright (C) 2020 Patrick Seidel
    ChibiOS - Copyright (C) 2021 Stefan Kerkmann

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
 * @file    crt0.S
 * @brief   RISC-V ECLIC startup file for ChibiOS.
 *
 * @addtogroup RISCV_ECLIC_STARTUP
 * @{
 */

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE                               0
#endif

#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE                                1
#endif

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Core initialization switch.
 */
#if !defined(CRT0_INIT_CORE) || defined(__DOXYGEN__)
#define CRT0_INIT_CORE                      TRUE
#endif

/**
 * @brief   Stack segments initialization switch.
 */
#if !defined(CRT0_STACKS_FILL_PATTERN) || defined(__DOXYGEN__)
#define CRT0_STACKS_FILL_PATTERN            0x55555555
#endif

/**
 * @brief   Stack segments initialization switch.
 */
#if !defined(CRT0_INIT_STACKS) || defined(__DOXYGEN__)
#define CRT0_INIT_STACKS                    TRUE
#endif

/**
 * @brief   DATA segment initialization switch.
 */
#if !defined(CRT0_INIT_DATA) || defined(__DOXYGEN__)
#define CRT0_INIT_DATA                      TRUE
#endif

/**
 * @brief   BSS segment initialization switch.
 */
#if !defined(CRT0_INIT_BSS) || defined(__DOXYGEN__)
#define CRT0_INIT_BSS                       TRUE
#endif

/**
 * @brief   RAM areas initialization switch.
 */
#if !defined(CRT0_INIT_RAM_AREAS) || defined(__DOXYGEN__)
#define CRT0_INIT_RAM_AREAS                 TRUE
#endif

/**
 * @brief   Constructors invocation switch.
 */
#if !defined(CRT0_CALL_CONSTRUCTORS) || defined(__DOXYGEN__)
#define CRT0_CALL_CONSTRUCTORS              TRUE
#endif

/**
 * @brief   Destructors invocation switch.
 */
#if !defined(CRT0_CALL_DESTRUCTORS) || defined(__DOXYGEN__)
#define CRT0_CALL_DESTRUCTORS               TRUE
#endif

/*===========================================================================*/
/* Code section.                                                             */
/*===========================================================================*/

#if !defined(__DOXYGEN__)

#include "riscv_encoding.h"

.section .crt0, "ax"
.global _start
.type _start, @function
_start:
    /* Disable interrupts globaly */
    csrc CSR_MSTATUS, MSTATUS_MIE

    /* Jump to logical address first to ensure correct operation of RAM region  */
    la      a0, _start
    li      a1, 1
    slli    a1, a1, 29
    bleu    a1, a0, _crt0_entry
    srli    a1, a1, 2
    bleu    a1, a0, _crt0_entry
    la      a0, _crt0_entry
    add     a0, a0, a1
    jr      a0

/*
 * CRT0 entry point.
 */
.global _crt0_entry
.type _crt0_entry, @function
_crt0_entry:
    /* Disable interrupts globaly */
    csrc CSR_MSTATUS, MSTATUS_MIE
    
    /* Initialize global pointer gp for linker relaxation.*/
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

    /* Initialize stack pointer sp */
    la sp, __process_stack_end__

    /*
    * Set the the NMI base mnvec to share
    * with mtvec by setting CSR_MMISC_CTL
    * bit 9 NMI_CAUSE_FFF to 1
    */
    li t0, MMISC_CTL_NMI_CAUSE_FFF
    csrs CSR_MMISC_CTL, t0

    /*
    * Intialize ECLIC vector interrupt
    * base address mtvt to vector_base
    */
    la t0, vector_base
    csrw CSR_MTVT, t0

    /*
    * Set ECLIC non-vector entry to be controlled
    * by mtvt2 CSR register.
    * Intialize ECLIC non-vector interrupt
    * base address mtvt2 to _irq_handler.
    */
    la t0, _irq_handler
    csrw CSR_MTVT2, t0
    csrs CSR_MTVT2, 0x1

    /*
    * Set Exception Entry MTVEC to _start_trap
    * Due to settings above, Exception and NMI
    * will share common entry.
    */
    la t0, _start_trap
    csrw CSR_MTVEC, t0

    /* Set the interrupt processing mode to ECLIC mode */
    li t0, 0x3f
    csrc CSR_MTVEC, t0
    csrs CSR_MTVEC, 0x3

    /* Enable mcycle and minstret counters */
    csrci CSR_MCOUNTINHIBIT, 0x5

#if CRT0_INIT_CORE == TRUE
    /* Core initialization.*/
    jal     ra, __core_init
#endif

    /* Early initialization..*/
    jal     ra, __early_init

#if CRT0_INIT_STACKS == TRUE
    li      a0, CRT0_STACKS_FILL_PATTERN
    /* Main Stack initialization. Note, it assumes that the
        stack size is a multiple of 4 so the linker file must
        ensure this.*/
    la      a1, __process_stack_end__
    la      a2, __process_stack_end__
psloop:
    bge     a1, a2, endpsloop
    sw      a0, 0(a1)
    addi    a1, a1, 4
    j       psloop
endpsloop:
#endif

#if CRT0_INIT_DATA == TRUE
    /* Data initialization. Note, it assumes that the DATA size
      is a multiple of 4 so the linker file must ensure this.*/
    la      a0, __textdata_base__
    la      a1, __data_base__
    la      a2, __data_end__
dloop:
    bge     a1, a2, enddloop
    lw      a3, 0(a0)
    sw      a3, 0(a1)
    addi    a0, a0, 4
    addi    a1, a1, 4
    j       dloop
enddloop:
#endif

#if CRT0_INIT_BSS == TRUE
    /* BSS initialization. Note, it assumes that the DATA size
      is a multiple of 4 so the linker file must ensure this.*/
    la      a1, __bss_base__
    la      a2, __bss_end__
bloop:
    bge     a1, a2, endbloop
    sw      zero, 0(a1)
    addi    a1, a1, 4
    j       bloop
endbloop:
#endif

#if CRT0_INIT_RAM_AREAS == TRUE
    /* RAM areas initialization.*/
    jal      ra, __init_ram_areas
#endif

    /* Late initialization..*/
    jal      ra, __late_init

#if CRT0_CALL_CONSTRUCTORS == TRUE
    /* Constructors invocation.*/
    la      s1, __init_array_base__
    la      s2, __init_array_end__
initloop:
    bge     s1, s2, endinitloop
    lw      a0, 0(s1)
    jalr    ra, a0, 0
    addi    s1, s1, 4
    j       initloop
endinitloop:
#endif

    /* Main program invocation, r0 contains the returned value.*/
    jal     ra, main

#if CRT0_CALL_DESTRUCTORS == TRUE
    /* Destructors invocation.*/
    la      s1, __fini_array_base__
    la      s2, __fini_array_end__
finiloop:
    bge     s1, s2, endfiniloop
    lw      a0, 0(s1)
    jalr    ra, a0, 0
    addi    s1, s1, 4
    j       finiloop
endfiniloop:
#endif

    /* Branching to the defined exit handler.*/
    jal       ra, __default_exit

#endif

/** @} */
