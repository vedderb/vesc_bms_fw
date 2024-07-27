/*
    ChibiOS - Copyright (C) 2021 Stefan Kerkmann.
    ChibiOS - Copyright (C) 2020 Patrick Seidel.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    compilers/GCC/chcoreasm.S
 * @brief   RISC-V ECLIC architecture port low level code.
 *
 * @addtogroup RISCV_ECLIC_GCC_CORE
 * @{
 */

#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE   0
#endif

#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE    1
#endif

#define _FROM_ASM_
#include "chlicense.h"
#include "chconf.h"
#include "chcore.h"
#include "riscv_encoding.h"

#if !defined(__DOXYGEN__)


# --------------------------------------------------------------------------
# RTOS-specific context offset.
# --------------------------------------------------------------------------
#if defined(_CHIBIOS_RT_CONF_)
#define CONTEXT_OFFSET  12
#elif defined(_CHIBIOS_NIL_CONF_)
#define CONTEXT_OFFSET  0
#else
#error "invalid chconf.h"
#endif

# Disable Interrupts globally.
.macro DISABLE_MIE
    csrc CSR_MSTATUS, MSTATUS_MIE
.endm

# Enable Interrupts globally.
.macro ENABLE_MIE
    csrs CSR_MSTATUS, MSTATUS_MIE
.endm

# --------------------------------------------------------------------------
# Interrupt context save macro. Saves all caller save registers
# and status csr registers on the stack.
# --------------------------------------------------------------------------
.macro SAVE_CONTEXT
    # Allocate stack space for context saving 
#if !defined(__riscv_32e)
    addi sp, sp, -20*REGBYTES
#else
    addi sp, sp, -14*REGBYTES
#endif /* __riscv_32e */
    
    # Store CSR mepc to stack using pushmepc
    csrrwi  zero, CSR_PUSHMEPC, 11
    # Store CSR mcause to stack using pushmcause
    csrrwi  zero, CSR_PUSHMCAUSE, 12
    # Store CSR msubm to stack using pushmsub
    csrrwi  zero, CSR_PUSHMSUBM, 13

    STORE ra, 0*REGBYTES(sp)
    STORE tp, 1*REGBYTES(sp)
    STORE t0, 2*REGBYTES(sp)
    STORE t1, 3*REGBYTES(sp)
    STORE t2, 4*REGBYTES(sp)
    STORE a0, 5*REGBYTES(sp)
    STORE a1, 6*REGBYTES(sp)
    STORE a2, 7*REGBYTES(sp)
    STORE a3, 8*REGBYTES(sp)
    STORE a4, 9*REGBYTES(sp)
    STORE a5, 10*REGBYTES(sp)
#if !defined(__riscv_32e)
    STORE a6, 14*REGBYTES(sp)
    STORE a7, 15*REGBYTES(sp)
    STORE t3, 16*REGBYTES(sp)
    STORE t4, 17*REGBYTES(sp)
    STORE t5, 18*REGBYTES(sp)
    STORE t6, 19*REGBYTES(sp)
#endif
.endm

# --------------------------------------------------------------------------
# Interrupt context restore macro. Restores all caller save 
# registers and status csr registers from stack.
# --------------------------------------------------------------------------
.macro RESTORE_CONTEXT
    LOAD t0, 11*REGBYTES(sp)
    csrw CSR_MEPC, t0
    LOAD t0, 12*REGBYTES(sp)
    csrw CSR_MCAUSE, t0
    LOAD t0, 13*REGBYTES(sp)
    csrw CSR_MSUBM, t0
    
    LOAD ra, 0*REGBYTES(sp)
    LOAD tp, 1*REGBYTES(sp)
    LOAD t0, 2*REGBYTES(sp)
    LOAD t1, 3*REGBYTES(sp)
    LOAD t2, 4*REGBYTES(sp)
    LOAD a0, 5*REGBYTES(sp)
    LOAD a1, 6*REGBYTES(sp)
    LOAD a2, 7*REGBYTES(sp)
    LOAD a3, 8*REGBYTES(sp)
    LOAD a4, 9*REGBYTES(sp)
    LOAD a5, 10*REGBYTES(sp)
#if !defined(__riscv_32e)
    LOAD a6, 14*REGBYTES(sp)
    LOAD a7, 15*REGBYTES(sp)
    LOAD t3, 16*REGBYTES(sp)
    LOAD t4, 17*REGBYTES(sp)
    LOAD t5, 18*REGBYTES(sp)
    LOAD t6, 19*REGBYTES(sp)
    
    # De-allocate the stack space
    addi sp, sp, 20*REGBYTES
#else
    addi sp, sp, 14*REGBYTES
#endif /* __riscv_32e */
.endm

# --------------------------------------------------------------------------
# Trap entry point (_start_trap)
# --------------------------------------------------------------------------
.section .trap, "ax"
.option push
.option norelax
.align 6
.globl _start_trap
    _start_trap:
    # Save the caller saving registers (context)
    SAVE_CONTEXT

    # Set the function argument
    csrr a0, mcause
    mv a1, sp
    csrr a2, CSR_MDCAUSE
    csrr a3, CSR_MSUBM

    # Call the function
    call handle_trap

    # Restore the caller saving registers (context)
    RESTORE_CONTEXT

    # Return to regular code
    mret
.option pop

# --------------------------------------------------------------------------
# Start a thread by invoking its work function.
# 
# Threads execution starts here, the code leaves the system critical zone
# and then jumps into the thread function passed in register S0. The
# register S1 contains the thread parameter. The function chThdExit() is
# called on thread function return.
# --------------------------------------------------------------------------
.globl  _port_thread_start
_port_thread_start:
#if CH_DBG_SYSTEM_STATE_CHECK
    jal     ra, _dbg_check_unlock
#endif
#if CH_DBG_STATISTICS
    jal     ra, _stats_stop_measure_crit_thd
#endif
    ENABLE_MIE
    mv      a0, s1
    jalr    ra, s0
    li      a0, 0              # MSG_OK
    jal     ra, chThdExit

_zombies:
       j       _zombies

# --------------------------------------------------------------------------
# Performs a context switch between two threads.
# a0 = ntp, a1 = otp
# --------------------------------------------------------------------------
.option push
.option norelax
.align 4
.globl  _port_switch
.type _port_switch,@function
_port_switch:
    # OLD THREAD CONTEXT SAVE BEGIN
    # Allocate space for port_intctx structure on the threading stack.
    # The stackpointer is 16 byte aligned to be compliant with risc-v abi.
#if !defined(__riscv_32e)
    addi sp, sp, -16*REGBYTES
#else
    addi sp, sp, -4*REGBYTES
#endif

    # Store callee save registers
    STORE      ra,  0*REGBYTES(sp)
    STORE      s0,  1*REGBYTES(sp)
    STORE      s1,  2*REGBYTES(sp)
#if !defined(__riscv_32e)
    STORE      s2,  3*REGBYTES(sp)
    STORE      s3,  4*REGBYTES(sp)
    STORE      s4,  5*REGBYTES(sp)
    STORE      s5,  6*REGBYTES(sp)
    STORE      s6,  7*REGBYTES(sp)
    STORE      s7,  8*REGBYTES(sp)
    STORE      s8,  9*REGBYTES(sp)
    STORE      s9,  10*REGBYTES(sp)
    STORE      s10, 11*REGBYTES(sp)
    STORE      s11, 12*REGBYTES(sp)
#endif

    # Store stackpointer in otp->ctx
    STORE      sp, CONTEXT_OFFSET(a1)
    # OLD THREAD CONTEXT SAVE END

    # NEW THREAD CONTEXT RESTORE BEGIN
    # Load stackpointer from ntp->ctx
    LOAD       sp, CONTEXT_OFFSET(a0)

    LOAD      ra,  0*REGBYTES(sp)
    LOAD      s0,  1*REGBYTES(sp)
    LOAD      s1,  2*REGBYTES(sp)
#if !defined(__riscv_32e)
    LOAD      s2,  3*REGBYTES(sp)
    LOAD      s3,  4*REGBYTES(sp)
    LOAD      s4,  5*REGBYTES(sp)
    LOAD      s5,  6*REGBYTES(sp)
    LOAD      s6,  7*REGBYTES(sp)
    LOAD      s7,  8*REGBYTES(sp)
    LOAD      s8,  9*REGBYTES(sp)
    LOAD      s9,  10*REGBYTES(sp)
    LOAD      s10, 11*REGBYTES(sp)
    LOAD      s11, 12*REGBYTES(sp)

    # De-allocate space on the threading stack
    addi sp, sp, 16*REGBYTES
#else
    addi sp, sp, 4*REGBYTES
#endif
    # NEW THREAD CONTEXT RESTORE END

    # Jump to return address loaded into ra
    ret
.option pop

# --------------------------------------------------------------------------
# IRQ entry point
# --------------------------------------------------------------------------
.section .text
.option push
.option norelax
.align 2
.globl _irq_handler
_irq_handler:
    # Save all caller registers and csr registers on the thread stack
    SAVE_CONTEXT

    # The special CSR read/write operation, which is actually Claim the CLIC to
    # find its pending highest ID, if the ID is not 0, then automatically enable
    # the mstatus.MIE, and jump to its vector-entry-label, and update the link register.
    csrrw   ra, CSR_JALMNXTI, ra

    # Critical Section Disable Interrupts Globaly
    DISABLE_MIE

    # No reschedule is necessary, just restore irq context and exit machine mode.
    beq     a0, zero, _port_exit_from_isr

    # Context switch is necessary, load switching routine into mepc
    la      a0, _port_switch_from_isr
    csrw    mepc, a0

    # Interrupt handling and context restoring is handled differently in nucleisys cores.
    # mstatus.mpie and mstatus.mpp are mirror fields of mcause.mpie and mcause.mpp.
    # Therefore we directly set the bits in mcause and not mstatus.
    # See https://doc.nucleisys.com/nuclei_spec/isa/core_csr.html#mcause

    # Context switch is a critical section, so disable interrupts on return.
    # Clear mcause.mpie.
    li a0, 0x8000000
    csrc mcause, a0

    # Set previous privelege mode to machine mode to enforce it on return.
    # Set mcause.mpp to 0x3 (== machine mode).
    li a0, 0x30000000
    csrs mcause, a0

    mret

.option pop

.globl handle_trap
.weak handle_trap
handle_trap:
    j       handle_trap  # jump to handle_trap

# --------------------------------------------------------------------------
# Post-IRQ switch code.
# 
# Exception handlers return here for context switching.
# --------------------------------------------------------------------------
.section .text
.option push
.option norelax
.align 4
.globl _port_switch_from_isr
_port_switch_from_isr:
#if CH_DBG_STATISTICS
    jal     ra, _stats_start_measure_crit_thd
#endif
#if CH_DBG_SYSTEM_STATE_CHECK
    jal     ra, _dbg_check_lock
#endif
    # Calls _port_switch at the end of the function
    jal     ra, chSchDoReschedule
#if CH_DBG_SYSTEM_STATE_CHECK
    jal     ra, _dbg_check_unlock
#endif
#if CH_DBG_STATISTICS
    jal     ra, _stats_stop_measure_crit_thd
#endif

.globl _port_exit_from_isr
_port_exit_from_isr:
    # Restore caller registers and csr registers from the thread stack
    RESTORE_CONTEXT

    # Leave interrupt handling and return to address stored in mepc.
    mret

.option pop

#endif /* !defined(__DOXYGEN__) */

/** @} */
