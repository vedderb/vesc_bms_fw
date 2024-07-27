/*
    ChibiOS - Copyright (C) 2019 Diego Ismirlian (dismirlian(at)google's mail)

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

                .syntax unified
                .cpu    cortex-m4

                .section .data._fault_stack
                .align 3
_fault_stack:
                .skip 256
_fault_stack_end:

                .thumb

                .section .text.HardFault_Handler
                .thumb_func
                .globl  HardFault_Handler
HardFault_Handler:
                /* disable further interrupts */
                cpsid i

                /* preserve the ISR sp for later */
                mov     r1, sp

                /* set the sp to a separate stack, in case of sp corruption */
                ldr     sp, .L1

                /* preserve the ISR lr and sp for later */
                push    {r1, lr}

                /* print info */
                bl       _hardfault_info

                /* restore the sp and the lr */
                pop     {r1, lr}
                mov     sp, r1

                /* Try to rebuild the stack for the debugger.
                 * The idea is that the debugger will unwind the stack, and
                 * show a call to the HardFault_Handler from the offending
                 * instruction */

                /* check which stack was in use */
                tst     lr, #4          //check bit 2 of EXC_RETURN
                ite     eq
                mrseq   r0, msp
                mrsne   r0, psp         //r0 points to the stack that was in use

                /* try to rebuild the stack for the debugger */
                mov     sp, r0          //sp points to the end of the IRQ stack
                /* check if FPU registers were stacked */
                tst     lr, #16         //check bit 4 of EXC_RETURN
                ite     eq
                addeq   sp, #104        //jump over the IRQ+FPU stack
                addne   sp, #32         //jump over the IRQ stack

                /* compensate padding */
                ldr     r1, [sp, #28]   //r1 = stacked xPSR
                tst     r1, #512        //check bit 9 of the stacked xPSR
                ite     eq
                addeq   sp, #0          //add 0 to sp if there was no padding
                addne   sp, #4          //add 4 to sp if there was padding
                /* here, sp finally points to the stack before the IRQ was triggered */

                /* set lr to the stacked PC address, so the debugger can show where the
                   fault was produced (may not be accurate, depending on the fault) */
                ldr     lr, [r0, #24]

                /* restore used registers */
                ldr     r0, [r0, #0]
                ldr     r1, [r0, #4]

                b       _hardfault_epilogue

                .align  2
.L1:            .word _fault_stack_end


                .section .text._hardfault_exit
                .thumb_func
                .globl  _hardfault_exit
_hardfault_exit:
                /* we are here if there is no debugger attached */

                /* restore the sp to the separate stack */
                ldr     sp, .L3

                /* call chSysHalt */
                ldr     r0, =.L2
                bl      chSysHalt

                b       .

                .align  2
.L3:            .word _fault_stack_end

                .align  2
.L2:            .asciz  "hard fault"

                .align  2
