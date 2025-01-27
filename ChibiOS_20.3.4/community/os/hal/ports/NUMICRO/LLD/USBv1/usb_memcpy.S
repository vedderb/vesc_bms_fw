/*
    ChibiOS - Copyright (C) 2020 Alex Lewontin

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
 * @file    usb_memcpy.S
 * @brief   Definition of the usb_memcpy function
 *
 * @addtogroup USB
 * @{
 */

#if !defined(__DOXYGEN__)

    .syntax     unified
    .cpu        cortex-m0
    .thumb

    .section    .text, "ax"
    .align      4
    .global     usb_memcpy

/* size_t usb_memcpy(volatile void* destination, const volatile void* source, size_t num); */
usb_memcpy:
  cmp     r2, #0
  beq.n   zero_return
  push    {r2, r4, r5, r6, r7, lr}

  movs    r3, #3

check_alignment:
  tst     r0, r3
  bne.n   offset
  tst     r1, r3
  bne.n   unaligned

aligned:
/* r0: dest, r1: src, r2: num, r3: #3 */
  movs    r4, r2

  bics    r4, r3
  movs    r6, r4

  b.n     aligned_loop_check
aligned_loop_top:
  subs    r4, r4, #4
  ldr     r5, [r1, r4]
  str     r5, [r0, r4]
aligned_loop_check:
  bne.n   aligned_loop_top


leftover_check:
  ands    r2, r3
  beq.n   return

/* if there are leftovers, we can read them all in one */

/* byte mode */
  ldr     r4, =0x40060000
  ldr     r5, [r4, #16]
  movs    r7, #128 
  lsls    r7, r7, #3 /* USBD_ATTR_BYTEM_Msk */
  orrs    r5, r7
  str     r5, [r4, #16]
  adds    r1, r1, r6
  adds    r0, r0, r6

/*
 * r0: dest
 * r1: src
 * r2 is 1, 2, or 3 (# of bytes left)
 * r3: #3
 * r4: &USBD
 * r5: USBD->ATTR
 * r6: leftover offset base
 * r7: USBD_ATTR_BYTEM_Msk
 */
  ldrb    r3, [r1, #0]
  strb    r3, [r0, #0]

  cmp     r2, #1
  beq.n   word_mode_return

  ldrb    r3, [r1, #1]
  strb    r3, [r0, #1]

  cmp     r2, #2
  beq.n   word_mode_return

  ldrb    r3, [r1, #2]
  strb    r3, [r0, #2]

word_mode_return:
  ldr     r5, [r4, #16]
  bics    r5, r7
  str     r5, [r4, #16]
return:
  pop     {r0, r4, r5, r6, r7, pc}
zero_return:
  movs    r0, #0
  bx.n    lr

offset:
  ldr     r4, =0x40060000
  ldr     r5, [r4, #16]
  movs    r7, #128 
  lsls    r7, r7, #3 /* USBD_ATTR_BYTEM_Msk */
  orrs    r5, r7
  str     r5, [r4, #16]

offset_loop:
  ldrb r6, [r1, #0]
  strb r6, [r0, #0]
  adds r0, r0, #1
  adds r1, r1, #1
  subs r2, r2, #1

  beq.n   word_mode_return
  tst     r0, r3
  bne.n   offset_loop
  tst     r1, r3
  bne.n   unaligned_loop_top

  ldr     r5, [r4, #16]
  bics    r5, r7
  str     r5, [r4, #16]
  b.n     aligned

unaligned:
  ldr     r4, =0x40060000
  ldr     r5, [r4, #16]
  movs    r7, #128 
  lsls    r7, r7, #3 /* USBD_ATTR_BYTEM_Msk */
  orrs    r5, r7
  str     r5, [r4, #16]

unaligned_loop_top:
  subs    r2, r2, #1
  ldrb    r5, [r1, r2]
  strb    r5, [r0, r2]
unaligned_loop_check:
  bne.n   unaligned_loop_top

  b.n     word_mode_return

#endif
