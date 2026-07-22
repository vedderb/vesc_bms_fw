/*
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
 * @file    vectors.S
 * @brief   Interrupt vectors for RISC-V ECLIC devices.
 *
 * @defgroup RISCV_ECLIC_STARTUP
 * @{
 */

#define _FROM_ASM_

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Code section.                                                             */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
.section .vectors, "a", %progbits
.globl vector_base
.type vector_base, %object
.option push
.option norelax
vector_base:
    .word vector0
    .word vector1
    .word vector2
    .word vector3
    .word vector4
    .word vector5
    .word vector6
    .word vector7
    .word vector8
    .word vector9
    .word vector10
    .word vector11
    .word vector12
    .word vector13
    .word vector14
    .word vector15
    .word vector16
    .word vector17
    .word vector18
    .word vector19
    .word vector20
    .word vector21
    .word vector22
    .word vector23
    .word vector24
    .word vector25
    .word vector26
    .word vector27
    .word vector28
    .word vector29
    .word vector30
    .word vector31
    .word vector32
    .word vector33
    .word vector34
    .word vector35
    .word vector36
    .word vector37
    .word vector38
    .word vector39
    .word vector40
    .word vector41
    .word vector42
    .word vector43
    .word vector44
    .word vector45
    .word vector46
    .word vector47
    .word vector48
    .word vector49
    .word vector50
    .word vector51
    .word vector52
    .word vector53
    .word vector54
    .word vector55
    .word vector56
    .word vector57
    .word vector58
    .word vector59
    .word vector60
    .word vector61
    .word vector62
    .word vector63
    .word vector64
    .word vector65
    .word vector66
    .word vector67
    .word vector68
    .word vector69
    .word vector70
    .word vector71
    .word vector72
    .word vector73
    .word vector74
    .word vector75
    .word vector76
    .word vector77
    .word vector78
    .word vector79
    .word vector80
    .word vector81
    .word vector82
    .word vector83
    .word vector84
    .word vector85
    .word vector86
    .size vector_base, . - vector_base
    .option pop

    .weak vector0
    .weak vector1
    .weak vector2
    .weak vector3
    .weak vector4
    .weak vector5
    .weak vector6
    .weak vector7
    .weak vector8
    .weak vector9
    .weak vector10
    .weak vector11
    .weak vector12
    .weak vector13
    .weak vector14
    .weak vector15
    .weak vector16
    .weak vector17
    .weak vector18
    .weak vector19
    .weak vector20
    .weak vector21
    .weak vector22
    .weak vector23
    .weak vector24
    .weak vector25
    .weak vector26
    .weak vector27
    .weak vector28
    .weak vector29
    .weak vector30
    .weak vector31
    .weak vector32
    .weak vector33
    .weak vector34
    .weak vector35
    .weak vector36
    .weak vector37
    .weak vector38
    .weak vector39
    .weak vector40
    .weak vector41
    .weak vector42
    .weak vector43
    .weak vector44
    .weak vector45
    .weak vector46
    .weak vector47
    .weak vector48
    .weak vector49
    .weak vector50
    .weak vector51
    .weak vector52
    .weak vector53
    .weak vector54
    .weak vector55
    .weak vector56
    .weak vector57
    .weak vector58
    .weak vector59
    .weak vector60
    .weak vector61
    .weak vector62
    .weak vector63
    .weak vector64
    .weak vector65
    .weak vector66
    .weak vector67
    .weak vector68
    .weak vector69
    .weak vector70
    .weak vector71
    .weak vector72
    .weak vector73
    .weak vector74
    .weak vector75
    .weak vector76
    .weak vector77
    .weak vector78
    .weak vector79
    .weak vector80
    .weak vector81
    .weak vector82
    .weak vector83
    .weak vector84
    .weak vector85
    .weak vector86

#endif /* !defined(__DOXYGEN__) */

/** @} */
