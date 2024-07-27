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
 * @file    devices/GD32VF103/eclic_parameters.h
 * @brief   GD32VF103 ECLIC configuration.
 *
 * @addtogroup GD32VF103
 * @{
 */
/**@} */
#ifndef _ECLIC_PARAMETERS_H_
#define _ECLIC_PARAMETERS_H_

/* Not defined because FPU isn't present. GCC produces alot 
of warnings if -Wundef is present, therefore this define. */
#define __riscv_flen 0

#include "gd32vf103.h"

#endif /* _ECLIC_PARAMETERS_H_ */