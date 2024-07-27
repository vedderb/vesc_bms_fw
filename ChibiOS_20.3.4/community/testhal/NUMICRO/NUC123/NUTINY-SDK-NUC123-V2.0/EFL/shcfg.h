/*
    Copyright (C) 2021 Alex Lewontin

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
 * @file    shcfg.h
 * @brief   Shell config header.
 *
 * @addtogroup Shell
 * @{
 */

#ifndef USBCFG_H
#define USBCFG_H

#include "hal.h"
#include "hal_mfs.h"
#include "shell.h"

#define SHELL_SERIAL_DRIVER SD0

#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(1024)

#ifdef __cplusplus
    extern "C" {
#endif

    extern const ShellConfig  shell_cfg;
    extern const ShellCommand commands[];

    extern MFSDriver       mfsd;
    extern const MFSConfig mfsd_config;

#ifdef __cplusplus
}
#endif

#endif /* USBCFG_H */

/** @} */
