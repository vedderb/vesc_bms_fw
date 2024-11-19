/*
    Copyright (C) 2020 Yaotian Feng

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
 * @file    hal_pal_lld.c
 * @brief   HT32 PAL subsystem low level driver source.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void initgpio(ioportid_t port, const struct port_setup * const setup) {
    PAL_PORT(port)->DIRCR = setup->DIR;
    PAL_PORT(port)->INER = setup->INE;
    PAL_PORT(port)->PUR = setup->PU;
    PAL_PORT(port)->PDR = setup->PD;
    PAL_PORT(port)->ODR = setup->OD;
    PAL_PORT(port)->DRVR = setup->DRV;
    const uint32_t portidx = HT32_PAL_IDX(port);
    AFIO->GPxCFGR[portidx][0] = setup->CFG[0];
    AFIO->GPxCFGR[portidx][1] = setup->CFG[1];
    if (setup->LOCK != 0) {
        PAL_PORT(port)->LOCKR = 0x5FA00000 | setup->LOCK;
    }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   HT32 I/O ports configuration.
 * @details Ports A-D(E, F, G, H) clocks enabled.
 *
 * @param[in] config    the HT32 ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config) {

    CKCU->AHBCCR |= (HT32_CCR_PAEN << HT32_NUM_GPIO) - HT32_CCR_PAEN;
    // enable AFIO
    CKCU->APBCCR0 |= CKCU_APBCCR0_AFIOEN;

    if (config == NULL)
        return;

    for (size_t i = 0; i < HT32_NUM_GPIO; i++) {
        initgpio(HT32_PAL_ID(i), &(config->setup[i]));
    }
    AFIO->ESSR[0] = config->ESSR[0];
    AFIO->ESSR[1] = config->ESSR[1];
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {

    if ((mode & PAL_HT32_MODE_AFE) != 0) {
        const uint32_t afn = (mode >> 8) & 0xf;
        const uint32_t portidx = HT32_PAL_IDX(port);
        uint32_t cfg[2];
        cfg[0] = AFIO->GPxCFGR[portidx][0];
        cfg[1] = AFIO->GPxCFGR[portidx][1];
        for (size_t i = 0; i < PAL_IOPORTS_WIDTH; i++) {
            if ((mask & (1U << i)) == 0)
                continue;
            cfg[i / 8] &= ~(0xfU << ((i % 8) * 4));
            cfg[i / 8] |= afn << ((i % 8) * 4);
        }
        AFIO->GPxCFGR[portidx][0] = cfg[0];
        AFIO->GPxCFGR[portidx][1] = cfg[1];
    }

    if ((mode & PAL_HT32_MODE_DIR) != 0) {
        PAL_PORT(port)->DIRCR |= mask;
    } else {
        PAL_PORT(port)->DIRCR &= ~mask;
    }

    if ((mode & PAL_HT32_MODE_INE) != 0) {
        PAL_PORT(port)->INER |= mask;
    } else {
        PAL_PORT(port)->INER &= ~mask;
    }

    if ((mode & PAL_HT32_MODE_PU) != 0) {
        PAL_PORT(port)->PUR |= mask;
    } else {
        PAL_PORT(port)->PUR &= ~mask;
    }

    if ((mode & PAL_HT32_MODE_PD) != 0) {
        PAL_PORT(port)->PDR |= mask;
    } else {
        PAL_PORT(port)->PDR &= ~mask;
    }

    if ((mode & PAL_HT32_MODE_OD) != 0) {
        PAL_PORT(port)->ODR |= mask;
    } else {
        PAL_PORT(port)->ODR &= ~mask;
    }

    if ((mode & PAL_HT32_MODE_DRV) != 0) {
        PAL_PORT(port)->DRVR |= mask;
    } else {
        PAL_PORT(port)->DRVR &= ~mask;
    }
}

#endif /* HAL_USE_PAL == TRUE */

/** @} */
