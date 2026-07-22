/*
     ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio
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
 * @file    xWDG/hal_wdg_lld.h
 * @brief   WDG Driver subsystem low level driver header.
 *
 * @addtogroup WDG
 * @{
 */

#ifndef HAL_WDG_LLD_H
#define HAL_WDG_LLD_H

#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    RLR register definitions
 * @{
 */
#define GD32_FWDGT_RLD_MASK                  (0x00000FFF << 0)
#define GD32_FWDGT_RLD(n)                    ((n) << 0)
/** @} */

/**
 * @name    PR register definitions
 * @{
 */
#define GD32_FWDGT_PSC_MASK                  (7 << 0)
#define GD32_FWDGT_PSC_4                     0U
#define GD32_FWDGT_PSC_8                     1U
#define GD32_FWDGT_PSC_16                    2U
#define GD32_FWDGT_PSC_32                    3U
#define GD32_FWDGT_PSC_64                    4U
#define GD32_FWDGT_PSC_128                   5U
#define GD32_FWDGT_PSC_256                   6U
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   FWDGT driver enable switch.
 * @details If set to @p TRUE the support for FWDGT is included.
 * @note    The default is @p FALSE.
 */
#if !defined(GD32_WDG_USE_FWDGT) || defined(__DOXYGEN__)
#define GD32_WDG_USE_FWDGT                  FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if GD32_WDG_USE_FWDGT && !GD32_HAS_FWDGT
#error "FWDGT not present in the selected device"
#endif

#if !GD32_WDG_USE_FWDGT
#error "WDG driver activated but no xWDG peripheral assigned"
#endif

#if !defined(GD32_IRC40K_ENABLED)
#error "GD32_IRC40K_ENABLED not defined"
#endif

#if (GD32_WDG_USE_FWDGT == TRUE) && (GD32_IRC40K_ENABLED == FALSE)
#error "FWDGT requires IRC40K clock"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an WDG driver.
 */
typedef struct WDGDriver WDGDriver;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Configuration of the FWDGT_PSC register.
   * @details See the GD32 reference manual for details.
   */
  uint32_t    psc;
  /**
   * @brief   Configuration of the FWDGT_RLD register.
   * @details See the GD32 reference manual for details.
   */
  uint32_t    rld;
} WDGConfig;

/**
 * @brief   Structure representing an WDG driver.
 */
struct WDGDriver {
  /**
   * @brief   Driver state.
   */
  wdgstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const WDGConfig           *config;
  /* End of the mandatory fields.*/
  /**
   * @brief   Pointer to the FWDGT registers block.
   */
  FWDGT_TypeDef              *wdg;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if GD32_WDG_USE_FWDGT && !defined(__DOXYGEN__)
extern WDGDriver WDGD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void wdg_lld_init(void);
  void wdg_lld_start(WDGDriver *wdgp);
  void wdg_lld_stop(WDGDriver *wdgp);
  void wdg_lld_reset(WDGDriver *wdgp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_WDG == TRUE */

#endif /* HAL_WDG_LLD_H */

/** @} */
