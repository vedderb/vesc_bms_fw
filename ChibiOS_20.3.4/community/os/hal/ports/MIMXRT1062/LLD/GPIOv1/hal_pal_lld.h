/*
    ChibiOS - Copyright (C) 2014-2015 Fabio Utzig

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
 * @file    GPIOv1/hal_pal_lld.h
 * @brief   PAL subsystem low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef HAL_PAL_LLD_H_
#define HAL_PAL_LLD_H_

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/

#define PAL_MODE_ALTERNATIVE_1      0x10
#define PAL_MODE_ALTERNATIVE_2      0x11
#define PAL_MODE_ALTERNATIVE_3      0x12
#define PAL_MODE_ALTERNATIVE_4      0x13
#define PAL_MODE_ALTERNATIVE_5      0x14
#define PAL_MODE_ALTERNATIVE_6      0x15
#define PAL_MODE_ALTERNATIVE_7      0x16

#define PIN_MUX_ALTERNATIVE(x)      IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(x)

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

#define TOTAL_PORTS                 9
#define PADS_PER_PORT               32

/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH 32

/**
 * @brief   Whole port mask.
 * @brief   This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT ((ioportmask_t)0xFFFFFFFF)

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint32_t iomode_t;

/**
 * @brief   Type of an I/O line.
 */
typedef uint32_t ioline_t;

/**
 * @brief   Port Identifier.
 * @details This type can be a scalar or some kind of pointer, do not make
 *          any assumption about it, use the provided macros when populating
 *          variables of this type.
 */
typedef GPIO_Type *ioportid_t;

/**
 * @brief   Type of an pad identifier.
 */
typedef uint32_t iopadid_t;

/**
 * @brief   Generic I/O ports static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialized the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {
} PALConfig;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/*===========================================================================*/

/**
 * @brief   GPIO1 identifier.
 */
#define IOPORT1          GPIO1

/**
 * @brief   GPIO2 identifier.
 */
#define IOPORT2          GPIO2

/**
 * @brief   GPIO3 identifier.
 */
#define IOPORT3          GPIO3

/**
 * @brief   GPIO4 identifier.
 */
#define IOPORT4          GPIO4

/**
 * @brief   GPIO5 identifier.
 */
#define IOPORT5          GPIO5

/**
 * @brief   GPIO6 identifier.
 */
#define IOPORT6          GPIO6

/**
 * @brief   GPIO7 identifier.
 */
#define IOPORT7          GPIO7

/**
 * @brief   GPIO8 identifier.
 */
#define IOPORT8          GPIO8

/**
 * @brief   GPIO9 identifier.
 */
#define IOPORT9          GPIO9

/**
 * @name    Line handling macros
 * @{
 */
/**
 * @brief   Forms a line identifier.
 * @details A port/pad pair are encoded into an @p ioline_t type. The encoding
 *          of this type is platform-dependent.
 * @note    In this driver the pad number is encoded in the byte of the GPIO
 *          address that's zero on all Kinetis devices.
 */
#define PAL_LINE(port, pad)                                                 \
  ((ioline_t)((uint32_t)(port) | ((uint32_t)(pad))))

/**
 * @brief   Decodes a port identifier from a line identifier.
 * @note    The GPIOn base addresses follow the division of port/pad
 *          and must match the division used in PAL_PORT and PAL_PAD.
 */
#define PAL_PORT(line)                                                      \
  ((GPIO_Type *)(((uint32_t)(line)) & 0xFFFFF000U))

/**
 * @brief   Decodes a pad identifier from a line identifier.
 * @note    The GPIOn base addresses follow the division of port/pad
 *          and must match the division used in PAL_PORT and PAL_PAD.
 */
#define PAL_PAD(line)                                                       \
  ((uint32_t)((uint32_t)(line) & 0x00000FFFU))

/**
 * @brief   Value identifying an invalid line.
 */
#define PAL_NOLINE                      0U
/** @} */

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @param[in] config    architecture-dependent ports configuration
 *
 * @notapi
 */
#define pal_lld_init(config) _pal_lld_init(config)

/**
 * @brief   Reads a logical state from an I/O pad.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @return              The logical state.
 * @retval PAL_LOW      low logical state.
 * @retval PAL_HIGH     high logical state.
 *
 * @notapi
 */
#define pal_lld_readpad(port, pad) _pal_lld_readpad(port, pad)

/**
 * @brief   Writes a logical state on an output pad.
 * @note    This function is not meant to be invoked directly by the
 *          application  code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] bit       logical value, the value must be @p PAL_LOW or
 *                      @p PAL_HIGH
 *
 * @notapi
 */
#define pal_lld_writepad(port, pad, bit) _pal_lld_writepad(port, pad, bit)

/**
 * @brief   Sets a pad logical state to @p PAL_HIGH.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_setpad(port, pad) _pal_lld_writepad(port, pad, PAL_HIGH)

/**
 * @brief   Clears a pad logical state to @p PAL_LOW.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_clearpad(port, pad) _pal_lld_writepad(port, pad, PAL_LOW)

/**
 * @brief   Toggles a pad logical state.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_togglepad(port, pad) _pal_lld_togglepad(port, pad)

/**
 * @brief   Pad mode setup.
 * @details This function programs a pad with the specified mode.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad mode
 *
 * @notapi
 */
#define pal_lld_setpadmode(port, pad, mode)                                 \
    _pal_lld_setpadmode(port, pad, mode)

#if !defined(__DOXYGEN__)
extern const PALConfig pal_default_config;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_init(const PALConfig *config);
  void _pal_lld_setpadmode(ioportid_t port,
                           uint8_t pad,
                           iomode_t mode);
  uint8_t _pal_lld_readpad(ioportid_t port,
                           uint8_t pad);
  void _pal_lld_writepad(ioportid_t port,
                         uint8_t pad,
                         uint8_t bit);
  void _pal_lld_togglepad(ioportid_t port,
			  uint8_t pad);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL */

#endif /* HAL_PAL_LLD_H_ */

/** @} */
