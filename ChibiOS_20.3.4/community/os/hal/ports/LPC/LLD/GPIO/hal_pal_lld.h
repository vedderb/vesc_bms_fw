/*
    ChibiOS - Copyright (C) 2020 Yaotian Feng / Codetector

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
 * @file    hal_pal_lld.h
 * @brief   LPC11Uxx PAL subsystem low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef HAL_PAL_LLD_H
#define HAL_PAL_LLD_H

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/

/* Specifies palInit() without parameter, required until all platforms will
   be updated to the new style.*/
#define PAL_NEW_INIT

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

/**
 * @name    Port related definitions
 * @{
 */
/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH           32U

/**
 * @brief   Whole port mask.
 * @details This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT              ((ioportmask_t)0xFFFFFFFFU)
/** @} */

/**
 * @name    Line handling macros
 * @{
 */
/**
 * @brief   Forms a line identifier.
 * @details A port/pad pair are encoded into an @p ioline_t type. The encoding
 *          of this type is platform-dependent.
 */
#define PAL_LINE(port, pad)                                                 \
  ((ioline_t)((uint32_t)(port)) | ((uint32_t)(pad)))

/**
 * @brief   Decodes a port identifier from a line identifier.
 */
#define PAL_PORT(line)                                                      \
   (((uint32_t)(line)) & 0xFFFFFF00U)

/**
 * @brief   Decodes a pad identifier from a line identifier.
 */
#define PAL_PAD(line)                                                       \
  (((uint32_t)(line) & 0x000000FFU))

/**
 * @brief   Value identifying an invalid line.
 */
#define PAL_NOLINE                      0U
/** @} */

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

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint32_t iomode_t;

#define   MODE_IOCONF_MASK        0xFFFF

#define   MODE_FUNC_POS           0U
#define   MODE_FUNC_MASK          (0x7U << MODE_FUNC_POS)
#define   MODE_FUNC_DEFAULT       (0x0U << MODE_FUNC_POS)
#define   MODE_FUNC_ALT1          (0x1U << MODE_FUNC_POS)
#define   MODE_FUNC_ALT2          (0x2U << MODE_FUNC_POS)
#define   MODE_FUNC_ALT3          (0x3U << MODE_FUNC_POS)
#define   MODE_FUNC_ALT4          (0x4U << MODE_FUNC_POS)
#define   MODE_FUNC_ALT5          (0x5U << MODE_FUNC_POS)
#define   MODE_FUNC_ALT6          (0x6U << MODE_FUNC_POS)
#define   MODE_FUNC_ALT7          (0x7U << MODE_FUNC_POS)

#define   MODE_MODE_POS           3U
#define   MODE_MODE_MASK          (0x3U << MODE_MODE_POS)
#define   MODE_MODE_INACTIVE      (0x0U << MODE_MODE_POS)
#define   MODE_MODE_PULL_DOWN     (0x1U << MODE_MODE_POS)
#define   MODE_MODE_PULL_UP       (0x2U << MODE_MODE_POS)
#define   MODE_MODE_REPEATER      (0x3U << MODE_MODE_POS)

#define   MODE_AD_POS             (7U)
#define   MODE_AD_MASK            (1U << MODE_AD_POS)
#define   MODE_AD_ANALOG          (0U << MODE_AD_POS)
#define   MODE_AD_DIGITAL         (1U << MODE_AD_POS)

#define   MODE_OD_ENABLE          (1U << 10U)

#define   MODE_DIR_POS            31U
#define   MODE_DIR_MASK           (0x1U << MODE_DIR_POS)
#define   MODE_DIR_IN             (0U << MODE_DIR_POS)
#define   MODE_DIR_OUT            (1U << MODE_DIR_POS)

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
typedef uint32_t ioportid_t;

/**
 * @brief   Type of an pad identifier.
 */
typedef uint32_t iopadid_t;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/*===========================================================================*/

/**
 * @brief   First I/O port identifier.
 * @details Low level drivers can define multiple ports, it is suggested to
 *          use this naming convention.
 */
#define LPC_IOPORT_ID(x)    ((x) << 8U)
#define LPC_IOPORT_NUM(x)   ((x) >> 8U)
#define IOPORT0         LPC_IOPORT_ID(0)
#define IOPORT1         LPC_IOPORT_ID(1)

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @notapi
 */
#define pal_lld_init() _pal_lld_init()

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port)                                              \
    (LPC_GPIO->PIN[LPC_IOPORT_NUM(port)])

/**
 * @brief   Reads the output latch.
 * @details The purpose of this function is to read back the latched output
 *          value.
 *
 * @param[in] port      port identifier
 * @return              The latched logical states.
 *
 * @notapi
 */
#define pal_lld_readlatch(port)                                           \
    (LPC_GPIO->PIN[LPC_IOPORT_NUM(port)])

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits)                                       \
  do {                                                                      \
    (LPC_GPIO->PIN[LPC_IOPORT_NUM(port)]) = bits;                           \
  } while (false)

/**
 * @brief   Pads group mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] mode      group mode
 *
 * @notapi
 */
#define pal_lld_setgroupmode(port, mask, offset, mode)                      \
  _pal_lld_setgroupmode(port, mask << offset, mode)

/**
 * @brief   Sets a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 *
 * @notapi
 */
#define pal_lld_setport(port, bits)                                         \
  do {                                                                      \
    (LPC_GPIO->SET[LPC_IOPORT_NUM(port)]) = bits;                           \
  } while (false)


/**
 * @brief   Clears a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 *
 * @notapi
 */
#define pal_lld_clearport(port, bits)                                       \
  do {                                                                      \
    (LPC_GPIO->CLR[LPC_IOPORT_NUM(port)]) = bits;                           \
  } while (false)


/**
 * @brief   Toggles a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be XORed on the specified port
 *
 * @notapi
 */
#define pal_lld_toggleport(port, bits)                                      \
  do {                                                                      \
    (LPC_GPIO->NOT[LPC_IOPORT_NUM(port)]) = bits;                           \
  } while (false)

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
#define pal_lld_readpad(port, pad)                                          \
  (LPC_GPIO->B[(PAL_IOPORTS_WIDTH * LPC_IOPORT_NUM(port)) + pad])

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
#define pal_lld_writepad(port, pad, bit)                                    \
  do {                                                                      \
    (LPC_GPIO->B[(PAL_IOPORTS_WIDTH * LPC_IOPORT_NUM(port)) + pad]) = bit;  \                                                              \
  } while (false)


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

/**
 * @brief   Returns a PAL event structure associated to a pad.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_get_pad_event(port, pad)                                    \
  &_pal_events[0]; (void)(port); (void)pad

/**
 * @brief   Returns a PAL event structure associated to a line.
 *
 * @param[in] line      line identifier
 *
 * @notapi
 */
#define pal_lld_get_line_event(line)                                        \
  &_pal_events[0]; (void)line

#if !defined(__DOXYGEN__)
#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
extern palevent_t _pal_events[1];
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_init(void);
  void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode);
  void _pal_lld_setpadmode(ioportid_t port,
                           iopadid_t pad,
                           iomode_t mode);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL == TRUE */

#endif /* HAL_PAL_LLD_H */
/** @} */
