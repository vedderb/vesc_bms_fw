/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_DEVICE_HSDCD_H__
#define __USB_DEVICE_HSDCD_H__
#include "fsl_common.h"
#include "fsl_device_registers.h"

/*!
 * @addtogroup usb_device_hsdcd_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/******************* Macro definitions ***************/

#include "usb_charger_detect.h"

/*! @brief USB DCD error code */
typedef enum _usb_phydcd_status
{
    kStatus_phydcd_Success = 0x00U, /*!< Success */
    kStatus_phydcd_Error,           /*!< Failed */
} usb_phydcd_status_t;
/*!
 * @brief Device dcd callback function typedef.
 *
 * This callback function is used to notify the upper layer that the device status has changed.
 * This callback pointer is passed by calling API dcd Init function.
 *
 * @param handle         The device handle. It equals the value returned from #USB_PHYDCD_Init.
 * @param callbackEvent  The callback event type. See enumeration #usb_device_event_t.
 * @param eventParam     The event parameter for this callback. The parameter type is determined by the callback event.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
typedef usb_phydcd_status_t (*usb_phydcd_callback_t)(void *handle, uint32_t callbackEvent, void *eventParam);
typedef void *usb_phydcd_handle;
/*! @brief dcd configuration structure */
typedef struct _usb_phydcd_config_struct
{
    usb_phydcd_callback_t dcdCallback; /*!< callbacl function for dcd detection*/
    void *dcdCallbackParam;            /*!< callbacl function parameter for dcd detection */
} usb_phydcd_config_struct_t;

/*! @brief Available common EVENT types in device callback */
typedef enum _usb_phydcd_control
{
    kUSB_DevicePHYDcdRun = 1U, /*!< USB dcd dectect start */
    kUSB_DevicePHYDcdStop,     /*!< USB dcd module stop */
    kUSB_DevicePHYDcdEnable,   /*!< Enable USB dcd dectect module */
    kUSB_DevicePHYDcdDisable,  /*!< Disable USB dcd module moudle */
} usb_phydcd_control_t;

/*!
 * @name USB device PHY dcd functions
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the USB dcd instance.
 *
 * This function initializes the USB dcd controller module specified by the base.
 *
 * @param[in] index      index of the usb analog .
 * @param[in] config    Pointer of the dcd configuration.
 * @param[out] dcdHandle   an output parameter, return the pointer of the dcd moudle handle to caller.
 *
 * @return A error code or kStatus_phydcd_Success.
 */
usb_phydcd_status_t USB_PHYDCD_Init(uint8_t index, usb_phydcd_config_struct_t *config, usb_phydcd_handle *dcdHandle);

/*!
 * @brief Deinitializes the USB dcd instance.
 *
 * This function Deinitializes the USB dcd module specified by the dcd handle.
 *
 * @param[in] dcdHandle      The dcd peripheral handle pointer.
 *
 * @return A dcd error code or kStatus_phydcd_Success.
 */

usb_phydcd_status_t USB_PHYDCD_Deinit(usb_phydcd_handle dcdHandle);

/*!
 * @brief Control the status of the selected item.
 *
 * This function is used to control the status of the selected item..
 *
 * @param handle                 The dcd handle. It equals the value returned from USB_DCD_Init.
 * @param type                   The control type, please refer to the enumeration usb_dcd_event_t.
 * @param param                  The param type is determined by the selected item. Or the param is NULL pointer.
 *
 * @retval kStatus_phydcd_Success              control the status successfully.
 * @retval kStatus_phydcd_Error                control the status failed .
 *
 */
usb_phydcd_status_t USB_PHYDCD_Control(usb_phydcd_handle handle, usb_phydcd_control_t type, void *param);
/*!
 * @brief update the status of the dcd detection based on time tick.
 *
 * This function is used to update dcd detction on time tick.
 *
 * @param handle                 The dcd handle. It equals the value returned from USB_DCD_Init.
 *
 * @retval kStatus_phydcd_Success              control the status successfully.
 * @retval kStatus_phydcd_Error                control the status failed .
 *
 */
usb_phydcd_status_t USB_PHYDCD_TimerIsrFunction(usb_phydcd_handle handle);
/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __USB_DEVICE_HSDCD_H__ */
