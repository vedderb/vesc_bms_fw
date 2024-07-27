/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"

#if ((defined(USB_DEVICE_CONFIG_DFU)) && (USB_DEVICE_CONFIG_DFU > 0U))
#include "usb_device_dfu.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static usb_status_t USB_DeviceDfuAllocateHandle(usb_device_dfu_struct_t **handle);
static usb_status_t USB_DeviceDfuFreeHandle(usb_device_dfu_struct_t *handle);

/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_GLOBAL static usb_device_dfu_struct_t s_UsbDeviceDfuHandle[USB_DEVICE_CONFIG_DFU];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Allocate a device dfu class handle.
 *
 * This function allocates a device dfu class handle.
 *
 * @param handle          It is out parameter, is used to return pointer of the device dfu class handle to the caller.
 *
 * @retval kStatus_USB_Success              Get a device dfu class handle successfully.
 * @retval kStatus_USB_Busy                 Cannot allocate a device dfu class handle.
 */
static usb_status_t USB_DeviceDfuAllocateHandle(usb_device_dfu_struct_t **handle)
{
    uint32_t count;
    for (count = 0U; count < USB_DEVICE_CONFIG_DFU; count++)
    {
        if (NULL == s_UsbDeviceDfuHandle[count].handle)
        {
            *handle = &s_UsbDeviceDfuHandle[count];
            return kStatus_USB_Success;
        }
    }

    return kStatus_USB_Busy;
}

/*!
 * @brief Free a device dfu class handle.
 *
 * This function frees a device dfu class handle.
 *
 * @param handle          The device dfu class handle.
 *
 * @retval kStatus_USB_Success              Free device dfu class handle successfully.
 */
static usb_status_t USB_DeviceDfuFreeHandle(usb_device_dfu_struct_t *handle)
{
    handle->handle       = NULL;
    handle->configStruct = (usb_device_class_config_struct_t *)NULL;
    return kStatus_USB_Success;
}

/*!
 * @brief Handle the event passed to the dfu class.
 *
 * This function handles the event passed to the dfu class.
 *
 * @param handle          The dfu class handle, got from the usb_device_class_config_struct_t::classHandle.
 * @param event           The event codes. Please refer to the enumeration usb_device_class_event_t.
 * @param param           The param type is determined by the event code.
 *
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success              Free device handle successfully.
 * @retval kStatus_USB_InvalidParameter     The device handle not be found.
 * @retval kStatus_USB_InvalidRequest       The request is invalid, and the control pipe will be stalled by the caller.
 */
usb_status_t USB_DeviceDfuEvent(void *handle, uint32_t event, void *param)
{
    usb_device_dfu_struct_t *dfuHandle;
    usb_status_t error                 = kStatus_USB_Error;
    usb_device_class_event_t eventCode = (usb_device_class_event_t)event;

    if ((NULL == param) || (NULL == handle))
    {
        return kStatus_USB_InvalidHandle;
    }

    /* Get the dfu class handle. */
    dfuHandle = (usb_device_dfu_struct_t *)handle;

    switch (eventCode)
    {
        case kUSB_DeviceClassEventDeviceReset:
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceClassEventSetConfiguration:
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceClassEventClassRequest:
        {
            /* Handle the dfu class specific request. */
            usb_device_control_request_struct_t *controlRequest = (usb_device_control_request_struct_t *)param;
            int32_t dfuRequest                                  = -1;

            if ((controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) !=
                USB_REQUEST_TYPE_RECIPIENT_INTERFACE)
            {
                break;
            }

            if ((controlRequest->setup->wIndex & 0xFFU) != 0x00U /* Interface number is always 0 */)
            {
                break;
            }

            switch (controlRequest->setup->bRequest)
            {
                case USB_DEVICE_DFU_DETACH:
                    dfuRequest = (int32_t)kUSB_DeviceDfuEventDetach;
                    break;
                case USB_DEVICE_DFU_DNLOAD:
                    dfuRequest = (int32_t)kUSB_DeviceDfuEventDownLoad;
                    break;
                case USB_DEVICE_DFU_UPLOAD:
                    dfuRequest = (int32_t)kUSB_DeviceDfuEventUpLoad;
                    break;
                case USB_DEVICE_DFU_GETSTATUS:
                    dfuRequest = (int32_t)kUSB_DeviceDfuEventGetStatus;
                    break;
                case USB_DEVICE_DFU_CLRSTATUS:
                    dfuRequest = (int32_t)kUSB_DeviceDfuEventClearStatus;
                    break;
                case USB_DEVICE_DFU_GETSTATE:
                    dfuRequest = (int32_t)kUSB_DeviceDfuEventGetState;
                    break;
                case USB_DEVICE_DFU_ABORT:
                    dfuRequest = (int32_t)kUSB_DeviceDfuEventAbort;
                    break;
                default:
                    error = kStatus_USB_InvalidRequest;
                    break;
            }
            if (dfuRequest >= 0)
            {
                /* ClassCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
                it is from the second parameter of classInit */
                error = dfuHandle->configStruct->classCallback((class_handle_t)dfuHandle, dfuRequest, controlRequest);
            }
        }
        break;
        default:
            /*no action*/
            break;
    }
    return error;
}

/*!
 * @brief Initialize the dfu class.
 *
 * This function is used to initialize the dfu class.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param config         The class configuration information.
 * @param handle         It is out parameter, is used to return pointer of the dfu class handle to the caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceDfuInit(uint8_t controllerId, usb_device_class_config_struct_t *config, class_handle_t *handle)
{
    usb_device_dfu_struct_t *dfuHandle;
    usb_status_t error;

    /* Allocate a dfu class handle. */
    error = USB_DeviceDfuAllocateHandle(&dfuHandle);

    if (kStatus_USB_Success != error)
    {
        return error;
    }

    /* Get the device handle according to the controller id. */
    error = USB_DeviceClassGetDeviceHandle(controllerId, &dfuHandle->handle);

    if (kStatus_USB_Success != error)
    {
        return error;
    }

    if (NULL == dfuHandle->handle)
    {
        return kStatus_USB_InvalidHandle;
    }
    /* Save the configuration of the class. */
    dfuHandle->configStruct = config;

    *handle = (class_handle_t)dfuHandle;
    return error;
}

/*!
 * @brief De-initialize the device dfu class.
 *
 * The function de-initializes the device dfu class.
 *
 * @param handle The dfu class handle got from usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceDfuDeinit(class_handle_t handle)
{
    usb_device_dfu_struct_t *dfuHandle;
    usb_status_t error = kStatus_USB_Error;

    dfuHandle = (usb_device_dfu_struct_t *)handle;

    if (NULL == dfuHandle)
    {
        return kStatus_USB_InvalidHandle;
    }

    /* Free the dfu class handle. */
    (void)USB_DeviceDfuFreeHandle(dfuHandle);
    return error;
}
#endif
