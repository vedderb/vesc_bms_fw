/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016,2019 - 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_host_config.h"
#if ((defined USB_HOST_CONFIG_AUDIO) && (USB_HOST_CONFIG_AUDIO))
#include "usb_host.h"
#include "usb_host_audio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef union usb_audio_ctrl_common_fu_desc_t
{
    usb_audio_ctrl_fu_desc_t fuDesc10;
    usb_audio_2_0_ctrl_fu_desc_t fuDesc20;
} usb_audio_ctrl_common_fu_desc;
/*******************************************************************************
 * Variables
 ******************************************************************************/

/* usb audio feature uint command request declaration */
static usb_audio_request_t s_usbAudioFuRequests[NUMBER_OF_FEATURE_COMMANDS] = {
    /* Mute command request  */
    {AUDIO_FU_MUTE_MASK, ITF_REQUEST, CUR_REQUEST, AUDIO_FU_MUTE, 1U},
    /* Volume command request  */
    {AUDIO_FU_VOLUME_MASK, ITF_REQUEST, CUR_REQUEST, AUDIO_FU_VOLUME, 2U},
    {AUDIO_FU_VOLUME_MASK, ITF_REQUEST, MIN_REQUEST, AUDIO_FU_VOLUME, 2U},
    {AUDIO_FU_VOLUME_MASK, ITF_REQUEST, MAX_REQUEST, AUDIO_FU_VOLUME, 2U},
    {AUDIO_FU_VOLUME_MASK, ITF_REQUEST, RES_REQUEST, AUDIO_FU_VOLUME, 2U},
};
/* USB audio endpoint command declaration */
static usb_audio_request_t s_usbAudioEpRequests[NUMBER_OF_ENDPOINT_COMMANDS] = {
    /* USB audio Pitch command request  */
    {AUDIO_PITCH_MASK, EP_REQUEST, CUR_REQUEST, AUDIO_PITCH_CONTROL, 1U},

    /* USB audio Sampling frequency command request  */
    {AUDIO_SAMPLING_FREQ_MASK, EP_REQUEST, CUR_REQUEST, AUDIO_SAMPLING_FREQ_CONTROL, 3U},
    {AUDIO_SAMPLING_FREQ_MASK, EP_REQUEST, MIN_REQUEST, AUDIO_SAMPLING_FREQ_CONTROL, 3U},
    {AUDIO_SAMPLING_FREQ_MASK, EP_REQUEST, MAX_REQUEST, AUDIO_SAMPLING_FREQ_CONTROL, 3U},
    {AUDIO_SAMPLING_FREQ_MASK, EP_REQUEST, RES_REQUEST, AUDIO_SAMPLING_FREQ_CONTROL, 3U},
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief initialize the audio instance.
 *
 * This function allocate the resource for audio instance.
 *
 * @param deviceHandle       The device handle.
 * @param classHandlePtr return class handle.
 *
 * @retval kStatus_USB_Success        The device is initialized successfully.
 * @retval kStatus_USB_AllocFail      Allocate memory fail.
 */
usb_status_t USB_HostAudioInit(usb_device_handle deviceHandle, usb_host_class_handle *classHandlePtr)
{
    audio_instance_t *audioPtr = (audio_instance_t *)OSA_MemoryAllocate(sizeof(audio_instance_t));
    uint32_t info_value;
    uint32_t *temp;

    if (audioPtr == NULL)
    {
        return kStatus_USB_AllocFail;
    }

    audioPtr->deviceHandle      = deviceHandle;
    audioPtr->controlIntfHandle = NULL;
    audioPtr->streamIntfHandle  = NULL;
    (void)USB_HostHelperGetPeripheralInformation(deviceHandle, (uint32_t)kUSB_HostGetHostHandle, &info_value);
    temp                 = (uint32_t *)info_value;
    audioPtr->hostHandle = (usb_host_handle)temp;
    (void)USB_HostHelperGetPeripheralInformation(deviceHandle, (uint32_t)kUSB_HostGetDeviceControlPipe, &info_value);
    temp                  = (uint32_t *)info_value;
    audioPtr->controlPipe = (usb_host_pipe_handle)temp;

    *classHandlePtr = audioPtr;
    return kStatus_USB_Success;
}

/*!
 * @brief de-initialize the audio instance.
 *
 * This function release the resource for audio instance.
 *
 * @param deviceHandle   the device handle.
 * @param classHandle the class handle.
 *
 * @retval kStatus_USB_Success        The device is de-initialized successfully.
 */
usb_status_t USB_HostAudioDeinit(usb_device_handle deviceHandle, usb_host_class_handle classHandle)
{
    usb_status_t status        = kStatus_USB_Success;
    audio_instance_t *audioPtr = (audio_instance_t *)classHandle;

    if (deviceHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (classHandle != NULL)
    {
        if (audioPtr->isoInPipe != NULL)
        {
            status = USB_HostCancelTransfer(audioPtr->hostHandle, audioPtr->isoInPipe, NULL);
            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("error when cancel pipe\r\n");
#endif
            }
            status = USB_HostClosePipe(audioPtr->hostHandle, audioPtr->isoInPipe);

            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("error when close pipe\r\n");
#endif
            }
            audioPtr->isoInPipe = NULL;
        }
        if (audioPtr->isoOutPipe != NULL)
        {
            status = USB_HostCancelTransfer(audioPtr->hostHandle, audioPtr->isoOutPipe, NULL);
            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("error when cancel pipe\r\n");
#endif
            }
            status = USB_HostClosePipe(audioPtr->hostHandle, audioPtr->isoOutPipe);

            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("error when close pipe\r\n");
#endif
            }
            audioPtr->isoOutPipe = NULL;
        }
        (void)USB_HostCloseDeviceInterface(deviceHandle, audioPtr->streamIntfHandle);

        if ((audioPtr->controlPipe != NULL) && (audioPtr->controlTransfer != NULL))
        {
            status = USB_HostCancelTransfer(audioPtr->hostHandle, audioPtr->controlPipe, audioPtr->controlTransfer);
        }
        (void)USB_HostCloseDeviceInterface(deviceHandle, audioPtr->controlIntfHandle);
        OSA_MemoryFree(audioPtr);
    }
    else
    {
        (void)USB_HostCloseDeviceInterface(deviceHandle, NULL);
    }

    return status;
}

/*!
 * @brief audiostream iso in pipe transfer callback.
 *
 * @param param       callback parameter.
 * @param transfer     callback transfer.
 * @param status        transfer status.
 */
static void _USB_HostAudioStreamIsoInPipeCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    audio_instance_t *audioPtr = (audio_instance_t *)param;

    if (audioPtr->inCallbackFn != NULL)
    {
        /* callback function is initialized in USB_HostAudioStreamRecv */
        audioPtr->inCallbackFn(audioPtr->inCallbackParam, transfer->transferBuffer, transfer->transferSofar, status);
    }
    (void)USB_HostFreeTransfer(audioPtr->hostHandle, transfer);
}

/*!
 * @brief audiostream iso out pipe transfer callback.
 *
 * @param param       callback parameter.
 * @param transfer     callback transfer.
 * @param status        transfer status.
 */
static void _USB_HostAudioStreamIsoOutPipeCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    audio_instance_t *audioPtr = (audio_instance_t *)param;

    if (audioPtr->outCallbackFn != NULL)
    {
        /* callback function is initialized in USB_HostAudioStreamSend */
        audioPtr->outCallbackFn(audioPtr->outCallbackParam, transfer->transferBuffer, transfer->transferSofar, status);
    }
    (void)USB_HostFreeTransfer(audioPtr->hostHandle, transfer);
}

/*!
 * @brief audiocontrol pipe transfer callback.
 *
 * @param param       callback parameter.
 * @param transfer     callback transfer.
 * @param status        transfer status.
 */
static void _USB_HostAudioControlCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    audio_instance_t *audioPtr = (audio_instance_t *)param;

    audioPtr->controlTransfer = NULL;
    if (audioPtr->controlCallbackFn != NULL)
    {
        /* callback to application, callback function is initialized in the _USB_HostAudioControl,
        USB_HostAudioStreamSetInterface
        or USB_HostAudioControlSetInterface, but is the same function */
        audioPtr->controlCallbackFn(audioPtr->controlCallbackParam, transfer->transferBuffer, transfer->transferSofar,
                                    status);
    }
    (void)USB_HostFreeTransfer(audioPtr->hostHandle, transfer);
    audioPtr->isSetup = 0U;
}

/*!
 * @brief hid send control transfer common code.
 *
 * @param classHandle   the class handle.
 * @param typeRequest   setup packet request type.
 * @param request           setup packet request value.
 * @param wvalue            setup packet wvalue value.
 * @param windex            setup packet index value.
 * @param wlength           setup packet wlength value.
 * @param data                data buffer pointer will be transfer.
 * @param callbackFn       this callback is called after this function completes.
 * @param callbackParam the first parameter in the callback function.
 *
 * @return An error code or kStatus_USB_Success.
 */
static usb_status_t _USB_HostAudioControl(usb_host_class_handle classHandle,
                                          uint8_t typeRequest,
                                          uint8_t request,
                                          uint16_t wvalue,
                                          uint16_t windex,
                                          uint16_t wlength,
                                          uint8_t *data,
                                          transfer_callback_t callbackFn,
                                          void *callbackParam)
{
    audio_instance_t *audioPtr = (audio_instance_t *)classHandle;
    usb_host_transfer_t *transfer;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (USB_HostMallocTransfer(audioPtr->hostHandle, &transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("error to get transfer\r\n");
#endif
        return kStatus_USB_Error;
    }
    audioPtr->controlCallbackFn    = callbackFn;
    audioPtr->controlCallbackParam = callbackParam;

    transfer->transferBuffer             = data;
    transfer->transferLength             = wlength;
    transfer->callbackFn                 = _USB_HostAudioControlCallback;
    transfer->callbackParam              = audioPtr;
    transfer->setupPacket->bmRequestType = typeRequest;
    transfer->setupPacket->bRequest      = request;
    transfer->setupPacket->wValue        = USB_SHORT_TO_LITTLE_ENDIAN(wvalue);
    transfer->setupPacket->wIndex        = USB_SHORT_TO_LITTLE_ENDIAN(windex);

    transfer->setupPacket->wLength = USB_SHORT_TO_LITTLE_ENDIAN(wlength);
    audioPtr->isSetup              = 1;

    if (USB_HostSendSetup(audioPtr->hostHandle, audioPtr->controlPipe, transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("failed for USB_HostSendSetup\r\n");
#endif
        (void)USB_HostFreeTransfer(audioPtr->hostHandle, transfer);
        return kStatus_USB_Error;
    }
    audioPtr->controlTransfer = transfer;

    return kStatus_USB_Success;
}

static usb_status_t _USB_HostAudioInitEndpoint(audio_instance_t *audioPtr, usb_descriptor_endpoint_t *ep_desc)
{
    usb_host_pipe_init_t pipe_init;
    usb_status_t status;

    pipe_init.devInstance     = audioPtr->deviceHandle;
    pipe_init.pipeType        = USB_ENDPOINT_ISOCHRONOUS;
    pipe_init.direction       = ((ep_desc->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
    pipe_init.endpointAddress = (ep_desc->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_NUMBER_MASK);
    pipe_init.interval        = ep_desc->bInterval;
    pipe_init.maxPacketSize   = (uint16_t)((USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(ep_desc->wMaxPacketSize) &
                                          USB_DESCRIPTOR_ENDPOINT_MAXPACKETSIZE_SIZE_MASK));
    pipe_init.numberPerUframe = (uint8_t)((USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(ep_desc->wMaxPacketSize) &
                                           USB_DESCRIPTOR_ENDPOINT_MAXPACKETSIZE_MULT_TRANSACTIONS_MASK));
    pipe_init.nakCount        = USB_HOST_CONFIG_MAX_NAK;

    if (pipe_init.direction == USB_IN)
    {
        audioPtr->inPacketSize = pipe_init.maxPacketSize;
        audioPtr->isoEpNum     = pipe_init.endpointAddress;
        status                 = USB_HostOpenPipe(audioPtr->hostHandle, &audioPtr->isoInPipe, &pipe_init);
    }
    else
    {
        audioPtr->outPacketSize = pipe_init.maxPacketSize;
        audioPtr->isoEpNum      = pipe_init.endpointAddress;
        status                  = USB_HostOpenPipe(audioPtr->hostHandle, &audioPtr->isoOutPipe, &pipe_init);
    }

    return status;
}

/*!
 * @brief audio open interface.
 *
 * @param audioPtr     audio instance pointer.
 *
 * @return kStatus_USB_Success or error codes.
 */
static usb_status_t _USB_HostAudioOpenInterface(audio_instance_t *audioPtr)
{
    usb_status_t status;
    uint8_t ep_index                   = 0U;
    usb_descriptor_endpoint_t *ep_desc = NULL;
    usb_host_interface_t *interface_ptr;

    if (audioPtr->isoInPipe != NULL)
    {
        status = USB_HostClosePipe(audioPtr->hostHandle, audioPtr->isoInPipe);

        if (status != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error when close pipe\r\n");
#endif
        }
        audioPtr->isoInPipe = NULL;
    }
    if (audioPtr->isoOutPipe != NULL)
    {
        status = USB_HostClosePipe(audioPtr->hostHandle, audioPtr->isoOutPipe);

        if (status != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error when close pipe\r\n");
#endif
        }
        audioPtr->isoOutPipe = NULL;
    }

    /* open interface pipes */
    interface_ptr = (usb_host_interface_t *)audioPtr->streamIntfHandle;
    for (ep_index = 0U; ep_index < interface_ptr->epCount; ++ep_index)
    {
        ep_desc = interface_ptr->epList[ep_index].epDesc;
        if (((ep_desc->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
             USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN) &&
            ((ep_desc->bmAttributes & USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_TYPE_MASK) == USB_ENDPOINT_ISOCHRONOUS))
        {
            status = _USB_HostAudioInitEndpoint(audioPtr, ep_desc);
            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("usb_host_audio_stream_set_interface fail to open pipe\r\n");
#endif
                return kStatus_USB_Error;
            }
        }
        else if (((ep_desc->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                  USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
                 ((ep_desc->bmAttributes & USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_TYPE_MASK) == USB_ENDPOINT_ISOCHRONOUS))
        {
            status = _USB_HostAudioInitEndpoint(audioPtr, ep_desc);
            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("usb_host_audio_stream_set_interface fail to open pipe\r\n");
#endif
                return kStatus_USB_Error;
            }
        }
        else
        {
        }
    }

    return kStatus_USB_Success;
}

/*!
 * @brief audio set interface callback, open pipes.
 *
 * @param param       callback parameter.
 * @param transfer    callback transfer.
 * @param status      transfer status.
 */
static void _USB_HostAudioSetInterfaceCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    audio_instance_t *audioPtr = (audio_instance_t *)param;

    audioPtr->controlTransfer = NULL;
    if (status == kStatus_USB_Success)
    {
        status = _USB_HostAudioOpenInterface(audioPtr);
    }

    if (audioPtr->controlCallbackFn != NULL)
    {
        /* callback to application, callback function is initialized in the _USB_HostAudioControl,
        USB_HostAudioStreamSetInterface
        or USB_HostAudioControlSetInterface, but is the same function */
        audioPtr->controlCallbackFn(audioPtr->controlCallbackParam, NULL, 0U, status);
    }
    (void)USB_HostFreeTransfer(audioPtr->hostHandle, transfer);
}

/*!
 * @brief set audioclass stream interface.
 *
 * This function bind the interface with the audio instance.
 *
 * @param classHandle        The class handle.
 * @param interfaceHandle          The interface handle.
 * @param alternateSetting  The alternate setting value.
 * @param callbackFn          This callback is called after this function completes.
 * @param callbackParam    The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success         The device is initialized successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy           There is no idle transfer.
 * @retval kStatus_USB_Error          send transfer fail, please reference to USB_HostSendSetup.
 * @retval kStatus_USB_Busy           callback return status, there is no idle pipe.
 * @retval kStatus_USB_TransferStall  callback return status, the transfer is stall by device.
 * @retval kStatus_USB_Error          callback return status, open pipe fail, please reference to USB_HostOpenPipe.
 */
usb_status_t USB_HostAudioStreamSetInterface(usb_host_class_handle classHandle,
                                             usb_host_interface_handle interfaceHandle,
                                             uint8_t alternateSetting,
                                             transfer_callback_t callbackFn,
                                             void *callbackParam)
{
    usb_status_t status;
    audio_instance_t *audioPtr = (audio_instance_t *)classHandle;
    usb_host_interface_t *interface_ptr;
    usb_host_transfer_t *transfer;
    audio_descriptor_union_t ptr1;
    uint32_t length = 0U, ep = 0U;
    void *temp;
    usb_descriptor_endpoint_t *endpointDesc;
    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidParameter;
    }

    audioPtr->streamIntfHandle = interfaceHandle;

    status = USB_HostOpenDeviceInterface(audioPtr->deviceHandle, interfaceHandle);
    if (status != kStatus_USB_Success)
    {
        return status;
    }

    if (audioPtr->isoInPipe != NULL)
    {
        status = USB_HostCancelTransfer(audioPtr->hostHandle, audioPtr->isoInPipe, NULL);

        if (status != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error when cancel pipe\r\n");
#endif
        }
    }

    if (audioPtr->isoOutPipe != NULL)
    {
        status = USB_HostCancelTransfer(audioPtr->hostHandle, audioPtr->isoOutPipe, NULL);

        if (status != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error when cancel pipe\r\n");
#endif
        }
    }
    /* open interface pipes */
    interface_ptr = (usb_host_interface_t *)interfaceHandle;
    ptr1.bufr     = interface_ptr->interfaceExtension;

    length = 0U;
    while (length < interface_ptr->interfaceExtensionLength)
    {
        if ((ptr1.common->bDescriptorType == 0x04U) && (ptr1.interface->bAlternateSetting == alternateSetting))
        {
            interface_ptr->epCount = ptr1.interface->bNumEndpoints;
            break;
        }
        ptr1.bufr += ptr1.common->bLength;
        length += ptr1.common->bLength;
    }
    while (ep < interface_ptr->epCount)
    {
        if (ptr1.common->bDescriptorType == 0x24U)
        {
            temp = (void *)ptr1.bufr;
            if (AUDIO_DEVICE_VERSION_01 == audioPtr->deviceAudioVersion)
            {
                if (ptr1.common->bData[0] == USB_AUDIO_DESC_SUBTYPE_AS_GENERAL)
                {
                    audioPtr->asIntfDesc = temp;
                }
                else if (ptr1.common->bData[0] == USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_TYPE)
                {
                    audioPtr->formatTypeDesc = temp;
                }
                else
                {
                    /*no action*/
                }
            }
            else if (AUDIO_DEVICE_VERSION_02 == audioPtr->deviceAudioVersion)
            {
                if (ptr1.common->bData[0] == USB_AUDIO_DESC_SUBTYPE_AS_GENERAL_20)
                {
                    audioPtr->asIntfDesc = temp;
                }
                else if (ptr1.common->bData[0] == USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_TYPE_20)
                {
                    audioPtr->formatTypeDesc = temp;
                }
                else
                {
                    /*no action*/
                }
            }
            else
            {
                /*no action*/
            }
        }
        else
        {
            /*no action*/
        }

        if (ptr1.common->bDescriptorType == 0x05U)
        {
            uint32_t descLength = 0;
            temp                = (void *)ptr1.bufr;
            /*Bits 5..4: Usage Type,
            00 = Data endpoint
            01 = Feedback endpoint
            10 = Implicit feedback Data endpoit, usb Spec 9.6*/
            endpointDesc = (usb_descriptor_endpoint_t *)temp;
            if (0x00U == ((endpointDesc->bmAttributes >> 4U) & 0x3U))
            {
                interface_ptr->epList[ep].epDesc = (usb_descriptor_endpoint_t *)temp;
                audioPtr->isoEndpDesc            = (usb_descriptor_endpoint_t *)temp;
            }
            else if (0x01U == ((endpointDesc->bmAttributes >> 4U) & 0x3U))
            {
                /*feedback endpoint*/
            }
            else
            {
                /*TO DO*/
            }

            descLength = ptr1.common->bLength;
            ptr1.bufr += ptr1.common->bLength;

            if ((0x25U == ptr1.common->bDescriptorType))
            {
                interface_ptr->epList[ep].epExtension       = ptr1.bufr;
                interface_ptr->epList[ep].epExtensionLength = ptr1.common->bLength;
            }
            else
            {
                ptr1.bufr -= descLength;
            }
            ep++;
        }
        ptr1.bufr += ptr1.common->bLength;
    }

    if (alternateSetting == 0U)
    {
        if (callbackFn != NULL)
        {
            status = _USB_HostAudioOpenInterface(audioPtr);
            callbackFn(callbackParam, NULL, 0U, kStatus_USB_Success);
        }
    }
    else
    {
        if (USB_HostMallocTransfer(audioPtr->hostHandle, &transfer) != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error to get transfer\r\n");
#endif
            return kStatus_USB_Error;
        }
        audioPtr->controlCallbackFn    = callbackFn;
        audioPtr->controlCallbackParam = callbackParam;
        /* initialize transfer */
        transfer->callbackFn                 = _USB_HostAudioSetInterfaceCallback;
        transfer->callbackParam              = audioPtr;
        transfer->setupPacket->bRequest      = USB_REQUEST_STANDARD_SET_INTERFACE;
        transfer->setupPacket->bmRequestType = USB_REQUEST_TYPE_RECIPIENT_INTERFACE;
        transfer->setupPacket->wIndex        = USB_SHORT_TO_LITTLE_ENDIAN(
            ((usb_host_interface_t *)audioPtr->streamIntfHandle)->interfaceDesc->bInterfaceNumber);
        transfer->setupPacket->wValue  = USB_SHORT_TO_LITTLE_ENDIAN(alternateSetting);
        transfer->setupPacket->wLength = 0;
        transfer->transferBuffer       = NULL;
        transfer->transferLength       = 0;
        status                         = USB_HostSendSetup(audioPtr->hostHandle, audioPtr->controlPipe, transfer);

        if (status == kStatus_USB_Success)
        {
            audioPtr->controlTransfer = transfer;
        }
        else
        {
            (void)USB_HostFreeTransfer(audioPtr->hostHandle, transfer);
        }
    }

    return status;
}

/*!
 * @brief set audioclass control interface.
 *
 * This function bind the interface with the audio instance.
 *
 * @param classHandle        The class handle.
 * @param interfaceHandle          The interface handle.
 * @param alternateSetting  The alternate setting value.
 * @param callbackFn          This callback is called after this function completes.
 * @param callbackParam    The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success         The device is initialized successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy           There is no idle transfer.
 * @retval kStatus_USB_Error          send transfer fail, please reference to USB_HostSendSetup.
 * @retval kStatus_USB_Busy           callback return status, there is no idle pipe.
 * @retval kStatus_USB_TransferStall  callback return status, the transfer is stall by device.
 * @retval kStatus_USB_Error          callback return status, open pipe fail, please reference to USB_HostOpenPipe.
 */
usb_status_t USB_HostAudioControlSetInterface(usb_host_class_handle classHandle,
                                              usb_host_interface_handle interfaceHandle,
                                              uint8_t alternateSetting,
                                              transfer_callback_t callbackFn,
                                              void *callbackParam)
{
    usb_status_t status;
    audio_instance_t *audioPtr = (audio_instance_t *)classHandle;
    usb_host_interface_t *interface_ptr;
    usb_host_transfer_t *transfer;
    audio_descriptor_union_t ptr1;
    void *temp;
    void *header;
    usb_audio_ctrl_common_header_desc_t *commonHeader;
    uint32_t length = 0U;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidParameter;
    }
    audioPtr->controlIntfHandle = interfaceHandle;
    interface_ptr               = (usb_host_interface_t *)interfaceHandle;

    status = USB_HostOpenDeviceInterface(audioPtr->deviceHandle, interfaceHandle);
    if (status != kStatus_USB_Success)
    {
        return status;
    }
    ptr1.bufr = interface_ptr->interfaceExtension;

    length = 0U;

    while (length < interface_ptr->interfaceExtensionLength)
    {
        if (((interface_ptr->interfaceDesc->bDescriptorType == 0x04U) &&
             (interface_ptr->interfaceDesc->bAlternateSetting == alternateSetting)) ||
            ((ptr1.common->bDescriptorType == 0x04U) && (ptr1.interface->bAlternateSetting == alternateSetting)))
        {
            break;
        }

        ptr1.bufr += ptr1.common->bLength;
        length += ptr1.common->bLength;
    }
    while (length < interface_ptr->interfaceExtensionLength)
    {
        if (ptr1.common->bDescriptorType == 0x24U)
        {
            temp = (void *)ptr1.bufr;
            if (ptr1.common->bData[0] == USB_AUDIO_DESC_SUBTYPE_CS_HEADER)
            {
                audioPtr->headerDesc         = (void *)temp;
                commonHeader                 = (usb_audio_ctrl_common_header_desc_t *)temp;
                header                       = (void *)&commonHeader->bcdcdc;
                audioPtr->deviceAudioVersion = USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(((uint8_t *)header));
            }
            else if (ptr1.common->bData[0] == USB_AUDIO_DESC_SUBTYPE_CS_INPUT)
            {
                audioPtr->itDesc = (void *)temp;
            }
            else if (ptr1.common->bData[0] == USB_AUDIO_DESC_SUBTYPE_CS_OUTPUT)
            {
                audioPtr->otDesc = (void *)temp;
            }
            else if (ptr1.common->bData[0] == USB_AUDIO_DESC_SUBTYPE_CS_FEATURE)
            {
                audioPtr->fuDesc = (void *)temp;
            }
            else if (ptr1.common->bData[0] == USB_AUDIO_DESC_SUBTYPE_CS_CLOCK_SOURE)
            {
                audioPtr->clockSource = (void *)temp;
            }
            else
            {
                /*no action*/
            }
        }
        ptr1.bufr += ptr1.common->bLength;
        length += ptr1.common->bLength;
    }

    if (alternateSetting == 0U)
    {
        if (callbackFn != NULL)
        {
            callbackFn(callbackParam, NULL, 0U, kStatus_USB_Success);
        }
    }
    else
    {
        if (USB_HostMallocTransfer(audioPtr->hostHandle, &transfer) != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error to get transfer\r\n");
#endif
            return kStatus_USB_Error;
        }
        audioPtr->controlCallbackFn    = callbackFn;
        audioPtr->controlCallbackParam = callbackParam;
        /* initialize transfer */
        transfer->callbackFn                 = _USB_HostAudioControlCallback;
        transfer->callbackParam              = audioPtr;
        transfer->setupPacket->bRequest      = USB_REQUEST_STANDARD_SET_INTERFACE;
        transfer->setupPacket->bmRequestType = USB_REQUEST_TYPE_RECIPIENT_INTERFACE;
        transfer->setupPacket->wIndex        = USB_SHORT_TO_LITTLE_ENDIAN(
            ((usb_host_interface_t *)audioPtr->controlIntfHandle)->interfaceDesc->bInterfaceNumber);
        transfer->setupPacket->wValue  = USB_SHORT_TO_LITTLE_ENDIAN(alternateSetting);
        transfer->setupPacket->wLength = 0;
        transfer->transferBuffer       = NULL;
        transfer->transferLength       = 0;
        status                         = USB_HostSendSetup(audioPtr->hostHandle, audioPtr->controlPipe, transfer);

        if (status == kStatus_USB_Success)
        {
            audioPtr->controlTransfer = transfer;
        }
        else
        {
            (void)USB_HostFreeTransfer(audioPtr->hostHandle, transfer);
        }
    }

    return status;
}

/*!
 * @brief get pipe max packet size.
 *
 * @param classHandle the class handle.
 * @param pipeType    It's value is USB_ENDPOINT_CONTROL, USB_ENDPOINT_ISOCHRONOUS, USB_ENDPOINT_BULK or
 * USB_ENDPOINT_INTERRUPT.
 *                     Please reference to usb_spec.h
 * @param direction    pipe direction.
 *
 * @retval 0        The classHandle is NULL.
 * @retval max packet size.
 */
uint16_t USB_HostAudioPacketSize(usb_host_class_handle classHandle, uint8_t pipeType, uint8_t direction)
{
    audio_instance_t *audioPtr = (audio_instance_t *)classHandle;
    if (classHandle == NULL)
    {
        return 0U;
    }

    if (pipeType == USB_ENDPOINT_ISOCHRONOUS)
    {
        if (direction == USB_IN)
        {
            return audioPtr->inPacketSize;
        }
        else
        {
            return audioPtr->outPacketSize;
        }
    }
    return 0U;
}

/*!
 * @brief audio stream receive data.
 *
 * This function implements audioreceiving data.
 *
 * @param classHandle      The class handle.
 * @param buffer                The buffer pointer.
 * @param bufferLen          The buffer length.
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success          Receive request successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy               There is no idle transfer.
 * @retval kStatus_USB_Error              pipe is not initialized.
 *                                                       Or, send transfer fail, please reference to USB_HostRecv.
 */
usb_status_t USB_HostAudioStreamRecv(usb_host_class_handle classHandle,
                                     uint8_t *buffer,
                                     uint32_t bufferLen,
                                     transfer_callback_t callbackFn,
                                     void *callbackParam)
{
    audio_instance_t *audioPtr = (audio_instance_t *)classHandle;
    usb_host_transfer_t *transfer;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (audioPtr->isoInPipe == NULL)
    {
        return kStatus_USB_Error;
    }

    if (USB_HostMallocTransfer(audioPtr->hostHandle, &transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("error to get transfer\r\n");
#endif
        return kStatus_USB_Error;
    }
    audioPtr->inCallbackFn    = callbackFn;
    audioPtr->inCallbackParam = callbackParam;
    transfer->transferBuffer  = buffer;
    transfer->transferLength  = bufferLen;
    transfer->callbackFn      = _USB_HostAudioStreamIsoInPipeCallback;
    transfer->callbackParam   = audioPtr;

    if (USB_HostRecv(audioPtr->hostHandle, audioPtr->isoInPipe, transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("failed to USB_HostRecv\r\n");
#endif
        (void)USB_HostFreeTransfer(audioPtr->hostHandle, transfer);
        return kStatus_USB_Error;
    }

    return kStatus_USB_Success;
}

/*!
 * @brief audio stream send data.
 *
 * This function implements audio sending data.
 *
 * @param classHandle      The class handle.
 * @param buffer                The buffer pointer.
 * @param bufferLen          The buffer length.
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success          Receive request successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy               There is no idle transfer.
 * @retval kStatus_USB_Error              pipe is not initialized.
 *                                                       Or, send transfer fail, please reference to USB_HostSend.
 */
usb_status_t USB_HostAudioStreamSend(usb_host_class_handle classHandle,
                                     uint8_t *buffer,
                                     uint32_t bufferLen,
                                     transfer_callback_t callbackFn,
                                     void *callbackParam)
{
    audio_instance_t *audioPtr = (audio_instance_t *)classHandle;
    usb_host_transfer_t *transfer;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (audioPtr->isoOutPipe == NULL)
    {
        return kStatus_USB_Error;
    }

    if (USB_HostMallocTransfer(audioPtr->hostHandle, &transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("error to get transfer\r\n");
#endif
        return kStatus_USB_Error;
    }
    audioPtr->outCallbackFn    = callbackFn;
    audioPtr->outCallbackParam = callbackParam;
    transfer->transferBuffer   = buffer;
    transfer->transferLength   = bufferLen;
    transfer->callbackFn       = _USB_HostAudioStreamIsoOutPipeCallback;
    transfer->callbackParam    = audioPtr;

    if (USB_HostSend(audioPtr->hostHandle, audioPtr->isoOutPipe, transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("failed to USB_HostSend\r\n");
#endif
        (void)USB_HostFreeTransfer(audioPtr->hostHandle, transfer);
        return kStatus_USB_Error;
    }

    return kStatus_USB_Success;
}
/*!
 * @brief get audio control current altsetting descriptor.
 *
 * This function implements get audio stream current altsetting descriptor.
 *
 * @param classHandle               The class handle.
 * @param DescriptorType            The descriptor type.
 * @param DescriptorSubType         The descriptor subtype, 0 for no subtype.
 * @param Descriptor                The pointer of descriptor pointer.
 *
 * @retval kStatus_USB_Success          Get audio stream current altsetting descriptor request successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 *
 */
usb_status_t USB_HostAudioControlGetCurrentAltsettingSpecificDescriptors(

    usb_host_class_handle classHandle, uint32_t DescriptorType, uint32_t DescriptorSubType, void **Descriptor)
{
    audio_instance_t *audioPtr   = (audio_instance_t *)classHandle;
    usb_status_t status          = kStatus_USB_Error;
    uint32_t **descriptorPointer = (uint32_t **)Descriptor;
    if ((classHandle == NULL) || NULL == Descriptor)
    {
        return kStatus_USB_InvalidParameter;
    }

    if (USB_AUDIO_DESCRIPTOR_TYPE_CS_INTERFACE == DescriptorType)
    {
        if (AUDIO_DEVICE_VERSION_01 == audioPtr->deviceAudioVersion)
        {
            if (USB_AUDIO_DESC_SUBTYPE_CS_EXTENSION <= DescriptorSubType)
            {
                /*descriptor type code bigger than EXTENSION is not support by audio 1.0*/
                return kStatus_USB_InvalidParameter;
            }
        }
        switch (DescriptorSubType)
        {
            case USB_AUDIO_DESC_SUBTYPE_CS_HEADER:
                *descriptorPointer = (uint32_t *)audioPtr->headerDesc;
                status             = kStatus_USB_Success;
                break;
            case USB_AUDIO_DESC_SUBTYPE_CS_INPUT:
                *descriptorPointer = (uint32_t *)audioPtr->itDesc;
                status             = kStatus_USB_Success;
                break;
            case USB_AUDIO_DESC_SUBTYPE_CS_OUTPUT:
                *descriptorPointer = (uint32_t *)audioPtr->otDesc;
                status             = kStatus_USB_Success;
                break;
            case USB_AUDIO_DESC_SUBTYPE_CS_FEATURE:
                *descriptorPointer = (uint32_t *)audioPtr->fuDesc;
                status             = kStatus_USB_Success;
                break;
            default:
                /*no action*/
                break;
        }
    }

    return status;
}
/*!
 * @brief get audio control current altsetting descriptor.
 *
 * This function implements get audio stream current altsetting descriptor.
 *
 * @param classHandle               The class handle.
 * @param DescriptorType            The descriptor type.
 * @param DescriptorSubType         The descriptor subtype, default 0.
 * @param Descriptor                The pointer of descriptor pointer.
 *
 * @retval kStatus_USB_Success          Get audio stream current altsetting descriptor request successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 *
 */
usb_status_t USB_HostAudioStreamGetCurrentAltsettingSpecificDescriptors(

    usb_host_class_handle classHandle, uint32_t DescriptorType, uint32_t DescriptorSubType, void **Descriptor)
{
    audio_instance_t *audioPtr   = (audio_instance_t *)classHandle;
    usb_status_t status          = kStatus_USB_Error;
    uint32_t **descriptorPointer = (uint32_t **)Descriptor;
    void *temp;

    if ((classHandle == NULL) || NULL == Descriptor)
    {
        return kStatus_USB_InvalidParameter;
    }

    if (USB_AUDIO_DESCRIPTOR_TYPE_CS_INTERFACE == DescriptorType)
    {
        if (AUDIO_DEVICE_VERSION_01 == audioPtr->deviceAudioVersion)
        {
            switch (DescriptorSubType)
            {
                case USB_AUDIO_DESC_SUBTYPE_AS_GENERAL:
                    *descriptorPointer = (uint32_t *)audioPtr->asIntfDesc;
                    status             = kStatus_USB_Success;
                    break;
                case USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_TYPE:
                    *descriptorPointer = (uint32_t *)audioPtr->formatTypeDesc;
                    status             = kStatus_USB_Success;
                    break;
                default:
                    /*no action*/
                    break;
            }
        }
        else if (AUDIO_DEVICE_VERSION_02 == audioPtr->deviceAudioVersion)
        {
            switch (DescriptorSubType)
            {
                case USB_AUDIO_DESC_SUBTYPE_AS_GENERAL_20:
                    *descriptorPointer = (uint32_t *)audioPtr->asIntfDesc;
                    status             = kStatus_USB_Success;
                    break;
                case USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_TYPE_20:
                    *descriptorPointer = (uint32_t *)audioPtr->formatTypeDesc;
                    status             = kStatus_USB_Success;
                    break;
                case USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_ENCODER_20:
                    /*no action*/
                    break;
                default:
                    /*no action*/
                    break;
            }
        }
        else
        {
            /*To DO*/
        }
    }
    else if (USB_DESCRIPTOR_TYPE_ENDPOINT == DescriptorType)
    {
        if (0x0U == DescriptorSubType)
        {
            /*iso data endpoint descriptor*/
            temp               = (void *)audioPtr->isoEndpDesc;
            *descriptorPointer = (uint32_t *)temp;
            status             = kStatus_USB_Success;
        }
        else
        {
            /*TO DO*/
        }
    }
    else
    {
        /*To DO*/
    }

    return status;
}

/*!
 * @brief get audio stream current altsetting descriptor.
 * @deprecated Do not use this function. It has been superceded by @ref
 * USB_HostAudioStreamGetCurrentAltsettingSpecificDescriptors.
 *
 * This function implements get audio stream current altsetting descriptor.
 *
 * @param classHandle               The class handle.
 * @param asIntfDesc                 The pointer of class specific AS interface descriptor.
 * @param formatTypeDesc       The pointer of format type descriptor.
 * @param isoEndpDesc    The pointer of specific iso endp descriptor.
 *
 * @retval kStatus_USB_Success          Get audio stream current altsetting descriptor request successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 *
 */
usb_status_t USB_HostAudioStreamGetCurrentAltsettingDescriptors(

    usb_host_class_handle classHandle,
    usb_audio_stream_spepific_as_intf_desc_t **asIntfDesc,
    usb_audio_stream_format_type_desc_t **formatTypeDesc,
    usb_descriptor_endpoint_t **isoEndpDesc)
{
    audio_instance_t *audioPtr = (audio_instance_t *)classHandle;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }
    *asIntfDesc     = (usb_audio_stream_spepific_as_intf_desc_t *)audioPtr->asIntfDesc;
    *formatTypeDesc = (usb_audio_stream_format_type_desc_t *)audioPtr->formatTypeDesc;
    *isoEndpDesc    = (usb_descriptor_endpoint_t *)audioPtr->isoEndpDesc;

    return kStatus_USB_Success;
}
/*!
 * @brief usb audio set/get feature unit request.
 *
 * This function implements usb audio feature unit request.
 *
 * @param classHandle      The class handle.
 * @param csAndCn          The CS and CN or MCN for wValue field in setup Request.
 * @param cmdCode          The bRequest code in lower 8bit of lower word and get(1)/set(0) flag in higher 8bit of lower
 * word.
 * @param buf              The feature unit request buffer pointer.
 * @param callbackFn       This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success            Feature unit request successfully.
 * @retval kStatus_USB_InvalidHandle    The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy                There is no idle transfer.
 * @retval kStatus_USB_Error                Send transfer fail, please reference to USB_HostSendSetup.
 *
 */
usb_status_t USB_HostAudioGetSetFeatureUnitRequest(usb_host_class_handle classHandle,
                                                   uint32_t csAndCn,
                                                   uint32_t cmdCode,
                                                   void *buf,
                                                   uint32_t bufLen,
                                                   transfer_callback_t callbackFn,
                                                   void *callbackParam)
{
    audio_instance_t *if_ptr;
    uint8_t *bmacontrols = NULL;
    usb_audio_ctrl_common_fu_desc *fuDesc;
    uint16_t windex;
    uint8_t request;
    uint8_t atribute_index;
    uint16_t length = 0U;
    uint8_t typeRequest;
    uint8_t controlSelector;

    usb_status_t status = kStatus_USB_Error;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if_ptr = (audio_instance_t *)classHandle;

    /* Check whether this attribute valid or not */
    if (if_ptr->fuDesc == NULL)
    {
        return kStatus_USB_Error;
    }
    fuDesc = (usb_audio_ctrl_common_fu_desc *)if_ptr->fuDesc;
    if (AUDIO_DEVICE_VERSION_01 == if_ptr->deviceAudioVersion)
    {
        windex         = (uint16_t)((uint16_t)((uint16_t)(fuDesc->fuDesc10.bunitid) << 8U) | (if_ptr->streamIfnum));
        atribute_index = (uint8_t)(fuDesc->fuDesc10.bcontrolsize * ((uint16_t)csAndCn & 0xFFU));
        /*Size of this descriptor, in bytes: 7+(ch+1)*n*/
        if (atribute_index < (fuDesc->fuDesc10.blength - 7U))
        {
            bmacontrols = &(fuDesc->fuDesc10.bcontrolsize);
        }
    }
    else if (AUDIO_DEVICE_VERSION_02 == if_ptr->deviceAudioVersion)
    {
        windex = (uint16_t)((uint16_t)((uint16_t)(fuDesc->fuDesc20.bunitid) << 8U) | (if_ptr->streamIfnum));
        /*BmControl is 4byte, refer to *audio2.0 4.7.2.8 Size of this descriptor, in bytes: 6+(ch+1)*4*/
        atribute_index = (uint8_t)(4U * ((uint16_t)csAndCn & 0xFFU));

        if (atribute_index < (fuDesc->fuDesc20.blength - 6U))
        {
            bmacontrols = &(fuDesc->fuDesc20.bmaControls0[0]);
        }
    }
    else
    {
        /*TO DO*/
    }
    if (bmacontrols == NULL)
    {
        return kStatus_USB_Error;
    }
    request = (uint8_t)(((uint16_t)cmdCode & 0xFFU));

    controlSelector = (uint8_t)((uint16_t)(csAndCn >> 8) & 0xFFU);
    if (AUDIO_DEVICE_VERSION_01 == if_ptr->deviceAudioVersion)
    {
        /*control selector */
        switch (controlSelector)
        {
            /*note ,all the length is defined in audio1.0 Spec chapter 5.2.2.4.3*/
            case AUDIO_FU_MUTE:
                status = kStatus_USB_Success;
                length = 1U;
                break;
            case AUDIO_FU_VOLUME:
                status = kStatus_USB_Success;
                length = 2U;
                break;
            case AUDIO_FU_BASS:
                /*TO DO*/
                break;
            default:
                /*no action*/
                break;
        }
    }
    else if (AUDIO_DEVICE_VERSION_02 == if_ptr->deviceAudioVersion)
    {
        if ((request != USB_AUDIO_CS_REQUEST_CODE_RANGE_20) && (request != USB_AUDIO_CS_REQUEST_CODE_CUR_20))
        {
            return kStatus_USB_Error;
        }
        switch (controlSelector)
        {
            /*note ,all the length is defined in audio2.0 Spec chapter 5.2.5.7*/
            case AUDIO_FU_MUTE:

                if (bufLen != 1U)
                {
                    return kStatus_USB_Error;
                }
                status = kStatus_USB_Success;
                length = 1U;
                break;
            case AUDIO_FU_VOLUME:
                status = kStatus_USB_Success;

                if (USB_AUDIO_CS_REQUEST_CODE_RANGE_20 == request)
                {
                    /*the len for volume is 2+6*n*/
                    if (bufLen < 8U)
                    {
                        return kStatus_USB_Error;
                    }
                }
                else if (USB_AUDIO_CS_REQUEST_CODE_CUR_20 == request)
                {
                    if (bufLen != 2U)
                    {
                        return kStatus_USB_Error;
                    }
                }
                else
                {
                    /*TO DO*/
                }
                length = (uint16_t)bufLen;

                break;
            case AUDIO_FU_BASS:
                /*TO DO*/
                break;
            default:
                /*no action*/
                break;
        }
    }
    else
    {
        /*TO DO*/
    }

    if (status != kStatus_USB_Success)
    {
        /*the request is not support*/
        return kStatus_USB_Error;
    }
    if (0U == ((uint16_t)(cmdCode >> 8U) & 0xFFU))
    {
        /*refer to audio Spec 2.0 chapter 5.2.2*/
        typeRequest = 0x21U;
    }
    else
    {
        typeRequest = 0xA1U;
    }

    status = _USB_HostAudioControl(classHandle, typeRequest, request, (uint16_t)csAndCn, windex, length, (uint8_t *)buf,
                                   callbackFn, callbackParam);

    return status;
}
/*!
 * @brief usb audio set/get feature unit request.
 *
 * This function implements usb audio feature unit request.
 *
 * @param classHandle      The class handle.
 * @param csAndCn          The CS and CN or MCN for wValue field in setup Request.
 * @param cmdCode          The bRequest code in lower 8bit of lower word and get(1)/set(0) flag in higher 8bit of lower
 word.
 * @param buf              The feature unit request buffer pointer.

 * @param callbackFn       This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success            Feature unit request successfully.
 * @retval kStatus_USB_InvalidHandle    The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy                There is no idle transfer.
 * @retval kStatus_USB_Error                Send transfer fail, please reference to USB_HostSendSetup.
 *
 */
usb_status_t USB_HostAudioGetSetClockSourceRequest(usb_host_class_handle classHandle,
                                                   uint32_t csAndCn,
                                                   uint32_t cmdCode,
                                                   void *buf,
                                                   uint32_t bufLen,
                                                   transfer_callback_t callbackFn,
                                                   void *callbackParam)
{
    audio_instance_t *if_ptr;
    usb_audio_2_0_ctrl_clock_source_desc_t *clockDesc;
    uint16_t windex;
    uint16_t controlSelector;
    uint8_t length = 0U;
    uint8_t typeRequest;
    uint8_t request;

    usb_status_t status = kStatus_USB_Error;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidParameter;
    }

    if_ptr = (audio_instance_t *)classHandle;

    if (AUDIO_DEVICE_VERSION_02 == if_ptr->deviceAudioVersion)
    {
        clockDesc = (usb_audio_2_0_ctrl_clock_source_desc_t *)if_ptr->clockSource;
        windex    = (uint16_t)((uint16_t)((uint16_t)(clockDesc->bClockID) << 8U) | (if_ptr->streamIfnum));
    }
    else
    {
        return kStatus_USB_Error;
    }
    request         = (uint8_t)(((uint16_t)cmdCode & 0xFFU));
    controlSelector = (uint8_t)((uint16_t)(csAndCn >> 8U) & 0xFFU);

    if (USB_AUDIO_CS_SAM_FREQ_CONTROL_20 == controlSelector)
    {
        /*note ,all the length is defined in audio2.0 Spec chapter 5.2.5.1.1*/

        if (USB_AUDIO_CS_REQUEST_CODE_RANGE_20 == request)
        {
            /*the len for volume is 2+6*n*/
            if (bufLen < 12U)
            {
                return kStatus_USB_Error;
            }
            else
            {
                length = (uint8_t)bufLen;
            }
        }
        else if (USB_AUDIO_CS_REQUEST_CODE_CUR_20 == request)
        {
            if (bufLen != 4U)
            {
                return kStatus_USB_Error;
            }
            length = (uint8_t)bufLen;
        }
        else
        {
            return kStatus_USB_Error;
        }
        status = kStatus_USB_Success;
    }
    else if (USB_AUDIO_CS_CLOCK_VALID_CONTROL_20 == controlSelector)
    {
        status = kStatus_USB_Success;
        if (bufLen < 1U)
        {
            return kStatus_USB_Error;
        }
        length = 1U;
    }
    else
    {
        /**TO DO*/
    }

    if (status != kStatus_USB_Success)
    {
        /*the request is not support*/
        return kStatus_USB_Error;
    }

    if (0U == ((uint16_t)(cmdCode >> 8U) & 0xFFU))
    {
        /*refer to audio Spec 2.0 chapter 5.2.2*/
        typeRequest = 0x21U;
    }
    else
    {
        typeRequest = 0xA1U;
    }

    status = _USB_HostAudioControl(classHandle, typeRequest, request, (uint16_t)csAndCn, windex, length, (uint8_t *)buf,
                                   callbackFn, callbackParam);

    return status;
}

/*!
 * @brief usb audio feature unit request.
 * @deprecated Do not use this function. It has been superceded by @ref USB_HostAudioGetSetFeatureUnitRequest.
 *
 * This function implements usb audio feature unit request.
 *
 * @param classHandle      The class handle.
 * @param channelNo        The channel number of audio feature unit.
 * @param buf              The feature unit request buffer pointer.
 * @param cmdCode          The feature unit command code, for example USB_AUDIO_GET_CUR_MUTE etc.
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success            Feature unit request successfully.
 * @retval kStatus_USB_InvalidHandle    The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy                There is no idle transfer.
 * @retval kStatus_USB_Error                Send transfer fail, please reference to USB_HostSendSetup.
 *
 */
usb_status_t USB_HostAudioFeatureUnitRequest(usb_host_class_handle classHandle,
                                             uint8_t channelNo,
                                             void *buf,
                                             uint32_t cmdCode,
                                             transfer_callback_t callbackFn,
                                             void *callbackParam)
{ /* Body */
    uint16_t windex;
    uint16_t request_value;
    audio_instance_t *if_ptr;
    usb_audio_request_t *p_feature_request;
    usb_audio_ctrl_fu_desc_t *fuDesc;
    uint8_t *bmacontrols = NULL;
    uint8_t atribute_index;
    usb_status_t status;

    if ((classHandle == NULL))
    {
        return kStatus_USB_InvalidHandle;
    }

    if_ptr = (audio_instance_t *)classHandle;
    if (AUDIO_DEVICE_VERSION_01 != if_ptr->deviceAudioVersion)
    {
        return kStatus_USB_Error;
    }
    /* pointer to command */
    p_feature_request = &(s_usbAudioFuRequests[(uint8_t)cmdCode & 0xfU]);
    /* get request value */
    request_value = (uint16_t)((uint16_t)((uint16_t)p_feature_request->requestValue << 8U) | channelNo);

    /* Check whether this attribute valid or not */
    if (if_ptr->fuDesc == NULL)
    {
        return kStatus_USB_Error;
    }
    fuDesc         = (usb_audio_ctrl_fu_desc_t *)if_ptr->fuDesc;
    windex         = (uint16_t)((uint16_t)((uint16_t)(fuDesc->bunitid) << 8U) | (if_ptr->streamIfnum));
    atribute_index = fuDesc->bcontrolsize * channelNo;

    if (atribute_index < (fuDesc->blength - 7U))
    {
        bmacontrols = &(fuDesc->bcontrolsize);
    }

    if (bmacontrols == NULL)
    {
        return kStatus_USB_Error;
    }

    status = _USB_HostAudioControl(classHandle, (p_feature_request->typeRequest | ((uint8_t)cmdCode & 0x80U)),
                                   (p_feature_request->codeRequest | ((uint8_t)cmdCode & 0x80U)), request_value, windex,
                                   p_feature_request->length, (uint8_t *)buf, callbackFn, callbackParam);

    return status;
}
/*!
 * @brief usb audio set/get endp unit request.
 *
 * This function implements usb audio feature unit request.
 *
 * @param classHandle      The class handle.
 * @param csAndCn          The CS for wValue field in setup Request.
 * @param cmdCode          The bRequest code in lower 8bit of lower word and get(1)/set(0) flag in higher 8bit of lower
 word.
 * @param buf              The feature unit request buffer pointer.

 * @param callbackFn       This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success            Feature unit request successfully.
 * @retval kStatus_USB_InvalidHandle    The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy                There is no idle transfer.
 * @retval kStatus_USB_Error                Send transfer fail, please reference to USB_HostSendSetup.
 *
 */
usb_status_t USB_HostAudioGetSetEndpointRequest(usb_host_class_handle classHandle,
                                                uint32_t csAndCn,
                                                uint32_t cmdCode,
                                                void *buf,
                                                uint32_t bufLen,
                                                transfer_callback_t callbackFn,
                                                void *callbackParam)
{
    audio_instance_t *if_ptr;
    uint16_t windex;
    uint8_t request;
    uint8_t length = 0U;
    uint8_t typeRequest;
    uint8_t controlSelector;

    usb_status_t status = kStatus_USB_Error;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if_ptr = (audio_instance_t *)classHandle;

    if (AUDIO_DEVICE_VERSION_01 == if_ptr->deviceAudioVersion)
    {
        if ((NULL == if_ptr->isoInPipe) && (NULL == if_ptr->isoOutPipe))
        {
            return kStatus_USB_InvalidParameter;
        }
        else if (NULL != if_ptr->isoInPipe)
        {
            windex = (((uint16_t)if_ptr->isoEpNum) | 0x80U);
        }
        else
        {
            windex = if_ptr->isoEpNum;
        }
    }
    else if (AUDIO_DEVICE_VERSION_02 == if_ptr->deviceAudioVersion)
    {
        /*TO DO*/
    }
    else
    {
        /*TO DO*/
    }

    request = (uint8_t)(((uint16_t)cmdCode & 0xFFU));

    controlSelector = (uint8_t)((uint16_t)(csAndCn >> 8U) & 0xFFU);
    if (AUDIO_DEVICE_VERSION_01 == if_ptr->deviceAudioVersion)
    {
        /*control selector */
        /*note ,all the length is defined in audio1.0 Spec chapter 5.2.3.2.3*/
        if (USB_AUDIO_EP_CS_SAMPING_FREQ_CONTROL == controlSelector)
        {
            if (bufLen > 3U)
            {
                /*fixed length*/
                length = 3U;
                status = kStatus_USB_Success;
            }
            else
            {
                /*TO DO*/
            }
        }
        else if (USB_AUDIO_EP_CS_PINTCH_CONTROL == controlSelector)
        {
            /*TO DO*/
        }
        else
        {
            /*TO DO*/
        }
    }
    else if (AUDIO_DEVICE_VERSION_02 == if_ptr->deviceAudioVersion)
    {
        switch (controlSelector)
        {
            /*note ,all the length is defined in audio2.0 Spec chapter 5.2.6*/
            case USB_AUDIO_EP_CS_PINTCH_CONTROL_20:
                /*TO DO*/
                break;
            case USB_AUDIO_EP_CS_OVERRUN_CONTROL_20:
                /*TO DO*/
                break;
            case USB_AUDIO_EP_CS_UNDERRUN_CONTROL_20:
                /*TO DO*/
                break;
            default:
                /*no action*/
                break;
        }
    }
    else
    {
        /*TO DO*/
    }

    if (status != kStatus_USB_Success)
    {
        /*the request is not support*/
        return kStatus_USB_Error;
    }
    if (0U == ((uint16_t)(cmdCode >> 8U) & 0xFFU))
    {
        /*refer to audio Spec 2.0 chapter 5.2.2*/
        typeRequest = 0x22U;
    }
    else
    {
        typeRequest = 0xA2U;
    }

    status = _USB_HostAudioControl(classHandle, typeRequest, request, (uint16_t)csAndCn, windex, length, (uint8_t *)buf,
                                   callbackFn, callbackParam);

    return status;
}
/*!
 * @brief usb audio endpoint request.
 * @deprecated Do not use this function. It has been superceded by @ref USB_HostAudioGetSetEndpointRequest.
 *
 * This function implements usb audio endpoint request.
 *
 * @param classHandle      The class handle.
 * @param buf                    The feature unit buffer pointer.
 * @param cmdCode          The feature unit command code, for example USB_AUDIO_GET_CUR_PITCH etc .
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam   The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success          Endpoint request successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy               There is no idle transfer.
 * @retval kStatus_USB_Error               Send transfer fail, please reference to USB_HostSendSetup.
 *
 */
usb_status_t USB_HostAudioEndpointRequest(
    usb_host_class_handle classHandle, void *buf, uint32_t cmdCode, transfer_callback_t callbackFn, void *callbackParam)
{
    uint8_t endp_num;
    usb_status_t status;
    uint16_t request_value;
    usb_audio_request_t *p_endpoint_request;
    audio_instance_t *audioPtr;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    audioPtr = (audio_instance_t *)classHandle;
    if (AUDIO_DEVICE_VERSION_01 != audioPtr->deviceAudioVersion)
    {
        return kStatus_USB_Error;
    }
    /* pointer to command */
    p_endpoint_request = &(s_usbAudioEpRequests[(uint8_t)cmdCode & 0xfU]);
    /* get request value */
    request_value = (uint16_t)((uint16_t)((uint16_t)p_endpoint_request->requestValue << 8U));

    /* Check whether this attribute valid or not */
    if (audioPtr->isoEndpDesc == NULL)
    {
        return kStatus_USB_Error;
    }
    if ((0U != audioPtr->isoEndpDesc->bmAttributes) && (0U != p_endpoint_request->controlMask))
    {
        status = kStatus_USB_Success;
    }
    else
    {
        status = kStatus_USB_InvalidRequest;
    }

    if (kStatus_USB_Success == status)
    {
        /* Any isochronous pipe is supported? */
        if ((NULL == audioPtr->isoInPipe) && (NULL == audioPtr->isoOutPipe))
        {
            return kStatus_USB_InvalidParameter;
        }
        else if (NULL != audioPtr->isoInPipe)
        {
            endp_num = (audioPtr->isoEpNum | 0x80U);
        }
        else
        {
            endp_num = audioPtr->isoEpNum;
        } /* Endif */

        status = _USB_HostAudioControl(classHandle, (p_endpoint_request->typeRequest | ((uint8_t)cmdCode & 0x80U)),
                                       (p_endpoint_request->codeRequest | ((uint8_t)cmdCode & 0x80U)), request_value,
                                       endp_num, p_endpoint_request->length, (uint8_t *)buf, callbackFn, callbackParam);
    }

    return status;
}

usb_status_t USB_HostAudioSetStreamOutDataInterval(usb_host_class_handle classHandle, uint8_t intervalValue)
{
    audio_instance_t *audioPtr         = (audio_instance_t *)classHandle;
    usb_status_t status                = kStatus_USB_Error;
    uint8_t ep_index                   = 0U;
    usb_descriptor_endpoint_t *ep_desc = NULL;
    usb_host_interface_t *interface_ptr;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (audioPtr->isoOutPipe != NULL)
    {
        (void)USB_HostClosePipe(audioPtr->hostHandle, audioPtr->isoOutPipe);
        audioPtr->isoOutPipe = NULL;
    }

    /* open interface pipes */
    interface_ptr = (usb_host_interface_t *)audioPtr->streamIntfHandle;
    for (ep_index = 0U; ep_index < interface_ptr->epCount; ++ep_index)
    {
        ep_desc = interface_ptr->epList[ep_index].epDesc;
        if (((ep_desc->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
             USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
            ((ep_desc->bmAttributes & USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_TYPE_MASK) == USB_ENDPOINT_ISOCHRONOUS))
        {
            uint8_t intervalBackUp = ep_desc->bInterval;
            ep_desc->bInterval     = intervalValue;
            status                 = _USB_HostAudioInitEndpoint(audioPtr, ep_desc);
            ep_desc->bInterval     = intervalBackUp;
        }
    }

    return status;
}
#endif /* USB_HOST_CONFIG_AUDIO */
