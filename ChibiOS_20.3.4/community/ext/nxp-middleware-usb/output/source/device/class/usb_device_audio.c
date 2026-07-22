/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"

#if ((defined(USB_DEVICE_CONFIG_AUDIO)) && (USB_DEVICE_CONFIG_AUDIO > 0U))
#include "usb_device_audio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static usb_status_t USB_DeviceAudioAllocateHandle(usb_device_audio_struct_t **handle);
static usb_status_t USB_DeviceAudioFreeHandle(usb_device_audio_struct_t *handle);
usb_status_t USB_DeviceAudioInterruptIn(usb_device_handle handle,
                                        usb_device_endpoint_callback_message_struct_t *message,
                                        void *callbackParam);
usb_status_t USB_DeviceAudioIsochronousIn(usb_device_handle handle,
                                          usb_device_endpoint_callback_message_struct_t *message,
                                          void *callbackParam);
usb_status_t USB_DeviceAudioIsochronousOut(usb_device_handle handle,
                                           usb_device_endpoint_callback_message_struct_t *message,
                                           void *callbackParam);
usb_status_t USB_DeviceAudioStreamEndpointsInit(usb_device_audio_struct_t *audioHandle);
usb_status_t USB_DeviceAudioStreamEndpointsDeinit(usb_device_audio_struct_t *audioHandle);
usb_status_t USB_DeviceAudioControlEndpointsInit(usb_device_audio_struct_t *audioHandle);
usb_status_t USB_DeviceAudioControlEndpointsDeinit(usb_device_audio_struct_t *audioHandle);
usb_status_t USB_DeviceAudioGetCurAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioSetCurAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
usb_status_t USB_DeviceAudioGetRangeAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioGetClockSource(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioSetClockSource(usb_device_audio_struct_t *audioHandle,
                                           usb_device_control_request_struct_t *controlRequest);
#else
usb_status_t USB_DeviceAudioGetMinAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioGetMaxAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioGetResAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioSetMinAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioSetMaxAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioSetResAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest);
#endif
usb_status_t USB_DeviceAudioSetFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                           usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioSetRequestEndpoint(usb_device_audio_struct_t *audioHandle,
                                               usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioGetRequestEndpoint(usb_device_audio_struct_t *audioHandle,
                                               usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioSetControlTerminal(usb_device_audio_struct_t *audioHandle,
                                               usb_device_control_request_struct_t *controlRequest, uint8_t terminal_type);
usb_status_t USB_DeviceAudioGetControlTerminal(usb_device_audio_struct_t *audioHandle,
                                               usb_device_control_request_struct_t *controlRequest, uint8_t terminal_type);
usb_status_t USB_DeviceAudioGetFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                           usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioSetRequestInterface(usb_device_audio_struct_t *audioHandle,
                                                usb_device_control_request_struct_t *controlRequest);
usb_status_t USB_DeviceAudioGetRequestInterface(usb_device_audio_struct_t *audioHandle,
                                                usb_device_control_request_struct_t *controlRequest);

/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_audio_struct_t
    s_UsbDeviceAudioHandle[USB_DEVICE_CONFIG_AUDIO];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Allocate a device audio class handle.
 *
 * This function allocates a device audio class handle.
 *
 * @param handle          It is out parameter, is used to return pointer of the device audio class handle to the caller.
 *
 * @retval kStatus_USB_Success              Get a device audio class handle successfully.
 * @retval kStatus_USB_Busy                 Cannot allocate a device audio class handle.
 */
static usb_status_t USB_DeviceAudioAllocateHandle(usb_device_audio_struct_t **handle)
{
    uint32_t count;
    OSA_SR_ALLOC();

    OSA_ENTER_CRITICAL();
    for (count = 0; count < USB_DEVICE_CONFIG_AUDIO; count++)
    {
        if (NULL == s_UsbDeviceAudioHandle[count].handle)
        {
            *handle = &s_UsbDeviceAudioHandle[count];
            OSA_EXIT_CRITICAL();
            return kStatus_USB_Success;
        }
    }
    OSA_EXIT_CRITICAL();
    return kStatus_USB_Busy;
}

/*!
 * @brief Free a device audio class handle.
 *
 * This function frees a device audio class handle.
 *
 * @param handle          The device audio class handle.
 *
 * @retval kStatus_USB_Success              Free device audio class handle successfully.
 */
static usb_status_t USB_DeviceAudioFreeHandle(usb_device_audio_struct_t *handle)
{
    OSA_SR_ALLOC();

    OSA_ENTER_CRITICAL();
    handle->handle           = NULL;
    handle->configStruct     = (usb_device_class_config_struct_t *)NULL;
    handle->configuration    = 0U;
    handle->controlAlternate = 0U;
    handle->streamAlternate  = 0U;
    OSA_EXIT_CRITICAL();
    return kStatus_USB_Success;
}

/*!
 * @brief Interrupt IN endpoint callback function.
 *
 * This callback function is used to notify uplayer the transfser result of a transfer.
 * This callback pointer is passed when the interrupt IN pipe initialized.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param message         The result of the interrupt IN pipe transfer.
 * @param callbackParam  The parameter for this callback. It is same with
 * usb_device_endpoint_callback_struct_t::callbackParam. In the class, the value is the audio class handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioInterruptIn(usb_device_handle handle,
                                        usb_device_endpoint_callback_message_struct_t *message,
                                        void *callbackParam)
{
    usb_device_audio_struct_t *audioHandle;
    usb_status_t error = kStatus_USB_Error;

    /* Get the audio class handle */
    audioHandle = (usb_device_audio_struct_t *)callbackParam;

    if (NULL == audioHandle)
    {
        return kStatus_USB_InvalidHandle;
    }

    if ((NULL != audioHandle->configStruct) && (NULL != audioHandle->configStruct->classCallback))
    {
        /* Notify the application control data sent by calling the audio class callback.
        classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        error = audioHandle->configStruct->classCallback((class_handle_t)audioHandle,
                                                         kUSB_DeviceAudioEventControlSendResponse, message);
    }

    return error;
}

/*!
 * @brief ISO IN endpoint callback function.
 *
 * This callback function is used to notify uplayer the transfser result of a transfer.
 * This callback pointer is passed when the ISO IN pipe initialized.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param message         The result of the ISO IN pipe transfer.
 * @param callbackParam  The parameter for this callback. It is same with
 * usb_device_endpoint_callback_struct_t::callbackParam. In the class, the value is the audio class handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioIsochronousIn(usb_device_handle handle,
                                          usb_device_endpoint_callback_message_struct_t *message,
                                          void *callbackParam)
{
    usb_device_audio_struct_t *audioHandle;
    usb_status_t status = kStatus_USB_Error;

    /* Get the audio class handle */
    audioHandle = (usb_device_audio_struct_t *)callbackParam;
    if (NULL == audioHandle)
    {
        return kStatus_USB_InvalidHandle;
    }
    audioHandle->streamInPipeBusy = 0U;
    if ((NULL != audioHandle->configStruct) && (NULL != audioHandle->configStruct->classCallback))
    {
        /* Notify the application stream data sent by calling the audio class callback.
        classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle,
                                                          kUSB_DeviceAudioEventStreamSendResponse, message);
    }

    return status;
}

/*!
 * @brief ISO OUT endpoint callback function.
 *
 * This callback function is used to notify uplayer the transfser result of a transfer.
 * This callback pointer is passed when the ISO OUT pipe initialized.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param message         The result of the ISO OUT pipe transfer.
 * @param callbackParam  The parameter for this callback. It is same with
 * usb_device_endpoint_callback_struct_t::callbackParam. In the class, the value is the audio class handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioIsochronousOut(usb_device_handle handle,
                                           usb_device_endpoint_callback_message_struct_t *message,
                                           void *callbackParam)
{
    usb_device_audio_struct_t *audioHandle;
    usb_status_t status = kStatus_USB_Error;

    /* Get the audio class handle */
    audioHandle = (usb_device_audio_struct_t *)callbackParam;
    if (NULL == audioHandle)
    {
        return kStatus_USB_InvalidHandle;
    }
    audioHandle->streamOutPipeBusy = 0U;

    if ((NULL != audioHandle->configStruct) && (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle,
                                                          kUSB_DeviceAudioEventStreamRecvResponse, message);
    }
    return status;
}

/*!
 * @brief Initialize the stream endpoints of the audio class.
 *
 * This callback function is used to initialize the stream endpoints of the audio class.
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioStreamEndpointsInit(usb_device_audio_struct_t *audioHandle)
{
    usb_device_interface_list_t *interfaceList;
    usb_device_interface_struct_t *interface = (usb_device_interface_struct_t *)NULL;
    usb_status_t status                      = kStatus_USB_Error;
    uint8_t count;
    uint8_t index;

    /* Check the configuration is valid or not. */
    if (0U == audioHandle->configuration)
    {
        return status;
    }

    /* Check the configuration is valid or not. */
    if (audioHandle->configuration > audioHandle->configStruct->classInfomation->configurations)
    {
        return status;
    }

    if (NULL == audioHandle->configStruct->classInfomation->interfaceList)
    {
        return status;
    }

    /* Get the interface list of the new configuration. */
    interfaceList = &audioHandle->configStruct->classInfomation->interfaceList[audioHandle->configuration - 1U];

    /* Find stream interface by using the alternate setting of the interface. */
    for (count = 0U; count < interfaceList->count; count++)
    {
        if ((USB_DEVICE_CONFIG_AUDIO_CLASS_CODE == interfaceList->interfaces[count].classCode) &&
            (USB_DEVICE_AUDIO_STREAM_SUBCLASS == interfaceList->interfaces[count].subclassCode))
        {
            for (index = 0; index < interfaceList->interfaces[count].count; index++)
            {
                if (interfaceList->interfaces[count].interface[index].alternateSetting == audioHandle->streamAlternate)
                {
                    interface = &interfaceList->interfaces[count].interface[index];
                    break;
                }
            }
            audioHandle->streamInterfaceNumber = interfaceList->interfaces[count].interfaceNumber;
            break;
        }
    }
    if (NULL == interface)
    {
        return status;
    }
    /* Keep new stream interface handle. */
    audioHandle->streamInterfaceHandle = interface;

    /* Initialize the endpoints of the new interface. */
    for (count = 0U; count < interface->endpointList.count; count++)
    {
        usb_device_endpoint_init_struct_t epInitStruct;
        usb_device_endpoint_callback_struct_t epCallback;
        epInitStruct.zlt             = 0U;
        epInitStruct.interval        = interface->endpointList.endpoint[count].interval;
        epInitStruct.endpointAddress = interface->endpointList.endpoint[count].endpointAddress;
        epInitStruct.maxPacketSize   = interface->endpointList.endpoint[count].maxPacketSize;
        epInitStruct.transferType    = interface->endpointList.endpoint[count].transferType;

        if ((USB_ENDPOINT_ISOCHRONOUS == (epInitStruct.transferType & USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_TYPE_MASK)) &&
            (USB_IN == ((epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                        USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT)))
        {
            epCallback.callbackFn = USB_DeviceAudioIsochronousIn;
        }
        else
        {
            epCallback.callbackFn = USB_DeviceAudioIsochronousOut;
        }
        epCallback.callbackParam = audioHandle;

        status = USB_DeviceInitEndpoint(audioHandle->handle, &epInitStruct, &epCallback);
    }
    return status;
}

/*!
 * @brief De-initialize the stream endpoints of the audio class.
 *
 * This callback function is used to de-initialize the stream endpoints of the audio class.
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioStreamEndpointsDeinit(usb_device_audio_struct_t *audioHandle)
{
    usb_status_t status = kStatus_USB_Error;
    usb_device_endpoint_callback_message_struct_t message;
    uint8_t count;

    if (NULL == audioHandle->streamInterfaceHandle)
    {
        return status;
    }
    /* De-initialize all stream endpoints of the interface */
    for (count = 0U; count < audioHandle->streamInterfaceHandle->endpointList.count; count++)
    {
        status = USB_DeviceDeinitEndpoint(
            audioHandle->handle, audioHandle->streamInterfaceHandle->endpointList.endpoint[count].endpointAddress);
    }

    for (count = 0U; count < audioHandle->streamInterfaceHandle->endpointList.count; count++)
    {
        if ((audioHandle->streamInterfaceHandle->endpointList.endpoint[count].endpointAddress &
             USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT ==
            USB_IN)
        {
            if (0U != audioHandle->streamInPipeBusy)
            {
                message.length = USB_CANCELLED_TRANSFER_LENGTH;
                (void)USB_DeviceAudioIsochronousIn(audioHandle->handle, &message, audioHandle);
            }
        }
        else
        {
            if (0U != audioHandle->streamOutPipeBusy)
            {
                message.length = USB_CANCELLED_TRANSFER_LENGTH;
                (void)USB_DeviceAudioIsochronousOut(audioHandle->handle, &message, audioHandle);
            }
        }
    }

    audioHandle->streamInterfaceHandle = NULL;
    return status;
}

/*!
 * @brief Initialize the control endpoints of the audio class.
 *
 * This callback function is used to initialize the control endpoints of the audio class.
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioControlEndpointsInit(usb_device_audio_struct_t *audioHandle)
{
    usb_device_interface_list_t *interfaceList;
    usb_device_interface_struct_t *interface = (usb_device_interface_struct_t *)NULL;
    usb_status_t status                      = kStatus_USB_Error;
    uint8_t count;
    uint8_t index;

    /* Check the configuration is valid or not. */
    if (0U == audioHandle->configuration)
    {
        return status;
    }

    /* Check the configuration is valid or not. */
    if (audioHandle->configuration > audioHandle->configStruct->classInfomation->configurations)
    {
        return status;
    }

    if (NULL == audioHandle->configStruct->classInfomation->interfaceList)
    {
        return status;
    }

    /* Get the control interface list of the new configuration. */
    interfaceList = &audioHandle->configStruct->classInfomation->interfaceList[audioHandle->configuration - 1U];

    /* Find control interface by using the alternate setting of the interface. */
    for (count = 0U; count < interfaceList->count; count++)
    {
        if ((USB_DEVICE_CONFIG_AUDIO_CLASS_CODE == interfaceList->interfaces[count].classCode) &&
            (USB_DEVICE_AUDIO_CONTROL_SUBCLASS == interfaceList->interfaces[count].subclassCode))
        {
            for (index = 0U; index < interfaceList->interfaces[count].count; index++)
            {
                if (interfaceList->interfaces[count].interface[index].alternateSetting == audioHandle->controlAlternate)
                {
                    interface = &interfaceList->interfaces[count].interface[index];
                    break;
                }
            }
            audioHandle->controlInterfaceNumber = interfaceList->interfaces[count].interfaceNumber;
            break;
        }
    }
    if (NULL == interface)
    {
        return status;
    }
    /* Keep new control interface handle. */
    audioHandle->controlInterfaceHandle = interface;

    /* Initialize the control endpoints of the new interface. */
    for (count = 0U; count < interface->endpointList.count; count++)
    {
        usb_device_endpoint_init_struct_t epInitStruct;
        usb_device_endpoint_callback_struct_t epCallback;
        epInitStruct.zlt             = 0U;
        epInitStruct.interval        = interface->endpointList.endpoint[count].interval;
        epInitStruct.endpointAddress = interface->endpointList.endpoint[count].endpointAddress;
        epInitStruct.maxPacketSize   = interface->endpointList.endpoint[count].maxPacketSize;
        epInitStruct.transferType    = interface->endpointList.endpoint[count].transferType;

        if ((USB_ENDPOINT_INTERRUPT == (epInitStruct.transferType & USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_TYPE_MASK)) &&
            (USB_IN == ((epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                        USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT)))
        {
            epCallback.callbackFn = USB_DeviceAudioInterruptIn;
        }
        else
        {
            continue;
        }
        epCallback.callbackParam = audioHandle;

        status = USB_DeviceInitEndpoint(audioHandle->handle, &epInitStruct, &epCallback);
    }
    return status;
}

/*!
 * @brief De-initialize the control endpoints of the audio class.
 *
 * This callback function is used to de-initialize the control endpoints of the audio class.
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioControlEndpointsDeinit(usb_device_audio_struct_t *audioHandle)
{
    usb_status_t status = kStatus_USB_Error;
    uint8_t count;

    if (NULL == audioHandle->controlInterfaceHandle)
    {
        return status;
    }
    /* De-initialize all control endpoints of the interface */
    for (count = 0U; count < audioHandle->controlInterfaceHandle->endpointList.count; count++)
    {
        status = USB_DeviceDeinitEndpoint(
            audioHandle->handle, audioHandle->controlInterfaceHandle->endpointList.endpoint[count].endpointAddress);
    }
    audioHandle->controlInterfaceHandle = NULL;
    return status;
}

/*!
 * @brief Handle the audio device GET CUR control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetCurAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0U;

    /* Select GET CUR request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_FU_MUTE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_MUTE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_AUTOMATIC_GAIN_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_AUTOMATIC_GAIN_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_DELAY_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_BOOST_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_BASS_BOOST_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_LOUDNESS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_CUR_LOUDNESS_CONTROL;
            break;
        default:
            /*no action*/
            break;
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }

    return status;
}

/*!
 * @brief Handle the audio device SET CUR control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioSetCurAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08U) & 0xFFU);
    uint32_t audioCommand   = 0;

    /* Select SET CUR request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_FU_MUTE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_MUTE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_AUTOMATIC_GAIN_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_AUTOMATIC_GAIN_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_DELAY_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_BOOST_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_BASS_BOOST_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_LOUDNESS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_CUR_LOUDNESS_CONTROL;
            break;
        default:
            /*no action*/
            break;
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }
    return status;
}

/*!
 * @brief Handle the audio device GET feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                           usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status = kStatus_USB_Error;

    /* Select GET request Control Feature Unit Module */
    switch (controlRequest->setup->bRequest)
    {
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
        case USB_DEVICE_AUDIO_CUR_REQUEST:
            status = USB_DeviceAudioGetCurAudioFeatureUnit(audioHandle, controlRequest);
            break;
        case USB_DEVICE_AUDIO_RANGE_REQUEST:
            status = USB_DeviceAudioGetRangeAudioFeatureUnit(audioHandle, controlRequest);
            break;
        default:
            /*no action*/
            break;
#else
        case USB_DEVICE_AUDIO_GET_CUR_REQUEST:
            status = USB_DeviceAudioGetCurAudioFeatureUnit(audioHandle, controlRequest);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_REQUEST:
            status = USB_DeviceAudioGetMinAudioFeatureUnit(audioHandle, controlRequest);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_REQUEST:
            status = USB_DeviceAudioGetMaxAudioFeatureUnit(audioHandle, controlRequest);
            break;
        case USB_DEVICE_AUDIO_GET_RES_REQUEST:
            status = USB_DeviceAudioGetResAudioFeatureUnit(audioHandle, controlRequest);
            break;
        default:
            /*no action*/
            break;
#endif
    }
    return status;
}

/*!
 * @brief Handle the audio device SET feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioSetFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                           usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status = kStatus_USB_Error;
    /* Select SET request Control Feature Unit Module */
    switch (controlRequest->setup->bRequest)
    {
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
        case USB_DEVICE_AUDIO_CUR_REQUEST:
            status = USB_DeviceAudioSetCurAudioFeatureUnit(audioHandle, controlRequest);
            break;
        case USB_DEVICE_AUDIO_RANGE_REQUEST:
            break;
        default:
            /*no action*/
            break;
#else
        case USB_DEVICE_AUDIO_SET_CUR_REQUEST:
            status = USB_DeviceAudioSetCurAudioFeatureUnit(audioHandle, controlRequest);
            break;
        case USB_DEVICE_AUDIO_SET_MIN_REQUEST:
            status = USB_DeviceAudioSetMinAudioFeatureUnit(audioHandle, controlRequest);
            break;
        case USB_DEVICE_AUDIO_SET_MAX_REQUEST:
            status = USB_DeviceAudioSetMaxAudioFeatureUnit(audioHandle, controlRequest);
            break;
        case USB_DEVICE_AUDIO_SET_RES_REQUEST:
            status = USB_DeviceAudioSetResAudioFeatureUnit(audioHandle, controlRequest);
            break;
        default:
            /*no action*/
            break;
#endif
    }
    return status;
}

#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
/*!
 * @brief Handle the audio device GET RANGE control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetRangeAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0U;

    /* Select GET RANGE request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_RANGE_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR:
            break;
        default:
            /*no action*/
            break;      
    }

    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }

    return status;
}

/*!
 * @brief Handle the audio device GET Clock Source control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetClockSource(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0;
    /* Select GET request Control Clock Source Module */
    switch (controlRequest->setup->bRequest)
    {
        case USB_DEVICE_AUDIO_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_CS_SAM_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_CS_GET_CUR_SAMPLING_FREQ_CONTROL;
                    break;
                case USB_DEVICE_AUDIO_CS_CLOCK_VALID_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_CS_GET_CUR_CLOCK_VALID_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_RANGE_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_CS_SAM_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_CS_GET_RANGE_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        default:
            /*no action*/
            break;
    }
    
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }
    return status;
}

/*!
 * @brief Handle the audio device SET Clock Source control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioSetClockSource(usb_device_audio_struct_t *audioHandle,
                                           usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0;
    /* Select SET request Control Clock Source Module */
    switch (controlRequest->setup->bRequest)
    {
        case USB_DEVICE_AUDIO_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_CS_SAM_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_CS_SET_CUR_SAMPLING_FREQ_CONTROL;
                    break;
                case USB_DEVICE_AUDIO_CS_CLOCK_VALID_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_CS_SET_CUR_CLOCK_VALID_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_RANGE_REQUEST:
            break;
        default:
            /*no action*/
            break;        
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
    (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }
    return status;
}

#else

/*!
 * @brief Handle the audio device set minimum volume/control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioSetMinAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0U;
    
    /* Select SET MIN request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MIN_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MIN_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MIN_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MIN_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MIN_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MIN_DELAY_CONTROL;
            break;
        default:
            /*no action*/
            break;
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }

    return status;
}

/*!
 * @brief Handle the audio device maximum volume/control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioSetMaxAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0;
    
    /* Select SET MAX request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MAX_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MAX_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MAX_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MAX_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MAX_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_MAX_DELAY_CONTROL;
            break;
        default:
            /*no action*/
            break;
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }

    return status;
}

/*!
 * @brief Handle the audio device set res volume/control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioSetResAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0;
 
    /* Select SET RES request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_RES_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_RES_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_RES_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_RES_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_RES_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_SET_RES_DELAY_CONTROL;
            break;
        default:
            /*no action*/
            break;
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }
    return status;
}

/*!
 * @brief Handle the audio device GET MIN control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetMinAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0U;

    /* Select GET MIN request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MIN_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MIN_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MIN_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MIN_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MIN_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MIN_DELAY_CONTROL;
            break;
        default:
            /* no action*/
            break;
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }
    return status;
}

/*!
 * @brief Handle the audio device GET MAX control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetMaxAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0U;

    /* Select GET MAX request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MAX_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MAX_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MAX_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MAX_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MAX_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_MAX_DELAY_CONTROL;
            break;
        default:
            /*no action*/
            break;
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }
    return status;
}

/*!
 * @brief Handle the audio device GET RES control feature unit request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetResAudioFeatureUnit(usb_device_audio_struct_t *audioHandle,
                                                   usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0U;

    /* Select GET RES request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_RES_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_RES_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_RES_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_RES_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_RES_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_FU_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_FU_GET_RES_DELAY_CONTROL;
            break;
        default:
            /*no action*/
            break;
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
           it is from the second parameter of classInit*/
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }

    return status;
}
#endif /* !USB_DEVICE_CONFIG_AUDIO_CLASS_2_0 */

/*!
 * @brief Handle the audio device SET request endpoint.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioSetRequestEndpoint(usb_device_audio_struct_t *audioHandle,
                                               usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0;

    /* Select SET request Control for endpoint */
    switch (controlRequest->setup->bRequest)
    {
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
        case USB_DEVICE_AUDIO_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_PITCH_CONTROL_SELECTOR_AUDIO20:
                    audioCommand = USB_DEVICE_AUDIO_EP_PITCH_CONTROL_SELECTOR_AUDIO20;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        default:
            /*no action*/
            break;
#else
        case USB_DEVICE_AUDIO_SET_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_SET_CUR_SAMPLING_FREQ_CONTROL;
                    break;
                case USB_DEVICE_AUDIO_EP_PITCH_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_SET_CUR_PITCH_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MIN_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_SET_MIN_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MAX_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_SET_MAX_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_SET_RES_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_SET_RES_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;

        default:
            /*no action*/
            break;
#endif /* USB_DEVICE_CONFIG_AUDIO_CLASS_2_0 */
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }
    return status;
}

/*!
 * @brief Handle the audio device GET request endpoint.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetRequestEndpoint(usb_device_audio_struct_t *audioHandle,
                                               usb_device_control_request_struct_t *controlRequest)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0;
    /* Select GET request Control for endpoint */
    switch (controlRequest->setup->bRequest)
    {
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
        case USB_DEVICE_AUDIO_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_DATA_OVERRUN_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_GET_CUR_DATA_OVERRUN_CONTROL;
                    break;
                case USB_DEVICE_AUDIO_EP_DATA_UNDERRUN_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_GET_CUR_DATA_UNDERRUN_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
        default:
            /*no action*/
            break;
#else
        case USB_DEVICE_AUDIO_GET_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_GET_CUR_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_GET_MIN_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_GET_MIN_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_GET_MAX_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_SAMPLING_FREQ_CONTROL_SELECTOR:

                    audioCommand = USB_DEVICE_AUDIO_EP_GET_MAX_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_GET_RES_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_EP_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_EP_GET_RES_SAMPLING_FREQ_CONTROL;

                    break;
                default:
                    /*no action*/
                    break;
            }
            break;
        default:
            /*no action*/
            break;
#endif
    }
    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }
    return status;
}

/*!
 * @brief Handle the audio device GET control terminal request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetControlTerminal(usb_device_audio_struct_t *audioHandle,
                                               usb_device_control_request_struct_t *controlRequest, uint8_t terminal_type)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0;
    /* Select GET request Control for endpoint */
    switch (controlRequest->setup->bRequest)
    {
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
        case USB_DEVICE_AUDIO_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_TE_COPY_PROTECT_CONTROL:
                    if (USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_INPUT_TERMINAL == terminal_type)
                    {
                        audioCommand = USB_DEVICE_AUDIO_TE_GET_CUR_COPY_PROTECT_CONTROL;
                    }
                    else
                    {
                        return kStatus_USB_Error; /* Input Terminals only support the Get Terminal Copy Protect Control request */
                    }
                    break;
                case USB_DEVICE_AUDIO_TE_CONNECTOR_CONTROL:
                    audioCommand = USB_DEVICE_AUDIO_TE_GET_CUR_CONNECTOR_CONTROL;
                    break;
                case USB_DEVICE_AUDIO_TE_OVERLOAD_CONTROL:
                    audioCommand = USB_DEVICE_AUDIO_TE_GET_CUR_OVERLOAD_CONTROL;
                    break;
                default:
                    /*no action*/
                    break;
            }
#else
        case USB_DEVICE_AUDIO_GET_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_TE_COPY_PROTECT_CONTROL:
                    if (USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_INPUT_TERMINAL == terminal_type)
                    {
                        audioCommand = USB_DEVICE_AUDIO_TE_GET_CUR_COPY_PROTECT_CONTROL;
                    }
                    else
                    {
                        return kStatus_USB_Error; /* Input Terminals only support the Get Terminal Copy Protect Control request */
                    }
                    break;
                default:
                    /*no action*/
                    break;
            }
#endif
    }

    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }

    return status;
}

/*!
 * @brief Handle the audio device SET control terminal request.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 * @param terminal_type        The terminal type:: input terminal or output terminal
*
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioSetControlTerminal(usb_device_audio_struct_t *audioHandle,
                                               usb_device_control_request_struct_t *controlRequest, uint8_t terminal_type)
{
    usb_status_t status     = kStatus_USB_Error;
    uint8_t controlSelector = (uint8_t)((controlRequest->setup->wValue >> 0x08) & 0xFFU);
    uint32_t audioCommand   = 0;
    /* Select GET request Control for endpoint */
    switch (controlRequest->setup->bRequest)
    {
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
        case USB_DEVICE_AUDIO_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_TE_COPY_PROTECT_CONTROL:
                    if (USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_OUTPUT_TERMINAL == terminal_type)
                    {
                        audioCommand = USB_DEVICE_AUDIO_TE_SET_CUR_COPY_PROTECT_CONTROL;
                    }
                    else
                    {
                       return kStatus_USB_Error; /* Output Terminals only support the Set Terminal Copy Protect Control request */
                    }
                    break;
                default:
                    /*no action*/
                    break;
            }
#else
        case USB_DEVICE_AUDIO_GET_CUR_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_TE_COPY_PROTECT_CONTROL:
                    if (USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_OUTPUT_TERMINAL == terminal_type)
                    {
                        audioCommand = USB_DEVICE_AUDIO_TE_SET_CUR_COPY_PROTECT_CONTROL;
                    }
                    else
                    {
                       return kStatus_USB_Error; /* Output Terminals only support the Set Terminal Copy Protect Control request */
                    }
                    break;
                default:
                    /*no action*/
                    break;
            }
#endif
    }

    if ((0U != audioCommand) && (NULL != audioHandle->configStruct) &&
        (NULL != audioHandle->configStruct->classCallback))
    {
        /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
        it is from the second parameter of classInit */
        status = audioHandle->configStruct->classCallback((class_handle_t)audioHandle, audioCommand, controlRequest);
    }

    return status;
}

/*!
 * @brief Handle the audio device set request interface.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioSetRequestInterface(usb_device_audio_struct_t *audioHandle,
                                                usb_device_control_request_struct_t *controlRequest)
{
    uint32_t i;
    usb_device_audio_entities_struct_t *entity_list;
    usb_status_t status = kStatus_USB_Error;
    uint8_t entityId    = (uint8_t)(controlRequest->setup->wIndex >> 0x08);

    entity_list = (usb_device_audio_entities_struct_t *)audioHandle->controlInterfaceHandle->classSpecific;
    for (i = 0; i < entity_list->count; i++)
    {
        if (entityId == entity_list->entity[i].entityId)
        {
            switch (entity_list->entity[i].entityType)
            {
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_OUTPUT_TERMINAL:
                case  USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_INPUT_TERMINAL:
                    /*Select SET Request Control Input Terminal Module */
                    status = USB_DeviceAudioSetControlTerminal(audioHandle, controlRequest, entity_list->entity[i].entityType);
                    break;
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_FEATURE_UNIT:
                    /* Select SET request Control Feature Unit Module */
                    status = USB_DeviceAudioSetFeatureUnit(audioHandle, controlRequest);
                    break;
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_SOURCE_UNIT:
                    /* Select SET request Control Clock Source Module */
                    status = USB_DeviceAudioSetClockSource(audioHandle, controlRequest);
                    break;
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_SELECTOR_UNIT:
                    /* Select SET request Control Clock Selector Module */
                    break;
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_MULTIPLIER_UNIT:
                    /* Select SET request Control Clock multiplier Module */
                    break;
#endif
                default:
                    /*no action*/
                    break;
            }
        }
    }
    return status;
}

/*!
 * @brief Handle the audio device get request interface.
 *
 * This callback function provides flexibility to add class and vendor specific requests
 *
 * @param audioHandle          The device audio class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 * @param controlRequest       The pointer of the control request structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGetRequestInterface(usb_device_audio_struct_t *audioHandle,
                                                usb_device_control_request_struct_t *controlRequest)
{
    uint32_t i;
    usb_device_audio_entities_struct_t *entity_list;
    usb_status_t status = kStatus_USB_Error;
    uint8_t entityId    = (uint8_t)(controlRequest->setup->wIndex >> 0x08);

    entity_list = (usb_device_audio_entities_struct_t *)audioHandle->controlInterfaceHandle->classSpecific;
    for (i = 0; i < entity_list->count; i++)
    {
        if (entityId == entity_list->entity[i].entityId)
        {
            switch (entity_list->entity[i].entityType)
            {
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_INPUT_TERMINAL:
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_OUTPUT_TERMINAL:
                    status = USB_DeviceAudioGetControlTerminal(audioHandle, controlRequest, entity_list->entity[i].entityType);
                    break;
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_FEATURE_UNIT:
                    status = USB_DeviceAudioGetFeatureUnit(audioHandle, controlRequest);
                    break;
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_SOURCE_UNIT:
                    /* Select SET request Control Clock Source Module */
                    status = USB_DeviceAudioGetClockSource(audioHandle, controlRequest);
                    break;
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_SELECTOR_UNIT:
                    /* Select SET request Control Clock Select Module */
                    break;
                case USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_MULTIPLIER_UNIT:
                    /* Select SET request Control Clock Multiplier Module */
                    break;
#endif
                default:
                    /*no action*/
                    break;
            }
        }
    }
    return status;
}

/*!
 * @brief Handle the event passed to the audio class.
 *
 * This function handles the event passed to the audio class.
 *
 * @param handle          The audio class handle, got from the usb_device_class_config_struct_t::classHandle.
 * @param event           The event codes. Please refer to the enumeration usb_device_class_event_t.
 * @param param           The param type is determined by the event code.
 *
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success              Free device handle successfully.
 * @retval kStatus_USB_InvalidParameter     The device handle not be found.
 * @retval kStatus_USB_InvalidRequest       The request is invalid, and the control pipe will be stalled by the caller.
 */
usb_status_t USB_DeviceAudioEvent(void *handle, uint32_t event, void *param)
{
    usb_device_audio_struct_t *audioHandle;
    usb_status_t error = kStatus_USB_Error;
    uint16_t interfaceAlternate;
    uint8_t *temp8;
    uint8_t alternate;
    uint32_t count;
    usb_device_class_event_t eventCode = (usb_device_class_event_t)event;
    if ((NULL == param) || (NULL == handle))
    {
        return kStatus_USB_InvalidHandle;
    }

    /* Get the audio class handle. */
    audioHandle = (usb_device_audio_struct_t *)handle;

    switch (eventCode)
    {
        case kUSB_DeviceClassEventDeviceReset:
            /* Bus reset, clear the configuration. */
            audioHandle->configuration     = 0U;
            audioHandle->streamOutPipeBusy = 0U;
            audioHandle->streamInPipeBusy  = 0U;
            break;
        case kUSB_DeviceClassEventSetConfiguration:
            /* Get the new configuration. */
            temp8 = ((uint8_t *)param);
            if (NULL == audioHandle->configStruct)
            {
                break;
            }
            if (*temp8 == audioHandle->configuration)
            {
                break;
            }
            /* De-initialize the endpoints when current configuration is none zero. */
            if (0U != audioHandle->configuration)
            {
                error = USB_DeviceAudioControlEndpointsDeinit(audioHandle);
                error = USB_DeviceAudioStreamEndpointsDeinit(audioHandle);
            }
            /* Save new configuration. */
            audioHandle->configuration = *temp8;
            /* Clear the alternate setting value. */
            audioHandle->controlAlternate = 0U;
            /* Clear the alternate setting value. */
            audioHandle->streamAlternate        = 0U;
            audioHandle->controlInterfaceHandle = NULL;
            audioHandle->streamInterfaceHandle  = NULL;
            /* Initialize the control endpoints of the new current configuration by using the alternate setting 0. */
            error = USB_DeviceAudioControlEndpointsInit(audioHandle);
            /* Initialize the stream endpoints of the new current configuration by using the alternate setting 0. */
            error = USB_DeviceAudioStreamEndpointsInit(audioHandle);
            break;
        case kUSB_DeviceClassEventSetInterface:
            if (NULL == audioHandle->configStruct)
            {
                break;
            }
            /* Get the new alternate setting of the interface */
            interfaceAlternate = *((uint16_t *)param);
            /* Get the alternate setting value */
            alternate = (uint8_t)(interfaceAlternate & 0xFFU);

            /* Whether the interface belongs to the class. */
            if (audioHandle->controlInterfaceNumber == ((uint8_t)(interfaceAlternate >> 8U)))
            {
                /* When the interface is control interface. */
                /* Only handle new alternate setting. */
                if (alternate == audioHandle->controlAlternate)
                {
                    break;
                }
                /* De-initialize old endpoints */
                error                         = USB_DeviceAudioControlEndpointsDeinit(audioHandle);
                audioHandle->controlAlternate = alternate;
                /* Initialize new endpoints */
                error = USB_DeviceAudioControlEndpointsInit(audioHandle);
            }
            else if (audioHandle->streamInterfaceNumber == ((uint8_t)(interfaceAlternate >> 8U)))
            {
                /* When the interface is stream interface. */
                /* Only handle new alternate setting. */
                if (alternate == audioHandle->streamAlternate)
                {
                    break;
                }
                /* De-initialize old endpoints */
                error                        = USB_DeviceAudioStreamEndpointsDeinit(audioHandle);
                audioHandle->streamAlternate = alternate;
                /* Initialize new endpoints */
                error = USB_DeviceAudioStreamEndpointsInit(audioHandle);
            }
            else
            {
                /*no action*/
            }
            break;
        case kUSB_DeviceClassEventSetEndpointHalt:
            if (NULL == audioHandle->configStruct)
            {
                break;
            }
            /* Get the endpoint address */
            temp8 = ((uint8_t *)param);
            if (NULL != audioHandle->controlInterfaceHandle)
            {
                for (count = 0U; count < audioHandle->controlInterfaceHandle->endpointList.count; count++)
                {
                    if (*temp8 == audioHandle->controlInterfaceHandle->endpointList.endpoint[count].endpointAddress)
                    {
                        /* Only stall the endpoint belongs to control interface of the class */
                        error = USB_DeviceStallEndpoint(audioHandle->handle, *temp8);
                    }
                }
            }
            if (NULL != audioHandle->streamInterfaceHandle)
            {
                for (count = 0U; count < audioHandle->streamInterfaceHandle->endpointList.count; count++)
                {
                    if (*temp8 == audioHandle->streamInterfaceHandle->endpointList.endpoint[count].endpointAddress)
                    {
                        /* Only stall the endpoint belongs to stream interface of the class */
                        error = USB_DeviceStallEndpoint(audioHandle->handle, *temp8);
                    }
                }
            }
            break;
        case kUSB_DeviceClassEventClearEndpointHalt:
            if (NULL == audioHandle->configStruct)
            {
                break;
            }
            /* Get the endpoint address */
            temp8 = ((uint8_t *)param);
            if (NULL != audioHandle->controlInterfaceHandle)
            {
                for (count = 0U; count < audioHandle->controlInterfaceHandle->endpointList.count; count++)
                {
                    if (*temp8 == audioHandle->controlInterfaceHandle->endpointList.endpoint[count].endpointAddress)
                    {
                        /* Only un-stall the endpoint belongs to control interface of the class */
                        error = USB_DeviceUnstallEndpoint(audioHandle->handle, *temp8);
                    }
                }
            }
            if (NULL != audioHandle->streamInterfaceHandle)
            {
                for (count = 0U; count < audioHandle->streamInterfaceHandle->endpointList.count; count++)
                {
                    if (*temp8 == audioHandle->streamInterfaceHandle->endpointList.endpoint[count].endpointAddress)
                    {
                        /* Only un-stall the endpoint belongs to stream interface of the class */
                        error = USB_DeviceUnstallEndpoint(audioHandle->handle, *temp8);
                    }
                }
            }
            break;
        case kUSB_DeviceClassEventClassRequest:
        {
            /* Handle the audio class specific request. */
            usb_device_control_request_struct_t *controlRequest = (usb_device_control_request_struct_t *)param;
            
            if ((controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) ==
                USB_REQUEST_TYPE_RECIPIENT_ENDPOINT)
            {
                if (USB_REQUEST_TYPE_TYPE_CLASS == (controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_TYPE_MASK))
                {
                    switch (controlRequest->setup->bmRequestType)
                    {
                        case USB_DEVICE_AUDIO_SET_REQUEST_ENDPOINT:
                            error = USB_DeviceAudioSetRequestEndpoint(audioHandle, controlRequest);
                            break;
                        case USB_DEVICE_AUDIO_GET_REQUEST_ENDPOINT:
                            error = USB_DeviceAudioGetRequestEndpoint(audioHandle, controlRequest);
                            break;
                        default:
                            /*no action*/
                            break;
                    }
                }
            }
            else if ((controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) ==
                USB_REQUEST_TYPE_RECIPIENT_INTERFACE)
            {
                uint8_t interface_index = (uint8_t)controlRequest->setup->wIndex;

                if (audioHandle->controlInterfaceNumber == interface_index)
                {
                    if (USB_REQUEST_TYPE_TYPE_CLASS ==
                        (controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_TYPE_MASK))
                    {
                        switch (controlRequest->setup->bmRequestType)
                        {
                            case USB_DEVICE_AUDIO_SET_REQUEST_INTERFACE:
                                error = USB_DeviceAudioSetRequestInterface(audioHandle, controlRequest);
                                break;
                            case USB_DEVICE_AUDIO_GET_REQUEST_INTERFACE:
                                error = USB_DeviceAudioGetRequestInterface(audioHandle, controlRequest);
                                break;
                            default:
                                /*no action*/
                                break;
                        }
                    }
                }
            }
            else
            {
                /* no action */
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
 * @brief Initialize the audio class.
 *
 * This function is used to initialize the audio class.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param config          The class configuration information.
 * @param handle          It is out parameter, is used to return pointer of the audio class handle to the caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioInit(uint8_t controllerId, usb_device_class_config_struct_t *config, class_handle_t *handle)
{
    usb_device_audio_struct_t *audioHandle;
    usb_status_t error = kStatus_USB_Error;

    /* Allocate a audio class handle. */
    error = USB_DeviceAudioAllocateHandle(&audioHandle);

    if (kStatus_USB_Success != error)
    {
        return error;
    }

    /* Get the device handle according to the controller id. */
    error = USB_DeviceClassGetDeviceHandle(controllerId, &audioHandle->handle);

    if (kStatus_USB_Success != error)
    {
        return error;
    }

    if (NULL == audioHandle->handle)
    {
        return kStatus_USB_InvalidHandle;
    }
    /* Save the configuration of the class. */
    audioHandle->configStruct = config;
    /* Clear the configuration value. */
    audioHandle->configuration    = 0U;
    audioHandle->streamAlternate  = 0xffU;
    audioHandle->controlAlternate = 0xffU;

    *handle = (class_handle_t)audioHandle;
    return error;
}

/*!
 * @brief De-initialize the device audio class.
 *
 * The function de-initializes the device audio class.
 *
 * @param handle The ccid class handle got from usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioDeinit(class_handle_t handle)
{
    usb_device_audio_struct_t *audioHandle;
    usb_status_t error = kStatus_USB_Error;

    audioHandle = (usb_device_audio_struct_t *)handle;

    if (NULL == audioHandle)
    {
        return kStatus_USB_InvalidHandle;
    }
    error = USB_DeviceAudioStreamEndpointsDeinit(audioHandle);
    error = USB_DeviceAudioControlEndpointsDeinit(audioHandle);
    (void)USB_DeviceAudioFreeHandle(audioHandle);
    return error;
}

/*!
 * @brief Send data through a specified endpoint.
 *
 * The function is used to send data through a specified endpoint.
 * The function calls USB_DeviceSendRequest internally.
 *
 * @param handle The audio class handle got from usb_device_class_config_struct_t::classHandle.
 * @param endpointAddress Endpoint index.
 * @param buffer The memory address to hold the data need to be sent.
 * @param length The data length need to be sent.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the sending request is successful or not; the transfer done is notified by
 * usb_device_audio_stream_in or usb_device_audio_control_in.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t USB_DeviceAudioSend(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length)
{
    usb_device_audio_struct_t *audioHandle;
    usb_status_t error = kStatus_USB_Error;

    if (NULL == handle)
    {
        return kStatus_USB_InvalidHandle;
    }
    audioHandle = (usb_device_audio_struct_t *)handle;

    if (0U != audioHandle->streamInPipeBusy)
    {
        return kStatus_USB_Busy;
    }
    audioHandle->streamInPipeBusy = 1U;

    error = USB_DeviceSendRequest(audioHandle->handle, ep, buffer, length);
    if (kStatus_USB_Success != error)
    {
        audioHandle->streamInPipeBusy = 0U;
    }
    return error;
}

/*!
 * @brief Receive data through a specified endpoint.
 *
 * The function is used to receive data through a specified endpoint.
 * The function calls USB_DeviceRecvRequest internally.
 *
 * @param handle The audio class handle got from usb_device_class_config_struct_t::classHandle.
 * @param endpointAddress Endpoint index.
 * @param buffer The memory address to save the received data.
 * @param length The data length want to be received.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the receiving request is successful or not; the transfer done is notified by
 * usb_device_audio_stream_out.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t USB_DeviceAudioRecv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length)
{
    usb_device_audio_struct_t *audioHandle;
    usb_status_t error = kStatus_USB_Error;

    if (NULL == handle)
    {
        return kStatus_USB_InvalidHandle;
    }
    audioHandle = (usb_device_audio_struct_t *)handle;

    if (0U != audioHandle->streamOutPipeBusy)
    {
        return kStatus_USB_Busy;
    }
    audioHandle->streamOutPipeBusy = 1U;

    error = USB_DeviceRecvRequest(audioHandle->handle, ep, buffer, length);
    if (kStatus_USB_Success != error)
    {
        audioHandle->streamOutPipeBusy = 0U;
    }
    return error;
}

#endif
