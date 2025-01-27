/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016,2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_host_config.h"
#if ((defined USB_HOST_CONFIG_PRINTER) && (USB_HOST_CONFIG_PRINTER))
#include "usb_host.h"
#include "usb_host_printer.h"

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief printer in pipe transfer callback.
 *
 * @param param       callback parameter.
 * @param transfer    callback transfer.
 * @param status      transfer status.
 */
static void USB_HostPrinterInPipeCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status);

/*!
 * @brief printer out pipe transfer callback.
 *
 * @param param       callback parameter.
 * @param transfer    callback transfer.
 * @param status      transfer status.
 */
static void USB_HostPrinterOutPipeCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status);

/*!
 * @brief printer control pipe transfer callback.
 *
 * @param param       callback parameter.
 * @param transfer    callback transfer.
 * @param status      transfer status.
 */
static void USB_HostPrinterControlPipeCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status);

/*!
 * @brief printer open interface. It is called when set interface request success or open alternate setting 0 interface.
 *
 * @param printerInstance     printer instance pointer.
 *
 * @return kStatus_USB_Success or error codes.
 */
static usb_status_t USB_HostPrinterOpenInterface(usb_host_printer_instance_t *printerInstance);

/*!
 * @brief printer set interface callback, open pipes.
 *
 * @param param       callback parameter.
 * @param transfer    callback transfer.
 * @param status      transfer status.
 */
static void USB_HostPrinterSetInterfaceCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status);

/*!
 * @brief printer send control transfer common code.
 *
 * @param classHandle    the class handle.
 * @param requestType    setup packet request type.
 * @param request        setup packet request value.
 * @param wvalue         setup packet wValue.
 * @param windex         setup packet wIndex.
 * @param wlength        setup packet wlength value.
 * @param data           data buffer pointer
 * @param callbackFn     this callback is called after this function completes.
 * @param callbackParam  the first parameter in the callback function.
 *
 * @return An error code or kStatus_USB_Success.
 */
static usb_status_t USB_HostPrinterControl(usb_host_printer_instance_t *printerInstance,
                                           uint8_t requestType,
                                           uint8_t request,
                                           uint16_t wvalue,
                                           uint16_t windex,
                                           uint16_t wlength,
                                           uint8_t *data,
                                           transfer_callback_t callbackFn,
                                           void *callbackParam);

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*******************************************************************************
 * Code
 ******************************************************************************/

#if ((defined USB_HOST_CONFIG_CLASS_AUTO_CLEAR_STALL) && USB_HOST_CONFIG_CLASS_AUTO_CLEAR_STALL)
static void USB_HostPrinterClearInHaltCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)param;

    if (printerInstance->inCallbackFn != NULL)
    {
        /* callback to application, callback function is initialized in USB_HostPrinterRecv */
        printerInstance->inCallbackFn(printerInstance->outCallbackParam, printerInstance->stallDataBuffer,
                                      printerInstance->stallDataLength, kStatus_USB_TransferStall);
    }
    (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
}

static void USB_HostPrinterClearOutHaltCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)param;

    if (printerInstance->outCallbackFn != NULL)
    {
        /* callback to application, callback function is initialized in USB_HostPrinterSend */
        printerInstance->outCallbackFn(printerInstance->outCallbackParam, printerInstance->stallDataBuffer,
                                       printerInstance->stallDataLength, kStatus_USB_TransferStall);
    }
    (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
}

static usb_status_t USB_HostPrinterClearHalt(usb_host_printer_instance_t *printerInstance,
                                             usb_host_transfer_t *stallTransfer,
                                             host_inner_transfer_callback_t callbackFn,
                                             uint8_t endpoint)
{
    usb_status_t status;
    usb_host_transfer_t *transfer;

    /* malloc one transfer */
    status = USB_HostMallocTransfer(printerInstance->hostHandle, &transfer);
    if (status != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("allocate transfer error\r\n");
#endif
        return status;
    }
    printerInstance->stallDataBuffer = stallTransfer->transferBuffer;
    printerInstance->stallDataLength = stallTransfer->transferSofar;
    /* save the application callback function */
    printerInstance->controlCallbackFn    = NULL;
    printerInstance->controlCallbackParam = NULL;
    /* initialize transfer */
    transfer->callbackFn                 = callbackFn;
    transfer->callbackParam              = printerInstance;
    transfer->transferBuffer             = NULL;
    transfer->transferLength             = 0;
    transfer->setupPacket->bRequest      = USB_REQUEST_STANDARD_CLEAR_FEATURE;
    transfer->setupPacket->bmRequestType = USB_REQUEST_TYPE_RECIPIENT_ENDPOINT;
    transfer->setupPacket->wValue  = USB_SHORT_TO_LITTLE_ENDIAN(USB_REQUEST_STANDARD_FEATURE_SELECTOR_ENDPOINT_HALT);
    transfer->setupPacket->wIndex  = USB_SHORT_TO_LITTLE_ENDIAN(endpoint);
    transfer->setupPacket->wLength = 0;
    status = USB_HostSendSetup(printerInstance->hostHandle, printerInstance->controlPipe, transfer);

    if (status != kStatus_USB_Success)
    {
        (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
    }
    printerInstance->controlTransfer = transfer;

    return status;
}
#endif

static void USB_HostPrinterInPipeCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)param;

#if ((defined USB_HOST_CONFIG_CLASS_AUTO_CLEAR_STALL) && USB_HOST_CONFIG_CLASS_AUTO_CLEAR_STALL)
    if (status == kStatus_USB_TransferStall)
    {
        if (USB_HostPrinterClearHalt(
                printerInstance, transfer, USB_HostPrinterClearInHaltCallback,
                (USB_REQUEST_TYPE_DIR_IN | ((usb_host_pipe_t *)printerInstance->inPipe)->endpointAddress)) ==
            kStatus_USB_Success)
        {
            (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
            return;
        }
    }
#endif

    if (printerInstance->inCallbackFn != NULL)
    {
        /* callback to application, callback function is initialized in USB_HostPrinterRecv */
        printerInstance->inCallbackFn(printerInstance->inCallbackParam, transfer->transferBuffer,
                                      transfer->transferSofar, status);
    }
    (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
}

static void USB_HostPrinterOutPipeCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)param;

#if ((defined USB_HOST_CONFIG_CLASS_AUTO_CLEAR_STALL) && USB_HOST_CONFIG_CLASS_AUTO_CLEAR_STALL)
    if (status == kStatus_USB_TransferStall)
    {
        if (USB_HostPrinterClearHalt(
                printerInstance, transfer, USB_HostPrinterClearOutHaltCallback,
                (USB_REQUEST_TYPE_DIR_OUT | ((usb_host_pipe_t *)printerInstance->outPipe)->endpointAddress)) ==
            kStatus_USB_Success)
        {
            (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
            return;
        }
    }
#endif
    if (printerInstance->outCallbackFn != NULL)
    {
        /* callback to application, callback function is initialized in USB_HostPrinterSend */
        printerInstance->outCallbackFn(printerInstance->outCallbackParam, transfer->transferBuffer,
                                       transfer->transferSofar, status);
    }
    (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
}

static void USB_HostPrinterControlPipeCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)param;

    printerInstance->controlTransfer = NULL;
    if (printerInstance->controlCallbackFn != NULL)
    {
        /* callback to application, callback to application, callback function is initialized in the
        USB_HostPrinterControl,
        USB_HostPrinterSetInterface or USB_HostHubClassRequestCommon, but is the same function */
        printerInstance->controlCallbackFn(printerInstance->controlCallbackParam, transfer->transferBuffer,
                                           transfer->transferSofar, status);
    }
    (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
}

static usb_status_t USB_HostPrinterOpenInterface(usb_host_printer_instance_t *printerInstance)
{
    usb_status_t status;
    uint8_t epIndex = 0;
    usb_host_pipe_init_t pipeInit;
    usb_descriptor_endpoint_t *epDesc = NULL;
    usb_host_interface_t *interfacePointer;

    if (printerInstance->inPipe != NULL)
    {
        status = USB_HostClosePipe(printerInstance->hostHandle, printerInstance->inPipe);

        if (status != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error when close pipe\r\n");
#endif
        }
        printerInstance->inPipe = NULL;
    }
    if (printerInstance->outPipe != NULL) /* close interrupt out pipe if it is open */
    {
        status = USB_HostClosePipe(printerInstance->hostHandle, printerInstance->outPipe);

        if (status != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error when close pipe\r\n");
#endif
        }
        printerInstance->outPipe = NULL;
    }

    /* open interface pipes */
    interfacePointer = (usb_host_interface_t *)printerInstance->interfaceHandle;
    for (epIndex = 0; epIndex < interfacePointer->epCount; ++epIndex)
    {
        epDesc = interfacePointer->epList[epIndex].epDesc;
        if (((epDesc->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
             USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN) &&
            ((epDesc->bmAttributes & USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_TYPE_MASK) == USB_ENDPOINT_BULK))
        {
            pipeInit.devInstance     = printerInstance->deviceHandle;
            pipeInit.pipeType        = USB_ENDPOINT_BULK;
            pipeInit.direction       = USB_IN;
            pipeInit.endpointAddress = (epDesc->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_NUMBER_MASK);
            pipeInit.interval        = epDesc->bInterval;
            pipeInit.maxPacketSize   = (uint16_t)((USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(epDesc->wMaxPacketSize) &
                                                 USB_DESCRIPTOR_ENDPOINT_MAXPACKETSIZE_SIZE_MASK));
            pipeInit.numberPerUframe = (uint8_t)((USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(epDesc->wMaxPacketSize) &
                                                  USB_DESCRIPTOR_ENDPOINT_MAXPACKETSIZE_MULT_TRANSACTIONS_MASK));
            pipeInit.nakCount        = USB_HOST_CONFIG_MAX_NAK;

            printerInstance->inPacketSize = pipeInit.maxPacketSize;

            status = USB_HostOpenPipe(printerInstance->hostHandle, &printerInstance->inPipe, &pipeInit);
            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("USB_HostPrinterSetInterface fail to open pipe\r\n");
#endif
                return kStatus_USB_Error;
            }
        }
        else if (((epDesc->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                  USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
                 ((epDesc->bmAttributes & USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_TYPE_MASK) == USB_ENDPOINT_BULK))
        {
            pipeInit.devInstance     = printerInstance->deviceHandle;
            pipeInit.pipeType        = USB_ENDPOINT_BULK;
            pipeInit.direction       = USB_OUT;
            pipeInit.endpointAddress = (epDesc->bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_NUMBER_MASK);
            pipeInit.interval        = epDesc->bInterval;
            pipeInit.maxPacketSize   = (uint16_t)((USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(epDesc->wMaxPacketSize) &
                                                 USB_DESCRIPTOR_ENDPOINT_MAXPACKETSIZE_SIZE_MASK));
            pipeInit.numberPerUframe = (uint8_t)((USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(epDesc->wMaxPacketSize) &
                                                  USB_DESCRIPTOR_ENDPOINT_MAXPACKETSIZE_MULT_TRANSACTIONS_MASK));
            pipeInit.nakCount        = USB_HOST_CONFIG_MAX_NAK;

            printerInstance->outPacketSize = pipeInit.maxPacketSize;

            status = USB_HostOpenPipe(printerInstance->hostHandle, &printerInstance->outPipe, &pipeInit);
            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("USB_HostPrinterSetInterface fail to open pipe\r\n");
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

static void USB_HostPrinterSetInterfaceCallback(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)param;

    printerInstance->controlTransfer = NULL;
    if (status == kStatus_USB_Success)
    {
        status = USB_HostPrinterOpenInterface(printerInstance); /* printer open interface */
    }

    if (printerInstance->controlCallbackFn != NULL)
    {
        /* callback to application, callback to application, callback function is initialized in the
        USB_HostPrinterControl,
        USB_HostPrinterSetInterface or USB_HostHubClassRequestCommon, but is the same function */
        printerInstance->controlCallbackFn(printerInstance->controlCallbackParam, NULL, 0, status);
    }
    (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
}

usb_status_t USB_HostPrinterInit(usb_device_handle deviceHandle, usb_host_class_handle *classHandle)
{
    uint32_t infoValue;
    uint32_t *temp;
    /* malloc printer class instance */
    usb_host_printer_instance_t *printerInstance =
        (usb_host_printer_instance_t *)OSA_MemoryAllocate(sizeof(usb_host_printer_instance_t));

    if (printerInstance == NULL)
    {
        return kStatus_USB_AllocFail;
    }

    /* initialize printer instance */
    printerInstance->deviceHandle    = deviceHandle;
    printerInstance->interfaceHandle = NULL;

    (void)USB_HostHelperGetPeripheralInformation(deviceHandle, (uint32_t)kUSB_HostGetHostHandle, &infoValue);
    temp                        = (uint32_t *)infoValue;
    printerInstance->hostHandle = (usb_host_handle)temp;
    (void)USB_HostHelperGetPeripheralInformation(deviceHandle, (uint32_t)kUSB_HostGetDeviceControlPipe, &infoValue);
    temp                         = (uint32_t *)infoValue;
    printerInstance->controlPipe = (usb_host_pipe_handle)temp;

    *classHandle = printerInstance;
    return kStatus_USB_Success;
}

usb_status_t USB_HostPrinterSetInterface(usb_host_class_handle classHandle,
                                         usb_host_interface_handle interfaceHandle,
                                         uint8_t alternateSetting,
                                         transfer_callback_t callbackFn,
                                         void *callbackParam)
{
    usb_status_t status;
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)classHandle;
    usb_host_transfer_t *transfer;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidParameter;
    }

    /* notify host driver the interface is open */
    status = USB_HostOpenDeviceInterface(printerInstance->deviceHandle, interfaceHandle);
    if (status != kStatus_USB_Success)
    {
        return status;
    }
    printerInstance->interfaceHandle = interfaceHandle;

    /* cancel transfers */
    if (printerInstance->inPipe != NULL)
    {
        status = USB_HostCancelTransfer(printerInstance->hostHandle, printerInstance->inPipe, NULL);

        if (status != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error when cancel pipe\r\n");
#endif
        }
    }
    if (printerInstance->outPipe != NULL)
    {
        status = USB_HostCancelTransfer(printerInstance->hostHandle, printerInstance->outPipe, NULL);

        if (status != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error when cancel pipe\r\n");
#endif
        }
    }

    if (alternateSetting == 0U) /* open interface directly */
    {
        if (callbackFn != NULL)
        {
            status = USB_HostPrinterOpenInterface(printerInstance);
            callbackFn(callbackParam, NULL, 0, status);
        }
    }
    else /* send setup transfer */
    {
        /* malloc one transfer */
        if (USB_HostMallocTransfer(printerInstance->hostHandle, &transfer) != kStatus_USB_Success)
        {
#ifdef HOST_ECHO
            usb_echo("error to get transfer\r\n");
#endif
            return kStatus_USB_Error;
        }

        /* save the application callback function */
        printerInstance->controlCallbackFn    = callbackFn;
        printerInstance->controlCallbackParam = callbackParam;
        /* initialize transfer */
        transfer->callbackFn                 = USB_HostPrinterSetInterfaceCallback;
        transfer->callbackParam              = printerInstance;
        transfer->setupPacket->bRequest      = USB_REQUEST_STANDARD_SET_INTERFACE;
        transfer->setupPacket->bmRequestType = USB_REQUEST_TYPE_RECIPIENT_INTERFACE;
        transfer->setupPacket->wIndex        = USB_SHORT_TO_LITTLE_ENDIAN(
            ((usb_host_interface_t *)printerInstance->interfaceHandle)->interfaceDesc->bInterfaceNumber);
        transfer->setupPacket->wValue  = USB_SHORT_TO_LITTLE_ENDIAN(alternateSetting);
        transfer->setupPacket->wLength = 0;
        transfer->transferBuffer       = NULL;
        transfer->transferLength       = 0;
        status = USB_HostSendSetup(printerInstance->hostHandle, printerInstance->controlPipe, transfer);

        if (status == kStatus_USB_Success)
        {
            printerInstance->controlTransfer = transfer;
        }
        else
        {
            (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
        }
    }

    return status;
}

usb_status_t USB_HostPrinterDeinit(usb_device_handle deviceHandle, usb_host_class_handle classHandle)
{
    usb_status_t status;
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)classHandle;

    if (deviceHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (classHandle != NULL) /* class instance has initialized */
    {
        if (printerInstance->inPipe != NULL)
        {
            status =
                USB_HostCancelTransfer(printerInstance->hostHandle, printerInstance->inPipe, NULL); /* cancel pipe */
            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("error when cancel pipe\r\n");
#endif
            }
            status = USB_HostClosePipe(printerInstance->hostHandle, printerInstance->inPipe); /* close pipe */

            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("error when close pipe\r\n");
#endif
            }
            printerInstance->inPipe = NULL;
        }
        if (printerInstance->outPipe != NULL)
        {
            status =
                USB_HostCancelTransfer(printerInstance->hostHandle, printerInstance->outPipe, NULL); /* cancel pipe */
            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("error when cancel pipe\r\n");
#endif
            }
            status = USB_HostClosePipe(printerInstance->hostHandle, printerInstance->outPipe); /* close pipe */

            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("error when close pipe\r\n");
#endif
            }
            printerInstance->outPipe = NULL;
        }
        if ((printerInstance->controlPipe != NULL) &&
            (printerInstance->controlTransfer !=
             NULL)) /* cancel control transfer if there is on-going control transfer */
        {
            status = USB_HostCancelTransfer(printerInstance->hostHandle, printerInstance->controlPipe,
                                            printerInstance->controlTransfer);
            if (status != kStatus_USB_Success)
            {
#ifdef HOST_ECHO
                usb_echo("error when cancel pipe\r\n");
#endif
            }
        }
        (void)USB_HostCloseDeviceInterface(
            deviceHandle, printerInstance->interfaceHandle); /* notify host driver the interface is closed */
        OSA_MemoryFree(printerInstance);
    }
    else
    {
        (void)USB_HostCloseDeviceInterface(deviceHandle, NULL);
    }

    return kStatus_USB_Success;
}

usb_status_t USB_HostPrinterRecv(usb_host_class_handle classHandle,
                                 uint8_t *buffer,
                                 uint32_t bufferLength,
                                 transfer_callback_t callbackFn,
                                 void *callbackParam)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)classHandle;
    usb_host_transfer_t *transfer;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (printerInstance->inPipe == NULL)
    {
        return kStatus_USB_Error;
    }

    /* malloc one transfer */
    if (USB_HostMallocTransfer(printerInstance->hostHandle, &transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("error to get transfer\r\n");
#endif
        return kStatus_USB_Busy;
    }
    /* save the application callback function */
    printerInstance->inCallbackFn    = callbackFn;
    printerInstance->inCallbackParam = callbackParam;
    /* initialize transfer */
    transfer->transferBuffer = buffer;
    transfer->transferLength = bufferLength;
    transfer->callbackFn     = USB_HostPrinterInPipeCallback;
    transfer->callbackParam  = printerInstance;

    /* call host driver api */
    if (USB_HostRecv(printerInstance->hostHandle, printerInstance->inPipe, transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("fail to USB_HostRecv\r\n");
#endif
        (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
        return kStatus_USB_Error;
    }

    return kStatus_USB_Success;
}

uint16_t USB_HostPrinterGetPacketsize(usb_host_class_handle classHandle, uint8_t pipeType, uint8_t direction)
{
    usb_host_printer_instance_t *phdcInstance = (usb_host_printer_instance_t *)classHandle;
    if (classHandle == NULL)
    {
        return 0;
    }

    if (pipeType == USB_ENDPOINT_INTERRUPT)
    {
        if (direction == USB_IN)
        {
            return phdcInstance->inPacketSize;
        }
        else
        {
            return phdcInstance->outPacketSize;
        }
    }

    return 0;
}

usb_status_t USB_HostPrinterSend(usb_host_class_handle classHandle,
                                 uint8_t *buffer,
                                 uint32_t bufferLength,
                                 transfer_callback_t callbackFn,
                                 void *callbackParam)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)classHandle;
    usb_host_transfer_t *transfer;

    if (classHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (printerInstance->outPipe == NULL)
    {
        return kStatus_USB_Error;
    }

    /* malloc one transfer */
    if (USB_HostMallocTransfer(printerInstance->hostHandle, &transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("error to get transfer\r\n");
#endif
        return kStatus_USB_Error;
    }
    /* save the application callback function */
    printerInstance->outCallbackFn    = callbackFn;
    printerInstance->outCallbackParam = callbackParam;
    /* initialize transfer */
    transfer->transferBuffer = buffer;
    transfer->transferLength = bufferLength;
    transfer->callbackFn     = USB_HostPrinterOutPipeCallback;
    transfer->callbackParam  = printerInstance;

    /* call host driver api */
    if (USB_HostSend(printerInstance->hostHandle, printerInstance->outPipe, transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("fail to USB_HostSend\r\n");
#endif
        (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
        return kStatus_USB_Error;
    }

    return kStatus_USB_Success;
}

static usb_status_t USB_HostPrinterControl(usb_host_printer_instance_t *printerInstance,
                                           uint8_t requestType,
                                           uint8_t request,
                                           uint16_t wvalue,
                                           uint16_t windex,
                                           uint16_t wlength,
                                           uint8_t *data,
                                           transfer_callback_t callbackFn,
                                           void *callbackParam)
{
    usb_host_transfer_t *transfer;

    if (printerInstance == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (USB_HostMallocTransfer(printerInstance->hostHandle, &transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("error to get transfer\r\n");
#endif
        return kStatus_USB_Busy;
    }
    /* save the application callback function */
    printerInstance->controlCallbackFn    = callbackFn;
    printerInstance->controlCallbackParam = callbackParam;
    /* initialize transfer */
    transfer->transferBuffer             = data;
    transfer->transferLength             = wlength;
    transfer->callbackFn                 = USB_HostPrinterControlPipeCallback;
    transfer->callbackParam              = printerInstance;
    transfer->setupPacket->bmRequestType = requestType;
    transfer->setupPacket->bRequest      = request;
    transfer->setupPacket->wValue        = USB_SHORT_TO_LITTLE_ENDIAN(wvalue);
    transfer->setupPacket->wIndex        = USB_SHORT_TO_LITTLE_ENDIAN(windex);
    transfer->setupPacket->wLength       = USB_SHORT_TO_LITTLE_ENDIAN(wlength);

    /* call host driver api */
    if (USB_HostSendSetup(printerInstance->hostHandle, printerInstance->controlPipe, transfer) != kStatus_USB_Success)
    {
#ifdef HOST_ECHO
        usb_echo("fail for USB_HostSendSetup\r\n");
#endif
        (void)USB_HostFreeTransfer(printerInstance->hostHandle, transfer);
        return kStatus_USB_Error;
    }
    printerInstance->controlTransfer = transfer;

    return kStatus_USB_Success;
}

usb_status_t USB_HostPrinterGetDeviceId(usb_host_class_handle classHandle,
                                        uint8_t interfaceIndex,
                                        uint8_t alternateSetting,
                                        uint8_t *buffer,
                                        uint32_t length,
                                        transfer_callback_t callbackFn,
                                        void *callbackParam)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)classHandle;
    uint32_t infoValue                           = 0U;

    (void)USB_HostHelperGetPeripheralInformation(printerInstance->deviceHandle, (uint32_t)kUSB_HostGetDeviceConfigIndex,
                                                 &infoValue);

    return USB_HostPrinterControl(
        printerInstance, USB_REQUEST_TYPE_DIR_IN | USB_REQUEST_TYPE_TYPE_CLASS | USB_REQUEST_TYPE_RECIPIENT_INTERFACE,
        USB_HOST_PRINTER_GET_DEVICE_ID, (uint16_t)infoValue,
        (uint16_t)((uint16_t)((uint16_t)interfaceIndex << 8U) | alternateSetting), (uint16_t)length, buffer, callbackFn,
        callbackParam);
}

usb_status_t USB_HostPrinterGetPortStatus(usb_host_class_handle classHandle,
                                          uint8_t *portStatus,
                                          transfer_callback_t callbackFn,
                                          void *callbackParam)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)classHandle;

    return USB_HostPrinterControl(
        printerInstance, USB_REQUEST_TYPE_DIR_IN | USB_REQUEST_TYPE_TYPE_CLASS | USB_REQUEST_TYPE_RECIPIENT_INTERFACE,
        USB_HOST_PRINTER_GET_PORT_STATUS, 0,
        (uint16_t)(((usb_host_interface_t *)printerInstance->interfaceHandle)->interfaceDesc->bInterfaceNumber), 1,
        portStatus, callbackFn, callbackParam);
}

usb_status_t USB_HostPrinterSoftReset(usb_host_class_handle classHandle,
                                      transfer_callback_t callbackFn,
                                      void *callbackParam)
{
    usb_host_printer_instance_t *printerInstance = (usb_host_printer_instance_t *)classHandle;

    return USB_HostPrinterControl(
        printerInstance, USB_REQUEST_TYPE_TYPE_CLASS | USB_REQUEST_TYPE_RECIPIENT_INTERFACE,
        USB_HOST_PRINTER_SOFT_RESET, 0,
        (uint16_t)(((usb_host_interface_t *)printerInstance->interfaceHandle)->interfaceDesc->bInterfaceNumber), 0,
        NULL, callbackFn, callbackParam);
}

#endif /* USB_HOST_CONFIG_PRINTER */
