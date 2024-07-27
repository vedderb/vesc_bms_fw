/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017,2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"

#if ((defined(USB_DEVICE_CONFIG_MSC)) && (USB_DEVICE_CONFIG_MSC > 0U))
#include "usb_device_msc.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
usb_status_t USB_DeviceMscRecv(usb_device_msc_struct_t *mscHandle);
usb_status_t USB_DeviceMscSend(usb_device_msc_struct_t *mscHandle);
usb_status_t USB_DeviceMscEndpointsInit(usb_device_msc_struct_t *mscHandle);
usb_status_t USB_DeviceMscEndpointsDeinit(usb_device_msc_struct_t *mscHandle);

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if (USB_DEVICE_CONFIG_MSC == 1U)
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_cbw_t s_MscCbw1;
#define MSCCBW_ARRAY \
    {                \
        &s_MscCbw1   \
    }
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_csw_t s_MscCsw1;
#define MSCCSW_ARRAY \
    {                \
        &s_MscCsw1   \
    }
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_request_sense_data_struct_t requestSense1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity_struct_t readCapacity1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity16_data_struct_t
    readCapacity161;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static uint8_t
    formatCapacityData1[sizeof(usb_device_capacity_list_header_struct_t) +
                        sizeof(usb_device_current_max_capacity_descriptor_struct_t) +
                        sizeof(usb_device_formattable_capacity_descriptor_struct_t) * 3];
#define MSCSENSE_ARRAY \
    {                  \
        &requestSense1 \
    }
#define MSCCPAP_ARRAY  \
    {                  \
        &readCapacity1 \
    }
#define MSCCAPA16_ARRAY  \
    {                    \
        &readCapacity161 \
    }
#define MSCFORMAT_ARRAY         \
    {                           \
        &formatCapacityData1[0] \
    }
#elif (USB_DEVICE_CONFIG_MSC == 2U)
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_cbw_t s_MscCbw1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_cbw_t s_MscCbw2;
#define MSCCBW_ARRAY {&s_MscCbw1, &s_MscCbw2}
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_csw_t s_MscCsw1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_csw_t s_MscCsw2;
#define MSCCSW_ARRAY {&s_MscCsw1, &s_MscCsw2}
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_request_sense_data_struct_t requestSense1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity_struct_t readCapacity1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity16_data_struct_t
    readCapacity161;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static uint8_t
    formatCapacityData1[sizeof(usb_device_capacity_list_header_struct_t) +
                        sizeof(usb_device_current_max_capacity_descriptor_struct_t) +
                        sizeof(usb_device_formattable_capacity_descriptor_struct_t) * 3];
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) usb_device_request_sense_data_struct_t requestSense2;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) usb_device_read_capacity_struct_t readCapacity2;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) usb_device_read_capacity16_data_struct_t readCapacity162;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) uint8_t
    formatCapacityData2[sizeof(usb_device_capacity_list_header_struct_t) +
                        sizeof(usb_device_current_max_capacity_descriptor_struct_t) +
                        sizeof(usb_device_formattable_capacity_descriptor_struct_t) * 3];
#define MSCSENSE_ARRAY {&requestSense1, &requestSense2}
#define MSCCPAP_ARRAY {&readCapacity1, &readCapacity2}
#define MSCCAPA16_ARRAY {&readCapacity161, &readCapacity162}
#define MSCFORMAT_ARRAY {&formatCapacityData1[0], &formatCapacityData2[0]}
#elif (USB_DEVICE_CONFIG_MSC == 3U)
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_cbw_t s_MscCbw1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_cbw_t s_MscCbw2;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_cbw_t s_MscCbw3;
#define MSCCBW_ARRAY {&s_MscCbw1, &s_MscCbw2, &s_MscCbw3}
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_csw_t s_MscCsw1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_csw_t s_MscCsw2;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_csw_t s_MscCsw3;
#define MSCCSW_ARRAY {&s_MscCsw1, &s_MscCsw2, &s_MscCsw3}
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_request_sense_data_struct_t requestSense1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity_struct_t readCapacity1;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity16_data_struct_t
    readCapacity161;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static uint8_t
    formatCapacityData1[sizeof(usb_device_capacity_list_header_struct_t) +
                        sizeof(usb_device_current_max_capacity_descriptor_struct_t) +
                        sizeof(usb_device_formattable_capacity_descriptor_struct_t) * 3];
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_request_sense_data_struct_t requestSense2;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity_struct_t readCapacity2;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity16_data_struct_t
    readCapacity162;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static uint8_t
    formatCapacityData2[sizeof(usb_device_capacity_list_header_struct_t) +
                        sizeof(usb_device_current_max_capacity_descriptor_struct_t) +
                        sizeof(usb_device_formattable_capacity_descriptor_struct_t) * 3];
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_request_sense_data_struct_t requestSense3;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity_struct_t readCapacity3;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_read_capacity16_data_struct_t
    readCapacity163;
USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static uint8_t
    formatCapacityData3[sizeof(usb_device_capacity_list_header_struct_t) +
                        sizeof(usb_device_current_max_capacity_descriptor_struct_t) +
                        sizeof(usb_device_formattable_capacity_descriptor_struct_t) * 3];
#define MSCSENSE_ARRAY {&requestSense1, &requestSense2, &requestSense3}
#define MSCCPAP_ARRAY {&readCapacity1, &readCapacity2, &readCapacity3}
#define MSCCAPA16_ARRAY \
    {&readCapacity161, &readCapacity162, &readCapacity163}
#define MSCFORMAT_ARRAY {&formatCapacityData1[0], &formatCapacityData2[0], &formatCapacityData3[0]}
#else
#error "the max support USB_DEVICE_CONFIG_MSC is 3"
#endif

USB_GLOBAL USB_RAM_ADDRESS_ALIGNMENT(USB_DATA_ALIGN_SIZE) static usb_device_msc_struct_t
    g_msc_handle[USB_DEVICE_CONFIG_MSC];
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Allocate a device msc class handle.
 *
 * This function allocates a device msc class handle.
 *
 * @param handle          It is out parameter, is used to return pointer of the device msc class handle to the caller.
 *
 * @retval kStatus_USB_Success              Get a device msc class handle successfully.
 * @retval kStatus_USB_Busy                 Cannot allocate a device msc class handle.
 */

static usb_status_t USB_DeviceMscAllocateHandle(usb_device_msc_struct_t **handle)
{
    uint32_t count;
    usb_device_msc_cbw_t *cbw_array[]                            = MSCCBW_ARRAY;
    usb_device_msc_csw_t *csw_array[]                            = MSCCSW_ARRAY;
    usb_device_request_sense_data_struct_t *sense[]              = MSCSENSE_ARRAY;
    usb_device_read_capacity_struct_t *capacity_array[]          = MSCCPAP_ARRAY;
    usb_device_read_capacity16_data_struct_t *capacity16_array[] = MSCCAPA16_ARRAY;
    uint8_t *formatCapacityData[]                                = MSCFORMAT_ARRAY;

    for (count = 0; count < USB_DEVICE_CONFIG_MSC; count++)
    {
        if (NULL == g_msc_handle[count].handle)
        {
            g_msc_handle[count].mscCbw                    = (cbw_array[count]);
            g_msc_handle[count].mscCsw                    = (csw_array[count]);
            g_msc_handle[count].mscUfi.requestSense       = (sense[count]);
            g_msc_handle[count].mscUfi.readCapacity       = (capacity_array[count]);
            g_msc_handle[count].mscUfi.readCapacity16     = (capacity16_array[count]);
            g_msc_handle[count].mscUfi.formatCapacityData = (formatCapacityData[count]);
            *handle                                       = &g_msc_handle[count];
            return kStatus_USB_Success;
        }
    }

    return kStatus_USB_Busy;
}

/*!
 * @brief Free a device msc class handle.
 *
 * This function frees a device msc class handle.
 *
 * @param handle          The device msc class handle.
 *
 * @retval kStatus_USB_Success              Free device msc class handle successfully.
 */
static usb_status_t USB_DeviceMscFreeHandle(usb_device_msc_struct_t *handle)
{
    handle->handle              = NULL;
    handle->configurationStruct = (usb_device_class_config_struct_t *)NULL;
    handle->configuration       = 0;
    handle->alternate           = 0;
    return kStatus_USB_Success;
}

/*!
 * @brief Process usb msc ufi command.
 *
 * This function analyse the cbw , get the command code.
 *
 * @param handle          The device msc class handle.
 *
 * @retval kStatus_USB_Success              Free device msc class handle successfully.
 */
static usb_status_t USB_DeviceMscProcessUfiCommand(usb_device_msc_struct_t *mscHandle)
{
    usb_status_t status;
    usb_device_msc_ufi_struct_t *ufi = NULL;

    ufi = &mscHandle->mscUfi;
    if (USB_DEVICE_MSC_REQUEST_SENSE_COMMAND != mscHandle->mscCbw->cbwcb[0])
    {
        ufi->requestSense->senseKey                = USB_DEVICE_MSC_UFI_NO_SENSE;
        ufi->requestSense->additionalSenseCode     = USB_DEVICE_MSC_UFI_NO_SENSE;
        ufi->requestSense->additionalSenseQualifer = USB_DEVICE_MSC_UFI_NO_SENSE;
    }
    ufi->thirteenCase.hostExpectedDataLength = mscHandle->mscCbw->dataTransferLength;
    ufi->thirteenCase.hostExpectedDirection = (uint8_t)(mscHandle->mscCbw->flags >> USB_DEVICE_MSC_CBW_DIRECTION_SHIFT);
    /*The first byte of all ufi command blocks shall contain an Operation Code, refer to ufi spec*/
    switch (mscHandle->mscCbw->cbwcb[0])
    {
        /* ufi command operation code*/
        case USB_DEVICE_MSC_INQUIRY_COMMAND: /*operation code : 0x12*/
            status = USB_DeviceMscUfiInquiryCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_READ_10_COMMAND: /*operation code : 0x28 */
        case USB_DEVICE_MSC_READ_12_COMMAND: /*operation code : 0xA8 */
            status = USB_DeviceMscUfiReadCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_REQUEST_SENSE_COMMAND: /*operation code : 0x03*/
            status = USB_DeviceMscUfiRequestSenseCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_TEST_UNIT_READY_COMMAND: /*operation code : 0x00 */
            status = USB_DeviceMscUfiTestUnitReadyCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_WRITE_10_COMMAND: /*operation code : 0x2A */
        case USB_DEVICE_MSC_WRITE_12_COMMAND: /*operation code : 0xAA */
            status = USB_DeviceMscUfiWriteCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_PREVENT_ALLOW_MEDIUM_REM_COMMAND: /*operation code :0x1E */
            status = USB_DeviceMscUfiPreventAllowMediumCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_FORMAT_UNIT_COMMAND: /*operation code : 0x04*/
            status = USB_DeviceMscUfiFormatUnitCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_READ_CAPACITY_10_COMMAND: /*operation code : 0x25*/
        case USB_DEVICE_MSC_READ_CAPACITY_16_COMMAND: /*operation code : 0x9E*/
            status = USB_DeviceMscUfiReadCapacityCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_MODE_SENSE_10_COMMAND: /* operation code :0x5A*/
        case USB_DEVICE_MSC_MODE_SENSE_6_COMMAND:  /* operation code : 0x1A */
            status = USB_DeviceMscUfiModeSenseCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_MODE_SELECT_10_COMMAND: /*operation code : 0x55 */
        case USB_DEVICE_MSC_MODE_SELECT_6_COMMAND:  /*operation code : 0x15 */
            status = USB_DeviceMscUfiModeSelectCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_READ_FORMAT_CAPACITIES_COMMAND: /*operation code : 0x23 */
            status = USB_DeviceMscUfiReadFormatCapacityCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_SEND_DIAGNOSTIC_COMMAND: /*operation code : 0x1D*/
            status = USB_DeviceMscUfiSendDiagnosticCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_VERIFY_COMMAND: /*operation code : 0x2F*/
            status = USB_DeviceMscUfiVerifyCommand(mscHandle);
            break;
        case USB_DEVICE_MSC_START_STOP_UNIT_COMMAND: /*operation code : 0x1B*/
            status = USB_DeviceMscUfiStartStopUnitCommand(mscHandle);
            break;
        default:
            status = USB_DeviceMscUfiUnsupportCommand(mscHandle);
            break;
    }
    if ((USB_DEVICE_MSC_UFI_NO_SENSE != ufi->requestSense->senseKey) &&
        (USB_DEVICE_MSC_COMMAND_PASSED == mscHandle->mscCsw->cswStatus) &&
        (USB_DEVICE_MSC_REQUEST_SENSE_COMMAND != mscHandle->mscCbw->cbwcb[0]))
    {
        mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
    }
    return status;
}

/*!
 * @brief Bulk IN endpoint callback function.
 *
 * This callback function is used to notify upper layer the transfer result of a transfer.
 * This callback pointer is passed when the Bulk IN pipe initialized.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param message         The result of the Bulk IN pipe transfer.
 * @param callbackParam  The parameter for this callback. It is same with
 * usb_device_endpoint_callback_struct_t::callbackParam. In the class, the value is the MSC class handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceMscBulkIn(usb_device_handle handle,
                                        usb_device_endpoint_callback_message_struct_t *message,
                                        void *callbackParam)
{
    usb_device_msc_struct_t *mscHandle = (usb_device_msc_struct_t *)callbackParam;
    usb_device_msc_csw_t *csw;
    void *temp;
    usb_status_t error = kStatus_USB_Error;
    /* endpoint callback length is USB_CANCELLED_TRANSFER_LENGTH (0xFFFFFFFFU) when transfer is canceled */
    if (message->length == USB_CANCELLED_TRANSFER_LENGTH)
    {
        /*this code is called when stack cancel the transfer, app should release the buffer it use */
        if ((0U != mscHandle->dataInFlag) && (mscHandle->configurationStruct->classCallback != NULL) &&
            ((USB_DEVICE_MSC_READ_10_COMMAND == mscHandle->mscCbw->cbwcb[0]) ||
             (USB_DEVICE_MSC_READ_12_COMMAND == mscHandle->mscCbw->cbwcb[0])))
        {
            usb_device_lba_app_struct_t lbaData;

            lbaData.size   = 0;
            lbaData.buffer = message->buffer;
            lbaData.offset = 0;
            lbaData.lun    = mscHandle->mscCbw->logicalUnitNumber;
            /*classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
            it is from the second parameter of classInit */
            (void)mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle,
                                                                kUSB_DeviceMscEventReadResponse, (void *)&lbaData);
        }
        if (mscHandle->inEndpointCswCancelFlag == 1U)
        {
            mscHandle->inEndpointCswCancelFlag = 0U;
            /*cancel the transfer and wait for the calcel to be complete in bulk in callback*/
            /*send csw*/
            mscHandle->mscCsw->dataResidue = USB_LONG_TO_LITTLE_ENDIAN(mscHandle->mscCsw->dataResidue);
            (void)USB_DeviceSendRequest(mscHandle->handle, mscHandle->bulkInEndpoint, (uint8_t *)mscHandle->mscCsw,
                                        USB_DEVICE_MSC_CSW_LENGTH);
            mscHandle->cswPrimeFlag = 1;
            mscHandle->stallStatus  = 0;
        }
        return error;
    }
    if (mscHandle->transferRemaining >= message->length)
    {
        mscHandle->transferRemaining -= message->length;
    }

    if (mscHandle->needInStallFlag == 1U)
    {
        mscHandle->needInStallFlag     = 0U;
        mscHandle->inEndpointStallFlag = 1U;
        mscHandle->dataInFlag          = 0U;
        (void)USB_DeviceStallEndpoint(mscHandle->handle, mscHandle->bulkInEndpoint);
        return error;
    }
    if ((0U == mscHandle->dataInFlag) && (message->length == USB_DEVICE_MSC_CSW_LENGTH))
    {
        temp = (void *)(message->buffer);
        csw  = (usb_device_msc_csw_t *)temp;
    }

    if (0U != mscHandle->dataInFlag)
    {
        if ((mscHandle->configurationStruct->classCallback != NULL))
        {
            usb_device_lba_app_struct_t lbaData;

            lbaData.size   = message->length;
            lbaData.buffer = message->buffer;
            lbaData.offset = mscHandle->currentOffset;
            lbaData.lun    = mscHandle->mscCbw->logicalUnitNumber;

            if ((USB_DEVICE_MSC_READ_10_COMMAND == mscHandle->mscCbw->cbwcb[0]) ||
                (USB_DEVICE_MSC_READ_12_COMMAND == mscHandle->mscCbw->cbwcb[0]))
            {
                /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
                it is from the second parameter of classInit */
                (void)mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle,
                                                                    kUSB_DeviceMscEventReadResponse, (void *)&lbaData);
            }

            if (0U != mscHandle->transferRemaining)
            {
                mscHandle->currentOffset +=
                    (message->length / mscHandle->luInformations[mscHandle->mscCbw->logicalUnitNumber].lengthOfEachLba);
                error = USB_DeviceMscSend(mscHandle);
            }
            if (0U == mscHandle->transferRemaining)
            {
                mscHandle->dataInFlag = 0U;
                /*data transfer has been done, send the csw to host */
                mscHandle->cswPrimeFlag        = 1U;
                mscHandle->mscCsw->dataResidue = USB_LONG_TO_LITTLE_ENDIAN(mscHandle->mscCsw->dataResidue);
                (void)USB_DeviceSendRequest(mscHandle->handle, mscHandle->bulkInEndpoint, (uint8_t *)mscHandle->mscCsw,
                                            USB_DEVICE_MSC_CSW_LENGTH);
            }
        }
    }
    else if ((message->length == USB_DEVICE_MSC_CSW_LENGTH) && (USB_DEVICE_MSC_DCSWSIGNATURE == csw->signature))
    {
        mscHandle->cbwValidFlag = 1U;
        mscHandle->cswPrimeFlag = 0U;
        (void)USB_DeviceRecvRequest(mscHandle->handle, mscHandle->bulkOutEndpoint, (uint8_t *)mscHandle->mscCbw,
                                    USB_DEVICE_MSC_CBW_LENGTH);
        mscHandle->cbwPrimeFlag = 1U;
    }
    else
    {
        /*no action*/
    }
    return error;
}

/*!
 * @brief Bulk OUT endpoint callback function.
 *
 * This callback function is used to notify upper layer the transfer result of a transfer.
 * This callback pointer is passed when the Bulk OUT pipe initialized.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param message         The result of the Bulk OUT pipe transfer.
 * @param callbackParam  The parameter for this callback. It is same with
 * usb_device_endpoint_callback_struct_t::callbackParam. In the class, the value is the MSC class handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceMscBulkOut(usb_device_handle handle,
                                         usb_device_endpoint_callback_message_struct_t *message,
                                         void *callbackParam)
{
    usb_device_msc_struct_t *mscHandle = (usb_device_msc_struct_t *)callbackParam;
    usb_status_t error                 = kStatus_USB_Success;

    /* endpoint callback length is USB_CANCELLED_TRANSFER_LENGTH (0xFFFFFFFFU) when transfer is canceled */
    if (message->length == USB_CANCELLED_TRANSFER_LENGTH)
    {
        /*this code is called when stack cancel the transfer, app should release the buffer it use*/
        if ((0U != mscHandle->dataOutFlag) && (mscHandle->configurationStruct->classCallback != NULL) &&
            ((USB_DEVICE_MSC_WRITE_10_COMMAND == mscHandle->mscCbw->cbwcb[0]) ||
             (USB_DEVICE_MSC_WRITE_12_COMMAND == mscHandle->mscCbw->cbwcb[0])))
        {
            usb_device_lba_app_struct_t lbaData;

            lbaData.size   = 0;
            lbaData.buffer = message->buffer;
            lbaData.offset = 0;
            lbaData.lun    = mscHandle->mscCbw->logicalUnitNumber;
            /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
            it is from the second parameter of classInit */
            (void)mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle,
                                                                kUSB_DeviceMscEventWriteResponse, (void *)&lbaData);
        }
        if ((mscHandle->cbwPrimeFlag == 0U) && (mscHandle->inEndpointStallFlag == 0U) &&
            (mscHandle->outEndpointStallFlag == 0U))
        {
            /*prime cbw*/
            (void)USB_DeviceRecvRequest(mscHandle->handle, mscHandle->bulkOutEndpoint, (uint8_t *)mscHandle->mscCbw,
                                        USB_DEVICE_MSC_CBW_LENGTH);
            mscHandle->cbwPrimeFlag = 1U;
        }
        return error;
    }

    if (mscHandle->transferRemaining >= message->length)
    {
        mscHandle->transferRemaining -= message->length;
    }

    if ((USB_DEVICE_MSC_MODE_SELECT_10_COMMAND == mscHandle->mscCbw->cbwcb[0]) ||
        (USB_DEVICE_MSC_MODE_SELECT_6_COMMAND == mscHandle->mscCbw->cbwcb[0]))
    {
        if ((mscHandle->configurationStruct->classCallback != NULL))
        {
            /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
            it is from the second parameter of classInit */
            (void)mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle,
                                                                kUSB_DeviceMscEventModeSelectResponse, (void *)NULL);
        }
    }
    if (mscHandle->needOutStallFlag == 1U)
    {
        mscHandle->needOutStallFlag     = 0U;
        mscHandle->outEndpointStallFlag = 1U;
        mscHandle->dataOutFlag          = 0U;
        mscHandle->cbwPrimeFlag         = 0U;
        (void)USB_DeviceStallEndpoint(mscHandle->handle, mscHandle->bulkOutEndpoint);
        return error;
    }

    if (0U != mscHandle->dataOutFlag)
    {
        usb_device_lba_app_struct_t lbaData;

        lbaData.size   = message->length;
        lbaData.buffer = message->buffer;
        lbaData.offset = mscHandle->currentOffset;
        lbaData.lun    = mscHandle->mscCbw->logicalUnitNumber;

        if ((mscHandle->configurationStruct->classCallback != NULL))
        {
            if ((USB_DEVICE_MSC_WRITE_10_COMMAND == mscHandle->mscCbw->cbwcb[0]) ||
                (USB_DEVICE_MSC_WRITE_12_COMMAND == mscHandle->mscCbw->cbwcb[0]))
            {
                /* classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
                   it is from the second parameter of classInit */
                (void)mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle,
                                                                    kUSB_DeviceMscEventWriteResponse, (void *)&lbaData);
            }

            if (0U != mscHandle->transferRemaining)
            {
                mscHandle->currentOffset +=
                    (message->length / mscHandle->luInformations[mscHandle->mscCbw->logicalUnitNumber].lengthOfEachLba);
                error = USB_DeviceMscRecv(mscHandle);
            }
        }

        if (0U == mscHandle->transferRemaining)
        {
            mscHandle->dataOutFlag = 0U;
            {
                mscHandle->mscCsw->dataResidue = USB_LONG_TO_LITTLE_ENDIAN(mscHandle->mscCsw->dataResidue);
                (void)USB_DeviceSendRequest(mscHandle->handle, mscHandle->bulkInEndpoint, (uint8_t *)mscHandle->mscCsw,
                                            USB_DEVICE_MSC_CSW_LENGTH);
                mscHandle->cswPrimeFlag = 1U;
            }
        }
    }
    else if ((0U != mscHandle->cbwValidFlag) && (message->length == USB_DEVICE_MSC_CBW_LENGTH) &&
             (USB_DEVICE_MSC_DCBWSIGNATURE == mscHandle->mscCbw->signature) &&
             (!((0U != (mscHandle->mscCbw->logicalUnitNumber & 0xF0U)) ||
                (0U != (mscHandle->mscCbw->cbLength & 0xE0U)))) &&
             (mscHandle->mscCbw->logicalUnitNumber < (mscHandle->logicalUnitNumber + 1U)) &&
             ((mscHandle->mscCbw->cbLength >= 0x01U) && (mscHandle->mscCbw->cbLength <= 0x10U)))
    {
        mscHandle->cbwPrimeFlag      = 0U;
        mscHandle->transferRemaining = 0U;

        mscHandle->mscCsw->signature   = USB_DEVICE_MSC_DCSWSIGNATURE;
        mscHandle->mscCsw->dataResidue = 0U;
        mscHandle->mscCsw->tag         = mscHandle->mscCbw->tag;

        mscHandle->cbwValidFlag = 0U;

        mscHandle->mscCbw->dataTransferLength = USB_LONG_FROM_LITTLE_ENDIAN(mscHandle->mscCbw->dataTransferLength);

        mscHandle->dataOutFlag = (uint8_t)(((0U == (mscHandle->mscCbw->flags & USB_DEVICE_MSC_CBW_DIRECTION_BIT)) &&
                                            (0U != mscHandle->mscCbw->dataTransferLength)) ?
                                               1U :
                                               0U);

        mscHandle->dataInFlag = (uint8_t)(((0U != (mscHandle->mscCbw->flags & USB_DEVICE_MSC_CBW_DIRECTION_BIT)) &&
                                           (0U != mscHandle->mscCbw->dataTransferLength)) ?
                                              1U :
                                              0U);

        if ((0U != mscHandle->dataInFlag) && (0U != mscHandle->inEndpointStallFlag))
        {
            error = kStatus_USB_Error;
            return error;
        }
        error = USB_DeviceMscProcessUfiCommand(mscHandle);
        if (error == kStatus_USB_InvalidRequest)
        {
            if (mscHandle->dataOutFlag == 1U)
            {
                if (mscHandle->outEndpointStallFlag == 0U)
                {
                    mscHandle->needOutStallFlag = 1U;
                }
                mscHandle->dataOutFlag = 0U;
            }
            else if (mscHandle->dataInFlag == 1U)
            {
                if (mscHandle->inEndpointStallFlag == 0U)
                {
                    mscHandle->needInStallFlag = 1U;
                }
                mscHandle->dataInFlag = 0U;
            }
            else
            {
                /*no action*/
            }
            mscHandle->stallStatus = (uint8_t)USB_DEVICE_MSC_STALL_IN_DATA;
        }

        if (!((0U != mscHandle->dataOutFlag) || ((0U != mscHandle->dataInFlag) || (0U != mscHandle->needInStallFlag))))
        {
            mscHandle->mscCsw->dataResidue = USB_LONG_TO_LITTLE_ENDIAN(mscHandle->mscCsw->dataResidue);
            (void)USB_DeviceSendRequest(mscHandle->handle, mscHandle->bulkInEndpoint, (uint8_t *)mscHandle->mscCsw,
                                        USB_DEVICE_MSC_CSW_LENGTH);
            mscHandle->cswPrimeFlag = 1U;
        }
    }
    else
    {
        (void)USB_DeviceStallEndpoint(mscHandle->handle, mscHandle->bulkOutEndpoint);
        (void)USB_DeviceStallEndpoint(mscHandle->handle, mscHandle->bulkInEndpoint);
        mscHandle->cbwPrimeFlag         = 0U;
        mscHandle->cbwValidFlag         = 0U;
        mscHandle->outEndpointStallFlag = 1U;
        mscHandle->inEndpointStallFlag  = 1U;
        mscHandle->stallStatus          = (uint8_t)USB_DEVICE_MSC_STALL_IN_CBW;
        mscHandle->performResetRecover  = 1;
    }
    return error;
}

/*!
 * @brief Initialize the endpoints of the msc class.
 *
 * This callback function is used to initialize the endpoints of the msc class.
 *
 * @param mscHandle          The device msc class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMscEndpointsInit(usb_device_msc_struct_t *mscHandle)
{
    usb_device_interface_list_t *interfaceList;
    usb_device_interface_struct_t *interface = (usb_device_interface_struct_t *)NULL;
    usb_status_t error                       = kStatus_USB_Error;
    uint32_t count;
    uint32_t index;

    /* Check the configuration is valid or not. */
    if ((0U == mscHandle->configuration) ||
        (mscHandle->configuration > mscHandle->configurationStruct->classInfomation->configurations))
    {
        return error;
    }

    /* Get the interface list of the new configuration. */
    /* Check the interface list is valid or not. */
    if (NULL == mscHandle->configurationStruct->classInfomation->interfaceList)
    {
        return error;
    }
    interfaceList = &mscHandle->configurationStruct->classInfomation->interfaceList[mscHandle->configuration - 1U];

    /* Find interface by using the alternate setting of the interface. */
    for (count = 0; count < interfaceList->count; count++)
    {
        if (USB_DEVICE_CONFIG_MSC_CLASS_CODE == interfaceList->interfaces[count].classCode)
        {
            for (index = 0; index < interfaceList->interfaces[count].count; index++)
            {
                if (interfaceList->interfaces[count].interface[index].alternateSetting == mscHandle->alternate)
                {
                    interface = &interfaceList->interfaces[count].interface[index];
                    break;
                }
            }
            mscHandle->interfaceNumber = interfaceList->interfaces[count].interfaceNumber;
            break;
        }
    }
    if (NULL == interface)
    {
        /* Return error if the interface is not found. */
        return error;
    }

    /* Keep new interface handle. */
    mscHandle->interfaceHandle = interface;
    /* Initialize the endpoints of the new interface. */
    for (count = 0; count < interface->endpointList.count; count++)
    {
        usb_device_endpoint_init_struct_t epInitStruct;
        usb_device_endpoint_callback_struct_t epCallback;
        epInitStruct.zlt             = 0U;
        epInitStruct.interval        = interface->endpointList.endpoint[count].interval;
        epInitStruct.endpointAddress = interface->endpointList.endpoint[count].endpointAddress;
        epInitStruct.maxPacketSize   = interface->endpointList.endpoint[count].maxPacketSize;
        epInitStruct.transferType    = interface->endpointList.endpoint[count].transferType;

        if (USB_IN == ((epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                       USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT))
        {
            mscHandle->bulkInEndpoint = epInitStruct.endpointAddress;
            epCallback.callbackFn     = USB_DeviceMscBulkIn;
        }
        else
        {
            mscHandle->bulkOutEndpoint = epInitStruct.endpointAddress;
            epCallback.callbackFn      = USB_DeviceMscBulkOut;
        }
        epCallback.callbackParam = mscHandle;

        error = USB_DeviceInitEndpoint(mscHandle->handle, &epInitStruct, &epCallback);
    }

    mscHandle->dataOutFlag             = 0U;
    mscHandle->dataInFlag              = 0U;
    mscHandle->outEndpointStallFlag    = 0U;
    mscHandle->inEndpointStallFlag     = 0U;
    mscHandle->needOutStallFlag        = 0U;
    mscHandle->needInStallFlag         = 0U;
    mscHandle->cbwValidFlag            = 1U;
    mscHandle->transferRemaining       = 0U;
    mscHandle->performResetRecover     = 0U;
    mscHandle->performResetDoneFlag    = 0U;
    mscHandle->stallStatus             = 0U;
    mscHandle->inEndpointCswCancelFlag = 0U;
    return error;
}

/*!
 * @brief De-initialize the endpoints of the msc class.
 *
 * This callback function is used to de-initialize the endpoints of the msc class.
 *
 * @param mscHandle          The device msc class handle. It equals the value returned from
 * usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMscEndpointsDeinit(usb_device_msc_struct_t *mscHandle)
{
    usb_status_t status = kStatus_USB_Error;
    uint32_t count;

    if (NULL == mscHandle->interfaceHandle)
    {
        return status;
    }
    /* De-initialize all endpoints of the interface */
    for (count = 0; count < mscHandle->interfaceHandle->endpointList.count; count++)
    {
        status = USB_DeviceDeinitEndpoint(mscHandle->handle,
                                          mscHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress);
    }
    mscHandle->interfaceHandle = NULL;
    return status;
}

/*!
 * @brief Initialize the msc class.
 *
 * This function is used to initialize the msc class.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param config          The class configuration information.
 * @param handle          It is out parameter, is used to return pointer of the msc class handle to the caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMscInit(uint8_t controllerId, usb_device_class_config_struct_t *config, class_handle_t *handle)
{
    usb_device_msc_struct_t *mscHandle;
    usb_status_t error;
    uint32_t implementingDiskDrive   = USB_DEVICE_CONFIG_MSC_IMPLEMENTING_DISK_DRIVE;
    usb_device_msc_ufi_struct_t *ufi = NULL;

    /* Allocate a msc class handle. */
    error = USB_DeviceMscAllocateHandle(&mscHandle);

    if (kStatus_USB_Success != error)
    {
        return error;
    }

    /* Get the device handle according to the controller id. */
    error = USB_DeviceClassGetDeviceHandle(controllerId, &mscHandle->handle);

    if (kStatus_USB_Success != error)
    {
        (void)USB_DeviceMscFreeHandle(mscHandle);
        return error;
    }
    if (NULL == mscHandle->handle)
    {
        (void)USB_DeviceMscFreeHandle(mscHandle);
        return kStatus_USB_InvalidHandle;
    }
    /* Save the configuration of the class. */
    mscHandle->configurationStruct = config;
    /* Clear the configuration value. */
    mscHandle->configuration = 0U;
    mscHandle->alternate     = 0xffU;

    ufi = &mscHandle->mscUfi;

    mscHandle->implementingDiskDrive = implementingDiskDrive;

    ufi->requestSense->validErrorCode          = USB_DEVICE_MSC_UFI_REQ_SENSE_VALID_ERROR_CODE;
    ufi->requestSense->additionalSenseLength   = USB_DEVICE_MSC_UFI_REQ_SENSE_ADDITIONAL_SENSE_LEN;
    ufi->requestSense->senseKey                = USB_DEVICE_MSC_UFI_NO_SENSE;
    ufi->requestSense->additionalSenseCode     = USB_DEVICE_MSC_UFI_NO_SENSE;
    ufi->requestSense->additionalSenseQualifer = USB_DEVICE_MSC_UFI_NO_SENSE;

    mscHandle->cbwPrimeFlag = 0U;
    mscHandle->cswPrimeFlag = 0U;

    *handle = (class_handle_t)mscHandle;
    return error;
}

/*!
 * @brief De-initialize the device msc class.
 *
 * The function de-initializes the device msc class.
 *
 * @param handle The msc class handle got from usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMscDeinit(class_handle_t handle)
{
    usb_device_msc_struct_t *mscHandle;
    usb_status_t status = kStatus_USB_Error;

    mscHandle = (usb_device_msc_struct_t *)handle;

    if (NULL == mscHandle)
    {
        return kStatus_USB_InvalidHandle;
    }
    status = USB_DeviceMscEndpointsDeinit(mscHandle);
    (void)USB_DeviceMscFreeHandle(mscHandle);
    return status;
}

/*!
 * @brief Handle the event passed to the msc class.
 *
 * This function handles the event passed to the msc class.
 *
 * @param handle          The msc class handle, got from the usb_device_class_config_struct_t::classHandle.
 * @param event           The event codes. Please refer to the enumeration usb_device_class_event_t.
 * @param param           The param type is determined by the event code.
 *
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success              Free device handle successfully.
 * @retval kStatus_USB_InvalidParameter     The device handle not be found.
 * @retval kStatus_USB_InvalidRequest       The request is invalid, and the control pipe will be stalled by the caller.
 */
usb_status_t USB_DeviceMscEvent(void *handle, uint32_t event, void *param)
{
    usb_status_t error               = kStatus_USB_Error;
    usb_device_msc_ufi_struct_t *ufi = NULL;
    usb_device_msc_struct_t *mscHandle;
    usb_device_lba_information_struct_t diskInformation;
    uint16_t interfaceAlternate;
    uint8_t *temp8;
    uint8_t alternate;
    uint8_t index;
    usb_device_class_event_t eventCode = (usb_device_class_event_t)event;
    if ((NULL == param) || (NULL == handle))
    {
        return kStatus_USB_InvalidHandle;
    }

    /* Get the msc class handle. */
    mscHandle = (usb_device_msc_struct_t *)handle;
    switch (eventCode)
    {
        case kUSB_DeviceClassEventDeviceReset:
            /* Bus reset, clear the configuration. */
            mscHandle->configuration = 0U;
            /* Get device information. classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
            it is from the second parameter of classInit */
            error = mscHandle->configurationStruct->classCallback(
                (class_handle_t)mscHandle, kUSB_DeviceMscEventGetLbaInformation, (void *)&diskInformation);
            if (0U >= diskInformation.logicalUnitNumberSupported)
            {
                diskInformation.logicalUnitNumberSupported = 0U;
            }
            else
            {
                mscHandle->logicalUnitNumber = (uint8_t)diskInformation.logicalUnitNumberSupported - 1U;
            }
            /*add for misra2.2*/
            if (0U == diskInformation.logicalUnitNumberSupported)
            {
                error = kStatus_USB_InvalidParameter;
                break;
            }
            /*initialize the basic device information*/
            for (index = 0U; (index <= mscHandle->logicalUnitNumber) && (index < USB_DEVICE_MSC_MAX_LUN); ++index)
            {
                if (((0U == diskInformation.logicalUnitInformations[index].lengthOfEachLba) ||
                     (0U == diskInformation.logicalUnitInformations[index].totalLbaNumberSupports)))
                {
                    error = kStatus_USB_Error;
                }
                else
                {
                    mscHandle->luInformations[index] = diskInformation.logicalUnitInformations[index];
                }
            }
            ufi                                        = &mscHandle->mscUfi;
            ufi->requestSense->validErrorCode          = USB_DEVICE_MSC_UFI_REQ_SENSE_VALID_ERROR_CODE;
            ufi->requestSense->additionalSenseLength   = USB_DEVICE_MSC_UFI_REQ_SENSE_ADDITIONAL_SENSE_LEN;
            ufi->requestSense->senseKey                = USB_DEVICE_MSC_UFI_NO_SENSE;
            ufi->requestSense->additionalSenseCode     = USB_DEVICE_MSC_UFI_NO_SENSE;
            ufi->requestSense->additionalSenseQualifer = USB_DEVICE_MSC_UFI_NO_SENSE;

            mscHandle->cbwPrimeFlag = 0U;
            mscHandle->cswPrimeFlag = 0U;
            break;
        case kUSB_DeviceClassEventSetConfiguration:
            /* Get the new configuration. */
            temp8 = ((uint8_t *)param);
            if (NULL == mscHandle->configurationStruct)
            {
                break;
            }
            if (*temp8 == mscHandle->configuration)
            {
                break;
            }

            if (0U != mscHandle->configuration)
            {
                /* De-initialize the endpoints when current configuration is none zero. */
                error = USB_DeviceMscEndpointsDeinit(mscHandle);
            }
            /* Save new configuration. */
            mscHandle->configuration = *temp8;
            /* Clear the alternate setting value. */
            mscHandle->alternate = 0;
            /* Initialize the endpoints of the new current configuration by using the alternate setting 0. */
            error = USB_DeviceMscEndpointsInit(mscHandle);
            if (kStatus_USB_Success == error)
            {
                (void)USB_DeviceRecvRequest(mscHandle->handle, mscHandle->bulkOutEndpoint, (uint8_t *)mscHandle->mscCbw,
                                            USB_DEVICE_MSC_CBW_LENGTH);
                mscHandle->cbwPrimeFlag = 1U;
            }
            break;
        case kUSB_DeviceClassEventSetInterface:

            if (NULL == mscHandle->configurationStruct)
            {
                break;
            }
            /* Get the new alternate setting of the interface */
            interfaceAlternate = *((uint16_t *)param);
            /* Get the alternate setting value */
            alternate = (uint8_t)(interfaceAlternate & 0xFFU);

            /* Whether the interface belongs to the class. */
            if (mscHandle->interfaceNumber != ((uint8_t)(interfaceAlternate >> 8)))
            {
                break;
            }
            /* Only handle new alternate setting. */
            if (alternate == mscHandle->alternate)
            {
                break;
            }
            error = USB_DeviceMscEndpointsDeinit(mscHandle);
            /* Initialize new endpoints */
            error = USB_DeviceMscEndpointsInit(mscHandle);
            if (kStatus_USB_Success == error)
            {
                (void)USB_DeviceRecvRequest(mscHandle->handle, mscHandle->bulkOutEndpoint, (uint8_t *)mscHandle->mscCbw,
                                            USB_DEVICE_MSC_CBW_LENGTH);
                mscHandle->cbwPrimeFlag = 1U;
            }
            mscHandle->alternate = alternate;

            break;
        case kUSB_DeviceClassEventSetEndpointHalt:
            if ((NULL == mscHandle->configurationStruct) || (NULL == mscHandle->interfaceHandle))
            {
                break;
            }
            /* Get the endpoint address */
            temp8 = ((uint8_t *)param);

            if ((mscHandle->inEndpointStallFlag == 0U) && (*temp8 == mscHandle->bulkInEndpoint))
            {
                /* Only stall the endpoint belongs to the class */
                mscHandle->inEndpointStallFlag = 1U;
                mscHandle->cswPrimeFlag        = 0U;
                error                          = USB_DeviceStallEndpoint(mscHandle->handle, *temp8);
            }
            if ((mscHandle->outEndpointStallFlag == 0U) && (*temp8 == mscHandle->bulkOutEndpoint))
            {
                mscHandle->outEndpointStallFlag = 1U;
                mscHandle->cbwPrimeFlag         = 0U;
                error                           = USB_DeviceStallEndpoint(mscHandle->handle, *temp8);
            }

            break;
        case kUSB_DeviceClassEventClearEndpointHalt:
            if ((NULL == mscHandle->configurationStruct) || (NULL == mscHandle->interfaceHandle) ||
                (mscHandle->performResetRecover == 1U))
            {
                break;
            }
            /* Get the endpoint address */
            temp8 = ((uint8_t *)param);
            /* Only un-stall the endpoint belongs to the class , If the endpoint is in stall status ,then
             * un-stall it*/
            if ((mscHandle->inEndpointStallFlag == 1U) && (*temp8 == mscHandle->bulkInEndpoint))
            {
                mscHandle->inEndpointStallFlag = 0;
                mscHandle->cswPrimeFlag        = 0;
                error                          = USB_DeviceUnstallEndpoint(mscHandle->handle, *temp8);
            }
            if ((mscHandle->outEndpointStallFlag == 1U) && (*temp8 == mscHandle->bulkOutEndpoint))
            {
                mscHandle->outEndpointStallFlag = 0U;
                mscHandle->cbwPrimeFlag         = 0U;
                error                           = USB_DeviceUnstallEndpoint(mscHandle->handle, *temp8);
            }
            if (((mscHandle->stallStatus == (uint8_t)USB_DEVICE_MSC_STALL_IN_CSW) ||
                 (mscHandle->stallStatus == (uint8_t)USB_DEVICE_MSC_STALL_IN_DATA)) &&
                (mscHandle->performResetDoneFlag != 1U))
            {
                if (mscHandle->cswPrimeFlag == 1U)
                {
                    /*cancel the transfer , after the cancel to be complete, and then prime csw in bulk in callback,  */
                    mscHandle->inEndpointCswCancelFlag = 1U;
                    (void)USB_DeviceCancel(mscHandle->handle, mscHandle->bulkInEndpoint);
                }
                else
                {
                    /*send csw*/
                    mscHandle->mscCsw->dataResidue = USB_LONG_TO_LITTLE_ENDIAN(mscHandle->mscCsw->dataResidue);
                    (void)USB_DeviceSendRequest(mscHandle->handle, mscHandle->bulkInEndpoint,
                                                (uint8_t *)mscHandle->mscCsw, USB_DEVICE_MSC_CSW_LENGTH);
                    mscHandle->cswPrimeFlag = 1U;
                }
                mscHandle->stallStatus = 0U;
            }
            if ((mscHandle->performResetDoneFlag == 1U) && (mscHandle->inEndpointStallFlag == 0U) &&
                (mscHandle->outEndpointStallFlag == 0U))
            {
                mscHandle->performResetDoneFlag = 0U;
                if (mscHandle->cbwPrimeFlag == 1U)
                {
                    mscHandle->cbwPrimeFlag = 0U;
                    (void)USB_DeviceCancel(mscHandle->handle, mscHandle->bulkOutEndpoint);
                }
                else
                {
                    /*prime cbw for new transfer*/
                    (void)USB_DeviceRecvRequest(mscHandle->handle, mscHandle->bulkOutEndpoint,
                                                (uint8_t *)mscHandle->mscCbw, USB_DEVICE_MSC_CBW_LENGTH);
                    mscHandle->cbwPrimeFlag = 1U;
                }
                mscHandle->stallStatus = 0;
            }
            break;
        case kUSB_DeviceClassEventClassRequest:
        {
            /* Handle the msc class specific request. */
            usb_device_control_request_struct_t *control_request = (usb_device_control_request_struct_t *)param;

            if ((control_request->setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) !=
                USB_REQUEST_TYPE_RECIPIENT_INTERFACE)
            {
                break;
            }

            switch (control_request->setup->bRequest)
            {
                case USB_DEVICE_MSC_GET_MAX_LUN:
                    /*Get Max LUN */
                    if ((control_request->setup->wIndex == mscHandle->interfaceNumber) &&
                        (0U == control_request->setup->wValue) && (control_request->setup->wLength == 0x0001U) &&
                        ((control_request->setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) ==
                         USB_REQUEST_TYPE_DIR_IN))
                    {
                        control_request->buffer = &mscHandle->logicalUnitNumber;
                        control_request->length = (uint32_t)control_request->setup->wLength;
                    }
                    else
                    {
                        error = kStatus_USB_InvalidRequest;
                    }

                    break;
                case USB_DEVICE_MSC_BULK_ONLY_MASS_STORAGE_RESET:
                    /*Bulk-Only Mass Storage Reset (class-specific request)*/
                    if ((control_request->setup->wIndex == mscHandle->interfaceNumber) &&
                        (0U == control_request->setup->wValue) && (0U == control_request->setup->wLength) &&
                        ((control_request->setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) ==
                         USB_REQUEST_TYPE_DIR_OUT))
                    {
                        /*Need to go to stall process, because reset recovery contains reset command and clare
                         * feature command*/
                        mscHandle->outEndpointStallFlag = 1U;
                        mscHandle->inEndpointStallFlag  = 1U;

                        mscHandle->cbwValidFlag         = 1U;
                        mscHandle->performResetRecover  = 0U;
                        mscHandle->performResetDoneFlag = 1U;
                    }
                    else
                    {
                        error = kStatus_USB_InvalidRequest;
                    }

                    break;
                default:
                    /*no action*/
                    break;
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
 * @brief Send data through a specified endpoint.
 *
 * The function is used to send data through a specified endpoint.
 * The function calls USB_DeviceSendRequest internally.
 *
 * @param handle The msc class handle got from usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the sending request is successful or not; the transfer done is notified by
 * USB_DeviceMscBulkIn.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t USB_DeviceMscSend(usb_device_msc_struct_t *mscHandle)
{
    usb_status_t status = kStatus_USB_Success;
    usb_device_lba_app_struct_t lba;

    lba.offset = mscHandle->currentOffset;
    /*bulkInBufferSize is the application buffer size, USB_DEVICE_MSC_MAX_SEND_TRANSFER_LENGTH is the max transfer
       length by the hardware,
        lba.size is the data pending  for transfer ,select the minimum size to transfer ,the remaining will be transfer
       next time*/
    lba.size = (mscHandle->luInformations[mscHandle->mscCbw->logicalUnitNumber].bulkInBufferSize >
                USB_DEVICE_MSC_MAX_SEND_TRANSFER_LENGTH) ?
                   USB_DEVICE_MSC_MAX_SEND_TRANSFER_LENGTH :
                   mscHandle->luInformations[mscHandle->mscCbw->logicalUnitNumber].bulkInBufferSize;
    lba.size =
        (mscHandle->transferRemaining > lba.size) ? lba.size : mscHandle->transferRemaining; /* which one is smaller */

    lba.buffer = NULL;
    lba.lun    = mscHandle->mscCbw->logicalUnitNumber;
    /*classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
    it is from the second parameter of classInit*/
    (void)mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle, kUSB_DeviceMscEventReadRequest,
                                                        &lba);

    if (mscHandle->currentOffset <
        mscHandle->luInformations[mscHandle->mscCbw->logicalUnitNumber].totalLbaNumberSupports)
    {
        status = USB_DeviceSendRequest(mscHandle->handle, mscHandle->bulkInEndpoint, lba.buffer, lba.size);
    }
    else
    {
        mscHandle->needInStallFlag     = 0U;
        mscHandle->inEndpointStallFlag = 1U;
        mscHandle->dataInFlag          = 0U;
        mscHandle->stallStatus         = (uint8_t)USB_DEVICE_MSC_STALL_IN_DATA;
        (void)USB_DeviceStallEndpoint(mscHandle->handle, mscHandle->bulkInEndpoint);
    }
    return status;
}

/*!
 * @brief Receive data through a specified endpoint.
 *
 * The function is used to receive data through a specified endpoint.
 * The function calls USB_DeviceRecvRequest internally.
 *
 * @param handle The msc class handle got from usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the receiving request is successful or not; the transfer done is notified by
 * USB_DeviceMscBulkOut.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t USB_DeviceMscRecv(usb_device_msc_struct_t *mscHandle)
{
    usb_status_t status = kStatus_USB_Success;
    usb_device_lba_app_struct_t lba;

    lba.offset = mscHandle->currentOffset;
    /*bulkOutBufferSize is the application buffer size, USB_DEVICE_MSC_MAX_RECV_TRANSFER_LENGTH is the max transfer
       length by the hardware,
       lba.size is the data pending  for transfer ,select the minimum size to transfer ,the remaining will be transfer
       next time*/
    lba.size = (mscHandle->luInformations[mscHandle->mscCbw->logicalUnitNumber].bulkOutBufferSize >
                USB_DEVICE_MSC_MAX_RECV_TRANSFER_LENGTH) ?
                   USB_DEVICE_MSC_MAX_RECV_TRANSFER_LENGTH :
                   mscHandle->luInformations[mscHandle->mscCbw->logicalUnitNumber].bulkOutBufferSize;
    lba.size =
        (mscHandle->transferRemaining > lba.size) ? lba.size : mscHandle->transferRemaining; /* whichever is smaller */

    lba.buffer = NULL;
    lba.lun    = mscHandle->mscCbw->logicalUnitNumber;
    /*classCallback is initialized in classInit of s_UsbDeviceClassInterfaceMap,
    it is from the second parameter of classInit*/
    (void)mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle, kUSB_DeviceMscEventWriteRequest,
                                                        &lba);

    if (mscHandle->currentOffset <
        mscHandle->luInformations[mscHandle->mscCbw->logicalUnitNumber].totalLbaNumberSupports)
    {
        status = USB_DeviceRecvRequest(mscHandle->handle, mscHandle->bulkOutEndpoint, lba.buffer, lba.size);
    }
    else
    {
        mscHandle->needOutStallFlag     = 0U;
        mscHandle->outEndpointStallFlag = 1U;
        mscHandle->dataOutFlag          = 0U;
        mscHandle->stallStatus          = (uint8_t)USB_DEVICE_MSC_STALL_IN_DATA;
        (void)USB_DeviceStallEndpoint(mscHandle->handle, mscHandle->bulkOutEndpoint);
    }
    return status;
}

/*!
 * @brief Recv Send data through a specified endpoint.
 *
 * The function is used when ufi process read/write command .
 * The function calls USB_DeviceMscRecv or  usb_device_send_recv as the direction internally.
 *
 * @param handle The msc class handle got from usb_device_class_config_struct_t::classHandle.
 * @param direction     Data direction: 0 = Data-Out from host to the device, 1 = Data-In from the device to the host.
 * @param buffer The memory address to hold the data need to be sent.
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the sending or receiving request is successful or not.
 */
usb_status_t USB_DeviceMscLbaTransfer(usb_device_msc_struct_t *mscHandle,
                                      uint8_t direction,
                                      usb_lba_transfer_information_struct_t *lba_info_ptr)
{
    usb_status_t error = kStatus_USB_Success;

    mscHandle->transferRemaining =
        lba_info_ptr->transferNumber * mscHandle->luInformations[mscHandle->mscCbw->logicalUnitNumber].lengthOfEachLba;
    mscHandle->currentOffset = lba_info_ptr->startingLogicalBlockAddress;

    if (direction == USB_IN)
    {
        error = USB_DeviceMscSend(mscHandle);
    }
    else
    {
        error = USB_DeviceMscRecv(mscHandle);
    }
    return error;
}
#endif
