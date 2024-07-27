/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "usb_phydcd.h"
#include "usb_phydcd_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USB_DCD_DATA_PIN_MAX_DETECTION_COUNT (100U)
/*time settting in ms*/
#define USB_DCD_DATA_PIN_DETECTION_TIME (10U)
#define USB_DCD_PRIMIARY_DETECTION_TIME (100U)
#define USB_DCD_SECONDARY_DETECTION_TIME (80U)
typedef enum _usb_phydcd_dev_status
{
    kUSB_DCDDetectInit = 0x0U,
    kUSB_DCDDetectIdle,
    kUSB_DCDDetectStart,
    kUSB_DCDDataContactDetection,
    kUSB_DCDPrimaryDetection,
    kUSB_DCDSecondaryDetection,
    kUSB_DCDDectionFinished,
} usb_phydcd_dev_status_t;

typedef struct _usb_phydcd_state_struct
{
    volatile uint64_t hwTick;           /*!< Current hw tick(ms)*/
    volatile uint64_t startTime;        /*!< start time for delay*/
    USB_ANALOG_Type *usbAnalogBase;     /*!< The base address of the dcd module */
    usb_phydcd_callback_t dcdCallback;  /*!< DCD callback function*/
    void *dcdCallbackParam;             /*!< DCD callback parameter*/
    void *phyBase;                      /*!< dcd phy base address, if no phy control needed, set to NULL*/
    uint8_t dcdDisable;                 /*!< whether enable dcd function or not*/
    uint8_t detectResult;               /*!< dcd detect result*/
    uint8_t index;                      /*!< analog instance index*/
    volatile uint8_t dataPinCheckTimes; /*!< the check times to make sure data pin is contacted*/
    volatile uint8_t dcdDetectState;    /*!< DCD callback parameter*/
} usb_phydcd_state_struct_t;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Apply for device dcd state structure */
static usb_phydcd_state_struct_t s_UsbDeviceDcdHSState[FSL_FEATURE_SOC_USBPHY_COUNT];
/*******************************************************************************
 * Code
 ******************************************************************************/

usb_phydcd_status_t USB_PHYDCD_Init(uint8_t index, usb_phydcd_config_struct_t *config, usb_phydcd_handle *dcdHandle)
{
    usb_phydcd_state_struct_t *dcdState;
    usb_phydcd_status_t dcdError = kStatus_phydcd_Success;
    uint32_t analog_base[]       = USB_ANALOG_BASE_ADDRS;
    USB_ANALOG_Type *base;
    uint32_t *temp;
    base               = (USB_ANALOG_Type *)analog_base[0];
    uint32_t phyBase[] = USBPHY_BASE_ADDRS;
    if ((NULL == config) || (NULL == base) || (NULL == config->dcdCallback))
    {
        return kStatus_phydcd_Error;
    }

    dcdState                   = &s_UsbDeviceDcdHSState[index];
    dcdState->index            = index;
    temp                       = (uint32_t *)phyBase[index + 1U];
    dcdState->phyBase          = (void *)temp;
    dcdState->usbAnalogBase    = base;
    dcdState->dcdCallbackParam = config->dcdCallbackParam;
    dcdState->dcdCallback      = config->dcdCallback;
    dcdState->dcdDisable       = 0U;
    dcdState->dcdDetectState   = (uint8_t)kUSB_DCDDetectInit;
    *dcdHandle                 = dcdState;
    return dcdError;
}
usb_phydcd_status_t USB_PHYDCD_Deinit(usb_phydcd_handle dcdHandle)
{
    usb_phydcd_state_struct_t *dcdState;
    dcdState                     = (usb_phydcd_state_struct_t *)dcdHandle;
    usb_phydcd_status_t dcdError = kStatus_phydcd_Success;

    dcdState->index            = 0U;
    dcdState->phyBase          = NULL;
    dcdState->usbAnalogBase    = NULL;
    dcdState->dcdCallbackParam = NULL;
    dcdState->dcdCallback      = NULL;
    dcdState->dcdDisable       = 0U;
    dcdState->dcdDetectState   = (uint8_t)kUSB_DCDDetectInit;

    return dcdError;
}
usb_phydcd_status_t USB_PHYDCD_Control(usb_phydcd_handle handle, usb_phydcd_control_t type, void *param)
{
    usb_phydcd_state_struct_t *dcdState;
    dcdState                     = (usb_phydcd_state_struct_t *)handle;
    usb_phydcd_status_t dcdError = kStatus_phydcd_Success;
    if (NULL == handle)
    {
        return kStatus_phydcd_Error;
    }
    switch (type)
    {
        case kUSB_DevicePHYDcdRun:
            if (0U == dcdState->dcdDisable)
            {
                dcdState->dcdDetectState = (uint8_t)kUSB_DCDDetectStart;
            }
            break;
        case kUSB_DevicePHYDcdStop:
            if (0U == dcdState->dcdDisable)
            {
                dcdState->dcdDetectState = (uint8_t)kUSB_DCDDetectInit;
            }
            break;
        case kUSB_DevicePHYDcdEnable:
            dcdState->dcdDisable = 0U;
            break;
        case kUSB_DevicePHYDcdDisable:
            dcdState->dcdDisable = 1U;
            break;
        default:
            /*no action*/
            break;
    }
    return dcdError;
}

usb_phydcd_status_t USB_PHYDCD_TimerIsrFunction(usb_phydcd_handle handle)
{
    usb_phydcd_status_t dcdError = kStatus_phydcd_Success;
    usb_phydcd_state_struct_t *dcdState;
    dcdState = (usb_phydcd_state_struct_t *)handle;
    USBPHY_Type *usbPhyBase;
    usb_phydcd_dev_status_t dcdStatus;
    if (NULL == handle)
    {
        return kStatus_phydcd_Error;
    }
    dcdState->hwTick++;

    dcdStatus = (usb_phydcd_dev_status_t)dcdState->dcdDetectState;
    switch (dcdStatus)
    {
        case kUSB_DCDDetectInit:
            break;
        case kUSB_DCDDetectIdle:
            break;
        case kUSB_DCDDetectStart:
            /*Enable the charger detector.*/
            dcdState->dcdDetectState    = (uint8_t)kUSB_DCDDataContactDetection;
            dcdState->detectResult      = (uint8_t)kUSB_DcdUnknownType;
            dcdState->dataPinCheckTimes = 0U;
            dcdState->usbAnalogBase->INSTANCE[dcdState->index].CHRG_DETECT_CLR |= USB_ANALOG_CHRG_DETECT_CLR_EN_B_MASK;
            dcdState->usbAnalogBase->INSTANCE[dcdState->index].CHRG_DETECT_SET |=
                USB_ANALOG_CHRG_DETECT_SET_CHK_CONTACT_MASK | USB_ANALOG_CHRG_DETECT_SET_CHK_CHRG_B_MASK;
            dcdState->startTime = dcdState->hwTick;
            break;
        case kUSB_DCDDataContactDetection:
            if (0U == ((dcdState->hwTick - dcdState->startTime) % USB_DCD_DATA_PIN_DETECTION_TIME))
            {
                if (0U != (dcdState->usbAnalogBase->INSTANCE[dcdState->index].CHRG_DETECT_STAT &
                           USB_ANALOG_CHRG_DETECT_STAT_PLUG_CONTACT_MASK))
                {
                    dcdState->dataPinCheckTimes++;
                    if (dcdState->dataPinCheckTimes >= 5U)
                    {
                        dcdState->dcdDetectState = (uint8_t)kUSB_DCDPrimaryDetection;
                        dcdState->usbAnalogBase->INSTANCE[dcdState->index].CHRG_DETECT_CLR |=
                            USB_ANALOG_CHRG_DETECT_CLR_CHK_CONTACT_MASK | USB_ANALOG_CHRG_DETECT_CLR_CHK_CHRG_B_MASK;
                        dcdState->startTime = dcdState->hwTick;
                    }
                }
                else
                {
                    dcdState->dataPinCheckTimes = 0U;
                }
            }

            if ((dcdState->hwTick - dcdState->startTime) >=
                USB_DCD_DATA_PIN_DETECTION_TIME * USB_DCD_DATA_PIN_MAX_DETECTION_COUNT)
            {
                if (((uint8_t)kUSB_DCDDataContactDetection) == dcdState->dcdDetectState)
                {
                    dcdState->dcdDetectState = (uint8_t)kUSB_DCDDetectInit;
                    dcdState->startTime      = 0U;
                    dcdState->usbAnalogBase->INSTANCE[dcdState->index].CHRG_DETECT_SET |=
                        USB_ANALOG_CHRG_DETECT_SET_EN_B_MASK | USB_ANALOG_CHRG_DETECT_SET_CHK_CHRG_B_MASK;
                    dcdState->detectResult   = (uint8_t)kUSB_DcdError;
                    dcdState->dcdDetectState = (uint8_t)kUSB_DCDDectionFinished;
                }
            }
            break;
        case kUSB_DCDPrimaryDetection:
            if (dcdState->hwTick - dcdState->startTime >= USB_DCD_PRIMIARY_DETECTION_TIME)
            {
                if (0U == (dcdState->usbAnalogBase->INSTANCE[dcdState->index].CHRG_DETECT_STAT &
                           USB_ANALOG_CHRG_DETECT_STAT_CHRG_DETECTED_MASK))
                {
                    dcdState->detectResult   = (uint8_t)kUSB_DcdSDP;
                    dcdState->dcdDetectState = (uint8_t)kUSB_DCDDectionFinished;
                }
                else
                {
                    dcdState->dcdDetectState = (uint8_t)kUSB_DCDSecondaryDetection;
                }
                dcdState->usbAnalogBase->INSTANCE[dcdState->index].CHRG_DETECT_SET |=
                    USB_ANALOG_CHRG_DETECT_SET_EN_B_MASK | USB_ANALOG_CHRG_DETECT_SET_CHK_CHRG_B_MASK;
                if (((uint8_t)kUSB_DCDSecondaryDetection) == dcdState->dcdDetectState)
                {
                    usbPhyBase = (USBPHY_Type *)dcdState->phyBase;
                    usbPhyBase->DEBUG_CLR |= USBPHY_DEBUG_CLR_CLKGATE_MASK;
                    dcdState->usbAnalogBase->INSTANCE[dcdState->index].LOOPBACK_SET |=
                        USB_ANALOG_LOOPBACK_UTMI_TESTSTART_MASK;
                }
                dcdState->startTime = dcdState->hwTick;
            }
            break;
        case kUSB_DCDSecondaryDetection:
            if (dcdState->hwTick - dcdState->startTime >= USB_DCD_SECONDARY_DETECTION_TIME)
            {
                if (0U != (dcdState->usbAnalogBase->INSTANCE[dcdState->index].CHRG_DETECT_STAT &
                           USB_ANALOG_CHRG_DETECT_STAT_DM_STATE_MASK))
                {
                    dcdState->detectResult = (uint8_t)kUSB_DcdDCP;
                }
                else
                {
                    dcdState->detectResult = (uint8_t)kUSB_DcdCDP;
                }
                dcdState->usbAnalogBase->INSTANCE[dcdState->index].LOOPBACK_CLR |=
                    USB_ANALOG_LOOPBACK_UTMI_TESTSTART_MASK;
                usbPhyBase = (USBPHY_Type *)dcdState->phyBase;
                usbPhyBase->DEBUG_SET |= USBPHY_DEBUG_CLR_CLKGATE_MASK;
                dcdState->dcdDetectState = (uint8_t)kUSB_DCDDectionFinished;
            }
            break;
        case kUSB_DCDDectionFinished:
            dcdState->dcdDetectState = (uint8_t)kUSB_DCDDetectIdle;
            (void)dcdState->dcdCallback(dcdState->dcdCallbackParam, dcdState->detectResult,
                                        (void *)&dcdState->detectResult);
            break;
        default:
            /* no action*/
            break;
    }
    return dcdError;
}
