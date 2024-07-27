/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016,2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USB_HOST_HUB_APP_H_
#define _USB_HOST_HUB_APP_H_

#if ((defined USB_HOST_CONFIG_HUB) && (USB_HOST_CONFIG_HUB))

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief HUB reset times*/
#define USB_HOST_HUB_PORT_RESET_TIMES (1)

#if ((defined(USB_HOST_CONFIG_LOW_POWER_MODE)) && (USB_HOST_CONFIG_LOW_POWER_MODE > 0U))
/*! @brief HUB Control tansaction retry times for remote wakeup*/
#define USB_HOST_HUB_REMOTE_WAKEUP_TIMES (3U)
#endif

/*! @brief HUB application global structure */
typedef struct _usb_host_hub_global
{
    usb_host_handle hostHandle;                            /*!< This HUB list belong to this host*/
    usb_host_hub_instance_t *hubProcess;                   /*!< HUB in processing*/
    usb_host_hub_instance_t *hubList;                      /*!< host's HUB list*/
    osa_mutex_handle_t hubMutex;                           /*!< HUB mutex*/
    uint32_t mutexBuffer[(OSA_MUTEX_HANDLE_SIZE + 3) / 4]; /*!< The mutex buffer. */
} usb_host_hub_global_t;

/*! @brief HUB application status */
typedef enum _usb_host_hub_app_status
{
    kHubRunIdle = 0,         /*!< Idle */
    kHubRunInvalid,          /*!< Invalid state */
    kHubRunWaitSetInterface, /*!< Wait callback of set interface */
    kHubRunGetDescriptor7,   /*!< Get 7 bytes HUB descriptor */
    kHubRunGetDescriptor,    /*!< Get all HUB descriptor */
    kHubRunSetPortPower,     /*!< Set HUB's port power */
    kHubRunGetStatusDone,    /*!< HUB status changed */
    kHubRunClearDone,        /*!< clear HUB feature callback */
} usb_host_hub_app_status_t;

/*! @brief HUB port application status */
typedef enum _usb_host_port_app_status
{
    kPortRunIdle = 0,             /*!< Idle */
    kPortRunInvalid,              /*!< Invalid state */
    kPortRunWaitPortChange,       /*!< Wait port status change */
    kPortRunCheckCPortConnection, /*!< Check C_PORT_CONNECTION */
    kPortRunGetPortConnection,    /*!< Get port status data */
    kPortRunCheckPortConnection,  /*!< Check PORT_CONNECTION   */
    kPortRunWaitPortResetDone,    /*!< Wait port reset transfer done */
    kPortRunWaitCPortReset,       /*!< Wait C_PORT_RESET */
    KPortRunCheckCPortReset,      /*!< Check C_PORT_RESET */
    kPortRunResetAgain,           /*!< Reset port again */
    kPortRunPortAttached,         /*!< Device is attached on the port */
    kPortRunCheckPortDetach,      /*!< Check port is detached */
    kPortRunGetConnectionBit,     /*!< Get the port status data */
    kPortRunCheckConnectionBit,   /*!< Check PORT_CONNECTION */
#if ((defined(USB_HOST_CONFIG_LOW_POWER_MODE)) && (USB_HOST_CONFIG_LOW_POWER_MODE > 0U))
    kPortRunClearCPortSuspend, /*!< Clear C_PORT_SUSPEND */
    kPortRunCheckPortSuspend,  /*!< Check PORT_SUSPEND */
    kPortRunPortSuspended,     /*!< Port is suspended */
#endif
} usb_host_port_app_status_t;

/*! @brief HUB data prime status */
typedef enum _usb_host_hub_prime_status
{
    kPrimeNone = 0U,   /*!< Don't prime data*/
    kPrimeHubControl,  /*!< Prime HUB control transfer*/
    kPrimePortControl, /*!< Prime port control transfer*/
    kPrimeInterrupt,   /*!< Prime interrupt transfer*/
} usb_host_hub_prime_status_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if ((defined(USB_HOST_CONFIG_LOW_POWER_MODE)) && (USB_HOST_CONFIG_LOW_POWER_MODE > 0U))
/*!
 * @brief Suspend the device.
 *
 * @param hostHandle  Host instance.
 *
 * @return kStatus_USB_Success or error codes.
 *
 */
usb_status_t USB_HostHubSuspendDevice(usb_host_handle hostHandle);
/*!
 * @brief Resume the device.
 *
 * @param hostHandle  Host instance.
 *
 * @return kStatus_USB_Success or error codes.
 *
 */
usb_status_t USB_HostHubResumeDevice(usb_host_handle hostHandle);
/*!
 * @brief get device's hub total think time.
 *
 * @param parent_hub_no  device's parent hub's address.
 *
 * @return think time value.
 */
#endif
uint32_t USB_HostHubGetTotalThinkTime(usb_host_handle hostHandle, uint8_t parentHubNo);
/*!
 * @brief get device's high-speed hub's address.
 *
 * @param parent_hub_no device's parent hub's address.
 *
 * @return hub number.
 */
uint32_t USB_HostHubGetHsHubNumber(usb_host_handle hostHandle, uint8_t parentHubNo);
/*!
 * @brief remove attached device. called by USB_HostRemoveDevice.
 *
 * @param hubNumber   the device attached hub.
 * @param portNumber  the device attached port.
 *
 * @return kStatus_USB_Success or error codes.
 */
usb_status_t USB_HostHubRemovePort(usb_host_handle hostHandle, uint8_t hubNumber, uint8_t portNumber);
/*!
 * @brief host hub callback function.
 *
 * This function should be called in the host callback function.
 *
 * @param hostHandle             host handle.
 * @param deviceHandle           device handle.
 * @param configurationHandle attached device's configuration descriptor information.
 * @param event_code           callback event code, please reference to enumeration host_event_t.
 *
 * @retval kStatus_USB_Success              The host is initialized successfully.
 * @retval kStatus_USB_NotSupported         The configuration don't contain hub interface.
 */
usb_status_t USB_HostHubDeviceEvent(usb_host_handle hostHandle,
                                    usb_device_handle deviceHandle,
                                    usb_host_configuration_handle configurationHandle,
                                    uint32_t eventCode);
/*!
 * @brief get device's high-speed hub's port number.
 *
 * @param parent_hub_no  device's parent hub's address.
 * @param parent_port_no device's parent port no.
 *
 * @return port number.
 */
uint32_t USB_HostHubGetHsHubPort(usb_host_handle hostHandle, uint8_t parentHubNo, uint8_t parentPortNo);
#endif /* USB_HOST_CONFIG_HUB */

#endif /* _USB_HOST_HUB_APP_H_ */
