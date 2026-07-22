/*
    Copyright (C) 2016 Jonathan Struebel
    Modifications copyright (C) 2020 Alex Lewontin

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
 * @file    usbcfg.c
 * @brief   USB driver config.
 *
 * @addtogroup USB
 * @{
 */
#include "hal.h"
#include "usbcfg.h"

#define VID 0x1209
#define PID 0x0003
/*
 * Endpoints to be used for USBD1.
 */
#define USBD1_DATA_REQUEST_EP           1
#define USBD1_DATA_AVAILABLE_EP         1

/*
 * USB HID Driver structure.
 */
USBHIDDriver UHD1;

/*
 * Data used for feedback
 */
uint8_t increment_var = 0;

/*
 * USB Device Descriptor.
 */
static const uint8_t hid_device_descriptor_data[] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x00,          /* bDeviceClass.                    */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         VID,           /* idVendor.                        */
                         PID,           /* idProduct.                       */
                         0x000,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor hid_device_descriptor = {
  sizeof hid_device_descriptor_data,
  hid_device_descriptor_data
};

/*
 * Configuration Descriptor tree for a HID device
 *
 * The HID Specifications version 1.11 require the following order:
 * - Configuration Descriptor
 * - Interface Descriptor
 * - HID Descriptor
 * - Endpoints Descriptors
 */
#define HID_DESCRIPTOR_OFFSET       18
#define HID_DESCRIPTOR_SIZE         USB_DESC_HID_SIZE

static const uint8_t hid_configuration_descriptor_data[] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(41,            /* wTotalLength.                    */
                         0x01,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x03,          /* bInterfaceClass (HID Interface
                                           Class).                          */
                         0x00,          /* bInterfaceSubClass (None).       */
                         0x00,          /* bInterfaceProtocol (None).       */
                         0),            /* iInterface.                      */
  /* HID Descriptor.*/
  USB_DESC_HID          (0x0110,        /* bcdHID.                          */
                         0x00,          /* bCountryCode.                    */
                         0x01,          /* bNumDescriptors.                 */
                         0x22,          /* bDescriptorType (Report
                                           Descriptor).                     */
                         34),           /* wDescriptorLength.               */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_AVAILABLE_EP,       /* bEndpointAddress.*/
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x0A),         /* bInterval (10ms).                */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_REQUEST_EP|0x80,    /* bEndpointAddress.*/
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x0A)          /* bInterval (10ms).                */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor hid_configuration_descriptor = {
  sizeof hid_configuration_descriptor_data,
  hid_configuration_descriptor_data
};

/*
 * HID Descriptor wrapper.
 */
static const USBDescriptor hid_descriptor = {
  HID_DESCRIPTOR_SIZE,
  &hid_configuration_descriptor_data[HID_DESCRIPTOR_OFFSET]
};

/*
 * HID Report Descriptor
 *
 * This is the description of the format and the content of the
 * different IN or/and OUT reports that your application can
 * receive/send
 *
 * See "Device Class Definition for Human Interface Devices (HID)"
 * (http://www.usb.org/developers/hidpage/HID1_11.pdf) for the
 * detailed description of all the fields
 */
static const uint8_t hid_report_descriptor_data[] = {
  USB_DESC_BYTE (0x06),                 /* Usage Page -                     */
  USB_DESC_WORD (0xFF00),               /*   Vendor Defined.                */
  USB_DESC_BYTE (0x09),                 /* Usage -                          */
  USB_DESC_BYTE (0x01),                 /*   Vendor Defined.                */
  USB_DESC_BYTE (0xA1),                 /* Collection -                     */
  USB_DESC_BYTE (0x01),                 /*   Application.                   */

  USB_DESC_BYTE (0x09),                 /* Usage -                          */
  USB_DESC_BYTE (0x01),                 /*   Vendor Defined.                */
  USB_DESC_BYTE (0x15),                 /* Logical Minimum -                */
  USB_DESC_BYTE (0x00),                 /*   0.                             */
  USB_DESC_BYTE (0x26),                 /* Logical Maximum -                */
  USB_DESC_WORD (0x00FF),               /*   255.                           */
  USB_DESC_BYTE (0x75),                 /* Report size -                    */
  USB_DESC_BYTE (0x08),                 /*   8 bits.                        */
  USB_DESC_BYTE (0x95),                 /* Report count -                   */
  USB_DESC_BYTE (0x01),                 /*   1.                             */
  USB_DESC_BYTE (0x81),                 /* Input -                          */
  USB_DESC_BYTE (0x02),                 /*   Data, Variable, Absolute.      */

  USB_DESC_BYTE (0x09),                 /* Usage -                          */
  USB_DESC_BYTE (0x01),                 /*   Vendor Defined.                */
  USB_DESC_BYTE (0x15),                 /* Logical Minimum -                */
  USB_DESC_BYTE (0x00),                 /*   0.                             */
  USB_DESC_BYTE (0x26),                 /* Logical Maximum -                */
  USB_DESC_WORD (0x00FF),               /*   255.                           */
  USB_DESC_BYTE (0x75),                 /* Report Size -                    */
  USB_DESC_BYTE (0x08),                 /*   8 bits.                        */
  USB_DESC_BYTE (0x95),                 /* Report Count -                   */
  USB_DESC_BYTE (0x01),                 /*   1.                             */
  USB_DESC_BYTE (0x91),                 /* Output -                         */
  USB_DESC_BYTE (0x02),                 /*   Data, Variable, Absolute.      */

  USB_DESC_BYTE (0xC0)                  /* End Collection.                  */
};

/*
 * HID Report Descriptor wrapper
 */
static const USBDescriptor hid_report_descriptor = {
  sizeof hid_report_descriptor_data,
  hid_report_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t usb_string_langid[] =
    USB_DESC_STRING(USB_DESC_WORD(0x0409)); /* wLANGID (U.S. English) */

/*
 * Vendor string.
 */
static const uint8_t usb_string_vendor[] =
    USB_DESC_STRING('N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0);

/*
 * Serial Number string.
 */
static const uint8_t usb_string_serial[] =
    USB_DESC_STRING('0', 0, 'x', 0, 'D', 0, 'E', 0, 'A', 0, 'D', 0, 'B', 0, 'E', 0, 'E', 0, 'F', 0);

/*
 * Device Description string.
 */
static const uint8_t usb_string_description[] =
    USB_DESC_STRING('C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0, 'H', 0, 'A', 0,
                    'L', 0, ' ', 0, 'U', 0, 'S', 0, 'B', 0, ' ', 0, 'D', 0, 'e', 0, 'm', 0, 'o', 0);
/*
 * Strings wrappers array.
 */
static const USBDescriptor hid_strings[] = {
  {sizeof usb_string_langid, usb_string_langid},
  {sizeof usb_string_vendor, usb_string_vendor},
  {sizeof usb_string_description, usb_string_description},
  {sizeof usb_string_serial, usb_string_serial}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {
  (void)usbp;
  (void)lang;
  switch (dtype) {
    case USB_DESCRIPTOR_DEVICE:
      return &hid_device_descriptor;
    case USB_DESCRIPTOR_CONFIGURATION:
      return &hid_configuration_descriptor;
    case USB_DESCRIPTOR_STRING:
      if (dindex < 4)
        return &hid_strings[dindex];
    case USB_DESCRIPTOR_INTERFACE:
      break;
    case USB_DESCRIPTOR_ENDPOINT:
      break;
    case USB_DESCRIPTOR_HID:
      return &hid_descriptor;
    case HID_REPORT:
      return &hid_report_descriptor;
    default:
      break;
  }
  return NULL;
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   OUT EP1 state.
 */
static USBOutEndpointState ep1outstate;

/**
 * @brief   EP1 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  hidDataTransmitted,
  hidDataReceived,
  0x0040,
  0x0040,
  &ep1instate,
  &ep1outstate,
  0,
  NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
  switch (event) {
  case USB_EVENT_RESET:
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    osalSysLockFromISR();

    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);

    /* Resetting the state of the CDC subsystem.*/
    hidConfigureHookI(&UHD1);

    osalSysUnlockFromISR();
    return;
  case USB_EVENT_UNCONFIGURED:
    return;
  case USB_EVENT_SUSPEND:
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

static bool req_handler(USBDriver *usbp) {
  size_t n;

  if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS) {
    switch (usbp->setup[1]) {
    case HID_GET_REPORT:
      n = hidGetReport(0, &increment_var, sizeof(increment_var));
      usbSetupTransfer(usbp, &increment_var, n, NULL);
      return true;
    default:
      return hidRequestsHook(usbp);
    }
  }
  return hidRequestsHook(usbp);
}

/**
 * @brief   Generate HID Report
 * @details This function generates the data for an HID report so
 *          that it can be transferred to the host.
 *
 * @param[in]  id       report ID
 * @param[out] bp       data buffer pointer
 * @param[in]  n        the maximum number of bytes for data buffer
 * @return              number of bytes of report in data buffer
 */
size_t hidGetReport(uint8_t id, uint8_t *bp, size_t n) {

  (void) id;
  (void) n;

  increment_var++;
  *bp = increment_var;
  return sizeof(increment_var);
}

/**
 * @brief   Set HID Report
 * @details This function sets the data for an HID report
 *          that was transferred from the host.
 *
 * @param[in]  id       report ID
 * @param[in]  bp       data buffer pointer
 * @param[in]  n        the number of bytes in data buffer
 * @return              The operation status.
 * @retval MSG_OK       if the report was set.
 */
msg_t hidSetReport(uint8_t id, uint8_t *bp, size_t n) {

  (void) id;
  (void) n;

  increment_var = *bp;
  return MSG_OK;
}

/*
 * USB driver configuration.
 */
const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  req_handler,
  NULL
};

/*
 * USB HID driver configuration.
 */
const USBHIDConfig usbhidcfg = {
  &USBD1,
  USBD1_DATA_REQUEST_EP,
  USBD1_DATA_AVAILABLE_EP
};

/** @} */
