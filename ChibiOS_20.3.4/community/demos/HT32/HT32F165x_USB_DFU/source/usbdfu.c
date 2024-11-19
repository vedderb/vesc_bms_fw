#include "hal.h"
#include "usbdfu.h"
#include "dfu_target.h"

uint8_t fw_buffer[FW_BUFFER_SIZE];
/*
 * USB Device Descriptor.
 */
static const uint8_t dfu_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0200,        /* bcdUSB (1.1).                    */
                         0x00,          /* bDeviceClass                     */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         64,            /* bMaxPacketSize.                  */
                         0x04d9,        /* idVendor (ST).                   */
                         0xf00d,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

static const USBDescriptor dfu_device_descriptor = {
  sizeof dfu_device_descriptor_data,
  dfu_device_descriptor_data
};


/* Configuration Descriptor tree for a DFU.*/
static const uint8_t dfu_configuration_descriptor_data[27] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(27,            /* wTotalLength.                    */
                         0x01,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x00,          /* bNumEndpoints.                   */
                         0xFE,          /* bInterfaceClass (DFU Class
                                                                            */
                         0x01,          /* bInterfaceSubClass               */
                         0x02,          /* bInterfaceProtocol               */
                         2),            /* iInterface.                      */
  /* DFU Class Descriptor.*/
  USB_DESC_BYTE         (9),            /* bLength.                         */
  USB_DESC_BYTE         (0x21),         /* bDescriptorType (DFU_FCUNTION).  */
  USB_DESC_BYTE         (0b1011),       /* bmAttributes (DETACH | DOWNLOAD) */
  USB_DESC_WORD         (  1000),       /* DetachTimeout.                   */
  USB_DESC_WORD         (  64),
  USB_DESC_BCD          (0x0110)
};

static const USBDescriptor dfu_configuration_descriptor = {
  sizeof dfu_configuration_descriptor_data,
  dfu_configuration_descriptor_data
};

static const uint8_t dfu_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

static const uint8_t dfu_string1[] = {
  USB_DESC_BYTE(14),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'H', 0, 'o', 0, 'l', 0, 't', 0, 'e', 0, 'k', 0
};

/*
 * Device Description string.
 */
static const uint8_t dfu_string2[] = {
  USB_DESC_BYTE(16),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'U', 0, 'S', 0, 'B', 0, ' ', 0 , 'D', 0, 'F', 0, 'U', 0
};

/*
 * Serial Number string.
 */
static const uint8_t dfu_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '1', 0, '.', 0, '1', 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor dfu_strings[] = {
  {sizeof dfu_string0, dfu_string0},
  {sizeof dfu_string1, dfu_string1},
  {sizeof dfu_string2, dfu_string2},
  {sizeof dfu_string3, dfu_string3}
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
    return &dfu_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &dfu_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &dfu_strings[dindex];
  }
  return NULL;
}

volatile enum dfu_state currentState = STATE_DFU_IDLE;
volatile enum dfu_status currentStatus = DFU_STATUS_OK;
size_t current_dfu_offset = 0;
size_t dfu_download_size = 0;

static void dfu_on_download_request(USBDriver *usbp) {
  (void)usbp;
  uint8_t *dest = (uint8_t*)(APP_BASE + current_dfu_offset);

  bool ok = true;
  target_flash_unlock();
  if (current_dfu_offset == 0) {
    ok = target_prepare_flash();
  }
  if (ok) {
    ok = target_flash_write(dest, fw_buffer, dfu_download_size);
  }
  target_flash_lock();

  if (ok) {
    current_dfu_offset += dfu_download_size;
    currentState = STATE_DFU_DNLOAD_IDLE;
  } else {
    currentState = STATE_DFU_ERROR;
    currentStatus = DFU_STATUS_ERR_VERIFY;
  }
}

static void dfu_on_download_complete(USBDriver *usbp) {
  (void)usbp;
  currentState = STATE_DFU_MANIFEST_SYNC;
}

static void dfu_on_manifest_request(USBDriver *usbp) {
  (void)usbp;
  target_complete_programming();
  currentState = STATE_DFU_MANIFEST_WAIT_RESET;
}

static void dfu_on_detach_complete(USBDriver *usbp) {
  (void)usbp;
  __asm__ __volatile__("dsb");
  SCB->AIRCR = 0x05FA0004; // System Reset
  __asm__ __volatile__("dsb");
  while(1){};
}

static inline void dfu_status_req(USBDriver *usbp) {
  static uint8_t status_response_buffer[6] = {};
  uint32_t pollTime = 10;
  usbcallback_t cb = NULL;

  switch(currentState) {
    case STATE_DFU_DNLOAD_SYNC: {
      currentState = STATE_DFU_DNBUSY;
      pollTime = target_get_timeout();
      cb = &dfu_on_download_request;
      break;
    }
    case STATE_DFU_MANIFEST_SYNC: {
      currentState = STATE_DFU_MANIFEST;
      cb = &dfu_on_manifest_request;
      break;
    }
    case STATE_DFU_MANIFEST_WAIT_RESET: {
      cb = &dfu_on_detach_complete;
      break;
    }

    default:
      break;
  }

  // Response Construction
  status_response_buffer[0] = (uint8_t) currentStatus;
  status_response_buffer[1] = (uint8_t) (pollTime & 0xFFU);
  status_response_buffer[2] = (uint8_t) ((pollTime >> 8U) & 0xFFU);
  status_response_buffer[3] = (uint8_t) ((pollTime >> 16U) & 0xFFU);
  status_response_buffer[4] = (uint8_t) currentState;
  status_response_buffer[5] = 0; // No Index

  usbSetupTransfer(usbp, status_response_buffer, 6, cb);
}

static bool request_handler(USBDriver *usbp) {
    struct usb_setup* setup = (struct usb_setup*) usbp->setup;
    if((setup->bmRequestType & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS) {
      switch (setup->bRequest) {
        case DFU_GETSTATUS: {
          dfu_status_req(usbp);
          return true;
        }
        case DFU_GETSTATE: {
            usbSetupTransfer(usbp, (uint8_t *)&currentState, 1, NULL);
            return true;
        }
        case DFU_UPLOAD: {
          switch (currentState) {
            case STATE_DFU_IDLE: {
              current_dfu_offset = 0;
              __attribute__((fallthrough));
            }
            case STATE_DFU_UPLOAD_IDLE: {
              uint16_t copy_len = setup->wLength;
              size_t fw_size = target_get_max_fw_size();
              if (current_dfu_offset + setup->wLength > fw_size) {
                copy_len = fw_size - current_dfu_offset;
                currentState = STATE_DFU_IDLE;
              } else {
                currentState = STATE_DFU_UPLOAD_IDLE;
              }
              usbSetupTransfer(usbp, (uint8_t *)(APP_BASE + current_dfu_offset), copy_len, NULL );
              current_dfu_offset += copy_len;
              break;
            }
            default:
              usbSetupTransfer(usbp, NULL, 0, NULL);
              break;
          }
          return true;
        }
        case DFU_CLRSTATUS: {
          currentStatus = DFU_STATUS_OK;
          if (currentState != STATE_DFU_ERROR) {
            usbSetupTransfer(usbp, NULL, 0, NULL);
            break;
          }
          __attribute__((fallthrough));
        }
        case DFU_ABORT: {
          currentState = STATE_DFU_IDLE;
          usbSetupTransfer(usbp, NULL, 0, NULL);
          return true;
        }
        case DFU_DETACH: {
          usbSetupTransfer(usbp, NULL, 0, &dfu_on_detach_complete);
          return true;
        }
        case DFU_DNLOAD: {
          switch (currentState) {
            case STATE_DFU_IDLE: {
              current_dfu_offset = 0;
              dfu_download_size = setup->wLength;
              currentState = STATE_DFU_DNLOAD_SYNC;
              usbSetupTransfer(usbp, &fw_buffer[0], dfu_download_size, NULL);
              break;
            }
            case STATE_DFU_DNLOAD_IDLE: {
              if (setup->wLength > 0) {
                dfu_download_size = setup->wLength;
                usbSetupTransfer(usbp, &fw_buffer[0], dfu_download_size, NULL);
                currentState = STATE_DFU_DNLOAD_SYNC;
              } else {
                usbSetupTransfer(usbp, NULL, 0, &dfu_on_download_complete);
              }
              break;
            }
            default: {
              usbSetupTransfer(usbp, NULL, 0, NULL);
              break;
            }
          }
          return true;
        }
      }
    }
    return false;
}

/*
 * USB driver configuration.
 */
const USBConfig usbcfg = {
  NULL,
  get_descriptor,
  request_handler,
  NULL
};
