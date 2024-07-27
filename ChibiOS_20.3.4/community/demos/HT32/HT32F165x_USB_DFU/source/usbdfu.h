#pragma once

#include "stdint.h"
#include "hal.h"
#include "dfu_target.h"

extern const USBConfig usbcfg;

#define FW_BUFFER_SIZE 256

enum dfu_req {
    DFU_DETACH,
    DFU_DNLOAD,
    DFU_UPLOAD,
    DFU_GETSTATUS,
    DFU_CLRSTATUS,
    DFU_GETSTATE,
    DFU_ABORT,
};

enum dfu_state {
    STATE_APP_IDLE,
    STATE_APP_DETACH,
    STATE_DFU_IDLE,
    STATE_DFU_DNLOAD_SYNC,
    STATE_DFU_DNBUSY,
    STATE_DFU_DNLOAD_IDLE,
    STATE_DFU_MANIFEST_SYNC,
    STATE_DFU_MANIFEST,
    STATE_DFU_MANIFEST_WAIT_RESET,
    STATE_DFU_UPLOAD_IDLE,
    STATE_DFU_ERROR,
};


enum dfu_status {
    DFU_STATUS_OK,
    DFU_STATUS_ERR_TARGET,
    DFU_STATUS_ERR_FILE,
    DFU_STATUS_ERR_WRITE,
    DFU_STATUS_ERR_ERASE,
    DFU_STATUS_ERR_CHECK_ERASED,
    DFU_STATUS_ERR_PROG,
    DFU_STATUS_ERR_VERIFY,
    DFU_STATUS_ERR_ADDRESS,
    DFU_STATUS_ERR_NOTDONE,
    DFU_STATUS_ERR_FIRMWARE,
    DFU_STATUS_ERR_VENDOR,
    DFU_STATUS_ERR_USBR,
    DFU_STATUS_ERR_POR,
    DFU_STATUS_ERR_UNKNOWN,
    DFU_STATUS_ERR_STALLEDPKT,
};

struct usb_setup {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__((packed));