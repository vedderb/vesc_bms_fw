/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @file    hal_usb_lld.c
 * @brief   PLATFORM USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>

#include "hal.h"

#include "hal_usb_lld.h"

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

#include "mcuconf.h"

#ifdef USB_DEBUG

#include "chmtx.h"
#include "rp_fifo.h"

mutex_t cmtx;

#define CMD_RESET 0x0F
#define CMD_SETUP 0x01
#define CMD_READ_SETUP 0x02
#define CMD_SET_ADDR 0x03
#define CMD_START_IN 0x04
#define CMD_START_OUT 0x05
#define CMD_BUFF_STATUS 0x06
#define CMD_EP_DONE 0x07
#define CMD_DATA_ERROR 0x09
#define CMD_PREP_IN_EP  0x0A
#define CMD_PREP_OUT_EP 0x0B
#define CMD_SET_ADDR_HW 0x0C
#define CMD_EP_NEXT 0x0D

void cmd_send(uint8_t cmd, uint8_t length) {
  uint32_t data = (cmd << 24) | (length << 16);
  fifoBlockingWrite(data);
}

void data_send(uint32_t data) {
  fifoBlockingWrite(data);
}
#endif // USB_DEBUG

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Get endpoint control register.
 */
#define EP_CTRL(ep)       (USB_DPSRAM->EPCTRL[ep - 1])
/**
 * @brief   Get buffer control register for endpoint.
 */
#define BUF_CTRL(ep)      (USB_DPSRAM->BUFCTRL[ep])

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if (RP_USB_USE_USBD0 == TRUE) || defined(__DOXYGEN__)
USBDriver USBD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static struct {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_EP_MODE_TYPE_CTRL,
  _usb_ep0setup,
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out,
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Buffer mode for isochronous in buffer control register.
 */
static uint16_t usb_isochronous_buffer_mode(uint16_t size) {
  switch (size) {
    case 128:
      return 0;
    case 256:
      return 1;
    case 512:
      return 2;
    case 1024:
      return 3;
    default:
      return 0;
  }
}

/**
 * @brief   Calculate isochronous buffer size in valid step.
 * @details Valid buffer size is one of 128, 256, 512 or 1024.
 */
static uint16_t usb_isochronous_buffer_size(uint16_t max_size) {
  uint16_t size;

  /* Double buffer offset must be one of 128, 256, 512 or 1024. */
  size = ((max_size - 1) / 128 + 1) * 128;
  if (size == 384) {
    size = 512;
  } else if (size == 640 || size == 768 || size > 1024) {
    size = 1024;
  }
  return size;
}

/**
 * @brief   Calculate next offset for buffer data, 64 bytes aligned.
 */
static uint16_t usb_buffer_next_offset(USBDriver *usbp, uint16_t size, bool is_double) {
  uint32_t offset;

  offset = usbp->noffset;
  usbp->noffset += is_double ? size * 2 : size;

  return offset;
}

/**
 * @brief   Reset endpoint 0.
 */
static void reset_ep0(USBDriver *usbp) {
  usbp->epc[0]->in_state->next_pid = 1U;
  usbp->epc[0]->in_state->active = false;
  usbp->epc[0]->in_state->stalled = false;
}

#if 0
/**
 * @brief   Reset specified endpoint.
 */
static void reset_endpoint(USBDriver *usbp, usbep_t ep, bool is_in) {
  const USBEndpointConfig *epcp = usbp->epc[ep];

  if (is_in) {
    USBInEndpointState *in_state = epcp->in_state;
    if (in_state) {
      in_state->active = false;
      in_state->stalled = false;
      in_state->next_pid = 0U;
    }
  } else {
    USBOutEndpointState *out_state = epcp->out_state;
    if (out_state) {
      out_state->active = false;
      out_state->stalled = false;
      out_state->next_pid = 0U;
    }
  }
}
#endif

/**
 * @brief   Prepare buffer for receiving data.
 */
uint32_t usb_prepare_out_ep_buffer(USBDriver *usbp, usbep_t ep, uint8_t buffer_index) {
  uint32_t buf_ctrl = 0;
  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBOutEndpointState *oesp = usbp->epc[ep]->out_state;

    /* PID */
    buf_ctrl |= oesp->next_pid ? USB_BUFFER_BUFFER0_DATA_PID : 0;
    oesp->next_pid ^= 1U;

    buf_ctrl |= USB_BUFFER_BUFFER0_AVAILABLE | epcp->out_maxsize;

    oesp->active = true;
    if (buffer_index) {
      buf_ctrl = buf_ctrl << 16;
    }

  return buf_ctrl;
}

/**
 * @brief   Prepare for receiving data from host.
 */
static void usb_prepare_out_ep(USBDriver *usbp, usbep_t ep) {
  uint32_t buf_ctrl;
  uint32_t ep_ctrl;

  if (ep == 0) {
    ep_ctrl = USB->SIECTRL;
  } else {
    ep_ctrl = EP_CTRL(ep).OUT;
  }

  /* Fill first buffer */
  buf_ctrl = usb_prepare_out_ep_buffer(usbp, ep, 0);

  /* To avoid short packet, we use single buffered here. */
  /* Single buffered */
  ep_ctrl &= ~(USB_EP_BUFFER_DOUBLE | USB_EP_BUFFER_IRQ_DOUBLE_EN);
  ep_ctrl |= USB_EP_BUFFER_IRQ_EN;

  if (ep == 0) {
    USB->SIECTRL = ep_ctrl;
  } else {
    EP_CTRL(ep).OUT = ep_ctrl;
  }

  BUF_CTRL(ep).OUT = buf_ctrl;

#ifdef USB_DEBUG
  cmd_send(CMD_PREP_OUT_EP, 2);
  data_send(ep_ctrl);
  data_send(buf_ctrl);
#endif
}

/**
 * @brief   Prepare buffer for sending data.
 */
static uint32_t usb_prepare_in_ep_buffer(USBDriver *usbp, usbep_t ep, uint8_t buffer_index) {
  uint8_t *buff;
  uint16_t buf_len;
  uint32_t buf_ctrl = 0;
  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBInEndpointState *iesp = usbp->epc[ep]->in_state;

  /* txsize - txlast gives size of data to be sent but not yet in the buffer */
  buf_len = epcp->in_maxsize < iesp->txsize - iesp->txlast ?
            epcp->in_maxsize : iesp->txsize - iesp->txlast;

    iesp->txlast += buf_len;

    /* Host only? */
    //if (iesp->txsize <= iesp->txlast) {
      /* Last buffer */
      //buf_ctrl |= USB_BUFFER_BUFFER0_LAST;
    //}
    /* PID */
    buf_ctrl |= iesp->next_pid ? USB_BUFFER_BUFFER0_DATA_PID : 0;
    iesp->next_pid ^= 1U;

    /* Copy data into hardware buffer */
    buff = (uint8_t*)iesp->hw_buf + (buffer_index == 0 ? 0 : iesp->buf_size);
    memcpy((void *)buff, (void *)iesp->txbuf, buf_len);
    iesp->txbuf += buf_len;

    buf_ctrl |= USB_BUFFER_BUFFER0_FULL |
                USB_BUFFER_BUFFER0_AVAILABLE |
                buf_len;

    iesp->active = true;
    if (buffer_index) {
      buf_ctrl = buf_ctrl << 16;
    }

  return buf_ctrl;
}

/**
 * @brief   Prepare endpoint for sending data.
 */
static void usb_prepare_in_ep(USBDriver *usbp, usbep_t ep) {
  uint32_t buf_ctrl;
  uint32_t ep_ctrl;
  USBInEndpointState *iesp = usbp->epc[ep]->in_state;

  if (ep == 0) {
    ep_ctrl = USB->SIECTRL;
  } else {
    ep_ctrl = EP_CTRL(ep).IN;
  }

  /* Fill first buffer */
  buf_ctrl = usb_prepare_in_ep_buffer(usbp, ep, 0);

  /* Second buffer if required */
  /* iesp->txsize - iesp->txlast gives size even not in buffer */
  if (iesp->txsize - iesp->txlast > 0) {
    buf_ctrl |= usb_prepare_in_ep_buffer(usbp, ep, 1);
  }

  if (buf_ctrl & USB_BUFFER_BUFFER1_AVAILABLE) {
    /* Double buffered */
    ep_ctrl &= ~USB_EP_BUFFER_IRQ_EN;
    ep_ctrl |= USB_EP_BUFFER_DOUBLE | USB_EP_BUFFER_IRQ_DOUBLE_EN;
  } else {
    /* Single buffered */
    ep_ctrl &= ~(USB_EP_BUFFER_DOUBLE | USB_EP_BUFFER_IRQ_DOUBLE_EN);
    ep_ctrl |= USB_EP_BUFFER_IRQ_EN;
  }

  if (ep == 0) {
    USB->SIECTRL = ep_ctrl;
  } else {
    EP_CTRL(ep).IN = ep_ctrl;
  }

  BUF_CTRL(ep).IN = buf_ctrl;

#ifdef USB_DEBUG
  cmd_send(CMD_PREP_IN_EP, 2);
  data_send(ep_ctrl);
  data_send(buf_ctrl);
#endif
}

/**
 * @brief   Work on an endpoint after transfer.
 */
static void usb_serve_endpoint(USBDriver *usbp, usbep_t ep, bool is_in) {
  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBOutEndpointState *oesp;
  USBInEndpointState *iesp;
  uint16_t n;

  if (is_in) {
    /* IN endpoint */
    iesp = usbp->epc[ep]->in_state;

    /* txlast is size sent + size of last buffer */
    iesp->txcnt = iesp->txlast;
    n = iesp->txsize - iesp->txcnt;
    if (n > 0) {
#ifdef USB_DEBUG
      cmd_send(CMD_EP_NEXT, 1);
      data_send((1 << 7) | ep);
#endif
      /* Transfer not completed, there are more packets to send. */
      usb_prepare_in_ep(usbp, ep);
    } else {
#ifdef USB_DEBUG
      cmd_send(CMD_EP_DONE, 1);
      data_send((1 << 7) | ep);
#endif
      /* Transfer complete */
      _usb_isr_invoke_in_cb(usbp, ep);

      iesp->active = iesp->stalled = false;
    }
  } else {
    /* OUT endpoint */
    oesp = usbp->epc[ep]->out_state;

    /* Length received */
    n = BUF_CTRL(ep).OUT & USB_BUFFER_BUFFER0_TRANS_LENGTH_Msk;

    /* Copy received data into user buffer */
    memcpy((void *)oesp->rxbuf, (void *)oesp->hw_buf, n);
    oesp->rxbuf += n;
    oesp->rxcnt += n;
    oesp->rxsize -= n;

    oesp->rxpkts -= 1;

    /* Short packet or all packetes have been received. */
    if (oesp->rxpkts <= 0 || n <= epcp->out_maxsize) {
#ifdef USB_DEBUG
      cmd_send(CMD_EP_DONE, 1);
      data_send(ep);
#endif
      /* Transifer complete */
      _usb_isr_invoke_out_cb(usbp, ep);
      usb_prepare_out_ep(usbp, ep);

      oesp->active = oesp->stalled = false;
    } else {
      /* Receive remained data */
      usb_prepare_out_ep(usbp, ep);
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

#if RP_USB_USE_USBD0 || defined(__DOXYGEN__)

/**
 * @brief   USB low priority interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(RP_USBCTRL_IRQ_HANDLER) {
  uint32_t ints;
  USBDriver *usbp = &USBD1;

  OSAL_IRQ_PROLOGUE();

  ints = USB->INTS;

  /* USB setup packet handling. */
  if (ints & USB_INTS_SETUP_REQ) {
#ifdef USB_DEBUG
    //cmd_send(CMD_SETUP, 0);
#endif
    USB->CLR.SIESTATUS = USB_SIE_STATUS_SETUP_REC;

    usbp->epc[0]->in_state->next_pid = 1U;

    _usb_isr_invoke_setup_cb(usbp, 0);

    reset_ep0(usbp);
  }

  /* USB bus reset condition handling. */
  if (ints & USB_INTS_BUS_RESET) {
#ifdef USB_DEBUG
    cmd_send(CMD_RESET, 0);
#endif
    USB->CLR.SIESTATUS = USB_SIE_STATUS_BUS_RESET;

    _usb_reset(usbp);
  }

  /* USB bus SUSPEND condition handling.*/
  if (ints & USB_INTS_DEV_SUSPEND) {
    USB->CLR.SIESTATUS = USB_SIE_STATUS_SUSPENDED;

    _usb_suspend(usbp);
  }

  /* Resume condition handling */
  if (ints & USB_INTS_DEV_RESUME_FROM_HOST) {
    USB->CLR.SIESTATUS = USB_SIE_STATUS_RESUME;

    _usb_wakeup(usbp);
  }

#if RP_USB_USE_SOF_INTR == TRUE
  /* SOF handling.*/
  if (ints & USB_INTS_DEV_SOF) {
    _usb_isr_invoke_sof_cb(usbp);

    /* Clear SOF flag by reading SOF_RD */
    (void)USB->SOFRD;
  }
#endif /* RP_USB_USE_SOF_INTR */

  /* Endpoint events handling.*/
  if (ints & USB_INTS_BUFF_STATUS) {
#ifdef USB_DEBUG
    cmd_send(CMD_BUFF_STATUS, 1);
    data_send(USB->BUFSTATUS);
#endif
    uint32_t buf_status = USB->BUFSTATUS;
    uint32_t bit = 1U;
    for (uint8_t i = 0; buf_status && i < 32; i++) {
      if (buf_status & bit) {
        /* Clear flag */
        USB->CLR.BUFSTATUS = bit;
#ifdef USB_DEBUG
        //cmd_send(CMD_BUFF_STATUS, 1);
        //data_send(((i >> 1U) << 16) | (i & 1U));
#endif
        /* Finish on the endpoint or transfer remained data */
        usb_serve_endpoint(&USBD1, i >> 1U, (i & 1U) == 0);

        buf_status &= ~bit;
      }
      bit <<= 1U;
    }
  }

#if RP_USB_USE_ERROR_DATA_SEQ_INTR == TRUE
  if (ints & USB_INTE_ERROR_DATA_SEQ) {
#ifdef USB_DEBUG
    cmd_send(CMD_DATA_ERROR, 0);
#endif
    USB->CLR.SIESTATUS = USB_SIE_STATUS_DATA_SEQ_ERROR;
  }
#endif /* RP_USB_USE_ERROR_DATA_SEQ_INTR */

  OSAL_IRQ_EPILOGUE();
}

#endif /* RP_USB_USE_USBD0 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {
#if RP_USB_USE_USBD0 == TRUE
  /* Driver initialization.*/
  usbObjectInit(&USBD1);

  /* Reset buffer offset. */
  USBD1.noffset = 0;
#endif
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {
#if RP_USB_USE_USBD0 == TRUE
#ifdef USB_DEBUG
  chMtxObjectInit(&cmtx);
#endif
  if (&USBD1 == usbp) {
    if (usbp->state == USB_STOP) {
      /* Reset usb controller */
      hal_lld_peripheral_reset(RESETS_ALLREG_USBCTRL);
      hal_lld_peripheral_unreset(RESETS_ALLREG_USBCTRL);

      /* Clear any previous state in dpram */
      memset(USB_DPSRAM, 0, sizeof(*USB_DPSRAM));

      /* Mux the controller to the onboard usb phy */
      USB->MUXING = USB_USB_MUXING_SOFTCON | USB_USB_MUXING_TO_PHY;

#if RP_USB_FORCE_VBUS_DETECT == TRUE
      /* Force VBUS detect so the device thinks it is plugged into a host */
      USB->PWR = USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN | USB_USB_PWR_VBUS_DETECT;
#else
#if RP_USE_EXTERNAL_VBUS_DETECT == TRUE
      /* If VBUS is detected by pin without USB VBUS DET pin,
       * define usb_vbus_detect which returns true if VBUS is enabled.
       */
      if (usb_vbus_detect()) {
        USB->PWR = USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN | USB_USB_PWR_VBUS_DETECT;
      }
#endif /* RP_USE_EXTERNAL_VBUS_DETECT */
#endif /* RP_USB_FORCE_VBUS_DETECT */

      /* Reset procedure enforced on driver start.*/
      usb_lld_reset(usbp);

      /* Enable the USB controller in device mode. */
      USB->MAINCTRL = USB_MAIN_CTRL_CONTROLLER_EN;

      /* Enable an interrupt per EP0 transaction */
      USB->SIECTRL = USB_SIE_CTRL_EP0_INT_1BUF;

      /* Enable interrupts */
      USB->INTE = USB_INTE_SETUP_REQ |
                  USB_INTE_DEV_RESUME_FROM_HOST |
                  USB_INTE_DEV_SUSPEND |
                  USB_INTE_BUS_RESET |
                  USB_INTE_BUFF_STATUS;
#if RP_USB_USE_ERROR_DATA_SEQ_INTR == TRUE
      USB->INTE |= USB_INTE_ERROR_DATA_SEQ;
#endif /* RP_USB_USE_ERROR_DATA_SEQ_INTR */
#if RP_USB_USE_SOF_INTR == TRUE
      USB->INTE |= USB_INTE_DEV_SOF;
#endif /* RP_USB_USE_SOF_INTR */

      /* Enable USB interrupt. */
      nvicEnableVector(RP_USBCTRL_IRQ_NUMBER, RP_IRQ_USB0_PRIORITY);

      /* Present full speed device by enabling pull up on DP */
      USB->SET.SIECTRL = USB_SIE_CTRL_PULLUP_EN;
    }
  }
#endif
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {
#if RP_USB_USE_USBD0 == TRUE
  if (&USBD1 == usbp) {
    if (usbp->state == USB_READY) {
      /* Disable interrupt */
      USB->INTE = 0;

      /* Disable controller */
      USB->CLR.MAINCTRL = USB_MAIN_CTRL_CONTROLLER_EN;
    }
  }
#endif
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {
  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);

  /* Reset device address. */
  USB->DEVADDRCTRL = 0;
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
  /* Set address to hardware here. */
  USB->DEVADDRCTRL = USB_ADDR_ENDP0_ADDRESS_Msk & (usbp->address << USB_ADDR_ENDP0_ADDRESS_Pos);

#ifdef USB_DEBUG
  cmd_send(CMD_SET_ADDR, 1);
  data_send(usbp->address);
#endif

  reset_ep0(usbp);
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {
  (void)usbp;

  uint16_t buf_size;
  uint16_t buf_offset;
  uint32_t buf_ctrl;
  const USBEndpointConfig *epcp = usbp->epc[ep];

  if (ep == 0) {
    epcp->in_state->hw_buf = (uint8_t*)&USB_DPSRAM->EP0BUF0;
    epcp->in_state->buf_size = 64;
    USB->SET.SIECTRL = USB_EP_BUFFER_IRQ_EN;
    BUF_CTRL(0).OUT = USB_BUFFER_BUFFER0_AVAILABLE;
    epcp->in_state->next_pid = 1U;
    epcp->in_state->active = false;
    epcp->in_state->stalled = false;
    return;
  }

  if (epcp->in_state) {
    buf_ctrl = 0;
    epcp->in_state->active = false;
    epcp->in_state->stalled = false;
    epcp->in_state->next_pid = 0U;

    if (epcp->ep_mode == USB_EP_MODE_TYPE_ISOC) {
      buf_size = usb_isochronous_buffer_size(epcp->in_maxsize);
      buf_ctrl |= usb_isochronous_buffer_mode(buf_size) << USB_BUFFER_DOUBLE_BUFFER_OFFSET_Pos;
    } else {
      buf_size = 64;
    }
    buf_offset = usb_buffer_next_offset(usbp, buf_size, true);
    epcp->in_state->hw_buf = (uint8_t*)&USB_DPSRAM->DATA[buf_offset];
    epcp->in_state->buf_size = buf_size;

    EP_CTRL(ep).IN = USB_EP_EN |
                      (epcp->ep_mode << USB_EP_TYPE_Pos) |
                      ((uint8_t*)epcp->in_state->hw_buf - (uint8_t*)USB_DPSRAM);
    BUF_CTRL(ep).IN = buf_ctrl;
  }

  if (epcp->out_state) {
    buf_ctrl = USB_BUFFER_BUFFER0_AVAILABLE | epcp->out_maxsize;
    epcp->out_state->active = false;
    epcp->out_state->stalled = false;
    epcp->out_state->next_pid = 1U;

    if (epcp->ep_mode == USB_EP_MODE_TYPE_ISOC) {
      buf_size = usb_isochronous_buffer_size(epcp->in_maxsize);
      buf_ctrl |= usb_isochronous_buffer_mode(buf_size) << USB_BUFFER_DOUBLE_BUFFER_OFFSET_Pos;
    } else {
      buf_size = 64;
    }
    buf_offset = usb_buffer_next_offset(usbp, buf_offset, false);
    epcp->out_state->hw_buf = (uint8_t*)&USB_DPSRAM->DATA[buf_offset];
    epcp->out_state->buf_size = buf_size;

    EP_CTRL(ep).OUT = USB_EP_EN | USB_EP_BUFFER_IRQ_EN |
                       (epcp->ep_mode << USB_EP_TYPE_Pos) |
                       ((uint8_t*)epcp->out_state->hw_buf - (uint8_t*)USB_DPSRAM);
    BUF_CTRL(ep).OUT = buf_ctrl;
  }
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
  uint8_t ep;

  /* Ignore zero */
  for (ep = 1; ep <= USB_ENDOPOINTS_NUMBER; ep++) {
    usbp->epc[ep]->in_state->active = false;
    usbp->epc[ep]->in_state->stalled = false;
    usbp->epc[ep]->in_state->next_pid = 0;
    EP_CTRL(ep).IN &= ~USB_EP_EN;

    usbp->epc[ep]->out_state->active = false;
    usbp->epc[ep]->out_state->stalled = false;
    usbp->epc[ep]->out_state->next_pid = 0;
    EP_CTRL(ep).OUT &= ~USB_EP_EN;
  }
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  USBOutEndpointState *out_state = usbp->epc[ep]->out_state;

  if (out_state) {
    if (out_state->active) {
      return EP_STATUS_ACTIVE;
    } else if (out_state->stalled) {
      return EP_STATUS_STALLED;
    }
  }
  return EP_STATUS_DISABLED;
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  USBInEndpointState *in_state = usbp->epc[ep]->in_state;

  if (in_state) {
    if (in_state->active) {
      return EP_STATUS_ACTIVE;
    } else if (in_state->stalled) {
      return EP_STATUS_STALLED;
    }
  }
  return EP_STATUS_DISABLED;
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {
  (void)usbp;
  (void)ep;
#ifdef USB_DEBUG
  uint32_t data1 = *((uint32_t*)USB_DPSRAM->SETUPPACKET);
  uint32_t data2 = *((uint32_t*)(USB_DPSRAM->SETUPPACKET + 4));
  cmd_send(CMD_READ_SETUP, 2);
  data_send(data1);
  data_send(data2);
#endif
  /* Copy data from hardware buffer to user buffer */
  memcpy((void *)buf, (void *)USB_DPSRAM->SETUPPACKET, 8);
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  USBOutEndpointState *oesp = usbp->epc[ep]->out_state;

  /* Transfer initialization.*/
  if (oesp->rxsize == 0) {
    /* Special case for zero sized packets.*/
    oesp->rxpkts = 1;
  } else {
    oesp->rxpkts = (uint16_t)((oesp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
                             usbp->epc[ep]->out_maxsize);
  }

#ifdef USB_DEBUG
  cmd_send(CMD_START_OUT, 1);
  data_send((ep << 24) | oesp->rxsize | (oesp->rxpkts << 16));
#endif
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {
  USBInEndpointState *iesp = usbp->epc[ep]->in_state;
#ifdef USB_DEBUG
  cmd_send(CMD_START_IN, 1);
  data_send((ep << 24) | iesp->txsize);
#endif
  iesp->txlast = 0;

  /* Prepare IN endpoint. */
  usb_prepare_in_ep(usbp, ep);
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;

  if (ep == 0) {
    USB->SET.EPSTALLARM = USB_EP_STALL_ARM_EP0_OUT;
  }
  BUF_CTRL(ep).OUT |= USB_BUFFER_STALL;
  usbp->epc[ep]->out_state->stalled = true;
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;

  if (ep == 0) {
    USB->SET.EPSTALLARM = USB_EP_STALL_ARM_EP0_IN;
  }
  BUF_CTRL(ep).IN |= USB_BUFFER_STALL;
  usbp->epc[ep]->in_state->stalled = true;
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;

  if (ep == 0) {
    USB->CLR.EPSTALLARM = USB_EP_STALL_ARM_EP0_OUT;
  }
  BUF_CTRL(ep).OUT &= ~USB_BUFFER_STALL;
  usbp->epc[ep]->out_state->stalled = false;
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;

  if (ep == 0) {
    USB->CLR.EPSTALLARM = USB_EP_STALL_ARM_EP0_IN;
  }
  BUF_CTRL(ep).IN &= ~USB_BUFFER_STALL;
  usbp->epc[ep]->in_state->stalled = false;
}

#endif /* HAL_USE_USB == TRUE */

/** @} */
