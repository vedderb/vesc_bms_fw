/*
    Copyright (C) 2021 Westberry Technology (ChangZhou) Corp., Ltd

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
 * @file    USBv1/hal_usb_lld.c
 * @brief   WB32 USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>

#include "hal.h"

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
static void _wbusb_ep0in(USBDriver *usbp, usbep_t ep);
static void _wbusb_ep0out(USBDriver *usbp, usbep_t ep);
/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USB1 driver identifier.*/
#if WB32_USB_USE_USB1 || defined(__DOXYGEN__)
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
static union {
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
 * @brief   Buffer for the EP0 setup packets.
 */
static uint8_t ep0setup_buffer[8];

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_EP_MODE_TYPE_CTRL,
  _usb_ep0setup,
  _wbusb_ep0in,
  _wbusb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out,
  1,
  ep0setup_buffer,
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
/**
 * @brief   WB32 EP0 IN callback.
 * @details This function is used by the low level driver as default handler
 *          for EP0 IN events.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number, always zero
 *
 * @notapi
 */
static void _wbusb_ep0in(USBDriver *usbp, usbep_t ep) {

  (void)ep;
  switch (usbp->ep0state) {
    case USB_EP0_IN_TX:
      /* Enter the state.*/
      if (usbp->ep0endcb != NULL) {
        usbp->ep0endcb(usbp);
      }
      usbp->ep0state = USB_EP0_STP_WAITING;
      return;
    case USB_EP0_IN_WAITING_TX0:
    case USB_EP0_IN_SENDING_STS:
    case USB_EP0_STP_WAITING:
    case USB_EP0_OUT_WAITING_STS:
    case USB_EP0_OUT_RX:
      /* All the above are invalid states in the IN phase.*/
      osalDbgAssert(false, "EP0 state machine error");
    /* Falls through.*/
    case USB_EP0_ERROR:
      /* Error response, the state machine goes into an error state, the
         low level layer will have to reset it to USB_EP0_WAITING_SETUP after
         receiving a SETUP packet.*/
      usb_lld_stall_in(usbp, 0);
      usb_lld_stall_out(usbp, 0);
      _usb_isr_invoke_event_cb(usbp, USB_EVENT_STALLED);
      usbp->ep0state = USB_EP0_ERROR;
      return;
    default:
      osalDbgAssert(false, "EP0 state machine invalid state");
  }
}

/**
 * @brief   WB32 EP0 OUT callback.
 * @details This function is used by the low level driver as default handler
 *          for EP0 OUT events.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number, always zero
 *
 * @notapi
 */
static void _wbusb_ep0out(USBDriver *usbp, usbep_t ep) {

  (void)ep;
  switch (usbp->ep0state) {
    case USB_EP0_OUT_RX:
      /* Enter the state.*/
      /* Status packet received, it must be zero sized, invoking the
       * callback if defined.*/
      if (usbp->ep0endcb != NULL) {
        usbp->ep0endcb(usbp);
      }
      usbp->ep0state = USB_EP0_STP_WAITING;
      return;
    /* Receive phase over, sending the zero sized status packet.*/
    case USB_EP0_OUT_WAITING_STS:
    case USB_EP0_STP_WAITING:
    case USB_EP0_IN_TX:
    case USB_EP0_IN_WAITING_TX0:
    case USB_EP0_IN_SENDING_STS:
      /* All the above are invalid states in the IN phase.*/
      osalDbgAssert(false, "EP0 state machine error");
    /* Falls through.*/
    case USB_EP0_ERROR:
      /* Error response, the state machine goes into an error state, the
         low level layer will have to reset it to USB_EP0_WAITING_SETUP after
         receiving a SETUP packet.*/
      usb_lld_stall_in(usbp, 0);
      usb_lld_stall_out(usbp, 0);
      _usb_isr_invoke_event_cb(usbp, USB_EVENT_STALLED);
      usbp->ep0state = USB_EP0_ERROR;
      return;
    default:
      osalDbgAssert(false, "EP0 state machine invalid state");
  }
}

/**
 * @brief   Reads from a dedicated packet buffer.
 *
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 * @return              The size of the receivee packet.
 *
 * @notapi
 */
static size_t usb_packet_read_to_buffer(usbep_t ep, uint8_t *buf) {
  size_t rx_cnt, n;
  uint8_t ep_num;
  uint32_t ep_fifo_addr;

  ep_num = ep & 0x0F;
  ep_fifo_addr = (uint32_t)&WB32_USB->FIFO[ep_num];

  WB32_USB->INDEX = ep_num;
  if (ep_num != 0) {
    rx_cnt = (size_t)((WB32_USB->OUTCOUNTH << 8) | WB32_USB->OUTCOUNTL);
  }
  else {
    rx_cnt = WB32_USB->COUNT0;
  }

  n = rx_cnt;

  while (n) {
    *buf++ = *((volatile uint8_t *)ep_fifo_addr);
    n--;
  }

  return rx_cnt;
}

/**
 * @brief   Writes to a dedicated packet buffer.
 *
 * @param[in] ep        endpoint number
 * @param[in] buf       buffer where to fetch the packet data
 * @param[in] n         maximum number of bytes to copy. This value must
 *                      not exceed the maximum packet size for this endpoint.
 *
 * @notapi
 */
static void usb_packet_write_from_buffer(usbep_t ep,
                                         const uint8_t *buf,
                                         size_t n) {
  uint32_t ep_fifo_addr = (uint32_t)&WB32_USB->FIFO[ep & 0x0F];
  while (n) {
    *((volatile uint8_t *)ep_fifo_addr) = *buf++;
    n--;
  }
}

/**
 * @brief   Common ISR code, serves the EP0-related interrupts.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void usb_serve_ep0_handler(USBDriver *usbp) {
  size_t n;
  uint8_t ControlReg;
  const USBEndpointConfig *epcp = usbp->epc[0];

  WB32_USB->INDEX = 0;
  ControlReg = WB32_USB->CSR0;

  if (ControlReg & USB_CSR0_SENTSTALL) {
    WB32_USB->CSR0 = 0x00;
    usbp->ep0state = USB_EP0_STP_WAITING;
  }

  if (ControlReg & USB_CSR0_SETUPEND) {
    WB32_USB->CSR0 = USB_CSR0_SVDSETUPEND;
    usbp->ep0state = USB_EP0_STP_WAITING;
  }

  if ((usbp->ep0state == USB_EP0_STP_WAITING) ||
      (usbp->ep0state == USB_EP0_ERROR)) {
    if (ControlReg & USB_CSR0_OUTPKTRDY) {
      /* Setup packets handling, setup packets are handled using a
       *  specific callback.*/
      _usb_isr_invoke_setup_cb(usbp, 0);
    }
  }
  else if (usbp->ep0state == USB_EP0_IN_TX) {
    /* IN endpoint, transmission.*/
    USBInEndpointState *isp = epcp->in_state;

    ControlReg = WB32_USB->CSR0;

    if (!(ControlReg & USB_CSR0_INPKTRDY)) {
      if ((!(ControlReg & USB_CSR0_SETUPEND)) ||
          (!(ControlReg & USB_CSR0_OUTPKTRDY))) {
        WB32_USB->INDEX = 0;
        ControlReg = USB_CSR0_INPKTRDY;

        isp->txcnt += isp->txlast;
        n = isp->txsize - isp->txcnt;
        if (n > 0) {
          /* Transfer not completed, there are more packets to send.*/
          if (n > epcp->in_maxsize)
            n = epcp->in_maxsize;

          /* Writes the packet from the defined buffer.*/
          isp->txbuf += isp->txlast;
          isp->txlast = n;
          usb_packet_write_from_buffer(0, isp->txbuf, n);

          if (n < epcp->in_maxsize)
            ControlReg |= USB_CSR0_DATAEND;

          /* Transfer not complete, there are more packets to send.*/
          WB32_USB->CSR0 = ControlReg;
        }
        else {
          if (isp->txlast == epcp->in_maxsize) {
            ControlReg |= USB_CSR0_DATAEND;
            WB32_USB->CSR0 = ControlReg;
            isp->txlast = 0;
          }
          else {
            /* Transfer completed, invokes the callback.*/
            _usb_isr_invoke_in_cb(usbp, 0);
          }
        }
      }
    }
  }
  else if (usbp->ep0state == USB_EP0_OUT_RX) {
    /* OUT endpoint, receive.*/
    USBOutEndpointState *osp = epcp->out_state;

    ControlReg = WB32_USB->CSR0;

    if (ControlReg & USB_CSR0_OUTPKTRDY) {
      /* Reads the packet into the defined buffer.*/
      n = usb_packet_read_to_buffer(0, osp->rxbuf);
      osp->rxbuf += n;

      /* Transaction data updated.*/
      osp->rxcnt += n;
      osp->rxsize -= n;
      osp->rxpkts -= 1;

      /* The transaction is completed if the specified number of packets
       *  has been received or the current packet is a short packet.*/
      if ((n < epcp->out_maxsize) || (osp->rxpkts == 0)) {
        WB32_USB->INDEX = 0;
        WB32_USB->CSR0 = USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND;
        /* Transfer complete, invokes the callback.*/
        _usb_isr_invoke_out_cb(usbp, 0);
      }
      else {
        /* Transfer not complete, there are more packets to receive.*/
        WB32_USB->INDEX = 0;
        WB32_USB->CSR0 = USB_CSR0_SVDOUTPKTRDY;
      }
    }
  }
}

/**
 * @brief   Common ISR code, serves the EPin-related interrupts.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void usb_serve_epin_handler(USBDriver *usbp, usbep_t ep) {
  size_t n;
  const USBEndpointConfig *epcp = usbp->epc[ep];
  /* IN endpoint, transmission.*/
  USBInEndpointState *isp = epcp->in_state;

  WB32_USB->INDEX = ep;

  isp->txcnt += isp->txlast;
  n = isp->txsize - isp->txcnt;

  if (n > 0) {
    /* Transfer not completed, there are more packets to send.*/
    if (n > epcp->in_maxsize)
      n = epcp->in_maxsize;

    /* Writes the packet from the defined buffer.*/
    isp->txbuf += isp->txlast;
    isp->txlast = n;
    usb_packet_write_from_buffer(ep, isp->txbuf, n);

    /* Starting IN operation.*/
    WB32_USB->INCSR1 = USB_INCSR1_INPKTRDY;
  }
  else {
    /* Transfer completed, invokes the callback.*/
    _usb_isr_invoke_in_cb(usbp, ep);
  }
}

/**
 * @brief   Common ISR code, serves the EPout-related interrupts.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void usb_serve_epout_handler(USBDriver *usbp, usbep_t ep) {
  size_t n;
  uint8_t ControlReg;
  const USBEndpointConfig *epcp = usbp->epc[ep];
  /* OUT endpoint, receive.*/
  USBOutEndpointState *osp = epcp->out_state;

  WB32_USB->INDEX = ep;

  ControlReg = WB32_USB->OUTCSR1;

  if (ControlReg & USB_OUTCSR1_OUTPKTRDY) {
    /* Reads the packet into the defined buffer.*/
    n = usb_packet_read_to_buffer(ep, osp->rxbuf);

    osp->rxbuf += n;

    /* Transaction data updated.*/
    osp->rxcnt += n;
    osp->rxsize -= n;
    osp->rxpkts -= 1;

    /* The transaction is completed if the specified number of packets
       has been received or the current packet is a short packet.*/
    if ((n < epcp->out_maxsize) || (osp->rxpkts == 0)) {
      /* Transfer complete, invokes the callback.*/
      _usb_isr_invoke_out_cb(usbp, ep);
    }
    else {
      /* Transfer not complete, there are more packets to receive.*/
      WB32_USB->OUTCSR1 &= ~USB_OUTCSR1_OUTPKTRDY;
    }
  }
}

/**
 * @brief   Common ISR code, serves the EP-related interrupts.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void usb_serve_endpoints(USBDriver *usbp) {
  uint8_t IntrIn = WB32_USB->INTRIN;
  uint8_t IntrOut = WB32_USB->INTROUT;

  if ((!IntrIn) && (!IntrOut)) return;

  if (IntrIn & USB_INTRIN_EP0) {
    usb_serve_ep0_handler(usbp);
  }

  if (IntrIn & USB_INTRIN_IN1) {
    usb_serve_epin_handler(usbp, 1);
  }

  if (IntrIn & USB_INTRIN_IN2) {
    usb_serve_epin_handler(usbp, 2);
  }

  if (IntrIn & USB_INTRIN_IN3) {
    usb_serve_epin_handler(usbp, 3);
  }

  if (IntrOut & USB_INTROUT_OUT1) {
    usb_serve_epout_handler(usbp, 1);
  }

  if (IntrOut & USB_INTROUT_OUT2) {
    usb_serve_epout_handler(usbp, 2);
  }

  if (IntrOut & USB_INTROUT_OUT3) {
    usb_serve_epout_handler(usbp, 3);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if WB32_USB_USE_USB1 || defined(__DOXYGEN__)
#if defined(WB32_USB1_IRQ_VECTOR)
/**
 * @brief   USB interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_USB1_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  uint8_t IntrUSB = (uint8_t)WB32_USB->INTRUSB;

  /* USB bus WAKEUP condition handling.*/
  if (IntrUSB & USB_INTRUSB_RSUIS) {
    /* Resume interrupt.*/
    _usb_wakeup(&USBD1);
  }

  /* USB bus reset condition handling.*/
  if (IntrUSB & USB_INTRUSB_RSTIS) {
    /* Reset interrupt.*/
    _usb_reset(&USBD1);
  }

  /* SOF handling.*/
  if (IntrUSB & USB_INTRUSB_SOFIS) {
    /* sof hook function */
    _usb_isr_invoke_sof_cb(&USBD1);
  }

  usb_serve_endpoints(&USBD1);

  /* USB bus SUSPEND condition handling.*/
  if (IntrUSB & USB_INTRUSB_SUSIS) {
    /* Suspend interrupt.*/
    _usb_suspend(&USBD1);
  }

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(WB32_USB1_DMA_IRQ_VECTOR)
/**
 * @brief   USB DMA interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_USB1_DMA_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  /* USB1 DMA handling.*/

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(WB32_USBP1_WKUP_IRQ_VECTOR)
/**
 * @brief   USB WKUP interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(WB32_USBP1_WKUP_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  /* USB1 DMA handling.*/

  OSAL_IRQ_EPILOGUE();
}
#endif

#endif /* WB32_USB_USE_USB1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {

  /* Driver initialization.*/
  usbObjectInit(&USBD1);
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {

  if (usbp->state == USB_STOP) {

    usbp->epc[0] = &ep0config;

    wb32_usb_init(usbp);

    WB32_USB->INTRUSBE = 0x00;
    WB32_USB->INTRINE = 0x01;
    WB32_USB->INTROUTE = 0x00;

    nvicEnableVector(WB32_USB1_NUMBER, WB32_USB_USB1_IRQ_PRIORITY);
  }
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {

  /* If in ready state then disables the USB clock.*/
  if (usbp->state != USB_STOP) {

    nvicDisableVector(WB32_USB1_NUMBER);
    WB32_USB->INTRUSBE = 0x00;
    WB32_USB->INTRINE = 0x00;
    WB32_USB->INTROUTE = 0x00;
    wb32_usb_deinit(usbp);
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {

  /* Post reset initialization.*/
  /* Disable  in all endpoint interrupt.*/
  WB32_USB->INTRINE = 0x00;
  /* Disable out all endpoint interrupt.*/
  WB32_USB->INTROUTE = 0x00;

  /* The SOF interrupt is only enabled if a callback is defined for
     this service because it is an high rate source.*/
  if (usbp->config->sof_cb != NULL)
    WB32_USB->INTRUSBE |= USB_INTRUSBE_SOFIE;

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);

  /* Enable USB suspend interrupt.*/
  WB32_USB->POWER = USB_POWER_SUSEN;
  WB32_USB->INTRUSBE |= USB_INTRUSBE_SUSIE;
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {

  WB32_USB->FADDR = (uint8_t)(usbp->address);
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

  const USBEndpointConfig *epcp = usbp->epc[ep];

  WB32_USB->INDEX = (ep & 0x0F);

  /* Setting the endpoint type. Note that isochronous endpoints cannot
     be bidirectional because it uses double buffering and both transmit and
     receive descriptor fields are used for either direction.*/
  if (ep & 0x0F) {
    switch (epcp->ep_mode & USB_EP_MODE_TYPE) {
      case USB_EP_MODE_TYPE_ISOC: {
        /* endpoint handling.*/
        if ((epcp->in_state != NULL) || (epcp->out_state != NULL)) {
        WB32_USB->INCSR2 = (USB_INCSR2_AUTOSET |
                            USB_INCSR2_ISO |
                            USB_INCSR2_DIRSEL);
          WB32_USB->INMAXP = (epcp->in_maxsize >> 3);
          WB32_USB->INCSR1 = USB_INCSR1_CLRDATATOG;
          WB32_USB->INCSR1 = USB_INCSR1_FLUSHFIFO;
          WB32_USB->INCSR1 = USB_INCSR1_FLUSHFIFO;
          WB32_USB->INTRINE |= (0x01 << (ep & 0x0F));

          WB32_USB->OUTCSR2 = (USB_OUTCSR2_AUTOCLR | USB_OUTCSR2_ISO);
          WB32_USB->OUTMAXP = (epcp->out_maxsize >> 3);
          WB32_USB->OUTCSR1 = USB_OUTCSR1_CLRDATATOG;
          WB32_USB->OUTCSR1 = USB_OUTCSR1_FLUSHFIFO;
          WB32_USB->OUTCSR1 = USB_OUTCSR1_FLUSHFIFO;
          WB32_USB->INTROUTE |= (0x01 << (ep & 0x0F));
        }
      } break;
      case USB_EP_MODE_TYPE_BULK: {
        /* IN endpoint handling.*/
        if (epcp->in_state != NULL) {
          WB32_USB->INCSR2 = 0x00;
          WB32_USB->INMAXP = (epcp->in_maxsize >> 3);
          WB32_USB->INCSR1 = USB_INCSR1_CLRDATATOG;
          WB32_USB->INCSR1 = USB_INCSR1_FLUSHFIFO;
          WB32_USB->INCSR1 = USB_INCSR1_FLUSHFIFO;
          WB32_USB->INTRINE |= (0x01 << (ep & 0x0F));
        }
        /* OUT endpoint handling.*/
        if (epcp->out_state != NULL) {
          WB32_USB->OUTCSR2 = 0x00;
          WB32_USB->OUTMAXP = (epcp->out_maxsize >> 3);
          WB32_USB->OUTCSR1 = USB_OUTCSR1_CLRDATATOG;
          WB32_USB->OUTCSR1 = USB_OUTCSR1_FLUSHFIFO;
          WB32_USB->OUTCSR1 = USB_OUTCSR1_FLUSHFIFO;
          WB32_USB->INTROUTE |= (0x01 << (ep & 0x0F));
        }
      } break;
      case USB_EP_MODE_TYPE_INTR: {
        /* IN endpoint handling.*/
        if (epcp->in_state != NULL) {
          WB32_USB->INCSR2 = 0x00;
          WB32_USB->INMAXP = (epcp->in_maxsize >> 3);
          WB32_USB->INCSR1 = USB_INCSR1_CLRDATATOG;
          WB32_USB->INCSR1 = USB_INCSR1_FLUSHFIFO;
          WB32_USB->INCSR1 = USB_INCSR1_FLUSHFIFO;
          WB32_USB->INTRINE |= (0x01 << (ep & 0x0F));
        }
        /* OUT endpoint handling.*/
        if (epcp->out_state != NULL) {
          WB32_USB->OUTCSR2 = 0x00;
          WB32_USB->OUTMAXP = (epcp->out_maxsize >> 3);
          WB32_USB->OUTCSR1 = USB_OUTCSR1_CLRDATATOG;
          WB32_USB->OUTCSR1 = USB_OUTCSR1_FLUSHFIFO;
          WB32_USB->OUTCSR1 = USB_OUTCSR1_FLUSHFIFO;
          WB32_USB->INTROUTE |= (0x01 << (ep & 0x0F));
        }
      } break;
      default: {

      } break;
    }
  }
  else {
    if ((epcp->ep_mode & USB_EP_MODE_TYPE) == USB_EP_MODE_TYPE_CTRL) {
      /* CTRL Mode.*/
      /* Enable in endpoint 0 interrupt.*/
      WB32_USB->INTRINE |= USB_INTRINE_EP0E;
    }
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
  uint8_t i;
  WB32_USB->INTRINE &= 0x01;
  WB32_USB->INTROUTE = 0x00;

  /* Disabling all endpoints.*/
  for (i = 1; i <= (USB_ENDOPOINTS_NUMBER & 0x0F); i++) {
    WB32_USB->INDEX = i;
    WB32_USB->INCSR2 = 0x00;
    WB32_USB->INCSR1 = USB_INCSR1_CLRDATATOG;
    WB32_USB->INCSR1 = USB_INCSR1_FLUSHFIFO;
    WB32_USB->INCSR1 = USB_INCSR1_FLUSHFIFO;

    WB32_USB->OUTCSR2 = 0x00;
    WB32_USB->OUTCSR1 = USB_OUTCSR1_CLRDATATOG;
    WB32_USB->OUTCSR1 = USB_OUTCSR1_FLUSHFIFO;
    WB32_USB->OUTCSR1 = USB_OUTCSR1_FLUSHFIFO;
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

  WB32_USB->INDEX = (ep & 0x0F);

  if (ep == 0) {
    if (WB32_USB->CSR0 & USB_CSR0_SENTSTALL) {
      return EP_STATUS_STALLED;
    }
    else if (WB32_USB->INTRINE & USB_INTRINE_EP0E) {
      return EP_STATUS_ACTIVE;
    }
    else {
      return EP_STATUS_DISABLED;
    }
  }
  else {
    if (WB32_USB->OUTCSR1 & USB_OUTCSR1_SENDSTALL) {
      return EP_STATUS_STALLED;
    }
    else if (WB32_USB->INTROUTE & (1 << (ep & 0x0F))) {
      return EP_STATUS_ACTIVE;
    }
    else {
      return EP_STATUS_DISABLED;
    }
  }
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

  WB32_USB->INDEX = (ep & 0x0F);

  if (ep == 0) {
    if (WB32_USB->CSR0 & USB_CSR0_SENTSTALL) {
      return EP_STATUS_STALLED;
    }
    else if (WB32_USB->INTRINE & USB_INTRINE_EP0E) {
      return EP_STATUS_ACTIVE;
    }
    else {
      return EP_STATUS_DISABLED;
    }
  }
  else {
    if (WB32_USB->INCSR1 & USB_INCSR1_SENDSTALL) {
      return EP_STATUS_STALLED;
    }
    else if (WB32_USB->INTRINE & (1 << (ep & 0x0F))) {
      return EP_STATUS_ACTIVE;
    }
    else {
      return EP_STATUS_DISABLED;
    }
  }
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
  uint32_t ep_fifo_addr;
  uint8_t n;

  (void)usbp;
  ep_fifo_addr = (uint32_t)&WB32_USB->FIFO[ep & 0x0F];
  for (n = 0; n < 8; n++) {
    *buf = *((volatile uint8_t *)ep_fifo_addr);
    buf++;
  }
}

/**
 * @brief   Ends a SETUP transaction
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to finish an entire setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_end_setup(USBDriver *usbp, usbep_t ep) {

  uint8_t ep_num = (ep & 0x0F);
  WB32_USB->INDEX = ep_num;
  WB32_USB->CSR0 = USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND;
  usbp->ep0state = USB_EP0_STP_WAITING;
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
  uint8_t ep_num = (ep & 0x0F);
  USBOutEndpointState *osp = usbp->epc[ep]->out_state;

  /* Transfer initialization.*/
  /* Special case for zero sized packets.*/
  if (osp->rxsize == 0)
    osp->rxpkts = 1;
  else
    osp->rxpkts = (uint16_t)((osp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
                              usbp->epc[ep]->out_maxsize);

  if (ep_num != 0) {
    /* Transfer not complete, there are more packets to receive.*/
    WB32_USB->INDEX = ep_num;
    WB32_USB->OUTCSR1 &= ~USB_OUTCSR1_OUTPKTRDY;
  }
  else {
    /* Transfer not complete, there are more packets to receive.*/
    WB32_USB->INDEX = 0;
    WB32_USB->CSR0 = USB_CSR0_SVDOUTPKTRDY;
  }
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
  size_t n;
  uint8_t ep_num = (ep & 0x0F);
  uint8_t ControlReg;
  USBInEndpointState *isp = usbp->epc[ep]->in_state;

  if (ep_num == 0) {
    WB32_USB->INDEX = 0;
    WB32_USB->CSR0 = USB_CSR0_SVDOUTPKTRDY;
  }

  /* Transfer initialization.*/
  n = isp->txsize;
  if (n > (size_t)usbp->epc[ep]->in_maxsize)
    n = (size_t)usbp->epc[ep]->in_maxsize;

  isp->txlast = n;
  usb_packet_write_from_buffer(ep, isp->txbuf, n);

  if (ep_num != 0) {
    WB32_USB->INDEX = ep_num;
    WB32_USB->INCSR1 = USB_INCSR1_INPKTRDY;
  }
  else {
    WB32_USB->INDEX = 0;
    ControlReg = WB32_USB->CSR0;
    ControlReg = USB_CSR0_INPKTRDY;
    if (n < (size_t)usbp->epc[0]->in_maxsize) {
      ControlReg |= USB_CSR0_DATAEND;
    }
    WB32_USB->CSR0 = ControlReg;
  }
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

  uint8_t ep_num;

  (void)usbp;

  ep_num = ep & 0x0F;
  WB32_USB->INDEX = ep_num;
  if (ep_num != 0) {
    /* OUT Endpoint */
    WB32_USB->OUTCSR1 = USB_OUTCSR1_SENDSTALL;
  }
  else {
    WB32_USB->CSR0 = (USB_CSR0_SVDOUTPKTRDY | USB_CSR0_SENDSTALL);
  }
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

  uint8_t ep_num;

  (void)usbp;

  ep_num = ep & 0x0F;
  WB32_USB->INDEX = ep_num;
  if (ep_num != 0) {
    /* IN Endpoint.*/
    WB32_USB->INCSR1 = USB_INCSR1_SENDSTALL;
  }
  else {
    WB32_USB->CSR0 = (USB_CSR0_SVDOUTPKTRDY | USB_CSR0_SENDSTALL);
  }
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

  uint8_t ep_num;

  (void)usbp;

  ep_num = ep & 0x0F;
  WB32_USB->INDEX = ep_num;
  if (ep_num != 0) {
    /* OUT Endpoint.*/
    WB32_USB->OUTCSR1 = USB_OUTCSR1_CLRDATATOG;
  }
  else {
    WB32_USB->CSR0 = 0x00;
  }
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

  uint8_t ep_num;

  (void)usbp;

  ep_num = ep & 0x0F;
  WB32_USB->INDEX = ep_num;
  if (ep_num != 0) {
    /* IN Endpoint.*/
    WB32_USB->INCSR1 = USB_INCSR1_CLRDATATOG;
  }
  else {
    WB32_USB->CSR0 = 0x00;
  }
}

#endif /* HAL_USE_USB */

/** @} */
