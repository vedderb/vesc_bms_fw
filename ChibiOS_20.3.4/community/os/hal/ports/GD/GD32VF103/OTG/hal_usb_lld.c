/*
     ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio
     ChibiOS - Copyright (C) 2021 Stefan Kerkmann

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
 * @file    OTG/hal_usb_lld.c
 * @brief   GD32 USB subsystem low level driver source.
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

#define TRDT_VALUE_FS           5
#define TRDT_VALUE_HS           9

#define EP0_MAX_INSIZE          64
#define EP0_MAX_OUTSIZE         64

#if defined(BOARD_USBFS_NOVBUSSENS)
#define GCCFG_INIT_VALUE        (GCCFG_VBUSIG | GCCFG_VBUSACEN |        \
                                 GCCFG_VBUSBCEN | GCCFG_PWRON)
#else
#define GCCFG_INIT_VALUE        (GCCFG_VBUSACEN | GCCFG_VBUSBCEN |          \
                                 GCCFG_PWRON)
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USBFS driver identifier.*/
#if GD32_USB_USE_USBFS || defined(__DOXYGEN__)
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
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out,
  1,
  ep0setup_buffer
};

#if GD32_USB_USE_USBFS
static const gd32_usbfs_params_t fsparams = {
  GD32_USB_USBFS_RX_FIFO_SIZE / 4,
  GD32_USBFS_FIFO_MEM_SIZE,
  GD32_USBFS_ENDPOINTS
};
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void otg_core_reset(USBDriver *usbp) {
  gd32_usbfs_t *otgp = usbp->otg;

  /* Core reset and delay of at least 3 PHY cycles.*/
  otgp->GRSTCTL = GRSTCTL_CSRST;
  osalSysPolledDelayX(12);
  while ((otgp->GRSTCTL & GRSTCTL_CSRST) != 0)
    ;

  osalSysPolledDelayX(18);
}

static void otg_disable_ep(USBDriver *usbp) {
  gd32_usbfs_t *otgp = usbp->otg;
  unsigned i;

  for (i = 0; i <= usbp->otgparams->num_endpoints; i++) {

    if ((otgp->ie[i].DIEPCTL & DIEPCTL_EPEN) != 0U) {
      otgp->ie[i].DIEPCTL |= DIEPCTL_EPD;
    }

    if ((otgp->oe[i].DOEPCTL & DIEPCTL_EPEN) != 0U) {
      otgp->oe[i].DOEPCTL |= DIEPCTL_EPD;
    }

    otgp->ie[i].DIEPINTF = 0xFFFFFFFF;
    otgp->oe[i].DOEPINTF = 0xFFFFFFFF;
  }
  otgp->DAEPINTEN = DAEPINTEN_OEPIE(0) | DAEPINTEN_IEPIE(0);
}

static void otg_rxfifo_flush(USBDriver *usbp) {
  gd32_usbfs_t *otgp = usbp->otg;

  otgp->GRSTCTL = GRSTCTL_RXFF;
  while ((otgp->GRSTCTL & GRSTCTL_RXFF) != 0)
    ;
  /* Wait for 3 PHY Clocks.*/
  osalSysPolledDelayX(18);
}

static void otg_txfifo_flush(USBDriver *usbp, uint32_t fifo) {
  gd32_usbfs_t *otgp = usbp->otg;

  otgp->GRSTCTL = GRSTCTL_TXFNUM(fifo) | GRSTCTL_TXFF;
  while ((otgp->GRSTCTL & GRSTCTL_TXFF) != 0)
    ;
  /* Wait for 3 PHY Clocks.*/
  osalSysPolledDelayX(18);
}

/**
 * @brief   Resets the FIFO RAM memory allocator.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void otg_ram_reset(USBDriver *usbp) {

  usbp->pmnext = usbp->otgparams->rx_fifo_size;
}

/**
 * @brief   Allocates a block from the FIFO RAM memory.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] size      size of the packet buffer to allocate in words
 *
 * @notapi
 */
static uint32_t otg_ram_alloc(USBDriver *usbp, size_t size) {
  uint32_t next;

  next = usbp->pmnext;
  usbp->pmnext += size;
  osalDbgAssert(usbp->pmnext <= usbp->otgparams->otg_ram_size,
                "USBFS FIFO memory overflow");
  return next;
}

/**
 * @brief   Writes to a TX FIFO.
 *
 * @param[in] fifop     pointer to the FIFO register
 * @param[in] buf       buffer where to copy the endpoint data
 * @param[in] n         maximum number of bytes to copy
 *
 * @notapi
 */
static void otg_fifo_write_from_buffer(volatile uint32_t *fifop,
                                       const uint8_t *buf,
                                       size_t n) {

  osalDbgAssert(n > 0, "is zero");

  while (true) {
    *fifop = *((uint32_t *)buf);
    if (n <= 4) {
      break;
    }
    n -= 4;
    buf += 4;
  }
}

/**
 * @brief   Reads a packet from the RXFIFO.
 *
 * @param[in] fifop     pointer to the FIFO register
 * @param[out] buf      buffer where to copy the endpoint data
 * @param[in] n         number of bytes to pull from the FIFO
 * @param[in] max       number of bytes to copy into the buffer
 *
 * @notapi
 */
static void otg_fifo_read_to_buffer(volatile uint32_t *fifop,
                                    uint8_t *buf,
                                    size_t n,
                                    size_t max) {
  uint32_t w = 0;
  size_t i = 0;

  while (i < n) {
    if ((i & 3) == 0) {
      w = *fifop;
    }
    if (i < max) {
      *buf++ = (uint8_t)w;
      w >>= 8;
    }
    i++;
  }
}

/**
 * @brief   Incoming packets handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void otg_rxfifo_handler(USBDriver *usbp) {
  uint32_t sts, cnt, ep;

  /* Popping the event word out of the RX FIFO.*/
  sts = usbp->otg->GRSTATP;

  /* Event details.*/
  cnt = (sts & GRSTATP_BCOUNT_MASK) >> GRSTATP_BCOUNT_OFF;
  ep  = (sts & GRSTATP_EPNUM_MASK) >> GRSTATP_EPNUM_OFF;

  switch (sts & GRSTATP_RPCKST_MASK) {
  case GRSTATP_SETUP_DATA:
    otg_fifo_read_to_buffer(usbp->otg->FIFO[0], usbp->epc[ep]->setup_buf,
                            cnt, 8);
    break;
  case GRSTATP_SETUP_COMP:
    break;
  case GRSTATP_OUT_DATA:
    otg_fifo_read_to_buffer(usbp->otg->FIFO[0],
                            usbp->epc[ep]->out_state->rxbuf,
                            cnt,
                            usbp->epc[ep]->out_state->rxsize -
                            usbp->epc[ep]->out_state->rxcnt);
    usbp->epc[ep]->out_state->rxbuf += cnt;
    usbp->epc[ep]->out_state->rxcnt += cnt;
    break;
  case GRSTATP_OUT_COMP:
    break;
  case GRSTATP_OUT_GLOBAL_NAK:
    break;
  default:
    break;
  }
}

/**
 * @brief   Outgoing packets handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static bool otg_txfifo_handler(USBDriver *usbp, usbep_t ep) {

  /* The TXFIFO is filled until there is space and data to be transmitted.*/
  while (true) {
    uint32_t n;

    /* Transaction end condition.*/
    if (usbp->epc[ep]->in_state->txcnt >= usbp->epc[ep]->in_state->txsize) {
#if 1
      usbp->otg->DIEPFEINTEN &= ~DIEPFEINTEN_IEPTXFEIE(ep);
#endif
      return true;
    }

    /* Number of bytes remaining in current transaction.*/
    n = usbp->epc[ep]->in_state->txsize - usbp->epc[ep]->in_state->txcnt;
    if (n > usbp->epc[ep]->in_maxsize)
      n = usbp->epc[ep]->in_maxsize;

    /* Checks if in the TXFIFO there is enough space to accommodate the
       next packet.*/
    if (((usbp->otg->ie[ep].DIEPTFSTAT & DIEPTFSTAT_IEPTFS_MASK) * 4) < n)
      return false;

// TODO Enable again or keep brute force?
/*#if GD32_USB_USBFSFIFO_FILL_BASEPRI
    uint8_t threshold_old = eclic_get_mth();
    eclic_set_mth(GD32_USB_USBFSFIFO_FILL_BASEPRI);
#endif*/
    osalSysLockFromISR();
    otg_fifo_write_from_buffer(usbp->otg->FIFO[ep],
                               usbp->epc[ep]->in_state->txbuf,
                               n);
    usbp->epc[ep]->in_state->txbuf += n;
    usbp->epc[ep]->in_state->txcnt += n;
    osalSysUnlockFromISR();
/*#if GD32_USB_USBFSFIFO_FILL_BASEPRI
    eclic_set_mth(threshold_old);
#endif*/
  }
}

/**
 * @brief   Generic endpoint IN handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void otg_epin_handler(USBDriver *usbp, usbep_t ep) {
  gd32_usbfs_t *otgp = usbp->otg;
  uint32_t epint = otgp->ie[ep].DIEPINTF;

  otgp->ie[ep].DIEPINTF = epint;

  if (epint & DIEPINTFF_CITO) {
    /* Timeouts not handled yet, not sure how to handle.*/
  }
  if ((epint & DIEPINTFF_TF) && (otgp->DIEPINTF & DIEPINTF_TFEN)) {
    /* Transmit transfer complete.*/
    USBInEndpointState *isp = usbp->epc[ep]->in_state;

    if (isp->txsize < isp->totsize) {
      /* In case the transaction covered only part of the total transfer
         then another transaction is immediately started in order to
         cover the remaining.*/
      isp->txsize = isp->totsize - isp->txsize;
      isp->txcnt  = 0;
      osalSysLockFromISR();
      usb_lld_start_in(usbp, ep);
      osalSysUnlockFromISR();
    }
    else {
      /* End on IN transfer.*/
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  }
  if ((epint & DIEPINTFF_TXFE) &&
      (otgp->DIEPFEINTEN & DIEPFEINTEN_IEPTXFEIE(ep))) {
    /* TX FIFO empty or emptying.*/
    otg_txfifo_handler(usbp, ep);
  }
}

/**
 * @brief   Generic endpoint OUT handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void otg_epout_handler(USBDriver *usbp, usbep_t ep) {
  gd32_usbfs_t *otgp = usbp->otg;
  uint32_t epint = otgp->oe[ep].DOEPINTF;

  /* Resets all EP IRQ sources.*/
  otgp->oe[ep].DOEPINTF = epint;

  if ((epint & DOEPINTF_STPF) && (otgp->DOEPINTF & DOEPINTF_STPFEN)) {
    /* Setup packets handling, setup packets are handled using a
       specific callback.*/
    _usb_isr_invoke_setup_cb(usbp, ep);
  }

  if ((epint & DOEPINTF_TF) && (otgp->DOEPINTF & DOEPINTF_TFEN)) {
    USBOutEndpointState *osp;

    /* OUT state structure pointer for this endpoint.*/
    osp = usbp->epc[ep]->out_state;

    /* EP0 requires special handling.*/
    if (ep == 0) {

#if defined(GD32_USBFS_SEQUENCE_WORKAROUND)
      /* If an OUT transaction end interrupt is processed while the state
         machine is not in an OUT state then it is ignored, this is caused
         on some devices (L4) apparently injecting spurious data complete
         words in the RX FIFO.*/
      if ((usbp->ep0state & USB_OUT_STATE) == 0)
        return;
#endif

      /* In case the transaction covered only part of the total transfer
         then another transaction is immediately started in order to
         cover the remaining.*/
      if (((osp->rxcnt % usbp->epc[ep]->out_maxsize) == 0) &&
          (osp->rxsize < osp->totsize)) {
        osp->rxsize = osp->totsize - osp->rxsize;
        osp->rxcnt  = 0;
        osalSysLockFromISR();
        usb_lld_start_out(usbp, ep);
        osalSysUnlockFromISR();
        return;
      }
    }

    /* End on OUT transfer.*/
    _usb_isr_invoke_out_cb(usbp, ep);
  }
}

/**
 * @brief   Isochronous IN transfer failed handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void otg_isoc_in_failed_handler(USBDriver *usbp) {
  usbep_t ep;
  gd32_usbfs_t *otgp = usbp->otg;

  for (ep = 0; ep <= usbp->otgparams->num_endpoints; ep++) {
    if (((otgp->ie[ep].DIEPCTL & DIEPCTL_EPTYPE_MASK) == DIEPCTL_EPTYPE_ISO) &&
        ((otgp->ie[ep].DIEPCTL & DIEPCTL_EPEN) != 0)) {
      /* Endpoint enabled -> ISOC IN transfer failed */
      /* Disable endpoint */
      otgp->ie[ep].DIEPCTL |= (DIEPCTL_EPD | DIEPCTL_SNAK);
      while (otgp->ie[ep].DIEPCTL & DIEPCTL_EPEN)
        ;

      /* Flush FIFO */
      otg_txfifo_flush(usbp, ep);

      /* Prepare data for next frame */
      _usb_isr_invoke_in_cb(usbp, ep);

      /* TX FIFO empty or emptying.*/
      otg_txfifo_handler(usbp, ep);
    }
  }
}

/**
 * @brief   Isochronous OUT transfer failed handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void otg_isoc_out_failed_handler(USBDriver *usbp) {
  usbep_t ep;
  gd32_usbfs_t *otgp = usbp->otg;

  for (ep = 0; ep <= usbp->otgparams->num_endpoints; ep++) {
    if (((otgp->oe[ep].DOEPCTL & DOEPCTL_EPTYPE_MASK) == DOEPCTL_EPTYPE_ISO) &&
        ((otgp->oe[ep].DOEPCTL & DOEPCTL_EPEN) != 0)) {
      /* Endpoint enabled -> ISOC OUT transfer failed */
      /* Disable endpoint */
      /* CHTODO:: Core stucks here */
      /*otgp->oe[ep].DOEPCTL |= (DOEPCTL_EPD | DOEPCTL_SNAK);
      while (otgp->oe[ep].DOEPCTL & DOEPCTL_EPEN)
        ;*/
      /* Prepare transfer for next frame.*/
      _usb_isr_invoke_out_cb(usbp, ep);
    }
  }
}

/**
 * @brief   USBFS shared ISR.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void usb_lld_serve_interrupt(USBDriver *usbp) {
  gd32_usbfs_t *otgp = usbp->otg;
  uint32_t sts, src;

  sts  = otgp->GINTF;
  sts &= otgp->GINTEN;
  otgp->GINTF = sts;

  /* Reset interrupt handling.*/
  if (sts & GINTF_RST) {
    /* Default reset action.*/
    _usb_reset(usbp);

    /* Preventing execution of more handlers, the core has been reset.*/
    return;
  }

  /* Wake-up handling.*/
  if (sts & GINTF_WKUPIF) {
    /* If clocks are gated off, turn them back on (may be the case if
       coming out of suspend mode).*/
    if (otgp->PWRCLKCTL & (PWRCLKCTL_SUCLK | PWRCLKCTL_SHCLK)) {
      /* Set to zero to un-gate the USB core clocks.*/
      otgp->PWRCLKCTL &= ~(PWRCLKCTL_SUCLK | PWRCLKCTL_SHCLK);
    }

    /* Clear the Remote Wake-up Signaling.*/
    otgp->DCTL &= ~DCTL_RWKUP;

    _usb_wakeup(usbp);
  }

  /* Suspend handling.*/
  if (sts & GINTF_SP) {
    /* Stopping all ongoing transfers.*/
    otg_disable_ep(usbp);

    /* Default suspend action.*/
    _usb_suspend(usbp);
  }

  /* Enumeration done.*/
  if (sts & GINTF_ENUMF) {
      otgp->GUSBCS = (otgp->GUSBCS & ~(GUSBCS_UTT_MASK)) |
                      GUSBCS_UTT(TRDT_VALUE_FS);
  }

  /* SOF interrupt handling.*/
  if (sts & GINTF_SOF) {
    _usb_isr_invoke_sof_cb(usbp);
  }

  /* Isochronous IN failed handling */
  if (sts & GINTF_ISOINCIF) {
    otg_isoc_in_failed_handler(usbp);
  }

  /* Isochronous OUT failed handling */
  if (sts & GINTF_ISOONCIF) {
    otg_isoc_out_failed_handler(usbp);
  }

  /* Performing the whole FIFO emptying in the ISR, it is advised to keep
     this IRQ at a very low priority level.*/
  if ((sts & GINTF_RXFNEIF) != 0U) {
    otg_rxfifo_handler(usbp);
  }

  /* IN/OUT endpoints event handling.*/
  src = otgp->DAEPINT;
  if (sts & GINTF_OEPIF) {
    if (src & (1 << 16))
      otg_epout_handler(usbp, 0);
    if (src & (1 << 17))
      otg_epout_handler(usbp, 1);
    if (src & (1 << 18))
      otg_epout_handler(usbp, 2);
    if (src & (1 << 19))
      otg_epout_handler(usbp, 3);
  }
  if (sts & GINTF_IEPIF) {
    if (src & (1 << 0))
      otg_epin_handler(usbp, 0);
    if (src & (1 << 1))
      otg_epin_handler(usbp, 1);
    if (src & (1 << 2))
      otg_epin_handler(usbp, 2);
    if (src & (1 << 3))
      otg_epin_handler(usbp, 3);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if GD32_USB_USE_USBFS || defined(__DOXYGEN__)
/**
 * @brief   USBFS interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_USBFS_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  usb_lld_serve_interrupt(&USBD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

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
  USBD1.otg       = USBFS;
  USBD1.otgparams = &fsparams;
}

/**
 * @brief   Configures and activates the USB peripheral.
 * @note    Starting the USBFS cell can be a slow operation carried out with
 *          interrupts disabled, perform it before starting time-critical
 *          operations.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {
  gd32_usbfs_t *otgp = usbp->otg;

  if (usbp->state == USB_STOP) {
    /* Clock activation.*/

  if (&USBD1 == usbp) {
    /* USBFS clock enable and reset.*/
    rcuEnableUSBFS(true);
    rcuResetUSBFS();

    /* Enables IRQ vector.*/
    eclicEnableVector(GD32_USBFS_NUMBER, GD32_USB_USBFS_IRQ_PRIORITY, GD32_USB_USBFS_IRQ_TRIGGER);

   /* - Forced device mode.
        - USB turn-around time = TRDT_VALUE_FS.
        - Full Speed 1.1 PHY.*/
    otgp->GUSBCS = GUSBCS_FDM | GUSBCS_UTT(TRDT_VALUE_FS);

    /* 48MHz 1.1 PHY.*/
    otgp->DCFG = 0x02200000 | DCFG_DS_FS11;
    }

    /* PHY enabled.*/
    otgp->PWRCLKCTL = 0;

    /* VBUS sensing and transceiver enabled.*/
    otgp->GCCFG = GCCFG_INIT_VALUE;

    /* Soft core reset.*/
    otg_core_reset(usbp);

    /* Interrupts on TXFIFOs half empty.*/
    otgp->GAHBCS = 0;

    /* Endpoints re-initialization.*/
    otg_disable_ep(usbp);

    /* Clear all pending Device Interrupts, only the USB Reset interrupt
       is required initially.*/
    otgp->DIEPINTF  = 0;
    otgp->DOEPINTF  = 0;
    otgp->DAEPINTEN = 0;
    if (usbp->config->sof_cb == NULL)
      otgp->GINTEN  = GINTEN_ENUMFIE | GINTEN_RSTIE | GINTEN_SPIE |
                       GINTEN_ESPIE | GINTEN_SESIE | GINTEN_WKUPIE |
                       GINTEN_ISOINCIE | GINTEN_ISOONCIE;
    else
      otgp->GINTEN  = GINTEN_ENUMFIE | GINTEN_RSTIE | GINTEN_SPIE |
                       GINTEN_ESPIE | GINTEN_SESIE | GINTEN_WKUPIE |
                       GINTEN_ISOINCIE | GINTEN_ISOONCIE |
                       GINTEN_SOFIE;

    /* Clears all pending IRQs, if any. */
    otgp->GINTF  = 0xFFFFFFFF;

    /* Global interrupts enable.*/
    otgp->GAHBCS |= GAHBCS_GINTEN;
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
  gd32_usbfs_t *otgp = usbp->otg;

  /* If in ready state then disables the USB clock.*/
  if (usbp->state != USB_STOP) {

    /* Disabling all endpoints in case the driver has been stopped while
       active.*/
    otg_disable_ep(usbp);

    otgp->DAEPINTEN   = 0;
    otgp->GAHBCS    = 0;
    otgp->GCCFG      = 0;

    if (&USBD1 == usbp) {
      eclicDisableVector(GD32_USBFS_NUMBER);
      rcuDisableUSBFS();
    }
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
  unsigned i;
  gd32_usbfs_t *otgp = usbp->otg;

  /* Flush the Tx FIFO.*/
  otg_txfifo_flush(usbp, 0);

  /* Endpoint interrupts all disabled and cleared.*/
  otgp->DIEPFEINTEN = 0;
  otgp->DAEPINTEN   = DAEPINTEN_OEPIE(0) | DAEPINTEN_IEPIE(0);

  /* All endpoints in NAK mode, interrupts cleared.*/
  for (i = 0; i <= usbp->otgparams->num_endpoints; i++) {
    otgp->ie[i].DIEPCTL = DIEPCTL_SNAK;
    otgp->oe[i].DOEPCTL = DOEPCTL_SNAK;
    otgp->ie[i].DIEPINTF = 0xFFFFFFFF;
    otgp->oe[i].DOEPINTF = 0xFFFFFFFF;
  }

  /* Resets the FIFO memory allocator.*/
  otg_ram_reset(usbp);

  /* Receive FIFO size initialization, the address is always zero.*/
  otgp->GRFLEN = usbp->otgparams->rx_fifo_size;
  otg_rxfifo_flush(usbp);

  /* Resets the device address to zero.*/
  otgp->DCFG = (otgp->DCFG & ~DCFG_DAR_MASK) | DCFG_DAR(0);

  /* Enables also EP-related interrupt sources.*/
  otgp->GINTEN  |= GINTEN_RXFNEIE | GINTEN_OEPIE  | GINTEN_IEPIE;
  otgp->DIEPINTF   = DIEPINTF_CITOEN    | DIEPINTF_TFEN;
  otgp->DOEPINTF   = DOEPINTF_STPFEN   | DOEPINTF_TFEN;

  /* EP0 initialization, it is a special case.*/
  usbp->epc[0] = &ep0config;
  otgp->oe[0].DOEPLEN = DOEPLEN_STPCNT(3);
  otgp->oe[0].DOEPCTL = DOEPCTL_SD0PID | DOEPCTL_EPACT | DOEPCTL_EPTYPE_CTRL |
                        DOEPCTL_MPL(ep0config.out_maxsize);
  otgp->ie[0].DIEPLEN = 0;
  otgp->ie[0].DIEPCTL = DIEPCTL_SD0PID | DIEPCTL_EPACT | DIEPCTL_EPTYPE_CTRL |
                        DIEPCTL_TXFNUM(0) | DIEPCTL_MPL(ep0config.in_maxsize);
  otgp->DIEPTFLEN0 = DIEPTFLEN_IEPTXFD(ep0config.in_maxsize / 4) |
                   DIEPTFLEN_IEPTXRSAR(otg_ram_alloc(usbp,
                                                  ep0config.in_maxsize / 4));
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
  gd32_usbfs_t *otgp = usbp->otg;

  otgp->DCFG = (otgp->DCFG & ~DCFG_DAR_MASK) | DCFG_DAR(usbp->address);
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
  uint32_t ctl, fsize;
  gd32_usbfs_t *otgp = usbp->otg;

  /* IN and OUT common parameters.*/
  switch (usbp->epc[ep]->ep_mode & USB_EP_MODE_TYPE) {
  case USB_EP_MODE_TYPE_CTRL:
    ctl = DIEPCTL_SD0PID | DIEPCTL_EPACT | DIEPCTL_EPTYPE_CTRL;
    break;
  case USB_EP_MODE_TYPE_ISOC:
    ctl = DIEPCTL_SD0PID | DIEPCTL_EPACT | DIEPCTL_EPTYPE_ISO;
    break;
  case USB_EP_MODE_TYPE_BULK:
    ctl = DIEPCTL_SD0PID | DIEPCTL_EPACT | DIEPCTL_EPTYPE_BULK;
    break;
  case USB_EP_MODE_TYPE_INTR:
    ctl = DIEPCTL_SD0PID | DIEPCTL_EPACT | DIEPCTL_EPTYPE_INTR;
    break;
  default:
    return;
  }

  /* OUT endpoint activation or deactivation.*/
  otgp->oe[ep].DOEPLEN = 0;
  if (usbp->epc[ep]->out_state != NULL) {
    otgp->oe[ep].DOEPCTL = ctl | DOEPCTL_MPL(usbp->epc[ep]->out_maxsize);
    otgp->DAEPINTEN |= DAEPINTEN_OEPIE(ep);
  }
  else {
    otgp->oe[ep].DOEPCTL &= ~DOEPCTL_EPACT;
    otgp->DAEPINTEN &= ~DAEPINTEN_OEPIE(ep);
  }

  /* IN endpoint activation or deactivation.*/
  otgp->ie[ep].DIEPLEN = 0;
  if (usbp->epc[ep]->in_state != NULL) {
    /* FIFO allocation for the IN endpoint.*/
    fsize = usbp->epc[ep]->in_maxsize / 4;
    if (usbp->epc[ep]->in_multiplier > 1)
      fsize *= usbp->epc[ep]->in_multiplier;
    otgp->DIEPTFLEN[ep - 1] = DIEPTFLEN_IEPTXFD(fsize) |
                            DIEPTFLEN_IEPTXRSAR(otg_ram_alloc(usbp, fsize));
    otg_txfifo_flush(usbp, ep);

    otgp->ie[ep].DIEPCTL = ctl |
                           DIEPCTL_TXFNUM(ep) |
                           DIEPCTL_MPL(usbp->epc[ep]->in_maxsize);
    otgp->DAEPINTEN |= DAEPINTEN_IEPIE(ep);
  }
  else {
    otgp->DIEPTFLEN[ep - 1] = 0x02000400; /* Reset value.*/
    otg_txfifo_flush(usbp, ep);
    otgp->ie[ep].DIEPCTL &= ~DIEPCTL_EPACT;
    otgp->DAEPINTEN &= ~DAEPINTEN_IEPIE(ep);
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

  /* Resets the FIFO memory allocator.*/
  otg_ram_reset(usbp);

  /* Disabling all endpoints.*/
  otg_disable_ep(usbp);
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
  uint32_t ctl;

  (void)usbp;

  ctl = usbp->otg->oe[ep].DOEPCTL;
  if (!(ctl & DOEPCTL_EPACT))
    return EP_STATUS_DISABLED;
  if (ctl & DOEPCTL_STALL)
    return EP_STATUS_STALLED;
  return EP_STATUS_ACTIVE;
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
  uint32_t ctl;

  (void)usbp;

  ctl = usbp->otg->ie[ep].DIEPCTL;
  if (!(ctl & DIEPCTL_EPACT))
    return EP_STATUS_DISABLED;
  if (ctl & DIEPCTL_STALL)
    return EP_STATUS_STALLED;
  return EP_STATUS_ACTIVE;
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

  memcpy(buf, usbp->epc[ep]->setup_buf, 8);
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
  uint32_t pcnt, rxsize;
  USBOutEndpointState *osp = usbp->epc[ep]->out_state;

  /* Transfer initialization.*/
  osp->totsize = osp->rxsize;
  if ((ep == 0) && (osp->rxsize > EP0_MAX_OUTSIZE))
      osp->rxsize = EP0_MAX_OUTSIZE;

  /* Transaction size is rounded to a multiple of packet size because the
     following requirement in the RM:
     "For OUT transfers, the transfer size field in the endpoint's transfer
     size register must be a multiple of the maximum packet size of the
     endpoint, adjusted to the Word boundary".*/
  pcnt   = (osp->rxsize + usbp->epc[ep]->out_maxsize - 1U) /
           usbp->epc[ep]->out_maxsize;
  rxsize = (pcnt * usbp->epc[ep]->out_maxsize + 3U) & 0xFFFFFFFCU;

  /*Setting up transaction parameters in DOEPLEN.*/
  usbp->otg->oe[ep].DOEPLEN = DOEPLEN_STPCNT(3) | DOEPLEN_PCNT(pcnt) |
                               DOEPLEN_TLEN(rxsize);

  /* Special case of isochronous endpoint.*/
  if ((usbp->epc[ep]->ep_mode & USB_EP_MODE_TYPE) == USB_EP_MODE_TYPE_ISOC) {
    /* Odd/even bit toggling for isochronous endpoint.*/
    if (usbp->otg->DSTAT & DSTAT_FNRSOF_ODD)
      usbp->otg->oe[ep].DOEPCTL |= DOEPCTL_SEVENFRM;
    else
      usbp->otg->oe[ep].DOEPCTL |= DOEPCTL_SODDFRM;
  }

  /* Starting operation.*/
  usbp->otg->oe[ep].DOEPCTL |= DOEPCTL_EPEN | DOEPCTL_CNAK;
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
  USBInEndpointState *isp = usbp->epc[ep]->in_state;

  /* Transfer initialization.*/
  isp->totsize = isp->txsize;
  if (isp->txsize == 0) {
    /* Special case, sending zero size packet.*/
    usbp->otg->ie[ep].DIEPLEN = DIEPLEN_PCNT(1) | DIEPLEN_TLEN(0);
  }
  else {
    if ((ep == 0) && (isp->txsize > EP0_MAX_INSIZE))
      isp->txsize = EP0_MAX_INSIZE;

    /* Normal case.*/
    uint32_t pcnt = (isp->txsize + usbp->epc[ep]->in_maxsize - 1) /
                    usbp->epc[ep]->in_maxsize;
    /* CHTODO: Support more than one packet per frame for isochronous transfers.*/
    usbp->otg->ie[ep].DIEPLEN = DIEPLEN_MCPF(1) | DIEPLEN_PCNT(pcnt) |
                                 DIEPLEN_TLEN(isp->txsize);
  }

  /* Special case of isochronous endpoint.*/
  if ((usbp->epc[ep]->ep_mode & USB_EP_MODE_TYPE) == USB_EP_MODE_TYPE_ISOC) {
    /* Odd/even bit toggling.*/
    if (usbp->otg->DSTAT & DSTAT_FNRSOF_ODD)
      usbp->otg->ie[ep].DIEPCTL |= DIEPCTL_SEVENFRM;
    else
      usbp->otg->ie[ep].DIEPCTL |= DIEPCTL_SODDFRM;
  }

  /* Starting operation.*/
  usbp->otg->ie[ep].DIEPCTL |= DIEPCTL_EPEN | DIEPCTL_CNAK;
  usbp->otg->DIEPFEINTEN |= DIEPFEINTEN_IEPTXFEIE(ep);
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

  usbp->otg->oe[ep].DOEPCTL |= DOEPCTL_STALL;
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

  usbp->otg->ie[ep].DIEPCTL |= DIEPCTL_STALL;
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

  usbp->otg->oe[ep].DOEPCTL &= ~DOEPCTL_STALL;
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

  usbp->otg->ie[ep].DIEPCTL &= ~DIEPCTL_STALL;
}

#endif /* HAL_USE_USB */

/** @} */
