/*
    ChibiOS - Copyright (C) 2020 Yaotian Feng / Codetector

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
 * @brief   LPC11Uxx USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include "hal.h"
#include <string.h>

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define roundup2(x, m) (((x) + (m) - 1) & ~((m) - 1))

#define LPC_USB_SRAM_START            0x20004000

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB- driver identifier.
 */
#if (LPC_USB_USE_USB1 == TRUE) || defined(__DOXYGEN__)
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
  0,
  NULL,
};

static struct {
  uint32_t entry[20];
} *USB_EPLIST = (void*)LPC_USB_SRAM_START;

#define EPLIST_ENTRY_BUFFER_RSHIFT            6U
#define EPLIST_ENTRY_BUFFER_POS               0U
#define EPLIST_ENTRY_BUFFER_MASK              (0xFFFF << EPLIST_ENTRY_BUFFER_POS)
#define EPLIST_ADDR(ADDR)                     \
  ((((uint32_t)(ADDR)) >> EPLIST_ENTRY_BUFFER_RSHIFT) & EPLIST_ENTRY_BUFFER_MASK)
#define EPLIST_ENTRY_NBYTES_POS               16U
#define EPLIST_ENTRY_NBYTES_MASK              (0x3FF << EPLIST_ENTRY_NBYTES_POS)
#define EPLIST_ENTRY_NBYTES(BYTES)            \
  (((BYTES) << EPLIST_ENTRY_NBYTES_POS) & EPLIST_ENTRY_NBYTES_MASK)
#define EPLIST_ENTRY_ACTIVE                   (1U << 31U)
#define EPLIST_ENTRY_DISABLE                  (1U << 30U)
#define EPLIST_ENTRY_STALL                    (1U << 29U)
#define EPLIST_ENTRY_TOGGLE_RESET             (1U << 28U)
#define EPLIST_ENTRY_RFTV                     (1U << 27U)
#define EPLIST_ENTRY_EPTYPE                   (1U << 26U)
/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static size_t usb_ep_malloc(USBDriver *usbp, size_t size, size_t alignment) {
  size_t current = usbp->epmem_next;
  if ((current & (alignment - 1)) != 0) { // Alignment issue
    current = (current + alignment) & (~(alignment - 1));
  }
  const size_t epmo = current;
  usbp->epmem_next = current + size;
  osalDbgAssert(usbp->epmem_next <= (LPC_USB_SRAM_START + 0x800), "UBSSRAM exhausted");
  return epmo;
}

static void usb_packet_transmit(USBDriver *usbp, usbep_t ep, size_t n)
{
  const USBEndpointConfig * const epc = usbp->epc[ep];
  USBInEndpointState * const isp = epc->in_state;

  if (n > epc->in_maxsize) {
    n = epc->in_maxsize;
  }

  if (ep == 0) {
    // When EP0 IN is received, set ACTIVE bit on both EP0 IN
    // and OUT.
    USB_EPLIST->entry[0] |= EPLIST_ENTRY_ACTIVE;
  }

  /* Get EP command/status List, update the length field and data pointer. */
  USB_EPLIST->entry[ep * 4 + 2] &= ~(0x3FFFFFF | EPLIST_ENTRY_STALL | EPLIST_ENTRY_ACTIVE);
  USB_EPLIST->entry[ep * 4 + 2] |= EPLIST_ENTRY_NBYTES(n) |
    EPLIST_ADDR(usbp->epn_buffer[ep * 2 + 1]);
  if (n > 0)
    memcpy(usbp->epn_buffer[ep*2 + 1], isp->txbuf, n);
  isp->txlastpktlen = n;
  USB_EPLIST->entry[ep * 4 + 2] |= EPLIST_ENTRY_ACTIVE;
}

static size_t usb_packet_receive(USBDriver *usbp, usbep_t ep) {
  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBOutEndpointState *osp = usbp->epc[ep]->out_state;
  uint32_t n = (USB_EPLIST->entry[4 * ep] & EPLIST_ENTRY_NBYTES_MASK) >> EPLIST_ENTRY_NBYTES_POS;
  n = epcp->out_maxsize - n;
  if (osp->rxbuf != NULL && n > 0) {
    memcpy(osp->rxbuf, usbp->epn_buffer[ep * 2], n);
    osp->rxbuf += n;
  }

  if (osp->rxpkts > 0)
    osp->rxpkts -= 1;
  osp->rxcnt += n;
  osp->rxsize -= n;

  USB_EPLIST->entry[4 * ep] &= ~(0x3FFFFFF | EPLIST_ENTRY_STALL | EPLIST_ENTRY_ACTIVE);
  if (osp->rxsize > 0) {
    // ReSetup for recieve
    USB_EPLIST->entry[4 * ep] |= EPLIST_ENTRY_NBYTES(epcp->out_maxsize) |
      EPLIST_ADDR(usbp->epn_buffer[ep * 2]) | EPLIST_ENTRY_ACTIVE;
  } else {
    if (ep != 0) {
      USB_EPLIST->entry[4 * ep] |= EPLIST_ENTRY_STALL;
    }
  }

  return n;
}


/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/
#ifndef LPC_USB_IRQ_VECTOR
#error "LPC_USB_IRQ_VECTOR not defined"
#endif
OSAL_IRQ_HANDLER(LPC_USB_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  USBDriver *usbp = &USBD1;

  uint32_t isr = (LPC_USB->INTSTAT & LPC_USB->INTEN);
  LPC_USB->INTSTAT &= 0xFFFFFFFF; // Clear Flags
  #define devstat   (LPC_USB->DEVCMDSTAT)

  // SOF
  if (isr & USB_INT_FRAME_INT) {
    _usb_isr_invoke_sof_cb(usbp);
  }

  if (isr & USB_INT_DEV_INT) {
    if (devstat & USB_DEVCMDSTAT_DSUS_C) {
      if (devstat & USB_DEVCMDSTAT_DCON) {
        if (devstat & USB_DEVCMDSTAT_DSUP) {
          // Suspend
          _usb_suspend(usbp);
        } else {
          // Wakeup
          _usb_wakeup(usbp);
        }
      }
    }

    if (devstat & USB_DEVCMDSTAT_DRES_C) {
      LPC_USB->DEVCMDSTAT |= USB_DEVCMDSTAT_DRES_C;
      _usb_reset(usbp);
    }
  } // USB_INT_DEV_INT

  for (int ep = 0; ep < 5; ++ep)
  {
    // EP0 OUT (Setup)
    if (isr & (USB_INT_EPn_INT << (2 * ep))) {
      if (devstat & USB_DEVCMDSTAT_SETUP) {
        _usb_isr_invoke_setup_cb(usbp, ep);
      } else {
        // OUT endpoint, receive
        USBOutEndpointState *osp = usbp->epc[ep]->out_state;
        size_t actual_rx = 0;
        if (osp->rxsize > 0) {
          osalSysLockFromISR();
          actual_rx = usb_packet_receive(usbp, ep);
          osalSysUnlockFromISR();
        }
        if (osp->rxsize == 0) { // TODO: Check if this is correct
          _usb_isr_invoke_out_cb(usbp, ep);
        } else if (actual_rx < usbp->epc[ep]->out_maxsize && ep != 0) {
          _usb_isr_invoke_out_cb(usbp, ep);
        }
      }
    }

    // EP0 IN
    if (isr & (USB_INT_EPn_INT << (2 * ep + 1))) {
      USBInEndpointState *isp = usbp->epc[ep]->in_state;
      size_t n = isp->txlastpktlen;
      isp->txcnt += n;
      if (isp->txcnt < isp->txsize) {
        isp->txbuf += n;
        osalSysLockFromISR();
        usb_packet_transmit(usbp, ep, isp->txsize - isp->txcnt);
        osalSysUnlockFromISR();
      } else {
        // IN callback
        _usb_isr_invoke_in_cb(usbp, ep);
      }
    }
  }

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
  void usb_lld_init(void) {

#if LPC_USB_USE_USB1 == TRUE
  /* Driver initialization.*/
  usbObjectInit(&USBD1);
  // Enable Clock
  LPC_SYSCON->SYSAHBCLKCTRL |=
  (
    SYSCON_SYSAHBCLKCTRL_USB | SYSCON_SYSAHBCLKCTRL_USBRAM |
    SYSCON_SYSAHBCLKCTRL_RAM0
    );

  USBD1.epmem_next = (0x50 + LPC_USB_SRAM_START);

#if defined(LPC_MAINCLK_FREQUENCY) && LPC_MAINCLK_FREQUENCY == 48000000U
// TODO: Implement proper PLL support
  LPC_SYSCON->PDRUNCFG &= ~SYSCON_PDRUNCFG_USBPLL_PD;
  LPC_SYSCON->USBPLLCLKSEL = SYSCON_USBPLLCLKSEL_SYSOSC;
  LPC_SYSCON->USBPLLCLKUEN = 0;
  LPC_SYSCON->USBPLLCLKUEN = SYSCON_USBPLLCLKUEN_ENA;
  LPC_SYSCON->USBPLLCTRL = LPC_SYSCON->SYSPLLCTRL; // BIG HACK. Steal Main PLL CFG
  while(LPC_SYSCON->USBPLLSTAT == 0){}
  LPC_SYSCON->USBCLKSEL = SYSCON_USBCLKSEL_USBPLLOUT;
  LPC_SYSCON->USBCLKDIV = 1; // Divide By 1
#else // LPC_MAINCLK_FREQUENCY
#error "USB Currently requires LPC_MAINCLK_FREQUENCY = 48MHz"
#endif // LPC_MAINCLK_FREQUENCY


#endif // LPC_USB_USE_USB1 == TRUE
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
    /* Enables the peripheral.*/
#if LPC_USB_USE_USB1 == TRUE
    if (&USBD1 == usbp) {
      // USB Transiver Powerup
      LPC_SYSCON->PDRUNCFG &= ~SYSCON_PDRUNCFG_USBPAD_PD;

      // Enable Vector
      #if !defined(LPC_USB_USB1_IRQ_PRIORITY)
      #error "LPC_USB_USB1_IRQ_PRIORITY is not defined"
      #endif
      nvicEnableVector(USB_IRQn, LPC_USB_USB1_IRQ_PRIORITY);
    }
#endif
    usb_lld_reset(usbp);
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {

  if (usbp->state == USB_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if LPC_USB_USE_USB1 == TRUE
    if (&USBD1 == usbp) {
      nvicDisableVector(USB_IRQn);
      LPC_USB->INTEN = 0;
      LPC_SYSCON->PDRUNCFG |= SYSCON_PDRUNCFG_USBPAD_PD;
      LPC_USB->DEVCMDSTAT &= ~USB_DEVCMDSTAT_DEV_EN;
    }
#endif
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
  // cfg Interrupt routing to IRQ
  LPC_USB->INTROUTING = 0;
  LPC_USB->EPINUSE = 0;
  LPC_USB->EPSKIP = 0;
  LPC_USB->EPBUFCFG = 0;
  // Points USB Buffers to correct places
  LPC_USB->EPLISTSTART = LPC_USB_SRAM_START & USB_EPLISTSTART_MASK;
  LPC_USB->DATABUFSTART = LPC_USB_SRAM_START & USB_DATABUFSTART_MASK;

  // Clear Existing Interrupts
  LPC_USB->INTSTAT |= USB_INT_DEV_INT | USB_INT_FRAME_INT | USB_INT_EP_ALL_INT;
  // Setup Interrupt Masks
  LPC_USB->INTEN = USB_INT_DEV_INT | 0b11;
  // SOF only if there is a handler registered.
  if ((usbp)->config->sof_cb != NULL) {
    LPC_USB->INTEN |= USB_INT_FRAME_INT;
  }


  // Reset Allocator
  usbp->epmem_next = (0x50 + LPC_USB_SRAM_START);

  usbp->setup_buffer = NULL;
  for (int i = 0; i < 10; ++i)
  {
    usbp->epn_buffer[i] = NULL;
  }

  // Disable all endpoints
  for (int i = 0; i < 16; ++i)
  {
    USB_EPLIST->entry[i + 4] = EPLIST_ENTRY_DISABLE;
  }

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);

  LPC_USB->DEVCMDSTAT |= USB_DEVCMDSTAT_DEV_EN; // USB Start Running
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
  LPC_USB->DEVCMDSTAT  &= ~USB_DEVCMDSTAT_DEVADDR_MASK;
  LPC_USB->DEVCMDSTAT  |= (USB_DEVCMDSTAT_DEV_EN |
  (usbp->address & USB_DEVCMDSTAT_DEVADDR_MASK));
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
  if(ep > USB_MAX_ENDPOINTS)
    return;

  const USBEndpointConfig *epcp = usbp->epc[ep];
  uint32_t usbep_cfg = 0;
  switch(epcp->ep_mode & USB_EP_MODE_TYPE) {
    case USB_EP_MODE_TYPE_CTRL:
      break;
    case USB_EP_MODE_TYPE_ISOC:
      usbep_cfg |= EPLIST_ENTRY_EPTYPE;
      break;
    case USB_EP_MODE_TYPE_BULK:
      break;
    case USB_EP_MODE_TYPE_INTR:
      break;
    default:
      break;
  }
  if (epcp->out_state != NULL || ep == 0) {
    uint32_t ep_mem = usb_ep_malloc(usbp, epcp->out_maxsize, 64);
    usbp->epn_buffer[(2 * ep)] = (void *)ep_mem;

    USB_EPLIST->entry[(4 * ep)] = usbep_cfg | EPLIST_ADDR(ep_mem)
      | EPLIST_ENTRY_NBYTES(epcp->out_maxsize);

    LPC_USB->INTEN |= USB_INT_EP((ep * 2));
  }

  if (ep == 0) {
    // Allocate Setup Bytes
    uint32_t ep_mem = usb_ep_malloc(usbp, 8, 64);
    usbp->setup_buffer = (void *)ep_mem;
    USB_EPLIST->entry[1] = EPLIST_ADDR(ep_mem);
  }

  if (epcp->in_state != NULL || ep == 0) {
    uint32_t ep_mem = usb_ep_malloc(usbp, epcp->in_maxsize, 64);
    usbp->epn_buffer[(2 * ep) + 1] = (void *)ep_mem;
    USB_EPLIST->entry[(4 * ep) + 2] = usbep_cfg | EPLIST_ADDR(ep_mem)
      | EPLIST_ENTRY_NBYTES(epcp->in_maxsize);
    LPC_USB->INTEN |= USB_INT_EP(((ep * 2) + 1));
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
  if (usbp != &USBD1)
    return;
  LPC_USB->EPSKIP = 0xFFFFFFFF;
  LPC_USB->INTEN &= (~USB_INT_EP_ALL_INT) | USB_INT_EP(0) | USB_INT_EP(1);
  while (LPC_USB->EPSKIP) {}
  for (int i = 1; i < 5; ++i) // 4 EPs needs to be disabled, EP0 can't
  {
    USB_EPLIST->entry[i * 4] &= ~EP_STATUS_ACTIVE;
    USB_EPLIST->entry[i * 4] |= EP_STATUS_DISABLED;
    USB_EPLIST->entry[i * 4 + 2] &= ~EP_STATUS_ACTIVE;
    USB_EPLIST->entry[i * 4 + 2] |= EP_STATUS_DISABLED;
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
  if (usbp != &USBD1)
    return EP_STATUS_DISABLED;
  if (USB_EPLIST->entry[ep * 4] & EPLIST_ENTRY_DISABLE) {
    return EP_STATUS_DISABLED;
  } else if (USB_EPLIST->entry[ep * 4] & EPLIST_ENTRY_STALL) {
    return EP_STATUS_STALLED;
  } else {
    return EP_STATUS_ACTIVE;
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
  if (usbp != &USBD1)
    return EP_STATUS_DISABLED;
  if (USB_EPLIST->entry[ep * 4 + 2] & EPLIST_ENTRY_DISABLE) {
      return EP_STATUS_DISABLED;
  } else if (USB_EPLIST->entry[ep * 4 + 2] & EPLIST_ENTRY_STALL) {
    return EP_STATUS_STALLED;
  } else {
    return EP_STATUS_ACTIVE;
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
  if (usbp == &USBD1) {
    if (ep == 0) {
      while(usbp->setup_buffer == 0){}
      /* Check/Clear STALL on both EP0 IN and OUT when SETUP is received. */
      USB_EPLIST->entry[0] &= ~(EPLIST_ENTRY_STALL | EPLIST_ENTRY_ACTIVE); // EP0OUT
      USB_EPLIST->entry[2] &= ~(EPLIST_ENTRY_STALL | EPLIST_ENTRY_ACTIVE); // EP0IN
      LPC_USB->DEVCMDSTAT |= USB_DEVCMDSTAT_SETUP; // Clear SETUP
      memcpy(buf, usbp->setup_buffer, 8);
      USB_EPLIST->entry[1] = EPLIST_ADDR(usbp->setup_buffer);
    }
  }
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
  USBOutEndpointState *osp = usbp->epc[ep]->out_state;
  const USBEndpointConfig *epcp = usbp->epc[ep];

  if (osp->rxsize == 0) {         /* Special case for zero sized packets.*/
    osp->rxpkts = 1;
  } else {
    osp->rxpkts = (uint16_t)((osp->rxsize + epcp->out_maxsize - 1) /
                             epcp->out_maxsize);
  }

  USB_EPLIST->entry[ep * 4] &= ~(0x3FFFFFF | EPLIST_ENTRY_STALL | EPLIST_ENTRY_ACTIVE);
  USB_EPLIST->entry[ep * 4] |= EPLIST_ENTRY_ACTIVE |
    EPLIST_ENTRY_NBYTES(epcp->out_maxsize) |
    EPLIST_ADDR(usbp->epn_buffer[ep * 2]);

  if (ep == 0)
    LPC_USB->DEVCMDSTAT |= USB_DEVCMDSTAT_INTONNAK_CO;
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
  USBInEndpointState * const isp = usbp->epc[ep]->in_state;
  usb_packet_transmit(usbp, ep, isp->txsize);
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
  if (usbp != &USBD1)
    return;
  const uint32_t skip_flag = 1U << (ep * 2);
  if (USB_EPLIST->entry[ep * 4] & EPLIST_ENTRY_ACTIVE) {
    LPC_USB->EPSKIP = skip_flag;
  }
  while(LPC_USB->EPSKIP & skip_flag) {} // Wait for not active
  USB_EPLIST->entry[ep*4] |= EPLIST_ENTRY_STALL;
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
  if (usbp != &USBD1)
    return;
  const uint32_t skip_flag = 1U << (ep * 2 + 1);
  if (USB_EPLIST->entry[ep * 4 + 2] & EPLIST_ENTRY_ACTIVE) {
    LPC_USB->EPSKIP = skip_flag;
  }
  while(LPC_USB->EPSKIP & skip_flag) {} // Wait for not active
  USB_EPLIST->entry[ep * 4 + 2] |= EPLIST_ENTRY_STALL;
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
  if (usbp != &USBD1)
    return;
  const uint32_t skip_flag = 1U << (ep * 2);
  if (USB_EPLIST->entry[ep * 4] & EPLIST_ENTRY_ACTIVE) {
    LPC_USB->EPSKIP = skip_flag;
  }
  while(LPC_USB->EPSKIP & skip_flag) {} // Wait for not active
  USB_EPLIST->entry[ep*4] &= ~EPLIST_ENTRY_STALL;
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
  if (usbp != &USBD1)
    return;
  const uint32_t skip_flag = 1U << (ep * 2 + 1);
  if (USB_EPLIST->entry[ep * 4 + 2] & EPLIST_ENTRY_ACTIVE) {
    LPC_USB->EPSKIP = skip_flag;
  }
  while(LPC_USB->EPSKIP & skip_flag) {} // Wait for not active
  USB_EPLIST->entry[ep * 4 + 2] &= ~EPLIST_ENTRY_STALL;
}

#endif /* HAL_USE_USB == TRUE */

/** @} */
