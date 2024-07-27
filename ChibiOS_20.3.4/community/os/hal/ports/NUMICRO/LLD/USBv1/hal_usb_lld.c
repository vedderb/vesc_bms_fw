/*
    Copyright (C) 2020 Alex Lewontin
    Copyright (C) 2021 Ein Terakawa

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
 * @brief   NUC123 USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include "hal.h"

/*
 * TODO:
 * As it stands, the behavior of hardware endpoint 6 (which corresponds to the
 * OUT direction of logical endpoint 3) may malfunction if hardware endpoint 5
 * (the IN direction of logical endpoint 2) is in use, due to a flaw in the
 * hardware (for more info, see the errata ER_6000_NUC123AN_EN_Rev.1.04.pdf).
 *
 * The only way to ensure both hardware endpoints function properly is to set
 * them both as OUT. Under the current alternating scheme, this would be tricky.
 * However, hardware endpoint 5 is currently configured as an IN endpoint, which
 * means hardware endpoint 6 will malfunction under any circumstances (from a
 * user's perspective, this means that logical endpoint 3 cannot handle outward
 * bound traffic).
 *
 * The first step in the fix is switching the alternation pattern so that
 * hardware endpoint 5 serves as an OUT endpoint. Then, logical endpoint 3 will
 * be able to handle IN traffic, and can handle two directional traffic IF it is
 * in ISOC mode.
 *
 * Eventually, it may be the case that the alternating scheme needs to be
 * abandoned, and some more sophisticated scheme for allocating hardware
 * endpoints adopted.
 */
/*
 * Solution to overcome the above mentioned shortcoming has been found.
 * That is to map Control-IN to endpoint 5 and Control-OUT to endpoint 6.
 * It is safe to assume interrupts for Control-IN and Control-OUT are not going
 * to occur at the same time. Because control transfers do not overwrap.
 * Other endpoints 0 1 2 3 4 and 7 can be assigned in whatever combination.
 */

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define NUC123_USB_HW_ENDPOINTS 8

#define NUC123_USBD_CFG_OUT (1UL << USBD_CFG_STATE_Pos)
#define NUC123_USBD_CFG_IN  (2UL << USBD_CFG_STATE_Pos)

#define USBD_SRAM_BASE ((volatile uint8_t*)(USBD_BASE + 0x100))

#define NUC123_USBD_BUSSTATUS_INACK       0UL
#define NUC123_USBD_BUSSTATUS_INNAK       1UL
#define NUC123_USBD_BUSSTATUS_OUTDATA0ACK 2UL
#define NUC123_USBD_BUSSTATUS_SETUPACK    3UL
#define NUC123_USBD_BUSSTATUS_OUTDATA1ACK 6UL
#define NUC123_USBD_BUSSTATUS_ISOEND      7UL

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if NUC123_USB_USE_USB1 || defined(__DOXYGEN__)
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
static const USBEndpointConfig ep0config = {USB_EP_MODE_TYPE_CTRL,
                                            _usb_ep0setup,
                                            _usb_ep0in,
                                            _usb_ep0out,
                                            0x40,
                                            0x40,
                                            &ep0_state.in,
                                            &ep0_state.out,
                                            1,
                                            ep0setup_buffer};

/**
 * @brief   Tracks the first unallocated word in the SRAM buffer
 */
static uint32_t sram_free_dword_offset = 1UL;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Converts a clock division factor to the appropriate bit setting
 */
#define NUC123_CLK_CLKDIV_USB(n) (((n)-1) << CLK_CLKDIV_USB_N_Pos)

/**
 * @name    Endpoint number convenience macros
 * @{
 */
#if NUC123_USB_WORKAROUND
// As work-around we map Control-OUT to EP6 and Control-IN to EP5 .
#define _HW_OUT_EPN(lepn)   (2 * (((lepn)+3)%4))
#define _HW_IN_EPN(lepn)    ((2 * (((lepn)+2)%4))+1)
#define LOGICAL_EPN(hwepn) ((((hwepn) + 3) / 2) % 4)
#else
#if defined(NUC123xxxANx)
// without workaround we use EP0 EP1 EP2 EP3 EP4 and EP7 for NUC123(AN)
#define _HW_OUT_EPN(lepn)   (2 * (lepn))
#define _HW_IN_EPN(lepn)    (2 * (((lepn)+3)%4) + 1)
#define LOGICAL_EPN(hwepn) (((hwepn)+1)%8 / 2)
#else
#define _HW_OUT_EPN(lepn)   (2 * (lepn))
#define _HW_IN_EPN(lepn)    (2 * (lepn) + 1)
#define LOGICAL_EPN(hwepn) ((hwepn) / 2)
#endif
#endif

#define _HW_EP(hwepn)       ((USBD->EP) + (hwepn))
#define HW_OUT_EP(lepn)    (_HW_EP(_HW_OUT_EPN(lepn)))
#define HW_IN_EP(lepn)     (_HW_EP(_HW_IN_EPN(lepn)))
/** @} */

/**
 * @brief   Convenience macro to round up to the nearest double word, used for SRAM allocation
 */
#define BYTES_TO_NEXT_DWORD(bytes) (((bytes) + 7) >> 3)

/**
 * @brief   Returns the minimum of two unsigned values. A safer implementation
 *          of the classic macro
 *
 * @param[in] x           An unsigned value
 * @param[in] y           An unsigned value
 *
 * @return                The smaller of x and y
 *
 * @notapi
 */
static inline unsigned min(unsigned x, unsigned y)
{
  return ((x > y) ? y : x);
}

/**
 * @brief   Returns the maximum of two unsigned values. A safer implementation
 *          of the classic macro
 *
 * @param[in] x           An unsigned value
 * @param[in] y           An unsigned value
 *
 * @return                The larger of x and y
 *
 * @notapi
 */
static inline unsigned max(unsigned x, unsigned y)
{
  return ((x > y) ? x : y);
}

/**
 * @brief   memcpy-like function that should be used when either the source
 *          or the destination is the dedicated USB SRAM buffer. Ensures the correct
 *          memory access sizes are used.
 *
 * @note    This is only the forward declaration. This function is defined in usb_memcpy.S
 *
 * @param[in] destination  Buffer to copy the data to
 * @param[in] source       Buffer to copy the data from
 * @param[in] num          Number of bytes to copy
 *
 * @return                 The number of bytes copied. This always exactly equals num, but
 *                         is returned purely for convenience.
 * @notapi
 */
size_t usb_memcpy(volatile void* destination, const volatile void* source,
                  size_t num);

/**
 * @brief   Common ISR code, serves the EP-related interrupts.
 *
 * @param[in] epn        endpoint number
 *
 * @notapi
 */
static void usb_serve_out_endpoint(uint32_t epn)
{
  USBDriver *const usbp = &USBD1;
  uint32_t mxpld = HW_OUT_EP(epn)->MXPLD;
  uint32_t rxsize_actual = min(mxpld, usbp->epc[epn]->out_state->rxsize - usbp->epc[epn]->out_state->rxcnt);

  usb_memcpy(usbp->epc[epn]->out_state->rxbuf +
                 usbp->epc[epn]->out_state->rxcnt,
             USBD_SRAM_BASE + (HW_OUT_EP(epn)->BUFSEG),
             rxsize_actual);
  usbp->epc[epn]->out_state->rxcnt += rxsize_actual;

  if ((rxsize_actual < usbp->epc[epn]->out_maxsize) ||
      (usbp->epc[epn]->out_state->rxcnt >=
       usbp->epc[epn]->out_state->rxsize)) {
    _usb_isr_invoke_out_cb(usbp, epn);
  } else {
    HW_OUT_EP(epn)->MXPLD = min(usbp->epc[epn]->out_maxsize,
                                usbp->epc[epn]->out_state->rxsize -
                                    usbp->epc[epn]->out_state->rxcnt);
  }
}

/**
 * @brief   Common ISR code, serves the EP-related interrupts.
 *
 * @param[in] epn        endpoint number
 *
 * @notapi
 */
static void usb_serve_in_endpoint(uint32_t epn)
{
  USBDriver *const usbp = &USBD1;

  usbp->epc[epn]->in_state->txcnt += HW_IN_EP(epn)->MXPLD;

  if (usbp->epc[epn]->in_state->txcnt >= usbp->epc[epn]->in_state->txsize) {
    _usb_isr_invoke_in_cb(usbp, epn);
    } else {
      HW_IN_EP(epn)->MXPLD = usb_memcpy(
          USBD_SRAM_BASE + (HW_IN_EP(epn)->BUFSEG),
          usbp->epc[epn]->in_state->txbuf + usbp->epc[epn]->in_state->txcnt,
          min(usbp->epc[epn]->in_maxsize,
              usbp->epc[epn]->in_state->txsize -
                  usbp->epc[epn]->in_state->txcnt));
    }
}

/**
 * @brief   Common ISR code, serves general events and calls the appropriate endpoint routine
 *
 * @param[in] usbp       pointer to a @p USBDriver object
 *
 * @notapi
 */
static void usb_lld_serve_interrupt(USBDriver* usbp)
{

  uint32_t intsts    = USBD->INTSTS;
  uint32_t bussts    = USBD->ATTR & 0xF;

  if (intsts & USBD_INTSTS_FLDET_STS_Msk) {
    if (USBD->FLDET & USBD_FLDET_FLDET_Msk) {
      USBD->ATTR |= (USBD_ATTR_PHY_EN_Msk | USBD_ATTR_USB_EN_Msk);
      usbConnectBus(&USBD1);
    } else {
      usbDisconnectBus(&USBD1);
      USBD->ATTR &= ~USBD_ATTR_USB_EN_Msk;
    }
    USBD->INTSTS = USBD_INTSTS_FLDET_STS_Msk;
  }

  if (intsts & USBD_INTSTS_WAKEUP_STS_Msk) {
    /* Clear event flag */
    USBD->INTSTS = USBD_INTSTS_WAKEUP_STS_Msk;
    _usb_wakeup(usbp);
  }

  if (intsts & USBD_INTSTS_BUS_STS_Msk) {
    /* Clear event flag */
    USBD->INTSTS = USBD_INTSTS_BUS_STS_Msk;

    if (bussts & USBD_ATTR_USBRST_Msk) {
      /* Bus reset */
      USBD->ATTR |= (USBD_ATTR_PHY_EN_Msk | USBD_ATTR_USB_EN_Msk);
      _usb_reset(usbp);
    }
    if (bussts & USBD_ATTR_SUSPEND_Msk) {
      /* Enable USB but disable PHY */
      USBD->ATTR &= ~USBD_ATTR_PHY_EN_Msk;
      _usb_suspend(usbp);
    }
    if (bussts & USBD_ATTR_RESUME_Msk) {
      /* Enable USB and enable PHY */
      USBD->ATTR |= (USBD_ATTR_PHY_EN_Msk | USBD_ATTR_USB_EN_Msk);
      _usb_wakeup(usbp);
    }
  }

  if (intsts & USBD_INTSTS_USB_STS_Msk) {
    if (intsts & USBD_INTSTS_SETUP_Msk) {
      /* Clear event flag */
      USBD->INTSTS = USBD_INTSTS_SETUP_Msk;

      /* Clear the data IN/OUT ready flag of control end-points */
      HW_IN_EP(0)->CFGP |= USBD_CFGP_CLRRDY_Msk;
      HW_OUT_EP(0)->CFGP |= USBD_CFGP_CLRRDY_Msk;

      HW_IN_EP(0)->CFG |= USBD_CFG_DSQ_SYNC_Msk;
      _usb_isr_invoke_setup_cb(&USBD1, 0);
    }

    /* EP events */
    if (intsts & USBD_INTSTS_EPEVT0_Msk) {
      /* Clear event flag */
      USBD->INTSTS = USBD_INTSTS_EPEVT0_Msk;
      usb_serve_out_endpoint(LOGICAL_EPN(0));
    }

    if (intsts & USBD_INTSTS_EPEVT1_Msk) {
      /* Clear event flag */
      USBD->INTSTS = (USBD_INTSTS_EPEVT1_Msk);
      usb_serve_in_endpoint(LOGICAL_EPN(1));
    }

    if (intsts & USBD_INTSTS_EPEVT2_Msk) {
      /* Clear event flag */
      USBD->INTSTS = (USBD_INTSTS_EPEVT2_Msk);
      usb_serve_out_endpoint(LOGICAL_EPN(2));
    }

    if (intsts & USBD_INTSTS_EPEVT3_Msk) {
      /* Clear event flag */
      USBD->INTSTS = (USBD_INTSTS_EPEVT3_Msk);
      usb_serve_in_endpoint(LOGICAL_EPN(3));
    }

    if (intsts & USBD_INTSTS_EPEVT4_Msk) {
      /* Clear event flag */
      USBD->INTSTS = (USBD_INTSTS_EPEVT4_Msk);
      usb_serve_out_endpoint(LOGICAL_EPN(4));
    }

#if NUC123_USB_WORKAROUND
    if (intsts & USBD_INTSTS_EPEVT5_Msk) {
      /* Clear event flag */
      /* in NUC123(AN) EP5-IN event also false triggers EP6 */
      USBD->INTSTS = (USBD_INTSTS_EPEVT5_Msk | USBD_INTSTS_EPEVT6_Msk);
      usb_serve_in_endpoint(LOGICAL_EPN(5));
      intsts &= ~USBD_INTSTS_EPEVT6_Msk;
    }
#else
    if (intsts & USBD_INTSTS_EPEVT5_Msk) {
      /* Clear event flag */
      USBD->INTSTS = (USBD_INTSTS_EPEVT5_Msk);
      usb_serve_in_endpoint(LOGICAL_EPN(5));
    }
#endif

#if NUC123_USB_HW_ENDPOINTS > 6
    if (intsts & USBD_INTSTS_EPEVT6_Msk) {
      /* Clear event flag */
      USBD->INTSTS = (USBD_INTSTS_EPEVT6_Msk);
      usb_serve_out_endpoint(LOGICAL_EPN(6));
    }

    if (intsts & USBD_INTSTS_EPEVT7_Msk) {
      /* Clear event flag */
      USBD->INTSTS = (USBD_INTSTS_EPEVT7_Msk);
      usb_serve_in_endpoint(LOGICAL_EPN(7));
    }
#endif
  }
}

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

#if NUC123_USB_USE_USB1 || defined(__DOXYGEN__)

/**
 * @brief   USB event handler
 *
 * @isr
 */
OSAL_IRQ_HANDLER(NUC123_USB1_HANDLER)
{
  OSAL_IRQ_PROLOGUE();
  usb_lld_serve_interrupt(&USBD1);
  OSAL_IRQ_EPILOGUE();
}

#endif /* NUC123_USB_USE_USB1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void)
{
  CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_USB_N_Msk)) |
                NUC123_CLK_CLKDIV_USB(NUC123_USBD_CLKDIV);

#if NUC123_USB_USE_USB1
  /* Driver initialization.*/
  usbObjectInit(&USBD1);
#endif /* NUC123_USB_USE_USB1 */
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver* usbp)
{
  uint32_t delay;

  if (usbp->state == USB_STOP) {
    /* Enables the peripheral.*/
    CLK->APBCLK |= CLK_APBCLK_USBD_EN_Msk;

#if NUC123_USB_USE_USB1
    if (&USBD1 == usbp) {
    }
#endif /* NUC123_USB_USE_USB1 */
  }
  /* Configures the peripheral.*/
  /* Reset procedure enforced on driver start.*/

  SYS->IPRSTC2 |= SYS_IPRSTC2_USBD_RST_Msk;
  for (delay = 0; delay < 0x800; ++delay)
    ;
  SYS->IPRSTC2 &= ~(SYS_IPRSTC2_USBD_RST_Msk);

  /* Post reset initialization.*/
  /* Initial USB engine */
  USBD->ATTR = USBD_ATTR_PWRDN_Msk | USBD_ATTR_DPPU_EN_Msk |
               USBD_ATTR_USB_EN_Msk | USBD_ATTR_PHY_EN_Msk;

  USBD->STBUFSEG = 0UL;

  USBD->INTSTS = USBD_INTSTS_BUS_STS_Msk | USBD_INTSTS_FLDET_STS_Msk |
                 USBD_INTSTS_USB_STS_Msk | USBD_INTSTS_WAKEUP_STS_Msk;

  USBD->INTEN |= (USBD_INTEN_BUS_IE_Msk | USBD_INTEN_FLDET_IE_Msk |
                  USBD_INTEN_USB_IE_Msk | USBD_INTEN_WAKEUP_IE_Msk |
                  USBD_INTEN_WAKEUP_EN_Msk);

  nvicEnableVector(USBD_IRQn, NUC123_USB_IRQ_PRIORITY);

  for (delay = 0; delay < 0x800; ++delay)
    ;
  usb_lld_reset(usbp);
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver* usbp)
{

  if (usbp->state == USB_READY) {
    /* Resets the peripheral. */

    /* Disables the peripheral. */
#if NUC123_USB_USE_USB1
    if (&USBD1 == usbp) {
    }
#endif /* NUC123_USB_USE_USB1 */

    usbDisconnectBus(usbp);
    CLK->APBCLK &= ~CLK_APBCLK_USBD_EN_Msk;
    nvicDisableVector(USBD_IRQn);
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver* usbp)
{
  sram_free_dword_offset = 1UL;
  USBD->FADDR = 0;
  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver* usbp)
{
  USBD->FADDR = USBD_FADDR_FADDR_Msk & usbp->address;
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver* usbp, usbep_t ep)
{
  if (usbp->epc[ep]->ep_mode == USB_EP_MODE_TYPE_ISOC) {
    osalDbgAssert(FALSE, "isochronous mode not yet supported");
    osalDbgAssert((usbp->epc[ep]->in_state == NULL) ||
                      (usbp->epc[ep]->out_state == NULL),
                  "isochronous EP cannot be IN and OUT");
  }

  /* If the out logical endpoint is used */
  if (usbp->epc[ep]->out_state) {
    switch (usbp->epc[ep]->ep_mode) {
    case USB_EP_MODE_TYPE_CTRL:
      HW_OUT_EP(ep)->CFG =
          ((ep << USBD_CFG_EP_NUM_Pos) & USBD_CFG_EP_NUM_Msk) |
          NUC123_USBD_CFG_OUT | USBD_CFG_CSTALL_Msk;
      break;
    case USB_EP_MODE_TYPE_ISOC:
      HW_OUT_EP(ep)->CFG =
          ((ep << USBD_CFG_EP_NUM_Pos) & USBD_CFG_EP_NUM_Msk) |
          NUC123_USBD_CFG_OUT | USBD_CFG_ISOCH_Msk;
      break;
    case USB_EP_MODE_TYPE_BULK:
    case USB_EP_MODE_TYPE_INTR:
    default:
      HW_OUT_EP(ep)->CFG =
          ((ep << USBD_CFG_EP_NUM_Pos) & USBD_CFG_EP_NUM_Msk) |
          NUC123_USBD_CFG_OUT;
    }
    HW_OUT_EP(ep)->BUFSEG = sram_free_dword_offset << USBD_BUFSEG_BUFSEG_Pos;
    sram_free_dword_offset +=
        BYTES_TO_NEXT_DWORD((uint32_t)usbp->epc[ep]->out_maxsize);

  } else {
    /* The only important bits here are STATE ([6:5]), and we just need
        to set them to 0 to disable the endpoint.
     */
    HW_OUT_EP(ep)->CFG = 0;
  }

  /* If the in logical endpoint is used */
  if (usbp->epc[ep]->in_state) {
    switch (usbp->epc[ep]->ep_mode) {
    case USB_EP_MODE_TYPE_CTRL:
      HW_IN_EP(ep)->CFG =
          ((ep << USBD_CFG_EP_NUM_Pos) & USBD_CFG_EP_NUM_Msk) |
          NUC123_USBD_CFG_IN | USBD_CFG_CSTALL_Msk;
      break;
    case USB_EP_MODE_TYPE_ISOC:
      HW_IN_EP(ep)->CFG =
          ((ep << USBD_CFG_EP_NUM_Pos) & USBD_CFG_EP_NUM_Msk) |
          NUC123_USBD_CFG_IN | USBD_CFG_ISOCH_Msk;
      break;
    case USB_EP_MODE_TYPE_BULK:
    case USB_EP_MODE_TYPE_INTR:
    default:
      HW_IN_EP(ep)->CFG =
          ((ep << USBD_CFG_EP_NUM_Pos) & USBD_CFG_EP_NUM_Msk) |
          NUC123_USBD_CFG_IN;
    }

    HW_IN_EP(ep)->BUFSEG = sram_free_dword_offset << USBD_BUFSEG_BUFSEG_Pos;
    sram_free_dword_offset +=
        BYTES_TO_NEXT_DWORD((uint32_t)usbp->epc[ep]->in_maxsize);
  } else {
    /* The only important bits here are STATE ([6:5]), and we just need
        to set them to 0 to disable the endpoint.
     */
    HW_IN_EP(ep)->CFG = 0;
  }
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver* usbp)
{
  for (uint8_t i = 0; i < NUC123_USB_HW_ENDPOINTS; ++i) {
    if (LOGICAL_EPN(i) != 0) {
      USBD->EP[i].CFGP |= USBD_CFGP_CLRRDY_Msk;
      USBD->EP[i].CFG &= ~USBD_CFG_STATE_Msk;
    }
  }

  sram_free_dword_offset = 1UL +
                           BYTES_TO_NEXT_DWORD(usbp->epc[0]->in_maxsize) +
                           BYTES_TO_NEXT_DWORD(usbp->epc[0]->out_maxsize);
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
usbepstatus_t usb_lld_get_status_out(USBDriver* usbp, usbep_t ep)
{

  (void)usbp;

  if (!(HW_OUT_EP(ep)->CFG & USBD_CFG_STATE_Msk)) {
    return EP_STATUS_DISABLED;
  }

  if (HW_OUT_EP(ep)->CFGP & USBD_CFGP_SSTALL_Msk) {
    return EP_STATUS_STALLED;
  }

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
usbepstatus_t usb_lld_get_status_in(USBDriver* usbp, usbep_t ep)
{
  (void)usbp;

  if (!(HW_IN_EP(ep)->CFG & USBD_CFG_STATE_Msk)) {
    return EP_STATUS_DISABLED;
  }

  if (HW_IN_EP(ep)->CFGP & USBD_CFGP_SSTALL_Msk) {
    return EP_STATUS_STALLED;
  }

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
void usb_lld_read_setup(USBDriver* usbp, usbep_t ep, uint8_t* buf)
{
  /* The setup packet should be exactly 8 bytes long.
    We skip the frills of memcpy because it is always word-aligned,
    and always 8 bytes long. However, we drop down into assembly because
    we need to guarantee we only ever use word accesses to SRAM
    (limitation of the hardware), and while I doubt any compiler
    would optimize 32-bit reads and writes to multiple ldrb/strb
    pairs, why not easily get the guarantee?
  */

  uint32_t rm, rn;

  __ASM volatile(
      "ldr %0, [%[src], #0]\n"
      "str %0, [%[dest], #0]\n"
      "ldr %0, [%[src], #4]\n"
      "str %0, [%[dest], #4]\n"
      : "=&r"(rm)                                  /* Output operands */
      : [src] "r"(USBD_SRAM_BASE), [dest] "r"(buf) /* Input operands */
      : "memory"                                   /* Clobber list */
  );

  (void)rn;
  (void)rm;
  (void)usbp;
  (void)ep;
  (void)buf;
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver* usbp, usbep_t ep)
{
  HW_OUT_EP(ep)->MXPLD =
      min(usbp->epc[ep]->out_state->rxsize, usbp->epc[ep]->out_maxsize);
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver* usbp, usbep_t ep)
{
  uint32_t txsize =
      min(usbp->epc[ep]->in_state->txsize, usbp->epc[ep]->in_maxsize);
  HW_IN_EP(ep)->MXPLD = usb_memcpy(USBD_SRAM_BASE + (HW_IN_EP(ep)->BUFSEG),
                                   usbp->epc[ep]->in_state->txbuf,
                                   txsize);
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver* usbp, usbep_t ep)
{
  (void)usbp;
  HW_OUT_EP(ep)->CFGP |= (USBD_CFGP_SSTALL_Msk | USBD_CFGP_CLRRDY_Msk);
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver* usbp, usbep_t ep)
{
  (void)usbp;
  HW_IN_EP(ep)->CFGP |= (USBD_CFGP_SSTALL_Msk | USBD_CFGP_CLRRDY_Msk);
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver* usbp, usbep_t ep)
{
  (void)usbp;
  HW_OUT_EP(ep)->CFGP &= ~USBD_CFGP_SSTALL_Msk;
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver* usbp, usbep_t ep)
{
  (void)usbp;
  HW_IN_EP(ep)->CFGP &= ~USBD_CFGP_SSTALL_Msk;
}

#endif /* HAL_USE_USB */

/** @} */
