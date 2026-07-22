/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio
              Copyright (C) 2020 Yaotian Feng

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
 * @brief   HT32 USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include "hal.h"

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define roundup2(x, m) (((x) + (m) - 1) & ~((m) - 1))

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if HT32_USB_USE_USB0 || defined(__DOXYGEN__)
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
    &ep0_state.out
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint32_t usb_get_int_flags(void){
    return USB->IER & USB->ISR;
}

static void usb_clear_int_flags(uint32_t flags){
    // flags are cleared by writing 1
    // except ESOFIF, which is written normally
    USB->ISR = flags ^ USBISR_ESOFIF;
}

static uint32_t usb_get_ep_int_flags(int ep){
    return USB->EP[ep].IER & USB->EP[ep].ISR;
}

static void usb_clear_ep_int_flags(int ep, uint32_t flags){
    // flags are cleared by writing 1
    USB->EP[ep].ISR = flags;
}

static size_t usb_epmem_alloc(USBDriver *usbp, size_t size) {
    const size_t epmo = usbp->epmem_next;

    usbp->epmem_next = roundup2(epmo + size, 4);
    osalDbgAssert(usbp->epmem_next <= 0x400, "EPSRAM exhausted");
    return epmo;
}

static void usb_packet_transmit(USBDriver *usbp, usbep_t ep, size_t n)
{
    const USBEndpointConfig * const epc = usbp->epc[ep];
    USBInEndpointState * const isp = epc->in_state;

    if (n > (size_t)epc->in_maxsize)
        n = (size_t)epc->in_maxsize;

    if ((USB->EP[ep].TCR & 0xffffU) == 0) {
        const uint32_t cfgr = USB->EP[ep].CFGR;
        volatile uint32_t * const EPSRAM = (void *)(USB_SRAM_BASE + (cfgr & 0x3ff));
        for (size_t i = 0; i < n; i += 4) {
            uint32_t word = 0;
            if (i + 0 < n)
                word |= (uint32_t)isp->txbuf[i+0]<<0;
            if (i + 1 < n)
                word |= (uint32_t)isp->txbuf[i+1]<<8;
            if (i + 2 < n)
                word |= (uint32_t)isp->txbuf[i+2]<<16;
            if (i + 3 < n)
                word |= (uint32_t)isp->txbuf[i+3]<<24;
            EPSRAM[i/4] = word;
        }

        USB->EP[ep].TCR = n;
        isp->txlastpktlen = n;
        USB->EP[ep].CSR &= USBEPnCSR_NAKTX;
    }
}

static size_t usb_packet_receive(USBDriver *usbp, usbep_t ep) {
    USBOutEndpointState *osp = usbp->epc[ep]->out_state;
    const size_t n = USB->EP[ep].TCR >> ((ep == 0) ? 16 : 0);
    const uint32_t cfgr = USB->EP[ep].CFGR;
    volatile uint32_t *const EPSRAM = (void *)(USB_SRAM_BASE + (cfgr & 0x3ff) + ((ep == 0) ? ((cfgr >> 10) & 0x7f) : 0));

    for (size_t i = 0; i < n; i += 4) {
        if(!osp->rxbuf)
            break;
        const uint32_t word = EPSRAM[i/4];
        if (i + 0 < n)
            osp->rxbuf[i + 0] = word >> 0;
        if (i + 1 < n)
            osp->rxbuf[i + 1] = word >> 8;
        if (i + 2 < n)
            osp->rxbuf[i + 2] = word >> 16;
        if (i + 3 < n)
            osp->rxbuf[i + 3] = word >> 24;
    }

    osp->rxbuf += n;
    osp->rxcnt += n;
    osp->rxsize -= n;
    osp->rxpkts--;
    return n;
}

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

#if (HT32_USB_USE_USB0 == TRUE) || defined(__DOXYGEN__) || 1

/**
 * @brief USB interrupt handler.
 * @isr
 */

OSAL_IRQ_HANDLER(HT32_USB_IRQ_VECTOR) {
    OSAL_IRQ_PROLOGUE();
    USBDriver *usbp = &USBD1;

    uint32_t isr = usb_get_int_flags();

    // Start of Frame Interrupt
    if(isr & USBISR_SOFIF){
        // Start of Frame callback
        _usb_isr_invoke_sof_cb(usbp);
        usb_clear_int_flags(USBISR_SOFIF);
    }

    // Suspend Interrupt
    if(isr & USBISR_SUSPIF){
        usb_clear_int_flags(USBISR_SUSPIF);
        // Suspend routine and event callback
        _usb_suspend(usbp);
#if 0
        USB->CSR &= USBCSR_DPPUEN; /* POWER_ON */
        USB->CSR |= USBCSR_GENRSM;
#endif
    }

    // Reset Interrupt
    if(isr & USBISR_URSTIF){
        // Reset routine and event callback
        _usb_reset(usbp);
        usb_clear_int_flags(USBISR_URSTIF);
    }

    // Resume Interrupt
    if(isr & USBISR_RSMIF){
        // Resume/Wakeup routine and callback
        _usb_wakeup(usbp);
        usb_clear_int_flags(USBISR_RSMIF);
    }

    // EP0 Interrupt
    if(isr & USBISR_EP0IF){
        uint32_t episr = usb_get_ep_int_flags(0);

#if 0
        // SETUP Token Received
        if(episr & USBEPnISR_STRXIF){
            usb_clear_ep_int_flags(0, USBEPnISR_STRXIF);
        }
#endif

        // SETUP Data Received
        if(episr & USBEPnISR_SDRXIF){
            // SETUP callback
            _usb_isr_invoke_setup_cb(usbp, 0);
            usb_clear_ep_int_flags(0, USBEPnISR_SDRXIF);
        }

#if 0
        // OUT Token Received
        if(episr & USBEPnISR_OTRXIF){
            usb_clear_ep_int_flags(0, USBEPnISR_OTRXIF);
        }
#endif

        // OUT Data Received
        if(episr & USBEPnISR_ODRXIF){
            USBOutEndpointState *osp = usbp->epc[0]->out_state;
            size_t n = usb_packet_receive(usbp, 0);
            if ((n < usbp->epc[0]->out_maxsize) || (osp->rxpkts == 0)) {
                // OUT callback
                _usb_isr_invoke_out_cb(usbp, 0);
            } else {
                USB->EP[0].CSR &= USBEPnCSR_NAKRX;
            }
            usb_clear_ep_int_flags(0, USBEPnISR_ODRXIF);
        }

#if 0
        // IN Token Received
        if(episr & USBEPnISR_ITRXIF){
            usb_clear_ep_int_flags(0, USBEPnISR_ITRXIF);
        }
#endif

        // IN Data Transmitted
        if(episr & USBEPnISR_IDTXIF){
            USBInEndpointState *isp = usbp->epc[0]->in_state;
            size_t n = isp->txlastpktlen;
            isp->txcnt += n;
            if (isp->txcnt < isp->txsize) {
                isp->txbuf += n;
                osalSysLockFromISR();
                usb_packet_transmit(usbp, 0, isp->txsize - isp->txcnt);
                osalSysUnlockFromISR();
            } else {
                // IN callback
                _usb_isr_invoke_in_cb(usbp, 0);
            }
            usb_clear_ep_int_flags(0, USBEPnISR_IDTXIF);
        }

#if 0
        // STALL Transmitted
        if(episr & USBEPnISR_STLIF){
            usb_clear_ep_int_flags(0, USBEPnISR_STLIF);
        }
#endif

        usb_clear_int_flags(USBISR_EP0IF);
    }

    // EP 1-7 Interrupt
    uint32_t mask = USBISR_EP1IF;
    for(int i = 1; i < 8; ++i){
        // EPn Interrupt
        if(isr & mask){
            uint32_t episr = usb_get_ep_int_flags(i);

            usb_clear_ep_int_flags(i, episr);
            usb_clear_int_flags(mask);

#if 0
            // OUT Token Received
            if(episr & USBEPnISR_OTRXIF){
                usb_clear_ep_int_flags(i, USBEPnISR_OTRXIF);
            }
#endif

            // OUT Data Received
            if(episr & USBEPnISR_ODRXIF){
                USBOutEndpointState *osp = usbp->epc[i]->out_state;
                size_t n = usb_packet_receive(usbp, i);
                if ((n < usbp->epc[i]->out_maxsize) || (osp->rxpkts == 0)) {
                    // OUT callback
                    _usb_isr_invoke_out_cb(usbp, i);
                } else {
                    USB->EP[i].CSR &= USBEPnCSR_NAKRX;
                }
            }

#if 0
            // IN Token Received
            if(episr & USBEPnISR_ITRXIF){
                usb_clear_ep_int_flags(i, USBEPnISR_ITRXIF);
            }
#endif

            // IN Data Transmitted
            if(episr & USBEPnISR_IDTXIF){
                USBInEndpointState *isp = usbp->epc[i]->in_state;
                size_t n = isp->txlastpktlen;
                isp->txcnt += n;
                if (isp->txcnt < isp->txsize) {
                    isp->txbuf += n;
                    osalSysLockFromISR();
                    usb_packet_transmit(usbp, i, isp->txsize - isp->txcnt);
                    osalSysUnlockFromISR();
                } else {
                    // IN callback
                    _usb_isr_invoke_in_cb(usbp, i);
                }
            }

#if 0
            // STALL Transmitted
            if(episr & USBEPnISR_STLIF){
                usb_clear_ep_int_flags(i, USBEPnISR_STLIF);
            }
#endif

        }
        mask = mask << 1;
    }

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
void usb_lld_init(void){
#if HT32_USB_USE_USB0 == TRUE
    /* Driver initialization.*/
    usbObjectInit(&USBD1);

    // USB prescaler
    CKCU->GCFGR = (CKCU->GCFGR & ~CKCU_GCFGR_USBPRE_MASK) | ((HT32_USB_PRESCALER - 1) << 22);
    // enable USB clock
    CKCU->AHBCCR |= CKCU_AHBCCR_USBEN;
#endif // HT32_USB_USE_USB1
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp){
    if (usbp->state == USB_STOP) {
        /* Enables the peripheral.*/
#if HT32_USB_USE_USB0 == TRUE
        if (&USBD1 == usbp) {
            // enable USB IRQ
            nvicEnableVector(USB_IRQn, HT32_USB_USB0_IRQ_PRIORITY);
            /* USBD_PowerUp */
            USB->CSR = USBCSR_DPWKEN | USBCSR_DPPUEN | USBCSR_LPMODE | USBCSR_PDWN;
            // enable usb interrupts
            USB->ISR = ~0U;
            USB->CSR &= ~USBCSR_DPWKEN;
            USB->IER = USBIER_UGIE | USBIER_SOFIE |
                USBIER_URSTIE | USBIER_RSMIE | USBIER_SUSPIE |
                USBIER_EP0IE;
        }
#endif // HT32_USB_USE_USB1
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
#if HT32_USB_USE_USB0 == TRUE
        if (&USBD1 == usbp) {
            nvicDisableVector(USB_IRQn);
            /* Resets the peripheral.*/
            RSTCU->AHBPRSTR = RSTCU_AHBPRSTR_USBRST;
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
    // USB Reset
    // Clear CSR, except for DP pull up
    USB->CSR &= USBCSR_DPPUEN;

    /* Post reset initialization.*/
    usbp->epmem_next = 8;

    /* EP0 initialization.*/
    usbp->epc[0] = &ep0config;
    usb_lld_init_endpoint(usbp, 0);

    USB->IER = USBIER_UGIE | USBIER_SOFIE |
        USBIER_URSTIE | USBIER_RSMIE | USBIER_SUSPIE |
        USBIER_EP0IE;
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
    USB->CSR |= USBCSR_ADRSET;
    USB->DEVAR = usbp->address & 0x7f;
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
    uint32_t cfgr = USBEPnCFGR_EPEN | ((uint32_t)ep<<24);
    size_t epmo;

    switch(epcp->ep_mode & USB_EP_MODE_TYPE){
        case USB_EP_MODE_TYPE_CTRL:
            break;
        case USB_EP_MODE_TYPE_ISOC:
            cfgr |= USBEPnCFGR_EPTYPE;
            break;
        case USB_EP_MODE_TYPE_BULK:
            break;
        case USB_EP_MODE_TYPE_INTR:
            break;
        default:
            return;
    }

    if (epcp->in_state != NULL) {
        epmo = usb_epmem_alloc(usbp, epcp->in_maxsize);
        cfgr |= epmo << 0;
        cfgr |= roundup2(epcp->in_maxsize, 4) << 10;
        cfgr |= USBEPnCFGR_EPDIR;
    }

    if (epcp->out_state != NULL) {
        epmo = usb_epmem_alloc(usbp, epcp->out_maxsize);
        if (ep > 0) {
            cfgr |= epmo << 0;
            cfgr |= roundup2(epcp->out_maxsize, 4) << 10;
        }
    }

    USB->EP[ep].CFGR = cfgr;
    USB->EP[ep].IER = (ep == 0) ?
      (USBEPnIER_SDRXIE|USBEPnIER_IDTXIE|USBEPnIER_ODRXIE) :
      (USBEPnIER_ODRXIE|USBEPnIER_IDTXIE);
    USB->IER |= (USBIER_EP0IE << ep);
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
    for(int i = 1; i < USB_MAX_ENDPOINTS; ++i) {
        USB->EP[i].CFGR &= ~USBEPnCFGR_EPEN;
        USB->IER &= ~(USBIER_EP0IE << i);
    }

    const uint32_t cfgr = USB->EP[0].CFGR;
    usbp->epmem_next = (cfgr & 0x3ff) + ((cfgr >> 10) & 0x7f);
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
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep)
{
    (void)usbp;

    if (ep > USB_MAX_ENDPOINTS)
        return EP_STATUS_DISABLED;
    if ((USB->EP[ep].CFGR & USBEPnCFGR_EPEN) == 0)
        return EP_STATUS_DISABLED;
    if ((USB->EP[ep].CSR & USBEPnCSR_STLRX) != 0)
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

  (void)usbp;

  if (ep > USB_MAX_ENDPOINTS)
    return EP_STATUS_DISABLED;
  if ((USB->EP[ep].CFGR & USBEPnCFGR_EPEN) == 0)
    return EP_STATUS_DISABLED;
  if ((USB->EP[ep].CSR & USBEPnCSR_STLRX) != 0)
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
    volatile uint32_t * const EPSRAM = (void *)(USB_SRAM_BASE + 0);
    for (size_t i = 0; i < 8; i += 4) {
        const uint32_t word = EPSRAM[i/4];
        buf[i + 0] = (word>>0);
        buf[i + 1] = (word>>8);
        buf[i + 2] = (word>>16);
        buf[i + 3] = (word>>24);
    }
    (void)usbp;
    (void)ep;
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
    USBOutEndpointState * const osp = usbp->epc[ep]->out_state;

    if (osp->rxsize == 0)
        osp->rxpkts = 1;
    else
        osp->rxpkts = (osp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
            usbp->epc[ep]->out_maxsize;
    USB->EP[ep].CSR &= USBEPnCSR_NAKRX;
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

  isp->txlastpktlen = 0;
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

  (void)usbp;

  USB->EP[ep].CSR = USBEPnCSR_STLRX;
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

  USB->EP[ep].CSR = USBEPnCSR_STLTX;
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

  USB->EP[ep].CSR &= USBEPnCSR_STLRX;
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

  USB->EP[ep].CSR &= USBEPnCSR_STLTX;
}

#endif /* HAL_USE_USB == TRUE */

/** @} */
