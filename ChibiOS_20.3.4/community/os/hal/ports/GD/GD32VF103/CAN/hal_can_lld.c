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
 * @file    CAN/hal_can_lld.c
 * @brief   GD32 CAN subsystem low level driver source.
 *
 * @addtogroup CAN
 * @{
 */

#include "hal.h"

#if HAL_USE_CAN || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*
 * Addressing differences in the headers, they seem unable to agree on names.
 */
#if GD32_CAN_USE_CAN0
#if !defined(CAN0)
#define CAN0 CAN
#endif
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief CAN0 driver identifier.*/
#if GD32_CAN_USE_CAN0 || defined(__DOXYGEN__)
CANDriver CAND1;
#endif

/** @brief CAN1 driver identifier.*/
#if GD32_CAN_USE_CAN1 || defined(__DOXYGEN__)
CANDriver CAND2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
/**
 * @brief   Programs the filters of CAN 1 and CAN 2.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] can2sb    number of the first filter assigned to CAN1
 * @param[in] num       number of entries in the filters array, if zero then
 *                      a default filter is programmed
 * @param[in] cfp       pointer to the filters array, can be @p NULL if
 *                      (num == 0)
 *
 * @notapi
 */
static void can_lld_set_filters(CANDriver* canp,
                                uint32_t can2sb,
                                uint32_t num,
                                const CANFilter *cfp) {

#if GD32_CAN_USE_CAN1
  if (canp == &CAND2) {
    /* Set handle to CAN0, because CAN0 manages the filters of CAN1.*/
    canp = &CAND1;
  }
#endif

  /* Temporarily enabling CAN clock.*/
#if GD32_CAN_USE_CAN0
  if (canp == &CAND1) {
    rcuEnableCAN0(true);
    /* Filters initialization.*/
    canp->can->FCTL = (canp->can->FCTL & 0xFFFF0000) | CAN_FCTL_FLD;
    canp->can->FCTL = (canp->can->FCTL & 0xFFFF0000) | (can2sb << 8) | CAN_FCTL_FLD;
  }
#endif

  if (num > 0) {
    uint32_t i, fmask;

    /* All filters cleared.*/
    canp->can->FW = 0;
    canp->can->FMCFG = 0;
    canp->can->FSCFG = 0;
    canp->can->FAFIFO = 0;

#if GD32_CAN_USE_CAN0
    if (canp == &CAND1) {
      for (i = 0; i < GD32_CAN_MAX_FILTERS; i++) {
        canp->can->sFilterRegister[i].FR1 = 0;
        canp->can->sFilterRegister[i].FR2 = 0;
      }
    }
#endif

    /* Scanning the filters array.*/
    for (i = 0; i < num; i++) {
      fmask = 1 << cfp->filter;
      if (cfp->mode)
        canp->can->FMCFG |= fmask;
      if (cfp->scale)
        canp->can->FSCFG |= fmask;
      if (cfp->assignment)
        canp->can->FAFIFO |= fmask;
      canp->can->sFilterRegister[cfp->filter].FR1 = cfp->register1;
      canp->can->sFilterRegister[cfp->filter].FR2 = cfp->register2;
      canp->can->FW |= fmask;
      cfp++;
    }
  }
  else {
    /* Setting up a single default filter that enables everything for both
       CANs.*/
    canp->can->sFilterRegister[0].FR1 = 0;
    canp->can->sFilterRegister[0].FR2 = 0;
#if GD32_CAN_USE_CAN1
    if (canp == &CAND1) {
      canp->can->sFilterRegister[can2sb].FR1 = 0;
      canp->can->sFilterRegister[can2sb].FR2 = 0;
    }
#endif
    canp->can->FMCFG = 0;
    canp->can->FAFIFO = 0;
    canp->can->FSCFG = 1;
    canp->can->FW = 1;
#if GD32_CAN_USE_CAN1
    if (canp == &CAND1) {
      canp->can->FSCFG |= 1 << can2sb;
      canp->can->FW |= 1 << can2sb;
    }
#endif
  }
  canp->can->FCTL &= ~CAN_FCTL_FLD;

  /* Clock disabled, it will be enabled again in can_lld_start().*/
  /* Temporarily enabling CAN clock.*/
#if GD32_CAN_USE_CAN0
  if (canp == &CAND1) {
    rcuDisableCAN0();
  }
#endif
}

/**
 * @brief   Common TX ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_tx_handler(CANDriver *canp) {
  uint32_t tstat;
  eventflags_t flags;

  /* Clearing IRQ sources.*/
  tstat = canp->can->TSTAT;
  canp->can->TSTAT = tstat;

  /* Flags to be signaled through the TX event source.*/
  flags = 0U;

  /* Checking mailbox 0.*/
  if ((tstat & CAN_TSTAT_MTF0) != 0U) {
    if ((tstat & (CAN_TSTAT_MAL0 | CAN_TSTAT_MTE0)) != 0U) {
      flags |= CAN_MAILBOX_TO_MASK(1U) << 16U;
    }
    else {
      flags |= CAN_MAILBOX_TO_MASK(1U);
    }
  }

  /* Checking mailbox 1.*/
  if ((tstat & CAN_TSTAT_MTF1) != 0U) {
    if ((tstat & (CAN_TSTAT_MAL1 | CAN_TSTAT_MTE1)) != 0U) {
      flags |= CAN_MAILBOX_TO_MASK(2U) << 16U;
    }
    else {
      flags |= CAN_MAILBOX_TO_MASK(2U);
    }
  }

  /* Checking mailbox 2.*/
  if ((tstat & CAN_TSTAT_MTF2) != 0U) {
    if ((tstat & (CAN_TSTAT_MAL2 | CAN_TSTAT_MTE2)) != 0U) {
      flags |= CAN_MAILBOX_TO_MASK(3U) << 16U;
    }
    else {
      flags |= CAN_MAILBOX_TO_MASK(3U);
    }
  }

  /* Signaling flags and waking up threads waiting for a transmission slot.*/
  _can_tx_empty_isr(canp, flags);
}

/**
 * @brief   Common RX0 ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_rx0_handler(CANDriver *canp) {
  uint32_t rfifo0;

  rfifo0 = canp->can->RFIFO0;
  if ((rfifo0 & CAN_RFIFO0_RFL0) > 0) {
    /* No more receive events until the queue 0 has been emptied.*/
    canp->can->INTEN &= ~CAN_INTEN_RFNEIE0;
    _can_rx_full_isr(canp, CAN_MAILBOX_TO_MASK(1U));
  }
  if ((rfifo0 & CAN_RFIFO0_RFO0) > 0) {
    /* Overflow events handling.*/
    canp->can->RFIFO0 = CAN_RFIFO0_RFO0;
    _can_error_isr(canp, CAN_OVERFLOW_ERROR);
  }
}

/**
 * @brief   Common RX1 ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_rx1_handler(CANDriver *canp) {
  uint32_t rfifo1;

  rfifo1 = canp->can->RFIFO1;
  if ((rfifo1 & CAN_RFIFO1_RFL1) > 0) {
    /* No more receive events until the queue 0 has been emptied.*/
    canp->can->INTEN &= ~CAN_INTEN_RFNEIE1;
    _can_rx_full_isr(canp, CAN_MAILBOX_TO_MASK(2U));
  }
  if ((rfifo1 & CAN_RFIFO1_RFO1) > 0) {
    /* Overflow events handling.*/
    canp->can->RFIFO1 = CAN_RFIFO1_RFO1;
    _can_error_isr(canp, CAN_OVERFLOW_ERROR);
  }
}

/**
 * @brief   Common SCE ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_sce_handler(CANDriver *canp) {
  uint32_t stat;

  /* Clearing IRQ sources.*/
  stat = canp->can->STAT;
  canp->can->STAT = stat;

  /* Wakeup event.*/
#if CAN_USE_SLEEP_MODE
  if (stat & CAN_STAT_WUIF) {
    canp->state = CAN_READY;
    canp->can->CTL &= ~CAN_CTL_SLPWMOD;
    _can_wakeup_isr(canp);
  }
#endif /* CAN_USE_SLEEP_MODE */
  /* Error event.*/
  if (stat & CAN_STAT_ERRIF) {
    eventflags_t flags;
    uint32_t err = canp->can->ERR;

#if GD32_CAN_REPORT_ALL_ERRORS
    flags = (eventflags_t)(err & 7);
    if ((err & CAN_ERR_ERRN) > 0)
      flags |= CAN_FRAMING_ERROR;
#else
    flags = 0;
#endif

    /* The content of the ESR register is copied unchanged in the upper
       half word of the listener flags mask.*/
    _can_error_isr(canp, flags | (eventflags_t)(err << 16U));
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if GD32_CAN_USE_CAN0 || defined(__DOXYGEN__)
#if !defined(GD32_CAN0_TX_HANDLER)
#error "GD32_CAN0_TX_HANDLER not defined"
#endif
#if !defined(GD32_CAN0_RX0_HANDLER)
#error "GD32_CAN0_RX0_HANDLER not defined"
#endif
#if !defined(GD32_CAN0_RX1_HANDLER)
#error "GD32_CAN0_RX1_HANDLER not defined"
#endif
#if !defined(GD32_CAN0_EWMC_HANDLER)
#error "GD32_CAN0_EWMC_HANDLER not defined"
#endif

/**
 * @brief   CAN0 TX interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_CAN0_TX_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  can_lld_tx_handler(&CAND1);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   CAN0 RX0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_CAN0_RX0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  can_lld_rx0_handler(&CAND1);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   CAN0 RX1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_CAN0_RX1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  can_lld_rx1_handler(&CAND1);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   CAN0 SCE interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_CAN0_EWMC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  can_lld_sce_handler(&CAND1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_CAN_USE_CAN0 */

#if GD32_CAN_USE_CAN1 || defined(__DOXYGEN__)
#if !defined(GD32_CAN0_TX_HANDLER)
#error "GD32_CAN0_TX_HANDLER not defined"
#endif
#if !defined(GD32_CAN0_RX0_HANDLER)
#error "GD32_CAN0_RX0_HANDLER not defined"
#endif
#if !defined(GD32_CAN0_RX1_HANDLER)
#error "GD32_CAN0_RX1_HANDLER not defined"
#endif
#if !defined(GD32_CAN0_EWMC_HANDLER)
#error "GD32_CAN0_EWMC_HANDLER not defined"
#endif

/**
 * @brief   CAN1 TX interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_CAN0_TX_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  can_lld_tx_handler(&CAND2);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   CAN1 RX0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_CAN0_RX0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  can_lld_rx0_handler(&CAND2);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   CAN1 RX1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_CAN0_RX1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  can_lld_rx1_handler(&CAND2);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   CAN1 SCE interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_CAN0_EWMC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  can_lld_sce_handler(&CAND2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_CAN_USE_CAN1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level CAN driver initialization.
 *
 * @notapi
 */
void can_lld_init(void) {

#if GD32_CAN_USE_CAN0
  /* Driver initialization.*/
  canObjectInit(&CAND1);
  CAND1.can = CAN0;
    eclicEnableVector(GD32_CAN0_TX_NUMBER, GD32_CAN_CAN0_IRQ_PRIORITY, GD32_CAN_CAN0_IRQ_TRIGGER);
    eclicEnableVector(GD32_CAN0_RX0_NUMBER, GD32_CAN_CAN0_IRQ_PRIORITY, GD32_CAN_CAN0_IRQ_TRIGGER);
    eclicEnableVector(GD32_CAN0_RX1_NUMBER, GD32_CAN_CAN0_IRQ_PRIORITY, GD32_CAN_CAN0_IRQ_TRIGGER);
    eclicEnableVector(GD32_CAN0_EWMC_NUMBER, GD32_CAN_CAN0_IRQ_PRIORITY, GD32_CAN_CAN0_IRQ_TRIGGER);
#endif

#if GD32_CAN_USE_CAN1
  /* Driver initialization.*/
  canObjectInit(&CAND2);
  CAND2.can = CAN1;
    eclicEnableVector(GD32_CAN0_TX_NUMBER, GD32_CAN_CAN1_IRQ_PRIORITY, GD32_CAN_CAN1_IRQ_TRIGGER);
    eclicEnableVector(GD32_CAN0_RX0_NUMBER, GD32_CAN_CAN1_IRQ_PRIORITY, GD32_CAN_CAN1_IRQ_TRIGGER);
    eclicEnableVector(GD32_CAN0_RX1_NUMBER, GD32_CAN_CAN1_IRQ_PRIORITY, GD32_CAN_CAN1_IRQ_TRIGGER);
    eclicEnableVector(GD32_CAN0_EWMC_NUMBER, GD32_CAN_CAN1_IRQ_PRIORITY, GD32_CAN_CAN1_IRQ_TRIGGER);
#endif

  /* Filters initialization.*/
#if GD32_CAN_USE_CAN0
#if GD32_HAS_CAN1
  can_lld_set_filters(&CAND1, GD32_CAN_MAX_FILTERS / 2, 0, NULL);
#else
  can_lld_set_filters(&CAND1, GD32_CAN_MAX_FILTERS, 0, NULL);
#endif
#endif
}

/**
 * @brief   Configures and activates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_start(CANDriver *canp) {

  /* Clock activation.*/
#if GD32_CAN_USE_CAN0
  if (&CAND1 == canp) {
    rcuEnableCAN0(true);
  }
#endif

#if GD32_CAN_USE_CAN1
  if (&CAND2 == canp) {
    rcuEnableCAN0(true);    /* CAN 2 requires CAN0, so enabling it first.*/
    rcuEnableCAN1(true);
  }
#endif

  /* Configuring CAN. */
  canp->can->CTL = CAN_CTL_IWMOD;
  while ((canp->can->STAT & CAN_STAT_IWS) == 0)
    osalThreadSleepS(1);
  canp->can->BT = canp->config->bt;
  canp->can->CTL = canp->config->ctl;

  /* Interrupt sources initialization.*/
#if GD32_CAN_REPORT_ALL_ERRORS
  canp->can->INTEN = CAN_INTEN_TMEIE  | CAN_INTEN_RFNEIE0 | CAN_INTEN_RFNEIE1 |
                   CAN_INTEN_WIE  | CAN_INTEN_ERRIE  | CAN_INTEN_ERRNIE  |
                   CAN_INTEN_BOIE  | CAN_INTEN_PERRIE  | CAN_INTEN_WERRIE  |
                   CAN_INTEN_RFOIE0 | CAN_INTEN_RFOIE1;
#else
  canp->can->INTEN = CAN_INTEN_TMEIE  | CAN_INTEN_RFNEIE0 | CAN_INTEN_RFNEIE1 |
                   CAN_INTEN_WIE  | CAN_INTEN_ERRIE  |
                   CAN_INTEN_BOIE  | CAN_INTEN_PERRIE  | CAN_INTEN_WERRIE  |
                   CAN_INTEN_RFOIE0 | CAN_INTEN_RFOIE1;
#endif
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_stop(CANDriver *canp) {

  /* If in ready state then disables the CAN peripheral.*/
  if (canp->state == CAN_READY) {
#if GD32_CAN_USE_CAN0
    if (&CAND1 == canp) {
      CAN0->CTL = 0x00010002;                   /* Register reset value.    */
      CAN0->INTEN = 0x00000000;                   /* All sources disabled.    */
#if GD32_CAN_USE_CAN1
      /* If CAND2 is stopped then CAN0 clock is stopped here.*/
      if (CAND2.state == CAN_STOP)
#endif
      {
        rcuDisableCAN0();
      }
    }
#endif

#if GD32_CAN_USE_CAN1
    if (&CAND2 == canp) {
      CAN1->CTL = 0x00010002;                   /* Register reset value.    */
      CAN1->INTEN = 0x00000000;                   /* All sources disabled.    */
#if GD32_CAN_USE_CAN0
      /* If CAND1 is stopped then CAN0 clock is stopped here.*/
      if (CAND1.state == CAN_STOP)
#endif
      {
        rcuDisableCAN0();
      }
      rcuDisableCAN1();
    }
#endif
  }
}

/**
 * @brief   Determines whether a frame can be transmitted.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval false        no space in the transmit queue.
 * @retval true         transmit slot available.
 *
 * @notapi
 */
bool can_lld_is_tx_empty(CANDriver *canp, canmbx_t mailbox) {

  switch (mailbox) {
  case CAN_ANY_MAILBOX:
    return (canp->can->TSTAT & CAN_TSTAT_TME) != 0;
  case 1:
    return (canp->can->TSTAT & CAN_TSTAT_TME0) != 0;
  case 2:
    return (canp->can->TSTAT & CAN_TSTAT_TME1) != 0;
  case 3:
    return (canp->can->TSTAT & CAN_TSTAT_TME2) != 0;
  default:
    return false;
  }
}

/**
 * @brief   Inserts a frame into the transmit queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @param[in] mailbox   mailbox number,  @p CAN_ANY_MAILBOX for any mailbox
 *
 * @notapi
 */
void can_lld_transmit(CANDriver *canp,
                      canmbx_t mailbox,
                      const CANTxFrame *ctfp) {
  uint32_t tmi;
  CAN_TxMailBox_TypeDef *tmbp;

  /* Pointer to a free transmission mailbox.*/
  switch (mailbox) {
  case CAN_ANY_MAILBOX:
    tmbp = &canp->can->sTxMailBox[(canp->can->TSTAT & CAN_TSTAT_NUM) >> 24];
    break;
  case 1:
    tmbp = &canp->can->sTxMailBox[0];
    break;
  case 2:
    tmbp = &canp->can->sTxMailBox[1];
    break;
  case 3:
    tmbp = &canp->can->sTxMailBox[2];
    break;
  default:
    return;
  }

  /* Preparing the message.*/
  if (ctfp->IDE)
    tmi = ((uint32_t)ctfp->EID << 3) | ((uint32_t)ctfp->RTR << 1) |
          CAN_TMI0_FF;
  else
    tmi = ((uint32_t)ctfp->SID << 21) | ((uint32_t)ctfp->RTR << 1);
  tmbp->TMP = ctfp->DLC;
  tmbp->TMDATA0 = ctfp->data32[0];
  tmbp->TMDATA1 = ctfp->data32[1];
  tmbp->TMI  = tmi | CAN_TMI0_TEN;
}

/**
 * @brief   Determines whether a frame has been received.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval false        no space in the transmit queue.
 * @retval true         transmit slot available.
 *
 * @notapi
 */
bool can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox) {

  switch (mailbox) {
  case CAN_ANY_MAILBOX:
    return ((canp->can->RFIFO0 & CAN_RFIFO0_RFL0) != 0 ||
            (canp->can->RFIFO1 & CAN_RFIFO1_RFL1) != 0);
  case 1:
    return (canp->can->RFIFO0 & CAN_RFIFO0_RFL0) != 0;
  case 2:
    return (canp->can->RFIFO1 & CAN_RFIFO1_RFL1) != 0;
  default:
    return false;
  }
}

/**
 * @brief   Receives a frame from the input queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 *
 * @notapi
 */
void can_lld_receive(CANDriver *canp,
                     canmbx_t mailbox,
                     CANRxFrame *crfp) {
  uint32_t rfifomi, rfifomp;

  if (mailbox == CAN_ANY_MAILBOX) {
    if ((canp->can->RFIFO0 & CAN_RFIFO0_RFL0) != 0)
      mailbox = 1;
    else if ((canp->can->RFIFO1 & CAN_RFIFO1_RFL1) != 0)
      mailbox = 2;
    else {
      /* Should not happen, do nothing.*/
      return;
    }
  }
  switch (mailbox) {
  case 1:
    /* Fetches the message.*/
    rfifomi  = canp->can->sFIFOMailBox[0].RFIFOMI;
    rfifomp = canp->can->sFIFOMailBox[0].RFIFOMP;
    crfp->data32[0] = canp->can->sFIFOMailBox[0].RFIFOMDATA0;
    crfp->data32[1] = canp->can->sFIFOMailBox[0].RFIFOMDATA1;

    /* Releases the mailbox.*/
    canp->can->RFIFO0 = CAN_RFIFO0_RFD0;

    /* If the queue is empty re-enables the interrupt in order to generate
       events again.*/
    if ((canp->can->RFIFO0 & CAN_RFIFO0_RFL0) == 0)
      canp->can->INTEN |= CAN_INTEN_RFNEIE0;
    break;
  case 2:
    /* Fetches the message.*/
    rfifomi  = canp->can->sFIFOMailBox[1].RFIFOMI;
    rfifomp = canp->can->sFIFOMailBox[1].RFIFOMP;
    crfp->data32[0] = canp->can->sFIFOMailBox[1].RFIFOMDATA0;
    crfp->data32[1] = canp->can->sFIFOMailBox[1].RFIFOMDATA1;

    /* Releases the mailbox.*/
    canp->can->RFIFO1 = CAN_RFIFO1_RFD1;

    /* If the queue is empty re-enables the interrupt in order to generate
       events again.*/
    if ((canp->can->RFIFO1 & CAN_RFIFO1_RFL1) == 0)
      canp->can->INTEN |= CAN_INTEN_RFNEIE1;
    break;
  default:
    /* Should not happen, do nothing.*/
    return;
  }

  /* Decodes the various fields in the RX frame.*/
  crfp->RTR = (rfifomi & CAN_RFIFOMI0_FT) >> 1;
  crfp->IDE = (rfifomi & CAN_RFIFOMI0_FF) >> 2;
  if (crfp->IDE)
    crfp->EID = rfifomi >> 3;
  else
    crfp->SID = rfifomi >> 21;
  crfp->DLC = rfifomp & CAN_RFIFOMP0_DLENC;
  crfp->FMI = (uint8_t)(rfifomp >> 8);
  crfp->TIME = (uint16_t)(rfifomp >> 16);
}

/**
 * @brief   Tries to abort an ongoing transmission.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number
 *
 * @notapi
 */
void can_lld_abort(CANDriver *canp,
                   canmbx_t mailbox) {

  canp->can->TSTAT = 128U << ((mailbox - 1U) * 8U);
}

#if CAN_USE_SLEEP_MODE || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_sleep(CANDriver *canp) {

  canp->can->CTL |= CAN_CTL_SLPWMOD;
}

/**
 * @brief   Enforces leaving the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_wakeup(CANDriver *canp) {

  canp->can->CTL &= ~CAN_CTL_SLPWMOD;
}
#endif /* CAN_USE_SLEEP_MODE */

/**
 * @brief   Programs the filters.
 * @note    This is an GD32-specific API.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] can2sb    number of the first filter assigned to CAN1
 * @param[in] num       number of entries in the filters array, if zero then
 *                      a default filter is programmed
 * @param[in] cfp       pointer to the filters array, can be @p NULL if
 *                      (num == 0)
 *
 * @api
 */
void canGD32SetFilters(CANDriver *canp, uint32_t can2sb,
                        uint32_t num, const CANFilter *cfp) {

#if GD32_CAN_USE_CAN1
  osalDbgCheck((can2sb <= GD32_CAN_MAX_FILTERS) &&
               (num <= GD32_CAN_MAX_FILTERS));
#endif

#if GD32_CAN_USE_CAN0
  osalDbgAssert(CAND1.state == CAN_STOP, "invalid state");
#endif
#if GD32_CAN_USE_CAN1
  osalDbgAssert(CAND2.state == CAN_STOP, "invalid state");
#endif

#if GD32_CAN_USE_CAN0
  if (canp == &CAND1) {
    can_lld_set_filters(canp, can2sb, num, cfp);
  }
#endif
}

#endif /* HAL_USE_CAN */

/** @} */
