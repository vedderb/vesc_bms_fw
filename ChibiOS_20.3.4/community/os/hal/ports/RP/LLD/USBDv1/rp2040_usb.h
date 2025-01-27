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
 * @file    USBDv1/rp2040_usb.h
 * @brief   RP2040 USB registers layout header.
 * @note    This file requires definitions from the RP2040 header files.
 *
 * @addtogroup USB
 * @{
 */

#ifndef RP2040_USB_H
#define RP2040_USB_H

/**
 * @brief   Number of the available endpoints.
 * @details This value does not include the endpoint 0 which is always present.
 */
#define USB_ENDOPOINTS_NUMBER           15


#define USB_ADDR_ENDP0_ENDPOINT_Pos         16U
#define USB_ADDR_ENDP0_ENDPOINT_Msk         (0xFU << USB_ADDR_ENDP0_ENDPOINT_Pos)
#define USB_ADDR_ENDP0_ADDRESS_Pos          0U
#define USB_ADDR_ENDP0_ADDRESS_Msk          (0x7FU << USB_ADDR_ENDP0_ADDRESS_Pos)

#define USB_ADDR_ENDP_INTEP_PREAMBLE        (1U << 26)
#define USB_ADDR_ENDP_INTEP_DIR             (1U << 25)
#define USB_ADDR_ENDP_ENDPOINT_Pos          16U
#define USB_ADDR_ENDP_ENDPOINT_Msk          (0xFU << USB_ADDR_ENDP_ENDPOINT_Pos)
#define USB_ADDR_ENDP_ADDRESS_Pos           0U
#define USB_ADDR_ENDP_ADDRESS_Msk           (0x7FU << USB_ADDR_ENDP_ADDRESS_Pos)

#define USB_MAIN_CTRL_SIM_TIMING            (1U << 31)
#define USB_MAIN_CTRL_HOST_NDEVICE          (1U << 1)
#define USB_MAIN_CTRL_CONTROLLER_EN         (1U << 0)

#define USB_SOF_WR_COUNT_Pos                0U
#define USB_SOF_WR_COUNT_Msk                (0x3FF << USB_SOF_WR_COUNT_Pos)

#define USB_SOF_RD_COUNT_Pos                0U
#define USB_SOF_RD_COUNT_Msk                (0x3FF << USB_SOF_RD_COUNT_Pos)

#define USB_SIE_CTRL_EP0_INT_STALL          (1U << 31)
#define USB_SIE_CTRL_EP0_DOUBLE_BUF         (1U << 30)
#define USB_SIE_CTRL_EP0_INT_1BUF           (1U << 29)
#define USB_SIE_CTRL_EP0_INT_2BUF           (1U << 28)
#define USB_SIE_CTRL_EP0_INT_NAK            (1U << 27)
#define USB_SIE_CTRL_DIRECT_EN              (1U << 26)
#define USB_SIE_CTRL_DIRECT_DP              (1U << 25)
#define USB_SIE_CTRL_DIRECT_DM              (1U << 24)
#define USB_SIE_CTRL_TRANSCEIVER_PD         (1U << 18)
#define USB_SIE_CTRL_RPU_OPT                (1U << 17)
#define USB_SIE_CTRL_PULLUP_EN              (1U << 16)
#define USB_SIE_CTRL_PULLDOWN_EN            (1U << 15)
#define USB_SIE_CTRL_RESET_BUS              (1U << 13)
#define USB_SIE_CTRL_RESUME                 (1U << 12)
#define USB_SIE_CTRL_VBUS_EN                (1U << 11)
#define USB_SIE_CTRL_KEEP_ALIVE_EN          (1U << 10)
#define USB_SIE_CTRL_SOF_EN                 (1U << 9)
#define USB_SIE_CTRL_SOF_SYNC               (1U << 8)
#define USB_SIE_CTRL_PREAMBLE_EN            (1U << 6)
#define USB_SIE_CTRL_STOP_TRANS             (1U << 4)
#define USB_SIE_CTRL_RECEIVE_DATA           (1U << 3)
#define USB_SIE_CTRL_SEND_DATA              (1U << 2)
#define USB_SIE_CTRL_SEND_SETUP             (1U << 1)
#define USB_SIE_CTRL_START_TRANS            (1U << 0)

#define USB_SIE_STATUS_DATA_SEQ_ERROR       (1U << 31)
#define USB_SIE_STATUS_ACK_REC              (1U << 30)
#define USB_SIE_STATUS_STALL_REC            (1U << 29)
#define USB_SIE_STATUS_NAK_REC              (1U << 28)
#define USB_SIE_STATUS_RX_TIMEOUT           (1U << 27)
#define USB_SIE_STATUS_RX_OVERFLOW          (1U << 26)
#define USB_SIE_STATUS_BIT_STUFF_ERROR      (1U << 25)
#define USB_SIE_STATUS_CRC_ERROR            (1U << 24)
#define USB_SIE_STATUS_BUS_RESET            (1U << 19)
#define USB_SIE_STATUS_TRANS_COMPLETE       (1U << 18)
#define USB_SIE_STATUS_SETUP_REC            (1U << 17)
#define USB_SIE_STATUS_CONNECTED            (1U << 16)
#define USB_SIE_STATUS_RESUME               (1U << 11)
#define USB_SIE_STATUS_VBUS_OVER_CURR       (1U << 10)
#define USB_SIE_STATUS_SPEED_Pos            8U
#define USB_SIE_STATUS_SPEED_Msk            (0x3U << USB_SIE_STATUS_SPEED_Pos)
#define USB_SIE_STATUS_SUSPENDED            (1U << 4)
#define USB_SIE_STATUS_LINE_STATE_Pos       2U
#define USB_SIE_STATUS_LINE_STATE_Msk       (0x3U << USB_SIE_STATUS_LINE_STATE_Pos)
#define USB_SIE_STATUS_VBUS_DETECTED        (1U << 0)

#define USB_INT_EP_CTRL_INT_EP_ACTIVE_Pos   0U
#define USB_INT_EP_CTRL_INT_EP_ACTIVE_Msk   (0xFFFE << USB_INT_EP_CTRL_INT_EP_ACTIVE_Pos)

#define USB_BUFF_STATUS_EP15_OUT     (1U << 31)
#define USB_BUFF_STATUS_EP15_IN      (1U << 30)
#define USB_BUFF_STATUS_EP14_OUT     (1U << 29)
#define USB_BUFF_STATUS_EP14_IN      (1U << 28)
#define USB_BUFF_STATUS_EP13_OUT     (1U << 27)
#define USB_BUFF_STATUS_EP13_IN      (1U << 26)
#define USB_BUFF_STATUS_EP12_OUT     (1U << 25)
#define USB_BUFF_STATUS_EP12_IN      (1U << 24)
#define USB_BUFF_STATUS_EP11_OUT     (1U << 23)
#define USB_BUFF_STATUS_EP11_IN      (1U << 22)
#define USB_BUFF_STATUS_EP10_OUT     (1U << 21)
#define USB_BUFF_STATUS_EP10_IN      (1U << 20)
#define USB_BUFF_STATUS_EP9_OUT      (1U << 19)
#define USB_BUFF_STATUS_EP9_IN       (1U << 18)
#define USB_BUFF_STATUS_EP8_OUT      (1U << 17)
#define USB_BUFF_STATUS_EP8_IN       (1U << 16)
#define USB_BUFF_STATUS_EP7_OUT      (1U << 15)
#define USB_BUFF_STATUS_EP7_IN       (1U << 14)
#define USB_BUFF_STATUS_EP6_OUT      (1U << 13)
#define USB_BUFF_STATUS_EP6_IN       (1U << 12)
#define USB_BUFF_STATUS_EP5_OUT      (1U << 11)
#define USB_BUFF_STATUS_EP5_IN       (1U << 10)
#define USB_BUFF_STATUS_EP4_OUT      (1U << 9)
#define USB_BUFF_STATUS_EP4_IN       (1U << 8)
#define USB_BUFF_STATUS_EP3_OUT      (1U << 7)
#define USB_BUFF_STATUS_EP3_IN       (1U << 6)
#define USB_BUFF_STATUS_EP2_OUT      (1U << 5)
#define USB_BUFF_STATUS_EP2_IN       (1U << 4)
#define USB_BUFF_STATUS_EP1_OUT      (1U << 3)
#define USB_BUFF_STATUS_EP1_IN       (1U << 2)
#define USB_BUFF_STATUS_EP0_OUT      (1U << 1)
#define USB_BUFF_STATUS_EP0_IN       (1U << 0)

#define USB_BUFF_CPU_SHOULD_HANDLE_EP15_OUT     (1U << 31)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP15_IN      (1U << 30)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP14_OUT     (1U << 29)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP14_IN      (1U << 28)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP13_OUT     (1U << 27)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP13_IN      (1U << 26)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP12_OUT     (1U << 25)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP12_IN      (1U << 24)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP11_OUT     (1U << 23)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP11_IN      (1U << 22)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP10_OUT     (1U << 21)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP10_IN      (1U << 20)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP9_OUT      (1U << 19)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP9_IN       (1U << 18)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP8_OUT      (1U << 17)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP8_IN       (1U << 16)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP7_OUT      (1U << 15)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP7_IN       (1U << 14)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP6_OUT      (1U << 13)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP6_IN       (1U << 12)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP5_OUT      (1U << 11)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP5_IN       (1U << 10)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP4_OUT      (1U << 9)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP4_IN       (1U << 8)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP3_OUT      (1U << 7)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP3_IN       (1U << 6)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP2_OUT      (1U << 5)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP2_IN       (1U << 4)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP1_OUT      (1U << 3)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP1_IN       (1U << 2)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP0_OUT      (1U << 1)
#define USB_BUFF_CPU_SHOULD_HANDLE_EP0_IN       (1U << 0)

#define USB_EP_ABORT_EP15_OUT     (1U << 31)
#define USB_EP_ABORT_EP15_IN      (1U << 30)
#define USB_EP_ABORT_EP14_OUT     (1U << 29)
#define USB_EP_ABORT_EP14_IN      (1U << 28)
#define USB_EP_ABORT_EP13_OUT     (1U << 27)
#define USB_EP_ABORT_EP13_IN      (1U << 26)
#define USB_EP_ABORT_EP12_OUT     (1U << 25)
#define USB_EP_ABORT_EP12_IN      (1U << 24)
#define USB_EP_ABORT_EP11_OUT     (1U << 23)
#define USB_EP_ABORT_EP11_IN      (1U << 22)
#define USB_EP_ABORT_EP10_OUT     (1U << 21)
#define USB_EP_ABORT_EP10_IN      (1U << 20)
#define USB_EP_ABORT_EP9_OUT      (1U << 19)
#define USB_EP_ABORT_EP9_IN       (1U << 18)
#define USB_EP_ABORT_EP8_OUT      (1U << 17)
#define USB_EP_ABORT_EP8_IN       (1U << 16)
#define USB_EP_ABORT_EP7_OUT      (1U << 15)
#define USB_EP_ABORT_EP7_IN       (1U << 14)
#define USB_EP_ABORT_EP6_OUT      (1U << 13)
#define USB_EP_ABORT_EP6_IN       (1U << 12)
#define USB_EP_ABORT_EP5_OUT      (1U << 11)
#define USB_EP_ABORT_EP5_IN       (1U << 10)
#define USB_EP_ABORT_EP4_OUT      (1U << 9)
#define USB_EP_ABORT_EP4_IN       (1U << 8)
#define USB_EP_ABORT_EP3_OUT      (1U << 7)
#define USB_EP_ABORT_EP3_IN       (1U << 6)
#define USB_EP_ABORT_EP2_OUT      (1U << 5)
#define USB_EP_ABORT_EP2_IN       (1U << 4)
#define USB_EP_ABORT_EP1_OUT      (1U << 3)
#define USB_EP_ABORT_EP1_IN       (1U << 2)
#define USB_EP_ABORT_EP0_OUT      (1U << 1)
#define USB_EP_ABORT_EP0_IN       (1U << 0)

#define USB_EP_ABORT_DONE_EP15_OUT     (1U << 31)
#define USB_EP_ABORT_DONE_EP15_IN      (1U << 30)
#define USB_EP_ABORT_DONE_EP14_OUT     (1U << 29)
#define USB_EP_ABORT_DONE_EP14_IN      (1U << 28)
#define USB_EP_ABORT_DONE_EP13_OUT     (1U << 27)
#define USB_EP_ABORT_DONE_EP13_IN      (1U << 26)
#define USB_EP_ABORT_DONE_EP12_OUT     (1U << 25)
#define USB_EP_ABORT_DONE_EP12_IN      (1U << 24)
#define USB_EP_ABORT_DONE_EP11_OUT     (1U << 23)
#define USB_EP_ABORT_DONE_EP11_IN      (1U << 22)
#define USB_EP_ABORT_DONE_EP10_OUT     (1U << 21)
#define USB_EP_ABORT_DONE_EP10_IN      (1U << 20)
#define USB_EP_ABORT_DONE_EP9_OUT      (1U << 19)
#define USB_EP_ABORT_DONE_EP9_IN       (1U << 18)
#define USB_EP_ABORT_DONE_EP8_OUT      (1U << 17)
#define USB_EP_ABORT_DONE_EP8_IN       (1U << 16)
#define USB_EP_ABORT_DONE_EP7_OUT      (1U << 15)
#define USB_EP_ABORT_DONE_EP7_IN       (1U << 14)
#define USB_EP_ABORT_DONE_EP6_OUT      (1U << 13)
#define USB_EP_ABORT_DONE_EP6_IN       (1U << 12)
#define USB_EP_ABORT_DONE_EP5_OUT      (1U << 11)
#define USB_EP_ABORT_DONE_EP5_IN       (1U << 10)
#define USB_EP_ABORT_DONE_EP4_OUT      (1U << 9)
#define USB_EP_ABORT_DONE_EP4_IN       (1U << 8)
#define USB_EP_ABORT_DONE_EP3_OUT      (1U << 7)
#define USB_EP_ABORT_DONE_EP3_IN       (1U << 6)
#define USB_EP_ABORT_DONE_EP2_OUT      (1U << 5)
#define USB_EP_ABORT_DONE_EP2_IN       (1U << 4)
#define USB_EP_ABORT_DONE_EP1_OUT      (1U << 3)
#define USB_EP_ABORT_DONE_EP1_IN       (1U << 2)
#define USB_EP_ABORT_DONE_EP0_OUT      (1U << 1)
#define USB_EP_ABORT_DONE_EP0_IN       (1U << 0)

#define USB_EP_STALL_ARM_EP0_OUT       (1U << 1)
#define USB_EP_STALL_ARM_EP0_IN        (1U << 0)

#define USB_NAK_POLL_DELAY_FS_Pos           16U
#define USB_NAK_POLL_DELAY_FS_Msk           (0x3FF << USB_NAK_POLL_DELAY_FS_Pos)
#define USB_NAK_POLL_DELAY_LS_Pos           0U
#define USB_NAK_POLL_DELAY_LS_Msk           (0x3FF << USB_NAK_POLL_DELAY_LS_Pos)

#define USB_EP_STATUS_STALL_NAK_EP15_OUT     (1U << 31)
#define USB_EP_STATUS_STALL_NAK_EP15_IN      (1U << 30)
#define USB_EP_STATUS_STALL_NAK_EP14_OUT     (1U << 29)
#define USB_EP_STATUS_STALL_NAK_EP14_IN      (1U << 28)
#define USB_EP_STATUS_STALL_NAK_EP13_OUT     (1U << 27)
#define USB_EP_STATUS_STALL_NAK_EP13_IN      (1U << 26)
#define USB_EP_STATUS_STALL_NAK_EP12_OUT     (1U << 25)
#define USB_EP_STATUS_STALL_NAK_EP12_IN      (1U << 24)
#define USB_EP_STATUS_STALL_NAK_EP11_OUT     (1U << 23)
#define USB_EP_STATUS_STALL_NAK_EP11_IN      (1U << 22)
#define USB_EP_STATUS_STALL_NAK_EP10_OUT     (1U << 21)
#define USB_EP_STATUS_STALL_NAK_EP10_IN      (1U << 20)
#define USB_EP_STATUS_STALL_NAK_EP9_OUT      (1U << 19)
#define USB_EP_STATUS_STALL_NAK_EP9_IN       (1U << 18)
#define USB_EP_STATUS_STALL_NAK_EP8_OUT      (1U << 17)
#define USB_EP_STATUS_STALL_NAK_EP8_IN       (1U << 16)
#define USB_EP_STATUS_STALL_NAK_EP7_OUT      (1U << 15)
#define USB_EP_STATUS_STALL_NAK_EP7_IN       (1U << 14)
#define USB_EP_STATUS_STALL_NAK_EP6_OUT      (1U << 13)
#define USB_EP_STATUS_STALL_NAK_EP6_IN       (1U << 12)
#define USB_EP_STATUS_STALL_NAK_EP5_OUT      (1U << 11)
#define USB_EP_STATUS_STALL_NAK_EP5_IN       (1U << 10)
#define USB_EP_STATUS_STALL_NAK_EP4_OUT      (1U << 9)
#define USB_EP_STATUS_STALL_NAK_EP4_IN       (1U << 8)
#define USB_EP_STATUS_STALL_NAK_EP3_OUT      (1U << 7)
#define USB_EP_STATUS_STALL_NAK_EP3_IN       (1U << 6)
#define USB_EP_STATUS_STALL_NAK_EP2_OUT      (1U << 5)
#define USB_EP_STATUS_STALL_NAK_EP2_IN       (1U << 4)
#define USB_EP_STATUS_STALL_NAK_EP1_OUT      (1U << 3)
#define USB_EP_STATUS_STALL_NAK_EP1_IN       (1U << 2)
#define USB_EP_STATUS_STALL_NAK_EP0_OUT      (1U << 1)
#define USB_EP_STATUS_STALL_NAK_EP0_IN       (1U << 0)

#define USB_USB_MUXING_SOFTCON              (1U << 3)
#define USB_USB_MUXING_TO_DIGITAL_PAD       (1U << 2)
#define USB_USB_MUXING_TO_EXTPHY            (1U << 1)
#define USB_USB_MUXING_TO_PHY               (1U << 0)

#define USB_USB_PWR_OVERCURR_DETECT_EN          (1U << 5)
#define USB_USB_PWR_OVERCURR_DETECT             (1U << 4)
#define USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN     (1U << 3)
#define USB_USB_PWR_VBUS_DETECT                 (1U << 2)
#define USB_USB_PWR_VBUS_EN_OVERRIDE_EN         (1U << 1)
#define USB_USB_PWR_VBUS_EN                     (1U << 0)

#define USB_USBPHY_DIRECT_DM_OVV            (1U << 22)
#define USB_USBPHY_DIRECT_DP_OVV            (1U << 21)
#define USB_USBPHY_DIRECT_DM_OVCN           (1U << 20)
#define USB_USBPHY_DIRECT_DP_OVCN           (1U << 19)
#define USB_USBPHY_DIRECT_RX_DM             (1U << 18)
#define USB_USBPHY_DIRECT_RX_DP             (1U << 17)
#define USB_USBPHY_DIRECT_RX_DD             (1U << 16)
#define USB_USBPHY_DIRECT_TX_DIFFMODE       (1U << 15)
#define USB_USBPHY_DIRECT_TX_FSSLEW         (1U << 14)
#define USB_USBPHY_DIRECT_TX_PD             (1U << 13)
#define USB_USBPHY_DIRECT_RX_PD             (1U << 12)
#define USB_USBPHY_DIRECT_TX_DM             (1U << 11)
#define USB_USBPHY_DIRECT_TX_DP             (1U << 10)
#define USB_USBPHY_DIRECT_TX_DM_OE          (1U << 9)
#define USB_USBPHY_DIRECT_TX_DP_OE          (1U << 8)
#define USB_USBPHY_DIRECT_DM_PULLDN_EN      (1U << 6)
#define USB_USBPHY_DIRECT_DM_PULLUP_EN      (1U << 5)
#define USB_USBPHY_DIRECT_DM_PULLUP_HISEL   (1U << 4)
#define USB_USBPHY_DIRECT_DP_PULLDN_EN      (1U << 1)
#define USB_USBPHY_DIRECT_DP_PULLUP_EN      (1U << 1)
#define USB_USBPHY_DIRECT_DP_PULLUP_HISEL   (1U << 0)

#define USB_USBPHY_DIRECT_OVERRIDE_TX_DIFFMODE_OVERRIDE_EN      (1U << 15)
#define USB_USBPHY_DIRECT_OVERRIDE_DM_PULLUP_OVERRIDE_EN        (1U << 12)
#define USB_USBPHY_DIRECT_OVERRIDE_TX_FSSLEW_OVERRIDE_EN        (1U << 11)
#define USB_USBPHY_DIRECT_OVERRIDE_TX_PD_OVERRIDE_EN            (1U << 10)
#define USB_USBPHY_DIRECT_OVERRIDE_RX_PD_OVERRIDE_EN            (1U << 9)
#define USB_USBPHY_DIRECT_OVERRIDE_TX_DM_OVERRIDE_EN            (1U << 8)
#define USB_USBPHY_DIRECT_OVERRIDE_TX_DP_OVERRIDE_EN            (1U << 7)
#define USB_USBPHY_DIRECT_OVERRIDE_TX_DM_OE_OVERRIDE_EN         (1U << 6)
#define USB_USBPHY_DIRECT_OVERRIDE_TX_DP_OE_OVERRIDE_EN         (1U << 5)
#define USB_USBPHY_DIRECT_OVERRIDE_DM_PULLDN_EN_OVERRIDE_EN     (1U << 4)
#define USB_USBPHY_DIRECT_OVERRIDE_DP_PULLDN_EN_OVERRIDE_EN     (1U << 3)
#define USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN     (1U << 2)
#define USB_USBPHY_DIRECT_OVERRIDE_DM_PULLUP_HISEL_OVERRIDE_EN  (1U << 1)
#define USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_HISEL_OVERRIDE_EN  (1U << 0)

#define USB_USBPHY_TRIM_DM_PULLDN_TRIM_Pos     8
#define USB_USBPHY_TRIM_DM_PULLDN_TRIM_Msk     (0x1F << USB_PHY_TRIM_DP_PULLDN_TRIM_Pos)
#define USB_USBPHY_TRIM_DP_PULLDN_TRIM_Pos     0
#define USB_USBPHY_TRIM_DP_PULLDN_TRIM_Msk     (0x1F << USB_PHY_TRIM_DM_PULLDN_TRIM_Pos)

#define USB_INTR_EP_STALL_NAK           (1U << 19)
#define USB_INTR_ABORT_DONE             (1U << 18)
#define USB_INTR_DEV_SOF                (1U << 17)
#define USB_INTR_SETUP_REQ              (1U << 16)
#define USB_INTR_DEV_RESUME_FROM_HOST   (1U << 15)
#define USB_INTR_DEV_SUSPEND            (1U << 14)
#define USB_INTR_DEV_CONN_DIS           (1U << 13)
#define USB_INTR_BUS_RESET              (1U << 12)
#define USB_INTR_VBUS_DETECT            (1U << 11)
#define USB_INTR_STALL                  (1U << 10)
#define USB_INTR_ERROR_CRC              (1U << 9)
#define USB_INTR_ERROR_BIT_STUFF        (1U << 8)
#define USB_INTR_ERROR_RX_OVERFLOW      (1U << 7)
#define USB_INTR_ERROR_RX_TIMEOUT       (1U << 6)
#define USB_INTR_ERROR_DATA_SEQ         (1U << 5)
#define USB_INTR_BUFF_STATUS            (1U << 4)
#define USB_INTR_TRANS_COMPLETE         (1U << 3)
#define USB_INTR_HOST_SOF               (1U << 2)
#define USB_INTR_HOST_RESUME            (1U << 1)
#define USB_INTR_HOST_CONN_DIS          (1U << 0)

#define USB_INTE_EP_STALL_NAK           (1U << 19)
#define USB_INTE_ABORT_DONE             (1U << 18)
#define USB_INTE_DEV_SOF                (1U << 17)
#define USB_INTE_SETUP_REQ              (1U << 16)
#define USB_INTE_DEV_RESUME_FROM_HOST   (1U << 15)
#define USB_INTE_DEV_SUSPEND            (1U << 14)
#define USB_INTE_DEV_CONN_DIS           (1U << 13)
#define USB_INTE_BUS_RESET              (1U << 12)
#define USB_INTE_VBUS_DETECT            (1U << 11)
#define USB_INTE_STALL                  (1U << 10)
#define USB_INTE_ERROR_CRC              (1U << 9)
#define USB_INTE_ERROR_BIT_STUFF        (1U << 8)
#define USB_INTE_ERROR_RX_OVERFLOW      (1U << 7)
#define USB_INTE_ERROR_RX_TIMEOUT       (1U << 6)
#define USB_INTE_ERROR_DATA_SEQ         (1U << 5)
#define USB_INTE_BUFF_STATUS            (1U << 4)
#define USB_INTE_TRANS_COMPLETE         (1U << 3)
#define USB_INTE_HOST_SOF               (1U << 2)
#define USB_INTE_HOST_RESUME            (1U << 1)
#define USB_INTE_HOST_CONN_DIS          (1U << 0)

#define USB_INTF_EP_STALL_NAK           (1U << 19)
#define USB_INTF_ABORT_DONE             (1U << 18)
#define USB_INTF_DEV_SOF                (1U << 17)
#define USB_INTF_SETUP_REQ              (1U << 16)
#define USB_INTF_DEV_RESUME_FROM_HOST   (1U << 15)
#define USB_INTF_DEV_SUSPEND            (1U << 14)
#define USB_INTF_DEV_CONN_DIS           (1U << 13)
#define USB_INTF_BUS_RESET              (1U << 12)
#define USB_INTF_VBUS_DETECT            (1U << 11)
#define USB_INTF_STALL                  (1U << 10)
#define USB_INTF_ERROR_CRC              (1U << 9)
#define USB_INTF_ERROR_BIT_STUFF        (1U << 8)
#define USB_INTF_ERROR_RX_OVERFLOW      (1U << 7)
#define USB_INTF_ERROR_RX_TIMEOUT       (1U << 6)
#define USB_INTF_ERROR_DATA_SEQ         (1U << 5)
#define USB_INTF_BUFF_STATUS            (1U << 4)
#define USB_INTF_TRANS_COMPLETE         (1U << 3)
#define USB_INTF_HOST_SOF               (1U << 2)
#define USB_INTF_HOST_RESUME            (1U << 1)
#define USB_INTF_HOST_CONN_DIS          (1U << 0)

#define USB_INTS_EP_STALL_NAK           (1U << 19)
#define USB_INTS_ABORT_DONE             (1U << 18)
#define USB_INTS_DEV_SOF                (1U << 17)
#define USB_INTS_SETUP_REQ              (1U << 16)
#define USB_INTS_DEV_RESUME_FROM_HOST   (1U << 15)
#define USB_INTS_DEV_SUSPEND            (1U << 14)
#define USB_INTS_DEV_CONN_DIS           (1U << 13)
#define USB_INTS_BUS_RESET              (1U << 12)
#define USB_INTS_VBUS_DETECT            (1U << 11)
#define USB_INTS_STALL                  (1U << 10)
#define USB_INTS_ERROR_CRC              (1U << 9)
#define USB_INTS_ERROR_BIT_STUFF        (1U << 8)
#define USB_INTS_ERROR_RX_OVERFLOW      (1U << 7)
#define USB_INTS_ERROR_RX_TIMEOUT       (1U << 6)
#define USB_INTS_ERROR_DATA_SEQ         (1U << 5)
#define USB_INTS_BUFF_STATUS            (1U << 4)
#define USB_INTS_TRANS_COMPLETE         (1U << 3)
#define USB_INTS_HOST_SOF               (1U << 2)
#define USB_INTS_HOST_RESUME            (1U << 1)
#define USB_INTS_HOST_CONN_DIS          (1U << 0)

typedef struct {
  __IO uint8_t      SETUPPACKET[8];
  struct {
    __IO uint32_t   IN;
    __IO uint32_t   OUT;
  } EPCTRL[15];
  struct {
    __IO uint32_t   IN;
    __IO uint32_t   OUT;
  } BUFCTRL[16];
  __IO uint8_t      EP0BUF0[64];
  __IO uint8_t      EP0BUF1[64];
  __IO uint8_t      DATA[3712];
} USB_DPSRAM_TypeDef;

#define __AHBPERIPH_BASE                  0x50000000U

#define __USB_DPSRAM_BASE     (__AHBPERIPH_BASE + 0x00100000U)

#define USB_DPSRAM      ((USB_DPSRAM_TypeDef *) __USB_DPSRAM_BASE)

// Endpoint control register
#define USB_EP_EN                       (1U << 31)
#define USB_EP_BUFFER_DOUBLE            (1U << 30)
#define USB_EP_BUFFER_IRQ_EN            (1U << 29)
#define USB_EP_BUFFER_IRQ_DOUBLE_EN     (1U << 28)
#define USB_EP_TYPE_Pos                 26U
#define USB_EP_TYPE_Msk                 (0x3 << USB_EP_TYPE_Pos)
#define USB_EP_IRQ_STALL                (1U << 17)
#define USB_EP_IRQ_NAK                  (1U << 16)
#define USB_EP_ADDR_OFFSET_Pos          0U
#define USB_EP_ADDR_OFFSET_Msk          (0x7F << USB_EP_ADDR_OFFSET_Pos)

// Buffer control register
#define USB_BUFFER_BUFFER1_FULL                 (1U << 31)
#define USB_BUFFER_BUFFER1_LAST                 (1U << 30)
#define USB_BUFFER_BUFFER1_DATA_PID             (1U << 29)
#define USB_BUFFER_DOUBLE_BUFFER_OFFSET_Pos     27U
#define USB_BUFFER_DOUBLE_BUFFER_OFFSET_Msk     (0x3 << USB_BUFFER_DOUBLE_BUFFER_OFFSET_Pos)
#define USB_BUFFER_BUFFER1_AVAILABLE            (1U << 26)
#define USB_BUFFER_BUFFER1_TRANS_LENGTH_Pos     16U
#define USB_BUFFER_BUFFER1_TRANS_LENGTH_Msk     (0x3FF << USB_BUFFER_BUFFER1_TRANS_LENGTH_Pos)
#define USB_BUFFER_BUFFER0_FULL                 (1U << 15)
#define USB_BUFFER_BUFFER0_LAST                 (1U << 14)
#define USB_BUFFER_BUFFER0_DATA_PID             (1U << 13)
#define USB_BUFFER_RESET_BUFFER                 (1U << 12)
// Send STALL for devices, STALL received for host
#define USB_BUFFER_STALL                        (1U << 11)
#define USB_BUFFER_BUFFER0_AVAILABLE            (1U << 10)
#define USB_BUFFER_BUFFER0_TRANS_LENGTH_Pos     0U
#define USB_BUFFER_BUFFER0_TRANS_LENGTH_Msk     (0x3FF << USB_BUFFER_BUFFER0_TRANS_LENGTH_Pos)

#endif /* RP2040_USB_H */

/** @} */
