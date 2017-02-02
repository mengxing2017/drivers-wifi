// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

namespace rt5370 {

enum UsbVendorRequest : uint8_t {
    kDeviceMode = 1,
    kSingleWrite = 2,
    kSingleRead = 3,
    kMultiWrite = 6,
    kMultiRead = 7,
    kEepromWrite = 8,
    kEepromRead = 9,
    kLedControl = 10,
    kRxControl = 12,
};

enum UsbModeOffset : uint8_t {
    kReset = 1,
    kUnplug = 2,
    kFunction = 3,
    kTest = 4,
    kFirmware = 8,
    kAutorun = 17,
};

constexpr uint16_t RT5390 = 0x5390;
constexpr uint16_t FW_IMAGE_BASE = 0x3000;

// Registers

constexpr uint16_t WPDMA_GLO_CFG = 0x0208;
constexpr int WPDMA_GLO_CFG_TX_DMA_EN = 0;
constexpr int WPDMA_GLO_CFG_TX_DMA_BUSY = 1;
constexpr int WPDMA_GLO_CFG_RX_DMA_EN = 2;
constexpr int WPDMA_GLO_CFG_RX_DMA_BUSY = 3;
constexpr int WPDMA_GLO_CFG_WPDMA_BT_SIZE_OFFSET = 4;
constexpr int WPDMA_GLO_CFG_WPDMA_BT_SIZE_WIDTH = 2;
constexpr int WPDMA_GLO_CFG_TX_WB_DDONE = 6;
constexpr int WPDMA_GLO_CFG_BIG_ENDIAN = 7;
constexpr int WPDMA_GLO_CFG_HDR_SEG_LEN_OFFSET = 8;
constexpr int WPDMA_GLO_CFG_HDR_SEG_LEN_WIDTH = 8;

constexpr uint16_t USB_DMA_CFG = 0x02a0;
constexpr int USB_DMA_CFG_RX_AGG_TO_OFFSET = 0;
constexpr int USB_DMA_CFG_RX_AGG_TO_WIDTH = 8;
constexpr int USB_DMA_CFG_RX_AGG_LIMIT_OFFSET = 8;
constexpr int USB_DMA_CFG_RX_AGG_LIMIT_WIDTH = 8;
constexpr int USB_DMA_CFG_PHY_WD_EN = 16;
constexpr int USB_DMA_CFG_TX_CLEAR = 19;
constexpr int USB_DMA_CFG_TXOP_HALT = 20;
constexpr int USB_DMA_CFG_RX_AGG_EN = 21;
constexpr int USB_DMA_CFG_UDMA_RX_EN = 22;
constexpr int USB_DMA_CFG_UDMA_TX_EN = 23;
constexpr int USB_DMA_CFG_EPOUT_VLD_OFFSET = 24;
constexpr int USB_DMA_CFG_EPOUT_VLD_WIDTH = 5;
constexpr int USB_DMA_CFG_RX_BUSY = 30;
constexpr int USB_DMA_CFG_TX_BUSY = 31;

constexpr uint16_t SYS_CTRL = 0x0400;
constexpr int SYS_CTRL_MCU_READY = 7;
constexpr int SYS_CTRL_PME_OEN = 13;

constexpr uint16_t HOST_CMD = 0x0404;

constexpr uint16_t MAX_PCNT = 0x040c;
constexpr int MAX_PCNT_WIDTH = 8;
constexpr int MAX_PCNT_MAX_RX0Q_PCNT_OFFSET = 0;
constexpr int MAX_PCNT_MAX_TX2Q_PCNT_OFFSET = 8;
constexpr int MAX_PCNT_MAX_TX1Q_PCNT_OFFSET = 16;
constexpr int MAX_PCNT_MAX_TX0Q_PCNT_OFFSET = 24;

constexpr uint16_t PBF_CFG = 0x0408;
constexpr int PBF_CFG_RX0Q_EN = 1;
constexpr int PBF_CFG_TX2Q_EN = 2;
constexpr int PBF_CFG_TX1Q_EN = 3;
constexpr int PBF_CFG_TX0Q_EN = 4;
constexpr int PBF_CFG_HCCA_MODE = 8;
constexpr int PBF_CFG_RX0Q_MODE = 9;
constexpr int PBF_CFG_TX2Q_MODE = 10;
constexpr int PBF_CFG_TX1Q_MODE = 11;
constexpr int PBF_CFG_TX0Q_MODE = 12;
constexpr int PBF_CFG_RX_DROP_MODE = 13;
constexpr int PBF_CFG_NULL1_MODE = 14;
constexpr int PBF_CFG_NULL0_MODE = 15;
constexpr int PBF_CFG_TX2Q_NUM_OFFSET = 16;
constexpr int PBF_CFG_TX2Q_NUM_WIDTH = 5;
constexpr int PBF_CFG_TX1Q_NUM_OFFSET = 21;
constexpr int PBF_CFG_TX1Q_NUM_WIDTH = 3;
constexpr int PBF_CFG_NULL2_SEL_OFFSET = 24;
constexpr int PBF_CFG_NULL2_SEL_WIDTH = 3;

constexpr uint16_t EFUSE_CTRL = 0x0580;
constexpr int EFUSE_CTRL_KICK = 30;
constexpr int EFUSE_CTRL_ADDR = 16;
constexpr int EFUSE_CTRL_ADDR_WIDTH = 10;
constexpr int EFUSE_CTRL_MODE = 6;

constexpr uint16_t EFUSE_DATA0 = 0x0590;
constexpr uint16_t EFUSE_DATA1 = 0x0594;
constexpr uint16_t EFUSE_DATA2 = 0x0598;
constexpr uint16_t EFUSE_DATA3 = 0x059c;

constexpr uint16_t MAC_CSR0 = 0x1000;

constexpr uint16_t MAC_SYS_CTRL = 0x1004;
constexpr int MAC_SYS_CTRL_MAC_SRST = 0;
constexpr int MAC_SYS_CTRL_BBP_HRST = 1;
constexpr int MAC_SYS_CTRL_MAC_TX_EN = 2;
constexpr int MAC_SYS_CTRL_MAC_RX_EN = 3;

constexpr uint16_t MAX_LEN_CFG = 0x1018;
constexpr int MAX_LEN_CFG_MAX_MPDU_LEN_OFFSET = 0;
constexpr int MAX_LEN_CFG_MAX_MPDU_LEN_WIDTH = 12;
constexpr int MAX_LEN_CFG_MAX_PSDU_LEN_OFFSET = 12;
constexpr int MAX_LEN_CFG_MAX_PSDU_LEN_WIDTH = 2;
// From Linux kernel source:
constexpr int MAX_LEN_CFG_MIN_PSDU_LEN_OFFSET = 14;
constexpr int MAX_LEN_CFG_MIN_PSDU_LEN_WIDTH = 2;
constexpr int MAX_LEN_CFG_MIN_MPDU_LEN_OFFSET = 16;
constexpr int MAX_LEN_CFG_MIN_MPDU_LEN_WIDTH = 4;

constexpr uint16_t BKOFF_SLOT_CFG = 0x1104;
constexpr int BKOFF_SLOT_CFG_SLOT_TIME_OFFSET = 0;
constexpr int BKOFF_SLOT_CFG_SLOT_TIME_WIDTH = 8;
constexpr int BKOFF_SLOT_CFG_CC_DELAY_TIME_OFFSET = 8;
constexpr int BKOFF_SLOT_CFG_CC_DELAY_TIME_WIDTH = 4;

constexpr uint16_t BCN_TIME_CFG = 0x1114;
constexpr int BCN_TIME_CFG_BCN_INTVAL_OFFSET = 0;
constexpr int BCN_TIME_CFG_BCN_INTVAL_WIDTH = 16;
constexpr int BCN_TIME_CFG_TSF_TIMER_EN = 16;
constexpr int BCN_TIME_CFG_TSF_SYNC_MODE_OFFSET = 17;
constexpr int BCN_TIME_CFG_TSF_SYNC_MODE_WIDTH = 2;
constexpr int BCN_TIME_CFG_TBTT_TIMER_EN = 19;
constexpr int BCN_TIME_CFG_BCN_TX_EN = 20;
constexpr int BCN_TIME_CFG_TSF_INS_COMP_OFFSET = 24;
constexpr int BCN_TIME_CFG_TSF_INS_COMP_WIDTH = 8;

constexpr uint16_t AUTO_WAKEUP_CFG = 0x1208;

// ALl TX_SW_CFG* values are 8 bits wide
constexpr int TX_SW_CFG_WIDTH = 8;

constexpr uint16_t TX_SW_CFG0 = 0x1330;
constexpr int TX_SW_CFG0_DLY_TXPE_EN = 0;
constexpr int TX_SW_CFG0_DLY_PAPE_EN = 8;
constexpr int TX_SW_CFG0_DLY_TRSW_EN = 16;
constexpr int TX_SW_CFG0_DLY_RFTR_EN = 24;

constexpr uint16_t TX_SW_CFG1 = 0x1334;
constexpr int TX_SW_CFG1_DLY_PAPE_DIS = 0;
constexpr int TX_SW_CFG1_DLY_TRSW_DIS = 8;
constexpr int TX_SW_CFG1_DLY_RFTR_DIS = 16;

constexpr uint16_t TX_SW_CFG2 = 0x1338;
constexpr int TX_SW_CFG2_DLY_DAC_DIS = 0;
constexpr int TX_SW_CFG2_DLY_DAC_EN = 8;
constexpr int TX_SW_CFG2_DLY_LNA_DIS = 16;
constexpr int TX_SW_CFG2_DLY_LNA_EN = 24;

constexpr uint16_t TXOP_CTRL_CFG = 0x1340;
constexpr int TXOP_CTRL_CFG_TXOP_TRUN_EN_OFFSET = 0;
constexpr int TXOP_CTRL_CFG_TXOP_TRUN_EN_WIDTH = 6;
constexpr int TXOP_CTRL_CFG_LSIG_TXOP_EN = 6;
constexpr int TXOP_CTRL_CFG_EXT_CCA_EN = 7;
constexpr int TXOP_CTRL_CFG_EXT_CCA_DLY_OFFSET = 8;
constexpr int TXOP_CTRL_CFG_EXT_CCA_DLY_WIDTH = 8;
constexpr int TXOP_CTRL_CFG_EXT_CW_MIN_OFFSET = 16;
constexpr int TXOP_CTRL_CFG_EXT_CW_MIN_WIDTH = 4;
constexpr int TXOP_CTRL_CFG_ED_CCA_EN = 20;

constexpr uint16_t TX_RTS_CFG = 0x1344;
constexpr int TX_RTS_CFG_RTS_RTY_LIMIT_OFFSET = 0;
constexpr int TX_RTS_CFG_RTS_RTY_LIMIT_WIDTH = 8;
constexpr int TX_RTS_CFG_RTS_THRES_OFFSET = 8;
constexpr int TX_RTS_CFG_RTS_THRES_WIDTH = 16;
constexpr int TX_RTS_CFG_RTS_FBK_EN = 24;

constexpr uint16_t TX_TIMEOUT_CFG = 0x1348;
constexpr int TX_TIMEOUT_CFG_MPDU_LIFE_TIME_OFFSET = 4;
constexpr int TX_TIMEOUT_CFG_MPDU_LIFE_TIME_WIDTH = 4;
constexpr int TX_TIMEOUT_CFG_RX_ACK_TIMEOUT_OFFSET = 8;
constexpr int TX_TIMEOUT_CFG_RX_ACK_TIMEOUT_WIDTH = 8;
constexpr int TX_TIMEOUT_CFG_TXOP_TIMEOUT_OFFSET = 16;
constexpr int TX_TIMEOUT_CFG_TXOP_TIMEOUT_WIDTH = 8;
constexpr int TX_TIMEOUT_CFG_ACKTO_END_TXOP = 24;

constexpr uint16_t TX_RTY_CFG = 0x134c;
constexpr int TX_RTY_CFG_SHORT_RTY_LIMIT_OFFSET = 0;
constexpr int TX_RTY_CFG_SHORT_RTY_LIMIT_WIDTH = 8;
constexpr int TX_RTY_CFG_LONG_RTY_LIMIT_OFFSET = 8;
constexpr int TX_RTY_CFG_LONG_RTY_LIMIT_WIDTH = 8;
constexpr int TX_RTY_CFG_LONG_RTY_THRES_OFFSET = 16;
constexpr int TX_RTY_CFG_LONG_RTY_THRES_WIDTH = 12;
constexpr int TX_RTY_CFG_NAG_RTY_MODE = 28;
constexpr int TX_RTY_CFG_AGG_RTY_MODE = 29;
constexpr int TX_RTY_CFG_TX_AUTOFB_EN = 30;

constexpr uint16_t TX_LINK_CFG = 0x1350;
constexpr int TX_LINK_CFG_REMOTE_MFB_LIFETIME_OFFSET = 0;
constexpr int TX_LINK_CFG_REMOTE_MFB_LIFETIME_WIDTH = 8;
constexpr int TX_LINK_CFG_TX_MFB_EN = 8;
constexpr int TX_LINK_CFG_REMOTE_UMFS_EN = 9;
constexpr int TX_LINK_CFG_TX_MRQ_EN = 10;
constexpr int TX_LINK_CFG_TX_RDG_EN = 11;
constexpr int TX_LINK_CFG_TX_CFACK_EN = 12;
constexpr int TX_LINK_CFG_REMOTE_MFB_OFFSET = 16;
constexpr int TX_LINK_CFG_REMOTE_MFB_WIDTH = 8;
constexpr int TX_LINK_CFG_REMOTE_MFS_OFFSET = 24;
constexpr int TX_LINK_CFG_REMOTE_MFS_WIDTH = 8;

constexpr uint16_t CCK_PROT_CFG = 0x1364;
constexpr int CCK_PROT_CFG_CCK_PROT_RATE_OFFSET = 0;
constexpr int CCK_PROT_CFG_CCK_PROT_RATE_WIDTH = 16;
constexpr int CCK_PROT_CFG_CCK_PROT_CTRL_OFFSET = 16;
constexpr int CCK_PROT_CFG_CCK_PROT_CTRL_WIDTH = 2;
constexpr int CCK_PROT_CFG_CCK_PROT_NAV_OFFSET = 18;
constexpr int CCK_PROT_CFG_CCK_PROT_NAV_WIDTH = 2;
constexpr int CCK_PROT_CFG_CCK_PROT_TXOP_ALLOW_CCK_TX = 20;
constexpr int CCK_PROT_CFG_CCK_PROT_TXOP_ALLOW_OFDM_TX = 21;
constexpr int CCK_PROT_CFG_CCK_PROT_TXOP_ALLOW_MM20_TX = 22;
constexpr int CCK_PROT_CFG_CCK_PROT_TXOP_ALLOW_MM40_TX = 23;
constexpr int CCK_PROT_CFG_CCK_PROT_TXOP_ALLOW_GF20_TX = 24;
constexpr int CCK_PROT_CFG_CCK_PROT_TXOP_ALLOW_GF40_TX = 25;
constexpr int CCK_PROT_CFG_CCK_RTSTH_EN = 26;

constexpr uint16_t OFDM_PROT_CFG = 0x1368;
constexpr int OFDM_PROT_CFG_OFDM_PROT_RATE_OFFSET = 0;
constexpr int OFDM_PROT_CFG_OFDM_PROT_RATE_WIDTH = 16;
constexpr int OFDM_PROT_CFG_OFDM_PROT_CTRL_OFFSET = 16;
constexpr int OFDM_PROT_CFG_OFDM_PROT_CTRL_WIDTH = 2;
constexpr int OFDM_PROT_CFG_OFDM_PROT_NAV_OFFSET = 18;
constexpr int OFDM_PROT_CFG_OFDM_PROT_NAV_WIDTH = 2;
constexpr int OFDM_PROT_CFG_OFDM_PROT_TXOP_ALLOW_CCK_TX = 20;
constexpr int OFDM_PROT_CFG_OFDM_PROT_TXOP_ALLOW_OFDM_TX = 21;
constexpr int OFDM_PROT_CFG_OFDM_PROT_TXOP_ALLOW_MM20_TX = 22;
constexpr int OFDM_PROT_CFG_OFDM_PROT_TXOP_ALLOW_MM40_TX = 23;
constexpr int OFDM_PROT_CFG_OFDM_PROT_TXOP_ALLOW_GF20_TX = 24;
constexpr int OFDM_PROT_CFG_OFDM_PROT_TXOP_ALLOW_GF40_TX = 25;
constexpr int OFDM_PROT_CFG_OFDM_RTSTH_EN = 26;

constexpr uint16_t MM20_PROT_CFG = 0x136c;
constexpr int MM20_PROT_CFG_MM20_PROT_RATE_OFFSET = 0;
constexpr int MM20_PROT_CFG_MM20_PROT_RATE_WIDTH = 16;
constexpr int MM20_PROT_CFG_MM20_PROT_CTRL_OFFSET = 16;
constexpr int MM20_PROT_CFG_MM20_PROT_CTRL_WIDTH = 2;
constexpr int MM20_PROT_CFG_MM20_PROT_NAV_OFFSET = 18;
constexpr int MM20_PROT_CFG_MM20_PROT_NAV_WIDTH = 2;
constexpr int MM20_PROT_CFG_MM20_PROT_TXOP_ALLOW_CCK_TX = 20;
constexpr int MM20_PROT_CFG_MM20_PROT_TXOP_ALLOW_OFDM_TX = 21;
constexpr int MM20_PROT_CFG_MM20_PROT_TXOP_ALLOW_MM20_TX = 22;
constexpr int MM20_PROT_CFG_MM20_PROT_TXOP_ALLOW_MM40_TX = 23;
constexpr int MM20_PROT_CFG_MM20_PROT_TXOP_ALLOW_GF20_TX = 24;
constexpr int MM20_PROT_CFG_MM20_PROT_TXOP_ALLOW_GF40_TX = 25;
constexpr int MM20_PROT_CFG_MM20_RTSTH_EN = 26;

constexpr uint16_t MM40_PROT_CFG = 0x1370;
constexpr int MM40_PROT_CFG_MM40_PROT_RATE_OFFSET = 0;
constexpr int MM40_PROT_CFG_MM40_PROT_RATE_WIDTH = 16;
constexpr int MM40_PROT_CFG_MM40_PROT_CTRL_OFFSET = 16;
constexpr int MM40_PROT_CFG_MM40_PROT_CTRL_WIDTH = 2;
constexpr int MM40_PROT_CFG_MM40_PROT_NAV_OFFSET = 18;
constexpr int MM40_PROT_CFG_MM40_PROT_NAV_WIDTH = 2;
constexpr int MM40_PROT_CFG_MM40_PROT_TXOP_ALLOW_CCK_TX = 20;
constexpr int MM40_PROT_CFG_MM40_PROT_TXOP_ALLOW_OFDM_TX = 21;
constexpr int MM40_PROT_CFG_MM40_PROT_TXOP_ALLOW_MM20_TX = 22;
constexpr int MM40_PROT_CFG_MM40_PROT_TXOP_ALLOW_MM40_TX = 23;
constexpr int MM40_PROT_CFG_MM40_PROT_TXOP_ALLOW_GF20_TX = 24;
constexpr int MM40_PROT_CFG_MM40_PROT_TXOP_ALLOW_GF40_TX = 25;
constexpr int MM40_PROT_CFG_MM40_RTSTH_EN = 26;

constexpr uint16_t GF20_PROT_CFG = 0x1374;
constexpr int GF20_PROT_CFG_GF20_PROT_RATE_OFFSET = 0;
constexpr int GF20_PROT_CFG_GF20_PROT_RATE_WIDTH = 16;
constexpr int GF20_PROT_CFG_GF20_PROT_CTRL_OFFSET = 16;
constexpr int GF20_PROT_CFG_GF20_PROT_CTRL_WIDTH = 2;
constexpr int GF20_PROT_CFG_GF20_PROT_NAV_OFFSET = 18;
constexpr int GF20_PROT_CFG_GF20_PROT_NAV_WIDTH = 2;
constexpr int GF20_PROT_CFG_GF20_PROT_TXOP_ALLOW_CCK_TX = 20;
constexpr int GF20_PROT_CFG_GF20_PROT_TXOP_ALLOW_OFDM_TX = 21;
constexpr int GF20_PROT_CFG_GF20_PROT_TXOP_ALLOW_MM20_TX = 22;
constexpr int GF20_PROT_CFG_GF20_PROT_TXOP_ALLOW_MM40_TX = 23;
constexpr int GF20_PROT_CFG_GF20_PROT_TXOP_ALLOW_GF20_TX = 24;
constexpr int GF20_PROT_CFG_GF20_PROT_TXOP_ALLOW_GF40_TX = 25;
constexpr int GF20_PROT_CFG_GF20_RTSTH_EN = 26;

constexpr uint16_t GF40_PROT_CFG = 0x1378;
constexpr int GF40_PROT_CFG_GF40_PROT_RATE_OFFSET = 0;
constexpr int GF40_PROT_CFG_GF40_PROT_RATE_WIDTH = 16;
constexpr int GF40_PROT_CFG_GF40_PROT_CTRL_OFFSET = 16;
constexpr int GF40_PROT_CFG_GF40_PROT_CTRL_WIDTH = 2;
constexpr int GF40_PROT_CFG_GF40_PROT_NAV_OFFSET = 18;
constexpr int GF40_PROT_CFG_GF40_PROT_NAV_WIDTH = 2;
constexpr int GF40_PROT_CFG_GF40_PROT_TXOP_ALLOW_CCK_TX = 20;
constexpr int GF40_PROT_CFG_GF40_PROT_TXOP_ALLOW_OFDM_TX = 21;
constexpr int GF40_PROT_CFG_GF40_PROT_TXOP_ALLOW_MM20_TX = 22;
constexpr int GF40_PROT_CFG_GF40_PROT_TXOP_ALLOW_MM40_TX = 23;
constexpr int GF40_PROT_CFG_GF40_PROT_TXOP_ALLOW_GF20_TX = 24;
constexpr int GF40_PROT_CFG_GF40_PROT_TXOP_ALLOW_GF40_TX = 25;
constexpr int GF40_PROT_CFG_GF40_RTSTH_EN = 26;

constexpr uint16_t EXP_ACK_TIME = 0x1380;
constexpr int EXP_ACK_TIME_EXP_CCK_ACK_TIME_OFFSET = 0;
constexpr int EXP_ACK_TIME_EXP_CCK_ACK_TIME_WIDTH = 15;
constexpr int EXP_ACK_TIME_EXP_OFDM_ACK_TIME_OFFSET = 16;
constexpr int EXP_ACK_TIME_EXP_OFDM_ACK_TIME_WIDTH = 15;

constexpr uint16_t RX_FILTR_CFG = 0x1400;
constexpr int RX_FILTR_CFG_DROP_CRC_ERR = 0;
constexpr int RX_FILTR_CFG_DROP_PHY_ERR = 1;
constexpr int RX_FILTR_CFG_DROP_UC_NOME = 2;
constexpr int RX_FILTR_CFG_DROP_NOT_MYBSS = 3;
constexpr int RX_FILTR_CFG_DROP_VER_ERR = 4;
constexpr int RX_FILTR_CFG_DROP_MC = 5;
constexpr int RX_FILTR_CFG_DROP_BC = 6;
constexpr int RX_FILTR_CFG_DROP_DUPL = 7;
constexpr int RX_FILTR_CFG_DROP_CFACK = 8;
constexpr int RX_FILTR_CFG_DROP_CFEND = 9;
constexpr int RX_FILTR_CFG_DROP_ACK = 10;
constexpr int RX_FILTR_CFG_DROP_CTS = 11;
constexpr int RX_FILTR_CFG_DROP_RTS = 12;
constexpr int RX_FILTR_CFG_DROP_PSPOLL = 13;
constexpr int RX_FILTR_CFG_DROP_BA = 14;
constexpr int RX_FILTR_CFG_DROP_BAR = 15;
constexpr int RX_FILTR_CFG_DROP_CTRL_RSV = 16;

constexpr uint16_t AUTO_RSP_CFG = 0x1404;
constexpr int AUTO_RSP_CFG_AUTO_RSP_EN = 0;
constexpr int AUTO_RSP_CFG_BAC_ACKPOLICY_EN = 1;
constexpr int AUTO_RSP_CFG_CTS_40M_MODE = 2;
constexpr int AUTO_RSP_CFG_CTS_40M_REF = 3;
constexpr int AUTO_RSP_CFG_CCK_SHORT_EN = 4;
constexpr int AUTO_RSP_CFG_CTRL_WRAP_EN = 5;
constexpr int AUTO_RSP_CFG_BAC_ACK_POLICY = 6;
constexpr int AUTO_RSP_CFG_CTRL_PWR_BIT = 7;

constexpr uint16_t LEGACY_BASIC_RATE = 0x1408;
constexpr int LEGACY_BASIC_RATE_1MBPS = 0;
constexpr int LEGACY_BASIC_RATE_2MBPS = 1;
constexpr int LEGACY_BASIC_RATE_5_5MBPS = 2;
constexpr int LEGACY_BASIC_RATE_11MBPS = 3;
constexpr int LEGACY_BASIC_RATE_6MBPS = 4;
constexpr int LEGACY_BASIC_RATE_9MBPS = 5;
constexpr int LEGACY_BASIC_RATE_12MBPS = 6;
constexpr int LEGACY_BASIC_RATE_18MBPS = 7;
constexpr int LEGACY_BASIC_RATE_24MBPS = 8;
constexpr int LEGACY_BASIC_RATE_36MBPS = 9;
constexpr int LEGACY_BASIC_RATE_48MBPS = 10;
constexpr int LEGACY_BASIC_RATE_54MBPS = 11;

constexpr uint16_t HT_BASIC_RATE = 0x140c;

constexpr uint16_t TXOP_HLDR_ET = 0x1608;
constexpr int TXOP_HLDR_ET_PER_RX_RST_EN = 0;
constexpr int TXOP_HLDR_ET_TX40M_BLK_EN = 1;
constexpr int TXOP_HLDR_ET_TX_BCN_HIPRI_DIS = 2;
constexpr int TXOP_HLDR_ET_PAPE_MAP1S_EN = 3;
constexpr int TXOP_HLDR_ET_PAPE_MAP = 4;
constexpr int TXOP_HLDR_ET_TX_FBK_THRES_OFFSET = 16;
constexpr int TXOP_HLDR_ET_TX_FBK_THRES_WIDTH = 2;
constexpr int TXOP_HLDR_ET_TX_FBK_THRES_EN = 18;
constexpr int TXOP_HLDR_ET_TX_DMA_TIMEOUT_OFFSET = 19;
constexpr int TXOP_HLDR_ET_TX_DMA_TIMEOUT_WIDTH = 5;
constexpr int TXOP_HLDR_ET_AMPDU_ACC_EN = 24;

// EEPROM offsets
constexpr uint16_t EEPROM_CHIP_ID = 0x0000;
constexpr uint16_t EEPROM_VERSION = 0x0001;
constexpr uint16_t EEPROM_MAC_ADDR_0 = 0x0002;
constexpr uint16_t EEPROM_MAC_ADDR_1 = 0x0003;
constexpr uint16_t EEPROM_MAC_ADDR_2 = 0x0004;
constexpr uint16_t EEPROM_NIC_CONF0 = 0x001a;
constexpr uint16_t EEPROM_NIC_CONF1 = 0x001b;
constexpr uint16_t EEPROM_FREQ = 0x001d;
constexpr uint16_t EEPROM_NIC_CONF2 = 0x0021;
constexpr uint16_t EEPROM_LNA = 0x0022;
constexpr uint16_t EEPROM_RSSI_BG = 0x0023;
constexpr uint16_t EEPROM_RSSI_BG2 = 0x0024;
constexpr uint16_t EEPROM_RSSI_A = 0x0025;
constexpr uint16_t EEPROM_RSSI_A2 = 0x0026;

constexpr int EEPROM_FREQ_OFFSET = 0;
constexpr int EEPROM_FREQ_WIDTH = 8;

constexpr int EEPROM_LNA_A0_OFFSET = 8;
constexpr int EEPROM_LNA_A0_WIDTH = 8;

constexpr int EEPROM_RSSI_BG_0_OFFSET = 0;
constexpr int EEPROM_RSSI_BG_0_WIDTH = 8;
constexpr int EEPROM_RSSI_BG_1_OFFSET = 8;
constexpr int EEPROM_RSSI_BG_1_WIDTH = 8;

constexpr int EEPROM_RSSI_BG2_OFFSET = 0;
constexpr int EEPROM_RSSI_BG2_WIDTH = 8;
constexpr int EEPROM_RSSI_BG2_LNA_A1_OFFSET = 8;
constexpr int EEPROM_RSSI_BG2_LNA_A1_WIDTH = 8;

constexpr int EEPROM_NIC_CONF1_HW_RADIO   = 0;  // 0x0001
constexpr int EEPROM_NIC_CONF1_BT_COEXIST = 14; // 0x4000

// Host to MCU communication
constexpr uint16_t H2M_MAILBOX_CSR = 0x7010;
constexpr int H2M_MAILBOX_CSR_ARG0_OFFSET = 0;
constexpr int H2M_MAILBOX_CSR_ARG1_OFFSET = 8;
constexpr int H2M_MAILBOX_CSR_CMD_TOKEN_OFFSET = 16;
constexpr int H2M_MAILBOX_CSR_OWNER_OFFSET = 24;
// All four fields have width 8
constexpr int H2M_MAILBOX_CSR_WIDTH = 8;

constexpr uint16_t H2M_MAILBOX_CID = 0x7014;
constexpr uint16_t H2M_MAILBOX_STATUS = 0x701c;
constexpr uint16_t H2M_INT_SRC = 0x7024;
constexpr uint16_t H2M_BBP_AGENT = 0x7028;

// MCU commands
constexpr uint8_t MCU_BOOT_SIGNAL = 0x72;
constexpr uint8_t MCU_WAKEUP = 0x31;

}  // namespace rt5370
