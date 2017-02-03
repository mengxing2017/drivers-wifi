// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include "register.h"

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

class WpdmaGloCfg : public Register<0x0208> {
  public:
    REG_FIELD(tx_dma_en, 0, 1);
    REG_FIELD(tx_dma_busy, 1, 1);
    REG_FIELD(rx_dma_en, 2, 1);
    REG_FIELD(rx_dma_busy, 3, 1);
    REG_FIELD(wpdma_bt_size, 4, 2);
    REG_FIELD(tx_wb_ddone, 6, 1);
    REG_FIELD(big_endian, 7, 1);
    REG_FIELD(hdr_seg_len, 8, 8);
};

class UsbDmaCfg : public Register<0x02a0> {
  public:
    REG_FIELD(rx_agg_to, 0, 8);
    REG_FIELD(rx_agg_limit, 8, 8);
    REG_FIELD(phy_wd_en, 16, 1);
    REG_FIELD(tx_clear, 19, 1);
    REG_FIELD(txop_hald, 20, 1);
    REG_FIELD(rx_agg_en, 21, 1);
    REG_FIELD(udma_rx_en, 22, 1);
    REG_FIELD(udma_tx_en, 23, 1);
    REG_FIELD(epout_vld, 24, 5);
    REG_FIELD(rx_busy, 30, 1);
    REG_FIELD(tx_busy, 31, 1);
};

class SysCtrl : public Register<0x0400> {
  public:
    REG_FIELD(mcu_ready, 7, 1);
    REG_FIELD(pme_oen, 13, 1);
};

class HostCmd : public Register<0x0404> {
  public:
    REG_FIELD(command, 0, 32);
};

class MaxPcnt : public Register<0x040c> {
  public:
    REG_FIELD(max_rx0q_pcnt, 0, 8);
    REG_FIELD(max_tx2q_pcnt, 8, 8);
    REG_FIELD(max_tx1q_pcnt, 16, 8);
    REG_FIELD(max_tx0q_pcnt, 24, 8);
};

class PbfCfg : public Register<0x0408> {
  public:
    REG_FIELD(rx0q_en, 1, 1);
    REG_FIELD(tx2q_en, 2, 1);
    REG_FIELD(tx1q_en, 3, 1);
    REG_FIELD(tx0q_en, 4, 1);
    REG_FIELD(hcca_mode, 8, 1);
    REG_FIELD(rx0q_mode, 9, 1);
    REG_FIELD(tx2q_mode, 10, 1);
    REG_FIELD(tx1q_mode, 11, 1);
    REG_FIELD(tx0q_mode, 12, 1);
    REG_FIELD(rx_drop_mode, 13, 1);
    REG_FIELD(null1_mode, 14, 1);
    REG_FIELD(null0_mode, 15, 1);
    REG_FIELD(tx2q_num, 16, 5);
    REG_FIELD(tx1q_num, 21, 3);
    REG_FIELD(null2_sel, 24, 3);
};

class EfuseCtrl : public Register<0x0580> {
  public:
    REG_FIELD(sel_efuse, 31, 1);
    REG_FIELD(efsrom_kick, 30, 1);
    REG_FIELD(efsrom_ain, 16, 10);
    REG_FIELD(efsrom_mode, 6, 2);
};

class RfuseData0 : public Register<0x059c> {};
class RfuseData1 : public Register<0x0598> {};
class RfuseData2 : public Register<0x0594> {};
class RfuseData3 : public Register<0x0590> {};

class AsicVerId : public Register<0x1000> {
  public:
    REG_FIELD(rev_id, 0, 16);
    REG_FIELD(ver_id, 16, 16);
};

class MacSysCtrl : public Register<0x1004> {
  public:
    REG_FIELD(mac_srst, 0, 1);
    REG_FIELD(bbp_hrst, 1, 1);
    REG_FIELD(mac_tx_en, 2, 1);
    REG_FIELD(mac_rx_en, 3, 1);
};

class MaxLenCfg : public Register<0x1018> {
  public:
    REG_FIELD(max_mpdu_len, 0, 12);
    REG_FIELD(max_psdu_len, 12, 2);
    REG_FIELD(min_psdu_len, 14, 2);  // From Linux kernel source
    REG_FIELD(min_mpdu_len, 16, 4);
};

class BkoffSlotCfg : public Register<0x1104> {
  public:
    REG_FIELD(slot_time, 0, 8);
    REG_FIELD(cc_delay_time, 8, 4);
};

class BcnTimeCfg : public Register<0x1114> {
  public:
    REG_FIELD(bcn_intval, 0, 16);
    REG_FIELD(tsf_timer_en, 16, 1);
    REG_FIELD(tsf_sync_mode, 17, 2);
    REG_FIELD(tbtt_timer_en, 19, 1);
    REG_FIELD(bcn_tx_en, 20, 1);
    REG_FIELD(tsf_ins_comp, 24, 8);
};

class AutoWakeupCfg : public Register<0x1208> {
  public:
    REG_FIELD(wakeup_lead_time, 0, 8);
    REG_FIELD(sleep_tbtt_num, 8, 7);
    REG_FIELD(auto_wakeup_en, 15, 1);
};

class TxSwCfg0 : public Register<0x1330> {
  public:
    REG_FIELD(dly_txpe_en, 0, 8);
    REG_FIELD(dly_pape_en, 8, 8);
    REG_FIELD(dly_trsw_en, 16, 8);
    REG_FIELD(dly_rftr_en, 24, 8);
};

class TxSwCfg1 : public Register<0x1334> {
  public:
    REG_FIELD(dly_pape_dis, 0, 8);
    REG_FIELD(dly_trsw_dis, 8, 8);
    REG_FIELD(dly_rftr_dis, 16, 8);
};

class TxSwCfg2 : public Register<0x1338> {
  public:
    REG_FIELD(dly_dac_dis, 0, 8);
    REG_FIELD(dly_dac_en, 8, 8);
    REG_FIELD(dly_lna_dis, 16, 8);
    REG_FIELD(dly_lna_en, 24, 8);
};

class TxopCtrlCfg : public Register<0x1340> {
  public:
    REG_FIELD(txop_trun_en, 0, 6);
    REG_FIELD(lsig_txop_en, 6, 1);
    REG_FIELD(ext_cca_en, 7, 1);
    REG_FIELD(ext_cca_dly, 8, 8);
    REG_FIELD(ext_cw_min, 16, 4);
    REG_FIELD(ed_cca_en, 20, 1);
};

class TxRtsCfg : public Register<0x1344> {
  public:
    REG_FIELD(rts_rty_limit, 0, 8);
    REG_FIELD(rts_thres, 8, 16);
    REG_FIELD(rts_fbk_en, 24, 1);
};

class TxTimeoutCfg : public Register<0x1348> {
  public:
    REG_FIELD(mpdu_life_time, 4, 4);
    REG_FIELD(rx_ack_timeout, 8, 8);
    REG_FIELD(txop_timeout, 16, 8);
    REG_FIELD(ackto_end_txop, 24, 1);
};

class TxRtyCfg : public Register<0x134c> {
  public:
    REG_FIELD(short_rty_limit, 0, 8);
    REG_FIELD(long_rty_limit, 8, 8);
    REG_FIELD(long_rty_thres, 16, 12);
    REG_FIELD(nag_rty_mode, 28, 1);
    REG_FIELD(agg_rty_mode, 29, 1);
    REG_FIELD(tx_autofb_en, 30, 1);
};

class TxLinkCfg : public Register<0x1350> {
  public:
    REG_FIELD(remote_mfb_lifetime, 0, 8);
    REG_FIELD(tx_mfb_en, 8, 1);
    REG_FIELD(remote_umfs_en, 9, 1);
    REG_FIELD(tx_mrq_en, 10, 1);
    REG_FIELD(tx_rdg_en, 11, 1);
    REG_FIELD(tx_cfack_en, 12, 1);
    REG_FIELD(remote_mfb, 16, 8);
    REG_FIELD(remote_mfs, 24, 8);
};

template<uint16_t A>
class ProtCfg : public Register<A> {
  public:
    REG_FIELD(prot_rate, 0, 16);
    REG_FIELD(prot_ctrl, 16, 2);
    REG_FIELD(prot_nav, 18, 2);
    REG_FIELD(txop_allow_cck_tx, 20, 1);
    REG_FIELD(txop_allow_ofdm_tx, 21, 1);
    REG_FIELD(txop_allow_mm20_tx, 22, 1);
    REG_FIELD(txop_allow_mm40_tx, 23, 1);
    REG_FIELD(txop_allow_gf20_tx, 24, 1);
    REG_FIELD(txop_allow_gf40_tx, 25, 1);
    REG_FIELD(rtsth_en, 26, 1);
};

class CckProtCfg : public ProtCfg<0x1364> {};
class OfdmProtCfg : public ProtCfg<0x1368> {};
class Mm20ProtCfg : public ProtCfg<0x136c> {};
class Mm40ProtCfg : public ProtCfg<0x1370> {};
class Gf20ProtCfg : public ProtCfg<0x1374> {};
class Gf40ProtCfg : public ProtCfg<0x1378> {};

class ExpAckTime : public Register<0x1380> {
  public:
    REG_FIELD(exp_cck_ack_time, 0, 15);
    REG_FIELD(exp_ofdm_ack_time, 16, 15);
};

class RxFiltrCfg : public Register<0x1400> {
  public:
    REG_FIELD(drop_crc_err, 0, 1);
    REG_FIELD(drop_phy_err, 1, 1);
    REG_FIELD(drop_uc_nome, 2, 1);
    REG_FIELD(drop_not_mybss, 3, 1);
    REG_FIELD(drop_ver_err, 4, 1);
    REG_FIELD(drop_mc, 5, 1);
    REG_FIELD(drop_bc, 6, 1);
    REG_FIELD(drop_dupl, 7, 1);
    REG_FIELD(drop_cfack, 8, 1);
    REG_FIELD(drop_cfend, 9, 1);
    REG_FIELD(drop_ack, 10, 1);
    REG_FIELD(drop_cts, 11, 1);
    REG_FIELD(drop_rts, 12, 1);
    REG_FIELD(drop_pspoll, 13, 1);
    REG_FIELD(drop_ba, 14, 1);
    REG_FIELD(drop_bar, 15, 1);
    REG_FIELD(drop_ctrl_rsv, 16, 1);
};

class AutoRspCfg : public Register<0x1404> {
  public:
    REG_FIELD(auto_rsp_en, 0, 1);
    REG_FIELD(bac_ackpolicy_en, 1, 1);
    REG_FIELD(cts_40m_mode, 2, 1);
    REG_FIELD(cts_40m_ref, 3, 1);
    REG_FIELD(cck_short_en, 4, 1);
    REG_FIELD(ctrl_wrap_en, 5, 1);
    REG_FIELD(bac_ack_policy, 6, 1);
    REG_FIELD(ctrl_pwr_bit, 7, 1);
};

class LegacyBasicRate : public Register<0x1408> {
  public:
    REG_FIELD(rate_1mbps, 0, 1);
    REG_FIELD(rate_2mbps, 1, 1);
    REG_FIELD(rate_5_5mbps, 2, 1);
    REG_FIELD(rate_11mbps, 3, 1);
    REG_FIELD(rate_6mbps, 4, 1);
    REG_FIELD(rate_9mbps, 5, 1);
    REG_FIELD(rate_12mbps, 6, 1);
    REG_FIELD(rate_18mbps, 7, 1);
    REG_FIELD(rate_24mbps, 8, 1);
    REG_FIELD(rate_36mbps, 9, 1);
    REG_FIELD(rate_48mbps, 10, 1);
    REG_FIELD(rate_54mbps, 11, 1);
};

class HtBasicRate : public Register<0x140c> {
    // TODO: figure out what these bits are
};

class TxopHldrEt : public Register<0x1608> {
  public:
    REG_FIELD(per_rx_rst_en, 0, 1);
    REG_FIELD(tx40m_blk_en, 1, 1);
    REG_FIELD(tx_bcn_hipri_dis, 2, 1);
    REG_FIELD(pape_map1s_en, 3, 1);
    REG_FIELD(pape_map, 4, 1);
    REG_FIELD(tx_fbk_thres, 16, 2);
    REG_FIELD(tx_fbk_thres_en, 18, 1);
    REG_FIELD(tx_dma_timeout, 19, 5);
    REG_FIELD(ampdu_acc_en, 24, 1);
};

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
class H2mMailboxCsr : public Register<0x7010> {
  public:
    REG_FIELD(arg0, 0, 8);
    REG_FIELD(arg1, 8, 8);
    REG_FIELD(cmd_token, 16, 8);
    REG_FIELD(owner, 24, 8);
};

constexpr uint16_t H2M_MAILBOX_CID = 0x7014;
constexpr uint16_t H2M_MAILBOX_STATUS = 0x701c;
constexpr uint16_t H2M_INT_SRC = 0x7024;
constexpr uint16_t H2M_BBP_AGENT = 0x7028;

// MCU commands
constexpr uint8_t MCU_BOOT_SIGNAL = 0x72;
constexpr uint8_t MCU_WAKEUP = 0x31;

}  // namespace rt5370
