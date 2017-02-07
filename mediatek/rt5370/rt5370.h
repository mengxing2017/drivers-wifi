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
constexpr uint16_t REV_RT5390F = 0x0502;
constexpr uint16_t REV_RT5390R = 0x1502;
constexpr uint16_t FW_IMAGE_BASE = 0x3000;

// Registers

class WpdmaGloCfg : public Register<0x0208> {
  public:
    BIT_FIELD(tx_dma_en, 0, 1);
    BIT_FIELD(tx_dma_busy, 1, 1);
    BIT_FIELD(rx_dma_en, 2, 1);
    BIT_FIELD(rx_dma_busy, 3, 1);
    BIT_FIELD(wpdma_bt_size, 4, 2);
    BIT_FIELD(tx_wb_ddone, 6, 1);
    BIT_FIELD(big_endian, 7, 1);
    BIT_FIELD(hdr_seg_len, 8, 8);
};

class UsbDmaCfg : public Register<0x02a0> {
  public:
    BIT_FIELD(rx_agg_to, 0, 8);
    BIT_FIELD(rx_agg_limit, 8, 8);
    BIT_FIELD(phy_wd_en, 16, 1);
    BIT_FIELD(tx_clear, 19, 1);
    BIT_FIELD(txop_hald, 20, 1);
    BIT_FIELD(rx_agg_en, 21, 1);
    BIT_FIELD(udma_rx_en, 22, 1);
    BIT_FIELD(udma_tx_en, 23, 1);
    BIT_FIELD(epout_vld, 24, 5);
    BIT_FIELD(rx_busy, 30, 1);
    BIT_FIELD(tx_busy, 31, 1);
};

class UsCycCnt : public Register<0x02a4> {
  public:
    BIT_FIELD(us_cyc_count, 0, 8);
    BIT_FIELD(bt_mode_en, 8, 1);
    BIT_FIELD(test_sel, 16, 8);
    BIT_FIELD(test_en, 24, 1);
    BIT_FIELD(edt_bypass, 28, 1);
};

class SysCtrl : public Register<0x0400> {
  public:
    BIT_FIELD(mcu_ready, 7, 1);
    BIT_FIELD(pme_oen, 13, 1);
};

class HostCmd : public Register<0x0404> {
  public:
    BIT_FIELD(command, 0, 32);
};

class MaxPcnt : public Register<0x040c> {
  public:
    BIT_FIELD(max_rx0q_pcnt, 0, 8);
    BIT_FIELD(max_tx2q_pcnt, 8, 8);
    BIT_FIELD(max_tx1q_pcnt, 16, 8);
    BIT_FIELD(max_tx0q_pcnt, 24, 8);
};

class PbfCfg : public Register<0x0408> {
  public:
    BIT_FIELD(rx0q_en, 1, 1);
    BIT_FIELD(tx2q_en, 2, 1);
    BIT_FIELD(tx1q_en, 3, 1);
    BIT_FIELD(tx0q_en, 4, 1);
    BIT_FIELD(hcca_mode, 8, 1);
    BIT_FIELD(rx0q_mode, 9, 1);
    BIT_FIELD(tx2q_mode, 10, 1);
    BIT_FIELD(tx1q_mode, 11, 1);
    BIT_FIELD(tx0q_mode, 12, 1);
    BIT_FIELD(rx_drop_mode, 13, 1);
    BIT_FIELD(null1_mode, 14, 1);
    BIT_FIELD(null0_mode, 15, 1);
    BIT_FIELD(tx2q_num, 16, 5);
    BIT_FIELD(tx1q_num, 21, 3);
    BIT_FIELD(null2_sel, 24, 3);
};

class RfCsrCfg : public Register<0x0500> {
  public:
    BIT_FIELD(rf_csr_data, 0, 8);
    BIT_FIELD(rf_csr_addr, 8, 6);
    BIT_FIELD(rf_csr_rw, 16, 1);
    BIT_FIELD(rf_csr_kick, 17, 1);
};

class EfuseCtrl : public Register<0x0580> {
  public:
    BIT_FIELD(sel_efuse, 31, 1);
    BIT_FIELD(efsrom_kick, 30, 1);
    BIT_FIELD(efsrom_ain, 16, 10);
    BIT_FIELD(efsrom_mode, 6, 2);
};

class RfuseData0 : public Register<0x059c> {};
class RfuseData1 : public Register<0x0598> {};
class RfuseData2 : public Register<0x0594> {};
class RfuseData3 : public Register<0x0590> {};

class AsicVerId : public Register<0x1000> {
  public:
    BIT_FIELD(rev_id, 0, 16);
    BIT_FIELD(ver_id, 16, 16);
};

class MacSysCtrl : public Register<0x1004> {
  public:
    BIT_FIELD(mac_srst, 0, 1);
    BIT_FIELD(bbp_hrst, 1, 1);
    BIT_FIELD(mac_tx_en, 2, 1);
    BIT_FIELD(mac_rx_en, 3, 1);
};

class MaxLenCfg : public Register<0x1018> {
  public:
    BIT_FIELD(max_mpdu_len, 0, 12);
    BIT_FIELD(max_psdu_len, 12, 2);
    BIT_FIELD(min_psdu_len, 14, 2);  // From Linux kernel source
    BIT_FIELD(min_mpdu_len, 16, 4);
};

class BbpCsrCfg : public Register<0x101c> {
  public:
    BIT_FIELD(bbp_data, 0, 8);
    BIT_FIELD(bbp_addr, 8, 8);
    BIT_FIELD(bbp_csr_rw, 16, 1);
    BIT_FIELD(bbp_csr_kick, 17, 1);
    BIT_FIELD(bbp_par_dur, 18, 1);
    BIT_FIELD(bbp_rw_mode, 19, 1);
};

class ForceBaWinsize : public Register<0x1040> {
  public:
    BIT_FIELD(force_ba_winsize, 0, 6);
    BIT_FIELD(force_ba_winsize_en, 6, 1);
};

class XifsTimeCfg : public Register<0x1100> {
  public:
    BIT_FIELD(cck_sifs_time, 0, 8);
    BIT_FIELD(ofdm_sifs_time, 8, 8);
    BIT_FIELD(ofdm_xifs_time, 16, 4);
    BIT_FIELD(eifs_time, 20, 9);
    BIT_FIELD(bb_rxend_en, 29, 1);
};

class BkoffSlotCfg : public Register<0x1104> {
  public:
    BIT_FIELD(slot_time, 0, 8);
    BIT_FIELD(cc_delay_time, 8, 4);
};

class ChTimeCfg : public Register<0x110c> {
  public:
    BIT_FIELD(ch_sta_timer_en, 0, 1);
    BIT_FIELD(tx_as_ch_busy, 1, 1);
    BIT_FIELD(rx_as_ch_busy, 2, 1);
    BIT_FIELD(nav_as_ch_busy, 3, 1);
    BIT_FIELD(eifs_as_ch_busy, 4, 1);
};

class BcnTimeCfg : public Register<0x1114> {
  public:
    BIT_FIELD(bcn_intval, 0, 16);
    BIT_FIELD(tsf_timer_en, 16, 1);
    BIT_FIELD(tsf_sync_mode, 17, 2);
    BIT_FIELD(tbtt_timer_en, 19, 1);
    BIT_FIELD(bcn_tx_en, 20, 1);
    BIT_FIELD(tsf_ins_comp, 24, 8);
};

class IntTimerCfg : public Register<0x1128> {
  public:
    BIT_FIELD(pre_tbtt_timer, 0, 16);
    BIT_FIELD(gp_timer, 16, 16);
};

class MacStatusReg : public Register<0x1200> {
  public:
    BIT_FIELD(tx_status, 0, 1);
    BIT_FIELD(rx_status, 1, 1);
};

class PwrPinCfg : public Register<0x1204> {
  public:
    BIT_FIELD(io_rf_pe, 0, 1);
    BIT_FIELD(io_ra_pe, 1, 1);
    BIT_FIELD(io_pll_pd, 2, 1);
    BIT_FIELD(io_adda_pd, 3, 1);
};

class AutoWakeupCfg : public Register<0x1208> {
  public:
    BIT_FIELD(wakeup_lead_time, 0, 8);
    BIT_FIELD(sleep_tbtt_num, 8, 7);
    BIT_FIELD(auto_wakeup_en, 15, 1);
};

class TxSwCfg0 : public Register<0x1330> {
  public:
    BIT_FIELD(dly_txpe_en, 0, 8);
    BIT_FIELD(dly_pape_en, 8, 8);
    BIT_FIELD(dly_trsw_en, 16, 8);
    BIT_FIELD(dly_rftr_en, 24, 8);
};

class TxSwCfg1 : public Register<0x1334> {
  public:
    BIT_FIELD(dly_pape_dis, 0, 8);
    BIT_FIELD(dly_trsw_dis, 8, 8);
    BIT_FIELD(dly_rftr_dis, 16, 8);
};

class TxSwCfg2 : public Register<0x1338> {
  public:
    BIT_FIELD(dly_dac_dis, 0, 8);
    BIT_FIELD(dly_dac_en, 8, 8);
    BIT_FIELD(dly_lna_dis, 16, 8);
    BIT_FIELD(dly_lna_en, 24, 8);
};

class TxopCtrlCfg : public Register<0x1340> {
  public:
    BIT_FIELD(txop_trun_en, 0, 6);
    BIT_FIELD(lsig_txop_en, 6, 1);
    BIT_FIELD(ext_cca_en, 7, 1);
    BIT_FIELD(ext_cca_dly, 8, 8);
    BIT_FIELD(ext_cw_min, 16, 4);
    BIT_FIELD(ed_cca_en, 20, 1);
};

class TxRtsCfg : public Register<0x1344> {
  public:
    BIT_FIELD(rts_rty_limit, 0, 8);
    BIT_FIELD(rts_thres, 8, 16);
    BIT_FIELD(rts_fbk_en, 24, 1);
};

class TxTimeoutCfg : public Register<0x1348> {
  public:
    BIT_FIELD(mpdu_life_time, 4, 4);
    BIT_FIELD(rx_ack_timeout, 8, 8);
    BIT_FIELD(txop_timeout, 16, 8);
    BIT_FIELD(ackto_end_txop, 24, 1);
};

class TxRtyCfg : public Register<0x134c> {
  public:
    BIT_FIELD(short_rty_limit, 0, 8);
    BIT_FIELD(long_rty_limit, 8, 8);
    BIT_FIELD(long_rty_thres, 16, 12);
    BIT_FIELD(nag_rty_mode, 28, 1);
    BIT_FIELD(agg_rty_mode, 29, 1);
    BIT_FIELD(tx_autofb_en, 30, 1);
};

class TxLinkCfg : public Register<0x1350> {
  public:
    BIT_FIELD(remote_mfb_lifetime, 0, 8);
    BIT_FIELD(tx_mfb_en, 8, 1);
    BIT_FIELD(remote_umfs_en, 9, 1);
    BIT_FIELD(tx_mrq_en, 10, 1);
    BIT_FIELD(tx_rdg_en, 11, 1);
    BIT_FIELD(tx_cfack_en, 12, 1);
    BIT_FIELD(remote_mfb, 16, 8);
    BIT_FIELD(remote_mfs, 24, 8);
};

class HtFbkCfg0 : public Register<0x1354> {
  public:
    BIT_FIELD(ht_mcs0_fbk, 0, 4);
    BIT_FIELD(ht_mcs1_fbk, 4, 4);
    BIT_FIELD(ht_mcs2_fbk, 8, 4);
    BIT_FIELD(ht_mcs3_fbk, 12, 4);
    BIT_FIELD(ht_mcs4_fbk, 16, 4);
    BIT_FIELD(ht_mcs5_fbk, 20, 4);
    BIT_FIELD(ht_mcs6_fbk, 24, 4);
    BIT_FIELD(ht_mcs7_fbk, 28, 4);
};

class HtFbkCfg1 : public Register<0x1358> {
  public:
    BIT_FIELD(ht_mcs8_fbk, 0, 4);
    BIT_FIELD(ht_mcs9_fbk, 4, 4);
    BIT_FIELD(ht_mcs10_fbk, 8, 4);
    BIT_FIELD(ht_mcs11_fbk, 12, 4);
    BIT_FIELD(ht_mcs12_fbk, 16, 4);
    BIT_FIELD(ht_mcs13_fbk, 20, 4);
    BIT_FIELD(ht_mcs14_fbk, 24, 4);
    BIT_FIELD(ht_mcs15_fbk, 28, 4);
};

class LgFbkCfg0 : public Register<0x135c> {
  public:
    BIT_FIELD(ofdm0_fbk, 0, 4);
    BIT_FIELD(ofdm1_fbk, 4, 4);
    BIT_FIELD(ofdm2_fbk, 8, 4);
    BIT_FIELD(ofdm3_fbk, 12, 4);
    BIT_FIELD(ofdm4_fbk, 16, 4);
    BIT_FIELD(ofdm5_fbk, 20, 4);
    BIT_FIELD(ofdm6_fbk, 24, 4);
    BIT_FIELD(ofdm7_fbk, 28, 4);
};

class LgFbkCfg1 : public Register<0x1360> {
  public:
    BIT_FIELD(cck0_fbk, 0, 4);
    BIT_FIELD(cck1_fbk, 4, 4);
    BIT_FIELD(cck2_fbk, 8, 4);
    BIT_FIELD(cck3_fbk, 12, 4);
};

template<uint16_t A>
class ProtCfg : public Register<A> {
  public:
    BIT_FIELD(prot_rate, 0, 16);
    BIT_FIELD(prot_ctrl, 16, 2);
    BIT_FIELD(prot_nav, 18, 2);
    BIT_FIELD(txop_allow_cck_tx, 20, 1);
    BIT_FIELD(txop_allow_ofdm_tx, 21, 1);
    BIT_FIELD(txop_allow_mm20_tx, 22, 1);
    BIT_FIELD(txop_allow_mm40_tx, 23, 1);
    BIT_FIELD(txop_allow_gf20_tx, 24, 1);
    BIT_FIELD(txop_allow_gf40_tx, 25, 1);
    BIT_FIELD(rtsth_en, 26, 1);
};

class CckProtCfg : public ProtCfg<0x1364> {};
class OfdmProtCfg : public ProtCfg<0x1368> {};
class Mm20ProtCfg : public ProtCfg<0x136c> {};
class Mm40ProtCfg : public ProtCfg<0x1370> {};
class Gf20ProtCfg : public ProtCfg<0x1374> {};
class Gf40ProtCfg : public ProtCfg<0x1378> {};

class ExpAckTime : public Register<0x1380> {
  public:
    BIT_FIELD(exp_cck_ack_time, 0, 15);
    BIT_FIELD(exp_ofdm_ack_time, 16, 15);
};

class RxFiltrCfg : public Register<0x1400> {
  public:
    BIT_FIELD(drop_crc_err, 0, 1);
    BIT_FIELD(drop_phy_err, 1, 1);
    BIT_FIELD(drop_uc_nome, 2, 1);
    BIT_FIELD(drop_not_mybss, 3, 1);
    BIT_FIELD(drop_ver_err, 4, 1);
    BIT_FIELD(drop_mc, 5, 1);
    BIT_FIELD(drop_bc, 6, 1);
    BIT_FIELD(drop_dupl, 7, 1);
    BIT_FIELD(drop_cfack, 8, 1);
    BIT_FIELD(drop_cfend, 9, 1);
    BIT_FIELD(drop_ack, 10, 1);
    BIT_FIELD(drop_cts, 11, 1);
    BIT_FIELD(drop_rts, 12, 1);
    BIT_FIELD(drop_pspoll, 13, 1);
    BIT_FIELD(drop_ba, 14, 1);
    BIT_FIELD(drop_bar, 15, 1);
    BIT_FIELD(drop_ctrl_rsv, 16, 1);
};

class AutoRspCfg : public Register<0x1404> {
  public:
    BIT_FIELD(auto_rsp_en, 0, 1);
    BIT_FIELD(bac_ackpolicy_en, 1, 1);
    BIT_FIELD(cts_40m_mode, 2, 1);
    BIT_FIELD(cts_40m_ref, 3, 1);
    BIT_FIELD(cck_short_en, 4, 1);
    BIT_FIELD(ctrl_wrap_en, 5, 1);
    BIT_FIELD(bac_ack_policy, 6, 1);
    BIT_FIELD(ctrl_pwr_bit, 7, 1);
};

class LegacyBasicRate : public Register<0x1408> {
  public:
    BIT_FIELD(rate_1mbps, 0, 1);
    BIT_FIELD(rate_2mbps, 1, 1);
    BIT_FIELD(rate_5_5mbps, 2, 1);
    BIT_FIELD(rate_11mbps, 3, 1);
    BIT_FIELD(rate_6mbps, 4, 1);
    BIT_FIELD(rate_9mbps, 5, 1);
    BIT_FIELD(rate_12mbps, 6, 1);
    BIT_FIELD(rate_18mbps, 7, 1);
    BIT_FIELD(rate_24mbps, 8, 1);
    BIT_FIELD(rate_36mbps, 9, 1);
    BIT_FIELD(rate_48mbps, 10, 1);
    BIT_FIELD(rate_54mbps, 11, 1);
};

class HtBasicRate : public Register<0x140c> {
    // TODO: figure out what these bits are
};

class TxopHldrEt : public Register<0x1608> {
  public:
    BIT_FIELD(per_rx_rst_en, 0, 1);
    BIT_FIELD(tx40m_blk_en, 1, 1);
    BIT_FIELD(tx_bcn_hipri_dis, 2, 1);
    BIT_FIELD(pape_map1s_en, 3, 1);
    BIT_FIELD(pape_map, 4, 1);
    BIT_FIELD(tx_fbk_thres, 16, 2);
    BIT_FIELD(tx_fbk_thres_en, 18, 1);
    BIT_FIELD(tx_dma_timeout, 19, 5);
    BIT_FIELD(ampdu_acc_en, 24, 1);
};

class RxStaCnt0 : public Register<0x1700> {
  public:
    BIT_FIELD(crc_errcnt, 0, 16);
    BIT_FIELD(phy_errcnt, 16, 16);
};

class RxStaCnt1 : public Register<0x1704> {
  public:
    BIT_FIELD(cca_errcnt, 0, 16);
    BIT_FIELD(plpc_errcnt, 16, 16);
};

class RxStaCnt2 : public Register<0x1708> {
  public:
    BIT_FIELD(rx_dupl_cnt, 0, 16);
    BIT_FIELD(rx_ovfl_cnt, 16, 16);
};

class TxStaCnt0 : public Register<0x170c> {
  public:
    BIT_FIELD(tx_fail_cnt, 0, 16);
    BIT_FIELD(tx_bcn_cnt, 16, 16);
};

class TxStaCnt1 : public Register<0x1710> {
  public:
    BIT_FIELD(tx_succ_cnt, 0, 16);
    BIT_FIELD(tx_rty_cnt, 16, 16);
};

class TxStaCnt2 : public Register<0x1714> {
  public:
    BIT_FIELD(tx_zero_cnt, 0, 16);
    BIT_FIELD(tx_udfl_cnt, 16, 16);
};

// EEPROM offsets
constexpr uint16_t EEPROM_CHIP_ID = 0x0000;
constexpr uint16_t EEPROM_VERSION = 0x0001;
constexpr uint16_t EEPROM_MAC_ADDR_0 = 0x0002;
constexpr uint16_t EEPROM_MAC_ADDR_1 = 0x0003;
constexpr uint16_t EEPROM_MAC_ADDR_2 = 0x0004;
constexpr uint16_t EEPROM_NIC_CONF2 = 0x0021;
constexpr uint16_t EEPROM_RSSI_A = 0x0025;
constexpr uint16_t EEPROM_RSSI_A2 = 0x0026;
constexpr uint16_t EEPROM_BBP_START = 0x0078;

constexpr size_t EEPROM_BBP_SIZE = 16;

class EepromNicConf0 : public EepromField<0x001a> {
  public:
    BIT_FIELD(rxpath, 0, 4);
    BIT_FIELD(txpath, 4, 4);
    BIT_FIELD(rf_type, 8, 4);
};

class EepromNicConf1 : public EepromField<0x001b> {
  public:
    BIT_FIELD(hw_radio, 0, 1);
    BIT_FIELD(external_tx_alc, 1, 1);
    BIT_FIELD(external_lna_2g, 2, 1);
    BIT_FIELD(external_lna_5g, 3, 1);
    BIT_FIELD(cardbus_accel, 4, 1);
    BIT_FIELD(bw40m_sb_2g, 5, 1);
    BIT_FIELD(bw40m_sb_5g, 6, 1);
    BIT_FIELD(wps_pbc, 7, 1);
    BIT_FIELD(bw40m_2g, 8, 1);
    BIT_FIELD(bw40m_5g, 9, 1);
    BIT_FIELD(broadband_ext_lna, 10, 1);
    BIT_FIELD(ant_diversity, 11, 2);
    BIT_FIELD(internal_tx_alc, 13, 1);
    BIT_FIELD(bt_coexist, 14, 1);
    BIT_FIELD(dac_test, 15, 1);
};

class EepromFreq : public EepromField<0x001d> {
  public:
    BIT_FIELD(offset, 0, 8);
};

class EepromLna : public EepromField<0x0022> {
  public:
    BIT_FIELD(bg, 0, 8);
    BIT_FIELD(a0, 8, 8);
};

class EepromRssiBg : public EepromField<0x0023> {
  public:
    BIT_FIELD(offset0, 0, 8);
    BIT_FIELD(offset1, 8, 8);
};

class EepromRssiBg2 : public EepromField<0x0024> {
  public:
    BIT_FIELD(offset2, 0, 8);
    BIT_FIELD(lna_a1, 8, 8);
};

// Host to MCU communication

class H2mMailboxCsr : public Register<0x7010> {
  public:
    BIT_FIELD(arg0, 0, 8);
    BIT_FIELD(arg1, 8, 8);
    BIT_FIELD(cmd_token, 16, 8);
    BIT_FIELD(owner, 24, 8);
};

class H2mMailboxCid : public Register<0x7014> {
  public:
    BIT_FIELD(cmd0, 0, 8);
    BIT_FIELD(cmd1, 8, 8);
    BIT_FIELD(cmd2, 16, 8);
    BIT_FIELD(cmd3, 24, 8);
};

class H2mMailboxStatus : public Register<0x701c> {};
class H2mBbpAgent : public Register<0x7028> {};
class H2mIntSrc : public Register<0x7024> {};

// MCU commands
constexpr uint8_t MCU_BOOT_SIGNAL = 0x72;
constexpr uint8_t MCU_WAKEUP = 0x31;

// BBP registers

class Bbp1 : public BbpRegister<1> {
  public:
    BIT_FIELD(tx_power_ctrl, 0, 2);
    BIT_FIELD(tx_antenna, 3, 2);
};

class Bbp3 : public BbpRegister<3> {
  public:
    BIT_FIELD(rx_adc, 0, 2);
    BIT_FIELD(rx_antenna, 3, 2);
    BIT_FIELD(ht40_minus, 5, 1);
    BIT_FIELD(adc_mode_switch, 6, 1);
    BIT_FIELD(adc_init_mode, 7, 1);
};

class Bbp4 : public BbpRegister<4> {
  public:
    BIT_FIELD(tx_bf, 0, 1);
    BIT_FIELD(bandwidth, 3, 2);
    BIT_FIELD(mac_if_ctrl, 6, 1);
};

class Bbp138 : public BbpRegister<138> {
  public:
    BIT_FIELD(rx_adc1, 1, 1);
    BIT_FIELD(rx_adc2, 2, 1);
    BIT_FIELD(tx_dac1, 5, 1);
    BIT_FIELD(tx_dac2, 6, 1);
};

class Bbp152 : public BbpRegister<152> {
  public:
    BIT_FIELD(rx_default_ant, 7, 1);
};

// RFCSR registers

class Rfcsr2 : public RfcsrRegister<2> {
  public:
    BIT_FIELD(rescal_en, 7, 1);
};

class Rfcsr30 : public RfcsrRegister<30> {
  public:
    BIT_FIELD(tx_h20m, 1, 1);
    BIT_FIELD(rx_h20m, 2, 1);
    BIT_FIELD(rx_vcm, 3, 2);
    BIT_FIELD(rf_calibration, 7, 1);
};

class Rfcsr38 : public RfcsrRegister<38> {
  public:
    BIT_FIELD(rx_lo1_en, 5, 1);
};

class Rfcsr39 : public RfcsrRegister<39> {
  public:
    BIT_FIELD(rx_div, 6, 1);
    BIT_FIELD(rx_lo2_en, 7, 1);
};

}  // namespace rt5370
