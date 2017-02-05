// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "device.h"
#include "rt5370.h"

#include <ddk/common/usb.h>
#include <magenta/hw/usb.h>
#include <mx/vmo.h>
#include <mx/time.h>

#include <endian.h>

#include <algorithm>
#include <cstdio>
#include <future>

#define CHECK_REG(reg, op, status) \
    do { \
        if (status < 0) { \
            std::printf("rt5370 #op##Register error for #reg: %d\n", status); \
        } \
    } while (0)
#define CHECK_READ(reg, status) CHECK_REG(reg, Read, status)
#define CHECK_WRITE(reg, status) CHECK_REG(reg, Write, status)

using namespace std::chrono_literals;

namespace {

template <class Rep, class Period>
mx_status_t mxsleep(std::chrono::duration<Rep, Period> duration) {
    return mx::nanosleep(std::chrono::nanoseconds(duration).count());
}

constexpr size_t kReadReqCount = 8;
constexpr size_t kReadBufSize = 4096;
constexpr size_t kWriteReqCount = 4;
constexpr size_t kWriteBufSize = 4096;  // todo: use endpt max size

constexpr char kFirmwareFile[] = "rt2870.bin";

constexpr int kMaxBusyReads = 20;

constexpr uint32_t get_bits(uint8_t offset, uint8_t len, uint32_t bitmask) {
    auto mask = ((1 << len) - 1) << offset;
    return (bitmask & mask) >> offset;
}

constexpr uint32_t get_bit(uint8_t offset, uint32_t bitmask) {
    return get_bits(offset, 1, bitmask);
}

constexpr uint32_t set_bits(uint8_t offset, uint8_t len, uint32_t orig, uint32_t value) {
    auto mask = ((1 << len) - 1) << offset;
    uint32_t ret = orig & ~mask;
    return ret | ((value << offset) & mask);
}

//constexpr uint32_t set_bit(uint8_t offset, uint32_t orig) {
//    return set_bits(offset, 1, orig, 1);
//}
//
//constexpr uint32_t clear_bit(uint8_t offset, uint32_t orig) {
//    return set_bits(offset, 1, orig, 0);
//}

// The <cstdlib> overloads confuse the compiler for <cstdint> types.
template <typename T>
constexpr T abs(T t) {
    return t < 0 ? -t : t;
}

}  // namespace

namespace rt5370 {

Device::Device(mx_driver_t* driver, mx_device_t* device, uint8_t bulk_in,
               std::vector<uint8_t>&& bulk_out)
  : driver_(driver),
    usb_device_(device),
    rx_endpt_(bulk_in),
    tx_endpts_(std::move(bulk_out)) {
    std::printf("rt5370::Device drv=%p dev=%p bulk_in=%u\n", driver_, usb_device_, rx_endpt_);
}

Device::~Device() {
    std::printf("rt5370::Device::~Device\n");
    for (auto txn : free_write_reqs_) {
        txn->ops->release(txn);
    }
    for (auto txn : completed_reads_) {
        txn->ops->release(txn);
    }
}

mx_status_t Device::Bind() {
    std::printf("rt5370::Device::Bind\n");

    AsicVerId avi;
    auto status = ReadRegister(&avi);
    CHECK_READ(MAC_CSR0, status);

    rt_type_ = avi.ver_id();
    rt_rev_ = avi.rev_id();
    std::printf("rt5370 RT chipset %#x, rev %#x\n", rt_type_, rt_rev_);

    bool autorun = false;
    status = DetectAutoRun(&autorun);
    if (status != NO_ERROR) {
        return status;
    }

    EfuseCtrl ec;
    status = ReadRegister(&ec);
    CHECK_READ(EFUSE_CTRL, status);

    std::printf("rt5370 efuse ctrl reg: %#x\n", ec.val());
    bool efuse_present = ec.sel_efuse() > 0;
    std::printf("rt5370 efuse present: %s\n", efuse_present ? "Y" : "N");

    status = ReadEeprom();
    if (status != NO_ERROR) {
        std::printf("rt5370 failed to read eeprom\n");
        return status;
    }

    status = ValidateEeprom();
    if (status != NO_ERROR) {
        std::printf("rt5370 failed to validate eeprom\n");
        return status;
    }

    if (rt_type_ == RT5390) {
        rf_type_ = eeprom_[EEPROM_CHIP_ID];
        std::printf("rt5370 RF chipset %#x\n", rf_type_);
    } else {
        // TODO: support other RF chipsets
        return ERR_NOT_SUPPORTED;
    }

    // TODO: default antenna configs

    auto conf1 = eeprom_[EEPROM_NIC_CONF1];
    std::printf("rt5370 NIC CONF1=%#x\n", conf1);
    std::printf("rt5370 has HW radio? %s\n", get_bit(EEPROM_NIC_CONF1_HW_RADIO, conf1) ? "Y" : "N");
    std::printf("rt5370 has BT coexist? %s\n", get_bit(EEPROM_NIC_CONF1_BT_COEXIST, conf1) ? "Y" : "N");

    auto freq = eeprom_[EEPROM_FREQ];
    std::printf("rt5370 freq offset=%#x\n", get_bits(EEPROM_FREQ_OFFSET, EEPROM_FREQ_WIDTH, freq));

    // TODO: rfkill switch GPIO

    // TODO: allocate and initialize some queues

    status = LoadFirmware();
    if (status != NO_ERROR) {
        std::printf("rt5370 failed to load firmware\n");
        return status;
    }
    std::printf("rt5370 firmware loaded\n");

    // Initialize queues
    for (size_t i = 0; i < kReadReqCount; i++) {
        auto* req = usb_alloc_iotxn(rx_endpt_, kReadBufSize, 0);
        if (req == nullptr) {
            std::printf("rt5370 failed to allocate rx iotxn\n");
            return ERR_NO_MEMORY;
        }
        req->length = kReadBufSize;
        req->complete_cb = &Device::ReadIotxnComplete;
        req->cookie = this;
        iotxn_queue(usb_device_, req);
    }
    auto tx_endpt = tx_endpts_.front();
    for (size_t i = 0; i < kWriteReqCount; i++) {
        auto* req = usb_alloc_iotxn(tx_endpt, kWriteBufSize, 0);
        if (req == nullptr) {
            std::printf("rt5370 failed to allocate tx iotxn\n");
            return ERR_NO_MEMORY;
        }
        req->length = kWriteBufSize;
        req->complete_cb = &Device::WriteIotxnComplete;
        req->cookie = this;
        free_write_reqs_.push_back(req);
    }

    status = EnableRadio();
    if (status != NO_ERROR) {
        std::printf("rt5370 could not enable radio\n");
        return status;
    }

    device_ops_.unbind = &Device::DdkUnbind;
    device_ops_.release = &Device::DdkRelease;
    device_ops_.read = &Device::DdkRead;
    device_ops_.write = &Device::DdkWrite;
    device_ops_.ioctl = &Device::DdkIoctl;
    device_init(&device_, driver_, "rt5370", &device_ops_);

    device_.ctx = this;
    status = device_add(&device_, usb_device_);
    if (status != NO_ERROR) {
        std::printf("rt5370 could not add device err=%d\n", status);
    }

    return status;
}

mx_status_t Device::ReadRegister(uint16_t offset, uint32_t* value) {
    return usb_control(usb_device_, (USB_DIR_IN | USB_TYPE_VENDOR), kMultiRead, 0,
            offset, value, sizeof(*value));
}

template <uint16_t A> mx_status_t Device::ReadRegister(Register<A>* reg) {
    return ReadRegister(A, reg->mut_val());
}

mx_status_t Device::WriteRegister(uint16_t offset, uint32_t value) {
    return usb_control(usb_device_, (USB_DIR_OUT | USB_TYPE_VENDOR), kMultiWrite, 0,
            offset, &value, sizeof(value));
}

template <uint16_t A> mx_status_t Device::WriteRegister(const Register<A>& reg) {
    return WriteRegister(A, reg.val());
}

mx_status_t Device::ReadEeprom() {
    // Read 4 entries at a time
    for (unsigned int i = 0; i < eeprom_.size(); i += 8) {
        EfuseCtrl ec;
        auto status = ReadRegister(&ec);
        CHECK_READ(EFUSE_CTRL, status);

        // Set the address and tell it to load the next four words. Addresses
        // must be 16-byte aligned.
        ec.set_efsrom_ain(i << 1);
        ec.set_efsrom_mode(0);
        ec.set_efsrom_kick(1);
        status = WriteRegister(ec);
        CHECK_WRITE(EFUSE_CTRL, status);

        // Wait until the registers are ready for reading.
        BusyPredicate<EfuseCtrl> pred = [](EfuseCtrl* ec) { return !ec->efsrom_kick(); };
        status = BusyWait(&ec, pred);
        if (status < 0) {
            if (status == ERR_TIMED_OUT) {
                std::printf("rt5370 busy wait for EFUSE_CTRL failed\n");
            }
            return status;
        }

        // Read the registers into the eeprom. EEPROM is read in descending
        // order, and are always return in host order but to be interpreted as
        // little endian.
        RfuseData0 rd0;
        status = ReadRegister(&rd0);
        CHECK_READ(EFUSE_DATA0, status);
        eeprom_[i] = htole32(rd0.val()) & 0xffff;
        eeprom_[i+1] = htole32(rd0.val()) >> 16;

        RfuseData1 rd1;
        status = ReadRegister(&rd1);
        CHECK_READ(EFUSE_DATA1, status);
        eeprom_[i+2] = htole32(rd1.val()) & 0xffff;
        eeprom_[i+3] = htole32(rd1.val()) >> 16;

        RfuseData2 rd2;
        status = ReadRegister(&rd2);
        CHECK_READ(EFUSE_DATA2, status);
        eeprom_[i+4] = htole32(rd2.val()) & 0xffff;
        eeprom_[i+5] = htole32(rd2.val()) >> 16;

        RfuseData3 rd3;
        status = ReadRegister(&rd3);
        CHECK_READ(EFUSE_DATA3, status);
        eeprom_[i+6] = htole32(rd3.val()) & 0xffff;
        eeprom_[i+7] = htole32(rd3.val()) >> 16;
    }

#if 0
    for (size_t i = 0; i < eeprom_.size(); i++) {
        std::printf("%04x ", eeprom_[i]);
        if (i % 8 == 7) std::printf("\n");
    }
#endif

    return NO_ERROR;
}

mx_status_t Device::ValidateEeprom() {
    auto mac_addr = reinterpret_cast<uint8_t*>(eeprom_.data() + EEPROM_MAC_ADDR_0);
    // TODO: validate mac address
    std::printf("rt5370 MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    auto data = eeprom_[EEPROM_NIC_CONF0];
    if (data == 0xffff || data == 0x2860 || data == 0x2872) {
        // These values need some eeprom patching; not supported yet.
        std::printf("unsupported value for EEPROM_NIC_CONF0=%#x\n", data);
        return ERR_NOT_SUPPORTED;
    }

    data = eeprom_[EEPROM_NIC_CONF1];
    if (data == 0xffff) {
        std::printf("unsupported value for EEPROM_NIC_CONF1=%#x\n", data);
        return ERR_NOT_SUPPORTED;
    }

    data = eeprom_[EEPROM_FREQ];
    if ((data & 0x00ff) == 0x00ff) {
        data = set_bits(EEPROM_FREQ_OFFSET, EEPROM_FREQ_WIDTH, data, 0);
        eeprom_[EEPROM_FREQ] = data;
        std::printf("rt5370 Freq: %#x\n", data);
    }
    // TODO: check/set LED mode

    auto default_lna_gain = get_bits(EEPROM_LNA_A0_OFFSET, EEPROM_LNA_A0_WIDTH, eeprom_[EEPROM_LNA]);

    data = eeprom_[EEPROM_RSSI_BG];
    if (abs(get_bits(EEPROM_RSSI_BG_0_OFFSET, EEPROM_RSSI_BG_0_WIDTH, data)) > 10) {
        data = set_bits(EEPROM_RSSI_BG_0_OFFSET, EEPROM_RSSI_BG_0_WIDTH, data, 0);
    }
    if (abs(get_bits(EEPROM_RSSI_BG_1_OFFSET, EEPROM_RSSI_BG_1_WIDTH, data)) > 10) {
        data = set_bits(EEPROM_RSSI_BG_1_OFFSET, EEPROM_RSSI_BG_1_WIDTH, data, 0);
    }
    eeprom_[EEPROM_RSSI_BG] = data;

    data = eeprom_[EEPROM_RSSI_BG2];
    if (abs(get_bits(EEPROM_RSSI_BG2_OFFSET, EEPROM_RSSI_BG2_WIDTH, data)) > 10) {
        data = set_bits(EEPROM_RSSI_BG2_OFFSET, EEPROM_RSSI_BG2_WIDTH, data, 0);
    }
    auto bg2_lna_a1 = get_bits(EEPROM_RSSI_BG2_LNA_A1_OFFSET, EEPROM_RSSI_BG2_LNA_A1_WIDTH, data);
    if (bg2_lna_a1 == 0x00 || bg2_lna_a1 == 0xff) {
        data = set_bits(EEPROM_RSSI_BG2_LNA_A1_OFFSET, EEPROM_RSSI_BG2_LNA_A1_WIDTH, data, default_lna_gain);
    }
    eeprom_[EEPROM_RSSI_BG2] = data;

    // TODO: check and set RSSI for A

    return NO_ERROR;
}

mx_status_t Device::LoadFirmware() {
    mx_handle_t fw_handle;
    size_t fw_size = 0;
    auto status = load_firmware(driver_, kFirmwareFile, &fw_handle, &fw_size);
    if (status != NO_ERROR) {
        std::printf("rt5370 failed to load firmware '%s': err=%d\n", kFirmwareFile, status);
        return status;
    }
    std::printf("rt5370 opened firmware '%s' (%zd bytes)\n", kFirmwareFile, fw_size);

    mx::vmo fw(fw_handle);
    uint8_t fwversion[2];
    size_t actual = 0;
    status = fw.read(fwversion, fw_size - 4, 2, &actual);
    if (status != NO_ERROR || actual != sizeof(fwversion)) {
        std::printf("rt5370 error reading fw version\n");
        return ERR_BAD_STATE;
    }
    std::printf("rt5370 FW version %u.%u\n", fwversion[0], fwversion[1]);
    // Linux rt2x00 driver has more intricate size checking for different
    // chipsets. We just care that it's 8kB for the rt5370.
    if (fw_size != 8192) {
        std::printf("rt5370 FW: bad length (%zu)\n", fw_size);
        return ERR_BAD_STATE;
    }

    // TODO: check crc, 4kB at a time

    AutoWakeupCfg awc;
    std::printf("rt5370 writing auto wakeup\n");
    status = WriteRegister(awc);
    CHECK_WRITE(AUTO_WAKEUP_CFG, status);
    std::printf("rt5370 auto wakeup written\n");

    // Wait for hardware to stabilize
    status = WaitForMacCsr();
    if (status != NO_ERROR) {
        std::printf("rt5370 unstable hardware\n");
        return status;
    }
    std::printf("rt5370 hardware stabilized\n");

    status = DisableWpdma();
    if (status != NO_ERROR) return status;

    bool autorun = false;
    status = DetectAutoRun(&autorun);
    if (status != NO_ERROR) {
        return status;
    }
    if (autorun) {
        std::printf("rt5370 not loading firmware, NIC is in autorun mode\n");
        return NO_ERROR;
    }
    std::printf("rt5370 autorun not enabled\n");

    // Send the firmware to the chip
    // For rt5370, start at offset 4096 and send 4096 bytes
    size_t offset = 4096;
    size_t remaining = fw_size - offset;
    uint8_t buf[64];
    uint16_t addr = FW_IMAGE_BASE;

    while (remaining) {
        size_t to_send = std::min(remaining, sizeof(buf));
        status = fw.read(buf, offset, to_send, &actual);
        if (status != NO_ERROR || actual != to_send) {
            std::printf("rt5370 error reading firmware\n");
            return ERR_BAD_STATE;
        }
        status = usb_control(usb_device_, (USB_DIR_OUT | USB_TYPE_VENDOR), kMultiWrite,
                             0, addr, buf, to_send);
        if (status < (ssize_t)to_send) {
            std::printf("rt5370 failed to send firmware\n");
            return ERR_BAD_STATE;
        }
        remaining -= to_send;
        offset += to_send;
        addr += to_send;
    }
    std::printf("rt5370 sent firmware\n");

    H2mMailboxCid hmc;
    hmc.set_val(~0);
    status = WriteRegister(hmc);
    CHECK_WRITE(H2M_MAILBOX_CID, status);

    H2mMailboxStatus hms;
    hms.set_val(~0);
    status = WriteRegister(hms);
    CHECK_WRITE(H2M_MAILBOX_STATUS, status);

    // Tell the device to load the firmware
    status = usb_control(usb_device_, (USB_DIR_OUT | USB_TYPE_VENDOR), kDeviceMode,
                         kFirmware, 0, NULL, 0);
    if (status != NO_ERROR) {
        std::printf("rt5370 failed to send load firmware command\n");
        return status;
    }
    mxsleep(10ms);

    SysCtrl sc;
    BusyPredicate<SysCtrl> pred = [](SysCtrl* sc) { return sc->mcu_ready(); };
    status = BusyWait(&sc, pred, 1ms);
    if (status < 0) {
        if (status == ERR_TIMED_OUT) {
            std::printf("rt5370 system MCU not ready\n");
        }
        return status;
    }

    // Disable WPDMA again
    status = DisableWpdma();
    if (status != NO_ERROR) return status;

    // Initialize firmware and boot the MCU
    H2mBbpAgent hba;
    status = WriteRegister(hba);
    CHECK_WRITE(H2M_BBP_AGENT, status);

    H2mMailboxCsr hmcsr;
    status = WriteRegister(hmcsr);
    CHECK_WRITE(H2M_MAILBOX_CSR, status);

    H2mIntSrc his;
    status = WriteRegister(his);
    CHECK_WRITE(H2M_INT_SRC, status);

    status = McuCommand(MCU_BOOT_SIGNAL, 0, 0, 0);
    if (status < 0) {
        std::printf("rt5370 error booting MCU err=%d\n", status);
        return status;
    }
    mxsleep(1ms);

    return NO_ERROR;
}

mx_status_t Device::EnableRadio() {
    std::printf("rt5370 %s\n", __func__);

    // Wakeup the MCU
    auto status = McuCommand(MCU_WAKEUP, 0xff, 0, 2);
    if (status < 0) {
        std::printf("rt5370 error waking MCU err=%d\n", status);
        return status;
    }
    mxsleep(1ms);

    // Wait for WPDMA to be ready
    WpdmaGloCfg wgc;
    BusyPredicate<WpdmaGloCfg> wpdma_pred =
        [](WpdmaGloCfg* wgc) { return !wgc->tx_dma_busy() && !wgc->rx_dma_busy(); };
    status = BusyWait(&wgc, wpdma_pred, 10ms);
    if (status < 0) {
        if (status == ERR_TIMED_OUT) {
            std::printf("rt5370 WPDMA busy\n");
        }
        return status;
    }

    // Set up USB DMA
    UsbDmaCfg udc;
    status = ReadRegister(&udc);
    CHECK_READ(USB_DMA_CFG, status);
    udc.set_phy_wd_en(0);
    udc.set_rx_agg_en(0);
    udc.set_rx_agg_to(128);
    // There appears to be a bug in the Linux driver, where an overflow is
    // setting the rx aggregation limit too low. For now, I'm using the
    // (incorrect) low value that Linux uses, but we should look into increasing
    // this.
    udc.set_rx_agg_limit(45);
    udc.set_udma_rx_en(1);
    udc.set_udma_tx_en(1);
    status = WriteRegister(udc);
    CHECK_WRITE(USB_DMA_CFG, status);

    // Wait for WPDMA again
    status = BusyWait(&wgc, wpdma_pred, 10ms);
    if (status < 0) {
        if (status == ERR_TIMED_OUT) {
            std::printf("rt5370 WPDMA busy\n");
        }
        return status;
    }

    status = InitRegisters();
    if (status < 0) {
        std::printf("rt5370 failed to initialize registers\n");
        return status;
    }

    // Wait for MAC status ready
    MacStatusReg msr;
    BusyPredicate<MacStatusReg> msr_pred =
        [](MacStatusReg* msr) { return !msr->tx_status() && !msr->rx_status(); };
    status = BusyWait(&msr, msr_pred, 10ms);
    if (status < 0) {
        if (status == ERR_TIMED_OUT) {
            std::printf("rt5370 BBP busy\n");
        }
        return status;
    }

    // Initialize firmware
    H2mBbpAgent hba;
    status = WriteRegister(hba);
    CHECK_WRITE(H2M_BBP_AGENT, status);

    H2mMailboxCsr hmc;
    status = WriteRegister(hmc);
    CHECK_WRITE(H2M_MAILBOX_CSR, status);

    H2mIntSrc his;
    status = WriteRegister(his);
    CHECK_WRITE(H2M_INT_SRC, status);

    status = McuCommand(MCU_BOOT_SIGNAL, 0, 0, 0);
    if (status < 0) {
        std::printf("rt5370 error booting MCU err=%d\n", status);
        return status;
    }
    mxsleep(1ms);
    return NO_ERROR;
}

mx_status_t Device::InitRegisters() {
    std::printf("rt5370 %s\n", __func__);

    auto status = DisableWpdma();
    if (status != NO_ERROR) return status;

    status = WaitForMacCsr();
    if (status != NO_ERROR) {
        std::printf("rt5370 hardware unstable\n");
        return status;
    }

    {
        SysCtrl sc;
        status = ReadRegister(&sc);
        CHECK_READ(SYS_CTRL, status);
        sc.set_pme_oen(0);
        status = WriteRegister(sc);
        CHECK_WRITE(SYS_CTRL, status);
    }

    {
        MacSysCtrl msc;
        msc.set_mac_srst(1);
        msc.set_bbp_hrst(1);
        status = WriteRegister(msc);
        CHECK_WRITE(MAC_SYS_CTRL, status);
    }

    {
        UsbDmaCfg udc;
        status = WriteRegister(udc);
        CHECK_WRITE(USB_DMA_CFG, status);
    }

    status = usb_control(usb_device_, (USB_DIR_OUT | USB_TYPE_VENDOR), kDeviceMode,
            kReset, 0, NULL, 0);
    if (status != NO_ERROR) {
        std::printf("rt5370 failed reset\n");
        return status;
    }

    {
        MacSysCtrl msc;
        status = WriteRegister(msc);
        CHECK_WRITE(MAC_SYS_CTRL, status);
    }

    {
        LegacyBasicRate lbr;
        lbr.set_rate_1mbps(1);
        lbr.set_rate_2mbps(1);
        lbr.set_rate_5_5mbps(1);
        lbr.set_rate_11mbps(1);
        lbr.set_rate_6mbps(1);
        lbr.set_rate_9mbps(1);
        lbr.set_rate_24mbps(1);
        status = WriteRegister(lbr);
        CHECK_WRITE(LEGACY_BASIC_RATE, status);
    }

    {
        // Magic number from Linux kernel driver
        HtBasicRate hbr;
        hbr.set_val(0x8003);
        status = WriteRegister(hbr);
        CHECK_WRITE(HT_BASIC_RATE, status);
    }

    {
        MacSysCtrl msc;
        status = WriteRegister(msc);
        CHECK_WRITE(MAC_SYS_CTRL, status);
    }

    {
        BcnTimeCfg btc;
        status = ReadRegister(&btc);
        CHECK_READ(BCN_TIME_CFG, status);
        btc.set_bcn_intval(1600);
        btc.set_tsf_timer_en(0);
        btc.set_tsf_sync_mode(0);
        btc.set_tbtt_timer_en(0);
        btc.set_bcn_tx_en(0);
        btc.set_tsf_ins_comp(0);
        status = WriteRegister(btc);
        CHECK_WRITE(BCN_TIME_CFG, status);
    }

    {
        RxFiltrCfg rfc;
        status = ReadRegister(&rfc);
        CHECK_READ(RX_FILTR_CFG, status);
        rfc.set_drop_crc_err(1);
        rfc.set_drop_phy_err(1);
        rfc.set_drop_uc_nome(1);
        rfc.set_drop_ver_err(1);
        rfc.set_drop_dupl(1);
        rfc.set_drop_cfack(1);
        rfc.set_drop_cfend(1);
        rfc.set_drop_ack(1);
        rfc.set_drop_cts(1);
        rfc.set_drop_rts(1);
        rfc.set_drop_pspoll(1);
        rfc.set_drop_bar(1);
        rfc.set_drop_ctrl_rsv(1);
        status = WriteRegister(rfc);
        CHECK_WRITE(RX_FILTR_CFG, status);
    }

    {
        BkoffSlotCfg bsc;
        status = ReadRegister(&bsc);
        CHECK_READ(BKOFF_SLOT_CFG, status);
        bsc.set_slot_time(9);
        bsc.set_cc_delay_time(2);
        status = WriteRegister(bsc);
        CHECK_WRITE(BKOFF_SLOT_CFG, status);
    }

    {
        TxSwCfg0 tsc0;
        // TX_SW_CFG register values come from Linux kernel driver
        tsc0.set_dly_txpe_en(0x04);
        tsc0.set_dly_pape_en(0x04);
        // All other TX_SW_CFG0 values are 0 (set by using 0 as starting value)
        status = WriteRegister(tsc0);
        CHECK_WRITE(TX_SW_CFG0, status);
    }

    {
        TxSwCfg1 tsc1;
        tsc1.set_dly_pape_dis(0x06);
        tsc1.set_dly_trsw_dis(0x06);
        tsc1.set_dly_rftr_dis(0x08);
        status = WriteRegister(tsc1);
        CHECK_WRITE(TX_SW_CFG1, status);
    }

    {
        TxSwCfg2 tsc2;
        // All bits set to zero.
        status = WriteRegister(tsc2);
        CHECK_WRITE(TX_SW_CFG2, status);
    }

    {
        TxLinkCfg tlc;
        status = ReadRegister(&tlc);
        CHECK_READ(TX_LINK_CFG, status);
        tlc.set_remote_mfb_lifetime(32);
        tlc.set_tx_mfb_en(0);
        tlc.set_remote_umfs_en(0);
        tlc.set_tx_mrq_en(0);
        tlc.set_tx_rdg_en(0);
        tlc.set_tx_cfack_en(1);
        tlc.set_remote_mfb(0);
        tlc.set_remote_mfs(0);
        status = WriteRegister(tlc);
        CHECK_WRITE(TX_LINK_CFG, status);
    }

    {
        TxTimeoutCfg ttc;
        status = ReadRegister(&ttc);
        CHECK_READ(TX_TIMEOUT_CFG, status);
        ttc.set_mpdu_life_time(9);
        ttc.set_rx_ack_timeout(32);
        ttc.set_txop_timeout(10);
        status = WriteRegister(ttc);
        CHECK_WRITE(TX_TIMEOUT_CFG, status);
    }

    {
        MaxLenCfg mlc;
        status = ReadRegister(&mlc);
        CHECK_READ(MAX_LEN_CFG, status);
        mlc.set_max_mpdu_len(3840);
        mlc.set_max_psdu_len(1);
        mlc.set_min_psdu_len(0);
        mlc.set_min_mpdu_len(0);
        status = WriteRegister(mlc);
        CHECK_WRITE(MAX_LEN_CFG, status);
    }

    // TODO: LED_CFG

    {
        MaxPcnt mp;
        mp.set_max_rx0q_pcnt(0x9f);
        mp.set_max_tx2q_pcnt(0xbf);
        mp.set_max_tx1q_pcnt(0x3f);
        mp.set_max_tx0q_pcnt(0x1f);
        status = WriteRegister(mp);
        CHECK_WRITE(MAX_PCNT, status);
    }

    {
        TxRtyCfg trc;
        status = ReadRegister(&trc);
        CHECK_READ(TX_RTY_CFG, status);
        trc.set_short_rty_limit(15);
        trc.set_long_rty_limit(31);
        trc.set_long_rty_thres(2000);
        trc.set_nag_rty_mode(0);
        trc.set_agg_rty_mode(0);
        trc.set_tx_autofb_en(1);
        status = WriteRegister(trc);
        CHECK_WRITE(TX_RTY_CFG, status);
    }

    {
        AutoRspCfg arc;
        status = ReadRegister(&arc);
        CHECK_READ(AUTO_RSP_CFG, status);
        arc.set_auto_rsp_en(1);
        arc.set_bac_ackpolicy_en(1);
        arc.set_cts_40m_mode(0);
        arc.set_cts_40m_ref(0);
        arc.set_cck_short_en(1);
        arc.set_bac_ack_policy(0);
        arc.set_ctrl_pwr_bit(0);
        status = WriteRegister(arc);
        CHECK_WRITE(AUTO_RSP_CFG, status);
    }

    {
        CckProtCfg cpc;
        status = ReadRegister(&cpc);
        CHECK_READ(CCK_PROT_CFG, status);
        cpc.set_prot_rate(3);
        cpc.set_prot_ctrl(0);
        cpc.set_prot_nav(1);
        cpc.set_txop_allow_cck_tx(1);
        cpc.set_txop_allow_ofdm_tx(1);
        cpc.set_txop_allow_mm20_tx(1);
        cpc.set_txop_allow_mm40_tx(0);
        cpc.set_txop_allow_gf20_tx(1);
        cpc.set_txop_allow_gf40_tx(0);
        cpc.set_rtsth_en(1);
        status = WriteRegister(cpc);
        CHECK_WRITE(CCK_PROT_CFG, status);
    }

    {
        OfdmProtCfg opc;
        status = ReadRegister(&opc);
        CHECK_READ(OFDM_PROT_CFG, status);
        opc.set_prot_rate(3);
        opc.set_prot_ctrl(0);
        opc.set_prot_nav(1);
        opc.set_txop_allow_cck_tx(1);
        opc.set_txop_allow_ofdm_tx(1);
        opc.set_txop_allow_mm20_tx(1);
        opc.set_txop_allow_mm40_tx(0);
        opc.set_txop_allow_gf20_tx(1);
        opc.set_txop_allow_gf40_tx(0);
        opc.set_rtsth_en(1);
        status = WriteRegister(opc);
        CHECK_WRITE(OFDM_PROT_CFG, status);
    }

    {
        Mm20ProtCfg mm20pc;
        status = ReadRegister(&mm20pc);
        CHECK_READ(MM20_PROT_CFG, status);
        mm20pc.set_prot_rate(0x4004);
        mm20pc.set_prot_ctrl(0);
        mm20pc.set_prot_nav(1);
        mm20pc.set_txop_allow_cck_tx(1);
        mm20pc.set_txop_allow_ofdm_tx(1);
        mm20pc.set_txop_allow_mm20_tx(1);
        mm20pc.set_txop_allow_mm40_tx(0);
        mm20pc.set_txop_allow_gf20_tx(1);
        mm20pc.set_txop_allow_gf40_tx(0);
        mm20pc.set_rtsth_en(0);
        status = WriteRegister(mm20pc);
        CHECK_WRITE(MM20_PROT_CFG, status);
    }

    {
        Mm40ProtCfg mm40pc;
        status = ReadRegister(&mm40pc);
        CHECK_READ(MM40_PROT_CFG, status);
        mm40pc.set_prot_rate(0x4084);
        mm40pc.set_prot_ctrl(1);
        mm40pc.set_prot_nav(1);
        mm40pc.set_txop_allow_cck_tx(0);
        mm40pc.set_txop_allow_ofdm_tx(1);
        mm40pc.set_txop_allow_mm20_tx(1);
        mm40pc.set_txop_allow_mm40_tx(0);
        mm40pc.set_txop_allow_gf20_tx(1);
        mm40pc.set_txop_allow_gf40_tx(0);
        mm40pc.set_rtsth_en(0);
        status = WriteRegister(mm40pc);
        CHECK_WRITE(MM40_PROT_CFG, status);
    }

    {
        Gf20ProtCfg gf20pc;
        status = ReadRegister(&gf20pc);
        CHECK_READ(GF20_PROT_CFG, status);
        gf20pc.set_prot_rate(0x4004);
        gf20pc.set_prot_ctrl(1);
        gf20pc.set_prot_nav(1);
        gf20pc.set_txop_allow_cck_tx(0);
        gf20pc.set_txop_allow_ofdm_tx(1);
        gf20pc.set_txop_allow_mm20_tx(1);
        gf20pc.set_txop_allow_mm40_tx(0);
        gf20pc.set_txop_allow_gf20_tx(1);
        gf20pc.set_txop_allow_gf40_tx(0);
        gf20pc.set_rtsth_en(0);
        status = WriteRegister(gf20pc);
        CHECK_WRITE(GF20_PROT_CFG, status);
    }

    {
        Gf40ProtCfg gf40pc;
        status = ReadRegister(&gf40pc);
        CHECK_READ(GF40_PROT_CFG, status);
        gf40pc.set_prot_rate(0x4084);
        gf40pc.set_prot_ctrl(1);
        gf40pc.set_prot_nav(1);
        gf40pc.set_txop_allow_cck_tx(0);
        gf40pc.set_txop_allow_ofdm_tx(1);
        gf40pc.set_txop_allow_mm20_tx(1);
        gf40pc.set_txop_allow_mm40_tx(1);
        gf40pc.set_txop_allow_gf20_tx(1);
        gf40pc.set_txop_allow_gf40_tx(1);
        gf40pc.set_rtsth_en(0);
        status = WriteRegister(gf40pc);
        CHECK_WRITE(GF40_PROT_CFG, status);
    }

    {
        PbfCfg pc;
        pc.set_rx0q_en(1);
        pc.set_tx2q_en(1);
        pc.set_tx2q_num(20);
        pc.set_tx1q_num(7);
        status = WriteRegister(pc);
        CHECK_WRITE(PBF_CFG, status);
    }

    {
        WpdmaGloCfg wgc;
        status = ReadRegister(&wgc);
        CHECK_READ(WPDMA_GLO_CFG, status);
        wgc.set_tx_dma_en(0);
        wgc.set_tx_dma_busy(0);
        wgc.set_rx_dma_en(0);
        wgc.set_rx_dma_busy(0);
        wgc.set_wpdma_bt_size(3);
        wgc.set_tx_wb_ddone(0);
        wgc.set_big_endian(0);
        wgc.set_hdr_seg_len(0);
        status = WriteRegister(wgc);
        CHECK_WRITE(WPDMA_GLO_CFG, status);
    }

    {
        TxopCtrlCfg tcc;
        status = ReadRegister(&tcc);
        CHECK_READ(TXOP_CTRL_CFG, status);
        tcc.set_txop_trun_en(0x3f);
        tcc.set_lsig_txop_en(0);
        tcc.set_ext_cca_en(0);
        tcc.set_ext_cca_dly(88);
        tcc.set_ext_cw_min(0);
        status = WriteRegister(tcc);
        CHECK_WRITE(TXOP_CTRL_CFG, status);
    }

    {
        TxopHldrEt the;
        the.set_tx40m_blk_en(1);
        status = WriteRegister(the);
        CHECK_WRITE(TXOP_HLDR_ET, status);
    }

    {
        TxRtsCfg txrtscfg;
        status = ReadRegister(&txrtscfg);
        CHECK_READ(TX_RTS_CFG, status);
        txrtscfg.set_rts_rty_limit(32);
        txrtscfg.set_rts_thres(2353);  // IEEE80211_MAX_RTS_THRESHOLD in Linux
        txrtscfg.set_rts_fbk_en(0);
        status = WriteRegister(txrtscfg);
        CHECK_WRITE(TX_RTS_CFG, status);
    }

    {
        ExpAckTime eat;
        eat.set_exp_cck_ack_time(0x00ca);
        eat.set_exp_ofdm_ack_time(0x0024);
        status = WriteRegister(eat);
        CHECK_WRITE(EXP_ACK_TIME, status);
    }

    {
        XifsTimeCfg xtc;
        status = ReadRegister(&xtc);
        CHECK_READ(XIFS_TIME_CFG, status);
        xtc.set_cck_sifs_time(16);
        xtc.set_ofdm_sifs_time(16);
        xtc.set_ofdm_xifs_time(4);
        xtc.set_eifs_time(314);
        xtc.set_bb_rxend_en(1);
        status = WriteRegister(xtc);
        CHECK_WRITE(XIFS_TIME_CFG, status);
    }

    {
        PwrPinCfg ppc;
        ppc.set_io_rf_pe(1);
        ppc.set_io_ra_pe(1);
        status = WriteRegister(ppc);
        CHECK_WRITE(PWR_PIN_CFG, status);
    }

    // SharedKeyModeEntries???

    // WCID stuff and MacIveivEntries???

    // Clear beacons

    {
        UsCycCnt ucc;
        status = ReadRegister(&ucc);
        CHECK_READ(US_CYC_CNT, status);
        ucc.set_us_cyc_count(30);
        status = WriteRegister(ucc);
        CHECK_WRITE(US_CYC_CNT, status);
    }

    {
        HtFbkCfg0 hfc0;
        status = ReadRegister(&hfc0);
        CHECK_READ(HT_FBK_CFG0, status);
        hfc0.set_ht_mcs0_fbk(0);
        hfc0.set_ht_mcs1_fbk(0);
        hfc0.set_ht_mcs2_fbk(1);
        hfc0.set_ht_mcs3_fbk(2);
        hfc0.set_ht_mcs4_fbk(3);
        hfc0.set_ht_mcs5_fbk(4);
        hfc0.set_ht_mcs6_fbk(5);
        hfc0.set_ht_mcs7_fbk(6);
        status = WriteRegister(hfc0);
        CHECK_WRITE(HT_FBK_CFG0, status);
    }

    {
        HtFbkCfg1 hfc1;
        status = ReadRegister(&hfc1);
        CHECK_READ(HT_FBK_CFG1, status);
        hfc1.set_ht_mcs8_fbk(8);
        hfc1.set_ht_mcs9_fbk(8);
        hfc1.set_ht_mcs10_fbk(9);
        hfc1.set_ht_mcs11_fbk(10);
        hfc1.set_ht_mcs12_fbk(11);
        hfc1.set_ht_mcs13_fbk(12);
        hfc1.set_ht_mcs14_fbk(13);
        hfc1.set_ht_mcs15_fbk(14);
        status = WriteRegister(hfc1);
        CHECK_WRITE(HT_FBK_CFG1, status);
    }

    {
        LgFbkCfg0 lfc0;
        status = ReadRegister(&lfc0);
        CHECK_READ(LG_FBK_CFG0, status);
        lfc0.set_ofdm0_fbk(8);
        lfc0.set_ofdm1_fbk(8);
        lfc0.set_ofdm2_fbk(9);
        lfc0.set_ofdm3_fbk(10);
        lfc0.set_ofdm4_fbk(11);
        lfc0.set_ofdm5_fbk(12);
        lfc0.set_ofdm6_fbk(13);
        lfc0.set_ofdm7_fbk(14);
        status = WriteRegister(lfc0);
        CHECK_WRITE(LG_FBK_CFG0, status);
    }

    {
        LgFbkCfg1 lfc1;
        status = ReadRegister(&lfc1);
        CHECK_READ(LG_FBK_CFG1, status);
        lfc1.set_cck0_fbk(0);
        lfc1.set_cck1_fbk(0);
        lfc1.set_cck2_fbk(1);
        lfc1.set_cck3_fbk(2);
        status = WriteRegister(lfc1);
        CHECK_WRITE(LG_FBK_CFG1, status);
    }

    {
        // Linux does not force BA window sizes.
        ForceBaWinsize fbw;
        status = ReadRegister(&fbw);
        CHECK_READ(FORCE_BA_WINSIZE, status);
        fbw.set_force_ba_winsize(0);
        fbw.set_force_ba_winsize_en(0);
        status = WriteRegister(fbw);
        CHECK_WRITE(FORCE_BA_WINSIZE, status);
    }

    {
        // Reading the stats counters will clear them. We don't need to look at the
        // values.
        RxStaCnt0 rsc0;
        ReadRegister(&rsc0);
        RxStaCnt1 rsc1;
        ReadRegister(&rsc1);
        RxStaCnt2 rsc2;
        ReadRegister(&rsc2);
        TxStaCnt0 tsc0;
        ReadRegister(&tsc0);
        TxStaCnt1 tsc1;
        ReadRegister(&tsc1);
        TxStaCnt2 tsc2;
        ReadRegister(&tsc2);
    }

    {
        IntTimerCfg itc;
        status = ReadRegister(&itc);
        CHECK_READ(INT_TIMER_CFG, status);
        itc.set_pre_tbtt_timer(6 << 4);
        status = WriteRegister(itc);
        CHECK_WRITE(INT_TIMER_CFG, status);
    }

    {
        ChTimeCfg ctc;
        status = ReadRegister(&ctc);
        CHECK_READ(CH_TIME_CFG, status);
        ctc.set_ch_sta_timer_en(1);
        ctc.set_tx_as_ch_busy(1);
        ctc.set_rx_as_ch_busy(1);
        ctc.set_nav_as_ch_busy(1);
        ctc.set_eifs_as_ch_busy(1);
        status = WriteRegister(ctc);
        CHECK_WRITE(CH_TIME_CFG, status);
    }

    return NO_ERROR;
}

mx_status_t Device::McuCommand(uint8_t command, uint8_t token, uint8_t arg0, uint8_t arg1) {
    H2mMailboxCsr hmc;
    BusyPredicate<H2mMailboxCsr> pred = [](H2mMailboxCsr* hmc) { return !hmc->owner(); };
    auto status = BusyWait(&hmc, pred);
    if (status < 0) {
        return status;
    }

    hmc.set_owner(1);
    hmc.set_cmd_token(token);
    hmc.set_arg0(arg0);
    hmc.set_arg1(arg1);
    status = WriteRegister(hmc);
    CHECK_WRITE(H2M_MAILBOX_CSR, status);

    HostCmd hc;
    hc.set_command(command);
    status = WriteRegister(hc);
    CHECK_WRITE(HOST_CMD, status);
    mxsleep(1ms);

    return status;
}

mx_status_t Device::ReadBbp(uint8_t addr, uint8_t* val) {
    BbpCsrCfg bcc;
    BusyPredicate<BbpCsrCfg> pred = [](BbpCsrCfg* bcc) { return !bcc->bbp_csr_kick(); };

    auto status = BusyWait(&bcc, pred);
    if (status < 0) {
        if (status == ERR_TIMED_OUT) {
            std::printf("rt5370 timed out waiting for BBP\n");
        }
        return status;
    }

    bcc.clear();
    bcc.set_bbp_addr(addr);
    bcc.set_bbp_csr_rw(1);
    bcc.set_bbp_csr_kick(1);
    bcc.set_bbp_rw_mode(1);
    status = WriteRegister(bcc);
    CHECK_WRITE(BBP_CSR_CFG, status);

    status = BusyWait(&bcc, pred);
    if (status < 0) {
        if (status == ERR_TIMED_OUT) {
            std::printf("rt5370 timed out waiting for BBP\n");
            *val = 0xff;
        }
        return status;
    }

    *val = bcc.bbp_data();
    return NO_ERROR;
}

mx_status_t Device::WriteBbp(uint8_t addr, uint8_t val) {
    BbpCsrCfg bcc;
    BusyPredicate<BbpCsrCfg> pred = [](BbpCsrCfg* bcc) { return !bcc->bbp_csr_kick(); };
    auto status = BusyWait(&bcc, pred);
    if (status < 0) {
        if (status == ERR_TIMED_OUT) {
            std::printf("rt5370 timed out waiting for BBP\n");
        }
        return status;
    }

    bcc.clear();
    bcc.set_bbp_data(val);
    bcc.set_bbp_addr(addr);
    bcc.set_bbp_csr_rw(0);
    bcc.set_bbp_csr_kick(1);
    bcc.set_bbp_rw_mode(1);
    status = WriteRegister(bcc);
    CHECK_WRITE(BBP_CSR_CFG, status);
    return status;
}

mx_status_t Device::WaitForBbp() {
    H2mBbpAgent hba;
    auto status = WriteRegister(hba);
    CHECK_WRITE(H2M_BBP_AGENT, status);

    H2mMailboxCsr hmc;
    status = WriteRegister(hmc);
    CHECK_WRITE(H2M_MAILBOX_CSR, status);
    mxsleep(1ms);

    uint8_t val;
    for (unsigned int i = 0; i < kMaxBusyReads; i++) {
        ReadBbp(0, &val);
        if ((val != 0xff) && (val != 0x00)) return NO_ERROR;
        mxsleep(kDefaultBusyWait);
    }
    std::printf("rt5370 timed out waiting for BBP ready\n");
    return ERR_TIMED_OUT;
}

mx_status_t Device::DisableWpdma() {
    WpdmaGloCfg wgc;
    auto status = ReadRegister(&wgc);
    CHECK_READ(WPDMA_GLO_CFG, status);
    wgc.set_tx_dma_en(0);
    wgc.set_tx_dma_busy(0);
    wgc.set_rx_dma_en(0);
    wgc.set_rx_dma_busy(0);
    wgc.set_tx_wb_ddone(1);
    status = WriteRegister(wgc);
    CHECK_WRITE(WPDMA_GLO_CFG, status);
    std::printf("rt5370 disabled WPDMA\n");
    return NO_ERROR;
}

mx_status_t Device::DetectAutoRun(bool* autorun) {
    uint32_t fw_mode = 0;
    mx_status_t status = usb_control(usb_device_, (USB_DIR_IN | USB_TYPE_VENDOR),
                                     kDeviceMode, kAutorun, 0, &fw_mode, sizeof(fw_mode));
    if (status < 0) {
        std::printf("rt5370 DeviceMode error: %d\n", status);
        return status;
    }

    fw_mode = letoh32(fw_mode);
    if ((fw_mode & 0x03) == 2) {
        std::printf("rt5370 AUTORUN\n");
        *autorun = true;
    } else {
        *autorun = false;
    }
    return NO_ERROR;
}

mx_status_t Device::WaitForMacCsr() {
    AsicVerId avi;
    BusyPredicate<AsicVerId> pred = [](AsicVerId* avi) { return avi->val() && avi->val() != ~0u; };
    return BusyWait(&avi, pred, 1ms);
}

template <typename R>
mx_status_t Device::BusyWait(R* reg, BusyPredicate<R> pred, std::chrono::microseconds delay) {
    mx_status_t status;
    unsigned int busy;
    for (busy = 0; busy < kMaxBusyReads; busy++) {
        status = ReadRegister(reg);
        if (status < 0) return status;
        if (pred(reg)) break;
        mxsleep(delay);
    }
    if (busy == kMaxBusyReads) {
        return ERR_TIMED_OUT;
    }
    return NO_ERROR;
}

void Device::HandleRxComplete(iotxn_t* request) {
    std::printf("rt5370::Device::HandleRxComplete\n");
    if (request->status == ERR_REMOTE_CLOSED) {
        request->ops->release(request);
        return;
    }

    std::lock_guard<std::mutex> guard(lock_);

    if (request->status == NO_ERROR) {
        // Handle completed rx
        std::printf("rt5370 rx complete\n");
        completed_reads_.push_back(request);
    } else {
        std::printf("rt5370 rx txn status %d\n", request->status);
        iotxn_queue(usb_device_, request);
    }
    UpdateSignals_Locked();
}

void Device::HandleTxComplete(iotxn_t* request) {
    std::printf("rt5370::Device::HandleTxComplete\n");
    if (request->status == ERR_REMOTE_CLOSED) {
        request->ops->release(request);
        return;
    }

    std::lock_guard<std::mutex> guard(lock_);

    free_write_reqs_.push_back(request);
    UpdateSignals_Locked();
}

void Device::UpdateSignals_Locked() {
    mx_signals_t new_signals = 0;

    if (dead_) {
        new_signals |= (DEV_STATE_READABLE | DEV_STATE_ERROR);
    }
    if (!completed_reads_.empty()) {
        new_signals |= DEV_STATE_READABLE;
    }
    if (!free_write_reqs_.empty()) {
        new_signals |= DEV_STATE_WRITABLE;
    }
    if (new_signals != signals_) {
        device_state_set_clr(&device_, new_signals & ~signals_, signals_ & ~new_signals);
        signals_ = new_signals;
    }
}

void Device::Unbind() {
    std::printf("rt5370::Device::Unbind\n");
    {
        std::lock_guard<std::mutex> guard(lock_);
        dead_ = true;
        UpdateSignals_Locked();
    }
    device_remove(&device_);
}

mx_status_t Device::Release() {
    std::printf("rt5370::Device::Release\n");
    delete this;
    return NO_ERROR;
}

ssize_t Device::Read(void* buf, size_t count) {
    std::printf("rt5370::Device::Read %p %zu\n", buf, count);

    if (dead_) {
        return ERR_REMOTE_CLOSED;
    }

    return 0;
}

ssize_t Device::Write(const void* buf, size_t count) {
    std::printf("rt5370::Device::Write %p %zu\n", buf, count);

    if (dead_) {
        return ERR_REMOTE_CLOSED;
    }

    std::lock_guard<std::mutex> guard(lock_);

    if (free_write_reqs_.empty()) {
        UpdateSignals_Locked();
        return ERR_BUFFER_TOO_SMALL;
    }

    auto txn = free_write_reqs_.back();
    free_write_reqs_.pop_back();
    if (count > kWriteBufSize) {
        UpdateSignals_Locked();
        return ERR_INVALID_ARGS;
    }

    // Add TX header
    txn->ops->copyto(txn, buf, count, 0);
    txn->length = count;
    iotxn_queue(usb_device_, txn);

    UpdateSignals_Locked();
    return count;
}

ssize_t Device::Ioctl(IoctlOp op, const void* in_buf, size_t in_len,
                      void* out_buf, size_t out_len) {
    std::printf("rt5370::Device::Ioctl op %u in %p (%zu bytes) out %p (%zu bytes)\n",
            static_cast<uint32_t>(op), in_buf, in_len, out_buf, out_len);
    switch (op) {
    case IoctlOp::Scan:
        break;
    case IoctlOp::Associate:
        break;
    case IoctlOp::Disassociate:
        break;
    }
    return 0;
}

void Device::DdkUnbind(mx_device_t* device) {
    auto dev = static_cast<Device*>(device->ctx);
    dev->Unbind();
}

mx_status_t Device::DdkRelease(mx_device_t* device) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Release();
}

ssize_t Device::DdkRead(mx_device_t* device, void* buf, size_t count, mx_off_t off) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Read(buf, count);
}

ssize_t Device::DdkWrite(mx_device_t* device, const void* buf, size_t count, mx_off_t off) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Write(buf, count);
}

ssize_t Device::DdkIoctl(mx_device_t* device, uint32_t op, const void* in_buf,
                 size_t in_len, void* out_buf, size_t out_len) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Ioctl(static_cast<IoctlOp>(op), in_buf, in_len, out_buf, out_len);
}

void Device::ReadIotxnComplete(iotxn_t* request, void* cookie) {
    auto dev = static_cast<Device*>(cookie);
    auto f = std::async(std::launch::async, &rt5370::Device::HandleRxComplete, dev, request);
}

void Device::WriteIotxnComplete(iotxn_t* request, void* cookie) {
    auto dev = static_cast<Device*>(cookie);
    auto f = std::async(std::launch::async, &rt5370::Device::HandleTxComplete, dev, request);
}

}  // namespace rt5370
