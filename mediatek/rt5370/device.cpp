// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "device.h"
#include "rt5370.h"

#include <ddk/common/usb.h>
#include <magenta/hw/usb.h>

#include <endian.h>

#include <cstdio>

#define CHECK_REG(reg, op, status) \
    do { \
        if (status < 0) { \
            std::printf("rt5370 #op##Register error for #reg: %d\n", status); \
        } \
    } while (0)
#define CHECK_READ(reg, status) CHECK_REG(reg, Read, status)
#define CHECK_WRITE(reg, status) CHECK_REG(reg, Write, status)

namespace {

constexpr static int kMaxBusyReads = 20;
constexpr static int kRegisterBusyWait = 100 * 1000;  // 100us in nanos

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
    return ret |= ((value << offset) & mask);
}

constexpr uint32_t set_bit(uint8_t offset, uint32_t orig) {
    return set_bits(offset, 1, orig, 1);
}

constexpr uint32_t clear_bit(uint8_t offset, uint32_t orig) {
    return set_bits(offset, 1, orig, 0);
}

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

Device::~Device() {}

mx_status_t Device::Bind() {
    std::printf("rt5370::Device::Bind\n");

    uint32_t mac_csr = 0;
    mx_status_t status = ReadRegister(MAC_CSR0, &mac_csr);
    CHECK_READ(status, MAC_CSR0);

    rt_type_ = (mac_csr >> 16) & 0xffff;
    rt_rev_ = mac_csr & 0xffff;
    std::printf("rt5370 RT chipset %#x, rev %#x\n", rt_type_, rt_rev_);

    uint32_t fw_mode = 0;
    status = usb_control(usb_device_, (USB_DIR_IN | USB_TYPE_VENDOR), kDeviceMode, kAutorun,
            0, &fw_mode, sizeof(fw_mode));
    if (status < 0) {
        std::printf("rt5370 DeviceMode error: %d\n", status);
        return status;
    }

    fw_mode = letoh32(fw_mode);
    if ((fw_mode & 0x03) == 2) {
        std::printf("rt5370 AUTORUN\n");
    }

    uint32_t efuse_ctrl = 0;
    status = ReadRegister(EFUSE_CTRL, &efuse_ctrl);
    CHECK_READ(EFUSE_CTRL, status);

    std::printf("rt5370 efuse ctrl reg: %#x\n", efuse_ctrl);
    bool efuse_present = (efuse_ctrl & 0x80000000) > 0;
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

    //status = LoadFirm
    return NO_ERROR;
}

mx_status_t Device::ReadRegister(uint16_t offset, uint32_t* value) {
    return usb_control(usb_device_, (USB_DIR_IN | USB_TYPE_VENDOR), kMultiRead, 0,
            offset, value, sizeof(*value));
}

mx_status_t Device::WriteRegister(uint16_t offset, uint32_t value) {
    return usb_control(usb_device_, (USB_DIR_OUT | USB_TYPE_VENDOR), kMultiWrite, 0,
            offset, &value, sizeof(value));
}

mx_status_t Device::ReadEeprom() {
    // Read 4 entries at a time
    for (unsigned int i = 0; i < eeprom_.size(); i += 8) {
        uint32_t reg = 0;
        mx_status_t status = ReadRegister(EFUSE_CTRL, &reg);
        CHECK_READ(EFUSE_CTRL, status);

        // Set the address and tell it to load the next four words. Addresses
        // must be 16-byte aligned.
        reg = set_bits(EFUSE_CTRL_ADDR, EFUSE_CTRL_ADDR_WIDTH, reg, (i << 1));
        reg = clear_bit(EFUSE_CTRL_MODE, reg);
        reg = set_bit(EFUSE_CTRL_KICK, reg);
        status = WriteRegister(EFUSE_CTRL, reg);
        CHECK_WRITE(EFUSE_CTRL, status);

        // Wait until the registers are ready for reading.
        unsigned int busy = 0;
        for (busy = 0; busy < kMaxBusyReads; busy++) {
            status = ReadRegister(EFUSE_CTRL, &reg);
            CHECK_READ(EFUSE_CTRL, status);
            if (!get_bit(EFUSE_CTRL_KICK, reg)) break;
            mx_nanosleep(kRegisterBusyWait);
        }
        if (busy == kMaxBusyReads) {
            std::printf("rt5370 Busy-wait for EFUSE_CTRL failed\n");
            return ERR_TIMED_OUT;
        }

        // Read the registers into the eeprom. EEPROM is read in descending
        // order, and are always return in host order but to be interpreted as
        // little endian.
        status = ReadRegister(EFUSE_DATA3, &reg);
        CHECK_READ(EFUSE_DATA3, status);
        eeprom_[i] = htole32(reg) & 0xffff;
        eeprom_[i+1] = htole32(reg) >> 16;

        status = ReadRegister(EFUSE_DATA2, &reg);
        CHECK_READ(EFUSE_DATA2, status);
        eeprom_[i+2] = htole32(reg) & 0xffff;
        eeprom_[i+3] = htole32(reg) >> 16;

        status = ReadRegister(EFUSE_DATA1, &reg);
        CHECK_READ(EFUSE_DATA1, status);
        eeprom_[i+4] = htole32(reg) & 0xffff;
        eeprom_[i+5] = htole32(reg) >> 16;

        status = ReadRegister(EFUSE_DATA0, &reg);
        CHECK_READ(EFUSE_DATA0, status);
        eeprom_[i+6] = htole32(reg) & 0xffff;
        eeprom_[i+7] = htole32(reg) >> 16;
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

void Device::Unbind() {
    std::printf("rt5370::Device::Unbind\n");
    device_remove(&device_);
}

mx_status_t Device::Release() {
    std::printf("rt5370::Device::Release\n");
    delete this;
    return NO_ERROR;
}

ssize_t Device::Read(void* buf, size_t count) {
    std::printf("rt5370::Device::Read %p %zu\n", buf, count);
    return 0;
}

ssize_t Device::Write(const void* buf, size_t count) {
    std::printf("rt5370::Device::Write %p %zu\n", buf, count);
    return 0;
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

ssize_t Device::DdkRead(mx_device_t* device, void* buf, size_t count) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Read(buf, count);
}

ssize_t Device::DdkWrite(mx_device_t* device, const void* buf, size_t count) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Write(buf, count);
}

ssize_t Device::DdkIoctl(mx_device_t* device, uint32_t op, const void* in_buf,
                 size_t in_len, void* out_buf, size_t out_len) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Ioctl(static_cast<IoctlOp>(op), in_buf, in_len, out_buf, out_len);
}

}  // namespace rt5370
