// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "wlan_device.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cinttypes>
#include <functional>

#define DEBUG_BEACON 0

namespace wlan {

Device::Device(mx_driver_t* driver, mx_device_t* device, wlanmac_protocol_t* wlanmac_ops)
  : driver_(driver), wlanmac_device_(device), wlanmac_ops_(wlanmac_ops) {
    std::memset(&driver_, 0, sizeof(driver_));
}

Device::~Device() {}

mx_status_t Device::Bind() {
    std::memset(&device_ops_, 0, sizeof(device_ops_));
    device_ops_.unbind = &Device::DdkUnbind;
    device_ops_.release = &Device::DdkRelease;
    device_ops_.ioctl = &Device::DdkIoctl;
    device_init(&device_, driver_, "wlan", &device_ops_);

    ethmac_ops_.query = &Device::EthQuery;
    ethmac_ops_.start = &Device::EthStart;
    ethmac_ops_.stop = &Device::EthStop;
    ethmac_ops_.send = &Device::EthSend;

    wlanmac_ifc_.status = &Device::WlanStatus;
    wlanmac_ifc_.recv = &Device::WlanRecv;

    device_.ctx = this;
    device_.protocol_id = MX_PROTOCOL_ETHERMAC;
    device_.protocol_ops = &ethmac_ops_;
    auto status = device_add(&device_, wlanmac_device_);
    if (status != NO_ERROR) {
        std::printf("wlan could not add device err=%d\n", status);
    } else {
        std::printf("wlan device added\n");
    }

    return status;
}

void Device::Unbind() {
    std::printf("wlan::Device::Unbind\n");
    device_remove(&device_);
}

mx_status_t Device::Release() {
    std::printf("wlan::Device::Release\n");
    delete this;
    return NO_ERROR;
}

mx_status_t Device::Query(uint32_t options, ethmac_info_t* info) {
    auto status = wlanmac_ops_->query(wlanmac_device_, options, info);
    if (status != NO_ERROR) {
        return status;
    }
    // Make sure this device is reported as a wlan device
    info->features |= ETHMAC_FEATURE_WLAN;
    return NO_ERROR;
}

mx_status_t Device::Start(ethmac_ifc_t* ifc, void* cookie) {
    ethmac_ifc_ = ifc;
    ethmac_cookie_ = cookie;
    return wlanmac_ops_->start(wlanmac_device_, &wlanmac_ifc_, this);
}

void Device::Stop() {
    wlanmac_ops_->stop(wlanmac_device_);
}

void Device::Status(uint32_t status) {
    std::printf("wlan::Device::Status = %u\n", status);
}

void Device::Recv(void* data, size_t length, uint32_t flags) {
    if (length < 2) return;
    auto bytes = reinterpret_cast<uint8_t*>(data);
    FrameControl fc(*reinterpret_cast<uint16_t*>(bytes));
    switch (fc.type()) {
        case kManagement:
            HandleMgmtFrame(fc, bytes + 2, length - 2, flags);
            break;
        default:
            break;
    }
}

void Device::Send(uint32_t options, void* data, size_t length) {
    wlanmac_ops_->tx(wlanmac_device_, options, data, length);
}

ssize_t Device::StartScan(const wlan_start_scan_args* args, mx_handle_t* out_channel) {
    mx::channel chan[2];
    auto status = mx::channel::create(0, &chan[0], &chan[1]);
    if (status < 0) {
        return status;
    }

    std::lock_guard<std::mutex> guard(lock_);
    scan_channels_.insert(std::move(chan[0]));
    *out_channel = chan[1].release();
    return sizeof(mx_handle_t);
}

void Device::HandleMgmtFrame(FrameControl fc, uint8_t* data, size_t length, uint32_t flags) {
    if (length < 26) {
        return;
    }

    uint8_t* head = data;
    MgmtFrame mf;
    mf.duration = *reinterpret_cast<uint16_t*>(data);
    data += sizeof(mf.duration);
    std::memcpy(mf.addr1, data, sizeof(mf.addr1));
    data += sizeof(mf.addr1);
    std::memcpy(mf.addr2, data, sizeof(mf.addr2));
    data += sizeof(mf.addr2);
    std::memcpy(mf.addr3, data, sizeof(mf.addr3));
    data += sizeof(mf.addr3);
    mf.sc.set_val(*reinterpret_cast<uint16_t*>(data));
    data += sizeof(mf.sc);
    if (fc.htc_order()) {
        mf.ht.set_val(*reinterpret_cast<uint32_t*>(data));
        data += sizeof(mf.ht);
    }
    mf.body = data;
    mf.body_len = length - (data - head);

    switch (fc.subtype()) {
        case kBeacon:
            if (mf.body_len < 16) break;
            HandleBeacon(fc, &mf, flags);
            break;
        default:
            break;
    }
}

void Device::HandleBeacon(FrameControl fc, MgmtFrame* mf, uint32_t flags) {
    Beacon bcn = {};
    std::memcpy(bcn.bssid, mf->addr3, sizeof(bcn.bssid));
    uint8_t* data = mf->body;
    bcn.timestamp = *reinterpret_cast<uint64_t*>(data);
    data += 8;
    bcn.beacon_interval = *reinterpret_cast<uint16_t*>(data);
    data += 2;
    bcn.cap.set_val(*reinterpret_cast<uint16_t*>(data));
    data += 2;
    while (data < mf->body + mf->body_len) {
        switch (*data) {
            case kSsid:
                bcn.ssid_len = std::min(data[1], static_cast<uint8_t>(32));
                std::memcpy(bcn.ssid, &data[2], bcn.ssid_len);
                break;
            case kSuppRates: {
                int num_rates = std::min(data[1], static_cast<uint8_t>(8));
                for (int i = 0; i < num_rates; i++) {
                    bcn.supp_rates[i] = data[2 + i];
                }
                break;
            }
            default:
                break;
        }
        data += 2 + data[1];
    }
    SendBeacon(bcn);

#if DEBUG_BEACON
    std::printf("wlan: beacon\n");
    std::printf("  bssid: %02x:%02x:%02x:%02x:%02x:%02x\n",
            bcn.bssid[0], bcn.bssid[1], bcn.bssid[2], bcn.bssid[3], bcn.bssid[4], bcn.bssid[5]);
    std::printf("  ts: %" PRIu64 "\n", bcn.timestamp);
    std::printf("  interval: %u\n", bcn.beacon_interval);
    // hack
    bcn.ssid[31] = 0;
    std::printf("  ssid: %s\n", bcn.ssid);
    std::printf("  supported rates:\n");
    for (int i = 0; i < 8; i++) {
        if (bcn.supp_rates[i] == 0) break;
        std::printf("    %.1f Mbps\n", (500.0f * (bcn.supp_rates[i] & 0x3f)) / 1000.0f);
    }
#endif
}

void Device::SendBeacon(const Beacon& beacon) {
    std::lock_guard<std::mutex> guard(lock_);
    if (scan_channels_.empty()) return;

    wlan_scan_report rpt;
    memset(&rpt, 0, sizeof(rpt));
    std::memcpy(rpt.bssid, beacon.bssid, sizeof(rpt.bssid));
    if (beacon.cap.ess()) {
        rpt.bss_type = WLAN_BSSTYPE_INFRASTRUCTURE;
    } else if (beacon.cap.ibss()) {
        rpt.bss_type = WLAN_BSSTYPE_INDEPENDENT;
    } else {
        rpt.bss_type = WLAN_BSSTYPE_UNKNOWN;
    }
    rpt.timestamp = beacon.timestamp;
    rpt.beacon_period = beacon.beacon_interval;
    rpt.capabilities = beacon.cap.val();
    std::memcpy(rpt.ssid, beacon.ssid, sizeof(rpt.ssid));
    rpt.ssid_len = beacon.ssid_len;
    std::memcpy(rpt.supported_rates, beacon.supp_rates, sizeof(rpt.supported_rates));

    for (auto iter = scan_channels_.cbegin(); iter != scan_channels_.cend(); ) {
        auto status = iter->write(0, &rpt, sizeof(rpt), NULL, 0);
        if (status == ERR_REMOTE_CLOSED) {
            iter = scan_channels_.erase(iter);
        } else {
            ++iter;
        }
    }
}

void Device::DdkUnbind(mx_device_t* device) {
    auto dev = static_cast<Device*>(device->ctx);
    dev->Unbind();
}

mx_status_t Device::DdkRelease(mx_device_t* device) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Release();
}

ssize_t Device::DdkIoctl(mx_device_t* device, uint32_t op, const void* in_buf, size_t in_len,
                         void* out_buf, size_t out_len) {
    auto dev = static_cast<Device*>(device->ctx);
    switch (op) {
    case IOCTL_WLAN_START_SCAN: {
        if (in_len < sizeof(wlan_start_scan_args)) return ERR_INVALID_ARGS;
        if (out_len < sizeof(mx_handle_t)) return ERR_INVALID_ARGS;
        const wlan_start_scan_args* args = reinterpret_cast<const wlan_start_scan_args*>(in_buf);
        mx_handle_t* h = reinterpret_cast<mx_handle_t*>(out_buf);
        return dev->StartScan(args, h);
    }
    default:
        return ERR_NOT_SUPPORTED;
    }
}

mx_status_t Device::EthQuery(mx_device_t* device, uint32_t options, ethmac_info_t* info) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Query(options, info);
}

void Device::EthStop(mx_device_t* device) {
    auto dev = static_cast<Device*>(device->ctx);
    dev->Stop();
}

mx_status_t Device::EthStart(mx_device_t* device, ethmac_ifc_t* ifc, void* cookie) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Start(ifc, cookie);
}

void Device::EthSend(mx_device_t* device, uint32_t options, void* data, size_t length) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Send(options, data, length);
}

void Device::WlanStatus(void* cookie, uint32_t status) {
    auto dev = static_cast<Device*>(cookie);
    dev->Status(status);
}

void Device::WlanRecv(void* cookie, void* data, size_t length, uint32_t flags) {
    auto dev = static_cast<Device*>(cookie);
    dev->Recv(data, length, flags);
}

std::size_t Device::ChannelHasher::operator()(const mx::channel& ch) const {
    return std::hash<mx_handle_t>{}(ch.get());
}

}  // namespace wlan
