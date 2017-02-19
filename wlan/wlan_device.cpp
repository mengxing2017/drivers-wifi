// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "wlan_device.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cinttypes>

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
    device_init(&device_, driver_, "wlan", &device_ops_);

    device_.ctx = this;
    device_.protocol_id = MX_PROTOCOL_WLAN;
    device_.protocol_ops = nullptr;
    auto status = device_add(&device_, wlanmac_device_);
    if (status != NO_ERROR) {
        std::printf("wlan could not add device err=%d\n", status);
    } else {
        std::printf("wlan device added\n");
    }

    ifc.status = &Device::WlanStatus;
    ifc.recv = &Device::WlanRecv;
    wlanmac_ops_->start(wlanmac_device_, &ifc, this);

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
}

void Device::DdkUnbind(mx_device_t* device) {
    auto dev = static_cast<Device*>(device->ctx);
    dev->Unbind();
}

mx_status_t Device::DdkRelease(mx_device_t* device) {
    auto dev = static_cast<Device*>(device->ctx);
    return dev->Release();
}

void Device::WlanStatus(void* cookie, uint32_t status) {
    auto dev = static_cast<Device*>(cookie);
    dev->Status(status);
}

void Device::WlanRecv(void* cookie, void* data, size_t length, uint32_t flags) {
    auto dev = static_cast<Device*>(cookie);
    dev->Recv(data, length, flags);
}

}  // namespace wlan
