// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "test-device.h"

#include <cstdio>

namespace wlan {
namespace testing {

Device::Device(mx_driver_t* driver, mx_device_t* device, test_protocol_t* test_ops)
  : TestBaseDevice("wlan-test", driver),
    test_proxy_(test_ops, device) {}

mx_status_t Device::Bind() {
    std::printf("wlan::testing::Device::Bind()\n");

    auto status = Add(test_proxy_.device());
    if (status != NO_ERROR) {
        std::printf("wlan-test: could not add test device: %d\n", status);
    }
    return status;
}

void Device::DdkUnbind() {
    std::printf("wlan::testing::Device::Unbind()\n");
    SetAndClearState(DEV_STATE_HANGUP, DEV_STATE_READABLE | DEV_STATE_WRITABLE);
    device_remove(mxdev());
}

void Device::DdkRelease() {
    std::printf("wlan::testing::Device::Release()\n");
    delete this;
}

mx_status_t Device::DdkIoctl(uint32_t op, const void* in_buf, size_t in_len, void* out_buf,
                             size_t out_len, size_t* out_actual) {
    return ERR_NOT_SUPPORTED;
}

mx_status_t Device::WlanmacQuery(uint32_t options, ethmac_info_t* info) {
    std::printf("wlan::testing::Device::WlanmacQuery()\n");
    static uint8_t mac[ETH_MAC_SIZE] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
    info->features = ETHMAC_FEATURE_WLAN;
    info->mtu = 1500;
    std::memcpy(info->mac, mac, ETH_MAC_SIZE);
    return NO_ERROR;
}

void Device::WlanmacStop() {
    std::printf("wlan::testing::Device::WlanmacStop()\n");
    std::lock_guard<std::mutex> lock(lock_);
    ClearState(DEV_STATE_READABLE | DEV_STATE_WRITABLE);
    wlanmac_proxy_.reset();
}

mx_status_t Device::WlanmacStart(mxtl::unique_ptr<ddk::WlanmacIfcProxy> proxy) {
    std::printf("wlan::testing::Device::WlanmacStart()\n");
    std::lock_guard<std::mutex> lock(lock_);
    SetState(DEV_STATE_READABLE | DEV_STATE_WRITABLE);
    if (wlanmac_proxy_ != nullptr) {
        return ERR_ALREADY_BOUND;
    } else {
        wlanmac_proxy_.swap(proxy);
    }
    return NO_ERROR;
}

void Device::WlanmacTx(uint32_t options, void* data, size_t length) {

}

mx_status_t Device::WlanmacSetChannel(uint32_t options, wlan_channel_t* chan) {
    std::printf("wlan::testing::Device::WlanmacSetChannel()  chan=%u\n", chan->channel_num);
    return NO_ERROR;
}

}  // namespace testing
}  // namespace wlan
