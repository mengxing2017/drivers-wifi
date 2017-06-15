// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "device.h"

#include <ddk/device.h>
#include <ddk/driver.h>

#include <cstdio>
#include <memory>

extern "C" mx_status_t wlan_bind(void* ctx, mx_device_t* device, void** cookie) {
    std::printf("%s\n", __func__);

    wlanmac_protocol_t wlanmac_proto;
    if (device_get_protocol(device, MX_PROTOCOL_WLANMAC, reinterpret_cast<void*>(&wlanmac_proto))) {
        std::printf("wlan: bind: no wlanmac protocol\n");
        return MX_ERR_INTERNAL;
    }

    auto wlandev = std::make_unique<wlan::Device>(device, &wlanmac_proto);
    auto status = wlandev->Bind();
    if (status != MX_OK) {
        std::printf("wlan: could not bind: %d\n", status);
    } else {
        // devhost is now responsible for the memory used by wlandev. It will be
        // cleaned up in the Device::Release() method.
        wlandev.release();
    }
    return status;
}
