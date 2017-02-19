// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "wlan_device.h"

#include <ddk/device.h>
#include <ddk/driver.h>

#include <cstdio>
#include <utility>

extern "C" mx_status_t wlan_bind(mx_driver_t* driver, mx_device_t* device, void** cookie) {
    std::printf("%s\n", __func__);

    wlanmac_protocol_t* wlanmac_ops;
    if (device_get_protocol(device, MX_PROTOCOL_WLANMAC, (void**)&wlanmac_ops)) {
        std::printf("wlan: bind: no wlanmac protocol\n");
        return ERR_INTERNAL;
    }

    auto wlandev = new wlan::Device(driver, device, wlanmac_ops);
    return wlandev->Bind();
}
