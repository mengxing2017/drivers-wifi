// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/binding.h>

#include <magenta/types.h>

extern mx_status_t wlan_test_bind(mx_driver_t* driver, mx_device_t* device, void** cookie);

mx_driver_t _driver_wlan_test = {
    .ops = {
        .bind = wlan_test_bind,
    },
};

MAGENTA_DRIVER_BEGIN(_driver_wlan_test, "wlan-test", "magenta", "0.1", 2)
    BI_ABORT_IF_AUTOBIND,
    BI_MATCH_IF(EQ, BIND_PROTOCOL, MX_PROTOCOL_TEST),
MAGENTA_DRIVER_END(_driver_wlan_test)
