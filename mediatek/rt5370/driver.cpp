// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/device.h>
#include <ddk/driver.h>

#include <cstdio>

extern "C" mx_status_t mediatek_rt5370_bind(mx_driver_t* driver, mx_device_t* device) {
    std::printf("%s\n", __func__);

    return NO_ERROR;
}
