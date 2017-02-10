// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/binding.h>

#include <magenta/hw/usb.h>
#include <magenta/device/usb.h>

#include <magenta/types.h>

extern mx_status_t mediatek_rt5370_bind(mx_driver_t* driver, mx_device_t* device, void** cookie);

mx_driver_t _driver_mediatek_rt5370 = {
    .ops = {
        .bind = mediatek_rt5370_bind,
    },
};

MAGENTA_DRIVER_BEGIN(_driver_mediatek_rt5370, "mediatek-rt5370", "magenta", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PROTOCOL, MX_PROTOCOL_USB),
    BI_ABORT_IF(NE, BIND_USB_VID, 0x148f),
    BI_MATCH_IF(EQ, BIND_USB_PID, 0x5370),
MAGENTA_DRIVER_END(_driver_mediatek_rt5370)
