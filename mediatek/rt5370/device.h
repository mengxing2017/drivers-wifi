// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/device.h>
#include <ddk/driver.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rt5370 {

class Device {
  public:
    Device(mx_driver_t* driver, mx_device_t* device, uint8_t bulk_in,
           std::vector<uint8_t>&& bulk_out);
    ~Device();

    mx_status_t Bind();

  private:
    enum class IoctlOp {
        Scan,
        Associate,
        Disassociate,
    };

    mx_status_t ReadRegister(uint16_t offset, uint32_t* value);
    mx_status_t WriteRegister(uint16_t offset, uint32_t value);

    mx_status_t ReadEeprom();
    mx_status_t ValidateEeprom();

    // DDK API
    void Unbind();
    mx_status_t Release();
    ssize_t Read(void* buf, size_t count);
    ssize_t Write(const void* buf, size_t count);
    ssize_t Ioctl(IoctlOp op, const void* in_buf, size_t in_len,
                  void* out_buf, size_t out_len);

    static void DdkUnbind(mx_device_t* device);
    static mx_status_t DdkRelease(mx_device_t* device);
    static ssize_t DdkRead(mx_device_t* device, void* buf, size_t count);
    static ssize_t DdkWrite(mx_device_t* device, const void* buf, size_t count);
    static ssize_t DdkIoctl(mx_device_t* device, uint32_t op, const void* in_buf,
                            size_t in_len, void* out_buf, size_t out_len);

    mx_driver_t* driver_;
    mx_device_t* usb_device_;
    mx_device_t device_;

    uint8_t rx_endpt_ = 0;
    //uint8_t beacon_endpt_ = 0;
    std::vector<uint8_t> tx_endpts_;

    constexpr static size_t kEepromSize = 0x0100;
    std::array<uint16_t, kEepromSize> eeprom_ = {};

    uint16_t rt_type_ = 0;
    uint16_t rt_rev_ = 0;
    uint16_t rf_type_ = 0;
};

}  // namespace rt5370
