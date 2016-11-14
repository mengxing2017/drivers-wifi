// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "device.h"
#include "rt5370.h"

#include <ddk/common/usb.h>
#include <magenta/hw/usb.h>

#include <cstdio>

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
    if (status < 0) {
        std::printf("rt5370 ReadRegister error: %d\n", status);
        return status;
    }

    std::printf("rt5370 MAC CSR: %#x:%#x\n", static_cast<uint16_t>(mac_csr & 0xffff),
            static_cast<uint16_t>((mac_csr >> 8) & 0xffff));

    return NO_ERROR;
}

mx_status_t Device::ReadRegister(uint16_t offset, uint32_t* value) {
    return usb_control(usb_device_, (USB_DIR_IN | USB_TYPE_VENDOR), kMultiRead, 0,
            offset, value, sizeof(*value));
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
