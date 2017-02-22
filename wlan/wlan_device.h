// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include "mac_frame.h"

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/ethernet.h>
#include <ddk/protocol/wlan.h>
#include <magenta/device/wlan.h>
#include <mx/channel.h>

#include <mutex>
#include <unistd.h>  // ssize_t
#include <unordered_set>

namespace wlan {

class Device {
  public:
    Device(mx_driver_t* driver, mx_device_t* device, wlanmac_protocol_t* wlanmac_ops);
    ~Device();

    mx_status_t Bind();

  private:
    // DDK API
    void Unbind();
    mx_status_t Release();

    mx_status_t Query(uint32_t options, ethmac_info_t* info);
    mx_status_t Start(ethmac_ifc_t* ifc, void* cookie);
    void Stop();

    void Status(uint32_t status);
    void Recv(void* data, size_t length, uint32_t flags);
    void Send(uint32_t options, void* data, size_t length);

    ssize_t StartScan(const wlan_start_scan_args* args, mx_handle_t* out_channel);

    void HandleMgmtFrame(FrameControl fc, uint8_t* data, size_t length, uint32_t flags);
    void HandleBeacon(FrameControl fc, MgmtFrame* mf, uint32_t flags);
    void SendBeacon(const Beacon& beacon);

    static void DdkUnbind(mx_device_t* device);
    static mx_status_t DdkRelease(mx_device_t* device);
    static ssize_t DdkIoctl(mx_device_t* device, uint32_t op, const void* in_buf, size_t in_len,
                            void* out_buf, size_t out_len);

    static mx_status_t EthQuery(mx_device_t* device, uint32_t options, ethmac_info_t* info);
    static void EthStop(mx_device_t* device);
    static mx_status_t EthStart(mx_device_t* device, ethmac_ifc_t* ifc, void* cookie);
    static void EthSend(mx_device_t* device, uint32_t options, void* data, size_t length);


    static void WlanStatus(void* cookie, uint32_t status);
    static void WlanRecv(void* cookie, void* data, size_t length, uint32_t flags);

    mx_driver_t* driver_;

    // Hardware driver
    mx_device_t* wlanmac_device_;
    wlanmac_protocol_t* wlanmac_ops_;
    wlanmac_ifc_t wlanmac_ifc_ = {};

    // Ethermac interface
    ethmac_protocol_t ethmac_ops_;
    ethmac_ifc_t* ethmac_ifc_ = nullptr;;
    void* ethmac_cookie_ = nullptr;;

    // Generic ddk device
    mx_device_t device_;
    mx_protocol_device_t device_ops_;

    std::mutex lock_;

    // TODO: put this somewhere general
    struct ChannelHasher {
        std::size_t operator()(const mx::channel& ch) const;
    };
    std::unordered_set<mx::channel, ChannelHasher> scan_channels_;
};

}  // namespace wlan
