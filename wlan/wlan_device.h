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

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
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
    void JoinVermont();

    void HandleMgmtFrame(FrameControl fc, uint8_t* data, size_t length, uint32_t flags);
    void HandleBeacon(FrameControl fc, MgmtFrame* mf, uint32_t flags);
    void SendBeacon(const Beacon& beacon);

    static ssize_t DdkIoctl(mx_device_t* device, uint32_t op, const void* in_buf, size_t in_len,
                            void* out_buf, size_t out_len);

    mx_driver_t* driver_;

    // Hardware driver
    mx_device_t* wlanmac_device_;
    wlanmac_protocol_t* wlanmac_ops_;
    static wlanmac_ifc_t wlanmac_ifc_;

    // Ethermac interface
    static ethmac_protocol_t ethmac_ops_;
    ethmac_ifc_t* ethmac_ifc_ = nullptr;;
    void* ethmac_cookie_ = nullptr;;

    // Generic ddk device
    mx_device_t device_;
    static mx_protocol_device_t device_ops_;

    std::mutex lock_;

    // MAC address (cached)
    uint8_t mac_addr_[6];

    // MAC sequence number
    std::atomic_uint16_t seqno_;

    // TODO: put this somewhere general
    struct ChannelHasher {
        std::size_t operator()(const mx::channel& ch) const;
    };
    std::unordered_set<mx::channel, ChannelHasher> scan_channels_;

    // Hardcoded state machinery for joining an open network named "Vermont"
    struct StateOfVermont {
        std::thread thr;
        std::mutex lock;

        bool ready() const {
            return (state == State::kDead) || (frame_body.get() != nullptr);
        }
        bool dead() const {
            return state == State::kDead;
        }

        enum class State {
            kProbing,
            kAuthenticating,
            kAssociating,
            kJoined,
            kError,
            kDead,
        };

        State state = State::kProbing;
        std::condition_variable cv;
        uint8_t retries = 0;
        FrameControl next_fc;
        MgmtFrame next_frame;
        std::unique_ptr<uint8_t[]> frame_body;
        size_t frame_body_len = 0;

        uint16_t channel = 0;
        uint8_t bssid[6];
        uint16_t aid;
        mx_time_t last_beacon = 0;
    };
    StateOfVermont sov_;

    StateOfVermont::State FindNetwork(uint16_t channel_num);
    StateOfVermont::State AuthToNetwork();
    StateOfVermont::State AssocToNetwork();
    StateOfVermont::State WaitForStateChange();
};

}  // namespace wlan
