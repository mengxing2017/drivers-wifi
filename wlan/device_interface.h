// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include "mac_frame.h"

#include <ddk/protocol/wlan.h>
#include <zircon/types.h>
#include <fbl/ref_counted.h>
#include <fbl/ref_ptr.h>
#include <fbl/unique_ptr.h>

#include <cstdint>
#include <cstring>

namespace wlan {

class Packet;
class Timer;

class DeviceAddress {
  public:
    static constexpr size_t kSize = 6;

    DeviceAddress() = default;
    explicit DeviceAddress(const uint8_t addr[kSize]) {
        std::memcpy(addr_, addr, DeviceAddress::kSize);
    }

    bool operator==(const DeviceAddress& other) const {
        return std::memcmp(addr_, other.addr_, DeviceAddress::kSize) == 0;
    }
    bool operator!=(const DeviceAddress& other) const { return !(*this == other); }

    uint64_t to_u64() const;
    const uint8_t* data() const { return addr_; }
    void set_data(const uint8_t addr[kSize]) {
        std::memcpy(addr_, addr, DeviceAddress::kSize);
    }

  private:
    uint8_t addr_[kSize] = {};
};

inline bool operator==(const DeviceAddress& a, const uint8_t b[DeviceAddress::kSize]) {
    return a == DeviceAddress(b);
}
inline bool operator!=(const DeviceAddress& a, const uint8_t b[DeviceAddress::kSize]) {
    return !(a == b);
}
inline bool operator==(const uint8_t a[DeviceAddress::kSize], const DeviceAddress& b) {
    return b == a;
}
inline bool operator!=(const uint8_t a[DeviceAddress::kSize], const DeviceAddress& b) {
    return !(a == b);
}

// DeviceState represents the common runtime state of a device needed for interacting with external
// systems.
class DeviceState : public fbl::RefCounted<DeviceState> {
  public:
    const DeviceAddress& address() const {
        return addr_;
    }
    void set_address(const DeviceAddress& addr) {
        addr_ = addr;
    }

    wlan_channel_t channel() const {
        return chan_;
    }
    void set_channel(const wlan_channel_t& chan) {
        chan_ = chan;
    }

    bool online() {
        return online_;
    }
    void set_online(bool online) {
        online_ = online;
    }

    uint16_t next_seq() {
        return seq_no_++ & kMaxSequenceNumber;
    }

  private:
    DeviceAddress addr_;
    wlan_channel_t chan_ = { 0 };
    uint16_t seq_no_ = 0;
    bool online_ = false;
};

// DeviceInterface represents the actions that may interact with external systems.
class DeviceInterface {
  public:
    virtual ~DeviceInterface() {}

    virtual zx_status_t GetTimer(uint64_t id, fbl::unique_ptr<Timer>* timer) = 0;

    virtual zx_status_t SendEthernet(fbl::unique_ptr<Packet> packet) = 0;
    virtual zx_status_t SendWlan(fbl::unique_ptr<Packet> packet) = 0;
    virtual zx_status_t SendService(fbl::unique_ptr<Packet> packet) = 0;

    virtual zx_status_t SetChannel(wlan_channel_t chan) = 0;
    virtual zx_status_t SetStatus(uint32_t status) = 0;

    virtual fbl::RefPtr<DeviceState> GetState() = 0;
};

}  // namespace wlan
