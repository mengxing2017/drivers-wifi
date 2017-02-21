// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include "bitfield.h"

#include <magenta/compiler.h>

#include <cstdint>

namespace wlan {

// IEEE Std 802.11-2016, 9.2,4,1.1
class FrameControl : public BitField<uint16_t> {
  public:
    constexpr explicit FrameControl(uint16_t fc) : BitField(fc) {}
    constexpr FrameControl() = default;

    BIT_FIELD(protocol_version, 0, 2);
    BIT_FIELD(type, 2, 2);
    BIT_FIELD(subtype, 4, 4);
    BIT_FIELD(to_ds, 8, 1);
    BIT_FIELD(from_ds, 9, 1);
    BIT_FIELD(more_frag, 10, 1);
    BIT_FIELD(retry, 11, 1);
    BIT_FIELD(pwr_mgmt, 12, 1);
    BIT_FIELD(more_data, 13, 1);
    BIT_FIELD(protected_frame, 14, 1);
    BIT_FIELD(htc_order, 15, 1);

    // For type == Control and subtype == Control Frame Extension
    BIT_FIELD(cf_extension, 8, 4);
};

// IEEE Std 802.11-2016, 9.2.4.1.3
enum FrameType : uint8_t {
    kManagement = 0x00,
    kControl = 0x01,
    kData = 0x02,
    kExtension = 0x03,
};

enum ManagementSubtype : uint8_t {
    kAssociationRequest = 0x00,
    kAssociationResponse = 0x01,
    kReassociationRequest = 0x02,
    kReassociationResponse = 0x03,
    kProbeRequest = 0x04,
    kProbeResponse = 0x05,
    kTimingAdvertisement = 0x06,
    kBeacon = 0x08,
    kAtim = 0x09,
    kDisassociation = 0x0a,
    kAuthentication = 0x0b,
    kDeauthentication = 0x0c,
    kAction = 0x0d,
    kActionNoAck = 0x0e,
};

enum ControlSubtype : uint8_t {
    kBeamformingReportPoll = 0x04,
    kVhtNdpAnnouncement = 0x05,
    kControlFrameExtension = 0x06,
    kControlWrapper = 0x07,
    kBlockAckRequest = 0x08,
    kBlockAck = 0x09,
    kPsPoll = 0x0a,
    kRts = 0x0b,
    kCts = 0x0c,
    kAck = 0x0d,
    kCfEnd = 0x0e,
    kCfEndCfAck = 0x0f,
};

// The subtypes for Data frames are essentially composed from the following
// bitmask.
enum DataSubtype : uint8_t {
    kCfAck =  (1 << 0),
    kCfPoll = (1 << 1),
    kNull =   (1 << 2),
    kQos =    (1 << 3),
};


class SequenceControl : public BitField<uint16_t> {
  public:
    BIT_FIELD(frag, 0, 4);
    BIT_FIELD(seq, 4, 12);
};

class HtControl : public BitField<uint32_t> {
  public:
    BIT_FIELD(vht, 0, 1);
    BIT_FIELD(middle, 1, 29);
    BIT_FIELD(ac_constraint, 30, 1);
    BIT_FIELD(rdg_more_ppdu, 31, 1);
};

struct MgmtFrame {
    uint16_t duration;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
    SequenceControl sc;
    HtControl ht;
    uint8_t* body;
    size_t body_len;
};

struct Element {
    uint8_t id;
    uint8_t len;
    uint8_t data[0];
}  __PACKED;

enum ElementId : uint8_t {
    kSsid = 0,
    kSuppRates = 1,
    kDsssParamSet = 3,
    kCfParamSet = 4,
    kTim = 5,
    kIbssParamSet = 6,
    kCountry = 7,
    kRequest = 10,
    kBssLoad = 11,
    kEdcaParamSet = 12,
    kTspec = 13,
    kTclas = 14,
    kSchedule = 15,
};

class CapabilityInfo : public BitField<uint16_t> {
  public:
    BIT_FIELD(ess, 0, 1);
    BIT_FIELD(ibss, 1, 1);
    BIT_FIELD(cf_pollable, 2, 1);
    BIT_FIELD(cf_poll_req, 3, 1);
    BIT_FIELD(privacy, 4, 1);
    BIT_FIELD(short_preamble, 5, 1);
    BIT_FIELD(spectrum_mgmt, 8, 1);
    BIT_FIELD(qos, 9, 1);
    BIT_FIELD(short_slot_time, 10, 1);
    BIT_FIELD(apsd, 11, 1);
    BIT_FIELD(radio_msmt, 12, 1);
    BIT_FIELD(delayed_block_ack, 14, 1);
    BIT_FIELD(immediate_block_ack, 15, 1);
};

struct Beacon {
    uint8_t bssid[6];

    uint64_t timestamp;
    uint16_t beacon_interval;
    CapabilityInfo cap;
    uint8_t ssid[32];
    size_t ssid_len;
    uint8_t supp_rates[8];
    // etc.
};

}  // namespace wlan
