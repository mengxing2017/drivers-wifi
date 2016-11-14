// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

namespace rt5370 {

enum UsbVendorRequest : uint8_t {
    kDeviceMode = 1,
    kSingleWrite = 2,
    kSingleRead = 3,
    kMultiWrite = 6,
    kMultiRead = 7,
    kEepromWrite = 8,
    kEepromRead = 9,
    kLedControl = 10,
    kRxControl = 12,
};

enum UsbModeOffset : uint8_t {
    kReset = 1,
    kUnplug = 2,
    kFunction = 3,
    kTest = 4,
    kAutorun = 17,
};

constexpr uint16_t MAC_CSR0 = 0x1000;

constexpr uint16_t EFUSE_CTRL = 0x0580;

constexpr uint16_t EFUSE_DATA0 = 0x0590;
constexpr uint16_t EFUSE_DATA1 = 0x0594;
constexpr uint16_t EFUSE_DATA2 = 0x0598;
constexpr uint16_t EFUSE_DATA3 = 0x059c;

}  // namespace rt5370
