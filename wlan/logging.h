// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <cstdio>

// TODO(tkilbourn): use standard logging infrastructure
namespace wlan {
constexpr int kLogError = 0;
constexpr int kLogWarning = 1;
constexpr int kLogInfo = 2;
constexpr int kLogDebug = 3;

// Set this to tune log output
constexpr int kLogLevel = kLogInfo;

#define wlogf(level, level_prefix, args...) do { \
    if (kLogLevel >= level) { \
        std::printf("wlan: " level_prefix args); \
    } \
} while (false)

#define errorf(args...) wlogf(kLogError, "[E] ", args)
#define warnf(args...) wlogf(kLogWarning, "[W] ", args)
#define infof(args...) wlogf(kLogInfo, "[I] ", args)
#define debugf(args...) wlogf(kLogDebug, "[D] ", args)
#define debugfn() debugf("%s\n", __PRETTY_FUNCTION__)
}  // namespace wlan
