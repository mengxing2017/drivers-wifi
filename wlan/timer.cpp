// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "timer.h"

#include <utility>

namespace wlan {

Timer::Timer(uint64_t id) : id_(id) {}

Timer::~Timer() {}

mx_status_t Timer::StartTimer(mx_time_t deadline) {
    deadline_ = deadline;
    return StartTimerImpl(deadline);
}

mx_status_t Timer::CancelTimer() {
    deadline_ = 0u;
    return CancelTimerImpl();
}

SystemTimer::SystemTimer(uint64_t id, mx::timer timer)
  : Timer(id), timer_(std::move(timer)) {}

mx_status_t SystemTimer::StartTimerImpl(mx_time_t deadline) {
    return timer_.start(deadline, 0, 0);
}

mx_status_t SystemTimer::CancelTimerImpl() {
    return timer_.cancel();
}

mx_status_t TestTimer::StartTimerImpl(mx_time_t duration) {
    return MX_OK;
}

mx_status_t TestTimer::CancelTimerImpl() {
    return MX_OK;
}

}  // namespace wlan
