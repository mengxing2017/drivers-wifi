// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <cstdlib>

namespace wlan {

template<typename ValueType>
class BitField {
  public:
    constexpr explicit BitField(ValueType val) : val_(val) {}
    constexpr BitField() = default;

    void clear() { val_ = 0; }
    void set_val(ValueType val) { val_ = val; }
    ValueType* mut_val() { return &val_; }

    constexpr ValueType val() const { return val_; }

    constexpr ValueType get_bits(unsigned int offset, size_t len) const {
        return (val_ & mask(offset, len)) >> offset;
    }

    constexpr void set_bits(unsigned int offset, size_t len, ValueType value) {
        ValueType cleared = val_ & ~mask(offset, len);
        val_ = cleared | ((value << offset) & mask(offset, len));
    }

  private:
    constexpr static ValueType mask(unsigned int offset, size_t len) {
        return ((1ull << len) - 1) << offset;
    }

    ValueType val_ = 0;
};

#define BIT_FIELD(name, offset, len) \
    constexpr void set_##name(uint32_t val) { this->set_bits(offset, len, val); } \
    constexpr uint32_t name() { return this->get_bits(offset, len); }

}  // namespace wlan
