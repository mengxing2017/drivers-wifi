// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

namespace rt5370 {

template<uint16_t A>
class Register {
  public:
    static constexpr uint16_t addr() { return A; }

    void clear() { val_ = 0; }
    void set_val(uint32_t val) { val_ = val; }
    uint32_t* mut_val() { return &val_; }

    constexpr uint32_t val() const { return val_; }

    constexpr uint32_t get_bits(unsigned int offset, size_t len) const {
        return (val_ & mask(offset, len)) >> offset;
    }

    constexpr void set_bits(unsigned int offset, size_t len, uint32_t value) {
        uint32_t cleared = val_ & ~mask(offset, len);
        val_ = cleared | ((value << offset) & mask(offset, len));
    }

  private:
    constexpr static uint32_t mask(unsigned int offset, size_t len) {
        return ((1 << len) - 1) << offset;
    }

    uint32_t val_ = 0;
};

#define REG_FIELD(name, offset, len) \
    constexpr void set_##name(uint32_t val) { this->set_bits(offset, len, val); } \
    constexpr uint32_t name() { return this->get_bits(offset, len); }

}  // namespace rt5370
