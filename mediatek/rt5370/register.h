// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

namespace rt5370 {

template<typename T>
class BitField {
  public:
    constexpr explicit BitField(T val) : val_(val) {}
    constexpr BitField() = default;

    void clear() { val_ = 0; }
    void set_val(T val) { val_ = val; }
    T* mut_val() { return &val_; }

    constexpr T val() const { return val_; }

    constexpr T get_bits(unsigned int offset, size_t len) const {
        return (val_ & mask(offset, len)) >> offset;
    }

    constexpr void set_bits(unsigned int offset, size_t len, T value) {
        T cleared = val_ & ~mask(offset, len);
        val_ = cleared | ((value << offset) & mask(offset, len));
    }

  private:
    constexpr static T mask(unsigned int offset, size_t len) {
        return ((1 << len) - 1) << offset;
    }

    T val_ = 0;
};

template<uint16_t A>
class Register : public BitField<uint32_t> {
  public:
    static constexpr uint16_t addr() { return A; }
    constexpr Register() = default;
};

template<uint16_t A>
class EepromField : public BitField<uint16_t> {
  public:
    static constexpr uint16_t addr() { return A; }
    constexpr EepromField() = default;
};

template<uint8_t A>
class BbpRegister : public BitField<uint8_t> {
  public:
    static constexpr uint8_t addr() { return A; }
    constexpr explicit BbpRegister(uint8_t val) : BitField(val) {}
    constexpr BbpRegister() = default;
};

#define BIT_FIELD(name, offset, len) \
    constexpr void set_##name(uint32_t val) { this->set_bits(offset, len, val); } \
    constexpr uint32_t name() { return this->get_bits(offset, len); }

}  // namespace rt5370
