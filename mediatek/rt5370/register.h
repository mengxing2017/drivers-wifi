// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

namespace rt5370 {

template<typename AddrType, typename ValueType, AddrType A>
class BitField {
  public:
    static constexpr AddrType addr() { return A; }

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

template<uint16_t A>
class Register : public BitField<uint16_t, uint32_t, A> {
  public:
    constexpr Register() = default;
};

template<uint16_t A>
class EepromField : public BitField<uint16_t, uint16_t, A> {
  public:
    constexpr EepromField() = default;
};

template<uint8_t A>
class BbpRegister : public BitField<uint8_t, uint8_t, A> {
  public:
    constexpr explicit BbpRegister(uint8_t val) : BitField<uint8_t, uint8_t, A>(val) {}
    constexpr BbpRegister() = default;
};

template<uint8_t A>
class RfcsrRegister : public BitField<uint8_t, uint8_t, A> {
  public:
    constexpr explicit RfcsrRegister(uint8_t val) : BitField<uint8_t, uint8_t, A>(val) {}
    constexpr RfcsrRegister() = default;
};

#define BIT_FIELD(name, offset, len) \
    constexpr void set_##name(uint32_t val) { this->set_bits(offset, len, val); } \
    constexpr uint32_t name() { return this->get_bits(offset, len); }

}  // namespace rt5370
