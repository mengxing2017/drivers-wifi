// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/iotxn.h>
#include <ddk/protocol/ethernet.h>
#include <ddk/protocol/wlan.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace rt5370 {

template <uint16_t A> class Register;
template <uint8_t A> class BbpRegister;
template <uint8_t A> class RfcsrRegister;
template <uint16_t A> class EepromField;

class Device {
  public:
    Device(mx_driver_t* driver, mx_device_t* device, uint8_t bulk_in,
           std::vector<uint8_t>&& bulk_out);
    ~Device();

    mx_status_t Bind();

  private:
    struct Channel {
        Channel(int channel, int hw_index, uint32_t N, uint32_t R, uint32_t K) :
            channel(channel), hw_index(hw_index), N(N), R(R), K(K) {}

        int channel;
        int hw_index;
        uint32_t N;
        uint32_t R;
        uint32_t K;

        uint16_t max_power = 0;
        uint16_t default_power1 = 0;
        uint16_t default_power2 = 0;
        uint16_t default_power3 = 0;
    };

    mx_status_t ReadRegister(uint16_t offset, uint32_t* value);
    template <uint16_t A> mx_status_t ReadRegister(Register<A>* reg);
    mx_status_t WriteRegister(uint16_t offset, uint32_t value);
    template <uint16_t A> mx_status_t WriteRegister(const Register<A>& reg);

    mx_status_t ReadEeprom();
    mx_status_t ReadEepromField(uint16_t addr, uint16_t* value);
    template <uint16_t A> mx_status_t ReadEepromField(EepromField<A>* field);
    template <uint16_t A> mx_status_t WriteEepromField(const EepromField<A>& field);
    mx_status_t ValidateEeprom();

    mx_status_t ReadBbp(uint8_t addr, uint8_t* val);
    template <uint8_t A> mx_status_t ReadBbp(BbpRegister<A>* reg);
    mx_status_t WriteBbp(uint8_t addr, uint8_t val);
    template <uint8_t A> mx_status_t WriteBbp(const BbpRegister<A>& reg);
    mx_status_t WaitForBbp();

    mx_status_t ReadRfcsr(uint8_t addr, uint8_t* val);
    template <uint8_t A> mx_status_t ReadRfcsr(RfcsrRegister<A>* reg);
    mx_status_t WriteRfcsr(uint8_t addr, uint8_t val);
    template <uint8_t A> mx_status_t WriteRfcsr(const RfcsrRegister<A>& reg);

    mx_status_t LoadFirmware();
    mx_status_t EnableRadio();
    mx_status_t InitRegisters();
    mx_status_t InitBbp();
    mx_status_t InitRfcsr();

    mx_status_t McuCommand(uint8_t command, uint8_t token, uint8_t arg0, uint8_t arg1);

    mx_status_t DetectAutoRun(bool* autorun);
    mx_status_t DisableWpdma();
    mx_status_t WaitForMacCsr();
    mx_status_t SetRxFilter();
    mx_status_t NormalModeSetup();
    mx_status_t StartQueues();
    mx_status_t StopRxQueue();
    mx_status_t SetupInterface();

    mx_status_t ConfigureChannel(const Channel& channel);
    mx_status_t ConfigureTxPower(const Channel& channel);

    template <typename R>
    mx_status_t BusyWait(R* reg, std::function<bool()> pred,
            std::chrono::microseconds delay = kDefaultBusyWait);

    void HandleRxComplete(iotxn_t* request);
    void HandleTxComplete(iotxn_t* request);

    // DDK API
    void Unbind();
    mx_status_t Release();

    // Wlan Mac API
    mx_status_t WlanQuery(uint32_t options, ethmac_info_t* info);
    mx_status_t WlanStart(wlanmac_ifc_t* ifc, void* cookie);
    void WlanStop();
    void WlanTx(uint32_t options, void* data, size_t len);
    mx_status_t WlanSetChannel(uint32_t options, wlan_channel_t* chan);

    static void DdkUnbind(mx_device_t* device);
    static mx_status_t DdkRelease(mx_device_t* device);

    static mx_status_t DdkWlanQuery(mx_device_t* device, uint32_t options, ethmac_info_t* info);
    static mx_status_t DdkWlanStart(mx_device_t* device, wlanmac_ifc_t* ifc, void* cookie);
    static void DdkWlanStop(mx_device_t* device);
    static void DdkWlanTx(mx_device_t* device, uint32_t options, void* data, size_t length);
    static mx_status_t DdkWlanSetChannel(mx_device_t* device, uint32_t options,
            wlan_channel_t* chan);

    static void ReadIotxnComplete(iotxn_t* request, void* cookie);
    static void WriteIotxnComplete(iotxn_t* request, void* cookie);

    mx_driver_t* driver_;
    mx_device_t* usb_device_;
    mx_device_t device_;
    mx_protocol_device_t device_ops_;

    wlanmac_ifc_t* wlanmac_ifc_ = nullptr;
    void* wlanmac_cookie_ = nullptr;
    wlanmac_protocol_t wlanmac_ops_;

    uint8_t rx_endpt_ = 0;
    //uint8_t beacon_endpt_ = 0;
    std::vector<uint8_t> tx_endpts_;

    constexpr static size_t kEepromSize = 0x0100;
    std::array<uint16_t, kEepromSize> eeprom_ = {};

    constexpr static std::chrono::microseconds kDefaultBusyWait = std::chrono::microseconds(100);

    uint16_t rt_type_ = 0;
    uint16_t rt_rev_ = 0;
    uint16_t rf_type_ = 0;

    uint8_t mac_addr_[ETH_MAC_SIZE];
    std::unordered_map<int, Channel> channels_;
    uint16_t lna_gain_ = 0;

    std::atomic_uint16_t seqno_;
    std::mutex lock_;
    std::vector<iotxn_t*> free_write_reqs_;
};

}  // namespace rt5370
