// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "wlan_device.h"

#include <mx/time.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cinttypes>
#include <functional>

#define DEBUG_BEACON 0

namespace wlan {

Device::Device(mx_driver_t* driver, mx_device_t* device, wlanmac_protocol_t* wlanmac_ops)
  : driver_(driver), wlanmac_device_(device), wlanmac_ops_(wlanmac_ops),
    seqno_(1) {
    std::memset(&driver_, 0, sizeof(driver_));
}

Device::~Device() {}

mx_protocol_device_t Device::device_ops_ = {
    .get_protocol = nullptr,
    .open = nullptr,
    .openat = nullptr,
    .close = nullptr,
    .unbind = [](mx_device_t* dev) {
        static_cast<Device*>(dev->ctx)->Unbind();
    },
    .release = [](mx_device_t* dev) {
        return static_cast<Device*>(dev->ctx)->Release();
    },
    .read = nullptr,
    .write = nullptr,
    .iotxn_queue = nullptr,
    .get_size = nullptr,
    .ioctl = &Device::DdkIoctl,
    .suspend = nullptr,
    .resume = nullptr,
};

ethmac_protocol_t Device::ethmac_ops_ = {
    .query = [](mx_device_t* dev, uint32_t options, ethmac_info_t* info) {
        return static_cast<Device*>(dev->ctx)->Query(options, info);
    },
    .start = [](mx_device_t* dev, ethmac_ifc_t* ifc, void* cookie) {
        return static_cast<Device*>(dev->ctx)->Start(ifc, cookie);
    },
    .stop = [](mx_device_t* dev) {
        static_cast<Device*>(dev->ctx)->Stop();
    },
    .send = [](mx_device_t* dev, uint32_t options, void* data, size_t length) {
        static_cast<Device*>(dev->ctx)->Send(options, data, length);
    },
    .queue_tx = nullptr,
    .queue_rx = nullptr,
};

wlanmac_ifc_t Device::wlanmac_ifc_ = {
    .status = [](void* cookie, uint32_t status) {
        static_cast<Device*>(cookie)->Status(status);
    },
    .recv = [](void* cookie, void* data, size_t length, uint32_t flags) {
        static_cast<Device*>(cookie)->Recv(data, length, flags);
    },
};

mx_status_t Device::Bind() {
    device_init(&device_, driver_, "wlan", &device_ops_);

    device_.ctx = this;
    device_.protocol_id = MX_PROTOCOL_ETHERMAC;
    device_.protocol_ops = &ethmac_ops_;
    auto status = device_add(&device_, wlanmac_device_);
    if (status != NO_ERROR) {
        std::printf("wlan could not add device err=%d\n", status);
    } else {
        std::printf("wlan device added\n");
    }

    return status;
}

void Device::Unbind() {
    std::printf("wlan::Device::Unbind\n");
    device_remove(&device_);
}

mx_status_t Device::Release() {
    std::printf("wlan::Device::Release\n");
    delete this;
    return NO_ERROR;
}

mx_status_t Device::Query(uint32_t options, ethmac_info_t* info) {
    std::printf("wlan query\n");
    auto status = wlanmac_ops_->query(wlanmac_device_, options, info);
    if (status != NO_ERROR) {
        return status;
    }
    std::memcpy(mac_addr_, info->mac, ETH_MAC_SIZE);
    // Make sure this device is reported as a wlan device
    info->features |= ETHMAC_FEATURE_WLAN;
    return NO_ERROR;
}

mx_status_t Device::Start(ethmac_ifc_t* ifc, void* cookie) {
    std::printf("wlan start\n");
    ethmac_ifc_ = ifc;
    ethmac_cookie_ = cookie;
    auto status = wlanmac_ops_->start(wlanmac_device_, &wlanmac_ifc_, this);
    if (status != NO_ERROR) {
        return status;
    }

    sov_.thr = std::thread(&Device::JoinVermont, this);
    sov_.thr.detach();

    return status;
}

void Device::Stop() {
    wlanmac_ops_->stop(wlanmac_device_);
}

void Device::Status(uint32_t status) {
    std::printf("wlan::Device::Status = %u\n", status);
}

void Device::Recv(void* data, size_t length, uint32_t flags) {
    if (length < 2) return;
    auto bytes = reinterpret_cast<uint8_t*>(data);
    FrameControl fc(*reinterpret_cast<uint16_t*>(bytes));
    switch (fc.type()) {
        case kManagement:
            HandleMgmtFrame(fc, bytes + 2, length - 2, flags);
            break;
        case kData: {
            //std::printf("wlan recv data frame len=%zu\n", length);
            if (fc.subtype() != 0) {
                //std::printf("wlan unsupported data subtype %02x\n", fc.subtype());
                break;
            }
            if (length < 24) {
                std::printf("wlan short packet\n");
                break;
            }
            //uint8_t* body = static_cast<uint8_t*>(data);
            //for (size_t i = 0; i < length; i++) {
            //    std::printf("%02x ", body[i]);
            //    if (i % 16 == 15) std::printf("\n");
            //}
            //std::printf("\n");
            uint8_t* d = static_cast<uint8_t*>(data);
            std::unique_ptr<uint8_t[]> pkt(new uint8_t[length - 32 + 14]);
            std::memcpy(pkt.get(), d + 4, ETH_MAC_SIZE);
            std::memcpy(pkt.get() + 6, d + 16, ETH_MAC_SIZE);
            std::memcpy(pkt.get() + 12, d + 30, 2);
            std::memcpy(pkt.get() + 14, d + 32, length - 32);
            ethmac_ifc_->recv(ethmac_cookie_, pkt.get(), length - 32 + 14, 0);
            break;
        }
        default:
            break;
    }
}

void Device::Send(uint32_t options, void* data, size_t length) {
    if (sov_.state != StateOfVermont::State::kJoined) return;
    uint8_t* pkt = static_cast<uint8_t*>(data);

    // TODO: prepare wlan mac header based on eth headers
    // TODO: make sure we're associated and ready to send data
    std::unique_ptr<uint8_t[]> buf(new uint8_t[24 + 8 + length - 14]);
    std::memset(buf.get(), 0, 24 + 8 + length - 14);
    FrameControl fc;
    fc.set_type(kData);
    fc.set_to_ds(1);
    *(uint16_t*)(buf.get()) = fc.val();

    std::memcpy(buf.get() + 4, sov_.bssid, ETH_MAC_SIZE);
    std::memcpy(buf.get() + 10, pkt + 6, ETH_MAC_SIZE);
    // Destination address is at the beginning of the mac frame
    std::memcpy(buf.get() + 16, pkt, ETH_MAC_SIZE);

    SequenceControl sc;
    sc.set_seq(seqno_++);

    *(uint16_t*)(buf.get() + 22) = sc.val();

    // LLC header (hack)
    buf[24] = 0xaa;
    buf[25] = 0xaa;
    buf[26] = 0x03;
    buf[27] = 0;
    buf[28] = 0;
    buf[29] = 0;
    buf[30] = pkt[12];
    buf[31] = pkt[13];

    std::memcpy(buf.get() + 32, pkt + 14, length - 14);

    //std::printf("wlan send data frame len=%zu\n", length + 24);
    //for (size_t i = 0; i < length + 24; i++) {
    //    std::printf("%02x ", buf[i]);
    //    if (i % 16 == 15) std::printf("\n");
    //}
    //std::printf("\n");

    wlanmac_ops_->tx(wlanmac_device_, options, buf.get(), length + 24);
}

ssize_t Device::StartScan(const wlan_start_scan_args* args, mx_handle_t* out_channel) {
    mx::channel chan[2];
    auto status = mx::channel::create(0, &chan[0], &chan[1]);
    if (status < 0) {
        return status;
    }

    std::lock_guard<std::mutex> guard(lock_);
    // TODO: the idea of having several concurrent scans going on is flawed.
    // This will need to be run through a state machine, and when there's a scan
    // ongoing, the ioctl needs to return ESHOULDWAIT or something.
    scan_channels_.insert(std::move(chan[0]));
    *out_channel = chan[1].release();

    std::printf("wlan scan on %u channels:", args->num_channels);
    for (int i = 0; i < args->num_channels; i++) {
        std::printf(" %u", args->channels[i]);
    }
    std::printf("\n");
    std::printf("wlan scanning for up to %u time-units\n", args->max_channel_time);
    // TODO: actual scanning code goes here. for now just passive scan on the
    // current channel.
    return sizeof(mx_handle_t);
}

void Device::JoinVermont() {
    auto status = ERR_NOT_FOUND;
    do {
        status = FindNetwork(6);
        if (status != NO_ERROR) mx::nanosleep(MX_SEC(2));
    } while (status != NO_ERROR);

    uint8_t auth_req[24 + 6];
    std::memset(auth_req, 0, sizeof(auth_req));

    FrameControl fc;
    fc.set_type(kManagement);
    fc.set_subtype(kAuthentication);
    *(uint16_t*)auth_req = fc.val();

    SequenceControl sc;

    std::memcpy(auth_req + 4, sov_.bssid, 6);
    std::memcpy(auth_req + 10, mac_addr_, 6);
    std::memcpy(auth_req + 16, sov_.bssid, 6);

    *(uint16_t*)(auth_req + 26) = 1;

    do {
        sc.set_seq(seqno_++);
        *(uint16_t*)(auth_req + 22) = sc.val();

        wlanmac_ops_->tx(wlanmac_device_, 0, auth_req, sizeof(auth_req));
        std::printf("wlan queued Authentication\n");

        std::unique_lock<std::mutex> lock(sov_.lock);
        auto ret = sov_.cv.wait_for(lock, std::chrono::seconds(1),
                [this]() { return sov_.frame_body.get() != nullptr; });
        if (!ret) continue;

        // process AuthenticationResponse
        uint16_t auth_alg = *(uint16_t*)sov_.next_frame.body;
        uint16_t auth_seq = *(uint16_t*)(sov_.next_frame.body + 2);
        uint16_t auth_status = *(uint16_t*)(sov_.next_frame.body + 4);
        if (auth_alg != 0 || auth_seq != 2 || auth_status != 0) {
            std::printf("auth failure\n");
        } else {
            sov_.state = StateOfVermont::State::kAssociating;
        }
        sov_.frame_body.release();
    } while (sov_.state == StateOfVermont::State::kAuthenticating);
        
    uint8_t assoc_req[24 + 25];
    std::memset(assoc_req, 0, sizeof(assoc_req));

    fc.set_subtype(kAssociationRequest);
    *(uint16_t*)assoc_req = fc.val();

    std::memcpy(assoc_req + 4, sov_.bssid, 6);
    std::memcpy(assoc_req + 10, mac_addr_, 6);
    std::memcpy(assoc_req + 16, sov_.bssid, 6);

    // Capabilities (2 octets) field is zero
    CapabilityInfo assoc_cap;
    assoc_cap.set_ess(1);
    //assoc_cap.set_short_slot_time(1);
    *(uint16_t*)(assoc_req + 24) = assoc_cap.val();

    // Listen interval (2 octets) field is zero (no power save)

    Element* ssid = reinterpret_cast<Element*>(assoc_req + 28);
    ssid->id = kSsid;
    ssid->len = 7;
    std::memcpy(ssid->data, "Vermont", 7);

    Element* supp_rates = reinterpret_cast<Element*>(assoc_req + 37);
    supp_rates->id = 1;
    supp_rates->len = 8;
    supp_rates->data[0] = 2;
    supp_rates->data[1] = 4;
    supp_rates->data[2] = 11;
    supp_rates->data[3] = 22;
    supp_rates->data[4] = 12;
    supp_rates->data[5] = 18;
    supp_rates->data[6] = 24;
    supp_rates->data[7] = 36;

    Element* ext_rates = reinterpret_cast<Element*>(assoc_req + 47);
    ext_rates->id = 50;
    ext_rates->len = 4;
    ext_rates->data[0] = 48;
    ext_rates->data[1] = 72;
    ext_rates->data[2] = 96;
    ext_rates->data[3] = 108;

    do {
        sc.set_seq(seqno_++);
        *(uint16_t*)(assoc_req + 22) = sc.val();

        wlanmac_ops_->tx(wlanmac_device_, 0, assoc_req, sizeof(assoc_req));
        std::printf("wlan queued Association\n");

        std::unique_lock<std::mutex> lock(sov_.lock);
        auto ret = sov_.cv.wait_for(lock, std::chrono::seconds(1),
                [this]() { return sov_.frame_body.get() != nullptr; });
        if (!ret) continue;

        // process AssociationResponse
        CapabilityInfo cap;
        cap.set_val(*(uint16_t*)sov_.next_frame.body);
        uint16_t assoc_status = *(uint16_t*)(sov_.next_frame.body + 2);
        uint16_t assoc_aid = *(uint16_t*)(sov_.next_frame.body + 4) & 0x01ff;
        if (assoc_status != 0) {
            std::printf("assoc failure %u\n", assoc_status);
        } else {
            sov_.state = StateOfVermont::State::kJoined;
            sov_.aid = assoc_aid;
        }
        sov_.frame_body.release();
        mx_nanosleep(1000000000ull);
    } while (sov_.state == StateOfVermont::State::kAssociating);
    std::printf("wlan joined Vermont aid=%u\n", sov_.aid);
}

mx_status_t Device::FindNetwork(uint16_t channel_num) {
    wlan_channel_t chan = { .channel_num = channel_num };
    auto status = wlanmac_ops_->set_channel(wlanmac_device_, 0, &chan);
    if (status != NO_ERROR) {
        std::printf("wlan could not set channel to %u\n", chan.channel_num);
        return ERR_BAD_STATE;
    }
    std::printf("wlan set channel to %u\n", chan.channel_num);

    // send ProbeRequests until we find Vermont
    uint8_t probe_req[24 + 18];
    std::memset(probe_req, 0, sizeof(probe_req));

    FrameControl fc;
    fc.set_type(kManagement);
    fc.set_subtype(kProbeRequest);
    *(uint16_t*)probe_req = fc.val();

    std::memset(probe_req + 4, 0xff, 6);
    std::memcpy(probe_req + 10, mac_addr_, 6);
    std::memset(probe_req + 16, 0xff, 6);

    // Used in the while loops
    SequenceControl sc;

    Element* supp_rates = reinterpret_cast<Element*>(probe_req + 26);
    supp_rates->id = 1;
    supp_rates->len = 8;
    supp_rates->data[0] = 2;
    supp_rates->data[1] = 4;
    supp_rates->data[2] = 11;
    supp_rates->data[3] = 22;
    supp_rates->data[4] = 12;
    supp_rates->data[5] = 18;
    supp_rates->data[6] = 24;
    supp_rates->data[7] = 36;

    Element* ext_rates = reinterpret_cast<Element*>(probe_req + 36);
    ext_rates->id = 50;
    ext_rates->len = 4;
    ext_rates->data[0] = 48;
    ext_rates->data[1] = 72;
    ext_rates->data[2] = 96;
    ext_rates->data[3] = 108;

    // Update sequence number before sending
    sc.set_seq(seqno_++);
    *(uint16_t*)(probe_req + 22) = sc.val();

    wlanmac_ops_->tx(wlanmac_device_, 0, probe_req, sizeof(probe_req));
    std::printf("wlan queued ProbeRequest\n");

    auto timeout = std::chrono::milliseconds(100);
    auto start_time = mx::time::get(MX_CLOCK_MONOTONIC);

    do {
        auto delta = mx::time::get(MX_CLOCK_MONOTONIC) - start_time;
        if (delta > MX_MSEC(100)) break;

        auto duration = timeout - std::chrono::nanoseconds(delta);
        std::unique_lock<std::mutex> lock(sov_.lock);
        auto ret = sov_.cv.wait_for(lock, duration,
            [this]() { return sov_.frame_body.get() != nullptr; });
        if (!ret) return ERR_NOT_FOUND;
        if (sov_.next_frame.body_len < sizeof(ProbeResponse)) continue;

        // process ProbeResponse
        ProbeResponse* resp = reinterpret_cast<ProbeResponse*>(sov_.next_frame.body);
        if (!resp->cap.ess()) continue;
        if (resp->cap.privacy()) continue;
        uint8_t* elms = sov_.next_frame.body + sizeof(ProbeResponse);
        while (elms < sov_.next_frame.body + sov_.next_frame.body_len) {
            Element* elem = reinterpret_cast<Element*>(elms);
            if (elem->id == kSsid && elem->len == 7) {
                if (std::memcmp(elem->data, "Vermont", 7) == 0) {
                    std::memcpy(sov_.bssid, sov_.next_frame.addr2, ETH_MAC_SIZE);
                    std::printf("wlan bssid: %02x:%02x:%02x:%02x:%02x:%02x\n",
                            sov_.bssid[0], sov_.bssid[1], sov_.bssid[2],
                            sov_.bssid[3], sov_.bssid[4], sov_.bssid[5]);
                    sov_.state = StateOfVermont::State::kAuthenticating;
                    break;
                }
            }
            elms += elem->len;
        }
        sov_.frame_body.release();
    } while (sov_.state == StateOfVermont::State::kProbing);

    if (sov_.state == StateOfVermont::State::kAuthenticating) {
        return NO_ERROR;
    }
    return ERR_NOT_FOUND;
}

mx_status_t Device::AuthToNetwork() {
    return NO_ERROR;
}

mx_status_t Device::AssocToNetwork() {
    return NO_ERROR;
}

void Device::HandleMgmtFrame(FrameControl fc, uint8_t* data, size_t length, uint32_t flags) {
    if (length < 26) {
        return;
    }

    uint8_t* head = data;
    MgmtFrame mf;
    mf.duration = *reinterpret_cast<uint16_t*>(data);
    data += sizeof(mf.duration);
    std::memcpy(mf.addr1, data, sizeof(mf.addr1));
    data += sizeof(mf.addr1);
    std::memcpy(mf.addr2, data, sizeof(mf.addr2));
    data += sizeof(mf.addr2);
    std::memcpy(mf.addr3, data, sizeof(mf.addr3));
    data += sizeof(mf.addr3);
    mf.sc.set_val(*reinterpret_cast<uint16_t*>(data));
    data += sizeof(mf.sc);
    if (fc.htc_order()) {
        mf.ht.set_val(*reinterpret_cast<uint32_t*>(data));
        data += sizeof(mf.ht);
    }
    mf.body = data;
    mf.body_len = length - (data - head);

    switch (fc.subtype()) {
        case kBeacon:
            if (mf.body_len < 16) break;
            HandleBeacon(fc, &mf, flags);
            break;
        case kProbeResponse:
        case kAuthentication:
        case kAssociationResponse:
            std::printf("wlan response subtype=%02x len=%zu\n", fc.subtype(), mf.body_len);
            for (size_t i = 0; i < mf.body_len; i++) {
                std::printf("%02x ", mf.body[i]);
                if (i % 16 == 15) std::printf("\n");
            }
            std::printf("\n");
            {
                std::lock_guard<std::mutex> guard(sov_.lock);
                sov_.next_frame = std::move(mf);
                sov_.frame_body.reset(new uint8_t[mf.body_len]);
                std::memcpy(sov_.frame_body.get(), mf.body, mf.body_len);
                sov_.next_frame.body = sov_.frame_body.get();
            }
            sov_.cv.notify_one();
            break;
        default:
            break;
    }
}

void Device::HandleBeacon(FrameControl fc, MgmtFrame* mf, uint32_t flags) {
    Beacon bcn = {};
    std::memcpy(bcn.bssid, mf->addr3, sizeof(bcn.bssid));
    uint8_t* data = mf->body;
    bcn.timestamp = *reinterpret_cast<uint64_t*>(data);
    data += 8;
    bcn.beacon_interval = *reinterpret_cast<uint16_t*>(data);
    data += 2;
    bcn.cap.set_val(*reinterpret_cast<uint16_t*>(data));
    data += 2;
    while (data < mf->body + mf->body_len) {
        switch (*data) {
            case kSsid:
                bcn.ssid_len = std::min(data[1], static_cast<uint8_t>(32));
                std::memcpy(bcn.ssid, &data[2], bcn.ssid_len);
                break;
            case kSuppRates: {
                int num_rates = std::min(data[1], static_cast<uint8_t>(8));
                for (int i = 0; i < num_rates; i++) {
                    bcn.supp_rates[i] = data[2 + i];
                }
                break;
            }
            default:
                break;
        }
        data += 2 + data[1];
    }
    SendBeacon(bcn);

#if DEBUG_BEACON
    std::printf("wlan: beacon\n");
    std::printf("  bssid: %02x:%02x:%02x:%02x:%02x:%02x\n",
            bcn.bssid[0], bcn.bssid[1], bcn.bssid[2], bcn.bssid[3], bcn.bssid[4], bcn.bssid[5]);
    std::printf("  ts: %" PRIu64 "\n", bcn.timestamp);
    std::printf("  interval: %u\n", bcn.beacon_interval);
    // hack
    bcn.ssid[31] = 0;
    std::printf("  ssid: %s\n", bcn.ssid);
    std::printf("  supported rates:\n");
    for (int i = 0; i < 8; i++) {
        if (bcn.supp_rates[i] == 0) break;
        std::printf("    %.1f Mbps\n", (500.0f * (bcn.supp_rates[i] & 0x3f)) / 1000.0f);
    }
#endif
}

void Device::SendBeacon(const Beacon& beacon) {
    std::lock_guard<std::mutex> guard(lock_);
    if (scan_channels_.empty()) return;

    wlan_scan_report rpt;
    memset(&rpt, 0, sizeof(rpt));
    std::memcpy(rpt.bssid, beacon.bssid, sizeof(rpt.bssid));
    if (beacon.cap.ess()) {
        rpt.bss_type = WLAN_BSSTYPE_INFRASTRUCTURE;
    } else if (beacon.cap.ibss()) {
        rpt.bss_type = WLAN_BSSTYPE_INDEPENDENT;
    } else {
        rpt.bss_type = WLAN_BSSTYPE_UNKNOWN;
    }
    rpt.timestamp = beacon.timestamp;
    rpt.beacon_period = beacon.beacon_interval;
    rpt.capabilities = beacon.cap.val();
    std::memcpy(rpt.ssid, beacon.ssid, sizeof(rpt.ssid));
    rpt.ssid_len = beacon.ssid_len;
    std::memcpy(rpt.supported_rates, beacon.supp_rates, sizeof(rpt.supported_rates));

    for (auto iter = scan_channels_.cbegin(); iter != scan_channels_.cend(); ) {
        auto status = iter->write(0, &rpt, sizeof(rpt), NULL, 0);
        if (status == ERR_REMOTE_CLOSED) {
            iter = scan_channels_.erase(iter);
        } else {
            ++iter;
        }
    }
}

ssize_t Device::DdkIoctl(mx_device_t* device, uint32_t op, const void* in_buf, size_t in_len,
                         void* out_buf, size_t out_len) {
    auto dev = static_cast<Device*>(device->ctx);
    switch (op) {
    case IOCTL_WLAN_START_SCAN: {
        if (in_len < sizeof(wlan_start_scan_args)) return ERR_INVALID_ARGS;
        if (out_len < sizeof(mx_handle_t)) return ERR_INVALID_ARGS;
        const wlan_start_scan_args* args = reinterpret_cast<const wlan_start_scan_args*>(in_buf);
        if (in_len < sizeof(wlan_start_scan_args) + args->num_channels * sizeof(uint16_t)) {
            return ERR_INVALID_ARGS;
        }
        mx_handle_t* h = reinterpret_cast<mx_handle_t*>(out_buf);
        return dev->StartScan(args, h);
    }
    default:
        return ERR_NOT_SUPPORTED;
    }
}

std::size_t Device::ChannelHasher::operator()(const mx::channel& ch) const {
    return std::hash<mx_handle_t>{}(ch.get());
}

}  // namespace wlan
