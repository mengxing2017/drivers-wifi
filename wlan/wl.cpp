// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <magenta/device/wlan.h>
#include <mx/channel.h>
#include <mx/socket.h>

#include <fcntl.h>

#include <cerrno>
#include <cstdlib>
#include <iomanip>
#include <iostream>

void print_scan_report(const wlan_scan_report& rpt) {
    std::ios state(nullptr);
    state.copyfmt(std::cout);

    std::cout << "SCAN REPORT" << std::endl;
    std::cout << "  bssid: ";
    std::cout << std::hex << std::setw(2) << std::setfill('0');
    std::cout << (uint32_t)rpt.bssid[0] << ":" << (uint32_t)rpt.bssid[1] << ":" << (uint32_t)rpt.bssid[2]
        << ":" << (uint32_t)rpt.bssid[3] << ":" << (uint32_t)rpt.bssid[4] << ":" << (uint32_t)rpt.bssid[5] << std::endl;
    std::cout.copyfmt(state);
    std::cout << "  bss type: ";
    switch (rpt.bss_type) {
    case WLAN_BSSTYPE_INFRASTRUCTURE:
        std::cout << "infrastructure";
        break;
    case WLAN_BSSTYPE_INDEPENDENT:
        std::cout << "ad-hoc";
        break;
    case WLAN_BSSTYPE_UNKNOWN:
        std::cout << "unknown (" << rpt.bss_type << ")";
        break;
    default:
        std::cout << "invalid (" << rpt.bss_type << ")";
        break;
    }
    std::cout << std::endl;
    std::cout << "  timestamp: " << rpt.timestamp << std::endl;
    std::cout << "  beacon period: " << rpt.beacon_period << std::endl;
    std::cout << std::hex << std::setw(4) << std::setfill('0') << std::showbase;
    std::cout << "  capabilities: " << rpt.capabilities << std::dec << std::endl;
    std::cout.copyfmt(state);
    std::string ssid(rpt.ssid, rpt.ssid + rpt.ssid_len);
    std::cout << "  ssid: " << ssid << std::endl;
    std::cout << "  supported rates:" << std::endl;
    for (auto r : rpt.supported_rates) {
        if (r == 0) break;
        std::cout << "    " << (500.0 * (r & 0x3f)) / 1000.0 << " Mbps" << std::endl;
    }
    std::cout << "===========" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <wlan device>" << std::endl;
        return 0;
    }

    int fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        std::cerr << "could not open " << argv[1] << ": " << errno << std::endl;
        return -1;
    }

    wlan_start_scan_args args = {};
    args.scan_type = WLAN_SCANTYPE_PASSIVE;
    mx_handle_t h;
    auto status = ioctl_wlan_start_scan(fd, &args, &h);
    if (status < 0) {
        std::cerr << "could not start scan: " << status << std::endl;
        return -1;
    }

    mx::channel ch(h);

    unsigned long num_reports = argc < 3 ? 5 : std::strtoul(argv[2], NULL, 10);
    for (unsigned long i = 0; i < num_reports; i++) {
        mx_signals_t pending = 0;
        status = ch.wait_one(MX_CHANNEL_READABLE | MX_CHANNEL_PEER_CLOSED, MX_SEC(5), &pending);
        if (status != NO_ERROR) {
            std::cerr << "could not wait for scan channel: " << status << std::endl;
            break;
        }
        wlan_scan_report rpt;
        uint32_t actual = 0;
        status = ch.read(0, &rpt, sizeof(rpt), &actual, NULL, 0, NULL);
        if (status != NO_ERROR) {
            std::cerr << "could not read scan channel: " << status << std::endl;
            break;
        }
        print_scan_report(rpt);
    }

    return 0;
}
