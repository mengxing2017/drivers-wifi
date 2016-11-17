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
    kFirmware = 8,
    kAutorun = 17,
};

constexpr uint16_t RT5390 = 0x5390;
constexpr uint16_t FW_IMAGE_BASE = 0x3000;

// Registers

constexpr uint16_t WPDMA_GLO_CFG = 0x0208;
constexpr int WPDMA_GLO_CFG_TX_DMA_EN = 0;
constexpr int WPDMA_GLO_CFG_TX_DMA_BUSY = 1;
constexpr int WPDMA_GLO_CFG_RX_DMA_EN = 2;
constexpr int WPDMA_GLO_CFG_RX_DMA_BUSY = 3;
constexpr int WPDMA_GLO_CFG_TX_WB_DDONE = 6;

constexpr uint16_t SYS_CTRL = 0x0400;
constexpr int SYS_CTRL_MCU_READY = 7;

constexpr uint16_t HOST_CMD = 0x0404;

constexpr uint16_t EFUSE_CTRL = 0x0580;
constexpr int EFUSE_CTRL_KICK = 30;
constexpr int EFUSE_CTRL_ADDR = 16;
constexpr int EFUSE_CTRL_ADDR_WIDTH = 10;
constexpr int EFUSE_CTRL_MODE = 6;

constexpr uint16_t EFUSE_DATA0 = 0x0590;
constexpr uint16_t EFUSE_DATA1 = 0x0594;
constexpr uint16_t EFUSE_DATA2 = 0x0598;
constexpr uint16_t EFUSE_DATA3 = 0x059c;

constexpr uint16_t MAC_CSR0 = 0x1000;

constexpr uint16_t AUTO_WAKEUP_CFG = 0x1208;

// EEPROM offsets
constexpr uint16_t EEPROM_CHIP_ID = 0x0000;
constexpr uint16_t EEPROM_VERSION = 0x0001;
constexpr uint16_t EEPROM_MAC_ADDR_0 = 0x0002;
constexpr uint16_t EEPROM_MAC_ADDR_1 = 0x0003;
constexpr uint16_t EEPROM_MAC_ADDR_2 = 0x0004;
constexpr uint16_t EEPROM_NIC_CONF0 = 0x001a;
constexpr uint16_t EEPROM_NIC_CONF1 = 0x001b;
constexpr uint16_t EEPROM_FREQ = 0x001d;
constexpr uint16_t EEPROM_NIC_CONF2 = 0x0021;
constexpr uint16_t EEPROM_LNA = 0x0022;
constexpr uint16_t EEPROM_RSSI_BG = 0x0023;
constexpr uint16_t EEPROM_RSSI_BG2 = 0x0024;
constexpr uint16_t EEPROM_RSSI_A = 0x0025;
constexpr uint16_t EEPROM_RSSI_A2 = 0x0026;

constexpr int EEPROM_FREQ_OFFSET = 0;
constexpr int EEPROM_FREQ_WIDTH = 8;

constexpr int EEPROM_LNA_A0_OFFSET = 8;
constexpr int EEPROM_LNA_A0_WIDTH = 8;

constexpr int EEPROM_RSSI_BG_0_OFFSET = 0;
constexpr int EEPROM_RSSI_BG_0_WIDTH = 8;
constexpr int EEPROM_RSSI_BG_1_OFFSET = 8;
constexpr int EEPROM_RSSI_BG_1_WIDTH = 8;

constexpr int EEPROM_RSSI_BG2_OFFSET = 0;
constexpr int EEPROM_RSSI_BG2_WIDTH = 8;
constexpr int EEPROM_RSSI_BG2_LNA_A1_OFFSET = 8;
constexpr int EEPROM_RSSI_BG2_LNA_A1_WIDTH = 8;

constexpr int EEPROM_NIC_CONF1_HW_RADIO   = 0;  // 0x0001
constexpr int EEPROM_NIC_CONF1_BT_COEXIST = 14; // 0x4000

// Host to MCU communication
constexpr uint16_t H2M_MAILBOX_CSR = 0x7010;
constexpr int H2M_MAILBOX_CSR_ARG0_OFFSET = 0;
constexpr int H2M_MAILBOX_CSR_ARG1_OFFSET = 8;
constexpr int H2M_MAILBOX_CSR_CMD_TOKEN_OFFSET = 16;
constexpr int H2M_MAILBOX_CSR_OWNER_OFFSET = 24;
// All four fields have width 8
constexpr int H2M_MAILBOX_CSR_WIDTH = 8;

constexpr uint16_t H2M_MAILBOX_CID = 0x7014;
constexpr uint16_t H2M_MAILBOX_STATUS = 0x701c;
constexpr uint16_t H2M_INT_SRC = 0x7024;
constexpr uint16_t H2M_BBP_AGENT = 0x7028;

// MCU commands
constexpr uint8_t MCU_BOOT_SIGNAL = 0x72;

}  // namespace rt5370
