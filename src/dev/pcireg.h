/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Nathan Binkert
 *          Miguel Serrano
 */

/* @file
 * Device register definitions for a device's PCI config space
 */

#ifndef __PCIREG_H__
#define __PCIREG_H__

#include <sys/types.h>

union PCIConfig {
    uint8_t data[64];

    struct {
        uint16_t vendor;
        uint16_t device;
        uint16_t command;
        uint16_t status;
        uint8_t revision;
        uint8_t progIF;
        uint8_t subClassCode;
        uint8_t classCode;
        uint8_t cacheLineSize;
        uint8_t latencyTimer;
        uint8_t headerType;
        uint8_t bist;
        uint32_t baseAddr[6];
        uint32_t cardbusCIS;
        uint16_t subsystemVendorID;
        uint16_t subsystemID;
        uint32_t expansionROM;
        uint32_t reserved0;
        uint32_t reserved1;
        uint8_t interruptLine;
        uint8_t interruptPin;
        uint8_t minimumGrant;
        uint8_t maximumLatency;
    };
};

// Common PCI offsets
#define PCI_VENDOR_ID           0x00    // Vendor ID                    ro
#define PCI_DEVICE_ID           0x02    // Device ID                    ro
#define PCI_COMMAND             0x04    // Command                      rw
#define PCI_STATUS              0x06    // Status                       rw
#define PCI_REVISION_ID         0x08    // Revision ID                  ro
#define PCI_CLASS_CODE          0x09    // Class Code                   ro
#define PCI_SUB_CLASS_CODE      0x0A    // Sub Class Code               ro
#define PCI_BASE_CLASS_CODE     0x0B    // Base Class Code              ro
#define PCI_CACHE_LINE_SIZE     0x0C    // Cache Line Size              ro+
#define PCI_LATENCY_TIMER       0x0D    // Latency Timer                ro+
#define PCI_HEADER_TYPE         0x0E    // Header Type                  ro
#define PCI_BIST                0x0F    // Built in self test           rw

// some pci command reg bitfields
#define PCI_CMD_BME     0x04 // Bus master function enable
#define PCI_CMD_MSE     0x02 // Memory Space Access enable
#define PCI_CMD_IOSE    0x01 // I/O space enable

// Type 0 PCI offsets
#define PCI0_BASE_ADDR0         0x10    // Base Address 0               rw
#define PCI0_BASE_ADDR1         0x14    // Base Address 1               rw
#define PCI0_BASE_ADDR2         0x18    // Base Address 2               rw
#define PCI0_BASE_ADDR3         0x1C    // Base Address 3               rw
#define PCI0_BASE_ADDR4         0x20    // Base Address 4               rw
#define PCI0_BASE_ADDR5         0x24    // Base Address 5               rw
#define PCI0_CIS                0x28    // CardBus CIS Pointer          ro
#define PCI0_SUB_VENDOR_ID      0x2C    // Sub-Vendor ID                ro
#define PCI0_SUB_SYSTEM_ID      0x2E    // Sub-System ID                ro
#define PCI0_ROM_BASE_ADDR      0x30    // Expansion ROM Base Address   rw
#define PCI0_RESERVED0          0x34
#define PCI0_RESERVED1          0x38
#define PCI0_INTERRUPT_LINE     0x3C    // Interrupt Line               rw
#define PCI0_INTERRUPT_PIN      0x3D    // Interrupt Pin                ro
#define PCI0_MINIMUM_GRANT      0x3E    // Maximum Grant                ro
#define PCI0_MAXIMUM_LATENCY    0x3F    // Maximum Latency              ro

// Type 1 PCI offsets
#define PCI1_BASE_ADDR0         0x10    // Base Address 0               rw
#define PCI1_BASE_ADDR1         0x14    // Base Address 1               rw
#define PCI1_PRI_BUS_NUM        0x18    // Primary Bus Number           rw
#define PCI1_SEC_BUS_NUM        0x19    // Secondary Bus Number         rw
#define PCI1_SUB_BUS_NUM        0x1A    // Subordinate Bus Number       rw
#define PCI1_SEC_LAT_TIMER      0x1B    // Secondary Latency Timer      ro+
#define PCI1_IO_BASE            0x1C    // I/O Base                     rw
#define PCI1_IO_LIMIT           0x1D    // I/O Limit                    rw
#define PCI1_SECONDARY_STATUS   0x1E    // Secondary Status             rw
#define PCI1_MEM_BASE           0x20    // Memory Base                  rw
#define PCI1_MEM_LIMIT          0x22    // Memory Limit                 rw
#define PCI1_PRF_MEM_BASE       0x24    // Prefetchable Memory Base     rw
#define PCI1_PRF_MEM_LIMIT      0x26    // Prefetchable Memory Limit    rw
#define PCI1_PRF_BASE_UPPER     0x28    // Prefetchable Base Upper 32   rw
#define PCI1_PRF_LIMIT_UPPER    0x2C    // Prefetchable Limit Upper 32  rw
#define PCI1_IO_BASE_UPPER      0x30    // I/O Base Upper 16 bits       rw
#define PCI1_IO_LIMIT_UPPER     0x32    // I/O Limit Upper 16 bits      rw
#define PCI1_RESERVED           0x34    // Reserved                     ro
#define PCI1_ROM_BASE_ADDR      0x38    // Expansion ROM Base Address   rw
#define PCI1_INTR_LINE          0x3C    // Interrupt Line               rw
#define PCI1_INTR_PIN           0x3D    // Interrupt Pin                ro
#define PCI1_BRIDGE_CTRL        0x3E    // Bridge Control               rw

// Device specific offsets
#define PCI_DEVICE_SPECIFIC             0x40    // 192 bytes
#define PCI_CONFIG_SIZE         0xFF

// Some Vendor IDs
#define PCI_VENDOR_DEC                  0x1011
#define PCI_VENDOR_NCR                  0x101A
#define PCI_VENDOR_QLOGIC               0x1077
#define PCI_VENDOR_SIMOS                0x1291

// Some Product IDs
#define PCI_PRODUCT_DEC_PZA             0x0008
#define PCI_PRODUCT_NCR_810             0x0001
#define PCI_PRODUCT_QLOGIC_ISP1020      0x1020
#define PCI_PRODUCT_SIMOS_SIMOS         0x1291
#define PCI_PRODUCT_SIMOS_ETHER         0x1292

#endif // __PCIREG_H__
