/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#include "base/bitfield.hh"
#include "base/bitunion.hh"

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
        uint8_t capabilityPtr;
        // Was 8 bytes in the legacy PCI spec, but to support PCIe
        // this field is now 7 bytes with PCIe's addition of the
        // capability list pointer.
        uint8_t reserved[7];
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
#define PCI0_CAP_PTR            0x34    // Capability list pointer      ro
#define PCI0_RESERVED           0x35
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

/**
 * PCIe capability list offsets internal to the entry.
 * Actual offsets in the PCI config space are defined in
 * the python files setting up the system.
 */
#define PMCAP_ID 0x00
#define PMCAP_PC 0x02
#define PMCAP_PMCS 0x04
#define PMCAP_SIZE 0x06

#define MSICAP_ID 0x00
#define MSICAP_MC 0x02
#define MSICAP_MA 0x04
#define MSICAP_MUA 0x08
#define MSICAP_MD 0x0C
#define MSICAP_MMASK 0x10
#define MSICAP_MPEND 0x14
#define MSICAP_SIZE 0x18

#define MSIXCAP_ID 0x00
#define MSIXCAP_MXC 0x02
#define MSIXCAP_MTAB 0x04
#define MSIXCAP_MPBA 0x08
#define MSIXCAP_SIZE 0x0C

#define PXCAP_ID 0x00
#define PXCAP_PXCAP 0x02
#define PXCAP_PXDCAP 0x04
#define PXCAP_PXDC 0x08
#define PXCAP_PXDS 0x0A
#define PXCAP_PXLCAP 0x0C
#define PXCAP_PXLC 0x10
#define PXCAP_PXLS 0x12
#define PXCAP_PXDCAP2 0x24
#define PXCAP_PXDC2 0x28
#define PXCAP_SIZE 0x30

/** @struct PMCAP
 *  Defines the Power Management capability register and all its associated
 *  bitfields for a PCIe device.
 */
struct PMCAP {
    BitUnion16(PID)
        Bitfield<7,0>   cid;
        Bitfield<15,8>  next;
    EndBitUnion(PID)
    PID pid;

    BitUnion16(PC)
        Bitfield<2,0>   vs;
        Bitfield<3>     pmec;
        Bitfield<4>     reserved;
        Bitfield<5>     dsi;
        Bitfield<8,6>   auxc;
        Bitfield<9>     d1s;
        Bitfield<10>    d2s;
        Bitfield<15,11> psup;
    EndBitUnion(PC)
    PC pc;

    BitUnion16(PMCS)
        Bitfield<1,0>   ps;
        Bitfield<2>     reserved0;
        Bitfield<3>     nsfrst;
        Bitfield<7,4>   reserved1;
        Bitfield<8>     pmee;
        Bitfield<12,9>  dse;
        Bitfield<14,13> dsc;
        Bitfield<15>    pmes;
    EndBitUnion(PMCS)
    PMCS pmcs;
};

/** @struct MSICAP
 *  Defines the MSI Capability register and its associated bitfields for
 *  the a PCI/PCIe device.  Both the MSI capability and the MSIX capability
 *  can be filled in if a device model supports both, but only 1 of
 *  MSI/MSIX/INTx interrupt mode can be selected at a given time.
 */
struct MSICAP {
    BitUnion16(MID)
        Bitfield<7,0>   cid;
        Bitfield<15,8>  next;
    EndBitUnion(MID)
    MID mid;

    BitUnion16(MC)
        Bitfield<0>     msie;
        Bitfield<3,1>   mmc;
        Bitfield<6,4>   mme;
        Bitfield<7>     c64;
        Bitfield<8>     pvm;
        Bitfield<15,9>  reserved;
    EndBitUnion(MC)
    MC mc;

    BitUnion32(MA)
        Bitfield<1,0>   reserved;
        Bitfield<31,2>  addr;
    EndBitUnion(MA)
    MA ma;

    uint32_t mua;

    BitUnion16(MD)
        Bitfield<15,0> data;
    EndBitUnion(MD)
    MD md;

    uint32_t mmask;
    uint32_t mpend;
};

/** @struct MSIX
 *  Defines the MSI-X Capability register and its associated bitfields for
 *  a PCIe device.
 */
struct MSIXCAP {
    BitUnion16(MXID)
        Bitfield<7,0>   cid;
        Bitfield<15,8>  next;
    EndBitUnion(MXID)
    MXID mxid;

    BitUnion16(MXC)
        Bitfield<10,0>  ts;
        Bitfield<13,11> reserved;
        Bitfield<14>    fm;
        Bitfield<15>    mxe;
    EndBitUnion(MXC)
    MXC mxc;

    BitUnion32(MTAB)
        Bitfield<31,3>  to;
        Bitfield<2,0>   tbir;
    EndBitUnion(MTAB)
    MTAB mtab;

    BitUnion32(MPBA)
        Bitfield<2,0>   pbir;
        Bitfield<31,3>  pbao;
    EndBitUnion(MPBA)
    MPBA mpba;
};

union MSIXTable {
    struct {
        uint32_t addr_lo;
        uint32_t addr_hi;
        uint32_t msg_data;
        uint32_t vec_ctrl;
    } fields;
    uint32_t data[4];
};

#define MSIXVECS_PER_PBA 64
struct MSIXPbaEntry {
    uint64_t bits;
};

/** @struct PXCAP
 *  Defines the PCI Express capability register and its associated bitfields
 *  for a PCIe device.
 */
struct PXCAP {
    BitUnion16(PXID)
        Bitfield<7,0>   cid;
        Bitfield<15,8>  next;
    EndBitUnion(PXID)
    PXID pxid;

    BitUnion16(_PXCAP)
        Bitfield<3,0>   ver;
        Bitfield<7,4>   dpt;
        Bitfield<8>     si;
        Bitfield<13,9>  imn;
        Bitfield<15,14> reserved;
    EndBitUnion(_PXCAP)
    _PXCAP pxcap;

    BitUnion32(PXDCAP)
        Bitfield<2,0>   mps;
        Bitfield<4,3>   pfs;
        Bitfield<5>     etfs;
        Bitfield<8,6>   l0sl;
        Bitfield<11,9>  l1l;
        Bitfield<14,12> reserved0;
        Bitfield<15>    rer;
        Bitfield<17,16> reserved1;
        Bitfield<25,18> csplv;
        Bitfield<27,26> cspls;
        Bitfield<28>    flrc;
        Bitfield<31,29> reserved2;
    EndBitUnion(PXDCAP)
    PXDCAP pxdcap;

    BitUnion16(PXDC)
        Bitfield<0>     cere;
        Bitfield<1>     nfere;
        Bitfield<2>     fere;
        Bitfield<3>     urre;
        Bitfield<4>     ero;
        Bitfield<7,5>   mps;
        Bitfield<8>     ete;
        Bitfield<9>     pfe;
        Bitfield<10>    appme;
        Bitfield<11>    ens;
        Bitfield<14,12> mrrs;
        Bitfield<15>    func_reset;
    EndBitUnion(PXDC)
    PXDC pxdc;

    BitUnion16(PXDS)
        Bitfield<0>     ced;
        Bitfield<1>     nfed;
        Bitfield<2>     fed;
        Bitfield<3>     urd;
        Bitfield<4>     apd;
        Bitfield<5>     tp;
        Bitfield<15,6>  reserved;
    EndBitUnion(PXDS)
    PXDS pxds;

    BitUnion32(PXLCAP)
        Bitfield<3,0>   sls;
        Bitfield<9,4>   mlw;
        Bitfield<11,10> aspms;
        Bitfield<14,12> l0sel;
        Bitfield<17,15> l1el;
        Bitfield<18>    cpm;
        Bitfield<19>    sderc;
        Bitfield<20>    dllla;
        Bitfield<21>    lbnc;
        Bitfield<23,22> reserved;
        Bitfield<31,24> pn;
    EndBitUnion(PXLCAP)
    PXLCAP pxlcap;

    BitUnion16(PXLC)
        Bitfield<1,0>   aspmc;
        Bitfield<2>     reserved0;
        Bitfield<3>     rcb;
        Bitfield<5,4>   reserved1;
        Bitfield<6>     ccc;
        Bitfield<7>     es;
        Bitfield<8>     ecpm;
        Bitfield<9>     hawd;
        Bitfield<15,10> reserved2;
    EndBitUnion(PXLC)
    PXLC pxlc;

    BitUnion16(PXLS)
        Bitfield<3,0>   cls;
        Bitfield<9,4>   nlw;
        Bitfield<11,10> reserved0;
        Bitfield<12>    slot_clk_config;
        Bitfield<15,13> reserved1;
    EndBitUnion(PXLS)
    PXLS pxls;

    BitUnion32(PXDCAP2)
        Bitfield<3,0>   ctrs;
        Bitfield<4>     ctds;
        Bitfield<5>     arifs;
        Bitfield<6>     aors;
        Bitfield<7>     aocs32;
        Bitfield<8>     aocs64;
        Bitfield<9>     ccs128;
        Bitfield<10>    nprpr;
        Bitfield<11>    ltrs;
        Bitfield<13,12> tphcs;
        Bitfield<17,14> reserved0;
        Bitfield<19,18> obffs;
        Bitfield<20>    effs;
        Bitfield<21>    eetps;
        Bitfield<23,22> meetp;
        Bitfield<31,24> reserved1;
    EndBitUnion(PXDCAP2)
    PXDCAP2 pxdcap2;

    BitUnion32(PXDC2)
        Bitfield<3,0>   ctv;
        Bitfield<4>     ctd;
        Bitfield<9,5>   reserved0;
        Bitfield<10>    ltrme;
        Bitfield<12,11> reserved1;
        Bitfield<14,13> obffe;
        Bitfield<31,15> reserved2;
    EndBitUnion(PXDC2)
    PXDC2 pxdc2;
};
#endif // __PCIREG_H__
