/*
 * Copyright (c) 2013, 2015 ARM Limited
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 *          Andrew Schultz
 *          Miguel Serrano
 */

/* @file
 * A single PCI device configuration space entry.
 */

#include "dev/pci/device.hh"

#include <list>
#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "debug/PciDevice.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"

PciDevice::PciDevice(const PciDeviceParams *p)
    : DmaDevice(p),
      _busAddr(p->pci_bus, p->pci_dev, p->pci_func),
      PMCAP_BASE(p->PMCAPBaseOffset),
      PMCAP_ID_OFFSET(p->PMCAPBaseOffset+PMCAP_ID),
      PMCAP_PC_OFFSET(p->PMCAPBaseOffset+PMCAP_PC),
      PMCAP_PMCS_OFFSET(p->PMCAPBaseOffset+PMCAP_PMCS),
      MSICAP_BASE(p->MSICAPBaseOffset),
      MSIXCAP_BASE(p->MSIXCAPBaseOffset),
      MSIXCAP_ID_OFFSET(p->MSIXCAPBaseOffset+MSIXCAP_ID),
      MSIXCAP_MXC_OFFSET(p->MSIXCAPBaseOffset+MSIXCAP_MXC),
      MSIXCAP_MTAB_OFFSET(p->MSIXCAPBaseOffset+MSIXCAP_MTAB),
      MSIXCAP_MPBA_OFFSET(p->MSIXCAPBaseOffset+MSIXCAP_MPBA),
      PXCAP_BASE(p->PXCAPBaseOffset),

      hostInterface(p->host->registerDevice(this, _busAddr,
                                            (PciIntPin)p->InterruptPin)),
      pioDelay(p->pio_latency),
      configDelay(p->config_latency)
{
    fatal_if(p->InterruptPin >= 5,
             "Invalid PCI interrupt '%i' specified.", p->InterruptPin);

    config.vendor = htole(p->VendorID);
    config.device = htole(p->DeviceID);
    config.command = htole(p->Command);
    config.status = htole(p->Status);
    config.revision = htole(p->Revision);
    config.progIF = htole(p->ProgIF);
    config.subClassCode = htole(p->SubClassCode);
    config.classCode = htole(p->ClassCode);
    config.cacheLineSize = htole(p->CacheLineSize);
    config.latencyTimer = htole(p->LatencyTimer);
    config.headerType = htole(p->HeaderType);
    config.bist = htole(p->BIST);

    config.baseAddr[0] = htole(p->BAR0);
    config.baseAddr[1] = htole(p->BAR1);
    config.baseAddr[2] = htole(p->BAR2);
    config.baseAddr[3] = htole(p->BAR3);
    config.baseAddr[4] = htole(p->BAR4);
    config.baseAddr[5] = htole(p->BAR5);
    config.cardbusCIS = htole(p->CardbusCIS);
    config.subsystemVendorID = htole(p->SubsystemVendorID);
    config.subsystemID = htole(p->SubsystemID);
    config.expansionROM = htole(p->ExpansionROM);
    config.capabilityPtr = htole(p->CapabilityPtr);
    // Zero out the 7 bytes of reserved space in the PCI Config space register.
    bzero(config.reserved, 7*sizeof(uint8_t));
    config.interruptLine = htole(p->InterruptLine);
    config.interruptPin = htole(p->InterruptPin);
    config.minimumGrant = htole(p->MinimumGrant);
    config.maximumLatency = htole(p->MaximumLatency);

    // Initialize the capability lists
    // These structs are bitunions, meaning the data is stored in host
    // endianess and must be converted to Little Endian when accessed
    // by the guest
    // PMCAP
    pmcap.pid = (uint16_t)p->PMCAPCapId; // pid.cid
    pmcap.pid |= (uint16_t)p->PMCAPNextCapability << 8; //pid.next
    pmcap.pc = p->PMCAPCapabilities;
    pmcap.pmcs = p->PMCAPCtrlStatus;

    // MSICAP
    msicap.mid = (uint16_t)p->MSICAPCapId; //mid.cid
    msicap.mid |= (uint16_t)p->MSICAPNextCapability << 8; //mid.next
    msicap.mc = p->MSICAPMsgCtrl;
    msicap.ma = p->MSICAPMsgAddr;
    msicap.mua = p->MSICAPMsgUpperAddr;
    msicap.md = p->MSICAPMsgData;
    msicap.mmask = p->MSICAPMaskBits;
    msicap.mpend = p->MSICAPPendingBits;

    // MSIXCAP
    msixcap.mxid = (uint16_t)p->MSIXCAPCapId; //mxid.cid
    msixcap.mxid |= (uint16_t)p->MSIXCAPNextCapability << 8; //mxid.next
    msixcap.mxc = p->MSIXMsgCtrl;
    msixcap.mtab = p->MSIXTableOffset;
    msixcap.mpba = p->MSIXPbaOffset;

    // allocate MSIX structures if MSIXCAP_BASE
    // indicates the MSIXCAP is being used by having a
    // non-zero base address.
    // The MSIX tables are stored by the guest in
    // little endian byte-order as according the
    // PCIe specification.  Make sure to take the proper
    // actions when manipulating these tables on the host
    uint16_t msixcap_mxc_ts = msixcap.mxc & 0x07ff;
    if (MSIXCAP_BASE != 0x0) {
        int msix_vecs = msixcap_mxc_ts + 1;
        MSIXTable tmp1 = {{0UL,0UL,0UL,0UL}};
        msix_table.resize(msix_vecs, tmp1);

        MSIXPbaEntry tmp2 = {0};
        int pba_size = msix_vecs / MSIXVECS_PER_PBA;
        if ((msix_vecs % MSIXVECS_PER_PBA) > 0) {
            pba_size++;
        }
        msix_pba.resize(pba_size, tmp2);
    }
    MSIX_TABLE_OFFSET = msixcap.mtab & 0xfffffffc;
    MSIX_TABLE_END = MSIX_TABLE_OFFSET +
                     (msixcap_mxc_ts + 1) * sizeof(MSIXTable);
    MSIX_PBA_OFFSET = msixcap.mpba & 0xfffffffc;
    MSIX_PBA_END = MSIX_PBA_OFFSET +
                   ((msixcap_mxc_ts + 1) / MSIXVECS_PER_PBA)
                   * sizeof(MSIXPbaEntry);
    if (((msixcap_mxc_ts + 1) % MSIXVECS_PER_PBA) > 0) {
        MSIX_PBA_END += sizeof(MSIXPbaEntry);
    }

    // PXCAP
    pxcap.pxid = (uint16_t)p->PXCAPCapId; //pxid.cid
    pxcap.pxid |= (uint16_t)p->PXCAPNextCapability << 8; //pxid.next
    pxcap.pxcap = p->PXCAPCapabilities;
    pxcap.pxdcap = p->PXCAPDevCapabilities;
    pxcap.pxdc = p->PXCAPDevCtrl;
    pxcap.pxds = p->PXCAPDevStatus;
    pxcap.pxlcap = p->PXCAPLinkCap;
    pxcap.pxlc = p->PXCAPLinkCtrl;
    pxcap.pxls = p->PXCAPLinkStatus;
    pxcap.pxdcap2 = p->PXCAPDevCap2;
    pxcap.pxdc2 = p->PXCAPDevCtrl2;

    BARSize[0] = p->BAR0Size;
    BARSize[1] = p->BAR1Size;
    BARSize[2] = p->BAR2Size;
    BARSize[3] = p->BAR3Size;
    BARSize[4] = p->BAR4Size;
    BARSize[5] = p->BAR5Size;

    legacyIO[0] = p->BAR0LegacyIO;
    legacyIO[1] = p->BAR1LegacyIO;
    legacyIO[2] = p->BAR2LegacyIO;
    legacyIO[3] = p->BAR3LegacyIO;
    legacyIO[4] = p->BAR4LegacyIO;
    legacyIO[5] = p->BAR5LegacyIO;

    for (int i = 0; i < 6; ++i) {
        if (legacyIO[i]) {
            BARAddrs[i] = p->LegacyIOBase + letoh(config.baseAddr[i]);
            config.baseAddr[i] = 0;
        } else {
            BARAddrs[i] = 0;
            uint32_t barsize = BARSize[i];
            if (barsize != 0 && !isPowerOf2(barsize)) {
                fatal("BAR %d size %d is not a power of 2\n", i, BARSize[i]);
            }
        }
    }
}

Tick
PciDevice::readConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;

    /* Return 0 for accesses to unimplemented PCI configspace areas */
    if (offset >= PCI_DEVICE_SPECIFIC &&
        offset < PCI_CONFIG_SIZE) {
        warn_once("Device specific PCI config space "
                  "not implemented for %s!\n", this->name());
        switch (pkt->getSize()) {
            case sizeof(uint8_t):
                pkt->set<uint8_t>(0);
                break;
            case sizeof(uint16_t):
                pkt->set<uint16_t>(0);
                break;
            case sizeof(uint32_t):
                pkt->set<uint32_t>(0);
                break;
            default:
                panic("invalid access size(?) for PCI configspace!\n");
        }
    } else if (offset > PCI_CONFIG_SIZE) {
        panic("Out-of-range access to PCI config space!\n");
    }

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        pkt->set<uint8_t>(config.data[offset]);
        DPRINTF(PciDevice,
            "readConfig:  dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->get<uint8_t>());
        break;
      case sizeof(uint16_t):
        pkt->set<uint16_t>(*(uint16_t*)&config.data[offset]);
        DPRINTF(PciDevice,
            "readConfig:  dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->get<uint16_t>());
        break;
      case sizeof(uint32_t):
        pkt->set<uint32_t>(*(uint32_t*)&config.data[offset]);
        DPRINTF(PciDevice,
            "readConfig:  dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->get<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->makeAtomicResponse();
    return configDelay;

}

AddrRangeList
PciDevice::getAddrRanges() const
{
    AddrRangeList ranges;
    int x = 0;
    for (x = 0; x < 6; x++)
        if (BARAddrs[x] != 0)
            ranges.push_back(RangeSize(BARAddrs[x],BARSize[x]));
    return ranges;
}

Tick
PciDevice::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;

    /* No effect if we write to config space that is not implemented*/
    if (offset >= PCI_DEVICE_SPECIFIC &&
        offset < PCI_CONFIG_SIZE) {
        warn_once("Device specific PCI config space "
                  "not implemented for %s!\n", this->name());
        switch (pkt->getSize()) {
            case sizeof(uint8_t):
            case sizeof(uint16_t):
            case sizeof(uint32_t):
                break;
            default:
                panic("invalid access size(?) for PCI configspace!\n");
        }
    } else if (offset > PCI_CONFIG_SIZE) {
        panic("Out-of-range access to PCI config space!\n");
    }

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        switch (offset) {
          case PCI0_INTERRUPT_LINE:
            config.interruptLine = pkt->get<uint8_t>();
            break;
          case PCI_CACHE_LINE_SIZE:
            config.cacheLineSize = pkt->get<uint8_t>();
            break;
          case PCI_LATENCY_TIMER:
            config.latencyTimer = pkt->get<uint8_t>();
            break;
          /* Do nothing for these read-only registers */
          case PCI0_INTERRUPT_PIN:
          case PCI0_MINIMUM_GRANT:
          case PCI0_MAXIMUM_LATENCY:
          case PCI_CLASS_CODE:
          case PCI_REVISION_ID:
            break;
          default:
            panic("writing to a read only register");
        }
        DPRINTF(PciDevice,
            "writeConfig: dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->get<uint8_t>());
        break;
      case sizeof(uint16_t):
        switch (offset) {
          case PCI_COMMAND:
            config.command = pkt->get<uint8_t>();
            break;
          case PCI_STATUS:
            config.status = pkt->get<uint8_t>();
            break;
          case PCI_CACHE_LINE_SIZE:
            config.cacheLineSize = pkt->get<uint8_t>();
            break;
          default:
            panic("writing to a read only register");
        }
        DPRINTF(PciDevice,
            "writeConfig: dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->get<uint16_t>());
        break;
      case sizeof(uint32_t):
        switch (offset) {
          case PCI0_BASE_ADDR0:
          case PCI0_BASE_ADDR1:
          case PCI0_BASE_ADDR2:
          case PCI0_BASE_ADDR3:
          case PCI0_BASE_ADDR4:
          case PCI0_BASE_ADDR5:
            {
                int barnum = BAR_NUMBER(offset);

                if (!legacyIO[barnum]) {
                    // convert BAR values to host endianness
                    uint32_t he_old_bar = letoh(config.baseAddr[barnum]);
                    uint32_t he_new_bar = letoh(pkt->get<uint32_t>());

                    uint32_t bar_mask =
                        BAR_IO_SPACE(he_old_bar) ? BAR_IO_MASK : BAR_MEM_MASK;

                    // Writing 0xffffffff to a BAR tells the card to set the
                    // value of the bar to a bitmask indicating the size of
                    // memory it needs
                    if (he_new_bar == 0xffffffff) {
                        he_new_bar = ~(BARSize[barnum] - 1);
                    } else {
                        // does it mean something special to write 0 to a BAR?
                        he_new_bar &= ~bar_mask;
                        if (he_new_bar) {
                            BARAddrs[barnum] = BAR_IO_SPACE(he_old_bar) ?
                                hostInterface.pioAddr(he_new_bar) :
                                hostInterface.memAddr(he_new_bar);
                            pioPort.sendRangeChange();
                        }
                    }
                    config.baseAddr[barnum] = htole((he_new_bar & ~bar_mask) |
                                                    (he_old_bar & bar_mask));
                }
            }
            break;

          case PCI0_ROM_BASE_ADDR:
            if (letoh(pkt->get<uint32_t>()) == 0xfffffffe)
                config.expansionROM = htole((uint32_t)0xffffffff);
            else
                config.expansionROM = pkt->get<uint32_t>();
            break;

          case PCI_COMMAND:
            // This could also clear some of the error bits in the Status
            // register. However they should never get set, so lets ignore
            // it for now
            config.command = pkt->get<uint32_t>();
            break;

          default:
            DPRINTF(PciDevice, "Writing to a read only register");
        }
        DPRINTF(PciDevice,
            "writeConfig: dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->get<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->makeAtomicResponse();
    return configDelay;
}

void
PciDevice::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ARRAY(BARSize, sizeof(BARSize) / sizeof(BARSize[0]));
    SERIALIZE_ARRAY(BARAddrs, sizeof(BARAddrs) / sizeof(BARAddrs[0]));
    SERIALIZE_ARRAY(config.data, sizeof(config.data) / sizeof(config.data[0]));

    // serialize the capability list registers
    paramOut(cp, csprintf("pmcap.pid"), uint16_t(pmcap.pid));
    paramOut(cp, csprintf("pmcap.pc"), uint16_t(pmcap.pc));
    paramOut(cp, csprintf("pmcap.pmcs"), uint16_t(pmcap.pmcs));

    paramOut(cp, csprintf("msicap.mid"), uint16_t(msicap.mid));
    paramOut(cp, csprintf("msicap.mc"), uint16_t(msicap.mc));
    paramOut(cp, csprintf("msicap.ma"), uint32_t(msicap.ma));
    SERIALIZE_SCALAR(msicap.mua);
    paramOut(cp, csprintf("msicap.md"), uint16_t(msicap.md));
    SERIALIZE_SCALAR(msicap.mmask);
    SERIALIZE_SCALAR(msicap.mpend);

    paramOut(cp, csprintf("msixcap.mxid"), uint16_t(msixcap.mxid));
    paramOut(cp, csprintf("msixcap.mxc"), uint16_t(msixcap.mxc));
    paramOut(cp, csprintf("msixcap.mtab"), uint32_t(msixcap.mtab));
    paramOut(cp, csprintf("msixcap.mpba"), uint32_t(msixcap.mpba));

    // Only serialize if we have a non-zero base address
    if (MSIXCAP_BASE != 0x0) {
        uint16_t msixcap_mxc_ts = msixcap.mxc & 0x07ff;
        int msix_array_size = msixcap_mxc_ts + 1;
        int pba_array_size = msix_array_size/MSIXVECS_PER_PBA;
        if ((msix_array_size % MSIXVECS_PER_PBA) > 0) {
            pba_array_size++;
        }

        SERIALIZE_SCALAR(msix_array_size);
        SERIALIZE_SCALAR(pba_array_size);

        for (int i = 0; i < msix_array_size; i++) {
            paramOut(cp, csprintf("msix_table[%d].addr_lo", i),
                     msix_table[i].fields.addr_lo);
            paramOut(cp, csprintf("msix_table[%d].addr_hi", i),
                     msix_table[i].fields.addr_hi);
            paramOut(cp, csprintf("msix_table[%d].msg_data", i),
                     msix_table[i].fields.msg_data);
            paramOut(cp, csprintf("msix_table[%d].vec_ctrl", i),
                     msix_table[i].fields.vec_ctrl);
        }
        for (int i = 0; i < pba_array_size; i++) {
            paramOut(cp, csprintf("msix_pba[%d].bits", i),
                     msix_pba[i].bits);
        }
    }

    paramOut(cp, csprintf("pxcap.pxid"), uint16_t(pxcap.pxid));
    paramOut(cp, csprintf("pxcap.pxcap"), uint16_t(pxcap.pxcap));
    paramOut(cp, csprintf("pxcap.pxdcap"), uint32_t(pxcap.pxdcap));
    paramOut(cp, csprintf("pxcap.pxdc"), uint16_t(pxcap.pxdc));
    paramOut(cp, csprintf("pxcap.pxds"), uint16_t(pxcap.pxds));
    paramOut(cp, csprintf("pxcap.pxlcap"), uint32_t(pxcap.pxlcap));
    paramOut(cp, csprintf("pxcap.pxlc"), uint16_t(pxcap.pxlc));
    paramOut(cp, csprintf("pxcap.pxls"), uint16_t(pxcap.pxls));
    paramOut(cp, csprintf("pxcap.pxdcap2"), uint32_t(pxcap.pxdcap2));
    paramOut(cp, csprintf("pxcap.pxdc2"), uint32_t(pxcap.pxdc2));
}

void
PciDevice::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(BARSize, sizeof(BARSize) / sizeof(BARSize[0]));
    UNSERIALIZE_ARRAY(BARAddrs, sizeof(BARAddrs) / sizeof(BARAddrs[0]));
    UNSERIALIZE_ARRAY(config.data,
                      sizeof(config.data) / sizeof(config.data[0]));

    // unserialize the capability list registers
    uint16_t tmp16;
    uint32_t tmp32;
    paramIn(cp, csprintf("pmcap.pid"), tmp16);
    pmcap.pid = tmp16;
    paramIn(cp, csprintf("pmcap.pc"), tmp16);
    pmcap.pc = tmp16;
    paramIn(cp, csprintf("pmcap.pmcs"), tmp16);
    pmcap.pmcs = tmp16;

    paramIn(cp, csprintf("msicap.mid"), tmp16);
    msicap.mid = tmp16;
    paramIn(cp, csprintf("msicap.mc"), tmp16);
    msicap.mc = tmp16;
    paramIn(cp, csprintf("msicap.ma"), tmp32);
    msicap.ma = tmp32;
    UNSERIALIZE_SCALAR(msicap.mua);
    paramIn(cp, csprintf("msicap.md"), tmp16);;
    msicap.md = tmp16;
    UNSERIALIZE_SCALAR(msicap.mmask);
    UNSERIALIZE_SCALAR(msicap.mpend);

    paramIn(cp, csprintf("msixcap.mxid"), tmp16);
    msixcap.mxid = tmp16;
    paramIn(cp, csprintf("msixcap.mxc"), tmp16);
    msixcap.mxc = tmp16;
    paramIn(cp, csprintf("msixcap.mtab"), tmp32);
    msixcap.mtab = tmp32;
    paramIn(cp, csprintf("msixcap.mpba"), tmp32);
    msixcap.mpba = tmp32;

    // Only allocate if MSIXCAP_BASE is not 0x0
    if (MSIXCAP_BASE != 0x0) {
        int msix_array_size;
        int pba_array_size;

        UNSERIALIZE_SCALAR(msix_array_size);
        UNSERIALIZE_SCALAR(pba_array_size);

        MSIXTable tmp1 = {{0UL, 0UL, 0UL, 0UL}};
        msix_table.resize(msix_array_size, tmp1);

        MSIXPbaEntry tmp2 = {0};
        msix_pba.resize(pba_array_size, tmp2);

        for (int i = 0; i < msix_array_size; i++) {
            paramIn(cp, csprintf("msix_table[%d].addr_lo", i),
                    msix_table[i].fields.addr_lo);
            paramIn(cp, csprintf("msix_table[%d].addr_hi", i),
                    msix_table[i].fields.addr_hi);
            paramIn(cp, csprintf("msix_table[%d].msg_data", i),
                    msix_table[i].fields.msg_data);
            paramIn(cp, csprintf("msix_table[%d].vec_ctrl", i),
                    msix_table[i].fields.vec_ctrl);
        }
        for (int i = 0; i < pba_array_size; i++) {
            paramIn(cp, csprintf("msix_pba[%d].bits", i),
                    msix_pba[i].bits);
        }
    }

    paramIn(cp, csprintf("pxcap.pxid"), tmp16);
    pxcap.pxid = tmp16;
    paramIn(cp, csprintf("pxcap.pxcap"), tmp16);
    pxcap.pxcap = tmp16;
    paramIn(cp, csprintf("pxcap.pxdcap"), tmp32);
    pxcap.pxdcap = tmp32;
    paramIn(cp, csprintf("pxcap.pxdc"), tmp16);
    pxcap.pxdc = tmp16;
    paramIn(cp, csprintf("pxcap.pxds"), tmp16);
    pxcap.pxds = tmp16;
    paramIn(cp, csprintf("pxcap.pxlcap"), tmp32);
    pxcap.pxlcap = tmp32;
    paramIn(cp, csprintf("pxcap.pxlc"), tmp16);
    pxcap.pxlc = tmp16;
    paramIn(cp, csprintf("pxcap.pxls"), tmp16);
    pxcap.pxls = tmp16;
    paramIn(cp, csprintf("pxcap.pxdcap2"), tmp32);
    pxcap.pxdcap2 = tmp32;
    paramIn(cp, csprintf("pxcap.pxdc2"), tmp32);
    pxcap.pxdc2 = tmp32;
    pioPort.sendRangeChange();
}

