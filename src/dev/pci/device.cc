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
 */

/* @file
 * A single PCI device configuration space entry.
 */

#include "dev/pci/device.hh"

#include <initializer_list>
#include <list>
#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "debug/PciBridge.hh"
#include "debug/PciDevice.hh"
#include "debug/PciEndpoint.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/byteswap.hh"

namespace gem5
{

PciDevice::PciDevice(const PciDeviceParams &p,
                     std::initializer_list<PciBar *> BARs_init)
    : DmaDevice(p),
      _busAddr(p.pci_bus, p.pci_dev, p.pci_func),
      PMCAP_BASE(p.PMCAPBaseOffset),
      PMCAP_ID_OFFSET(p.PMCAPBaseOffset+PMCAP_ID),
      PMCAP_PC_OFFSET(p.PMCAPBaseOffset+PMCAP_PC),
      PMCAP_PMCS_OFFSET(p.PMCAPBaseOffset+PMCAP_PMCS),
      MSICAP_BASE(p.MSICAPBaseOffset),
      MSIXCAP_BASE(p.MSIXCAPBaseOffset),
      MSIXCAP_ID_OFFSET(p.MSIXCAPBaseOffset+MSIXCAP_ID),
      MSIXCAP_MXC_OFFSET(p.MSIXCAPBaseOffset+MSIXCAP_MXC),
      MSIXCAP_MTAB_OFFSET(p.MSIXCAPBaseOffset+MSIXCAP_MTAB),
      MSIXCAP_MPBA_OFFSET(p.MSIXCAPBaseOffset+MSIXCAP_MPBA),
      PXCAP_BASE(p.PXCAPBaseOffset),
      BARs(BARs_init),

      hostInterface(p.host->registerDevice(this, _busAddr,
                                            (PciIntPin)p.InterruptPin)),
      pioDelay(p.pio_latency),
      configDelay(p.config_latency)
{
    fatal_if(p.InterruptPin >= 5,
             "Invalid PCI interrupt '%i' specified.", p.InterruptPin);

    int idx = 0;
    for (auto *bar: BARs) {
        auto *mu = dynamic_cast<PciMemUpperBar *>(bar);
        // If this is the upper 32 bits of a memory BAR, try to connect it to
        // the lower 32 bits.
        if (mu) {
            fatal_if(idx == 0,
                    "First BAR in %s is upper 32 bits of a memory BAR.", idx);
            auto *ml = dynamic_cast<PciMemBar *>(BARs[idx - 1]);
            fatal_if(!ml, "Upper 32 bits of memory BAR in %s doesn't come "
                    "after the lower 32.");
            mu->lower(ml);
        }
        idx++;
    }

    _config.common.vendor = htole(p.VendorID);
    _config.common.device = htole(p.DeviceID);
    _config.common.command = htole(p.Command);
    _config.common.status = htole(p.Status);
    _config.common.revision = htole(p.Revision);
    _config.common.progIF = htole(p.ProgIF);
    _config.common.subClassCode = htole(p.SubClassCode);
    _config.common.classCode = htole(p.ClassCode);
    _config.common.cacheLineSize = htole(p.CacheLineSize);
    _config.common.latencyTimer = htole(p.LatencyTimer);
    _config.common.headerType = htole(p.HeaderType);
    _config.common.bist = htole(p.BIST);
    _config.common.capabilityPtr = htole(p.CapabilityPtr);
    _config.common.interruptPin = htole(p.InterruptPin);
    _config.common.interruptLine = htole(p.InterruptLine);

    // Initialize the capability lists
    // These structs are bitunions, meaning the data is stored in host
    // endianess and must be converted to Little Endian when accessed
    // by the guest
    // PMCAP
    pmcap.pid = (uint16_t)p.PMCAPCapId; // pid.cid
    pmcap.pid |= (uint16_t)p.PMCAPNextCapability << 8; //pid.next
    pmcap.pc = p.PMCAPCapabilities;
    pmcap.pmcs = p.PMCAPCtrlStatus;

    // MSICAP
    msicap.mid = (uint16_t)p.MSICAPCapId; //mid.cid
    msicap.mid |= (uint16_t)p.MSICAPNextCapability << 8; //mid.next
    msicap.mc = p.MSICAPMsgCtrl;
    msicap.ma = p.MSICAPMsgAddr;
    msicap.mua = p.MSICAPMsgUpperAddr;
    msicap.md = p.MSICAPMsgData;
    msicap.mmask = p.MSICAPMaskBits;
    msicap.mpend = p.MSICAPPendingBits;

    // MSIXCAP
    msixcap.mxid = (uint16_t)p.MSIXCAPCapId; //mxid.cid
    msixcap.mxid |= (uint16_t)p.MSIXCAPNextCapability << 8; //mxid.next
    msixcap.mxc = p.MSIXMsgCtrl;
    msixcap.mtab = p.MSIXTableOffset;
    msixcap.mpba = p.MSIXPbaOffset;

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
    pxcap.pxid = (uint16_t)p.PXCAPCapId; //pxid.cid
    pxcap.pxid |= (uint16_t)p.PXCAPNextCapability << 8; //pxid.next
    pxcap.pxcap = p.PXCAPCapabilities;
    pxcap.pxdcap = p.PXCAPDevCapabilities;
    pxcap.pxdc = p.PXCAPDevCtrl;
    pxcap.pxds = p.PXCAPDevStatus;
    pxcap.pxlcap = p.PXCAPLinkCap;
    pxcap.pxlc = p.PXCAPLinkCtrl;
    pxcap.pxls = p.PXCAPLinkStatus;
    pxcap.pxscap = p.PXCAPSlotCap;
    pxcap.pxsc = p.PXCAPSlotCtrl;
    pxcap.pxss = p.PXCAPSlotStatus;
    pxcap.pxrcap = p.PXCAPRootCap;
    pxcap.pxrc = p.PXCAPRootCtrl;
    pxcap.pxrs = p.PXCAPRootStatus;
    pxcap.pxdcap2 = p.PXCAPDevCap2;
    pxcap.pxdc2 = p.PXCAPDevCtrl2;
    pxcap.pxds2 = p.PXCAPDevStatus2;
    pxcap.pxlcap2 = p.PXCAPLinkCap2;
    pxcap.pxlc2 = p.PXCAPLinkCtrl2;
    pxcap.pxls2 = p.PXCAPLinkStatus2;
    pxcap.pxscap2 = p.PXCAPSlotCap2;
    pxcap.pxsc2 = p.PXCAPSlotCtrl2;
    pxcap.pxss2 = p.PXCAPSlotStatus2;
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
                pkt->setLE<uint8_t>(0);
                break;
            case sizeof(uint16_t):
                pkt->setLE<uint16_t>(0);
                break;
            case sizeof(uint32_t):
                pkt->setLE<uint32_t>(0);
                break;
            default:
                panic("invalid access size(?) for PCI configspace!\n");
        }
    } else if (offset > PCI_CONFIG_SIZE) {
        panic("Out-of-range access to PCI config space!\n");
    }

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        pkt->setLE<uint8_t>(_config.data[offset]);
        DPRINTF(PciDevice,
            "readConfig:  dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint8_t>());
        break;
      case sizeof(uint16_t):
        pkt->setLE<uint16_t>(*(uint16_t *)&_config.data[offset]);
        DPRINTF(PciDevice,
            "readConfig:  dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint16_t>());
        break;
      case sizeof(uint32_t):
        pkt->setLE<uint32_t>(*(uint32_t *)&_config.data[offset]);
        DPRINTF(PciDevice,
            "readConfig:  dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint32_t>());
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
    PciCommandRegister command = letoh(_config.common.command);
    for (auto *bar: BARs) {
        if (command.ioSpace && bar->isIo())
            ranges.push_back(bar->range());
        if (command.memorySpace && bar->isMem())
            ranges.push_back(bar->range());
    }
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
    } else if (!isCommonConfig(offset)) {
        warn_once("Device specific PCI config header "
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
          case PCI_INTERRUPT_LINE:
            _config.common.interruptLine = pkt->getLE<uint8_t>();
            break;
          case PCI_CACHE_LINE_SIZE:
            _config.common.cacheLineSize = pkt->getLE<uint8_t>();
            break;
          case PCI_LATENCY_TIMER:
            _config.common.latencyTimer = pkt->getLE<uint8_t>();
            break;
          /* Do nothing for these read-only registers */
          case PCI_INTERRUPT_PIN:
          case PCI_CLASS_CODE:
          case PCI_REVISION_ID:
            break;
          default:
            panic("writing to a read only register");
        }
        DPRINTF(PciDevice,
            "writeConfig: dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint8_t>());
        break;
      case sizeof(uint16_t):
        switch (offset) {
          case PCI_COMMAND:
            _config.common.command = pkt->getLE<uint8_t>();
            // IO or memory space may have been enabled/disabled.
            pioPort.sendRangeChange();
            break;
          case PCI_STATUS:
            _config.common.status = pkt->getLE<uint8_t>();
            break;
          case PCI_CACHE_LINE_SIZE:
            _config.common.cacheLineSize = pkt->getLE<uint8_t>();
            break;
          default:
            panic("writing to a read only register");
        }
        DPRINTF(PciDevice,
            "writeConfig: dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint16_t>());
        break;
      case sizeof(uint32_t):
        switch (offset) {
          case PCI_COMMAND:
            // This could also clear some of the error bits in the Status
            // register. However they should never get set, so lets ignore
            // it for now
            _config.common.command = pkt->getLE<uint32_t>();
            // IO or memory space may have been enabled/disabled.
            pioPort.sendRangeChange();
            break;

          default:
            DPRINTF(PciDevice, "Writing to a read only register");
        }
        DPRINTF(PciDevice,
            "writeConfig: dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            _busAddr.dev, _busAddr.func, offset,
            (uint32_t)pkt->getLE<uint32_t>());
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
    SERIALIZE_ARRAY(_config.data,
                    sizeof(_config.data) / sizeof(_config.data[0]));

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
    paramOut(cp, csprintf("pxcap.pxscap"), uint32_t(pxcap.pxscap));
    paramOut(cp, csprintf("pxcap.pxsc"), uint16_t(pxcap.pxsc));
    paramOut(cp, csprintf("pxcap.pxss"), uint16_t(pxcap.pxss));
    paramOut(cp, csprintf("pxcap.pxrcap"), uint16_t(pxcap.pxrcap));
    paramOut(cp, csprintf("pxcap.pxrc"), uint16_t(pxcap.pxrc));
    paramOut(cp, csprintf("pxcap.pxrs"), uint32_t(pxcap.pxrs));
    paramOut(cp, csprintf("pxcap.pxdcap2"), uint32_t(pxcap.pxdcap2));
    paramOut(cp, csprintf("pxcap.pxdc2"), uint16_t(pxcap.pxdc2));
    paramOut(cp, csprintf("pxcap.pxds2"), uint16_t(pxcap.pxds2));
    paramOut(cp, csprintf("pxcap.pxlcap2"), uint32_t(pxcap.pxlcap2));
    paramOut(cp, csprintf("pxcap.pxlc2"), uint16_t(pxcap.pxlc2));
    paramOut(cp, csprintf("pxcap.pxls2"), uint16_t(pxcap.pxls2));
    paramOut(cp, csprintf("pxcap.pxscap2"), uint32_t(pxcap.pxscap2));
    paramOut(cp, csprintf("pxcap.pxsc2"), uint16_t(pxcap.pxsc2));
    paramOut(cp, csprintf("pxcap.pxss2"), uint16_t(pxcap.pxss2));
}

void
PciDevice::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(_config.data,
                      sizeof(_config.data) / sizeof(_config.data[0]));

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
    paramIn(cp, csprintf("pxcap.pxscap"), tmp32);
    pxcap.pxscap = tmp32;
    paramIn(cp, csprintf("pxcap.pxsc"), tmp16);
    pxcap.pxsc = tmp16;
    paramIn(cp, csprintf("pxcap.pxss"), tmp16);
    pxcap.pxss = tmp16;
    paramIn(cp, csprintf("pxcap.pxrcap"), tmp16);
    pxcap.pxrcap = tmp16;
    paramIn(cp, csprintf("pxcap.pxrc"), tmp16);
    pxcap.pxrc = tmp16;
    paramIn(cp, csprintf("pxcap.pxrs"), tmp32);
    pxcap.pxrs = tmp32;
    paramIn(cp, csprintf("pxcap.pxdcap2"), tmp32);
    pxcap.pxdcap2 = tmp32;
    paramIn(cp, csprintf("pxcap.pxdc2"), tmp16);
    pxcap.pxdc2 = tmp16;
    paramIn(cp, csprintf("pxcap.pxds2"), tmp16);
    pxcap.pxds2 = tmp16;
    paramIn(cp, csprintf("pxcap.pxlcap2"), tmp32);
    pxcap.pxlcap2 = tmp32;
    paramIn(cp, csprintf("pxcap.pxlc2"), tmp16);
    pxcap.pxlc2 = tmp16;
    paramIn(cp, csprintf("pxcap.pxls2"), tmp16);
    pxcap.pxls2 = tmp16;
    paramIn(cp, csprintf("pxcap.pxscap2"), tmp32);
    pxcap.pxscap2 = tmp32;
    paramIn(cp, csprintf("pxcap.pxsc2"), tmp16);
    pxcap.pxsc2 = tmp16;
    paramIn(cp, csprintf("pxcap.pxss2"), tmp16);
    pxcap.pxss2 = tmp16;
}

PciEndpoint::PciEndpoint(const PciEndpointParams &p)
    : PciDevice(p, { p.BAR0, p.BAR1, p.BAR2, p.BAR3, p.BAR4, p.BAR5 })
{
    fatal_if((_config.common.headerType & 0x7F) != 0, "HeaderType is invalid");

    int idx = 0;
    for (auto *bar : BARs)
        _config.type0.baseAddr[idx++] = bar->write(hostInterface, 0);

    _config.type0.cardbusCIS = htole(p.CardbusCIS);
    _config.type0.subsystemVendorID = htole(p.SubsystemVendorID);
    _config.type0.subsystemID = htole(p.SubsystemID);
    _config.type0.expansionROM = htole(p.ExpansionROM);

    // Zero out the 7 bytes of reserved space in the PCI Config space register.
    bzero(_config.type0.reserved, 7 * sizeof(uint8_t));

    _config.type0.minimumGrant = htole(p.MinimumGrant);
    _config.type0.maximumLatency = htole(p.MaximumLatency);
}

Tick
PciEndpoint::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;

    if (isCommonConfig(offset)) {
        return PciDevice::writeConfig(pkt);
    } else if (offset >= PCI_DEVICE_SPECIFIC && offset < PCI_CONFIG_SIZE) {
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
          /* Do nothing for these read-only registers */
          case PCI0_MINIMUM_GRANT:
          case PCI0_MAXIMUM_LATENCY:
            break;
          default:
            panic("writing to a read only register");
        }
        DPRINTF(PciEndpoint,
                "writeConfig: dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
                _busAddr.dev, _busAddr.func, offset,
                (uint32_t)pkt->getLE<uint8_t>());
        break;
      case sizeof(uint16_t):
        panic("writing to a read only register");
      case sizeof(uint32_t):
        switch (offset) {
          case PCI0_BASE_ADDR0:
          case PCI0_BASE_ADDR1:
          case PCI0_BASE_ADDR2:
          case PCI0_BASE_ADDR3:
          case PCI0_BASE_ADDR4:
          case PCI0_BASE_ADDR5:
            {
                int num = PCI0_BAR_NUMBER(offset);
                auto *bar = BARs[num];
                _config.type0.baseAddr[num] =
                    htole(bar->write(hostInterface, pkt->getLE<uint32_t>()));
                pioPort.sendRangeChange();
            }
            break;

          case PCI0_ROM_BASE_ADDR:
            if (letoh(pkt->getLE<uint32_t>()) == 0xfffffffe)
                _config.type0.expansionROM = htole((uint32_t)0xffffffff);
            else
                _config.type0.expansionROM = pkt->getLE<uint32_t>();
            break;

          default:
            DPRINTF(PciEndpoint, "Writing to a read only register");
        }
        DPRINTF(PciEndpoint,
                "writeConfig: dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
                _busAddr.dev, _busAddr.func, offset,
                (uint32_t)pkt->getLE<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->makeAtomicResponse();
    return configDelay;
}

void
PciEndpoint::unserialize(CheckpointIn &cp)
{
    PciDevice::unserialize(cp);

    for (int idx = 0; idx < BARs.size(); idx++)
        BARs[idx]->write(hostInterface, _config.type0.baseAddr[idx]);

    pioPort.sendRangeChange();
}

PciBridge::PciBridge(const PciBridgeParams &p)
    : PciDevice(p, { p.BAR0, p.BAR1 })
{
    fatal_if((_config.common.headerType & 0x7F) != 1, "HeaderType is invalid");

    int idx = 0;
    for (auto *bar : BARs)
        _config.type1.baseAddr[idx++] = bar->write(hostInterface, 0);

    _config.type1.primaryBusNum = htole(p.PrimaryBusNumber);
    _config.type1.secondaryBusNum = htole(p.SecondaryBusNumber);
    _config.type1.subordinateBusNum = htole(p.SubordinateBusNumber);
    _config.type1.secondaryLatencyTimer = htole(p.SecondaryLatencyTimer);
    _config.type1.ioBase = htole(p.IOBase);
    _config.type1.ioLimit = htole(p.IOLimit);
    _config.type1.secondaryStatus = htole(p.SecondaryStatus);
    _config.type1.memBase = htole(p.MemoryBase);
    _config.type1.memLimit = htole(p.MemoryLimit);
    _config.type1.prefetchMemBase = htole(p.PrefetchableMemoryBase);
    _config.type1.prefetchMemLimit = htole(p.PrefetchableMemoryLimit);
    _config.type1.prefetchBaseUpper = htole(p.PrefetchableBaseUpper);
    _config.type1.prefetchLimitUpper = htole(p.PrefetchableLimitUpper);
    _config.type1.ioBaseUpper = htole(p.IOBaseUpper);
    _config.type1.ioLimitUpper = htole(p.IOLimitUpper);
    _config.type1.expansionROM = htole(p.ExpansionROM);
    _config.type1.bridgeControl = htole(p.BridgeControl);
}

Tick
PciBridge::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;

    if (isCommonConfig(offset)) {
        return PciDevice::writeConfig(pkt);
    } else if (offset >= PCI_DEVICE_SPECIFIC && offset < PCI_CONFIG_SIZE) {
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
          case PCI1_PRI_BUS_NUM:
            _config.type1.primaryBusNum = pkt->getLE<uint8_t>();
            break;
          case PCI1_SEC_BUS_NUM:
            _config.type1.secondaryBusNum = pkt->getLE<uint8_t>();
            break;
          case PCI1_SUB_BUS_NUM:
            _config.type1.subClassCode = pkt->getLE<uint8_t>();
            break;
          case PCI1_SEC_LAT_TIMER:
            _config.type1.secondaryLatencyTimer = pkt->getLE<uint8_t>();
            break;
          case PCI1_IO_BASE:
            _config.type1.ioBase = pkt->getLE<uint8_t>();
            break;
          case PCI1_IO_LIMIT:
            _config.type1.ioLimit = pkt->getLE<uint8_t>();
            break;
          default:
              panic("writing to a read only register");
        }
        DPRINTF(PciBridge,
                "writeConfig: dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
                _busAddr.dev, _busAddr.func, offset,
                (uint32_t)pkt->getLE<uint8_t>());
        break;
      case sizeof(uint16_t):
        switch (offset) {
          case PCI1_SECONDARY_STATUS:
            _config.type1.secondaryStatus = pkt->getLE<uint16_t>();
            break;
          case PCI1_MEM_BASE:
            _config.type1.memBase = pkt->getLE<uint16_t>();
            break;
          case PCI1_MEM_LIMIT:
            _config.type1.memLimit = pkt->getLE<uint16_t>();
            break;
          case PCI1_PRF_MEM_BASE:
            _config.type1.prefetchMemBase = pkt->getLE<uint16_t>();
            break;
          case PCI1_PRF_MEM_LIMIT:
            _config.type1.prefetchMemLimit = pkt->getLE<uint16_t>();
            break;
          case PCI1_IO_BASE_UPPER:
            _config.type1.ioBaseUpper = pkt->getLE<uint16_t>();
            break;
          case PCI1_IO_LIMIT_UPPER:
            _config.type1.ioLimitUpper = pkt->getLE<uint16_t>();
            break;
          case PCI1_BRIDGE_CTRL:
            _config.type1.bridgeControl = pkt->getLE<uint16_t>();
            break;
          default:
            panic("writing to a read only register");
        }
        DPRINTF(PciBridge,
                "writeConfig: dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
                _busAddr.dev, _busAddr.func, offset,
                (uint32_t)pkt->getLE<uint16_t>());
        break;
      case sizeof(uint32_t):
        switch (offset) {
          case PCI1_BASE_ADDR0:
          case PCI1_BASE_ADDR1:
            {
              int num = PCI1_BAR_NUMBER(offset);
              auto *bar = BARs[num];
              _config.type1.baseAddr[num] = htole(
                  bar->write(hostInterface, pkt->getLE<uint32_t>()));
              pioPort.sendRangeChange();
            }
            break;
          case PCI1_PRF_BASE_UPPER:
            _config.type1.prefetchBaseUpper = pkt->getLE<uint32_t>();
            break;
          case PCI1_PRF_LIMIT_UPPER:
            _config.type1.prefetchLimitUpper = pkt->getLE<uint32_t>();
            break;
          case PCI1_ROM_BASE_ADDR:
            if (letoh(pkt->getLE<uint32_t>()) == 0xfffffffe)
                _config.type1.expansionROM = htole((uint32_t)0xffffffff);
            else
                _config.type1.expansionROM = pkt->getLE<uint32_t>();
            break;
          default:
            panic("writing to a read only register");
        }
        DPRINTF(PciBridge,
                "writeConfig: dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
                _busAddr.dev, _busAddr.func, offset,
                (uint32_t)pkt->getLE<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->makeAtomicResponse();
    return configDelay;
}

void
PciBridge::unserialize(CheckpointIn &cp)
{
    PciDevice::unserialize(cp);

    for (int idx = 0; idx < BARs.size(); idx++)
        BARs[idx]->write(hostInterface, _config.type1.baseAddr[idx]);

    pioPort.sendRangeChange();
}

} // namespace gem5
