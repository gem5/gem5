/*
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

#include <list>
#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/intmath.hh"
#include "base/misc.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "debug/PCIDEV.hh"
#include "dev/alpha/tsunamireg.h"
#include "dev/pciconfigall.hh"
#include "dev/pcidev.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"


PciDev::PciConfigPort::PciConfigPort(PciDev *dev, int busid, int devid,
        int funcid, Platform *p)
    : SimpleTimingPort(dev->name() + "-pciconf", dev), device(dev),
      platform(p), busId(busid), deviceId(devid), functionId(funcid)
{
    configAddr = platform->calcPciConfigAddr(busId, deviceId, functionId);
}


Tick
PciDev::PciConfigPort::recvAtomic(PacketPtr pkt)
{
    assert(pkt->getAddr() >= configAddr &&
           pkt->getAddr() < configAddr + PCI_CONFIG_SIZE);
    return pkt->isRead() ? device->readConfig(pkt) : device->writeConfig(pkt);
}

AddrRangeList
PciDev::PciConfigPort::getAddrRanges()
{
    AddrRangeList ranges;
    if (configAddr != ULL(-1))
        ranges.push_back(RangeSize(configAddr, PCI_CONFIG_SIZE+1));
    return ranges;
}


PciDev::PciDev(const Params *p)
    : DmaDevice(p), platform(p->platform), pioDelay(p->pio_latency),
      configDelay(p->config_latency),
      configPort(this, params()->pci_bus, params()->pci_dev,
                 params()->pci_func, params()->platform)
{
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
    config.reserved0 = 0;
    config.reserved1 = 0;
    config.interruptLine = htole(p->InterruptLine);
    config.interruptPin = htole(p->InterruptPin);
    config.minimumGrant = htole(p->MinimumGrant);
    config.maximumLatency = htole(p->MaximumLatency);

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
            BARAddrs[i] = platform->calcPciIOAddr(letoh(config.baseAddr[i]));
            config.baseAddr[i] = 0;
        } else {
            BARAddrs[i] = 0;
            uint32_t barsize = BARSize[i];
            if (barsize != 0 && !isPowerOf2(barsize)) {
                fatal("BAR %d size %d is not a power of 2\n", i, BARSize[i]);
            }
        }
    }

    platform->registerPciDevice(p->pci_bus, p->pci_dev, p->pci_func,
            letoh(config.interruptLine));
}

void
PciDev::init()
{
    if (!configPort.isConnected())
        panic("PCI config port on %s not connected to anything!\n", name());
   configPort.sendRangeChange();
   DmaDevice::init();
}

unsigned int
PciDev::drain(Event *de)
{
    unsigned int count;
    count = pioPort.drain(de) + dmaPort.drain(de) + configPort.drain(de);
    if (count)
        changeState(Draining);
    else
        changeState(Drained);
    return count;
}

Tick
PciDev::readConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset >= PCI_DEVICE_SPECIFIC)
        panic("Device specific PCI config space not implemented!\n");

    pkt->allocate();

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        pkt->set<uint8_t>(config.data[offset]);
        DPRINTF(PCIDEV,
            "readConfig:  dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            params()->pci_dev, params()->pci_func, offset,
            (uint32_t)pkt->get<uint8_t>());
        break;
      case sizeof(uint16_t):
        pkt->set<uint16_t>(*(uint16_t*)&config.data[offset]);
        DPRINTF(PCIDEV,
            "readConfig:  dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            params()->pci_dev, params()->pci_func, offset,
            (uint32_t)pkt->get<uint16_t>());
        break;
      case sizeof(uint32_t):
        pkt->set<uint32_t>(*(uint32_t*)&config.data[offset]);
        DPRINTF(PCIDEV,
            "readConfig:  dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            params()->pci_dev, params()->pci_func, offset,
            (uint32_t)pkt->get<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->makeAtomicResponse();
    return configDelay;

}

AddrRangeList
PciDev::getAddrRanges()
{
    AddrRangeList ranges;
    int x = 0;
    for (x = 0; x < 6; x++)
        if (BARAddrs[x] != 0)
            ranges.push_back(RangeSize(BARAddrs[x],BARSize[x]));
    return ranges;
}

Tick
PciDev::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset >= PCI_DEVICE_SPECIFIC)
        panic("Device specific PCI config space not implemented!\n");

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
        DPRINTF(PCIDEV,
            "writeConfig: dev %#x func %#x reg %#x 1 bytes: data = %#x\n",
            params()->pci_dev, params()->pci_func, offset,
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
        DPRINTF(PCIDEV,
            "writeConfig: dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            params()->pci_dev, params()->pci_func, offset,
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
                                platform->calcPciIOAddr(he_new_bar) :
                                platform->calcPciMemAddr(he_new_bar);
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
            DPRINTF(PCIDEV, "Writing to a read only register");
        }
        DPRINTF(PCIDEV,
            "writeConfig: dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            params()->pci_dev, params()->pci_func, offset,
            (uint32_t)pkt->get<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->makeAtomicResponse();
    return configDelay;
}

void
PciDev::serialize(std::ostream &os)
{
    SERIALIZE_ARRAY(BARSize, sizeof(BARSize) / sizeof(BARSize[0]));
    SERIALIZE_ARRAY(BARAddrs, sizeof(BARAddrs) / sizeof(BARAddrs[0]));
    SERIALIZE_ARRAY(config.data, sizeof(config.data) / sizeof(config.data[0]));
}

void
PciDev::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(BARSize, sizeof(BARSize) / sizeof(BARSize[0]));
    UNSERIALIZE_ARRAY(BARAddrs, sizeof(BARAddrs) / sizeof(BARAddrs[0]));
    UNSERIALIZE_ARRAY(config.data,
                      sizeof(config.data) / sizeof(config.data[0]));
    pioPort.sendRangeChange();

}

