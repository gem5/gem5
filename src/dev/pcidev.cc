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
#include "base/misc.hh"
#include "base/str.hh"	// for to_number
#include "base/trace.hh"
#include "dev/pciconfigall.hh"
#include "dev/pcidev.hh"
#include "dev/tsunamireg.h"
#include "mem/packet.hh"
#include "sim/builder.hh"
#include "sim/byteswap.hh"
#include "sim/param.hh"
#include "sim/root.hh"

using namespace std;


PciDev::PciConfigPort::PciConfigPort(PciDev *dev, int busid, int devid,
        int funcid, Platform *p)
        : PioPort(dev,p->system,"-pciconf"), device(dev), platform(p),
          busId(busid), deviceId(devid), functionId(funcid)
{
    configAddr = platform->calcConfigAddr(busId, deviceId, functionId);
}


Tick
PciDev::PciConfigPort::recvAtomic(Packet *pkt)
{
    assert(pkt->result == Packet::Unknown);
    assert(pkt->getAddr() >= configAddr && pkt->getAddr() < configAddr +
            PCI_CONFIG_SIZE);
    return device->recvConfig(pkt);
}

void
PciDev::PciConfigPort::recvFunctional(Packet *pkt)
{
    assert(pkt->result == Packet::Unknown);
    assert(pkt->getAddr() >= configAddr && pkt->getAddr() < configAddr +
            PCI_CONFIG_SIZE);
    device->recvConfig(pkt);
}

void
PciDev::PciConfigPort::getDeviceAddressRanges(AddrRangeList &resp,
                                              AddrRangeList &snoop)
{
    snoop.clear();
    resp.push_back(RangeSize(configAddr, PCI_CONFIG_SIZE+1));
}


bool
PciDev::PciConfigPort::recvTiming(Packet *pkt)
{
    if (pkt->result == Packet::Nacked) {
        resendNacked(pkt);
    } else {
        assert(pkt->result == Packet::Unknown);
        assert(pkt->getAddr() >= configAddr && pkt->getAddr() < configAddr +
                PCI_CONFIG_SIZE);
        Tick latency = device->recvConfig(pkt);
        // turn packet around to go back to requester
        pkt->makeTimingResponse();
        sendTiming(pkt, latency);
    }
    return true;
}

PciDev::PciDev(Params *p)
    : DmaDevice(p), plat(p->platform), configData(p->configData),
      pioDelay(p->pio_delay), configDelay(p->config_delay),
      configPort(NULL)
{
    // copy the config data from the PciConfigData object
    if (configData) {
        memcpy(config.data, configData->config.data, sizeof(config.data));
        memcpy(BARSize, configData->BARSize, sizeof(BARSize));
    } else
        panic("NULL pointer to configuration data");

    memset(BARAddrs, 0, sizeof(BARAddrs));

    plat->registerPciDevice(0, p->deviceNum, p->functionNum,
            letoh(configData->config.interruptLine));
}

void
PciDev::init()
{
    if (!configPort)
        panic("pci config port not connected to anything!");
   configPort->sendStatusChange(Port::RangeChange);
   PioDevice::init();
}

unsigned int
PciDev::drain(Event *de)
{
    unsigned int count;
    count = pioPort->drain(de) + dmaPort->drain(de) + configPort->drain(de);
    if (count)
        changeState(Draining);
    else
        changeState(Drained);
    return count;
}

Tick
PciDev::readConfig(Packet *pkt)
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
            params()->deviceNum, params()->functionNum, offset,
            (uint32_t)pkt->get<uint8_t>());
        break;
      case sizeof(uint16_t):
        pkt->set<uint16_t>(*(uint16_t*)&config.data[offset]);
        DPRINTF(PCIDEV,
            "readConfig:  dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            params()->deviceNum, params()->functionNum, offset,
            (uint32_t)pkt->get<uint16_t>());
        break;
      case sizeof(uint32_t):
        pkt->set<uint32_t>(*(uint32_t*)&config.data[offset]);
        DPRINTF(PCIDEV,
            "readConfig:  dev %#x func %#x reg %#x 4 bytes: data = %#x\n",
            params()->deviceNum, params()->functionNum, offset,
            (uint32_t)pkt->get<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->result = Packet::Success;
    return configDelay;

}

void
PciDev::addressRanges(AddrRangeList &range_list)
{
    int x = 0;
    range_list.clear();
    for (x = 0; x < 6; x++)
        if (BARAddrs[x] != 0)
            range_list.push_back(RangeSize(BARAddrs[x],BARSize[x]));
}

Tick
PciDev::writeConfig(Packet *pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset >= PCI_DEVICE_SPECIFIC)
        panic("Device specific PCI config space not implemented!\n");

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        switch (offset) {
          case PCI0_INTERRUPT_LINE:
            config.interruptLine = pkt->get<uint8_t>();
          case PCI_CACHE_LINE_SIZE:
            config.cacheLineSize = pkt->get<uint8_t>();
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
            params()->deviceNum, params()->functionNum, offset,
            (uint32_t)pkt->get<uint8_t>());
        break;
      case sizeof(uint16_t):
        switch (offset) {
          case PCI_COMMAND:
            config.command = pkt->get<uint8_t>();
          case PCI_STATUS:
            config.status = pkt->get<uint8_t>();
          case PCI_CACHE_LINE_SIZE:
            config.cacheLineSize = pkt->get<uint8_t>();
            break;
          default:
            panic("writing to a read only register");
        }
        DPRINTF(PCIDEV,
            "writeConfig: dev %#x func %#x reg %#x 2 bytes: data = %#x\n",
            params()->deviceNum, params()->functionNum, offset,
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

            uint32_t barnum, bar_mask;
            Addr base_addr, base_size, space_base;

            barnum = BAR_NUMBER(offset);

            if (BAR_IO_SPACE(letoh(config.baseAddr[barnum]))) {
                bar_mask = BAR_IO_MASK;
                space_base = TSUNAMI_PCI0_IO;
            } else {
                bar_mask = BAR_MEM_MASK;
                space_base = TSUNAMI_PCI0_MEMORY;
            }

            // Writing 0xffffffff to a BAR tells the card to set the
            // value of the bar to size of memory it needs
            if (letoh(pkt->get<uint32_t>()) == 0xffffffff) {
                // This is I/O Space, bottom two bits are read only

                config.baseAddr[barnum] = letoh(
                        (~(BARSize[barnum] - 1) & ~bar_mask) |
                        (letoh(config.baseAddr[barnum]) & bar_mask));
            } else {
                config.baseAddr[barnum] = letoh(
                    (letoh(pkt->get<uint32_t>()) & ~bar_mask) |
                    (letoh(config.baseAddr[barnum]) & bar_mask));

                if (letoh(config.baseAddr[barnum]) & ~bar_mask) {
                    base_addr = (letoh(pkt->get<uint32_t>()) & ~bar_mask) + space_base;
                    base_size = BARSize[barnum];
                    BARAddrs[barnum] = base_addr;

                pioPort->sendStatusChange(Port::RangeChange);
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
            params()->deviceNum, params()->functionNum, offset,
            (uint32_t)pkt->get<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->result = Packet::Success;
    return configDelay;

}

void
PciDev::serialize(ostream &os)
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
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(PciConfigData)

    Param<uint16_t> VendorID;
    Param<uint16_t> DeviceID;
    Param<uint16_t> Command;
    Param<uint16_t> Status;
    Param<uint8_t> Revision;
    Param<uint8_t> ProgIF;
    Param<uint8_t> SubClassCode;
    Param<uint8_t> ClassCode;
    Param<uint8_t> CacheLineSize;
    Param<uint8_t> LatencyTimer;
    Param<uint8_t> HeaderType;
    Param<uint8_t> BIST;
    Param<uint32_t> BAR0;
    Param<uint32_t> BAR1;
    Param<uint32_t> BAR2;
    Param<uint32_t> BAR3;
    Param<uint32_t> BAR4;
    Param<uint32_t> BAR5;
    Param<uint32_t> CardbusCIS;
    Param<uint16_t> SubsystemVendorID;
    Param<uint16_t> SubsystemID;
    Param<uint32_t> ExpansionROM;
    Param<uint8_t> InterruptLine;
    Param<uint8_t> InterruptPin;
    Param<uint8_t> MinimumGrant;
    Param<uint8_t> MaximumLatency;
    Param<uint32_t> BAR0Size;
    Param<uint32_t> BAR1Size;
    Param<uint32_t> BAR2Size;
    Param<uint32_t> BAR3Size;
    Param<uint32_t> BAR4Size;
    Param<uint32_t> BAR5Size;

END_DECLARE_SIM_OBJECT_PARAMS(PciConfigData)

BEGIN_INIT_SIM_OBJECT_PARAMS(PciConfigData)

    INIT_PARAM(VendorID, "Vendor ID"),
    INIT_PARAM(DeviceID, "Device ID"),
    INIT_PARAM_DFLT(Command, "Command Register", 0x00),
    INIT_PARAM_DFLT(Status, "Status Register", 0x00),
    INIT_PARAM_DFLT(Revision, "Device Revision", 0x00),
    INIT_PARAM_DFLT(ProgIF, "Programming Interface", 0x00),
    INIT_PARAM(SubClassCode, "Sub-Class Code"),
    INIT_PARAM(ClassCode, "Class Code"),
    INIT_PARAM_DFLT(CacheLineSize, "System Cacheline Size", 0x00),
    INIT_PARAM_DFLT(LatencyTimer, "PCI Latency Timer", 0x00),
    INIT_PARAM_DFLT(HeaderType, "PCI Header Type", 0x00),
    INIT_PARAM_DFLT(BIST, "Built In Self Test", 0x00),
    INIT_PARAM_DFLT(BAR0, "Base Address Register 0", 0x00),
    INIT_PARAM_DFLT(BAR1, "Base Address Register 1", 0x00),
    INIT_PARAM_DFLT(BAR2, "Base Address Register 2", 0x00),
    INIT_PARAM_DFLT(BAR3, "Base Address Register 3", 0x00),
    INIT_PARAM_DFLT(BAR4, "Base Address Register 4", 0x00),
    INIT_PARAM_DFLT(BAR5, "Base Address Register 5", 0x00),
    INIT_PARAM_DFLT(CardbusCIS, "Cardbus Card Information Structure", 0x00),
    INIT_PARAM_DFLT(SubsystemVendorID, "Subsystem Vendor ID", 0x00),
    INIT_PARAM_DFLT(SubsystemID, "Subsystem ID", 0x00),
    INIT_PARAM_DFLT(ExpansionROM, "Expansion ROM Base Address Register", 0x00),
    INIT_PARAM(InterruptLine, "Interrupt Line Register"),
    INIT_PARAM(InterruptPin, "Interrupt Pin Register"),
    INIT_PARAM_DFLT(MinimumGrant, "Minimum Grant", 0x00),
    INIT_PARAM_DFLT(MaximumLatency, "Maximum Latency", 0x00),
    INIT_PARAM_DFLT(BAR0Size, "Base Address Register 0 Size", 0x00),
    INIT_PARAM_DFLT(BAR1Size, "Base Address Register 1 Size", 0x00),
    INIT_PARAM_DFLT(BAR2Size, "Base Address Register 2 Size", 0x00),
    INIT_PARAM_DFLT(BAR3Size, "Base Address Register 3 Size", 0x00),
    INIT_PARAM_DFLT(BAR4Size, "Base Address Register 4 Size", 0x00),
    INIT_PARAM_DFLT(BAR5Size, "Base Address Register 5 Size", 0x00)

END_INIT_SIM_OBJECT_PARAMS(PciConfigData)

CREATE_SIM_OBJECT(PciConfigData)
{
    PciConfigData *data = new PciConfigData(getInstanceName());

    data->config.vendor = htole(VendorID);
    data->config.device = htole(DeviceID);
    data->config.command = htole(Command);
    data->config.status = htole(Status);
    data->config.revision = htole(Revision);
    data->config.progIF = htole(ProgIF);
    data->config.subClassCode = htole(SubClassCode);
    data->config.classCode = htole(ClassCode);
    data->config.cacheLineSize = htole(CacheLineSize);
    data->config.latencyTimer = htole(LatencyTimer);
    data->config.headerType = htole(HeaderType);
    data->config.bist = htole(BIST);

    data->config.baseAddr0 = htole(BAR0);
    data->config.baseAddr1 = htole(BAR1);
    data->config.baseAddr2 = htole(BAR2);
    data->config.baseAddr3 = htole(BAR3);
    data->config.baseAddr4 = htole(BAR4);
    data->config.baseAddr5 = htole(BAR5);
    data->config.cardbusCIS = htole(CardbusCIS);
    data->config.subsystemVendorID = htole(SubsystemVendorID);
    data->config.subsystemID = htole(SubsystemVendorID);
    data->config.expansionROM = htole(ExpansionROM);
    data->config.interruptLine = htole(InterruptLine);
    data->config.interruptPin = htole(InterruptPin);
    data->config.minimumGrant = htole(MinimumGrant);
    data->config.maximumLatency = htole(MaximumLatency);

    data->BARSize[0] = BAR0Size;
    data->BARSize[1] = BAR1Size;
    data->BARSize[2] = BAR2Size;
    data->BARSize[3] = BAR3Size;
    data->BARSize[4] = BAR4Size;
    data->BARSize[5] = BAR5Size;

    return data;
}

REGISTER_SIM_OBJECT("PciConfigData", PciConfigData)

#endif // DOXYGEN_SHOULD_SKIP_THIS
