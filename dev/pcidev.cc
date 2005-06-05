/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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

#include <list>
#include <sstream>
#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/misc.hh"
#include "base/str.hh"	// for to_number
#include "base/trace.hh"
#include "dev/pcidev.hh"
#include "dev/pciconfigall.hh"
#include "mem/bus/bus.hh"
#include "mem/functional/memory_control.hh"
#include "sim/builder.hh"
#include "sim/param.hh"
#include "sim/root.hh"
#include "dev/tsunamireg.h"

using namespace std;

PciDev::PciDev(Params *p)
    : DmaDevice(p->name, p->plat), _params(p), plat(p->plat),
      configData(p->configData)
{
    // copy the config data from the PciConfigData object
    if (configData) {
        memcpy(config.data, configData->config.data, sizeof(config.data));
        memcpy(BARSize, configData->BARSize, sizeof(BARSize));
        memcpy(BARAddrs, configData->BARAddrs, sizeof(BARAddrs));
    } else
        panic("NULL pointer to configuration data");

    // Setup pointer in config space to point to this entry
    if (p->configSpace->deviceExists(p->deviceNum, p->functionNum))
        panic("Two PCI devices occuping same dev: %#x func: %#x",
              p->deviceNum, p->functionNum);
    else
        p->configSpace->registerDevice(p->deviceNum, p->functionNum, this);
}

void
PciDev::ReadConfig(int offset, int size, uint8_t *data)
{
    if (offset >= PCI_DEVICE_SPECIFIC)
        panic("Device specific PCI config space not implemented!\n");

    switch(size) {
      case sizeof(uint32_t):
        memcpy((uint8_t*)data, config.data + offset, sizeof(uint32_t));
        *(uint32_t*)data = htoa(*(uint32_t*)data);
        DPRINTF(PCIDEV,
                "read device: %#x function: %#x register: %#x %d bytes: data: %#x\n",
                params()->deviceNum, params()->functionNum, offset, size,
                *(uint32_t*)(config.data + offset));
        break;

      case sizeof(uint16_t):
        memcpy((uint8_t*)data, config.data + offset, sizeof(uint16_t));
        *(uint16_t*)data = htoa(*(uint16_t*)data);
        DPRINTF(PCIDEV,
                "read device: %#x function: %#x register: %#x %d bytes: data: %#x\n",
                params()->deviceNum, params()->functionNum, offset, size,
                *(uint16_t*)(config.data + offset));
        break;

      case sizeof(uint8_t):
        memcpy((uint8_t*)data, config.data + offset, sizeof(uint8_t));
        DPRINTF(PCIDEV,
                "read device: %#x function: %#x register: %#x %d bytes: data: %#x\n",
                params()->deviceNum, params()->functionNum, offset, size,
                (uint16_t)(*(uint8_t*)(config.data + offset)));
        break;

      default:
        panic("Invalid Read Size");
    }
}

void
PciDev::WriteConfig(int offset, int size, uint32_t data)
{
    if (offset >= PCI_DEVICE_SPECIFIC)
        panic("Device specific PCI config space not implemented!\n");

    uint32_t barnum;

    union {
        uint8_t byte_value;
        uint16_t half_value;
        uint32_t word_value;
    };
    word_value = data;

    DPRINTF(PCIDEV,
            "write device: %#x function: %#x reg: %#x size: %d data: %#x\n",
            params()->deviceNum, params()->functionNum, offset, size,
            word_value);

    barnum = (offset - PCI0_BASE_ADDR0) >> 2;

    switch (size) {
      case sizeof(uint8_t): // 1-byte access
        switch (offset) {
          case PCI0_INTERRUPT_LINE:
          case PCI_CACHE_LINE_SIZE:
          case PCI_LATENCY_TIMER:
            *(uint8_t *)&config.data[offset] = htoa(byte_value);
            break;

          default:
            panic("writing to a read only register");
        }
        break;

      case sizeof(uint16_t): // 2-byte access
        switch (offset) {
          case PCI_COMMAND:
          case PCI_STATUS:
          case PCI_CACHE_LINE_SIZE:
            *(uint16_t *)&config.data[offset] = htoa(half_value);
            break;

          default:
            panic("writing to a read only register");
        }
        break;

      case sizeof(uint16_t)+1: // 3-byte access
        panic("invalid access size");

      case sizeof(uint32_t): // 4-byte access
        switch (offset) {
          case PCI0_BASE_ADDR0:
          case PCI0_BASE_ADDR1:
          case PCI0_BASE_ADDR2:
          case PCI0_BASE_ADDR3:
          case PCI0_BASE_ADDR4:
          case PCI0_BASE_ADDR5:
            // Writing 0xffffffff to a BAR tells the card to set the
            // value of the bar
            // to size of memory it needs
            if (word_value == 0xffffffff) {
                // This is I/O Space, bottom two bits are read only
                if (htoa(config.data[offset]) & 0x1) {
                    *(uint32_t *)&config.data[offset] = htoa(
                        ~(BARSize[barnum] - 1) |
                        (htoa(config.data[offset]) & 0x3));
                } else {
                    // This is memory space, bottom four bits are read only
                    *(uint32_t *)&config.data[offset] = htoa(
                        ~(BARSize[barnum] - 1) |
                        (htoa(config.data[offset]) & 0xF));
                }
            } else {
                MemoryController *mmu = params()->mmu;

                // This is I/O Space, bottom two bits are read only
                if(htoa(config.data[offset]) & 0x1) {
                    *(uint32_t *)&config.data[offset] =
                        htoa((word_value & ~0x3) |
                        (htoa(config.data[offset]) & 0x3));

                    if (word_value & ~0x1) {
                        Addr base_addr = (word_value & ~0x1) + TSUNAMI_PCI0_IO;
                        Addr base_size = BARSize[barnum];

                        // It's never been set
                        if (BARAddrs[barnum] == 0)
                            mmu->add_child((FunctionalMemory *)this,
                                           RangeSize(base_addr, base_size));
                        else
                            mmu->update_child((FunctionalMemory *)this,
                                              RangeSize(BARAddrs[barnum],
                                                        base_size),
                                              RangeSize(base_addr, base_size));

                        BARAddrs[barnum] = base_addr;
                    }

                } else {
                    // This is memory space, bottom four bits are read only
                    *(uint32_t *)&config.data[offset] =
                        htoa((word_value & ~0xF) |
                        (htoa(config.data[offset]) & 0xF));

                    if (word_value & ~0x3) {
                        Addr base_addr = (word_value & ~0x3) +
                            TSUNAMI_PCI0_MEMORY;

                        Addr base_size = BARSize[barnum];

                        // It's never been set
                        if (BARAddrs[barnum] == 0)
                            mmu->add_child((FunctionalMemory *)this,
                                           RangeSize(base_addr, base_size));
                        else
                            mmu->update_child((FunctionalMemory *)this,
                                              RangeSize(BARAddrs[barnum],
                                                        base_size),
                                              RangeSize(base_addr, base_size));

                        BARAddrs[barnum] = base_addr;
                    }
                 }
            }
            break;

          case PCI0_ROM_BASE_ADDR:
            if (word_value == 0xfffffffe)
                *(uint32_t *)&config.data[offset] = 0xffffffff;
            else
                *(uint32_t *)&config.data[offset] = htoa(word_value);
            break;

          case PCI_COMMAND:
            // This could also clear some of the error bits in the Status
            // register. However they should never get set, so lets ignore
            // it for now
            *(uint16_t *)&config.data[offset] = htoa(half_value);
            break;

          default:
            DPRINTF(PCIDEV, "Writing to a read only register");
        }
        break;
    }
}

void
PciDev::serialize(ostream &os)
{
    SERIALIZE_ARRAY(BARSize, 6);
    SERIALIZE_ARRAY(BARAddrs, 6);
    SERIALIZE_ARRAY(config.data, 64);
}

void
PciDev::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(BARSize, 6);
    UNSERIALIZE_ARRAY(BARAddrs, 6);
    UNSERIALIZE_ARRAY(config.data, 64);

    // Add the MMU mappings for the BARs
    for (int i=0; i < 6; i++) {
        if (BARAddrs[i] != 0)
            params()->mmu->add_child(this, RangeSize(BARAddrs[i], BARSize[i]));
    }
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

    data->config.hdr.vendor = htoa(VendorID);
    data->config.hdr.device = htoa(DeviceID);
    data->config.hdr.command = htoa(Command);
    data->config.hdr.status = htoa(Status);
    data->config.hdr.revision = htoa(Revision);
    data->config.hdr.progIF = htoa(ProgIF);
    data->config.hdr.subClassCode = htoa(SubClassCode);
    data->config.hdr.classCode = htoa(ClassCode);
    data->config.hdr.cacheLineSize = htoa(CacheLineSize);
    data->config.hdr.latencyTimer = htoa(LatencyTimer);
    data->config.hdr.headerType = htoa(HeaderType);
    data->config.hdr.bist = htoa(BIST);

    data->config.hdr.pci0.baseAddr0 = htoa(BAR0);
    data->config.hdr.pci0.baseAddr1 = htoa(BAR1);
    data->config.hdr.pci0.baseAddr2 = htoa(BAR2);
    data->config.hdr.pci0.baseAddr3 = htoa(BAR3);
    data->config.hdr.pci0.baseAddr4 = htoa(BAR4);
    data->config.hdr.pci0.baseAddr5 = htoa(BAR5);
    data->config.hdr.pci0.cardbusCIS = htoa(CardbusCIS);
    data->config.hdr.pci0.subsystemVendorID = htoa(SubsystemVendorID);
    data->config.hdr.pci0.subsystemID = htoa(SubsystemVendorID);
    data->config.hdr.pci0.expansionROM = htoa(ExpansionROM);
    data->config.hdr.pci0.interruptLine = htoa(InterruptLine);
    data->config.hdr.pci0.interruptPin = htoa(InterruptPin);
    data->config.hdr.pci0.minimumGrant = htoa(MinimumGrant);
    data->config.hdr.pci0.maximumLatency = htoa(MaximumLatency);

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
