/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
#include "dev/pciareg.h"
#include "dev/scsi_ctrl.hh"
#include "dev/pcidev.hh"
#include "dev/pciconfigall.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"
#include "sim/param.hh"
#include "sim/universe.hh"

using namespace std;

PciDev::PciDev(const string &name, PCIConfigAll *cf, uint32_t bus,
               uint32_t dev, uint32_t func)
    : MMapDevice(name), ConfigSpace(cf), Bus(bus), Device(dev), Function(func)
{
    memset(config.data, 0, sizeof(config.data));

    // Setup pointer in config space to point to this entry
    if(cf->devices[dev][func] != NULL)
        panic("Two PCI devices occuping same dev: %#x func: %#x", dev, func);
    else
        cf->devices[dev][func] = this;
}


void
PciDev::ReadConfig(int offset, int size, uint8_t *data)
{
    switch(size) {
        case sizeof(uint32_t):
            memcpy((uint32_t*)data, config.data + offset, sizeof(uint32_t));
            DPRINTF(PCIDEV, "read device: %#x function: %#x register: %#x data: %#x\n",
                Device, Function, offset, *(uint32_t*)(config.data + offset));
            break;
        case sizeof(uint16_t):
            memcpy((uint16_t*)data, config.data + offset, sizeof(uint16_t));
            DPRINTF(PCIDEV, "read device: %#x function: %#x register: %#x data: %#x\n",
                Device, Function, offset, *(uint16_t*)(config.data + offset));
            break;
        case sizeof(uint8_t):
            memcpy((uint8_t*)data, config.data + offset, sizeof(uint8_t));
            printf("data: %#x\n", *(uint8_t*)(config.data + offset));
            DPRINTF(PCIDEV, "read device: %#x function: %#x register: %#x data: %#x\n",
                Device, Function, offset, *(uint8_t*)(config.data + offset));
            break;
        default:
            panic("Invalid Read Size");
    }
}


void
PciDev::WriteConfig(int offset, int size, uint32_t data)
{
    union {
        uint8_t byte_value;
        uint16_t half_value;
        uint32_t word_value;
    };
    word_value = data;

    DPRINTF(PCIDEV, "write device: %#x function: %#x register: %#x size: %#x data: %#x\n",
                Device, Function, offset, size, word_value);

    switch (size) {
      case sizeof(uint8_t): // 1-byte access
        switch (offset) {
          case PCI0_INTERRUPT_LINE:
          case PCI_CACHE_LINE_SIZE:
            *(uint8_t *)&config.data[offset] = byte_value;
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
            *(uint16_t *)&config.data[offset] = half_value;
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
              // Writing 0xffffffff to a BAR tells the card to set the value of the bar
              // to size of memory it needs
              if (word_value == 0xffffffff) {
                 // This is I/O Space, bottom two bits are read only
                  if(config.data[offset] & 0x1) {
                    *(uint32_t *)&config.data[offset] =
                        ~(BARSize[offset-PCI0_BASE_ADDR0] - 1) | (config.data[offset] & 0x3);
                  } else {
                  // This is memory space, bottom four bits are read only
                    *(uint32_t *)&config.data[offset] =
                        ~(BARSize[(offset-PCI0_BASE_ADDR0)>>2] - 1) | (config.data[offset] & 0xF);
                  }


              } else {
                  // This is I/O Space, bottom two bits are read only
                  if(config.data[offset] & 0x1) {
                    *(uint32_t *)&config.data[offset] = (word_value & ~0x3) |
                                                        (config.data[offset] & 0x3);
                  } else {
                  // This is memory space, bottom four bits are read only
                    *(uint32_t *)&config.data[offset] = (word_value & ~0xF) |
                                                        (config.data[offset] & 0xF);
                  }
              }
            break;
          case PCI0_ROM_BASE_ADDR:
                    if (word_value == 0xfffffffe)
                        *(uint32_t *)&config.data[offset] = 0xffffffff;
                    else
                        *(uint32_t *)&config.data[offset] = word_value;
                break;
          case PCI_COMMAND:
                // This could also clear some of the error bits in the Status register
                // However they should never get set, so lets ignore it for now
                *(uint16_t *)&config.data[offset] = half_value;
                break;


          default:
            panic("writing to a read only register");
        }
        break;
    }

}

void
PciDev::serialize(ostream &os)
{
    SERIALIZE_ARRAY(config.data, 64);
}

void
PciDev::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(config.data, 64);
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(PciDev)

    Param<int> VendorID;
    Param<int> DeviceID;
    Param<int> Command;
    Param<int> Status;
    Param<int> Revision;
    Param<int> ProgIF;
    Param<int> SubClassCode;
    Param<int> ClassCode;
    Param<int> CacheLineSize;
    Param<int> LatencyTimer;
    Param<int> HeaderType;
    Param<int> BIST;
    Param<uint32_t> BAR0;
    Param<uint32_t> BAR1;
    Param<uint32_t> BAR2;
    Param<uint32_t> BAR3;
    Param<uint32_t> BAR4;
    Param<uint32_t> BAR5;
    Param<uint32_t> CardbusCIS;
    Param<int> SubsystemVendorID;
    Param<int> SubsystemID;
    Param<uint32_t> ExpansionROM;
    Param<int> InterruptLine;
    Param<int> InterruptPin;
    Param<int> MinimumGrant;
    Param<int> MaximumLatency;
    Param<uint32_t> BAR0Size;
    Param<uint32_t> BAR1Size;
    Param<uint32_t> BAR2Size;
    Param<uint32_t> BAR3Size;
    Param<uint32_t> BAR4Size;
    Param<uint32_t> BAR5Size;

    SimObjectParam<MemoryController *> mmu;
    SimObjectParam<PCIConfigAll*> cf;
    Param<Addr> addr;
    Param<Addr> mask;
    Param<uint32_t> bus;
    Param<uint32_t> device;
    Param<uint32_t> func;


END_DECLARE_SIM_OBJECT_PARAMS(PciDev)

BEGIN_INIT_SIM_OBJECT_PARAMS(PciDev)

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
    INIT_PARAM_DFLT(BAR5Size, "Base Address Register 5 Size", 0x00),

    INIT_PARAM(cf, "Pointer to Configspace device"),
    INIT_PARAM(bus, "PCI Bus Number"),
    INIT_PARAM(device, "PCI Device number"),
    INIT_PARAM(func, "PCI Function Number")


END_INIT_SIM_OBJECT_PARAMS(PciDev)

CREATE_SIM_OBJECT(PciDev)
{
    PciDev *dev = new PciDev(getInstanceName(), cf, bus, device, func);

    dev->config.hdr.vendor = VendorID;
    dev->config.hdr.device = DeviceID;
    dev->config.hdr.command = Command;
    dev->config.hdr.status = Status;
    dev->config.hdr.revision = Revision;
    dev->config.hdr.progIF = ProgIF;
    dev->config.hdr.subClassCode = SubClassCode;
    dev->config.hdr.classCode = ClassCode;
    dev->config.hdr.cacheLineSize = CacheLineSize;
    dev->config.hdr.latencyTimer = LatencyTimer;
    dev->config.hdr.headerType = HeaderType;
    dev->config.hdr.bist = BIST;

    dev->config.hdr.pci0.baseAddr0 = BAR0;
    dev->config.hdr.pci0.baseAddr1 = BAR1;
    dev->config.hdr.pci0.baseAddr2 = BAR2;
    dev->config.hdr.pci0.baseAddr3 = BAR3;
    dev->config.hdr.pci0.baseAddr4 = BAR4;
    dev->config.hdr.pci0.baseAddr5 = BAR5;
    dev->config.hdr.pci0.cardbusCIS = CardbusCIS;
    dev->config.hdr.pci0.subsystemVendorID = SubsystemVendorID;
    dev->config.hdr.pci0.subsystemID = SubsystemVendorID;
    dev->config.hdr.pci0.expansionROM = ExpansionROM;
    dev->config.hdr.pci0.interruptLine = InterruptLine;
    dev->config.hdr.pci0.interruptPin = InterruptPin;
    dev->config.hdr.pci0.minimumGrant = MinimumGrant;
    dev->config.hdr.pci0.maximumLatency = MaximumLatency;

    dev->BARSize[0] = BAR0Size;
    dev->BARSize[1] = BAR1Size;
    dev->BARSize[2] = BAR2Size;
    dev->BARSize[3] = BAR3Size;
    dev->BARSize[4] = BAR4Size;
    dev->BARSize[5] = BAR5Size;

    return dev;
}

REGISTER_SIM_OBJECT("PciDev", PciDev)
