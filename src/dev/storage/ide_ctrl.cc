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

#include "dev/storage/ide_ctrl.hh"

#include <string>

#include "cpu/intr_control.hh"
#include "debug/IdeCtrl.hh"
#include "dev/storage/ide_disk.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/IdeController.hh"
#include "sim/byteswap.hh"

// clang complains about std::set being overloaded with Packet::set if
// we open up the entire namespace std
using std::string;

// Bus master IDE registers
enum BMIRegOffset {
    BMICommand = 0x0,
    BMIStatus = 0x2,
    BMIDescTablePtr = 0x4
};

// PCI config space registers
enum ConfRegOffset {
    PrimaryTiming = 0x40,
    SecondaryTiming = 0x42,
    DeviceTiming = 0x44,
    UDMAControl = 0x48,
    UDMATiming = 0x4A,
    IDEConfig = 0x54
};

static const uint16_t timeRegWithDecodeEn = 0x8000;

IdeController::Channel::Channel(
        string newName, Addr _cmdSize, Addr _ctrlSize) :
    _name(newName),
    cmdAddr(0), cmdSize(_cmdSize), ctrlAddr(0), ctrlSize(_ctrlSize),
    device0(NULL), device1(NULL), selected(NULL)
{
    bmiRegs.reset();
    bmiRegs.status.dmaCap0 = 1;
    bmiRegs.status.dmaCap1 = 1;
}

IdeController::Channel::~Channel()
{
}

IdeController::IdeController(Params *p)
    : PciDevice(p), primary(name() + ".primary", BARSize[0], BARSize[1]),
    secondary(name() + ".secondary", BARSize[2], BARSize[3]),
    bmiAddr(0), bmiSize(BARSize[4]),
    primaryTiming(htole(timeRegWithDecodeEn)),
    secondaryTiming(htole(timeRegWithDecodeEn)),
    deviceTiming(0), udmaControl(0), udmaTiming(0), ideConfig(0),
    ioEnabled(false), bmEnabled(false),
    ioShift(p->io_shift), ctrlOffset(p->ctrl_offset)
{

    // Assign the disks to channels
    for (int i = 0; i < params()->disks.size(); i++) {
        if (!params()->disks[i])
            continue;
        switch (i) {
          case 0:
            primary.device0 = params()->disks[0];
            break;
          case 1:
            primary.device1 = params()->disks[1];
            break;
          case 2:
            secondary.device0 = params()->disks[2];
            break;
          case 3:
            secondary.device1 = params()->disks[3];
            break;
          default:
            panic("IDE controllers support a maximum "
                  "of 4 devices attached!\n");
        }
        params()->disks[i]->setController(this, sys->getPageBytes());
    }

    primary.select(false);
    secondary.select(false);

    if ((BARAddrs[0] & ~BAR_IO_MASK) && (!legacyIO[0] || ioShift)) {
        primary.cmdAddr = BARAddrs[0];  primary.cmdSize = BARSize[0];
        primary.ctrlAddr = BARAddrs[1]; primary.ctrlSize = BARSize[1];
    }
    if ((BARAddrs[2] & ~BAR_IO_MASK) && (!legacyIO[2] || ioShift)) {
        secondary.cmdAddr = BARAddrs[2];  secondary.cmdSize = BARSize[2];
        secondary.ctrlAddr = BARAddrs[3]; secondary.ctrlSize = BARSize[3];
    }

    ioEnabled = (config.command & htole(PCI_CMD_IOSE));
    bmEnabled = (config.command & htole(PCI_CMD_BME));
}

bool
IdeController::isDiskSelected(IdeDisk *diskPtr)
{
    return (primary.selected == diskPtr || secondary.selected == diskPtr);
}

void
IdeController::intrPost()
{
    primary.bmiRegs.status.intStatus = 1;
    PciDevice::intrPost();
}

void
IdeController::setDmaComplete(IdeDisk *disk)
{
    Channel *channel;
    if (disk == primary.device0 || disk == primary.device1) {
        channel = &primary;
    } else if (disk == secondary.device0 || disk == secondary.device1) {
        channel = &secondary;
    } else {
        panic("Unable to find disk based on pointer %#x\n", disk);
    }

    channel->bmiRegs.command.startStop = 0;
    channel->bmiRegs.status.active = 0;
    channel->bmiRegs.status.intStatus = 1;
}

Tick
IdeController::readConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC) {
        return PciDevice::readConfig(pkt);
    }

    switch (pkt->getSize()) {
      case sizeof(uint8_t):
        switch (offset) {
          case DeviceTiming:
            pkt->setLE<uint8_t>(deviceTiming);
            break;
          case UDMAControl:
            pkt->setLE<uint8_t>(udmaControl);
            break;
          case PrimaryTiming + 1:
            pkt->setLE<uint8_t>(bits(htole(primaryTiming), 15, 8));
            break;
          case SecondaryTiming + 1:
            pkt->setLE<uint8_t>(bits(htole(secondaryTiming), 15, 8));
            break;
          case IDEConfig:
            pkt->setLE<uint8_t>(bits(htole(ideConfig), 7, 0));
            break;
          case IDEConfig + 1:
            pkt->setLE<uint8_t>(bits(htole(ideConfig), 15, 8));
            break;
          default:
            panic("Invalid PCI configuration read for size 1 at offset: %#x!\n",
                    offset);
        }
        DPRINTF(IdeCtrl, "PCI read offset: %#x size: 1 data: %#x\n", offset,
                (uint32_t)pkt->getLE<uint8_t>());
        break;
      case sizeof(uint16_t):
        switch (offset) {
          case UDMAControl:
            pkt->setLE<uint16_t>(udmaControl);
            break;
          case PrimaryTiming:
            pkt->setLE<uint16_t>(primaryTiming);
            break;
          case SecondaryTiming:
            pkt->setLE<uint16_t>(secondaryTiming);
            break;
          case UDMATiming:
            pkt->setLE<uint16_t>(udmaTiming);
            break;
          case IDEConfig:
            pkt->setLE<uint16_t>(ideConfig);
            break;
          default:
            panic("Invalid PCI configuration read for size 2 offset: %#x!\n",
                    offset);
        }
        DPRINTF(IdeCtrl, "PCI read offset: %#x size: 2 data: %#x\n", offset,
                (uint32_t)pkt->getLE<uint16_t>());
        break;
      case sizeof(uint32_t):
        switch (offset) {
          case PrimaryTiming:
            pkt->setLE<uint32_t>(primaryTiming);
            break;
          case IDEConfig:
            pkt->setLE<uint32_t>(ideConfig);
            break;
          default:
            panic("No 32bit reads implemented for this device.");
        }
        DPRINTF(IdeCtrl, "PCI read offset: %#x size: 4 data: %#x\n", offset,
                (uint32_t)pkt->getLE<uint32_t>());
        break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->makeAtomicResponse();
    return configDelay;
}


Tick
IdeController::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC) {
        PciDevice::writeConfig(pkt);
    } else {
        switch (pkt->getSize()) {
          case sizeof(uint8_t):
            switch (offset) {
              case DeviceTiming:
                deviceTiming = pkt->getLE<uint8_t>();
                break;
              case UDMAControl:
                udmaControl = pkt->getLE<uint8_t>();
                break;
              case IDEConfig:
                replaceBits(ideConfig, 7, 0, pkt->getLE<uint8_t>());
                break;
              case IDEConfig + 1:
                replaceBits(ideConfig, 15, 8, pkt->getLE<uint8_t>());
                break;
              default:
                panic("Invalid PCI configuration write "
                        "for size 1 offset: %#x!\n", offset);
            }
            DPRINTF(IdeCtrl, "PCI write offset: %#x size: 1 data: %#x\n",
                    offset, (uint32_t)pkt->getLE<uint8_t>());
            break;
          case sizeof(uint16_t):
            switch (offset) {
              case UDMAControl:
                udmaControl = pkt->getLE<uint16_t>();
                break;
              case PrimaryTiming:
                primaryTiming = pkt->getLE<uint16_t>();
                break;
              case SecondaryTiming:
                secondaryTiming = pkt->getLE<uint16_t>();
                break;
              case UDMATiming:
                udmaTiming = pkt->getLE<uint16_t>();
                break;
              case IDEConfig:
                ideConfig = pkt->getLE<uint16_t>();
                break;
              default:
                panic("Invalid PCI configuration write "
                        "for size 2 offset: %#x!\n",
                        offset);
            }
            DPRINTF(IdeCtrl, "PCI write offset: %#x size: 2 data: %#x\n",
                    offset, (uint32_t)pkt->getLE<uint16_t>());
            break;
          case sizeof(uint32_t):
            switch (offset) {
              case PrimaryTiming:
                primaryTiming = pkt->getLE<uint32_t>();
                break;
              case IDEConfig:
                ideConfig = pkt->getLE<uint32_t>();
                break;
              default:
                panic("Write of unimplemented PCI config. register: %x\n", offset);
            }
            break;
          default:
            panic("invalid access size(?) for PCI configspace!\n");
        }
        pkt->makeAtomicResponse();
    }

    /* Trap command register writes and enable IO/BM as appropriate as well as
     * BARs. */
    switch(offset) {
      case PCI0_BASE_ADDR0:
        if (BARAddrs[0] != 0)
            primary.cmdAddr = BARAddrs[0];
        break;

      case PCI0_BASE_ADDR1:
        if (BARAddrs[1] != 0)
            primary.ctrlAddr = BARAddrs[1];
        break;

      case PCI0_BASE_ADDR2:
        if (BARAddrs[2] != 0)
            secondary.cmdAddr = BARAddrs[2];
        break;

      case PCI0_BASE_ADDR3:
        if (BARAddrs[3] != 0)
            secondary.ctrlAddr = BARAddrs[3];
        break;

      case PCI0_BASE_ADDR4:
        if (BARAddrs[4] != 0)
            bmiAddr = BARAddrs[4];
        break;

      case PCI_COMMAND:
        DPRINTF(IdeCtrl, "Writing to PCI Command val: %#x\n", config.command);
        ioEnabled = (config.command & htole(PCI_CMD_IOSE));
        bmEnabled = (config.command & htole(PCI_CMD_BME));
        break;
    }
    return configDelay;
}

void
IdeController::Channel::accessCommand(Addr offset,
        int size, uint8_t *data, bool read)
{
    const Addr SelectOffset = 6;
    const uint8_t SelectDevBit = 0x10;

    if (!read && offset == SelectOffset)
        select(*data & SelectDevBit);

    if (selected == NULL) {
        assert(size == sizeof(uint8_t));
        *data = 0;
    } else if (read) {
        selected->readCommand(offset, size, data);
    } else {
        selected->writeCommand(offset, size, data);
    }
}

void
IdeController::Channel::accessControl(Addr offset,
        int size, uint8_t *data, bool read)
{
    if (selected == NULL) {
        assert(size == sizeof(uint8_t));
        *data = 0;
    } else if (read) {
        selected->readControl(offset, size, data);
    } else {
        selected->writeControl(offset, size, data);
    }
}

void
IdeController::Channel::accessBMI(Addr offset,
        int size, uint8_t *data, bool read)
{
    assert(offset + size <= sizeof(BMIRegs));
    if (read) {
        memcpy(data, (uint8_t *)&bmiRegs + offset, size);
    } else {
        switch (offset) {
          case BMICommand:
            {
                if (size != sizeof(uint8_t))
                    panic("Invalid BMIC write size: %x\n", size);

                BMICommandReg oldVal = bmiRegs.command;
                BMICommandReg newVal = *data;

                // if a DMA transfer is in progress, R/W control cannot change
                if (oldVal.startStop && oldVal.rw != newVal.rw)
                    oldVal.rw = newVal.rw;

                if (oldVal.startStop != newVal.startStop) {
                    if (selected == NULL)
                        panic("DMA start for disk which does not exist\n");

                    if (oldVal.startStop) {
                        DPRINTF(IdeCtrl, "Stopping DMA transfer\n");
                        bmiRegs.status.active = 0;

                        selected->abortDma();
                    } else {
                        DPRINTF(IdeCtrl, "Starting DMA transfer\n");
                        bmiRegs.status.active = 1;

                        selected->startDma(letoh(bmiRegs.bmidtp));
                    }
                }

                bmiRegs.command = newVal;
            }
            break;
          case BMIStatus:
            {
                if (size != sizeof(uint8_t))
                    panic("Invalid BMIS write size: %x\n", size);

                BMIStatusReg oldVal = bmiRegs.status;
                BMIStatusReg newVal = *data;

                // the BMIDEA bit is read only
                newVal.active = oldVal.active;

                // to reset (set 0) IDEINTS and IDEDMAE, write 1 to each
                if ((oldVal.intStatus == 1) && (newVal.intStatus == 1)) {
                    newVal.intStatus = 0; // clear the interrupt?
                } else {
                    // Assigning two bitunion fields to each other does not
                    // work as intended, so we need to use this temporary variable
                    // to get around the bug.
                    uint8_t tmp = oldVal.intStatus;
                    newVal.intStatus = tmp;
                }
                if ((oldVal.dmaError == 1) && (newVal.dmaError == 1)) {
                    newVal.dmaError = 0;
                } else {
                    uint8_t tmp = oldVal.dmaError;
                    newVal.dmaError = tmp;
                }

                bmiRegs.status = newVal;
            }
            break;
          case BMIDescTablePtr:
            if (size != sizeof(uint32_t))
                panic("Invalid BMIDTP write size: %x\n", size);
            bmiRegs.bmidtp = htole(*(uint32_t *)data & ~0x3);
            break;
          default:
            if (size != sizeof(uint8_t) && size != sizeof(uint16_t) &&
                    size != sizeof(uint32_t))
                panic("IDE controller write of invalid write size: %x\n", size);
            memcpy((uint8_t *)&bmiRegs + offset, data, size);
        }
    }
}

void
IdeController::dispatchAccess(PacketPtr pkt, bool read)
{
    if (pkt->getSize() != 1 && pkt->getSize() != 2 && pkt->getSize() !=4)
         panic("Bad IDE read size: %d\n", pkt->getSize());

    if (!ioEnabled) {
        pkt->makeAtomicResponse();
        DPRINTF(IdeCtrl, "io not enabled\n");
        return;
    }

    Addr addr = pkt->getAddr();
    int size = pkt->getSize();
    uint8_t *dataPtr = pkt->getPtr<uint8_t>();

    if (addr >= primary.cmdAddr &&
            addr < (primary.cmdAddr + primary.cmdSize)) {
        addr -= primary.cmdAddr;
        // linux may have shifted the address by ioShift,
        // here we shift it back, similarly for ctrlOffset.
        addr >>= ioShift;
        primary.accessCommand(addr, size, dataPtr, read);
    } else if (addr >= primary.ctrlAddr &&
               addr < (primary.ctrlAddr + primary.ctrlSize)) {
        addr -= primary.ctrlAddr;
        addr += ctrlOffset;
        primary.accessControl(addr, size, dataPtr, read);
    } else if (addr >= secondary.cmdAddr &&
               addr < (secondary.cmdAddr + secondary.cmdSize)) {
        addr -= secondary.cmdAddr;
        secondary.accessCommand(addr, size, dataPtr, read);
    } else if (addr >= secondary.ctrlAddr &&
               addr < (secondary.ctrlAddr + secondary.ctrlSize)) {
        addr -= secondary.ctrlAddr;
        secondary.accessControl(addr, size, dataPtr, read);
    } else if (addr >= bmiAddr && addr < (bmiAddr + bmiSize)) {
        if (!read && !bmEnabled)
            return;
        addr -= bmiAddr;
        if (addr < sizeof(Channel::BMIRegs)) {
            primary.accessBMI(addr, size, dataPtr, read);
        } else {
            addr -= sizeof(Channel::BMIRegs);
            secondary.accessBMI(addr, size, dataPtr, read);
        }
    } else {
        panic("IDE controller access to invalid address: %#x\n", addr);
    }

#ifndef NDEBUG
    uint32_t data;
    if (pkt->getSize() == 1)
        data = pkt->getLE<uint8_t>();
    else if (pkt->getSize() == 2)
        data = pkt->getLE<uint16_t>();
    else
        data = pkt->getLE<uint32_t>();
    DPRINTF(IdeCtrl, "%s from offset: %#x size: %#x data: %#x\n",
            read ? "Read" : "Write", pkt->getAddr(), pkt->getSize(), data);
#endif

    pkt->makeAtomicResponse();
}

Tick
IdeController::read(PacketPtr pkt)
{
    dispatchAccess(pkt, true);
    return pioDelay;
}

Tick
IdeController::write(PacketPtr pkt)
{
    dispatchAccess(pkt, false);
    return pioDelay;
}

void
IdeController::serialize(CheckpointOut &cp) const
{
    // Serialize the PciDevice base class
    PciDevice::serialize(cp);

    // Serialize channels
    primary.serialize("primary", cp);
    secondary.serialize("secondary", cp);

    // Serialize config registers
    SERIALIZE_SCALAR(primaryTiming);
    SERIALIZE_SCALAR(secondaryTiming);
    SERIALIZE_SCALAR(deviceTiming);
    SERIALIZE_SCALAR(udmaControl);
    SERIALIZE_SCALAR(udmaTiming);
    SERIALIZE_SCALAR(ideConfig);

    // Serialize internal state
    SERIALIZE_SCALAR(ioEnabled);
    SERIALIZE_SCALAR(bmEnabled);
    SERIALIZE_SCALAR(bmiAddr);
    SERIALIZE_SCALAR(bmiSize);
}

void
IdeController::Channel::serialize(const std::string &base,
                                  CheckpointOut &cp) const
{
    paramOut(cp, base + ".cmdAddr", cmdAddr);
    paramOut(cp, base + ".cmdSize", cmdSize);
    paramOut(cp, base + ".ctrlAddr", ctrlAddr);
    paramOut(cp, base + ".ctrlSize", ctrlSize);
    uint8_t command = bmiRegs.command;
    paramOut(cp, base + ".bmiRegs.command", command);
    paramOut(cp, base + ".bmiRegs.reserved0", bmiRegs.reserved0);
    uint8_t status = bmiRegs.status;
    paramOut(cp, base + ".bmiRegs.status", status);
    paramOut(cp, base + ".bmiRegs.reserved1", bmiRegs.reserved1);
    paramOut(cp, base + ".bmiRegs.bmidtp", bmiRegs.bmidtp);
    paramOut(cp, base + ".selectBit", selectBit);
}

void
IdeController::unserialize(CheckpointIn &cp)
{
    // Unserialize the PciDevice base class
    PciDevice::unserialize(cp);

    // Unserialize channels
    primary.unserialize("primary", cp);
    secondary.unserialize("secondary", cp);

    // Unserialize config registers
    UNSERIALIZE_SCALAR(primaryTiming);
    UNSERIALIZE_SCALAR(secondaryTiming);
    UNSERIALIZE_SCALAR(deviceTiming);
    UNSERIALIZE_SCALAR(udmaControl);
    UNSERIALIZE_SCALAR(udmaTiming);
    UNSERIALIZE_SCALAR(ideConfig);

    // Unserialize internal state
    UNSERIALIZE_SCALAR(ioEnabled);
    UNSERIALIZE_SCALAR(bmEnabled);
    UNSERIALIZE_SCALAR(bmiAddr);
    UNSERIALIZE_SCALAR(bmiSize);
}

void
IdeController::Channel::unserialize(const std::string &base, CheckpointIn &cp)
{
    paramIn(cp, base + ".cmdAddr", cmdAddr);
    paramIn(cp, base + ".cmdSize", cmdSize);
    paramIn(cp, base + ".ctrlAddr", ctrlAddr);
    paramIn(cp, base + ".ctrlSize", ctrlSize);
    uint8_t command;
    paramIn(cp, base +".bmiRegs.command", command);
    bmiRegs.command = command;
    paramIn(cp, base + ".bmiRegs.reserved0", bmiRegs.reserved0);
    uint8_t status;
    paramIn(cp, base + ".bmiRegs.status", status);
    bmiRegs.status = status;
    paramIn(cp, base + ".bmiRegs.reserved1", bmiRegs.reserved1);
    paramIn(cp, base + ".bmiRegs.bmidtp", bmiRegs.bmidtp);
    paramIn(cp, base + ".selectBit", selectBit);
    select(selectBit);
}

IdeController *
IdeControllerParams::create()
{
    return new IdeController(this);
}
