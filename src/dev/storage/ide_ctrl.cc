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

#include "base/cprintf.hh"
#include "debug/IdeCtrl.hh"
#include "dev/storage/ide_disk.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/IdeController.hh"
#include "sim/byteswap.hh"

namespace gem5
{

// Bus master IDE registers
enum BMIRegOffset
{
    BMICommand = 0x0,
    BMIStatus = 0x2,
    BMIDescTablePtr = 0x4
};

IdeController::Channel::Channel(std::string new_name, IdeController *new_ctrl,
        bool new_primary) :
    Named(new_name), ctrl(new_ctrl), primary(new_primary)
{
    bmiRegs.reset();
    bmiRegs.status.dmaCap0 = 1;
    bmiRegs.status.dmaCap1 = 1;
}

IdeController::IdeController(const Params &p)
    : PciEndpoint(p), configSpaceRegs(name() + ".config_space_regs"),
    primary(name() + ".primary", this, true),
    secondary(name() + ".secondary", this, false),
    ioShift(p.io_shift), ctrlOffset(p.ctrl_offset)
{
    panic_if(params().disks.size() > 4,
            "IDE controllers support a maximum of 4 devices attached!");

    // Assign the disks to channels
    for (int i = 0; i < params().disks.size(); i++) {
        auto *disk = params().disks[i];
        auto &channel = (i < 2) ? primary : secondary;

        if (!disk)
            continue;

        if (i % 2 == 0)
            channel.setDevice0(disk);
        else
            channel.setDevice1(disk);

        // Arbitrarily set the chunk size to 4K.
        disk->setChannel(&channel, 4 * 1024);
    }

    primary.select(false);
    secondary.select(false);
}

void
IdeController::ConfigSpaceRegs::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(primaryTiming);
    SERIALIZE_SCALAR(secondaryTiming);
    SERIALIZE_SCALAR(deviceTiming);
    SERIALIZE_SCALAR(udmaControl);
    SERIALIZE_SCALAR(udmaTiming);
}

void
IdeController::ConfigSpaceRegs::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(primaryTiming);
    UNSERIALIZE_SCALAR(secondaryTiming);
    UNSERIALIZE_SCALAR(deviceTiming);
    UNSERIALIZE_SCALAR(udmaControl);
    UNSERIALIZE_SCALAR(udmaTiming);
}

void
IdeController::Channel::postInterrupt()
{
    bmiRegs.status.intStatus = 1;
    _pendingInterrupt = true;
    ctrl->postInterrupt(isPrimary());
}

void
IdeController::Channel::clearInterrupt()
{
    bmiRegs.status.intStatus = 0;
    _pendingInterrupt = false;
    ctrl->clearInterrupt(isPrimary());
}

void
IdeController::postInterrupt(bool is_primary)
{
    auto &other = is_primary ? secondary : primary;
    // If an interrupt isn't already posted for the other channel...
    if (!other.pendingInterrupt())
        PciEndpoint::intrPost();
}

void
IdeController::clearInterrupt(bool is_primary)
{
    auto &other = is_primary ? secondary : primary;
    // If the interrupt isn't still needed by the other channel...
    if (!other.pendingInterrupt())
        PciEndpoint::intrClear();
}

Tick
IdeController::readConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC)
        return PciEndpoint::readConfig(pkt);

    size_t size = pkt->getSize();

    configSpaceRegs.read(offset, pkt->getPtr<void>(), size);

    DPRINTF(IdeCtrl, "PCI read offset: %#x size: %d data: %#x\n", offset, size,
            pkt->getUintX(ByteOrder::little));

    pkt->makeAtomicResponse();
    return configDelay;
}


Tick
IdeController::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;

    if (offset < PCI_DEVICE_SPECIFIC)
        return PciEndpoint::writeConfig(pkt);

    size_t size = pkt->getSize();

    DPRINTF(IdeCtrl, "PCI write offset: %#x size: %d data: %#x\n",
            offset, size, pkt->getUintX(ByteOrder::little));

    configSpaceRegs.write(offset, pkt->getConstPtr<void>(), size);

    pkt->makeAtomicResponse();
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

    if (selected() == NULL) {
        assert(size == sizeof(uint8_t));
        *data = 0;
    } else if (read) {
        selected()->readCommand(offset, size, data);
    } else {
        selected()->writeCommand(offset, size, data);
    }
}

void
IdeController::Channel::accessControl(Addr offset,
        int size, uint8_t *data, bool read)
{
    if (selected() == NULL) {
        assert(size == sizeof(uint8_t));
        *data = 0;
    } else if (read) {
        selected()->readControl(offset, size, data);
    } else {
        selected()->writeControl(offset, size, data);
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
                    if (selected() == NULL)
                        panic("DMA start for disk which does not exist\n");

                    if (oldVal.startStop) {
                        DPRINTF(IdeCtrl, "Stopping DMA transfer\n");
                        bmiRegs.status.active = 0;

                        selected()->abortDma();
                    } else {
                        DPRINTF(IdeCtrl, "Starting DMA transfer\n");
                        bmiRegs.status.active = 1;

                        selected()->startDma(letoh(bmiRegs.bmidtp));
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
                    // work as intended, so we need to use this temporary
                    // variable to get around the bug.
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
                    size != sizeof(uint32_t)) {
                panic("IDE controller write of invalid size: %x\n", size);
            }
            memcpy((uint8_t *)&bmiRegs + offset, data, size);
        }
    }
}

void
IdeController::dispatchAccess(PacketPtr pkt, bool read)
{
    if (pkt->getSize() != 1 && pkt->getSize() != 2 && pkt->getSize() !=4)
         panic("Bad IDE read size: %d\n", pkt->getSize());

    Addr addr = pkt->getAddr();
    int size = pkt->getSize();
    uint8_t *dataPtr = pkt->getPtr<uint8_t>();

    int bar_num;
    Addr offset;
    panic_if(!getBAR(addr, bar_num, offset),
        "IDE controller access to invalid address: %#x.", addr);

    switch (bar_num) {
      case 0:
        // linux may have shifted the address by ioShift,
        // here we shift it back, similarly for ctrlOffset.
        offset >>= ioShift;
        primary.accessCommand(offset, size, dataPtr, read);
        break;
      case 1:
        offset += ctrlOffset;
        primary.accessControl(offset, size, dataPtr, read);
        break;
      case 2:
        secondary.accessCommand(offset, size, dataPtr, read);
        break;
      case 3:
        secondary.accessControl(offset, size, dataPtr, read);
        break;
      case 4:
        {
            PciCommandRegister command = letoh(config().command);
            if (!read && !command.busMaster)
                return;

            if (offset < sizeof(Channel::BMIRegs)) {
                primary.accessBMI(offset, size, dataPtr, read);
            } else {
                offset -= sizeof(Channel::BMIRegs);
                secondary.accessBMI(offset, size, dataPtr, read);
            }
        }
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

void
IdeController::Channel::setDmaComplete()
{
    bmiRegs.command.startStop = 0;
    bmiRegs.status.active = 0;
    bmiRegs.status.intStatus = 1;
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
    // Serialize the PciEndpoint base class
    PciEndpoint::serialize(cp);

    // Serialize channels
    primary.serialize("primary", cp);
    secondary.serialize("secondary", cp);

    // Serialize config registers
    configSpaceRegs.serialize(cp);
}

void
IdeController::Channel::serialize(const std::string &base,
                                  CheckpointOut &cp) const
{
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
    // Unserialize the PciEndpoint base class
    PciEndpoint::unserialize(cp);

    // Unserialize channels
    primary.unserialize("primary", cp);
    secondary.unserialize("secondary", cp);

    // Unserialize config registers
    configSpaceRegs.unserialize(cp);
}

void
IdeController::Channel::unserialize(const std::string &base, CheckpointIn &cp)
{
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

} // namespace gem5
