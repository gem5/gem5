/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#include "dev/x86/i82094aa.hh"

#include "arch/x86/interrupts.hh"
#include "arch/x86/intmessage.hh"
#include "cpu/base.hh"
#include "debug/I82094AA.hh"
#include "dev/x86/i8259.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

X86ISA::I82094AA::I82094AA(Params *p)
    : BasicPioDevice(p, 20), IntDevice(this, p->int_latency),
      extIntPic(p->external_int_pic), lowestPriorityOffset(0)
{
    // This assumes there's only one I/O APIC in the system and since the apic
    // id is stored in a 8-bit field with 0xff meaning broadcast, the id must
    // be less than 0xff

    assert(p->apic_id < 0xff);
    initialApicId = id = p->apic_id;
    arbId = id;
    regSel = 0;
    RedirTableEntry entry = 0;
    entry.mask = 1;
    for (int i = 0; i < TableSize; i++) {
        redirTable[i] = entry;
        pinStates[i] = false;
    }
}

void
X86ISA::I82094AA::init()
{
    // The io apic must register its address ranges on both its pio port
    // via the piodevice init() function and its int port that it inherited
    // from IntDevice.  Note IntDevice is not a SimObject itself.

    BasicPioDevice::init();
    IntDevice::init();
}

BaseMasterPort &
X86ISA::I82094AA::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "int_master")
        return intMasterPort;
    return BasicPioDevice::getMasterPort(if_name, idx);
}

AddrRangeList
X86ISA::I82094AA::getIntAddrRange() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeEx(x86InterruptAddress(initialApicId, 0),
                             x86InterruptAddress(initialApicId, 0) +
                             PhysAddrAPICRangeSize));
    return ranges;
}

Tick
X86ISA::I82094AA::recvResponse(PacketPtr pkt)
{
    // Packet instantiated calling sendMessage() in signalInterrupt()
    delete pkt->req;
    delete pkt;
    return 0;
}

Tick
X86ISA::I82094AA::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 4);
    Addr offset = pkt->getAddr() - pioAddr;
    switch(offset) {
      case 0:
        pkt->set<uint32_t>(regSel);
        break;
      case 16:
        pkt->set<uint32_t>(readReg(regSel));
        break;
      default:
        panic("Illegal read from I/O APIC.\n");
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
X86ISA::I82094AA::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 4);
    Addr offset = pkt->getAddr() - pioAddr;
    switch(offset) {
      case 0:
        regSel = pkt->get<uint32_t>();
        break;
      case 16:
        writeReg(regSel, pkt->get<uint32_t>());
        break;
      default:
        panic("Illegal write to I/O APIC.\n");
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
X86ISA::I82094AA::writeReg(uint8_t offset, uint32_t value)
{
    if (offset == 0x0) {
        id = bits(value, 31, 24);
    } else if (offset == 0x1) {
        // The IOAPICVER register is read only.
    } else if (offset == 0x2) {
        arbId = bits(value, 31, 24);
    } else if (offset >= 0x10 && offset <= (0x10 + TableSize * 2)) {
        int index = (offset - 0x10) / 2;
        if (offset % 2) {
            redirTable[index].topDW = value;
            redirTable[index].topReserved = 0;
        } else {
            redirTable[index].bottomDW = value;
            redirTable[index].bottomReserved = 0;
        }
    } else {
        warn("Access to undefined I/O APIC register %#x.\n", offset);
    }
    DPRINTF(I82094AA,
            "Wrote %#x to I/O APIC register %#x .\n", value, offset);
}

uint32_t
X86ISA::I82094AA::readReg(uint8_t offset)
{
    uint32_t result = 0;
    if (offset == 0x0) {
        result = id << 24;
    } else if (offset == 0x1) {
        result = ((TableSize - 1) << 16) | APICVersion;
    } else if (offset == 0x2) {
        result = arbId << 24;
    } else if (offset >= 0x10 && offset <= (0x10 + TableSize * 2)) {
        int index = (offset - 0x10) / 2;
        if (offset % 2) {
            result = redirTable[index].topDW;
        } else {
            result = redirTable[index].bottomDW;
        }
    } else {
        warn("Access to undefined I/O APIC register %#x.\n", offset);
    }
    DPRINTF(I82094AA,
            "Read %#x from I/O APIC register %#x.\n", result, offset);
    return result;
}

void
X86ISA::I82094AA::signalInterrupt(int line)
{
    DPRINTF(I82094AA, "Received interrupt %d.\n", line);
    assert(line < TableSize);
    RedirTableEntry entry = redirTable[line];
    if (entry.mask) {
        DPRINTF(I82094AA, "Entry was masked.\n");
        return;
    } else {
        TriggerIntMessage message = 0;
        message.destination = entry.dest;
        if (entry.deliveryMode == DeliveryMode::ExtInt) {
            assert(extIntPic);
            message.vector = extIntPic->getVector();
        } else {
            message.vector = entry.vector;
        }
        message.deliveryMode = entry.deliveryMode;
        message.destMode = entry.destMode;
        message.level = entry.polarity;
        message.trigger = entry.trigger;
        ApicList apics;
        int numContexts = sys->numContexts();
        if (message.destMode == 0) {
            if (message.deliveryMode == DeliveryMode::LowestPriority) {
                panic("Lowest priority delivery mode from the "
                        "IO APIC aren't supported in physical "
                        "destination mode.\n");
            }
            if (message.destination == 0xFF) {
                for (int i = 0; i < numContexts; i++) {
                    apics.push_back(i);
                }
            } else {
                apics.push_back(message.destination);
            }
        } else {
            for (int i = 0; i < numContexts; i++) {
                Interrupts *localApic = sys->getThreadContext(i)->
                    getCpuPtr()->getInterruptController(0);
                if ((localApic->readReg(APIC_LOGICAL_DESTINATION) >> 24) &
                        message.destination) {
                    apics.push_back(localApic->getInitialApicId());
                }
            }
            if (message.deliveryMode == DeliveryMode::LowestPriority &&
                    apics.size()) {
                // The manual seems to suggest that the chipset just does
                // something reasonable for these instead of actually using
                // state from the local APIC. We'll just rotate an offset
                // through the set of APICs selected above.
                uint64_t modOffset = lowestPriorityOffset % apics.size();
                lowestPriorityOffset++;
                ApicList::iterator apicIt = apics.begin();
                while (modOffset--) {
                    apicIt++;
                    assert(apicIt != apics.end());
                }
                int selected = *apicIt;
                apics.clear();
                apics.push_back(selected);
            }
        }
        intMasterPort.sendMessage(apics, message, sys->isTimingMode());
    }
}

void
X86ISA::I82094AA::raiseInterruptPin(int number)
{
    assert(number < TableSize);
    if (!pinStates[number])
        signalInterrupt(number);
    pinStates[number] = true;
}

void
X86ISA::I82094AA::lowerInterruptPin(int number)
{
    assert(number < TableSize);
    pinStates[number] = false;
}

void
X86ISA::I82094AA::serialize(CheckpointOut &cp) const
{
    uint64_t* redirTableArray = (uint64_t*)redirTable;
    SERIALIZE_SCALAR(regSel);
    SERIALIZE_SCALAR(initialApicId);
    SERIALIZE_SCALAR(id);
    SERIALIZE_SCALAR(arbId);
    SERIALIZE_SCALAR(lowestPriorityOffset);
    SERIALIZE_ARRAY(redirTableArray, TableSize);
    SERIALIZE_ARRAY(pinStates, TableSize);
}

void
X86ISA::I82094AA::unserialize(CheckpointIn &cp)
{
    uint64_t redirTableArray[TableSize];
    UNSERIALIZE_SCALAR(regSel);
    UNSERIALIZE_SCALAR(initialApicId);
    UNSERIALIZE_SCALAR(id);
    UNSERIALIZE_SCALAR(arbId);
    UNSERIALIZE_SCALAR(lowestPriorityOffset);
    UNSERIALIZE_ARRAY(redirTableArray, TableSize);
    UNSERIALIZE_ARRAY(pinStates, TableSize);
    for (int i = 0; i < TableSize; i++) {
        redirTable[i] = (RedirTableEntry)redirTableArray[i];
    }
}

X86ISA::I82094AA *
I82094AAParams::create()
{
    return new X86ISA::I82094AA(this);
}
