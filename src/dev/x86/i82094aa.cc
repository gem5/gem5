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

#include "arch/x86/intmessage.hh"
#include "dev/x86/i82094aa.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

X86ISA::I82094AA::I82094AA(Params *p) : PioDevice(p), IntDev(this),
   latency(p->pio_latency), pioAddr(p->pio_addr)
{
    // This assumes there's only one I/O APIC in the system
    id = sys->getNumCPUs();
    assert(id <= 0xf);
    arbId = id;
    regSel = 0;
    RedirTableEntry entry = 0;
    entry.mask = 1;
    for (int i = 0; i < TableSize; i++) {
        redirTable[i] = entry;
    }
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
    return latency;
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
    return latency;
}

void
X86ISA::I82094AA::writeReg(uint8_t offset, uint32_t value)
{
    if (offset == 0x0) {
        id = bits(value, 27, 24);
    } else if (offset == 0x1) {
        // The IOAPICVER register is read only.
    } else if (offset == 0x2) {
        arbId = bits(value, 27, 24);
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
        if (DTRACE(I82094AA)) {
            switch(entry.deliveryMode) {
              case 0:
                DPRINTF(I82094AA, "Delivery mode is: Fixed.\n");
                break;
              case 1:
                DPRINTF(I82094AA, "Delivery mode is: Lowest Priority.\n");
                break;
              case 2:
                DPRINTF(I82094AA, "Delivery mode is: SMI.\n");
                break;
              case 3:
                fatal("Tried to use reserved delivery mode "
                        "for IO APIC entry %d.\n", line);
                break;
              case 4:
                DPRINTF(I82094AA, "Delivery mode is: NMI.\n");
                break;
              case 5:
                DPRINTF(I82094AA, "Delivery mode is: INIT.\n");
                break;
              case 6:
                fatal("Tried to use reserved delivery mode "
                        "for IO APIC entry %d.\n", line);
                break;
              case 7:
                DPRINTF(I82094AA, "Delivery mode is: ExtINT.\n");
                break;
            }
            DPRINTF(I82094AA, "Vector is %#x.\n", entry.vector);
        }

        TriggerIntMessage message;
        message.destination = entry.dest;
        message.vector = entry.vector;
        message.deliveryMode = entry.deliveryMode;
        message.destMode = entry.destMode;

        if (entry.destMode == 0) {
            DPRINTF(I82094AA,
                    "Sending interrupt to APIC ID %d.\n", entry.dest);
            PacketPtr pkt = buildIntRequest(entry.dest, message);
            if (sys->getMemoryMode() == Enums::timing)
                intPort->sendMessageTiming(pkt, latency);
            else if (sys->getMemoryMode() == Enums::atomic)
                intPort->sendMessageAtomic(pkt);
            else
                panic("Unrecognized memory mode.\n");
        } else {
            DPRINTF(I82094AA, "Sending interrupts to APIC IDs:"
                    "%s%s%s%s%s%s%s%s\n",
                    bits((int)entry.dest, 0) ? " 0": "",
                    bits((int)entry.dest, 1) ? " 1": "",
                    bits((int)entry.dest, 2) ? " 2": "",
                    bits((int)entry.dest, 3) ? " 3": "",
                    bits((int)entry.dest, 4) ? " 4": "",
                    bits((int)entry.dest, 5) ? " 5": "",
                    bits((int)entry.dest, 6) ? " 6": "",
                    bits((int)entry.dest, 7) ? " 7": ""
                    );
            uint8_t dests = entry.dest;
            uint8_t id = 0;
            while(dests) {
                if (dests & 0x1) {
                    PacketPtr pkt = buildIntRequest(id, message);
                    if (sys->getMemoryMode() == Enums::timing)
                        intPort->sendMessageTiming(pkt, latency);
                    else if (sys->getMemoryMode() == Enums::atomic)
                        intPort->sendMessageAtomic(pkt);
                    else
                        panic("Unrecognized memory mode.\n");
                }
                dests >>= 1;
                id++;
            }
        }
    }
}

X86ISA::I82094AA *
I82094AAParams::create()
{
    return new X86ISA::I82094AA(this);
}
