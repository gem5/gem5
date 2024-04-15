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
 */

#include "dev/x86/cmos.hh"

#include "base/trace.hh"
#include "debug/CMOS.hh"
#include "dev/x86/intdev.hh"
#include "mem/packet_access.hh"

namespace gem5
{

void
X86ISA::Cmos::X86RTC::handleEvent()
{
    for (auto *wire : intPin) {
        wire->raise();
        // XXX This is a hack.
        wire->lower();
    }
}

Tick
X86ISA::Cmos::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    switch (pkt->getAddr() - pioAddr) {
    case 0x0:
        pkt->setLE(address);
        break;
    case 0x1:
        pkt->setLE(readRegister(address.regNum));
        break;
    default:
        panic("Read from undefined CMOS port.\n");
    }
    pkt->makeAtomicResponse();
    return latency;
}

Tick
X86ISA::Cmos::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    switch (pkt->getAddr() - pioAddr) {
    case 0x0:
        address = pkt->getLE<uint8_t>();
        break;
    case 0x1:
        // Ignore the NMI mask bit since we never try to generate one anyway.
        writeRegister(address.regNum, pkt->getLE<uint8_t>());
        break;
    default:
        panic("Write to undefined CMOS port.\n");
    }
    pkt->makeAtomicResponse();
    return latency;
}

uint8_t
X86ISA::Cmos::readRegister(uint8_t reg)
{
    assert(reg < numRegs);
    uint8_t val;
    if (reg <= 0xD) {
        val = rtc.readData(reg);
        DPRINTF(CMOS, "Reading CMOS RTC reg %x as %x.\n", reg, val);
    } else {
        val = regs[reg];
        DPRINTF(CMOS, "Reading non-volitile CMOS address %x as %x.\n", reg,
                val);
    }
    return val;
}

void
X86ISA::Cmos::writeRegister(uint8_t reg, uint8_t val)
{
    assert(reg < numRegs);
    if (reg <= 0xD) {
        DPRINTF(CMOS, "Writing CMOS RTC reg %x with %x.\n", reg, val);
        rtc.writeData(reg, val);
    } else {
        DPRINTF(CMOS, "Writing non-volitile CMOS address %x with %x.\n", reg,
                val);
        regs[reg] = val;
    }
}

void
X86ISA::Cmos::startup()
{
    rtc.startup();
}

void
X86ISA::Cmos::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(address);
    SERIALIZE_ARRAY(regs, numRegs);

    // Serialize the timer
    rtc.serialize("rtc", cp);
}

void
X86ISA::Cmos::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(address);
    UNSERIALIZE_ARRAY(regs, numRegs);

    // Serialize the timer
    rtc.unserialize("rtc", cp);
}

} // namespace gem5
