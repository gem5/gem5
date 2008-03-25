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
 * Authors: Gabe Black
 */

#include "dev/x86/south_bridge/cmos.hh"
#include "mem/packet_access.hh"

Tick
X86ISA::Cmos::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    switch(pkt->getAddr() - addrRange.start)
    {
      case 0x0:
        pkt->set(address);
        break;
      case 0x1:
        pkt->set(readRegister(address));
        break;
      default:
        panic("Read from undefined CMOS port.\n");
    }
    return latency;
}

Tick
X86ISA::Cmos::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    switch(pkt->getAddr() - addrRange.start)
    {
      case 0x0:
        address = pkt->get<uint8_t>();
        break;
      case 0x1:
        writeRegister(address, pkt->get<uint8_t>());
        break;
      default:
        panic("Write to undefined CMOS port.\n");
    }
    return latency;
}

uint8_t
X86ISA::Cmos::readRegister(uint8_t reg)
{
    assert(reg < numRegs);
    if (reg <= 0xD) {
        return rtc.readData(reg);
    } else {
        warn("Reading non-volitile CMOS address %x as %x.\n", reg, regs[reg]);
    }
    return regs[reg];
}

void
X86ISA::Cmos::writeRegister(uint8_t reg, uint8_t val)
{
    assert(reg < numRegs);
    if (reg <= 0xD) {
        rtc.writeData(reg, val);
        return;
    } else {
        warn("Writing non-volitile CMOS address %x with %x.\n", reg, val);
    }
    regs[reg] = val;
}
