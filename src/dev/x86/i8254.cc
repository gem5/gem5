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
 */

#include "dev/x86/i8254.hh"

#include "debug/I8254.hh"
#include "dev/x86/intdev.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

void
X86ISA::I8254::counterInterrupt(unsigned int num)
{
    DPRINTF(I8254, "Interrupt from counter %d.\n", num);
    if (num == 0) {
        for (auto *wire: intPin) {
            wire->raise();
            //XXX This is a hack.
            wire->lower();
        }
    }
}

Tick
X86ISA::I8254::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    Addr offset = pkt->getAddr() - pioAddr;
    if (offset < 3) {
        pkt->setLE(pit.readCounter(offset));
    } else if (offset == 3) {
        pkt->setLE(uint8_t(-1));
    } else {
        panic("Read from undefined i8254 register.\n");
    }
    pkt->makeAtomicResponse();
    return latency;
}

Tick
X86ISA::I8254::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    Addr offset = pkt->getAddr() - pioAddr;
    if (offset < 3) {
        pit.writeCounter(offset, pkt->getLE<uint8_t>());
    } else if (offset == 3) {
        pit.writeControl(pkt->getLE<uint8_t>());
    } else {
        panic("Write to undefined i8254 register.\n");
    }
    pkt->makeAtomicResponse();
    return latency;
}

void
X86ISA::I8254::serialize(CheckpointOut &cp) const
{
    pit.serialize("pit", cp);
}

void
X86ISA::I8254::unserialize(CheckpointIn &cp)
{
    pit.unserialize("pit", cp);
}

void
X86ISA::I8254::startup()
{
    pit.startup();
}

X86ISA::I8254 *
I8254Params::create()
{
    return new X86ISA::I8254(this);
}
