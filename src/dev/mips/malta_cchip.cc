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

/** @file
 * Emulation of the Malta CChip CSRs
 */

#include "dev/mips/malta_cchip.hh"

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Malta.hh"
#include "dev/mips/malta.hh"
#include "dev/mips/maltareg.h"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "params/MaltaCChip.hh"
#include "sim/system.hh"

namespace gem5
{

MaltaCChip::MaltaCChip(const Params &p)
    : BasicPioDevice(p, 0xfffffff), malta(p.malta)
{
    warn("MaltaCCHIP::MaltaCChip() not implemented.");

    //Put back pointer in malta
    malta->cchip = this;

}

Tick
MaltaCChip::read(PacketPtr pkt)
{
    panic("MaltaCCHIP::read() not implemented.");
    return pioDelay;
}

Tick
MaltaCChip::write(PacketPtr pkt)
{
    panic("MaltaCCHIP::write() not implemented.");
    return pioDelay;
}

void
MaltaCChip::clearIPI(uint64_t ipintr)
{
    panic("MaltaCCHIP::clear() not implemented.");
}

void
MaltaCChip::clearITI(uint64_t itintr)
{
    panic("MaltaCCHIP::clearITI() not implemented.");
}

void
MaltaCChip::reqIPI(uint64_t ipreq)
{
    panic("MaltaCCHIP::reqIPI() not implemented.");
}


void
MaltaCChip::postRTC()
{
    panic("MaltaCCHIP::postRTC() not implemented.");
}

void
MaltaCChip::postIntr(uint32_t interrupt)
{
    uint64_t size = sys->threads.size();
    assert(size <= Malta::Max_CPUs);

    for (int i=0; i < size; i++) {
        //Note: Malta does not use index, but this was added to use the
        //pre-existing implementation
        auto tc = sys->threads[i];
        tc->getCpuPtr()->postInterrupt(tc->threadId(), interrupt, 0);
        DPRINTF(Malta, "posting  interrupt to cpu %d, interrupt %d\n",
                i, interrupt);
   }
}

void
MaltaCChip::clearIntr(uint32_t interrupt)
{
    uint64_t size = sys->threads.size();
    assert(size <= Malta::Max_CPUs);

    for (int i=0; i < size; i++) {
        //Note: Malta does not use index, but this was added to use the
        //pre-existing implementation
        auto tc = sys->threads[i];
        tc->getCpuPtr()->clearInterrupt(tc->threadId(), interrupt, 0);
        DPRINTF(Malta, "clearing interrupt to cpu %d, interrupt %d\n",
                i, interrupt);
   }
}


void
MaltaCChip::serialize(CheckpointOut &cp) const
{
}

void
MaltaCChip::unserialize(CheckpointIn &cp)
{
}

} // namespace gem5
