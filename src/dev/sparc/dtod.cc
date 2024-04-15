/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 * Time of date device implementation
 */
#include "dev/sparc/dtod.hh"

#include <sys/time.h>

#include <deque>
#include <string>
#include <vector>

#include "base/time.hh"
#include "base/trace.hh"
#include "dev/platform.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "sim/system.hh"

namespace gem5
{

DumbTOD::DumbTOD(const Params &p) : BasicPioDevice(p, 0x08)
{
    struct tm tm = p.time;
    todTime = mkutctime(&tm);

    DPRINTFN("Real-time clock set to %s\n", asctime(&tm));
    DPRINTFN("Real-time clock set to %d\n", todTime);
}

Tick
DumbTOD::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 8);

    pkt->setBE(todTime);
    todTime += 1000;

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
DumbTOD::write(PacketPtr pkt)
{
    panic("Dumb tod device doesn't support writes\n");
}

void
DumbTOD::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(todTime);
}

void
DumbTOD::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(todTime);
}

} // namespace gem5
