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
 *
 * Authors: Ali Saidi
 */

/** @file
 * Time of date device implementation
 */
#include <sys/time.h>

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "dev/sparc/dtod.hh"
#include "dev/platform.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

DumbTOD::DumbTOD(Params *p)
    : BasicPioDevice(p)
{
    struct tm tm;
    char *tz;

    pioSize = 0x08;

    parseTime(p->init_time, &tm);
    tz = getenv("TZ");
    setenv("TZ", "", 1);
    tzset();
    todTime = mktime(&tm);
    if (tz)
        setenv("TZ", tz, 1);
    else
        unsetenv("TZ");
    tzset();

    DPRINTFN("Real-time clock set to %s\n", asctime(&tm));
    DPRINTFN("Real-time clock set to %d\n", todTime);
}

Tick
DumbTOD::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 8);

    pkt->allocate();
    pkt->set(todTime);
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
DumbTOD::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(todTime);
}

void
DumbTOD::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(todTime);
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(DumbTOD)

    Param<Addr> pio_addr;
    Param<Tick> pio_latency;
    SimObjectParam<Platform *> platform;
    SimObjectParam<System *> system;
    VectorParam<int> time;

END_DECLARE_SIM_OBJECT_PARAMS(DumbTOD)

BEGIN_INIT_SIM_OBJECT_PARAMS(DumbTOD)

    INIT_PARAM(pio_addr, "Device Address"),
    INIT_PARAM(pio_latency, "Programmed IO latency"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(time, "")

END_INIT_SIM_OBJECT_PARAMS(DumbTOD)

CREATE_SIM_OBJECT(DumbTOD)
{
    DumbTOD::Params *p = new DumbTOD::Params;
    p->name =getInstanceName();
    p->pio_addr = pio_addr;
    p->pio_delay = pio_latency;
    p->platform = platform;
    p->system = system;
    p->init_time = time;
    return new DumbTOD(p);
}

REGISTER_SIM_OBJECT("DumbTOD", DumbTOD)
