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
 * Authors: Andrew Schultz
 *          Ali Saidi
 */

/* @file
 * PCI Configspace implementation
 */

#include "base/trace.hh"
#include "dev/pciconfigall.hh"
#include "dev/pcireg.h"
#include "dev/platform.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

PciConfigAll::PciConfigAll(Params *p)
    : PioDevice(p)
{
    pioAddr = p->platform->calcConfigAddr(params()->bus,0,0);
}


Tick
PciConfigAll::read(PacketPtr pkt)
{

    pkt->allocate();

    DPRINTF(PciConfigAll, "read  va=%#x size=%d\n", pkt->getAddr(),
            pkt->getSize());

    switch (pkt->getSize()) {
      case sizeof(uint32_t):
         pkt->set<uint32_t>(0xFFFFFFFF);
         break;
      case sizeof(uint16_t):
         pkt->set<uint16_t>(0xFFFF);
         break;
      case sizeof(uint8_t):
         pkt->set<uint8_t>(0xFF);
         break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt->makeAtomicResponse();
    return params()->pio_delay;
}

Tick
PciConfigAll::write(PacketPtr pkt)
{
    panic("Attempting to write to config space on non-existant device\n");
    M5_DUMMY_RETURN
}


void
PciConfigAll::addressRanges(AddrRangeList &range_list)
{
    range_list.clear();
    range_list.push_back(RangeSize(pioAddr, params()->size));
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(PciConfigAll)

    Param<Tick> pio_latency;
    Param<int> bus;
    Param<Addr> size;
    SimObjectParam<Platform *> platform;
    SimObjectParam<System *> system;

END_DECLARE_SIM_OBJECT_PARAMS(PciConfigAll)

BEGIN_INIT_SIM_OBJECT_PARAMS(PciConfigAll)

    INIT_PARAM(pio_latency, "Programmed IO latency"),
    INIT_PARAM(bus, "Bus that this object handles config space for"),
    INIT_PARAM(size, "The size of config space"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM(system, "system object")

END_INIT_SIM_OBJECT_PARAMS(PciConfigAll)

CREATE_SIM_OBJECT(PciConfigAll)
{
    PciConfigAll::Params *p = new PciConfigAll::Params;
    p->pio_delay = pio_latency;
    p->platform = platform;
    p->system = system;
    p->bus = bus;
    p->size = size;

    return new PciConfigAll(p);
}

REGISTER_SIM_OBJECT("PciConfigAll", PciConfigAll)

#endif // DOXYGEN_SHOULD_SKIP_THIS
