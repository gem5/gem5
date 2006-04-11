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
 * Isa Fake Device implementation
 */

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "dev/isa_fake.hh"
#include "mem/packet.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

IsaFake::IsaFake(Params *p)
    : BasicPioDevice(p)
{
    pioSize = p->pio_size;
}

Tick
IsaFake::read(Packet &pkt)
{
    assert(pkt.result == Unknown);
    assert(pkt.addr >= pioAddr && pkt.addr < pioAddr + pioSize);

    pkt.time = curTick + pioDelay;

    DPRINTF(Tsunami, "read  va=%#x size=%d\n", pkt.addr, pkt.size);

    uint8_t *data8;
    uint16_t *data16;
    uint32_t *data32;
    uint64_t *data64;

    switch (pkt.size) {
      case sizeof(uint64_t):
         if (!pkt.data) {
             data64 = new uint64_t;
             pkt.data = (uint8_t*)data64;
         } else {
             data64 = (uint64_t*)pkt.data;
         }
         *data64 = 0xFFFFFFFFFFFFFFFFULL;
         break;
      case sizeof(uint32_t):
         if (!pkt.data) {
             data32 = new uint32_t;
             pkt.data = (uint8_t*)data32;
         } else {
             data32 = (uint32_t*)pkt.data;
         }
         *data32 = 0xFFFFFFFF;
         break;
      case sizeof(uint16_t):
         if (!pkt.data) {
             data16 = new uint16_t;
             pkt.data = (uint8_t*)data16;
         } else {
             data16 = (uint16_t*)pkt.data;
         }
         *data16 = 0xFFFF;
         break;
      case sizeof(uint8_t):
         if (!pkt.data) {
             data8 = new uint8_t;
             pkt.data = data8;
         } else {
             data8 = (uint8_t*)pkt.data;
         }
         *data8 = 0xFF;
         break;
      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    pkt.result = Success;
    return pioDelay;
}

Tick
IsaFake::write(Packet &pkt)
{
    pkt.time = curTick + pioDelay;
    DPRINTF(Tsunami, "write - va=%#x size=%d \n", pkt.addr, pkt.size);
    pkt.result = Success;
    return pioDelay;
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(IsaFake)

    Param<Addr> pio_addr;
    Param<Tick> pio_latency;
    Param<Addr> pio_size;
    SimObjectParam<Platform *> platform;
    SimObjectParam<System *> system;

END_DECLARE_SIM_OBJECT_PARAMS(IsaFake)

BEGIN_INIT_SIM_OBJECT_PARAMS(IsaFake)

    INIT_PARAM(pio_addr, "Device Address"),
    INIT_PARAM(pio_latency, "Programmed IO latency"),
    INIT_PARAM(pio_size, "Size of address range"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM(system, "system object")

END_INIT_SIM_OBJECT_PARAMS(IsaFake)

CREATE_SIM_OBJECT(IsaFake)
{
    IsaFake::Params *p = new IsaFake::Params;
    p->name = getInstanceName();
    p->pio_addr = pio_addr;
    p->pio_delay = pio_latency;
    p->pio_size = pio_size;
    p->platform = platform;
    p->system = system;
    return new IsaFake(p);
}

REGISTER_SIM_OBJECT("IsaFake", IsaFake)
