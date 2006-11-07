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
 * Authors: Ali Saidi
 */

/** @file
 * Isa Fake Device implementation
 */

#include "base/trace.hh"
#include "dev/isa_fake.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

IsaFake::IsaFake(Params *p)
    : BasicPioDevice(p)
{
    if (!params()->retBadAddr)
        pioSize = p->pio_size;

    memset(&retData, p->retData, sizeof(retData));
}

void
IsaFake::init()
{
    // Only init this device if it's connected to anything.
    if (pioPort)
        PioDevice::init();
}


Tick
IsaFake::read(PacketPtr pkt)
{
    assert(pkt->result == Packet::Unknown);

    if (params()->retBadAddr) {
        DPRINTF(Tsunami, "read to bad address va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        pkt->result = Packet::BadAddress;
    } else {
        assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
        DPRINTF(Tsunami, "read  va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        switch (pkt->getSize()) {
          case sizeof(uint64_t):
             pkt->set(retData);
             break;
          case sizeof(uint32_t):
             pkt->set((uint32_t)retData);
             break;
          case sizeof(uint16_t):
             pkt->set((uint16_t)retData);
             break;
          case sizeof(uint8_t):
             pkt->set((uint8_t)retData);
             break;
          default:
            panic("invalid access size!\n");
        }
        pkt->result = Packet::Success;
    }
    return pioDelay;
}

Tick
IsaFake::write(PacketPtr pkt)
{
    if (params()->retBadAddr) {
        DPRINTF(Tsunami, "write to bad address va=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());
        pkt->result = Packet::BadAddress;
    } else {
        DPRINTF(Tsunami, "write - va=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());
        pkt->result = Packet::Success;
    }
    return pioDelay;
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(IsaFake)

    Param<Addr> pio_addr;
    Param<Tick> pio_latency;
    Param<Addr> pio_size;
    Param<bool> ret_bad_addr;
    Param<uint8_t> ret_data;
    SimObjectParam<Platform *> platform;
    SimObjectParam<System *> system;

END_DECLARE_SIM_OBJECT_PARAMS(IsaFake)

BEGIN_INIT_SIM_OBJECT_PARAMS(IsaFake)

    INIT_PARAM(pio_addr, "Device Address"),
    INIT_PARAM(pio_latency, "Programmed IO latency"),
    INIT_PARAM(pio_size, "Size of address range"),
    INIT_PARAM(ret_bad_addr, "Return pkt status BadAddr"),
    INIT_PARAM(ret_data, "Data to return if not bad addr"),
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
    p->retBadAddr = ret_bad_addr;
    p->retData = ret_data;
    p->platform = platform;
    p->system = system;
    return new IsaFake(p);
}

REGISTER_SIM_OBJECT("IsaFake", IsaFake)
