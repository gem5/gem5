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

    retData8 = params()->retData8;
    retData16 = params()->retData16;
    retData32 = params()->retData32;
    retData64 = params()->retData64;
}

Tick
IsaFake::read(PacketPtr pkt)
{

    if (params()->warnAccess != "")
        warn("Device %s accessed by read to address %#x size=%d\n",
                name(), pkt->getAddr(), pkt->getSize());
    if (params()->retBadAddr) {
        DPRINTF(Tsunami, "read to bad address va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else {
        assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
        DPRINTF(Tsunami, "read  va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        switch (pkt->getSize()) {
          case sizeof(uint64_t):
             pkt->set(retData64);
             break;
          case sizeof(uint32_t):
             pkt->set(retData32);
             break;
          case sizeof(uint16_t):
             pkt->set(retData16);
             break;
          case sizeof(uint8_t):
             pkt->set(retData8);
             break;
          default:
            panic("invalid access size!\n");
        }
        pkt->makeAtomicResponse();
    }
    return pioDelay;
}

Tick
IsaFake::write(PacketPtr pkt)
{
    if (params()->warnAccess != "") {
        uint64_t data;
        switch (pkt->getSize()) {
          case sizeof(uint64_t):
            data = pkt->get<uint64_t>();
            break;
          case sizeof(uint32_t):
            data = pkt->get<uint32_t>();
            break;
          case sizeof(uint16_t):
            data = pkt->get<uint16_t>();
            break;
          case sizeof(uint8_t):
            data = pkt->get<uint8_t>();
            break;
          default:
            panic("invalid access size!\n");
        }
        warn("Device %s accessed by write to address %#x size=%d data=%#x\n",
                name(), pkt->getAddr(), pkt->getSize(), data);
    }
    if (params()->retBadAddr) {
        DPRINTF(Tsunami, "write to bad address va=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else {
        DPRINTF(Tsunami, "write - va=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());

        if (params()->updateData) {
            switch (pkt->getSize()) {
              case sizeof(uint64_t):
                retData64 = pkt->get<uint64_t>();
                break;
              case sizeof(uint32_t):
                retData32 = pkt->get<uint32_t>();
                break;
              case sizeof(uint16_t):
                retData16 = pkt->get<uint16_t>();
                break;
              case sizeof(uint8_t):
                retData8 = pkt->get<uint8_t>();
                break;
              default:
                panic("invalid access size!\n");
            }
        }
        pkt->makeAtomicResponse();
    }
    return pioDelay;
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(IsaFake)

    Param<Addr> pio_addr;
    Param<Tick> pio_latency;
    Param<Addr> pio_size;
    Param<bool> ret_bad_addr;
    Param<bool> update_data;
    Param<std::string> warn_access;
    Param<uint8_t> ret_data8;
    Param<uint16_t> ret_data16;
    Param<uint32_t> ret_data32;
    Param<uint64_t> ret_data64;
    SimObjectParam<Platform *> platform;
    SimObjectParam<System *> system;

END_DECLARE_SIM_OBJECT_PARAMS(IsaFake)

BEGIN_INIT_SIM_OBJECT_PARAMS(IsaFake)

    INIT_PARAM(pio_addr, "Device Address"),
    INIT_PARAM(pio_latency, "Programmed IO latency"),
    INIT_PARAM(pio_size, "Size of address range"),
    INIT_PARAM(ret_bad_addr, "Return pkt status BadAddr"),
    INIT_PARAM(update_data, "Update returned data"),
    INIT_PARAM(warn_access, "Warn if this device is touched"),
    INIT_PARAM(ret_data8, "Data to return if not bad addr"),
    INIT_PARAM(ret_data16, "Data to return if not bad addr"),
    INIT_PARAM(ret_data32, "Data to return if not bad addr"),
    INIT_PARAM(ret_data64, "Data to return if not bad addr"),
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
    p->updateData = update_data;
    p->warnAccess = warn_access;
    p->retData8= ret_data8;
    p->retData16 = ret_data16;
    p->retData32 = ret_data32;
    p->retData64 = ret_data64;
    p->platform = platform;
    p->system = system;
    return new IsaFake(p);
}

REGISTER_SIM_OBJECT("IsaFake", IsaFake)
