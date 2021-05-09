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

#include "dev/isa_fake.hh"

#include "base/trace.hh"
#include "debug/IsaFake.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

IsaFake::IsaFake(const Params &p)
    : BasicPioDevice(p, p.ret_bad_addr ? 0 : p.pio_size)
{
    retData8 = p.ret_data8;
    retData16 = p.ret_data16;
    retData32 = p.ret_data32;
    retData64 = p.ret_data64;
}

Tick
IsaFake::read(PacketPtr pkt)
{
    pkt->makeAtomicResponse();

    if (params().warn_access != "")
        warn("Device %s accessed by read to address %#x size=%d\n",
                name(), pkt->getAddr(), pkt->getSize());
    if (params().ret_bad_addr) {
        DPRINTF(IsaFake, "read to bad address va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else {
        assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
        DPRINTF(IsaFake, "read  va=%#x size=%d\n",
                pkt->getAddr(), pkt->getSize());
        switch (pkt->getSize()) {
          case sizeof(uint64_t):
             pkt->setLE(retData64);
             break;
          case sizeof(uint32_t):
             pkt->setLE(retData32);
             break;
          case sizeof(uint16_t):
             pkt->setLE(retData16);
             break;
          case sizeof(uint8_t):
             pkt->setLE(retData8);
             break;
          default:
             if (params().fake_mem)
                 std::memset(pkt->getPtr<uint8_t>(), 0, pkt->getSize());
             else
                 panic("invalid access size! Device being accessed by cache?\n");
        }
    }
    return pioDelay;
}

Tick
IsaFake::write(PacketPtr pkt)
{
    pkt->makeAtomicResponse();
    if (params().warn_access != "") {
        uint64_t data;
        switch (pkt->getSize()) {
          case sizeof(uint64_t):
            data = pkt->getLE<uint64_t>();
            break;
          case sizeof(uint32_t):
            data = pkt->getLE<uint32_t>();
            break;
          case sizeof(uint16_t):
            data = pkt->getLE<uint16_t>();
            break;
          case sizeof(uint8_t):
            data = pkt->getLE<uint8_t>();
            break;
          default:
            panic("invalid access size: %u\n", pkt->getSize());
        }
        warn("Device %s accessed by write to address %#x size=%d data=%#x\n",
                name(), pkt->getAddr(), pkt->getSize(), data);
    }
    if (params().ret_bad_addr) {
        DPRINTF(IsaFake, "write to bad address va=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());
        pkt->setBadAddress();
    } else {
        DPRINTF(IsaFake, "write - va=%#x size=%d \n",
                pkt->getAddr(), pkt->getSize());

        if (params().update_data) {
            switch (pkt->getSize()) {
              case sizeof(uint64_t):
                retData64 = pkt->getLE<uint64_t>();
                break;
              case sizeof(uint32_t):
                retData32 = pkt->getLE<uint32_t>();
                break;
              case sizeof(uint16_t):
                retData16 = pkt->getLE<uint16_t>();
                break;
              case sizeof(uint8_t):
                retData8 = pkt->getLE<uint8_t>();
                break;
              default:
                panic("invalid access size!\n");
            }
        }
    }
    return pioDelay;
}

} // namespace gem5
