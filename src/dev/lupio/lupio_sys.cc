/*
 * Copyright (c) 2021 The Regents of the University of California
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

#include "dev/lupio/lupio_sys.hh"

#include "debug/LupioSYS.hh"
#include "mem/packet_access.hh"
#include "params/LupioSYS.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

LupioSYS::LupioSYS(const Params &params)
    : BasicPioDevice(params, params.pio_size)
{
    DPRINTF(LupioSYS, "LupioSYS initalized\n");
}

uint8_t
LupioSYS::lupioSYSRead(uint8_t addr)
{
    return 0;
}

void
LupioSYS::lupioSYSWrite(uint8_t addr, uint64_t val64)
{
    switch (addr >> 2) {
    case LUPIO_SYS_HALT:
        DPRINTF(LupioSYS, "Trying to halt\n");
        exitSimLoopNow("LUPIO_SYS_HALT called, exiting", val64, 0, false);
        break;
    case LUPIO_SYS_REBT:
        DPRINTF(LupioSYS, "Trying to reboot\n");
        exitSimLoopNow("LUPIO_SYS_REBT called, exiting", val64, 0, false);
        break;

    default:
        panic("Unexpected write to the LupioRTC device at address %d!", addr);
        break;
    }
}

Tick
LupioSYS::read(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioSYS, "Read request - addr: %#x, size: %#x\n", daddr,
            pkt->getSize());

    uint64_t sys_read = lupioSYSRead(daddr);
    DPRINTF(LupioSYS, "Packet Read: %#x\n", sys_read);
    pkt->setUintX(sys_read, byteOrder);
    pkt->makeResponse();

    return pioDelay;
}

Tick
LupioSYS::write(PacketPtr pkt)
{
    Addr daddr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioSYS, "Write register %#x value %#x\n", daddr,
            pkt->getUintX(byteOrder));

    lupioSYSWrite(daddr, pkt->getUintX(byteOrder));
    DPRINTF(LupioSYS, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));

    pkt->makeResponse();

    return pioDelay;
}
} // namespace gem5
