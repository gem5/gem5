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

#include "dev/lupio/lupio_ipi.hh"

#include "cpu/base.hh"
#include "debug/LupioIPI.hh"
#include "mem/packet_access.hh"
#include "params/LupioIPI.hh"

namespace gem5
{

LupioIPI::LupioIPI(const Params &params) :
    BasicPioDevice(params, params.pio_size),
    system(params.system),
    intType(params.int_type),
    nThread(params.num_threads)
{
    word.resize(nThread, 0);

    DPRINTF(LupioIPI, "LupioIPI initalized--number of CPUs: %d\n", nThread);
}

uint64_t
LupioIPI::lupioIPIRead(uint8_t addr, int size)
{
    int cpu = addr >> 2;
    uint32_t r = 0;
    // Reading automatically lowers corresponding IRQ
    r = word[cpu];

    auto tc = system->threads[cpu];
    tc->getCpuPtr()->clearInterrupt(tc->threadId(), intType, 0);
    // Also reset value after reading
    word[cpu] = 0;

    return r;
}

void
LupioIPI::lupioIPIWrite(uint8_t addr, uint64_t val64, int size)
{
    int cpu = addr >> 2;;
    uint32_t val = val64;

    word[cpu] = val;

    // Raise IRQ
    auto tc = system->threads[cpu];

    tc->getCpuPtr()->postInterrupt(tc->threadId(), intType, 0);
}

Tick
LupioIPI::read(PacketPtr pkt)
{
    Addr ipi_addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioIPI,
        "Read request - addr: %#x, size: %#x\n", ipi_addr, pkt->getSize());

    uint64_t read_val = lupioIPIRead(ipi_addr, pkt->getSize());
    DPRINTF(LupioIPI, "Packet Read: %#x\n", read_val);
    pkt->setUintX(read_val, byteOrder);
    pkt->makeResponse();

    return pioDelay;
}

Tick
LupioIPI::write(PacketPtr pkt)
{
    Addr ipi_addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioIPI, "Write register %#x value %#x\n", ipi_addr,
            pkt->getUintX(byteOrder));

    lupioIPIWrite(ipi_addr, pkt->getUintX(byteOrder), pkt->getSize());
    DPRINTF(LupioIPI, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));

    pkt->makeResponse();

    return pioDelay;
}
} // namespace gem5

