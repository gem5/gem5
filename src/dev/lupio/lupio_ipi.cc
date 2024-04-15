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

LupioIPI::LupioIPI(const Params &params)
    : BasicPioDevice(params, params.pio_size),
      system(params.system),
      intType(params.int_type),
      nThread(params.num_threads)
{
    mask.resize(nThread, 0);
    pending.resize(nThread, 0);

    DPRINTF(LupioIPI, "LupioIPI initalized--number of CPUs: %d\n", nThread);
}

void
LupioIPI::lupioIPIUpdateIRQ()
{
    for (int cpu = 0; cpu < nThread; cpu++) {
        auto tc = system->threads[cpu];

        if (mask[cpu] & pending[cpu]) {
            tc->getCpuPtr()->postInterrupt(tc->threadId(), intType, 0);
        } else {
            tc->getCpuPtr()->clearInterrupt(tc->threadId(), intType, 0);
        }
    }
}

uint64_t
LupioIPI::lupioIPIRead(uint8_t addr, int size)
{
    uint32_t r = 0;

    int cpu = addr / LUPIO_IPI_MAX;
    int reg = addr % LUPIO_IPI_MAX;

    switch (reg) {
    case LUPIO_IPI_MASK:
        r = mask[cpu];
        DPRINTF(LupioIPI, "Read IPI_MASK[%d]: %#x\n", cpu, r);
        break;
    case LUPIO_IPI_PEND:
        r = pending[cpu];
        DPRINTF(LupioIPI, "Read IPI_PEND[%d]: %#x\n", cpu, r);
        break;

    default:
        panic("Unexpected read to LupioIPI device at address %#llx!", addr);
        break;
    }

    return r;
}

void
LupioIPI::lupioIPIWrite(uint8_t addr, uint64_t val64, int size)
{
    uint32_t val = val64;

    int cpu = addr / LUPIO_IPI_MAX;
    int reg = addr % LUPIO_IPI_MAX;

    switch (reg) {
    case LUPIO_IPI_MASK:
        mask[cpu] = val;
        DPRINTF(LupioIPI, "Write IPI_MASK[%d]: %#x\n", cpu, mask[cpu]);
        lupioIPIUpdateIRQ();
        break;
    case LUPIO_IPI_PEND:
        pending[cpu] = val;
        DPRINTF(LupioIPI, "Write IPI_PEND[%d]: %#x\n", cpu, pending[cpu]);
        lupioIPIUpdateIRQ();
        break;

    default:
        panic("Unexpected write to LupioIPI device at address %#llx!", addr);
        break;
    }
}

Tick
LupioIPI::read(PacketPtr pkt)
{
    Addr ipi_addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioIPI, "Read request - addr: %#x, size: %#x\n", ipi_addr,
            pkt->getSize());

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
