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

#include "dev/lupio/lupio_pic.hh"

#include "cpu/base.hh"
#include "debug/LupioPIC.hh"
#include "mem/packet_access.hh"
#include "params/LupioPIC.hh"
#include "sim/system.hh"

namespace gem5
{

LupioPIC::LupioPIC(const Params &params) :
    BasicPioDevice(params, params.pio_size),
    system(params.system),
    nSrc(params.n_src),
    nThread(params.num_threads),
    intType(params.int_type),
    mask{0},
    enable{0}
{
    // CPU0 receives all IRQ sources by default
    enable[0] = 0xFFFFFFFF;
    DPRINTF(LupioPIC, "LupioPIC initalized\n");
}

void
LupioPIC::lupioPicUpdateIRQ()
{
    for (int cpu = 0; cpu < nThread; cpu++) {
        auto tc = system->threads[cpu];

        if (enable[cpu] & mask[cpu] & pending) {
            tc->getCpuPtr()->postInterrupt(tc->threadId(), intType, 0);
        } else {
            tc->getCpuPtr()->clearInterrupt(tc->threadId(), intType, 0);
        }
    }
}

void
LupioPIC::post(int src_id)
{
    gem5_assert(src_id < nSrc && src_id >= 0);

    uint32_t irq_mask = 1UL << src_id;
    pending |= irq_mask;
    lupioPicUpdateIRQ();
}

void
LupioPIC::clear(int src_id)
{
    gem5_assert(src_id < nSrc && src_id >= 0);

    uint32_t irq_mask = 1UL << src_id;
    pending &= ~irq_mask;
    lupioPicUpdateIRQ();
}

uint64_t
LupioPIC::lupioPicRead(uint8_t addr)
{
    uint32_t r = 0;

    int cpu = addr / LUPIO_PIC_MAX;
    int reg = addr % LUPIO_PIC_MAX;

    switch (reg) {
        case LUPIO_PIC_PRIO:
            // Value will be 32 if there is no unmasked pending IRQ
            r = ctz32(pending & mask[cpu] & enable[cpu]);
            DPRINTF(LupioPIC, "Read PIC_PRIO: %d\n", r);
            break;
        case LUPIO_PIC_MASK:
            r = mask[cpu];
            DPRINTF(LupioPIC, "Read PIC_MASK: %d\n", r);
            break;
        case LUPIO_PIC_PEND:
            r = (enable[cpu] & pending);
            DPRINTF(LupioPIC, "Read PIC_PEND: %d\n", r);
            break;
         case LUPIO_PIC_ENAB:
            r = enable[cpu];
            break;

        default:
            panic("Unexpected read to the LupioPIC device at address %#llx!",
                    addr);
            break;
    }
    return r;
}

void
LupioPIC::lupioPicWrite(uint8_t addr, uint64_t val64)
{
    uint32_t val = val64;

    int cpu = addr / LUPIO_PIC_MAX;
    int reg = addr % LUPIO_PIC_MAX;

    switch (reg) {
        case LUPIO_PIC_MASK:
            mask[cpu] = val;
            DPRINTF(LupioPIC, "Write PIC_MASK: %d\n", mask[cpu]);
            lupioPicUpdateIRQ();
            break;
        case LUPIO_PIC_ENAB:
            enable[cpu] = val;
            DPRINTF(LupioPIC, "Write PIC_ENAB: %d\n", enable[cpu]);
            lupioPicUpdateIRQ();
            break;

        default:
            panic("Unexpected write to the LupioPIC device at address %#llx!",
                    addr);
            break;
    }
}

Tick
LupioPIC::read(PacketPtr pkt)
{
    Addr pic_addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioPIC,
        "Read request - addr: %#x, size: %#x\n", pic_addr, pkt->getSize());

    uint64_t read_val = lupioPicRead(pic_addr);
    DPRINTF(LupioPIC, "Packet Read: %#x\n", read_val);
    pkt->setUintX(read_val, byteOrder);
    pkt->makeResponse();

    return pioDelay;
}

Tick
LupioPIC::write(PacketPtr pkt)
{
    Addr pic_addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioPIC, "Write register %#x value %#x\n", pic_addr,
            pkt->getUintX(byteOrder));

    lupioPicWrite(pic_addr, pkt->getUintX(byteOrder));
    DPRINTF(LupioPIC, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));

    pkt->makeResponse();

    return pioDelay;
}
} // namespace gem5
