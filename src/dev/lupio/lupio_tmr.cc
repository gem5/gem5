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

#include "dev/lupio/lupio_tmr.hh"

#include "cpu/base.hh"
#include "debug/LupioTMR.hh"
#include "mem/packet_access.hh"
#include "params/LupioTMR.hh"

// Specific fields for CTRL
#define LUPIO_TMR_IRQE 0x1
#define LUPIO_TMR_PRDC 0x2

// Specific fields for STAT
#define LUPIO_TMR_EXPD 0x1

namespace gem5
{

LupioTMR::LupioTMR(const Params &params)
    : BasicPioDevice(params, params.pio_size),
      system(params.system),
      nThread(params.num_threads),
      intType(params.int_type)
{
    timers.resize(nThread);

    for (int cpu = 0; cpu < nThread; cpu++) {
        timers[cpu].tmrEvent = new EventFunctionWrapper(
            [=] { lupioTMRCallback(cpu); }, name() + "done");
    }

    DPRINTF(LupioTMR, "LupioTMR initalized\n");
}

LupioTMR::~LupioTMR()
{
    for (int cpu = 0; cpu < nThread; cpu++) {
        delete timers[cpu].tmrEvent;
    }
}

void
LupioTMR::updateIRQ(int level, int cpu)
{
    auto tc = system->threads[cpu];
    // post an interrupt
    if (level) {
        tc->getCpuPtr()->postInterrupt(tc->threadId(), intType, 0);
    } else {
        // clear the interrupt
        tc->getCpuPtr()->clearInterrupt(tc->threadId(), intType, 0);
    }
}

uint64_t
LupioTMR::lupioTMRCurrentTime()
{
    return curTick() / sim_clock::as_int::ns;
}

void
LupioTMR::lupioTMRSet(int cpu)
{
    // Start the timer
    timers[cpu].startTime = curTick();

    // Schedule the timer to fire at the number of ticks stored
    // in the reload register from the current tick
    if (!timers[cpu].tmrEvent->scheduled()) {
        // Convert the reload value to ticks from nanoseconds
        schedule(*(timers[cpu].tmrEvent),
                 (timers[cpu].reload * sim_clock::as_int::ns) + curTick());
    }
}

void
LupioTMR::lupioTMRCallback(int cpu)
{
    // Signal expiration
    timers[cpu].expired = true;
    if (timers[cpu].ie) {
        updateIRQ(1, cpu);
    }

    // If periodic timer, reload
    if (timers[cpu].pd && timers[cpu].reload) {
        lupioTMRSet(cpu);
    }
}

uint64_t
LupioTMR::lupioTMRRead(uint8_t addr, int size)
{
    uint32_t r = 0;

    size_t cpu = addr / LUPIO_TMR_MAX;
    size_t reg = addr % LUPIO_TMR_MAX;

    switch (reg) {
    case LUPIO_TMR_TIME:
        r = lupioTMRCurrentTime();
        DPRINTF(LupioTMR, "Read LUPIO_TMR_TME: %d\n", r);
        break;
    case LUPIO_TMR_LOAD:
        r = timers[cpu].reload;
        DPRINTF(LupioTMR, "Read LUPIO_TMR_LOAD: %d\n", r);
        break;
    case LUPIO_TMR_STAT:
        if (timers[cpu].expired) {
            r |= LUPIO_TMR_EXPD;
        }

        // Acknowledge expiration
        timers[cpu].expired = false;
        DPRINTF(LupioTMR, "Read LUPIO_TMR_STAT: %d\n", r);
        updateIRQ(0, cpu);
        break;

    default:
        panic("Unexpected read to the LupioTMR device at address %#llx!",
              addr);
        break;
    }
    return r;
}

void
LupioTMR::lupioTMRWrite(uint8_t addr, uint64_t val64, int size)
{
    uint32_t val = val64;

    size_t cpu = addr / LUPIO_TMR_MAX;
    size_t reg = addr % LUPIO_TMR_MAX;

    switch (reg) {
    case LUPIO_TMR_LOAD:
        timers[cpu].reload = val;
        DPRINTF(LupioTMR, "Write LUPIO_TMR_LOAD: %d\n", timers[cpu].reload);
        break;

    case LUPIO_TMR_CTRL:
        timers[cpu].ie = val & LUPIO_TMR_IRQE;
        timers[cpu].pd = val & LUPIO_TMR_PRDC;
        DPRINTF(LupioTMR, "Write LUPIO_TMR_CTRL\n");

        // Stop current timer if any
        if (curTick() < timers[cpu].startTime +
                            (timers[cpu].reload * sim_clock::as_int::ns) &&
            (timers[cpu].tmrEvent)->scheduled()) {
            deschedule(*(timers[cpu].tmrEvent));
        }

        // If reload isn't 0, start a new one
        if (timers[cpu].reload) {
            lupioTMRSet(cpu);
        }
        break;

    default:
        panic("Unexpected write to the LupioTMR device at address %#llx!",
              addr);
        break;
    }
}

Tick
LupioTMR::read(PacketPtr pkt)
{
    Addr tmr_addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioTMR, "Read request - addr: %#x, size: %#x\n", tmr_addr,
            pkt->getSize());

    uint64_t read_val = lupioTMRRead(tmr_addr, pkt->getSize());
    DPRINTF(LupioTMR, "Packet Read: %#x\n", read_val);
    pkt->setUintX(read_val, byteOrder);
    pkt->makeResponse();

    return pioDelay;
}

Tick
LupioTMR::write(PacketPtr pkt)
{
    Addr tmr_addr = pkt->getAddr() - pioAddr;

    DPRINTF(LupioTMR, "Write register %#x value %#x\n", tmr_addr,
            pkt->getUintX(byteOrder));

    lupioTMRWrite(tmr_addr, pkt->getUintX(byteOrder), pkt->getSize());
    DPRINTF(LupioTMR, "Packet Write Value: %d\n", pkt->getUintX(byteOrder));

    pkt->makeResponse();

    return pioDelay;
}
} // namespace gem5
