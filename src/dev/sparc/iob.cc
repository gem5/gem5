/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * This device implemetns the niagara I/O bridge chip. It manages incomming
 * interrupts and posts them to the CPU when needed. It holds mask registers and
 * various status registers for CPUs to check what interrupts are pending as
 * well as facilities to send IPIs to other cpus.
 */

#include "dev/sparc/iob.hh"

#include <cstring>

#include "arch/sparc/faults.hh"
#include "arch/sparc/interrupts.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Iob.hh"
#include "dev/platform.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"
#include "sim/faults.hh"
#include "sim/system.hh"

namespace gem5
{

Iob::Iob(const Params &p) : PioDevice(p)
{
    iobManAddr = 0x9800000000ULL;
    iobManSize = 0x0100000000ULL;
    iobJBusAddr = 0x9F00000000ULL;
    iobJBusSize = 0x0100000000ULL;
    assert(params().system->threads.size() <= MaxNiagaraProcs);

    pioDelay = p.pio_latency;

    for (int x = 0; x < NumDeviceIds; ++x) {
        intMan[x].cpu = 0;
        intMan[x].vector = 0;
        intCtl[x].mask = true;
        intCtl[x].pend = false;
    }

}

Tick
Iob::read(PacketPtr pkt)
{

    if (pkt->getAddr() >= iobManAddr && pkt->getAddr() < iobManAddr + iobManSize)
        readIob(pkt);
    else if (pkt->getAddr() >= iobJBusAddr && pkt->getAddr() < iobJBusAddr+iobJBusSize)
        readJBus(pkt);
    else
        panic("Invalid address reached Iob\n");

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Iob::readIob(PacketPtr pkt)
{
        Addr accessAddr = pkt->getAddr() - iobManAddr;

        assert(IntManAddr == 0);
        if (accessAddr < IntManAddr + IntManSize) {
            int index = (accessAddr - IntManAddr) >> 3;
            uint64_t data = intMan[index].cpu << 8 | intMan[index].vector << 0;
            pkt->setBE(data);
            return;
        }

        if (accessAddr >= IntCtlAddr && accessAddr < IntCtlAddr + IntCtlSize) {
            int index = (accessAddr - IntCtlAddr) >> 3;
            uint64_t data = (intCtl[index].mask  ? (1 << 2) : 0) |
                (intCtl[index].pend  ? (1 << 0) : 0);
            pkt->setBE(data);
            return;
        }

        if (accessAddr == JIntVecAddr) {
            pkt->setBE(jIntVec);
            return;
        }

        panic("Read to unknown IOB offset 0x%x\n", accessAddr);
}

void
Iob::readJBus(PacketPtr pkt)
{
        Addr accessAddr = pkt->getAddr() - iobJBusAddr;
        ContextID cpuid = pkt->req->contextId();
        int index;
        uint64_t data;




        if (accessAddr >= JIntData0Addr && accessAddr < JIntData1Addr) {
            index = (accessAddr - JIntData0Addr) >> 3;
            pkt->setBE(jBusData0[index]);
            return;
        }

        if (accessAddr >= JIntData1Addr && accessAddr < JIntDataA0Addr) {
            index = (accessAddr - JIntData1Addr) >> 3;
            pkt->setBE(jBusData1[index]);
            return;
        }

        if (accessAddr == JIntDataA0Addr) {
            pkt->setBE(jBusData0[cpuid]);
            return;
        }

        if (accessAddr == JIntDataA1Addr) {
            pkt->setBE(jBusData1[cpuid]);
            return;
        }

        if (accessAddr >= JIntBusyAddr && accessAddr < JIntBusyAddr + JIntBusySize) {
            index = (accessAddr - JIntBusyAddr) >> 3;
            data = jIntBusy[index].busy ? 1 << 5 : 0 |
                   jIntBusy[index].source;
            pkt->setBE(data);
            return;
        }
        if (accessAddr == JIntABusyAddr) {
            data = jIntBusy[cpuid].busy ? 1 << 5 : 0 |
                   jIntBusy[cpuid].source;
            pkt->setBE(data);
            return;
        };

        panic("Read to unknown JBus offset 0x%x\n", accessAddr);
}

Tick
Iob::write(PacketPtr pkt)
{
    if (pkt->getAddr() >= iobManAddr && pkt->getAddr() < iobManAddr + iobManSize)
        writeIob(pkt);
    else if (pkt->getAddr() >= iobJBusAddr && pkt->getAddr() < iobJBusAddr+iobJBusSize)
        writeJBus(pkt);
    else
        panic("Invalid address reached Iob\n");


    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Iob::writeIob(PacketPtr pkt)
{
        Addr accessAddr = pkt->getAddr() - iobManAddr;
        int index;
        uint64_t data;

        assert(IntManAddr == 0);
        if (accessAddr < IntManAddr + IntManSize) {
            index = (accessAddr - IntManAddr) >> 3;
            data = pkt->getBE<uint64_t>();
            intMan[index].cpu = bits(data,12,8);
            intMan[index].vector = bits(data,5,0);
            DPRINTF(Iob, "Wrote IntMan %d cpu %d, vec %d\n", index,
                    intMan[index].cpu, intMan[index].vector);
            return;
        }

        if (accessAddr >= IntCtlAddr && accessAddr < IntCtlAddr + IntCtlSize) {
            index = (accessAddr - IntCtlAddr) >> 3;
            data = pkt->getBE<uint64_t>();
            intCtl[index].mask = bits(data,2,2);
            if (bits(data,1,1))
                intCtl[index].pend = false;
            DPRINTF(Iob, "Wrote IntCtl %d pend %d cleared %d\n", index,
                    intCtl[index].pend, bits(data,2,2));
            return;
        }

        if (accessAddr == JIntVecAddr) {
            jIntVec = bits(pkt->getBE<uint64_t>(), 5,0);
            DPRINTF(Iob, "Wrote jIntVec %d\n", jIntVec);
            return;
        }

        if (accessAddr >= IntVecDisAddr && accessAddr < IntVecDisAddr + IntVecDisSize) {
            Type type;
            int cpu_id;
            int vector;
            index = (accessAddr - IntManAddr) >> 3;
            data = pkt->getBE<uint64_t>();
            type = (Type)bits(data,17,16);
            cpu_id = bits(data, 12,8);
            vector = bits(data,5,0);
            generateIpi(type,cpu_id, vector);
            return;
        }

        panic("Write to unknown IOB offset 0x%x\n", accessAddr);
}

void
Iob::writeJBus(PacketPtr pkt)
{
        Addr accessAddr = pkt->getAddr() - iobJBusAddr;
        ContextID cpuid = pkt->req->contextId();
        int index;
        uint64_t data;

        if (accessAddr >= JIntBusyAddr && accessAddr < JIntBusyAddr + JIntBusySize) {
            index = (accessAddr - JIntBusyAddr) >> 3;
            data = pkt->getBE<uint64_t>();
            jIntBusy[index].busy = bits(data,5,5);
            DPRINTF(Iob, "Wrote jIntBusy index %d busy: %d\n", index,
                    jIntBusy[index].busy);
            return;
        }
        if (accessAddr == JIntABusyAddr) {
            data = pkt->getBE<uint64_t>();
            jIntBusy[cpuid].busy = bits(data,5,5);
            DPRINTF(Iob, "Wrote jIntBusy index %d busy: %d\n", cpuid,
                    jIntBusy[cpuid].busy);
            return;
        };

        panic("Write to unknown JBus offset 0x%x\n", accessAddr);
}

void
Iob::receiveDeviceInterrupt(DeviceId devid)
{
    assert(devid < NumDeviceIds);
    if (intCtl[devid].mask)
        return;
    intCtl[devid].mask = true;
    intCtl[devid].pend = true;
    DPRINTF(Iob, "Receiving Device interrupt: %d for cpu %d vec %d\n",
            devid, intMan[devid].cpu, intMan[devid].vector);
    auto tc = sys->threads[intMan[devid].cpu];
    tc->getCpuPtr()->postInterrupt(tc->threadId(), SparcISA::IT_INT_VEC,
            intMan[devid].vector);
}


void
Iob::generateIpi(Type type, int cpu_id, int vector)
{
    SparcISA::SparcFault<SparcISA::PowerOnReset> *por =
        new SparcISA::PowerOnReset();
    if (cpu_id >= sys->threads.size())
        return;

    auto tc = sys->threads[cpu_id];
    switch (type) {
      case 0: // interrupt
        DPRINTF(Iob,
                "Generating interrupt because of I/O write to cpu: "
                "%d vec %d\n",
                cpu_id, vector);
        tc->getCpuPtr()->postInterrupt(
                tc->threadId(), SparcISA::IT_INT_VEC, vector);
        break;
      case 1: // reset
        warn("Sending reset to CPU: %d\n", cpu_id);
        if (vector != por->trapType())
            panic("Don't know how to set non-POR reset to cpu\n");
        por->invoke(tc);
        tc->activate();
        break;
      case 2: // idle -- this means stop executing and don't wake on interrupts
        DPRINTF(Iob, "Idling CPU because of I/O write cpu: %d\n", cpu_id);
        tc->halt();
        break;
      case 3: // resume
        DPRINTF(Iob, "Resuming CPU because of I/O write cpu: %d\n", cpu_id);
        tc->activate();
        break;
      default:
        panic("Invalid type to generate ipi\n");
    }
}

bool
Iob::receiveJBusInterrupt(int cpu_id, int source, uint64_t d0, uint64_t d1)
{
    // If we are already dealing with an interrupt for that cpu we can't deal
    // with another one right now... come back later
    if (jIntBusy[cpu_id].busy)
        return false;

    DPRINTF(Iob, "Receiving jBus interrupt: %d for cpu %d vec %d\n",
            source, cpu_id, jIntVec);

    jIntBusy[cpu_id].busy = true;
    jIntBusy[cpu_id].source = source;
    jBusData0[cpu_id] = d0;
    jBusData1[cpu_id] = d1;

    auto tc = sys->threads[cpu_id];
    tc->getCpuPtr()->postInterrupt(
            tc->threadId(), SparcISA::IT_INT_VEC, jIntVec);
    return true;
}

AddrRangeList
Iob::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(iobManAddr, iobManSize));
    ranges.push_back(RangeSize(iobJBusAddr, iobJBusSize));
    return ranges;
}


void
Iob::serialize(CheckpointOut &cp) const
{

    SERIALIZE_SCALAR(jIntVec);
    SERIALIZE_ARRAY(jBusData0, MaxNiagaraProcs);
    SERIALIZE_ARRAY(jBusData1, MaxNiagaraProcs);
    for (int x = 0; x < NumDeviceIds; x++) {
        ScopedCheckpointSection sec(cp, csprintf("Int%d", x));
        paramOut(cp, "cpu", intMan[x].cpu);
        paramOut(cp, "vector", intMan[x].vector);
        paramOut(cp, "mask", intCtl[x].mask);
        paramOut(cp, "pend", intCtl[x].pend);
    };
    for (int x = 0; x < MaxNiagaraProcs; x++) {
        ScopedCheckpointSection sec(cp, csprintf("jIntBusy%d", x));
        paramOut(cp, "busy", jIntBusy[x].busy);
        paramOut(cp, "source", jIntBusy[x].source);
    };
}

void
Iob::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(jIntVec);
    UNSERIALIZE_ARRAY(jBusData0, MaxNiagaraProcs);
    UNSERIALIZE_ARRAY(jBusData1, MaxNiagaraProcs);
    for (int x = 0; x < NumDeviceIds; x++) {
        ScopedCheckpointSection sec(cp, csprintf("Int%d", x));
        paramIn(cp, "cpu", intMan[x].cpu);
        paramIn(cp, "vector", intMan[x].vector);
        paramIn(cp, "mask", intCtl[x].mask);
        paramIn(cp, "pend", intCtl[x].pend);
    };
    for (int x = 0; x < MaxNiagaraProcs; x++) {
        ScopedCheckpointSection sec(cp, csprintf("jIntBusy%d", x));
        paramIn(cp, "busy", jIntBusy[x].busy);
        paramIn(cp, "source", jIntBusy[x].source);
    };
}

} // namespace gem5
