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
 *
 * Authors: Ali Saidi
 */

/** @file
 * This device implemetns the niagara I/O bridge chip. It manages incomming
 * interrupts and posts them to the CPU when needed. It holds mask registers and
 * various status registers for CPUs to check what interrupts are pending as
 * well as facilities to send IPIs to other cpus.
 */

#include <cstring>

#include "arch/sparc/isa_traits.hh"
#include "base/trace.hh"
#include "cpu/intr_control.hh"
#include "dev/sparc/iob.hh"
#include "dev/platform.hh"
#include "mem/port.hh"
#include "mem/packet_access.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

Iob::Iob(Params *p)
    : PioDevice(p), ic(p->platform->intrctrl)
{
    iobManAddr = ULL(0x9800000000);
    iobManSize = ULL(0x0100000000);
    iobJBusAddr = ULL(0x9F00000000);
    iobJBusSize = ULL(0x0100000000);
    assert (params()->system->threadContexts.size() <= MaxNiagaraProcs);
    // Get the interrupt controller from the platform
    ic = platform->intrctrl;

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
    assert(pkt->result == Packet::Unknown);

    if (pkt->getAddr() >= iobManAddr && pkt->getAddr() < iobManAddr + iobManSize)
        readIob(pkt);
    else if (pkt->getAddr() >= iobJBusAddr && pkt->getAddr() < iobJBusAddr+iobJBusSize)
        readJBus(pkt);
    else
        panic("Invalid address reached Iob\n");

    pkt->result = Packet::Success;
    return pioDelay;
}

void
Iob::readIob(PacketPtr pkt)
{
        Addr accessAddr = pkt->getAddr() - iobManAddr;
        int index;
        uint64_t data;

        if (accessAddr >= IntManAddr && accessAddr < IntManAddr + IntManSize) {
            index = (accessAddr - IntManAddr) >> 3;
            data = intMan[index].cpu << 8 | intMan[index].vector << 0;
            pkt->set(data);
            return;
        }

        if (accessAddr >= IntCtlAddr && accessAddr < IntCtlAddr + IntCtlSize) {
            index = (accessAddr - IntManAddr) >> 3;
            data = intCtl[index].mask  ? 1 << 2 : 0 |
                   intCtl[index].pend  ? 1 << 0 : 0;
            pkt->set(data);
            return;
        }

        if (accessAddr == JIntVecAddr) {
            pkt->set(jIntVec);
            return;
        }

        panic("Read to unknown IOB offset 0x%x\n", accessAddr);
}

void
Iob::readJBus(PacketPtr pkt)
{
        Addr accessAddr = pkt->getAddr() - iobJBusAddr;
        int cpuid = pkt->req->getCpuNum();
        int index;
        uint64_t data;




        if (accessAddr >= JIntData0Addr && accessAddr < JIntData1Addr) {
            index = (accessAddr - JIntData0Addr) >> 3;
            pkt->set(jBusData0[index]);
            return;
        }

        if (accessAddr >= JIntData1Addr && accessAddr < JIntDataA0Addr) {
            index = (accessAddr - JIntData1Addr) >> 3;
            pkt->set(jBusData1[index]);
            return;
        }

        if (accessAddr == JIntDataA0Addr) {
            pkt->set(jBusData0[cpuid]);
            return;
        }

        if (accessAddr == JIntDataA1Addr) {
            pkt->set(jBusData1[cpuid]);
            return;
        }

        if (accessAddr >= JIntBusyAddr && accessAddr < JIntBusyAddr + JIntBusySize) {
            index = (accessAddr - JIntBusyAddr) >> 3;
            data = jIntBusy[index].busy ? 1 << 5 : 0 |
                   jIntBusy[index].source;
            pkt->set(data);
            return;
        }
        if (accessAddr == JIntABusyAddr) {
            data = jIntBusy[cpuid].busy ? 1 << 5 : 0 |
                   jIntBusy[cpuid].source;
            pkt->set(data);
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


    pkt->result = Packet::Success;
    return pioDelay;
}

void
Iob::writeIob(PacketPtr pkt)
{
        Addr accessAddr = pkt->getAddr() - iobManAddr;
        int index;
        uint64_t data;

        if (accessAddr >= IntManAddr && accessAddr < IntManAddr + IntManSize) {
            index = (accessAddr - IntManAddr) >> 3;
            data = pkt->get<uint64_t>();
            intMan[index].cpu = bits(data,12,8);
            intMan[index].vector = bits(data,5,0);
            return;
        }

        if (accessAddr >= IntCtlAddr && accessAddr < IntCtlAddr + IntCtlSize) {
            index = (accessAddr - IntManAddr) >> 3;
            data = pkt->get<uint64_t>();
            intCtl[index].mask = bits(data,2,2);
            if (bits(data,1,1))
                intCtl[index].pend = false;
            return;
        }

        if (accessAddr == JIntVecAddr) {
            jIntVec = bits(pkt->get<uint64_t>(), 5,0);
            return;
        }

        if (accessAddr >= IntVecDisAddr && accessAddr < IntVecDisAddr + IntVecDisSize) {
            Type type;
            int cpu_id;
            int vector;
            index = (accessAddr - IntManAddr) >> 3;
            data = pkt->get<uint64_t>();
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
        int cpuid = pkt->req->getCpuNum();
        int index;
        uint64_t data;

        if (accessAddr >= JIntBusyAddr && accessAddr < JIntBusyAddr + JIntBusySize) {
            index = (accessAddr - JIntBusyAddr) >> 3;
            data = pkt->get<uint64_t>();
            jIntBusy[index].busy = bits(data,5,5);
            return;
        }
        if (accessAddr == JIntABusyAddr) {
            data = pkt->get<uint64_t>();
            jIntBusy[cpuid].busy = bits(data,5,5);
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
    ic->post(intMan[devid].cpu, SparcISA::IT_INT_VEC, intMan[devid].vector);
}


void
Iob::generateIpi(Type type, int cpu_id, int vector)
{
    // Only handle interrupts for the moment... Cpu Idle/reset/resume will be
    // later
    if (type != 0)
        return;

    assert(type == 0);
    ic->post(cpu_id, SparcISA::IT_INT_VEC, vector);
}

bool
Iob::receiveJBusInterrupt(int cpu_id, int source, uint64_t d0, uint64_t d1)
{
    // If we are already dealing with an interrupt for that cpu we can't deal
    // with another one right now... come back later
    if (jIntBusy[cpu_id].busy)
        return false;

    jIntBusy[cpu_id].busy = true;
    jIntBusy[cpu_id].source = source;
    jBusData0[cpu_id] = d0;
    jBusData1[cpu_id] = d1;

    ic->post(cpu_id, SparcISA::IT_INT_VEC, jIntVec);
    return true;
}

void
Iob::addressRanges(AddrRangeList &range_list)
{
    range_list.clear();
    range_list.push_back(RangeSize(iobManAddr, iobManSize));
    range_list.push_back(RangeSize(iobJBusAddr, iobJBusSize));
}


void
Iob::serialize(std::ostream &os)
{

    SERIALIZE_SCALAR(jIntVec);
    SERIALIZE_ARRAY(jBusData0, MaxNiagaraProcs);
    SERIALIZE_ARRAY(jBusData1, MaxNiagaraProcs);
    for (int x = 0; x < NumDeviceIds; x++) {
        nameOut(os, csprintf("%s.Int%d", name(), x));
        paramOut(os, "cpu", intMan[x].cpu);
        paramOut(os, "vector", intMan[x].vector);
        paramOut(os, "mask", intCtl[x].mask);
        paramOut(os, "pend", intCtl[x].pend);
    };
    for (int x = 0; x < MaxNiagaraProcs; x++) {
        nameOut(os, csprintf("%s.jIntBusy%d", name(), x));
        paramOut(os, "busy", jIntBusy[x].busy);
        paramOut(os, "source", jIntBusy[x].source);
    };
}

void
Iob::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(jIntVec);
    UNSERIALIZE_ARRAY(jBusData0, MaxNiagaraProcs);
    UNSERIALIZE_ARRAY(jBusData1, MaxNiagaraProcs);
    for (int x = 0; x < NumDeviceIds; x++) {
        paramIn(cp, csprintf("%s.Int%d", name(), x), "cpu", intMan[x].cpu);
        paramIn(cp, csprintf("%s.Int%d", name(), x), "vector", intMan[x].vector);
        paramIn(cp, csprintf("%s.Int%d", name(), x), "mask", intCtl[x].mask);
        paramIn(cp, csprintf("%s.Int%d", name(), x), "pend", intCtl[x].pend);
    };
    for (int x = 0; x < MaxNiagaraProcs; x++) {
        paramIn(cp, csprintf("%s.jIntBusy%d", name(), x), "busy", jIntBusy[x].busy);
        paramIn(cp, csprintf("%s.jIntBusy%d", name(), x), "source", jIntBusy[x].source);
    };
}




BEGIN_DECLARE_SIM_OBJECT_PARAMS(Iob)
    Param<Tick> pio_latency;
    SimObjectParam<Platform *> platform;
    SimObjectParam<System *> system;
END_DECLARE_SIM_OBJECT_PARAMS(Iob)

BEGIN_INIT_SIM_OBJECT_PARAMS(Iob)

    INIT_PARAM(pio_latency, "Programmed IO latency"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM(system, "system object")

END_INIT_SIM_OBJECT_PARAMS(Iob)

CREATE_SIM_OBJECT(Iob)
{
    Iob::Params *p = new Iob::Params;
    p->name = getInstanceName();
    p->pio_delay = pio_latency;
    p->platform = platform;
    p->system = system;
    return new Iob(p);
}

REGISTER_SIM_OBJECT("Iob", Iob)
