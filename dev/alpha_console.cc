/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

/* @file
 * System Console Definition
 */

#include <cstddef>
#include <cstdio>
#include <string>

#include "base/inifile.hh"
#include "base/str.hh"	// for to_number()
#include "base/trace.hh"
#include "cpu/base_cpu.hh"
#include "cpu/exec_context.hh"
#include "dev/alpha_console.hh"
#include "dev/console.hh"
#include "dev/simple_disk.hh"
#include "dev/tlaser_clock.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

AlphaConsole::AlphaConsole(const string &name, SimConsole *cons,
                           SimpleDisk *d, int size, System *system,
                           BaseCPU *cpu, TlaserClock *clock, int num_cpus,
                           Addr addr, Addr mask, MemoryController *mmu)
    : MmapDevice(name, addr, mask, mmu), disk(d), console(cons)
{
    consoleData = new uint8_t[size];
    memset(consoleData, 0, size);

    alphaAccess->last_offset = size - 1;
    alphaAccess->kernStart = system->getKernelStart();
    alphaAccess->kernEnd = system->getKernelEnd();
    alphaAccess->entryPoint = system->getKernelEntry();

    alphaAccess->version = ALPHA_ACCESS_VERSION;
    alphaAccess->numCPUs = num_cpus;
    alphaAccess->mem_size = system->physmem->getSize();
    alphaAccess->cpuClock = cpu->getFreq() / 1000000;
    alphaAccess->intrClockFrequency = clock->frequency();

    alphaAccess->diskUnit = 1;
}

Fault
AlphaConsole::read(MemReqPtr req, uint8_t *data)
{
    memset(data, 0, req->size);

    if (req->size == sizeof(uint32_t)) {
        Addr daddr = req->paddr & addr_mask;
        *(uint32_t *)data = *(uint32_t *)(consoleData + daddr);

#if 0
        DPRINTF(AlphaConsole, "read: offset=%#x val=%#x\n",
                daddr, *(uint32_t *)data);
#endif
    }

    return No_Fault;
}

Fault
AlphaConsole::write(MemReqPtr req, const uint8_t *data)
{
    uint64_t val;

    switch (req->size) {
      case sizeof(uint32_t):
        val = *(uint32_t *)data;
        break;
      case sizeof(uint64_t):
        val = *(uint64_t *)data;
        break;
      default:
        return Machine_Check_Fault;
    }

    Addr paddr = req->paddr & addr_mask;

    if (paddr == offsetof(AlphaAccess, diskUnit)) {
        alphaAccess->diskUnit = val;
        return No_Fault;
    }

    if (paddr == offsetof(AlphaAccess, diskCount)) {
        alphaAccess->diskCount = val;
        return No_Fault;
    }

    if (paddr == offsetof(AlphaAccess, diskPAddr)) {
        alphaAccess->diskPAddr = val;
        return No_Fault;
    }

    if (paddr == offsetof(AlphaAccess, diskBlock)) {
        alphaAccess->diskBlock = val;
        return No_Fault;
    }

    if (paddr == offsetof(AlphaAccess, diskOperation)) {
        if (val == 0x13)
            disk->read(alphaAccess->diskPAddr, alphaAccess->diskBlock,
                       alphaAccess->diskCount);
        else
            panic("Invalid disk operation!");

        return No_Fault;
    }

    if (paddr == offsetof(AlphaAccess, outputChar)) {
        console->out((char)(val & 0xff), false);
        return No_Fault;
    }

    if (paddr == offsetof(AlphaAccess, bootStrapImpure)) {
        alphaAccess->bootStrapImpure = val;
        return No_Fault;
    }

    if (paddr == offsetof(AlphaAccess, bootStrapCPU)) {
        warn("%d: Trying to launch another CPU!", curTick);
        int cpu = val;
        assert(cpu > 0 && "Must not access primary cpu");

        ExecContext *other_xc = req->xc->system->execContexts[cpu];
        other_xc->regs.intRegFile[16] = cpu;
        other_xc->regs.ipr[TheISA::IPR_PALtemp16] = cpu;
        other_xc->regs.intRegFile[0] = cpu;
        other_xc->regs.intRegFile[30] = alphaAccess->bootStrapImpure;
        other_xc->setStatus(ExecContext::Active); //Start the cpu
        return No_Fault;
    }

    return No_Fault;
}

void
AlphaAccess::serialize(ostream &os)
{
    SERIALIZE_MEMBER(last_offset);
    SERIALIZE_MEMBER(version);
    SERIALIZE_MEMBER(numCPUs);
    SERIALIZE_MEMBER(mem_size);
    SERIALIZE_MEMBER(cpuClock);
    SERIALIZE_MEMBER(intrClockFrequency);
    SERIALIZE_MEMBER(kernStart);
    SERIALIZE_MEMBER(kernEnd);
    SERIALIZE_MEMBER(entryPoint);
    SERIALIZE_MEMBER(diskUnit);
    SERIALIZE_MEMBER(diskCount);
    SERIALIZE_MEMBER(diskPAddr);
    SERIALIZE_MEMBER(diskBlock);
    SERIALIZE_MEMBER(diskOperation);
    SERIALIZE_MEMBER(outputChar);
    SERIALIZE_MEMBER(bootStrapImpure);
    SERIALIZE_MEMBER(bootStrapCPU);
}

void
AlphaAccess::unserialize(const IniFile *db, const std::string &section)
{
    UNSERIALIZE_MEMBER(last_offset);
    UNSERIALIZE_MEMBER(version);
    UNSERIALIZE_MEMBER(numCPUs);
    UNSERIALIZE_MEMBER(mem_size);
    UNSERIALIZE_MEMBER(cpuClock);
    UNSERIALIZE_MEMBER(intrClockFrequency);
    UNSERIALIZE_MEMBER(kernStart);
    UNSERIALIZE_MEMBER(kernEnd);
    UNSERIALIZE_MEMBER(entryPoint);
    UNSERIALIZE_MEMBER(diskUnit);
    UNSERIALIZE_MEMBER(diskCount);
    UNSERIALIZE_MEMBER(diskPAddr);
    UNSERIALIZE_MEMBER(diskBlock);
    UNSERIALIZE_MEMBER(diskOperation);
    UNSERIALIZE_MEMBER(outputChar);
    UNSERIALIZE_MEMBER(bootStrapImpure);
    UNSERIALIZE_MEMBER(bootStrapCPU);
}

void
AlphaConsole::serialize(ostream &os)
{
    alphaAccess->serialize(os);
}

void
AlphaConsole::unserialize(const IniFile *db, const std::string &section)
{
    alphaAccess->unserialize(db, section);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(AlphaConsole)

    SimObjectParam<SimConsole *> sim_console;
    SimObjectParam<SimpleDisk *> disk;
    Param<int> size;
    Param<int> num_cpus;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    Param<Addr> mask;
    SimObjectParam<System *> system;
    SimObjectParam<BaseCPU *> cpu;
    SimObjectParam<TlaserClock *> clock;

END_DECLARE_SIM_OBJECT_PARAMS(AlphaConsole)

BEGIN_INIT_SIM_OBJECT_PARAMS(AlphaConsole)

    INIT_PARAM(sim_console, "The Simulator Console"),
    INIT_PARAM(disk, "Simple Disk"),
    INIT_PARAM_DFLT(size, "AlphaConsole size", sizeof(AlphaAccess)),
    INIT_PARAM_DFLT(num_cpus, "Number of CPU's", 1),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(cpu, "Processor"),
    INIT_PARAM(clock, "Turbolaser Clock")

END_INIT_SIM_OBJECT_PARAMS(AlphaConsole)

CREATE_SIM_OBJECT(AlphaConsole)
{
    return  new AlphaConsole(getInstanceName(), sim_console,
                             disk, size, system,
                             cpu, clock, num_cpus,
                             addr, mask, mmu);
}

REGISTER_SIM_OBJECT("AlphaConsole", AlphaConsole)
