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
#include "dev/tsunami_io.hh"

using namespace std;

AlphaConsole::AlphaConsole(const string &name, SimConsole *cons,
                           SimpleDisk *d, int size, System *system,
                           BaseCPU *cpu, TsunamiIO *clock, int num_cpus,
                           Addr addr, Addr mask, MemoryController *mmu)
    : MmapDevice(name, addr, mask, mmu), disk(d), console(cons)
{
    mmu->add_child(this, Range<Addr>(addr, addr + size));

    consoleData = new uint8_t[size];
    memset(consoleData, 0, size);

    alphaAccess->last_offset = size - 1;
    alphaAccess->kernStart = system->getKernelStart();
    alphaAccess->kernEnd = system->getKernelEnd();
    alphaAccess->entryPoint = system->getKernelEntry();

    alphaAccess->version = ALPHA_ACCESS_VERSION;
    alphaAccess->numCPUs = num_cpus;
    alphaAccess->mem_size = system->physmem->size();
    alphaAccess->cpuClock = cpu->getFreq() / 1000000;
    alphaAccess->intrClockFrequency = clock->frequency();

    alphaAccess->diskUnit = 1;
}

Fault
AlphaConsole::read(MemReqPtr &req, uint8_t *data)
{
    memset(data, 0, req->size);
    uint64_t val;

    Addr daddr = req->paddr - addr;

    switch (daddr) {
      case offsetof(AlphaAccess, inputChar):
        val = console->console_in();
        break;

      default:
        val = *(uint64_t *)(consoleData + daddr);
        break;
    }

    DPRINTF(AlphaConsole, "read: offset=%#x val=%#x\n", daddr, val);

    switch (req->size) {
      case sizeof(uint32_t):
        *(uint32_t *)data = (uint32_t)val;
        break;

      case sizeof(uint64_t):
        *(uint64_t *)data = val;
        break;

      default:
        return Machine_Check_Fault;
    }


    return No_Fault;
}

Fault
AlphaConsole::write(MemReqPtr &req, const uint8_t *data)
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

    Addr daddr = req->paddr - addr;
    ExecContext *other_xc;

    switch (daddr) {
      case offsetof(AlphaAccess, diskUnit):
        alphaAccess->diskUnit = val;
        break;

      case offsetof(AlphaAccess, diskCount):
        alphaAccess->diskCount = val;
        break;

      case offsetof(AlphaAccess, diskPAddr):
        alphaAccess->diskPAddr = val;
        break;

      case offsetof(AlphaAccess, diskBlock):
        alphaAccess->diskBlock = val;
        break;

      case offsetof(AlphaAccess, diskOperation):
        if (val == 0x13)
            disk->read(alphaAccess->diskPAddr, alphaAccess->diskBlock,
                       alphaAccess->diskCount);
        else
            panic("Invalid disk operation!");

        break;

      case offsetof(AlphaAccess, outputChar):
        console->out((char)(val & 0xff), false);
        break;

      case offsetof(AlphaAccess, bootStrapImpure):
        alphaAccess->bootStrapImpure = val;
        break;

      case offsetof(AlphaAccess, bootStrapCPU):
        warn("%d: Trying to launch another CPU!", curTick);
        assert(val > 0 && "Must not access primary cpu");

        other_xc = req->xc->system->execContexts[val];
        other_xc->regs.intRegFile[16] = val;
        other_xc->regs.ipr[TheISA::IPR_PALtemp16] = val;
        other_xc->regs.intRegFile[0] = val;
        other_xc->regs.intRegFile[30] = alphaAccess->bootStrapImpure;
        other_xc->activate(); //Start the cpu
        break;

      default:
        return Machine_Check_Fault;
    }

    return No_Fault;
}

void
AlphaAccess::serialize(ostream &os)
{
    SERIALIZE_SCALAR(last_offset);
    SERIALIZE_SCALAR(version);
    SERIALIZE_SCALAR(numCPUs);
    SERIALIZE_SCALAR(mem_size);
    SERIALIZE_SCALAR(cpuClock);
    SERIALIZE_SCALAR(intrClockFrequency);
    SERIALIZE_SCALAR(kernStart);
    SERIALIZE_SCALAR(kernEnd);
    SERIALIZE_SCALAR(entryPoint);
    SERIALIZE_SCALAR(diskUnit);
    SERIALIZE_SCALAR(diskCount);
    SERIALIZE_SCALAR(diskPAddr);
    SERIALIZE_SCALAR(diskBlock);
    SERIALIZE_SCALAR(diskOperation);
    SERIALIZE_SCALAR(outputChar);
    SERIALIZE_SCALAR(inputChar);
    SERIALIZE_SCALAR(bootStrapImpure);
    SERIALIZE_SCALAR(bootStrapCPU);
}

void
AlphaAccess::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(last_offset);
    UNSERIALIZE_SCALAR(version);
    UNSERIALIZE_SCALAR(numCPUs);
    UNSERIALIZE_SCALAR(mem_size);
    UNSERIALIZE_SCALAR(cpuClock);
    UNSERIALIZE_SCALAR(intrClockFrequency);
    UNSERIALIZE_SCALAR(kernStart);
    UNSERIALIZE_SCALAR(kernEnd);
    UNSERIALIZE_SCALAR(entryPoint);
    UNSERIALIZE_SCALAR(diskUnit);
    UNSERIALIZE_SCALAR(diskCount);
    UNSERIALIZE_SCALAR(diskPAddr);
    UNSERIALIZE_SCALAR(diskBlock);
    UNSERIALIZE_SCALAR(diskOperation);
    UNSERIALIZE_SCALAR(outputChar);
    UNSERIALIZE_SCALAR(inputChar);
    UNSERIALIZE_SCALAR(bootStrapImpure);
    UNSERIALIZE_SCALAR(bootStrapCPU);
}

void
AlphaConsole::serialize(ostream &os)
{
    alphaAccess->serialize(os);
}

void
AlphaConsole::unserialize(Checkpoint *cp, const std::string &section)
{
    alphaAccess->unserialize(cp, section);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(AlphaConsole)

    SimObjectParam<SimConsole *> sim_console;
    SimObjectParam<SimpleDisk *> disk;
    Param<int> num_cpus;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    SimObjectParam<System *> system;
    SimObjectParam<BaseCPU *> cpu;
    SimObjectParam<TsunamiIO *> clock;

END_DECLARE_SIM_OBJECT_PARAMS(AlphaConsole)

BEGIN_INIT_SIM_OBJECT_PARAMS(AlphaConsole)

    INIT_PARAM(sim_console, "The Simulator Console"),
    INIT_PARAM(disk, "Simple Disk"),
    INIT_PARAM_DFLT(num_cpus, "Number of CPU's", 1),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(cpu, "Processor"),
    INIT_PARAM(clock, "Turbolaser Clock")

END_INIT_SIM_OBJECT_PARAMS(AlphaConsole)

CREATE_SIM_OBJECT(AlphaConsole)
{
    return  new AlphaConsole(getInstanceName(), sim_console, disk,
                             system, cpu, clock, num_cpus, mmu, addr);
}

REGISTER_SIM_OBJECT("AlphaConsole", AlphaConsole)
