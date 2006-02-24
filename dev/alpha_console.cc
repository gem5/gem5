/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Alpha Console Definition
 */

#include <cstddef>
#include <cstdio>
#include <string>

#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "dev/alpha_console.hh"
#include "dev/simconsole.hh"
#include "dev/simple_disk.hh"
#include "dev/tsunami_io.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "sim/builder.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

using namespace std;
using namespace AlphaISA;

AlphaConsole::AlphaConsole(const string &name, SimConsole *cons, SimpleDisk *d,
                           System *s, BaseCPU *c, Platform *p,
                           MemoryController *mmu, Addr a,
                           HierParams *hier, Bus *pio_bus)
    : PioDevice(name, p), disk(d), console(cons), system(s), cpu(c), addr(a)
{
    mmu->add_child(this, RangeSize(addr, size));

    if (pio_bus) {
        pioInterface = newPioInterface(name + ".pio", hier, pio_bus, this,
                                       &AlphaConsole::cacheAccess);
        pioInterface->addAddrRange(RangeSize(addr, size));
    }

    alphaAccess = new Access;
    alphaAccess->last_offset = size - 1;

    alphaAccess->version = ALPHA_ACCESS_VERSION;
    alphaAccess->diskUnit = 1;

    alphaAccess->diskCount = 0;
    alphaAccess->diskPAddr = 0;
    alphaAccess->diskBlock = 0;
    alphaAccess->diskOperation = 0;
    alphaAccess->outputChar = 0;
    alphaAccess->inputChar = 0;
    alphaAccess->bootStrapImpure = 0;
    alphaAccess->bootStrapCPU = 0;
    alphaAccess->align2 = 0;

    system->setAlphaAccess(addr);
}

void
AlphaConsole::startup()
{
    alphaAccess->numCPUs = system->getNumCPUs();
    alphaAccess->kernStart = system->getKernelStart();
    alphaAccess->kernEnd = system->getKernelEnd();
    alphaAccess->entryPoint = system->getKernelEntry();
    alphaAccess->mem_size = system->physmem->size();
    alphaAccess->cpuClock = cpu->frequency() / 1000000; // In MHz
    alphaAccess->intrClockFrequency = platform->intrFrequency();
}

Fault
AlphaConsole::read(MemReqPtr &req, uint8_t *data)
{
    memset(data, 0, req->size);

    Addr daddr = req->paddr - (addr & EV5::PAddrImplMask);

    switch (req->size)
    {
        case sizeof(uint32_t):
            DPRINTF(AlphaConsole, "read: offset=%#x val=%#x\n", daddr,
                    *(uint32_t*)data);
            switch (daddr)
            {
                case offsetof(AlphaAccess, last_offset):
                    *(uint32_t*)data = alphaAccess->last_offset;
                    break;
                case offsetof(AlphaAccess, version):
                    *(uint32_t*)data = alphaAccess->version;
                    break;
                case offsetof(AlphaAccess, numCPUs):
                    *(uint32_t*)data = alphaAccess->numCPUs;
                    break;
                case offsetof(AlphaAccess, bootStrapCPU):
                    *(uint32_t*)data = alphaAccess->bootStrapCPU;
                    break;
                case offsetof(AlphaAccess, intrClockFrequency):
                    *(uint32_t*)data = alphaAccess->intrClockFrequency;
                    break;
                default:
                    // Old console code read in everyting as a 32bit int
                    *(uint32_t*)data = *(uint32_t*)(consoleData + daddr);

            }
            break;
        case sizeof(uint64_t):
            DPRINTF(AlphaConsole, "read: offset=%#x val=%#x\n", daddr,
                    *(uint64_t*)data);
            switch (daddr)
            {
                case offsetof(AlphaAccess, inputChar):
                    *(uint64_t*)data = console->console_in();
                    break;
                case offsetof(AlphaAccess, cpuClock):
                    *(uint64_t*)data = alphaAccess->cpuClock;
                    break;
                case offsetof(AlphaAccess, mem_size):
                    *(uint64_t*)data = alphaAccess->mem_size;
                    break;
                case offsetof(AlphaAccess, kernStart):
                    *(uint64_t*)data = alphaAccess->kernStart;
                    break;
                case offsetof(AlphaAccess, kernEnd):
                    *(uint64_t*)data = alphaAccess->kernEnd;
                    break;
                case offsetof(AlphaAccess, entryPoint):
                    *(uint64_t*)data = alphaAccess->entryPoint;
                    break;
                case offsetof(AlphaAccess, diskUnit):
                    *(uint64_t*)data = alphaAccess->diskUnit;
                    break;
                case offsetof(AlphaAccess, diskCount):
                    *(uint64_t*)data = alphaAccess->diskCount;
                    break;
                case offsetof(AlphaAccess, diskPAddr):
                    *(uint64_t*)data = alphaAccess->diskPAddr;
                    break;
                case offsetof(AlphaAccess, diskBlock):
                    *(uint64_t*)data = alphaAccess->diskBlock;
                    break;
                case offsetof(AlphaAccess, diskOperation):
                    *(uint64_t*)data = alphaAccess->diskOperation;
                    break;
                case offsetof(AlphaAccess, outputChar):
                    *(uint64_t*)data = alphaAccess->outputChar;
                    break;
                case offsetof(AlphaAccess, bootStrapImpure):
                    *(uint64_t*)data = alphaAccess->bootStrapImpure;
                    break;
                default:
                    panic("Unknown 64bit access, %#x\n", daddr);
            }
            break;
        default:
            return new MachineCheckFault;
    }

    return NoFault;
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
        return new MachineCheckFault;
    }

    Addr daddr = req->paddr - (addr & EV5::PAddrImplMask);
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
        console->out((char)(val & 0xff));
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
        return new MachineCheckFault;
    }

    return NoFault;
}

Tick
AlphaConsole::cacheAccess(MemReqPtr &req)
{
    return curTick + 1000;
}

void
AlphaConsole::Access::serialize(ostream &os)
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
AlphaConsole::Access::unserialize(Checkpoint *cp, const std::string &section)
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
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    SimObjectParam<System *> system;
    SimObjectParam<BaseCPU *> cpu;
    SimObjectParam<Platform *> platform;
    SimObjectParam<Bus*> pio_bus;
    Param<Tick> pio_latency;
    SimObjectParam<HierParams *> hier;

END_DECLARE_SIM_OBJECT_PARAMS(AlphaConsole)

BEGIN_INIT_SIM_OBJECT_PARAMS(AlphaConsole)

    INIT_PARAM(sim_console, "The Simulator Console"),
    INIT_PARAM(disk, "Simple Disk"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(cpu, "Processor"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM(pio_bus, "The IO Bus to attach to"),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency", 1000),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams)

END_INIT_SIM_OBJECT_PARAMS(AlphaConsole)

CREATE_SIM_OBJECT(AlphaConsole)
{
    return new AlphaConsole(getInstanceName(), sim_console, disk,
                            system, cpu, platform, mmu, addr, hier, pio_bus);
}

REGISTER_SIM_OBJECT("AlphaConsole", AlphaConsole)
