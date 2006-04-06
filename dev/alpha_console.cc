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

#include "arch/alpha/system.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "dev/alpha_console.hh"
#include "dev/platform.hh"
#include "dev/simconsole.hh"
#include "dev/simple_disk.hh"
#include "mem/physical.hh"
#include "sim/builder.hh"
#include "sim/sim_object.hh"

using namespace std;
using namespace AlphaISA;

AlphaConsole::AlphaConsole(Params *p)
    : BasicPioDevice(p), disk(p->disk),
      console(params()->cons), system(params()->alpha_sys), cpu(params()->cpu)
{

    pioSize = sizeof(struct AlphaAccess);

    alphaAccess = new Access();
    alphaAccess->last_offset = pioSize - 1;

    alphaAccess->version = ALPHA_ACCESS_VERSION;
    alphaAccess->diskUnit = 1;

    alphaAccess->diskCount = 0;
    alphaAccess->diskPAddr = 0;
    alphaAccess->diskBlock = 0;
    alphaAccess->diskOperation = 0;
    alphaAccess->outputChar = 0;
    alphaAccess->inputChar = 0;
    bzero(alphaAccess->cpuStack, sizeof(alphaAccess->cpuStack));

    system->setAlphaAccess(pioAddr);
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
    alphaAccess->intrClockFrequency = params()->platform->intrFrequency();
}

void
AlphaConsole::addressRanges(AddrRangeList &range_list)
{
    assert(pioSize != 0);
    range_list.clear();
    range_list.push_back(RangeSize(pioAddr, sizeof(struct AlphaAccess)));
}


Tick
AlphaConsole::read(Packet &pkt)
{

    /** XXX Do we want to push the addr munging to a bus brige or something? So
     * the device has it's physical address and then the bridge adds on whatever
     * machine dependent address swizzle is required?
     */

    assert(pkt.result == Unknown);
    assert(pkt.addr >= pioAddr && pkt.addr < pioAddr + pioSize);

    pkt.time = curTick + pioDelay;
    Addr daddr = pkt.addr - pioAddr;

    uint32_t *data32;
    uint64_t *data64;

    switch (pkt.size)
    {
        case sizeof(uint32_t):
            if (!pkt.data) {
                data32 = new uint32_t;
                pkt.data = (uint8_t*)data32;
            }
            else
                data32 = (uint32_t*)pkt.data;

            switch (daddr)
            {
                case offsetof(AlphaAccess, last_offset):
                    *data32 = alphaAccess->last_offset;
                    break;
                case offsetof(AlphaAccess, version):
                    *data32 = alphaAccess->version;
                    break;
                case offsetof(AlphaAccess, numCPUs):
                    *data32 = alphaAccess->numCPUs;
                    break;
                case offsetof(AlphaAccess, intrClockFrequency):
                    *data32 = alphaAccess->intrClockFrequency;
                    break;
                default:
                    /* Old console code read in everyting as a 32bit int
                     * we now break that for better error checking.
                     */
                    pkt.result = BadAddress;
            }
            DPRINTF(AlphaConsole, "read: offset=%#x val=%#x\n", daddr, *data32);
            break;
        case sizeof(uint64_t):
            if (!pkt.data) {
                data64 = new uint64_t;
                pkt.data = (uint8_t*)data64;
            }
            else
                data64 = (uint64_t*)pkt.data;
            switch (daddr)
            {
                case offsetof(AlphaAccess, inputChar):
                    *data64 = console->console_in();
                    break;
                case offsetof(AlphaAccess, cpuClock):
                    *data64 = alphaAccess->cpuClock;
                    break;
                case offsetof(AlphaAccess, mem_size):
                    *data64 = alphaAccess->mem_size;
                    break;
                case offsetof(AlphaAccess, kernStart):
                    *data64 = alphaAccess->kernStart;
                    break;
                case offsetof(AlphaAccess, kernEnd):
                    *data64 = alphaAccess->kernEnd;
                    break;
                case offsetof(AlphaAccess, entryPoint):
                    *data64 = alphaAccess->entryPoint;
                    break;
                case offsetof(AlphaAccess, diskUnit):
                    *data64 = alphaAccess->diskUnit;
                    break;
                case offsetof(AlphaAccess, diskCount):
                    *data64 = alphaAccess->diskCount;
                    break;
                case offsetof(AlphaAccess, diskPAddr):
                    *data64 = alphaAccess->diskPAddr;
                    break;
                case offsetof(AlphaAccess, diskBlock):
                    *data64 = alphaAccess->diskBlock;
                    break;
                case offsetof(AlphaAccess, diskOperation):
                    *data64 = alphaAccess->diskOperation;
                    break;
                case offsetof(AlphaAccess, outputChar):
                    *data64 = alphaAccess->outputChar;
                    break;
                default:
                    int cpunum = (daddr - offsetof(AlphaAccess, cpuStack)) /
                                 sizeof(alphaAccess->cpuStack[0]);

                    if (cpunum >= 0 && cpunum < 64)
                        *data64 = alphaAccess->cpuStack[cpunum];
                    else
                        panic("Unknown 64bit access, %#x\n", daddr);
            }
            DPRINTF(AlphaConsole, "read: offset=%#x val=%#x\n", daddr, *data64);
            break;
        default:
            pkt.result = BadAddress;
    }
    if (pkt.result == Unknown) pkt.result = Success;
    return pioDelay;
}

Tick
AlphaConsole::write(Packet &pkt)
{
    pkt.time = curTick + pioDelay;

    assert(pkt.result == Unknown);
    assert(pkt.addr >= pioAddr && pkt.addr < pioAddr + pioSize);
    Addr daddr = pkt.addr - pioAddr;

    uint64_t val = *(uint64_t *)pkt.data;
    assert(pkt.size == sizeof(uint64_t));

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

      default:
        int cpunum = (daddr - offsetof(AlphaAccess, cpuStack)) /
                     sizeof(alphaAccess->cpuStack[0]);
        warn("%d: Trying to launch CPU number %d!", curTick, cpunum);
        assert(val > 0 && "Must not access primary cpu");
        if (cpunum >= 0 && cpunum < 64)
            alphaAccess->cpuStack[cpunum] = val;
        else
            panic("Unknown 64bit access, %#x\n", daddr);
    }

    pkt.result = Success;

    return pioDelay;
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
    SERIALIZE_ARRAY(cpuStack,64);
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
    UNSERIALIZE_ARRAY(cpuStack, 64);
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
    Param<Addr> addr;
    SimObjectParam<AlphaSystem *> system;
    SimObjectParam<BaseCPU *> cpu;
    SimObjectParam<Platform *> platform;
    Param<Tick> pio_latency;

END_DECLARE_SIM_OBJECT_PARAMS(AlphaConsole)

BEGIN_INIT_SIM_OBJECT_PARAMS(AlphaConsole)

    INIT_PARAM(sim_console, "The Simulator Console"),
    INIT_PARAM(disk, "Simple Disk"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(cpu, "Processor"),
    INIT_PARAM(platform, "platform"),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency", 1000)

END_INIT_SIM_OBJECT_PARAMS(AlphaConsole)

CREATE_SIM_OBJECT(AlphaConsole)
{
    AlphaConsole::Params *p = new AlphaConsole::Params;
    p->name = getInstanceName();
    p->platform = platform;
    p->pio_addr = addr;
    p->pio_delay = pio_latency;
    p->cons = sim_console;
    p->disk = disk;
    p->alpha_sys = system;
    p->system = system;
    p->cpu = cpu;
    return new AlphaConsole(p);
}

REGISTER_SIM_OBJECT("AlphaConsole", AlphaConsole)
