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
 *
 * Authors: Nathan Binkert
 *          Ali Saidi
 *          Steve Reinhardt
 *          Erik Hallnor
 */

/** @file
 * Mips Console Backdoor Definition
 */
#include <cstddef>
#include <string>

#include "arch/mips/system.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "dev/mips/backdoor.hh"
#include "dev/platform.hh"
#include "dev/simple_disk.hh"
#include "dev/terminal.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/MipsBackdoor.hh"
#include "sim/sim_object.hh"


using namespace std;
using namespace MipsISA;

MipsBackdoor::MipsBackdoor(const Params *p)
  : BasicPioDevice(p), disk(p->disk), terminal(p->terminal),
    system(p->system), cpu(p->cpu)
{

    pioSize = sizeof(struct MipsAccess);

    mipsAccess = new Access();
    mipsAccess->last_offset = pioSize - 1;

    mipsAccess->version = MIPS_ACCESS_VERSION;
    mipsAccess->diskUnit = 1;

    mipsAccess->diskCount = 0;
    mipsAccess->diskPAddr = 0;
    mipsAccess->diskBlock = 0;
    mipsAccess->diskOperation = 0;
    mipsAccess->outputChar = 0;
    mipsAccess->inputChar = 0;
    bzero(mipsAccess->cpuStack, sizeof(mipsAccess->cpuStack));

}

void
MipsBackdoor::startup()
{
    system->setMipsAccess(pioAddr);
    mipsAccess->numCPUs = system->numContexts();
    mipsAccess->kernStart = MipsISA::Phys2K0Seg(system->getKernelStart());
    mipsAccess->kernEnd = MipsISA::Phys2K0Seg(system->getKernelEnd());
    mipsAccess->entryPoint = MipsISA::Phys2K0Seg(system->getKernelEntry());
    mipsAccess->mem_size = system->physmem->size();
    mipsAccess->cpuClock = cpu->frequency() / 1000000; // In MHz
    mipsAccess->intrClockFrequency = params()->platform->intrFrequency();
}

Tick
MipsBackdoor::read(PacketPtr pkt)
{

    /** XXX Do we want to push the addr munging to a bus brige or something? So
     * the device has it's physical address and then the bridge adds on whatever
     * machine dependent address swizzle is required?
     */
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    pkt->allocate();

    switch (pkt->getSize())
    {
        case sizeof(uint32_t):
            switch (daddr)
            {
                case offsetof(MipsAccess, last_offset):
                    pkt->set(mipsAccess->last_offset);
                    break;
                case offsetof(MipsAccess, version):
                    pkt->set(mipsAccess->version);
                    break;
                case offsetof(MipsAccess, numCPUs):
                    pkt->set(mipsAccess->numCPUs);
                    break;
                case offsetof(MipsAccess, intrClockFrequency):
                    pkt->set(mipsAccess->intrClockFrequency);
                    break;
                case offsetof(MipsAccess, inputChar):
                    pkt->set(terminal->console_in());
                    break;
                case offsetof(MipsAccess, cpuClock):
                    pkt->set(mipsAccess->cpuClock);
                    break;
                case offsetof(MipsAccess, mem_size):
                    pkt->set(mipsAccess->mem_size);
                    break;
                case offsetof(MipsAccess, kernStart):
                    pkt->set(mipsAccess->kernStart);
                    break;
                case offsetof(MipsAccess, kernEnd):
                    pkt->set(mipsAccess->kernEnd);
                    break;
                case offsetof(MipsAccess, entryPoint):
                    pkt->set(mipsAccess->entryPoint);
                    break;
               case offsetof(MipsAccess, diskUnit):
                    pkt->set(mipsAccess->diskUnit);
                    break;
                case offsetof(MipsAccess, diskCount):
                    pkt->set(mipsAccess->diskCount);
                    break;
                case offsetof(MipsAccess, diskPAddr):
                    pkt->set(mipsAccess->diskPAddr);
                    break;
                case offsetof(MipsAccess, diskBlock):
                    pkt->set(mipsAccess->diskBlock);
                    break;
                case offsetof(MipsAccess, diskOperation):
                    pkt->set(mipsAccess->diskOperation);
                    break;
                case offsetof(MipsAccess, outputChar):
                    pkt->set(mipsAccess->outputChar);
                    break;
                default:
                    int cpunum = (daddr - offsetof(MipsAccess, cpuStack)) /
                                 sizeof(mipsAccess->cpuStack[0]);

                    if (cpunum >= 0 && cpunum < 64)
                        pkt->set(mipsAccess->cpuStack[cpunum]);
                    else
                        panic("Unknown 32bit access, %#x\n", daddr);
            }
            //DPRINTF(MipsBackdoor, "read: offset=%#x val=%#x\n", daddr,
                   // pkt->get<uint64_t>());
            break;
        default:
          pkt->setBadAddress();
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
MipsBackdoor::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr daddr = pkt->getAddr() - pioAddr;

    uint32_t val = pkt->get<uint32_t>();
    assert(pkt->getSize() == sizeof(uint32_t));
    switch (daddr) {
      case offsetof(MipsAccess, diskUnit):
        mipsAccess->diskUnit = val;
        break;

      case offsetof(MipsAccess, diskCount):
        mipsAccess->diskCount = val;
        break;

      case offsetof(MipsAccess, diskPAddr):
        mipsAccess->diskPAddr = val;
        break;

      case offsetof(MipsAccess, diskBlock):
        mipsAccess->diskBlock = val;
        break;

      case offsetof(MipsAccess, diskOperation):
        if (val == 0x13)
            disk->read(mipsAccess->diskPAddr, mipsAccess->diskBlock,
                       mipsAccess->diskCount);
        else
            panic("Invalid disk operation!");

        break;

      case offsetof(MipsAccess, outputChar):
        terminal->out((char)(val & 0xff));
        break;

      default:
        int cpunum = (daddr - offsetof(MipsAccess, cpuStack)) /
                     sizeof(mipsAccess->cpuStack[0]);
        warn("%d: Trying to launch CPU number %d!", curTick, cpunum);
        assert(val > 0 && "Must not access primary cpu");
        if (cpunum >= 0 && cpunum < 64)
            mipsAccess->cpuStack[cpunum] = val;
        else
            panic("Unknown 32bit access, %#x\n", daddr);
    }

    pkt->makeAtomicResponse();

    return pioDelay;
}

void
MipsBackdoor::Access::serialize(ostream &os)
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
MipsBackdoor::Access::unserialize(Checkpoint *cp, const std::string &section)
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
MipsBackdoor::serialize(ostream &os)
{
    mipsAccess->serialize(os);
}

void
MipsBackdoor::unserialize(Checkpoint *cp, const std::string &section)
{
    mipsAccess->unserialize(cp, section);
}

MipsBackdoor *
MipsBackdoorParams::create()
{
    return new MipsBackdoor(this);
}
