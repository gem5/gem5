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
 * Alpha Console Backdoor Definition
 */

#include <cstddef>
#include <string>

#include "arch/alpha/system.hh"
#include "base/inifile.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/AlphaBackdoor.hh"
#include "dev/alpha/backdoor.hh"
#include "dev/alpha/tsunami.hh"
#include "dev/alpha/tsunami_cchip.hh"
#include "dev/alpha/tsunami_io.hh"
#include "dev/platform.hh"
#include "dev/storage/simple_disk.hh"
#include "dev/terminal.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/AlphaBackdoor.hh"
#include "sim/sim_object.hh"

using namespace std;
using namespace AlphaISA;

AlphaBackdoor::AlphaBackdoor(const Params *p)
    : BasicPioDevice(p, sizeof(struct AlphaAccess)),
      disk(p->disk), terminal(p->terminal),
      system(p->system), cpu(p->cpu)
{
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
    std::memset(alphaAccess->cpuStack, 0, sizeof(alphaAccess->cpuStack));

}

void
AlphaBackdoor::startup()
{
    system->setAlphaAccess(pioAddr);
    alphaAccess->numCPUs = system->numContexts();
    alphaAccess->kernStart = system->getKernelStart();
    alphaAccess->kernEnd = system->getKernelEnd();
    alphaAccess->entryPoint = system->getKernelEntry();
    alphaAccess->mem_size = system->memSize();
    alphaAccess->cpuClock = cpu->frequency() / 1000000; // In MHz
    Tsunami *tsunami = dynamic_cast<Tsunami *>(params()->platform);
    if (!tsunami)
        fatal("Platform is not Tsunami.\n");
    alphaAccess->intrClockFrequency = tsunami->io->frequency();
}

Tick
AlphaBackdoor::read(PacketPtr pkt)
{

    /** XXX Do we want to push the addr munging to a bus brige or something? So
     * the device has it's physical address and then the bridge adds on whatever
     * machine dependent address swizzle is required?
     */

    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr daddr = pkt->getAddr() - pioAddr;

    pkt->makeAtomicResponse();

    switch (pkt->getSize())
    {
        case sizeof(uint32_t):
            switch (daddr)
            {
                case offsetof(AlphaAccess, last_offset):
                    pkt->set(alphaAccess->last_offset);
                    break;
                case offsetof(AlphaAccess, version):
                    pkt->set(alphaAccess->version);
                    break;
                case offsetof(AlphaAccess, numCPUs):
                    pkt->set(alphaAccess->numCPUs);
                    break;
                case offsetof(AlphaAccess, intrClockFrequency):
                    pkt->set(alphaAccess->intrClockFrequency);
                    break;
                default:
                    /* Old console code read in everyting as a 32bit int
                     * we now break that for better error checking.
                     */
                  pkt->setBadAddress();
            }
            DPRINTF(AlphaBackdoor, "read: offset=%#x val=%#x\n", daddr,
                    pkt->get<uint32_t>());
            break;
        case sizeof(uint64_t):
            switch (daddr)
            {
                case offsetof(AlphaAccess, inputChar):
                    pkt->set(terminal->console_in());
                    break;
                case offsetof(AlphaAccess, cpuClock):
                    pkt->set(alphaAccess->cpuClock);
                    break;
                case offsetof(AlphaAccess, mem_size):
                    pkt->set(alphaAccess->mem_size);
                    break;
                case offsetof(AlphaAccess, kernStart):
                    pkt->set(alphaAccess->kernStart);
                    break;
                case offsetof(AlphaAccess, kernEnd):
                    pkt->set(alphaAccess->kernEnd);
                    break;
                case offsetof(AlphaAccess, entryPoint):
                    pkt->set(alphaAccess->entryPoint);
                    break;
                case offsetof(AlphaAccess, diskUnit):
                    pkt->set(alphaAccess->diskUnit);
                    break;
                case offsetof(AlphaAccess, diskCount):
                    pkt->set(alphaAccess->diskCount);
                    break;
                case offsetof(AlphaAccess, diskPAddr):
                    pkt->set(alphaAccess->diskPAddr);
                    break;
                case offsetof(AlphaAccess, diskBlock):
                    pkt->set(alphaAccess->diskBlock);
                    break;
                case offsetof(AlphaAccess, diskOperation):
                    pkt->set(alphaAccess->diskOperation);
                    break;
                case offsetof(AlphaAccess, outputChar):
                    pkt->set(alphaAccess->outputChar);
                    break;
                default:
                    int cpunum = (daddr - offsetof(AlphaAccess, cpuStack)) /
                                 sizeof(alphaAccess->cpuStack[0]);

                    if (cpunum >= 0 && cpunum < 64)
                        pkt->set(alphaAccess->cpuStack[cpunum]);
                    else
                        panic("Unknown 64bit access, %#x\n", daddr);
            }
            DPRINTF(AlphaBackdoor, "read: offset=%#x val=%#x\n", daddr,
                    pkt->get<uint64_t>());
            break;
        default:
          pkt->setBadAddress();
    }
    return pioDelay;
}

Tick
AlphaBackdoor::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr daddr = pkt->getAddr() - pioAddr;

    uint64_t val = pkt->get<uint64_t>();
    assert(pkt->getSize() == sizeof(uint64_t));

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
        terminal->out((char)(val & 0xff));
        break;

      default:
        int cpunum = (daddr - offsetof(AlphaAccess, cpuStack)) /
                     sizeof(alphaAccess->cpuStack[0]);
        inform("Launching CPU %d @ %d", cpunum, curTick());
        assert(val > 0 && "Must not access primary cpu");
        if (cpunum >= 0 && cpunum < 64)
            alphaAccess->cpuStack[cpunum] = val;
        else
            panic("Unknown 64bit access, %#x\n", daddr);
    }

    pkt->makeAtomicResponse();

    return pioDelay;
}

void
AlphaBackdoor::Access::serialize(CheckpointOut &cp) const
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
AlphaBackdoor::Access::unserialize(CheckpointIn &cp)
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
AlphaBackdoor::serialize(CheckpointOut &cp) const
{
    alphaAccess->serialize(cp);
}

void
AlphaBackdoor::unserialize(CheckpointIn &cp)
{
    alphaAccess->unserialize(cp);
}

AlphaBackdoor *
AlphaBackdoorParams::create()
{
    return new AlphaBackdoor(this);
}
