/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Isa Fake Device implementation
 */

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "dev/isa_fake.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional/memory_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

IsaFake::IsaFake(const string &name, Addr a, MemoryController *mmu,
                         HierParams *hier, Bus *pio_bus, Addr size)
    : PioDevice(name, NULL), addr(a)
{
    mmu->add_child(this, RangeSize(addr, size));

    if (pio_bus) {
        pioInterface = newPioInterface(name + ".pio", hier, pio_bus, this,
                                      &IsaFake::cacheAccess);
        pioInterface->addAddrRange(RangeSize(addr, size));
    }
}

Fault *
IsaFake::read(MemReqPtr &req, uint8_t *data)
{
    DPRINTF(Tsunami, "read  va=%#x size=%d\n",
            req->vaddr, req->size);

#if TRACING_ON
    Addr daddr = (req->paddr - (addr & EV5::PAddrImplMask)) >> 6;
#endif

    switch (req->size) {

      case sizeof(uint64_t):
         *(uint64_t*)data = 0xFFFFFFFFFFFFFFFFULL;
         return NoFault;
      case sizeof(uint32_t):
         *(uint32_t*)data = 0xFFFFFFFF;
         return NoFault;
      case sizeof(uint16_t):
         *(uint16_t*)data = 0xFFFF;
         return NoFault;
      case sizeof(uint8_t):
         *(uint8_t*)data = 0xFF;
         return NoFault;

      default:
        panic("invalid access size(?) for PCI configspace!\n");
    }
    DPRINTFN("Isa FakeSMC  ERROR: read  daddr=%#x size=%d\n", daddr, req->size);

    return NoFault;
}

Fault *
IsaFake::write(MemReqPtr &req, const uint8_t *data)
{
    DPRINTF(Tsunami, "write - va=%#x size=%d \n",
            req->vaddr, req->size);

    //:Addr daddr = (req->paddr & addr_mask) >> 6;

    return NoFault;
}

Tick
IsaFake::cacheAccess(MemReqPtr &req)
{
    return curTick;
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(IsaFake)

    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    SimObjectParam<Bus*> pio_bus;
    Param<Tick> pio_latency;
    SimObjectParam<HierParams *> hier;
    Param<Addr> size;

END_DECLARE_SIM_OBJECT_PARAMS(IsaFake)

BEGIN_INIT_SIM_OBJECT_PARAMS(IsaFake)

    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM_DFLT(pio_bus, "The IO Bus to attach to", NULL),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency", 1000),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams),
    INIT_PARAM_DFLT(size, "Size of address range", 0x8)

END_INIT_SIM_OBJECT_PARAMS(IsaFake)

CREATE_SIM_OBJECT(IsaFake)
{
    return new IsaFake(getInstanceName(), addr, mmu, hier, pio_bus, size);
}

REGISTER_SIM_OBJECT("IsaFake", IsaFake)
