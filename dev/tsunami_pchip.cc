/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
 * Tsunami PChip (pci)
 */

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "dev/tsunami_pchip.hh"
#include "dev/tsunamireg.h"
#include "dev/tsunami.hh"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional_mem/memory_control.hh"
#include "mem/functional_mem/physical_memory.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

TsunamiPChip::TsunamiPChip(const string &name, Tsunami *t, Addr a,
                           MemoryController *mmu, HierParams *hier,
                           Bus *bus, Tick pio_latency)
    : PioDevice(name, t), addr(a), tsunami(t)
{
    mmu->add_child(this, RangeSize(addr, size));

    for (int i = 0; i < 4; i++) {
        wsba[i] = 0;
        wsm[i] = 0;
        tba[i] = 0;
    }

    if (bus) {
        pioInterface = newPioInterface(name, hier, bus, this,
                                      &TsunamiPChip::cacheAccess);
        pioInterface->addAddrRange(RangeSize(addr, size));
        pioLatency = pio_latency * bus->clockRate;
    }


    // initialize pchip control register
    pctl = (ULL(0x1) << 20) | (ULL(0x1) << 32) | (ULL(0x2) << 36);

    //Set back pointer in tsunami
    tsunami->pchip = this;
}

Fault
TsunamiPChip::read(MemReqPtr &req, uint8_t *data)
{
    DPRINTF(Tsunami, "read  va=%#x size=%d\n",
            req->vaddr, req->size);

    Addr daddr = (req->paddr - (addr & EV5::PAddrImplMask)) >> 6;

    switch (req->size) {

      case sizeof(uint64_t):
          switch(daddr) {
              case TSDEV_PC_WSBA0:
                    *(uint64_t*)data = wsba[0];
                    return No_Fault;
              case TSDEV_PC_WSBA1:
                    *(uint64_t*)data = wsba[1];
                    return No_Fault;
              case TSDEV_PC_WSBA2:
                    *(uint64_t*)data = wsba[2];
                    return No_Fault;
              case TSDEV_PC_WSBA3:
                    *(uint64_t*)data = wsba[3];
                    return No_Fault;
              case TSDEV_PC_WSM0:
                    *(uint64_t*)data = wsm[0];
                    return No_Fault;
              case TSDEV_PC_WSM1:
                    *(uint64_t*)data = wsm[1];
                    return No_Fault;
              case TSDEV_PC_WSM2:
                    *(uint64_t*)data = wsm[2];
                    return No_Fault;
              case TSDEV_PC_WSM3:
                    *(uint64_t*)data = wsm[3];
                    return No_Fault;
              case TSDEV_PC_TBA0:
                    *(uint64_t*)data = tba[0];
                    return No_Fault;
              case TSDEV_PC_TBA1:
                    *(uint64_t*)data = tba[1];
                    return No_Fault;
              case TSDEV_PC_TBA2:
                    *(uint64_t*)data = tba[2];
                    return No_Fault;
              case TSDEV_PC_TBA3:
                    *(uint64_t*)data = tba[3];
                    return No_Fault;
              case TSDEV_PC_PCTL:
                    *(uint64_t*)data = pctl;
                    return No_Fault;
              case TSDEV_PC_PLAT:
                    panic("PC_PLAT not implemented\n");
              case TSDEV_PC_RES:
                    panic("PC_RES not implemented\n");
              case TSDEV_PC_PERROR:
                    *(uint64_t*)data = 0x00;
                    return No_Fault;
              case TSDEV_PC_PERRMASK:
                    *(uint64_t*)data = 0x00;
                    return No_Fault;
              case TSDEV_PC_PERRSET:
                    panic("PC_PERRSET not implemented\n");
              case TSDEV_PC_TLBIV:
                    panic("PC_TLBIV not implemented\n");
              case TSDEV_PC_TLBIA:
                    *(uint64_t*)data = 0x00; // shouldn't be readable, but linux
                    return No_Fault;
              case TSDEV_PC_PMONCTL:
                    panic("PC_PMONCTL not implemented\n");
              case TSDEV_PC_PMONCNT:
                    panic("PC_PMONCTN not implemented\n");
              default:
                  panic("Default in PChip Read reached reading 0x%x\n", daddr);

           } // uint64_t

      break;
      case sizeof(uint32_t):
      case sizeof(uint16_t):
      case sizeof(uint8_t):
      default:
        panic("invalid access size(?) for tsunami register!\n\n");
    }
    DPRINTFN("Tsunami PChip ERROR: read  daddr=%#x size=%d\n", daddr, req->size);

    return No_Fault;
}

Fault
TsunamiPChip::write(MemReqPtr &req, const uint8_t *data)
{
    DPRINTF(Tsunami, "write - va=%#x size=%d \n",
            req->vaddr, req->size);

    Addr daddr = (req->paddr - (addr & EV5::PAddrImplMask)) >> 6;

    switch (req->size) {

      case sizeof(uint64_t):
          switch(daddr) {
              case TSDEV_PC_WSBA0:
                    wsba[0] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSBA1:
                    wsba[1] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSBA2:
                    wsba[2] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSBA3:
                    wsba[3] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSM0:
                    wsm[0] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSM1:
                    wsm[1] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSM2:
                    wsm[2] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSM3:
                    wsm[3] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_TBA0:
                    tba[0] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_TBA1:
                    tba[1] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_TBA2:
                    tba[2] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_TBA3:
                    tba[3] = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_PCTL:
                    pctl = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_PLAT:
                    panic("PC_PLAT not implemented\n");
              case TSDEV_PC_RES:
                    panic("PC_RES not implemented\n");
              case TSDEV_PC_PERROR:
                    return No_Fault;
              case TSDEV_PC_PERRMASK:
                    panic("PC_PERRMASK not implemented\n");
              case TSDEV_PC_PERRSET:
                    panic("PC_PERRSET not implemented\n");
              case TSDEV_PC_TLBIV:
                    panic("PC_TLBIV not implemented\n");
              case TSDEV_PC_TLBIA:
                    return No_Fault; // value ignored, supposted to invalidate SG TLB
              case TSDEV_PC_PMONCTL:
                    panic("PC_PMONCTL not implemented\n");
              case TSDEV_PC_PMONCNT:
                    panic("PC_PMONCTN not implemented\n");
              default:
                  panic("Default in PChip Read reached reading 0x%x\n", daddr);

           } // uint64_t

      break;
      case sizeof(uint32_t):
      case sizeof(uint16_t):
      case sizeof(uint8_t):
      default:
        panic("invalid access size(?) for tsunami register!\n\n");
    }

    DPRINTFN("Tsunami ERROR: write daddr=%#x size=%d\n", daddr, req->size);

    return No_Fault;
}

#define DMA_ADDR_MASK ULL(0x3ffffffff)

Addr
TsunamiPChip::translatePciToDma(Addr busAddr)
{
    // compare the address to the window base registers
    uint64_t tbaMask = 0;
    uint64_t baMask = 0;

    uint64_t windowMask = 0;
    uint64_t windowBase = 0;

    uint64_t pteEntry = 0;

    Addr pteAddr;
    Addr dmaAddr;

#if 0
    DPRINTF(IdeDisk, "Translation for bus address: %#x\n", busAddr);
    for (int i = 0; i < 4; i++) {
        DPRINTF(IdeDisk, "(%d) base:%#x mask:%#x\n",
                i, wsba[i], wsm[i]);

        windowBase = wsba[i];
        windowMask = ~wsm[i] & (ULL(0xfff) << 20);

        if ((busAddr & windowMask) == (windowBase & windowMask)) {
            DPRINTF(IdeDisk, "Would have matched %d (wb:%#x wm:%#x --> ba&wm:%#x wb&wm:%#x)\n",
                    i, windowBase, windowMask, (busAddr & windowMask),
                    (windowBase & windowMask));
        }
    }
#endif

    for (int i = 0; i < 4; i++) {

        windowBase = wsba[i];
        windowMask = ~wsm[i] & (ULL(0xfff) << 20);

        if ((busAddr & windowMask) == (windowBase & windowMask)) {

            if (wsba[i] & 0x1) {   // see if enabled
                if (wsba[i] & 0x2) { // see if SG bit is set
                    /** @todo
                        This currently is faked by just doing a direct
                        read from memory, however, to be realistic, this
                        needs to actually do a bus transaction.  The process
                        is explained in the tsunami documentation on page
                        10-12 and basically munges the address to look up a
                        PTE from a table in memory and then uses that mapping
                        to create an address for the SG page
                    */

                    tbaMask = ~(((wsm[i] & (ULL(0xfff) << 20)) >> 10) | ULL(0x3ff));
                    baMask = (wsm[i] & (ULL(0xfff) << 20)) | (ULL(0x7f) << 13);
                    pteAddr = (tba[i] & tbaMask) | ((busAddr & baMask) >> 10);

                    memcpy((void *)&pteEntry,
                           tsunami->system->
                           physmem->dma_addr(pteAddr, sizeof(uint64_t)),
                           sizeof(uint64_t));

                    dmaAddr = ((pteEntry & ~ULL(0x1)) << 12) | (busAddr & ULL(0x1fff));

                } else {
                    baMask = (wsm[i] & (ULL(0xfff) << 20)) | ULL(0xfffff);
                    tbaMask = ~baMask;
                    dmaAddr = (tba[i] & tbaMask) | (busAddr & baMask);
                }

                return (dmaAddr & DMA_ADDR_MASK);
            }
        }
    }

    // if no match was found, then return the original address
    return busAddr;
}

void
TsunamiPChip::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(pctl);
    SERIALIZE_ARRAY(wsba, 4);
    SERIALIZE_ARRAY(wsm, 4);
    SERIALIZE_ARRAY(tba, 4);
}

void
TsunamiPChip::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(pctl);
    UNSERIALIZE_ARRAY(wsba, 4);
    UNSERIALIZE_ARRAY(wsm, 4);
    UNSERIALIZE_ARRAY(tba, 4);
}

Tick
TsunamiPChip::cacheAccess(MemReqPtr &req)
{
    return curTick + pioLatency;
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(TsunamiPChip)

    SimObjectParam<Tsunami *> tsunami;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    SimObjectParam<Bus*> io_bus;
    Param<Tick> pio_latency;
    SimObjectParam<HierParams *> hier;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiPChip)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiPChip)

    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM_DFLT(io_bus, "The IO Bus to attach to", NULL),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams)

END_INIT_SIM_OBJECT_PARAMS(TsunamiPChip)

CREATE_SIM_OBJECT(TsunamiPChip)
{
    return new TsunamiPChip(getInstanceName(), tsunami, addr, mmu, hier,
                            io_bus, pio_latency);
}

REGISTER_SIM_OBJECT("TsunamiPChip", TsunamiPChip)
