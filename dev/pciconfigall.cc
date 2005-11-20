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

/* @file
 * PCI Configspace implementation
 */

#include <deque>
#include <string>
#include <vector>
#include <bitset>

#include "base/trace.hh"
#include "dev/pciconfigall.hh"
#include "dev/pcidev.hh"
#include "dev/pcireg.h"
#include "mem/bus/bus.hh"
#include "mem/bus/pio_interface.hh"
#include "mem/bus/pio_interface_impl.hh"
#include "mem/functional/memory_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

PciConfigAll::PciConfigAll(const string &name,
                           Addr a, MemoryController *mmu,
                           HierParams *hier, Bus *pio_bus, Tick pio_latency)
    : PioDevice(name, NULL), addr(a)
{
    mmu->add_child(this, RangeSize(addr, size));

    if (pio_bus) {
        pioInterface = newPioInterface(name + ".pio", hier, pio_bus, this,
                                      &PciConfigAll::cacheAccess);
        pioInterface->addAddrRange(RangeSize(addr, size));
        pioLatency = pio_latency * pio_bus->clockRate;
    }

    // Make all the pointers to devices null
    for(int x=0; x < MAX_PCI_DEV; x++)
        for(int y=0; y < MAX_PCI_FUNC; y++)
            devices[x][y] = NULL;
}

// If two interrupts share the same line largely bad things will happen.
// Since we don't track how many times an interrupt was set and correspondingly
// cleared two devices on the same interrupt line and assert and deassert each
// others interrupt "line". Interrupts will not work correctly.
void
PciConfigAll::startup()
{
    bitset<256> intLines;
    PciDev *tempDev;
    uint8_t intline;

    for (int x = 0; x < MAX_PCI_DEV; x++) {
        for (int y = 0; y < MAX_PCI_FUNC; y++) {
           if (devices[x][y] != NULL) {
               tempDev = devices[x][y];
               intline = tempDev->interruptLine();
               if (intLines.test(intline))
                   warn("Interrupt line %#X is used multiple times"
                        "(You probably want to fix this).\n", (uint32_t)intline);
               else
                   intLines.set(intline);
           } // devices != NULL
        } // PCI_FUNC
    } // PCI_DEV

}

Fault
PciConfigAll::read(MemReqPtr &req, uint8_t *data)
{

    Addr daddr = (req->paddr - (addr & EV5::PAddrImplMask));

    DPRINTF(PciConfigAll, "read  va=%#x da=%#x size=%d\n",
            req->vaddr, daddr, req->size);

    int device = (daddr >> 11) & 0x1F;
    int func = (daddr >> 8) & 0x7;
    int reg = daddr & 0xFF;

    if (devices[device][func] == NULL) {
        switch (req->size) {
           // case sizeof(uint64_t):
           //     *(uint64_t*)data = 0xFFFFFFFFFFFFFFFF;
           //     return No_Fault;
            case sizeof(uint32_t):
                *(uint32_t*)data = 0xFFFFFFFF;
                return No_Fault;
            case sizeof(uint16_t):
                *(uint16_t*)data = 0xFFFF;
                return No_Fault;
            case sizeof(uint8_t):
                *(uint8_t*)data = 0xFF;
                return No_Fault;
            default:
                panic("invalid access size(?) for PCI configspace!\n");
        }
    } else {
        switch (req->size) {
            case sizeof(uint32_t):
            case sizeof(uint16_t):
            case sizeof(uint8_t):
                devices[device][func]->readConfig(reg, req->size, data);
                return No_Fault;
            default:
                panic("invalid access size(?) for PCI configspace!\n");
        }
    }

    DPRINTFN("PCI Configspace  ERROR: read  daddr=%#x size=%d\n",
             daddr, req->size);

    return No_Fault;
}

Fault
PciConfigAll::write(MemReqPtr &req, const uint8_t *data)
{
    Addr daddr = (req->paddr - (addr & EV5::PAddrImplMask));

    int device = (daddr >> 11) & 0x1F;
    int func = (daddr >> 8) & 0x7;
    int reg = daddr & 0xFF;

    if (devices[device][func] == NULL)
        panic("Attempting to write to config space on non-existant device\n");
    else if (req->size != sizeof(uint8_t) &&
             req->size != sizeof(uint16_t) &&
             req->size != sizeof(uint32_t))
        panic("invalid access size(?) for PCI configspace!\n");

    DPRINTF(PciConfigAll, "write - va=%#x size=%d data=%#x\n",
            req->vaddr, req->size, *(uint32_t*)data);

    devices[device][func]->writeConfig(reg, req->size, data);

    return No_Fault;
}

void
PciConfigAll::serialize(std::ostream &os)
{
    /*
     * There is no state associated with this object that requires
     * serialization.  The only real state are the device pointers
     * which are all setup by the constructor of the PciDev class
     */
}

void
PciConfigAll::unserialize(Checkpoint *cp, const std::string &section)
{
    /*
     * There is no state associated with this object that requires
     * serialization.  The only real state are the device pointers
     * which are all setup by the constructor of the PciDev class
     */
}

Tick
PciConfigAll::cacheAccess(MemReqPtr &req)
{
    return curTick + pioLatency;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(PciConfigAll)

    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    Param<Addr> mask;
    SimObjectParam<Bus*> pio_bus;
    Param<Tick> pio_latency;
    SimObjectParam<HierParams *> hier;

END_DECLARE_SIM_OBJECT_PARAMS(PciConfigAll)

BEGIN_INIT_SIM_OBJECT_PARAMS(PciConfigAll)

    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask"),
    INIT_PARAM_DFLT(pio_bus, "The IO Bus to attach to", NULL),
    INIT_PARAM_DFLT(pio_latency, "Programmed IO latency in bus cycles", 1),
    INIT_PARAM_DFLT(hier, "Hierarchy global variables", &defaultHierParams)

END_INIT_SIM_OBJECT_PARAMS(PciConfigAll)

CREATE_SIM_OBJECT(PciConfigAll)
{
    return new PciConfigAll(getInstanceName(), addr, mmu, hier, pio_bus,
                            pio_latency);
}

REGISTER_SIM_OBJECT("PciConfigAll", PciConfigAll)

#endif // DOXYGEN_SHOULD_SKIP_THIS
