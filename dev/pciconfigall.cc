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
 * PCI Configspace implementation
 */

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "dev/scsi_ctrl.hh"
#include "dev/pciconfigall.hh"
#include "dev/pcidev.hh"
#include "dev/tsunamireg.h"
#include "dev/tsunami.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

PciConfigAll::PciConfigAll(const string &name, Tsunami *t, Addr a,
                           MemoryController *mmu)
    : FunctionalMemory(name), addr(a), tsunami(t)
{
    mmu->add_child(this, Range<Addr>(addr, addr + size));

    // Put back pointer in tsunami
    tsunami->pciconfig = this;

    // Make all the pointers to devices null
    for(int x=0; x < MAX_PCI_DEV; x++)
        for(int y=0; y < MAX_PCI_FUNC; y++)
          devices[x][y] = NULL;
}

Fault
PciConfigAll::read(MemReqPtr &req, uint8_t *data)
{
    DPRINTF(PciConfigAll, "read  va=%#x size=%d\n",
            req->vaddr, req->size);

    Addr daddr = (req->paddr - (addr & PA_IMPL_MASK));

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
                devices[device][func]->ReadConfig(reg, req->size, data);
                return No_Fault;
            default:
                panic("invalid access size(?) for PCI configspace!\n");
        }
    }

    DPRINTFN("Tsunami PCI Configspace  ERROR: read  daddr=%#x size=%d\n",
             daddr, req->size);

    return No_Fault;
}

Fault
PciConfigAll::write(MemReqPtr &req, const uint8_t *data)
{
    Addr daddr = (req->paddr - (addr & PA_IMPL_MASK));

    int device = (daddr >> 11) & 0x1F;
    int func = (daddr >> 8) & 0x7;
    int reg = daddr & 0xFF;

    union {
        uint8_t byte_value;
        uint16_t half_value;
        uint32_t word_value;
    };

    if (devices[device][func] == NULL)
        panic("Attempting to write to config space on non-existant device\n");
    else {
            switch (req->size) {
            case sizeof(uint8_t):
                byte_value = *(uint8_t*)data;
                break;
            case sizeof(uint16_t):
                half_value = *(uint16_t*)data;
                break;
            case sizeof(uint32_t):
                word_value = *(uint32_t*)data;
                break;
            default:
                panic("invalid access size(?) for PCI configspace!\n");
            }
    }

    DPRINTF(PciConfigAll, "write - va=%#x size=%d data=%#x\n",
            req->vaddr, req->size, word_value);

    devices[device][func]->WriteConfig(reg, req->size, word_value);

    return No_Fault;
}

void
PciConfigAll::serialize(std::ostream &os)
{
    // code should be written
}

void
PciConfigAll::unserialize(Checkpoint *cp, const std::string &section)
{
    //code should be written
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_DECLARE_SIM_OBJECT_PARAMS(PciConfigAll)

    SimObjectParam<Tsunami *> tsunami;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    Param<Addr> mask;

END_DECLARE_SIM_OBJECT_PARAMS(PciConfigAll)

BEGIN_INIT_SIM_OBJECT_PARAMS(PciConfigAll)

    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask")

END_INIT_SIM_OBJECT_PARAMS(PciConfigAll)

CREATE_SIM_OBJECT(PciConfigAll)
{
    return new PciConfigAll(getInstanceName(), tsunami, addr, mmu);
}

REGISTER_SIM_OBJECT("PciConfigAll", PciConfigAll)

#endif // DOXYGEN_SHOULD_SKIP_THIS
