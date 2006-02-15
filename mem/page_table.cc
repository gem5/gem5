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

/**
 * @file
 * Definitions of page table.
 */
#include <string>
#include <map>
#include <fstream>

using namespace std;

#include "base/intmath.hh"
#include "base/trace.hh"
#include "mem/physical.hh"
#include "mem/page_table.hh"
#include "sim/builder.hh"
#include "sim/sim_object.hh"

PageTable::PageTable(const std::string &name)
    : SimObject(name)
{
}

PageTable::~PageTable()
{
    //Iterate the page table freeing the memoruy
    //Addr addr;
    //std::map<Addr,Addr>::iterator iter;

    //iter = pTable.begin();
    //while(iter != pTable.end())
    //{
    //delete [] (uint8_t *)iter->second;
//	iter ++;
    //  }

}

Fault
PageTable::page_check(Addr addr, int size) const
{
    if (size < sizeof(uint64_t)) {
        if (!isPowerOf2(size)) {
            panic("Invalid request size!\n");
            return Machine_Check_Fault;
        }

        if ((size - 1) & addr)
            return Alignment_Fault;
    }
    else {
        if ((addr & (VMPageSize - 1)) + size > VMPageSize) {
            panic("Invalid request size!\n");
            return Machine_Check_Fault;
        }

        if ((sizeof(uint64_t) - 1) & addr)
            return Alignment_Fault;
    }

    return No_Fault;
}


Fault
PageTable::translate(CpuRequestPtr &req)
{
//Should I check here for accesses that are > VMPageSize?
    req->paddr = translate(req->vaddr, req->asid);
    return page_check(req->paddr, req->size);
}


Addr
PageTable::translate(Addr vaddr, unsigned asid)
{
    Addr hash_addr;
    std::map<Addr,Addr>::iterator iter;

    //DPRINTF(PageTable,"PageTable: Virtual Address %#x Translating for ASID %i\n",
    //    vaddr,asid);

    //Create the hash_addr
    //Combine vaddr and asid
    hash_addr = vaddr & (~(VMPageSize - 1)) | asid;

    //DPRINTF(PageTable,"PageTable: Hash Address %#x\n",hash_addr);

    //Look into the page table
    iter=pTable.find(hash_addr);

    //bool page_fault = true;

    //Store the translated address if found, and return
    if (iter != pTable.end()) //Found??
    {
      Addr return_addr = iter->second + (vaddr & (VMPageSize - 1));

      return return_addr;
    }
    else//Alocate a new page, register translation
    {
        Addr return_addr;

        //DPRINTF(PageTable,"PageTable: Page Not Found. Allocating new page\n");

        Addr new_page = mem->new_page();

        pTable[hash_addr] = new_page;

        return_addr = new_page + (vaddr & (VMPageSize - 1));

        return return_addr;
    }
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(PageTable)

    SimObjectParam<PhysicalMemory *> physmem;

END_DECLARE_SIM_OBJECT_PARAMS(PageTable)

BEGIN_INIT_SIM_OBJECT_PARAMS(PageTable)

    INIT_PARAM_DFLT(physmem, "Pointer to functional memory", NULL)

END_INIT_SIM_OBJECT_PARAMS(PageTable)

CREATE_SIM_OBJECT(PageTable)
{
    PageTable *pTable = new PageTable(getInstanceName());

    if (physmem)
        pTable->setPhysMem(physmem);

    return pTable;
}

REGISTER_SIM_OBJECT("PageTable", PageTable)
