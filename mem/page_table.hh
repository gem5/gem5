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
 * Declaration of a non-full system Page Table.
 */

#ifndef __PAGE_TABLE__
#define __PAGE_TABLE__

#include <string>
#include <map>

#include "base/trace.hh"
#include "mem/request.hh"
#include "mem/packet.hh"
#include "sim/sim_object.hh"

class PhysicalMemory;

/**
 * Page Table Decleration.
 */
class PageTable : public SimObject
{
  protected:
    std::map<Addr,Addr> pTable;

    PhysicalMemory *mem;

  public:

    /**
     * Construct this interface.
     * @param name The name of this interface.
     * @param hier Pointer to the hierarchy wide parameters.
     * @param _mem the connected memory.
     */
    PageTable(const std::string &name);

    ~PageTable();

    void setPhysMem(PhysicalMemory *_mem) { mem = _mem; }

    Fault page_check(Addr addr, int size) const;

    /**
     * Translate function
     * @param vaddr The virtual address.
     * @param asid The address space id.
     * @return Physical address from translation.
     */
    Addr translate(Addr vaddr, unsigned asid);

    /**
     * Perform a translation on the memory request, fills in paddr field of mem_req.
     * @param req The memory request.
     */
    Fault translate(CpuRequestPtr &req);

};

#endif
