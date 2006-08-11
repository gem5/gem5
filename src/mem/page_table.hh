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
 *
 * Authors: Steve Reinhardt
 */

/**
 * @file
 * Declaration of a non-full system Page Table.
 */

#ifndef __PAGE_TABLE__
#define __PAGE_TABLE__

#include <string>

#include "arch/faults.hh"
#include "arch/isa_traits.hh"
#include "base/hashmap.hh"
#include "base/trace.hh"
#include "mem/request.hh"
#include "mem/packet.hh"
#include "sim/sim_object.hh"

class System;

/**
 * Page Table Decleration.
 */
class PageTable
{
  protected:
    m5::hash_map<Addr,Addr> pTable;

    struct cacheElement {
        Addr paddr;
        Addr vaddr;
    } ;

    struct cacheElement pTableCache[3];

    const Addr pageSize;
    const Addr offsetMask;

    System *system;

  public:

    PageTable(System *_system, Addr _pageSize = TheISA::VMPageSize);

    ~PageTable();

    Addr pageAlign(Addr a)  { return (a & ~offsetMask); }
    Addr pageOffset(Addr a) { return (a &  offsetMask); }

    Fault page_check(Addr addr, int size) const;

    void allocate(Addr vaddr, int size);

    /**
     * Translate function
     * @param vaddr The virtual address.
     * @return Physical address from translation.
     */
    bool translate(Addr vaddr, Addr &paddr);

    /**
     * Perform a translation on the memory request, fills in paddr
     * field of mem_req.
     * @param req The memory request.
     */
    Fault translate(RequestPtr &req);

};

#endif
