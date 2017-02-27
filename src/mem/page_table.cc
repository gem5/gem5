/*
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
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
 *          Ron Dreslinski
 *          Ali Saidi
 */

/**
 * @file
 * Definitions of functional page table.
 */
#include "mem/page_table.hh"

#include <string>

#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/MMU.hh"
#include "sim/faults.hh"
#include "sim/serialize.hh"

using namespace std;
using namespace TheISA;

FuncPageTable::FuncPageTable(const std::string &__name,
                             uint64_t _pid, Addr _pageSize)
        : PageTableBase(__name, _pid, _pageSize)
{
}

FuncPageTable::~FuncPageTable()
{
}

void
FuncPageTable::map(Addr vaddr, Addr paddr, int64_t size, uint64_t flags)
{
    bool clobber = flags & Clobber;
    // starting address must be page aligned
    assert(pageOffset(vaddr) == 0);

    DPRINTF(MMU, "Allocating Page: %#x-%#x\n", vaddr, vaddr+ size);

    for (; size > 0; size -= pageSize, vaddr += pageSize, paddr += pageSize) {
        if (!clobber && (pTable.find(vaddr) != pTable.end())) {
            // already mapped
            fatal("FuncPageTable::allocate: addr 0x%x already mapped", vaddr);
        }

        pTable[vaddr] = TheISA::TlbEntry(pid, vaddr, paddr,
                                         flags & Uncacheable,
                                         flags & ReadOnly);
        eraseCacheEntry(vaddr);
        updateCache(vaddr, pTable[vaddr]);
    }
}

void
FuncPageTable::remap(Addr vaddr, int64_t size, Addr new_vaddr)
{
    assert(pageOffset(vaddr) == 0);
    assert(pageOffset(new_vaddr) == 0);

    DPRINTF(MMU, "moving pages from vaddr %08p to %08p, size = %d\n", vaddr,
            new_vaddr, size);

    for (; size > 0;
         size -= pageSize, vaddr += pageSize, new_vaddr += pageSize)
    {
        assert(pTable.find(vaddr) != pTable.end());

        pTable[new_vaddr] = pTable[vaddr];
        pTable.erase(vaddr);
        eraseCacheEntry(vaddr);
        pTable[new_vaddr].updateVaddr(new_vaddr);
        updateCache(new_vaddr, pTable[new_vaddr]);
    }
}

void
FuncPageTable::getMappings(std::vector<std::pair<Addr, Addr>> *addr_maps)
{
    for (auto &iter : pTable)
        addr_maps->push_back(make_pair(iter.first, iter.second.pageStart()));
}

void
FuncPageTable::unmap(Addr vaddr, int64_t size)
{
    assert(pageOffset(vaddr) == 0);

    DPRINTF(MMU, "Unmapping page: %#x-%#x\n", vaddr, vaddr+ size);

    for (; size > 0; size -= pageSize, vaddr += pageSize) {
        assert(pTable.find(vaddr) != pTable.end());
        pTable.erase(vaddr);
        eraseCacheEntry(vaddr);
    }

}

bool
FuncPageTable::isUnmapped(Addr vaddr, int64_t size)
{
    // starting address must be page aligned
    assert(pageOffset(vaddr) == 0);

    for (; size > 0; size -= pageSize, vaddr += pageSize) {
        if (pTable.find(vaddr) != pTable.end()) {
            return false;
        }
    }

    return true;
}

bool
FuncPageTable::lookup(Addr vaddr, TheISA::TlbEntry &entry)
{
    Addr page_addr = pageAlign(vaddr);

    if (pTableCache[0].valid && pTableCache[0].vaddr == page_addr) {
        entry = pTableCache[0].entry;
        return true;
    }
    if (pTableCache[1].valid && pTableCache[1].vaddr == page_addr) {
        entry = pTableCache[1].entry;
        return true;
    }
    if (pTableCache[2].valid && pTableCache[2].vaddr == page_addr) {
        entry = pTableCache[2].entry;
        return true;
    }

    PTableItr iter = pTable.find(page_addr);

    if (iter == pTable.end()) {
        return false;
    }

    updateCache(page_addr, iter->second);
    entry = iter->second;
    return true;
}

bool
PageTableBase::translate(Addr vaddr, Addr &paddr)
{
    TheISA::TlbEntry entry;
    if (!lookup(vaddr, entry)) {
        DPRINTF(MMU, "Couldn't Translate: %#x\n", vaddr);
        return false;
    }
    paddr = pageOffset(vaddr) + entry.pageStart();
    DPRINTF(MMU, "Translating: %#x->%#x\n", vaddr, paddr);
    return true;
}

Fault
PageTableBase::translate(RequestPtr req)
{
    Addr paddr;
    assert(pageAlign(req->getVaddr() + req->getSize() - 1)
           == pageAlign(req->getVaddr()));
    if (!translate(req->getVaddr(), paddr)) {
        return Fault(new GenericPageTableFault(req->getVaddr()));
    }
    req->setPaddr(paddr);
    if ((paddr & (pageSize - 1)) + req->getSize() > pageSize) {
        panic("Request spans page boundaries!\n");
        return NoFault;
    }
    return NoFault;
}

void
FuncPageTable::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "ptable.size", pTable.size());

    PTable::size_type count = 0;
    for (auto &pte : pTable) {
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", count++));

        paramOut(cp, "vaddr", pte.first);
        pte.second.serialize(cp);
    }
    assert(count == pTable.size());
}

void
FuncPageTable::unserialize(CheckpointIn &cp)
{
    int count;
    paramIn(cp, "ptable.size", count);

    for (int i = 0; i < count; ++i) {
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", i));

        std::unique_ptr<TheISA::TlbEntry> entry;
        Addr vaddr;

        paramIn(cp, "vaddr", vaddr);
        entry.reset(new TheISA::TlbEntry());
        entry->unserialize(cp);

        pTable[vaddr] = *entry;
    }
}

