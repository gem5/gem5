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

EmulationPageTable::~EmulationPageTable()
{
    for (auto &iter : pTable)
        delete iter.second;
}

void
EmulationPageTable::map(Addr vaddr, Addr paddr, int64_t size, uint64_t flags)
{
    bool clobber = flags & Clobber;
    // starting address must be page aligned
    assert(pageOffset(vaddr) == 0);

    DPRINTF(MMU, "Allocating Page: %#x-%#x\n", vaddr, vaddr + size);

    while (size > 0) {
        auto it = pTable.find(vaddr);
        if (it != pTable.end()) {
            if (clobber) {
                delete it->second;
            } else {
                // already mapped
                panic("EmulationPageTable::allocate: addr %#x already mapped",
                      vaddr);
            }
        } else {
            it = pTable.emplace(vaddr, nullptr).first;
        }

        it->second = new TheISA::TlbEntry(pid, vaddr, paddr,
                                         flags & Uncacheable,
                                         flags & ReadOnly);
        size -= pageSize;
        vaddr += pageSize;
        paddr += pageSize;
    }
}

void
EmulationPageTable::remap(Addr vaddr, int64_t size, Addr new_vaddr)
{
    assert(pageOffset(vaddr) == 0);
    assert(pageOffset(new_vaddr) == 0);

    DPRINTF(MMU, "moving pages from vaddr %08p to %08p, size = %d\n", vaddr,
            new_vaddr, size);

    while (size > 0) {
        auto new_it = pTable.find(new_vaddr);
        auto old_it = pTable.find(vaddr);
        assert(old_it != pTable.end() && new_it == pTable.end());

        new_it->second = old_it->second;
        pTable.erase(old_it);
        new_it->second->updateVaddr(new_vaddr);
        size -= pageSize;
        vaddr += pageSize;
        new_vaddr += pageSize;
    }
}

void
EmulationPageTable::getMappings(std::vector<std::pair<Addr, Addr>> *addr_maps)
{
    for (auto &iter : pTable)
        addr_maps->push_back(make_pair(iter.first, iter.second->pageStart()));
}

void
EmulationPageTable::unmap(Addr vaddr, int64_t size)
{
    assert(pageOffset(vaddr) == 0);

    DPRINTF(MMU, "Unmapping page: %#x-%#x\n", vaddr, vaddr+ size);

    while (size > 0) {
        auto it = pTable.find(vaddr);
        assert(it != pTable.end());
        delete it->second;
        pTable.erase(it);
        size -= pageSize;
        vaddr += pageSize;
    }
}

bool
EmulationPageTable::isUnmapped(Addr vaddr, int64_t size)
{
    // starting address must be page aligned
    assert(pageOffset(vaddr) == 0);

    for (int64_t offset = 0; offset < size; offset += pageSize)
        if (pTable.find(vaddr + offset) != pTable.end())
            return false;

    return true;
}

TheISA::TlbEntry *
EmulationPageTable::lookup(Addr vaddr)
{
    Addr page_addr = pageAlign(vaddr);
    PTableItr iter = pTable.find(page_addr);
    if (iter == pTable.end())
        return nullptr;
    return iter->second;
}

bool
EmulationPageTable::translate(Addr vaddr, Addr &paddr)
{
    TheISA::TlbEntry *entry = lookup(vaddr);
    if (!entry) {
        DPRINTF(MMU, "Couldn't Translate: %#x\n", vaddr);
        return false;
    }
    paddr = pageOffset(vaddr) + entry->pageStart();
    DPRINTF(MMU, "Translating: %#x->%#x\n", vaddr, paddr);
    return true;
}

Fault
EmulationPageTable::translate(RequestPtr req)
{
    Addr paddr;
    assert(pageAlign(req->getVaddr() + req->getSize() - 1) ==
           pageAlign(req->getVaddr()));
    if (!translate(req->getVaddr(), paddr))
        return Fault(new GenericPageTableFault(req->getVaddr()));
    req->setPaddr(paddr);
    if ((paddr & (pageSize - 1)) + req->getSize() > pageSize) {
        panic("Request spans page boundaries!\n");
        return NoFault;
    }
    return NoFault;
}

void
EmulationPageTable::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "ptable.size", pTable.size());

    PTable::size_type count = 0;
    for (auto &pte : pTable) {
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", count++));

        paramOut(cp, "vaddr", pte.first);
        pte.second->serialize(cp);
    }
    assert(count == pTable.size());
}

void
EmulationPageTable::unserialize(CheckpointIn &cp)
{
    int count;
    paramIn(cp, "ptable.size", count);

    for (int i = 0; i < count; ++i) {
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", i));

        TheISA::TlbEntry *entry = new TheISA::TlbEntry();
        entry->unserialize(cp);

        Addr vaddr;
        paramIn(cp, "vaddr", vaddr);

        pTable[vaddr] = entry;
    }
}

