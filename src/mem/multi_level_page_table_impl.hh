/*
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
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
 * Authors: Alexandru Dutu
 */

/**
 * @file
 * Definitions of page table
 */
#include <string>

#include "arch/isa_traits.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/MMU.hh"
#include "mem/multi_level_page_table.hh"
#include "mem/page_table.hh"

using namespace std;
using namespace TheISA;

template <class ISAOps>
MultiLevelPageTable<ISAOps>::MultiLevelPageTable(
        const std::string &__name, uint64_t _pid, System *_sys,
        Addr pageSize, const std::vector<uint8_t> &layout)
    : EmulationPageTable(__name, _pid, pageSize), system(_sys),
      logLevelSize(layout), numLevels(logLevelSize.size())
{
}

template <class ISAOps>
MultiLevelPageTable<ISAOps>::~MultiLevelPageTable()
{
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::initState(ThreadContext* tc)
{
    /* setting first level of the page table */
    uint64_t log_req_size = floorLog2(sizeof(PageTableEntry)) +
                            logLevelSize[numLevels - 1];
    assert(log_req_size >= PageShift);
    uint64_t npages = 1 << (log_req_size - PageShift);

    Addr _basePtr = system->allocPhysPages(npages);

    PortProxy &p = system->physProxy;
    p.memsetBlob(_basePtr, 0, npages << PageShift);
}


template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::walk(Addr vaddr, bool allocate, Addr &PTE_addr)
{
    std::vector<uint64_t> offsets = pTableISAOps.getOffsets(vaddr);

    Addr level_base = _basePtr;
    for (int i = numLevels - 1; i > 0; i--) {

        Addr entry_addr = (level_base<<PageShift) +
                          offsets[i] * sizeof(PageTableEntry);

        PortProxy &p = system->physProxy;
        PageTableEntry entry = p.read<PageTableEntry>(entry_addr);

        Addr next_entry_pnum = pTableISAOps.getPnum(entry);
        if (next_entry_pnum == 0) {

            fatal_if(!allocate, "Page fault while walking the page table.");

            uint64_t log_req_size = floorLog2(sizeof(PageTableEntry)) +
                                    logLevelSize[i - 1];
            assert(log_req_size >= PageShift);
            uint64_t npages = 1 << (log_req_size - PageShift);

            DPRINTF(MMU, "Allocating %d pages needed for entry in level %d\n",
                    npages, i - 1);

            /* allocate new entry */
            Addr next_entry_paddr = system->allocPhysPages(npages);
            p.memsetBlob(next_entry_paddr, 0, npages << PageShift);

            next_entry_pnum = next_entry_paddr >> PageShift;
            pTableISAOps.setPnum(entry, next_entry_pnum);
            pTableISAOps.setPTEFields(entry);
            p.write<PageTableEntry>(entry_addr, entry);

        }
        DPRINTF(MMU, "Level %d base: %d offset: %d entry: %d\n",
                i, level_base, offsets[i], next_entry_pnum);
        level_base = next_entry_pnum;

    }
    PTE_addr = (level_base << PageShift) +
        offsets[0] * sizeof(PageTableEntry);
    DPRINTF(MMU, "Returning PTE_addr: %x\n", PTE_addr);
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::map(Addr vaddr, Addr paddr,
                                 int64_t size, uint64_t flags)
{
    EmulationPageTable::map(vaddr, paddr, size, flags);

    PortProxy &p = system->physProxy;

    while (size > 0) {
        Addr PTE_addr;
        walk(vaddr, true, PTE_addr);
        PageTableEntry PTE = p.read<PageTableEntry>(PTE_addr);
        pTableISAOps.setPnum(PTE, paddr >> PageShift);
        uint64_t PTE_flags = 0;
        if (flags & NotPresent)
            PTE_flags |= TheISA::PTE_NotPresent;
        if (flags & Uncacheable)
            PTE_flags |= TheISA::PTE_Uncacheable;
        if (flags & ReadOnly)
            PTE_flags |= TheISA::PTE_ReadOnly;
        pTableISAOps.setPTEFields(PTE, PTE_flags);
        p.write<PageTableEntry>(PTE_addr, PTE);
        DPRINTF(MMU, "New mapping: %#x-%#x\n", vaddr, paddr);
        size -= pageSize;
        vaddr += pageSize;
        paddr += pageSize;
    }
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::remap(Addr vaddr, int64_t size, Addr new_vaddr)
{
    EmulationPageTable::remap(vaddr, size, new_vaddr);

    PortProxy &p = system->physProxy;

    while (size > 0) {
        Addr PTE_addr;
        walk(vaddr, false, PTE_addr);
        PageTableEntry PTE = p.read<PageTableEntry>(PTE_addr);
        Addr paddr = pTableISAOps.getPnum(PTE);

        fatal_if(paddr == 0, "Page fault while remapping");
        /* unmapping vaddr */
        pTableISAOps.setPnum(PTE, 0);
        p.write<PageTableEntry>(PTE_addr, PTE);

        /* maping new_vaddr */
        Addr new_PTE_addr;
        walk(new_vaddr, true, new_PTE_addr);
        PageTableEntry new_PTE = p.read<PageTableEntry>(new_PTE_addr);

        pTableISAOps.setPnum(new_PTE, paddr >> PageShift);
        pTableISAOps.setPTEFields(new_PTE);
        p.write<PageTableEntry>(new_PTE_addr, new_PTE);
        DPRINTF(MMU, "Remapping: %#x-%#x\n", vaddr, new_PTE_addr);
        size -= pageSize;
        vaddr += pageSize;
        new_vaddr += pageSize;
    }
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::unmap(Addr vaddr, int64_t size)
{
    EmulationPageTable::unmap(vaddr, size);

    PortProxy &p = system->physProxy;

    while (size > 0) {
        Addr PTE_addr;
        walk(vaddr, false, PTE_addr);
        PageTableEntry PTE = p.read<PageTableEntry>(PTE_addr);
        Addr paddr = pTableISAOps.getPnum(PTE);
        fatal_if(paddr == 0,
                 "PageTable::allocate: address %#x not mapped", vaddr);
        pTableISAOps.setPnum(PTE, 0);
        p.write<PageTableEntry>(PTE_addr, PTE);
        DPRINTF(MMU, "Unmapping: %#x\n", vaddr);
        size -= pageSize;
        vaddr += pageSize;
    }
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::serialize(CheckpointOut &cp) const
{
    EmulationPageTable::serialize(cp);
    /** Since, the page table is stored in system memory
     * which is serialized separately, we will serialize
     * just the base pointer
     */
    paramOut(cp, "ptable.pointer", _basePtr);
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::unserialize(CheckpointIn &cp)
{
    EmulationPageTable::unserialize(cp);
    paramIn(cp, "ptable.pointer", _basePtr);
}
