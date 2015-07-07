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
#include <fstream>
#include <map>
#include <string>

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/MMU.hh"
#include "mem/multi_level_page_table.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

using namespace std;
using namespace TheISA;

template <class ISAOps>
MultiLevelPageTable<ISAOps>::MultiLevelPageTable(const std::string &__name,
                                                 uint64_t _pid, System *_sys)
    : PageTableBase(__name, _pid), system(_sys),
    logLevelSize(PageTableLayout),
    numLevels(logLevelSize.size())
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
    basePtr = pTableISAOps.getBasePtr(tc);
    if (basePtr == 0) basePtr++;
    DPRINTF(MMU, "basePtr: %d\n", basePtr);

    system->pagePtr = basePtr;

    /* setting first level of the page table */
    uint64_t log_req_size = floorLog2(sizeof(PageTableEntry)) +
                            logLevelSize[numLevels-1];
    assert(log_req_size >= PageShift);
    uint64_t npages = 1 << (log_req_size - PageShift);

    Addr paddr = system->allocPhysPages(npages);

    PortProxy &p = system->physProxy;
    p.memsetBlob(paddr, 0, npages << PageShift);
}


template <class ISAOps>
bool
MultiLevelPageTable<ISAOps>::walk(Addr vaddr, bool allocate, Addr &PTE_addr)
{
    std::vector<uint64_t> offsets = pTableISAOps.getOffsets(vaddr);

    Addr level_base = basePtr;
    for (int i = numLevels - 1; i > 0; i--) {

        Addr entry_addr = (level_base<<PageShift) +
                          offsets[i] * sizeof(PageTableEntry);

        PortProxy &p = system->physProxy;
        PageTableEntry entry = p.read<PageTableEntry>(entry_addr);

        Addr next_entry_pnum = pTableISAOps.getPnum(entry);
        if (next_entry_pnum == 0) {

            if (!allocate) return false;

            uint64_t log_req_size = floorLog2(sizeof(PageTableEntry)) +
                                    logLevelSize[i-1];
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
    PTE_addr = (level_base<<PageShift) +
        offsets[0] * sizeof(PageTableEntry);
    DPRINTF(MMU, "Returning PTE_addr: %x\n", PTE_addr);
    return true;
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::map(Addr vaddr, Addr paddr,
                                 int64_t size, uint64_t flags)
{
    bool clobber = flags & Clobber;
    // starting address must be page aligned
    assert(pageOffset(vaddr) == 0);

    DPRINTF(MMU, "Allocating Page: %#x-%#x\n", vaddr, vaddr + size);

    PortProxy &p = system->physProxy;

    for (; size > 0; size -= pageSize, vaddr += pageSize, paddr += pageSize) {
        Addr PTE_addr;
        if (walk(vaddr, true, PTE_addr)) {
            PageTableEntry PTE = p.read<PageTableEntry>(PTE_addr);
            Addr entry_paddr = pTableISAOps.getPnum(PTE);
            if (!clobber && entry_paddr != 0) {
                fatal("addr 0x%x already mapped to %x", vaddr, entry_paddr);
            }
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

            eraseCacheEntry(vaddr);
            updateCache(vaddr, TlbEntry(pid, vaddr, paddr,
                                        flags & Uncacheable,
                                        flags & ReadOnly));
        }

    }
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::remap(Addr vaddr, int64_t size, Addr new_vaddr)
{
    assert(pageOffset(vaddr) == 0);
    assert(pageOffset(new_vaddr) == 0);

    DPRINTF(MMU, "moving pages from vaddr %08p to %08p, size = %d\n", vaddr,
            new_vaddr, size);

    PortProxy &p = system->physProxy;

    for (; size > 0;
         size -= pageSize, vaddr += pageSize, new_vaddr += pageSize)
    {
        Addr PTE_addr;
        if (walk(vaddr, false, PTE_addr)) {
            PageTableEntry PTE = p.read<PageTableEntry>(PTE_addr);
            Addr paddr = pTableISAOps.getPnum(PTE);

            if (paddr == 0) {
                fatal("Page fault while remapping");
            } else {
                /* unmapping vaddr */
                pTableISAOps.setPnum(PTE, 0);
                p.write<PageTableEntry>(PTE_addr, PTE);

                /* maping new_vaddr */
                Addr new_PTE_addr;
                walk(new_vaddr, true, new_PTE_addr);
                PageTableEntry new_PTE = p.read<PageTableEntry>(new_PTE_addr);

                pTableISAOps.setPnum(new_PTE, paddr>>PageShift);
                pTableISAOps.setPTEFields(new_PTE);
                p.write<PageTableEntry>(new_PTE_addr, new_PTE);
                DPRINTF(MMU, "Remapping: %#x-%#x\n", vaddr, new_PTE_addr);
            }

            eraseCacheEntry(vaddr);
            updateCache(new_vaddr, TlbEntry(pid, new_vaddr, paddr,
                                            pTableISAOps.isUncacheable(PTE),
                                            pTableISAOps.isReadOnly(PTE)));
        } else {
            fatal("Page fault while remapping");
        }
    }
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::unmap(Addr vaddr, int64_t size)
{
    assert(pageOffset(vaddr) == 0);

    DPRINTF(MMU, "Unmapping page: %#x-%#x\n", vaddr, vaddr+ size);

    PortProxy &p = system->physProxy;

    for (; size > 0; size -= pageSize, vaddr += pageSize) {
        Addr PTE_addr;
        if (walk(vaddr, false, PTE_addr)) {
            PageTableEntry PTE = p.read<PageTableEntry>(PTE_addr);
            Addr paddr = pTableISAOps.getPnum(PTE);
            if (paddr == 0) {
                fatal("PageTable::allocate: address 0x%x not mapped", vaddr);
            } else {
                pTableISAOps.setPnum(PTE, 0);
                p.write<PageTableEntry>(PTE_addr, PTE);
                DPRINTF(MMU, "Unmapping: %#x\n", vaddr);
            }
           eraseCacheEntry(vaddr);
        } else {
            fatal("Page fault while unmapping");
        }
    }

}

template <class ISAOps>
bool
MultiLevelPageTable<ISAOps>::isUnmapped(Addr vaddr, int64_t size)
{
    // starting address must be page aligned
    assert(pageOffset(vaddr) == 0);
    PortProxy &p = system->physProxy;

    for (; size > 0; size -= pageSize, vaddr += pageSize) {
        Addr PTE_addr;
        if (walk(vaddr, false, PTE_addr)) {
            PageTableEntry PTE = p.read<PageTableEntry>(PTE_addr);
            if (pTableISAOps.getPnum(PTE) != 0)
                return false;
        }
    }

    return true;
}

template <class ISAOps>
bool
MultiLevelPageTable<ISAOps>::lookup(Addr vaddr, TlbEntry &entry)
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

    DPRINTF(MMU, "lookup page_addr: %#x\n", page_addr);
    Addr PTE_addr;
    if (walk(page_addr, false, PTE_addr)) {
        PortProxy &p = system->physProxy;
        PageTableEntry PTE = p.read<PageTableEntry>(PTE_addr);
        Addr pnum = pTableISAOps.getPnum(PTE);
        if (pnum == 0)
            return false;

        entry = TlbEntry(pid, vaddr, pnum << PageShift,
                         pTableISAOps.isUncacheable(PTE),
                         pTableISAOps.isReadOnly(PTE));
        updateCache(page_addr, entry);
    } else {
        return false;
    }
    return true;
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::serialize(CheckpointOut &cp) const
{
    /** Since, the page table is stored in system memory
     * which is serialized separately, we will serialize
     * just the base pointer
     */
    paramOut(cp, "ptable.pointer", basePtr);
}

template <class ISAOps>
void
MultiLevelPageTable<ISAOps>::unserialize(CheckpointIn &cp)
{
    paramIn(cp, "ptable.pointer", basePtr);
}
