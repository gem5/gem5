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
 */

/**
 * @file
 * Declaration of a multi-level page table.
 */

#ifndef __MEM_MULTI_LEVEL_PAGE_TABLE_HH__
#define __MEM_MULTI_LEVEL_PAGE_TABLE_HH__

#include <string>

#include "base/types.hh"
#include "mem/page_table.hh"
#include "sim/system.hh"

/**
 * This class implements an in-memory multi-level page table that can be
 * configured to follow ISA specifications. It can be used instead of the
 * PageTable class in SE mode to allow CPU models (e.g. X86KvmCPU)
 * to do a normal page table walk.
 *
 * To reduce memory required to store the page table, a multi-level page
 * table stores its translations similarly with a radix tree. Let n be
 * the number of levels and {Ln, Ln-1, ..., L1, L0} a set that specifies
 * the number of entries for each level as base 2 logarithm values. A
 * multi-level page table will store its translations at level 0 (the
 * leaves of the tree) and it will be layed out in memory in the
 * following way:
 *
 *                              +------------------------------+
 * level n                      |Ln-1_E0|Ln-1_E1|...|Ln-1_E2^Ln|
 *                              +------------------------------+
 *                                 /       \
 *            +------------------------+   +------------------------+
 * level n-1  |Ln-2_E0|...|Ln-2_E2^Ln-1|   |Ln-2_E0|...|Ln-2_E2^Ln-1|
 *            +------------------------+   +------------------------+
 *                /            \             /              \
 *                                  .
 *                                  .
 *                                  .
 *               /                      /               \
 *          +------------------+   +------------+     +------------+
 * level 1  |L0_E1|...|L0_E2^L1|   |...|L0_E2^L1| ... |...|L0_E2^L1|
 *          +------------------+   +------------+     +------------+
 * , where
 * +------------------------------+
 * |Lk-1_E0|Lk-1_E1|...|Lk-1_E2^Lk|
 * +------------------------------+
 * is a level k entry that holds 2^Lk entries in Lk-1 level.
 *
 * Essentially, a level n entry will contain 2^Ln level n-1 entries,
 * a level n-1 entry will hold 2^Ln-1 level n-2 entries etc.
 *
 * The virtual address is split into offsets that index into the
 * different levels of the page table.
 *
 * +--------------------------------+
 * |LnOffset|...|L1Offset|PageOffset|
 * +--------------------------------+
 *
 * For example L0Offset will be formed by the bits in range
 * [log2(PageOffset), log2(PageOffset)+L0].
 *
 * For every level of the page table, from n to 1, the base address
 * of the entry is loaded, the offset in the virtual address for
 * that particular level is used to index into the entry which
 * will reveal the memory address of the entry in the next level.
 *
 * @see MultiLevelPageTable
 */

namespace {

template <class First, class ...Rest>
Addr
prepTopTable(System *system, Addr pageSize)
{
    Addr addr = system->allocPhysPages(First::tableSize());
    PortProxy &p = system->physProxy;
    p.memsetBlob(addr, 0, First::tableSize() * pageSize);
    return addr;
}

template <class ...Types>
struct LastType;

template <class First, class Second, class ...Rest>
struct LastType<First, Second, Rest...>
{
    typedef typename LastType<Second, Rest...>::type type;
};

template <class Only>
struct LastType<Only>
{
    typedef Only type;
};


template <class ...Types>
struct WalkWrapper;

template <class Final, class Only>
struct WalkWrapper<Final, Only>
{
    static void
    walk(System *system, Addr pageSize, Addr table, Addr vaddr,
         bool allocate, Final *entry)
    {
        entry->read(system->physProxy, table, vaddr);
    }
};

template <class Final, class First, class Second, class ...Rest>
struct WalkWrapper<Final, First, Second, Rest...>
{
    static void
    walk(System *system, Addr pageSize, Addr table, Addr vaddr,
         bool allocate, Final *entry)
    {
        First first;
        first.read(system->physProxy, table, vaddr);

        Addr next;
        if (!first.present()) {
            fatal_if(!allocate,
                     "Page fault while walking the page table.");
            next = prepTopTable<Second>(system, pageSize);
            first.reset(next);
            first.write(system->physProxy);
        } else {
            next = first.paddr();
        }
        WalkWrapper<Final, Second, Rest...>::walk(
                system, pageSize, next, vaddr, allocate, entry);
    }
};

template <class ...EntryTypes>
void
walk(System *system, Addr pageSize, Addr table, Addr vaddr,
     bool allocate, typename LastType<EntryTypes...>::type *entry)
{
    WalkWrapper<typename LastType<EntryTypes...>::type, EntryTypes...>::walk(
            system, pageSize, table, vaddr, allocate, entry);
}

}


template <class ...EntryTypes>
class MultiLevelPageTable : public EmulationPageTable
{
    typedef typename LastType<EntryTypes...>::type Final;

    /**
     * Pointer to System object
     */
    System *system;

    /**
     * Physical address to the last level of the page table
     */
    Addr _basePtr;

public:
    MultiLevelPageTable(const std::string &__name, uint64_t _pid,
                        System *_sys, Addr pageSize) :
            EmulationPageTable(__name, _pid, pageSize), system(_sys)
    {}

    ~MultiLevelPageTable() {}

    void
    initState() override
    {
        if (shared)
            return;

        _basePtr = prepTopTable<EntryTypes...>(system, pageSize);
    }

    Addr basePtr() { return _basePtr; }

    void
    map(Addr vaddr, Addr paddr, int64_t size, uint64_t flags = 0) override
    {
        EmulationPageTable::map(vaddr, paddr, size, flags);

        Final entry;

        for (int64_t offset = 0; offset < size; offset += pageSize) {
            walk<EntryTypes...>(system, pageSize, _basePtr,
                                vaddr + offset, true, &entry);

            entry.reset(paddr + offset, true, flags & Uncacheable,
                        flags & ReadOnly);
            entry.write(system->physProxy);

            DPRINTF(MMU, "New mapping: %#x-%#x\n",
                    vaddr + offset, paddr + offset);
        }
    }

    void
    remap(Addr vaddr, int64_t size, Addr new_vaddr) override
    {
        EmulationPageTable::remap(vaddr, size, new_vaddr);

        Final old_entry, new_entry;

        for (int64_t offset = 0; offset < size; offset += pageSize) {
            // Unmap the original mapping.
            walk<EntryTypes...>(system, pageSize, _basePtr, vaddr + offset,
                                false, &old_entry);
            old_entry.present(false);
            old_entry.write(system->physProxy);

            // Map the new one.
            walk<EntryTypes...>(system, pageSize, _basePtr, new_vaddr + offset,
                                true, &new_entry);
            new_entry.reset(old_entry.paddr(), true, old_entry.uncacheable(),
                            old_entry.readonly());
            new_entry.write(system->physProxy);
        }
    }

    void
    unmap(Addr vaddr, int64_t size) override
    {
        EmulationPageTable::unmap(vaddr, size);

        Final entry;

        for (int64_t offset = 0; offset < size; offset += pageSize) {
            walk<EntryTypes...>(system, pageSize, _basePtr,
                                vaddr + offset, false, &entry);
            fatal_if(!entry.present(),
                     "PageTable::unmap: Address %#x not mapped.", vaddr);
            entry.present(false);
            entry.write(system->physProxy);
            DPRINTF(MMU, "Unmapping: %#x\n", vaddr);
        }
    }

    void
    serialize(CheckpointOut &cp) const override
    {
        EmulationPageTable::serialize(cp);
        /** Since, the page table is stored in system memory
         * which is serialized separately, we will serialize
         * just the base pointer
         */
        paramOut(cp, "ptable.pointer", _basePtr);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        EmulationPageTable::unserialize(cp);
        paramIn(cp, "ptable.pointer", _basePtr);
    }
};
#endif // __MEM_MULTI_LEVEL_PAGE_TABLE_HH__
