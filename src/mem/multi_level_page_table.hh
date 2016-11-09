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
 * Declaration of a multi-level page table.
 */

#ifndef __MEM_MULTI_LEVEL_PAGE_TABLE_HH__
#define __MEM_MULTI_LEVEL_PAGE_TABLE_HH__

#include <string>

#include "arch/tlb.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "mem/page_table.hh"

class System;

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
template <class ISAOps>
class MultiLevelPageTable : public PageTableBase
{
    /**
     * ISA specific operations
     */
    ISAOps pTableISAOps;

    /**
     * Pointer to System object
     */
    System *system;

    /**
     * Physical address to the last level of the page table
     */
    Addr basePtr;

    /**
     * Vector with sizes of all levels in base 2 logarithmic
     */
    const std::vector<uint8_t> logLevelSize;

    /**
     * Number of levels contained by the page table
     */
    const uint64_t numLevels;

    /**
     * Method for walking the page table
     *
     * @param vaddr Virtual address that is being looked-up
     * @param allocate Specifies whether memory should be allocated while
     *                  walking the page table
     * @return PTE_addr The address of the found PTE
     * @retval true if the page table walk has succeded, false otherwhise
     */
    bool walk(Addr vaddr, bool allocate, Addr &PTE_addr);

public:
    MultiLevelPageTable(const std::string &__name, uint64_t _pid,
                        System *_sys);
    ~MultiLevelPageTable();

    void initState(ThreadContext* tc) override;

    void map(Addr vaddr, Addr paddr, int64_t size,
             uint64_t flags = 0) override;
    void remap(Addr vaddr, int64_t size, Addr new_vaddr) override;
    void unmap(Addr vaddr, int64_t size) override;
    bool isUnmapped(Addr vaddr, int64_t size) override;
    bool lookup(Addr vaddr, TheISA::TlbEntry &entry) override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};
#endif // __MEM_MULTI_LEVEL_PAGE_TABLE_HH__
