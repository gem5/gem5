/*
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Gabe Black
 */

#ifndef __ARCH_X86_PAGETABLE_HH__
#define __ARCH_X86_PAGETABLE_HH__

#include <iostream>
#include <string>
#include <vector>

#include "base/bitunion.hh"
#include "base/types.hh"
#include "base/trie.hh"
#include "arch/x86/system.hh"
#include "debug/MMU.hh"

class Checkpoint;
class ThreadContext;

namespace X86ISA
{
    struct TlbEntry;
}

typedef Trie<Addr, X86ISA::TlbEntry> TlbEntryTrie;

namespace X86ISA
{
    BitUnion64(VAddr)
        Bitfield<20, 12> longl1;
        Bitfield<29, 21> longl2;
        Bitfield<38, 30> longl3;
        Bitfield<47, 39> longl4;

        Bitfield<20, 12> pael1;
        Bitfield<29, 21> pael2;
        Bitfield<31, 30> pael3;

        Bitfield<21, 12> norml1;
        Bitfield<31, 22> norml2;
    EndBitUnion(VAddr)

    // Unfortunately, the placement of the base field in a page table entry is
    // very erratic and would make a mess here. It might be moved here at some
    // point in the future.
    BitUnion64(PageTableEntry)
        Bitfield<63> nx;
        Bitfield<51, 12> base;
        Bitfield<11, 9> avl;
        Bitfield<8> g;
        Bitfield<7> ps;
        Bitfield<6> d;
        Bitfield<5> a;
        Bitfield<4> pcd;
        Bitfield<3> pwt;
        Bitfield<2> u;
        Bitfield<1> w;
        Bitfield<0> p;
    EndBitUnion(PageTableEntry)


    struct TlbEntry : public Serializable
    {
        // The base of the physical page.
        Addr paddr;

        // The beginning of the virtual page this entry maps.
        Addr vaddr;
        // The size of the page this represents, in address bits.
        unsigned logBytes;

        // Read permission is always available, assuming it isn't blocked by
        // other mechanisms.
        bool writable;
        // Whether this page is accesible without being in supervisor mode.
        bool user;
        // Whether to use write through or write back. M5 ignores this and
        // lets the caches handle the writeback policy.
        //bool pwt;
        // Whether the page is cacheable or not.
        bool uncacheable;
        // Whether or not to kick this page out on a write to CR3.
        bool global;
        // A bit used to form an index into the PAT table.
        bool patBit;
        // Whether or not memory on this page can be executed.
        bool noExec;
        // A sequence number to keep track of LRU.
        uint64_t lruSeq;

        TlbEntryTrie::Handle trieHandle;

        TlbEntry(Addr asn, Addr _vaddr, Addr _paddr,
                 bool uncacheable, bool read_only);
        TlbEntry();

        void
        updateVaddr(Addr new_vaddr)
        {
            vaddr = new_vaddr;
        }

        Addr pageStart()
        {
            return paddr;
        }

        // Return the page size in bytes
        int size()
        {
            return (1 << logBytes);
        }

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    /** The size of each level of the page table expressed in base 2
     * logarithmic values
     */
    const std::vector<uint8_t> PageTableLayout = {9, 9, 9, 9};

    /* x86 specific PTE flags */
    enum PTEField{
        PTE_NotPresent  = 1,
        PTE_Supervisor  = 2,
        PTE_ReadOnly    = 4,
        PTE_Uncacheable = 8,
    };

    /** Page table operations specific to x86 ISA.
     * Indended to be used as parameter of MultiLevelPageTable.
     */
    class PageTableOps
    {
      public:
        void setPTEFields(PageTableEntry& PTE, uint64_t flags = 0)
        {
            PTE.p   = flags & PTE_NotPresent  ? 0 : 1;
            PTE.pcd = flags & PTE_Uncacheable ? 1 : 0;
            PTE.w   = flags & PTE_ReadOnly    ? 0 : 1;
            PTE.u   = flags & PTE_Supervisor  ? 0 : 1;
        }

        /** returns the physical memory address of the page table */
        Addr getBasePtr(ThreadContext* tc)
        {
            CR3 cr3 = pageTablePhysAddr;
            DPRINTF(MMU, "CR3: %d\n", cr3);
            return cr3.longPdtb;
        }

        /** returns the page number out of a page table entry */
        Addr getPnum(PageTableEntry PTE)
        {
            return PTE.base;
        }

        bool isUncacheable(const PageTableEntry PTE)
        {
            return PTE.pcd;
        }

        bool isReadOnly(PageTableEntry PTE)
        {
            return !PTE.w;
        }

        /** sets the page number in a page table entry */
        void setPnum(PageTableEntry& PTE, Addr paddr)
        {
            PTE.base = paddr;
        }

        /** returns the offsets to index in every level of a page
         * table, contained in a virtual address
         */
        std::vector<uint64_t> getOffsets(Addr vaddr)
        {
            X86ISA::VAddr addr(vaddr);
            return {addr.longl1, addr.longl2, addr.longl3, addr.longl4};
        }
    };

}

#endif
