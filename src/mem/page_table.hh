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
 */

/**
 * @file
 * Declarations of a non-full system Page Table.
 */

#ifndef __MEM_PAGE_TABLE_HH__
#define __MEM_PAGE_TABLE_HH__

#include <string>
#include <unordered_map>

#include "arch/isa_traits.hh"
#include "arch/tlb.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "mem/request.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

class ThreadContext;

/**
 * Declaration of base class for page table
 */
class PageTableBase : public Serializable
{
  protected:
    struct cacheElement {
        bool valid;
        Addr vaddr;
        TheISA::TlbEntry entry;
    };

    struct cacheElement pTableCache[3];

    const Addr pageSize;
    const Addr offsetMask;

    const uint64_t pid;
    const std::string _name;

  public:

    PageTableBase(const std::string &__name, uint64_t _pid,
              Addr _pageSize = TheISA::PageBytes)
            : pageSize(_pageSize), offsetMask(mask(floorLog2(_pageSize))),
              pid(_pid), _name(__name)
    {
        assert(isPowerOf2(pageSize));
        pTableCache[0].valid = false;
        pTableCache[1].valid = false;
        pTableCache[2].valid = false;
    }

    virtual ~PageTableBase() {};

    /* generic page table mapping flags
     *              unset | set
     * bit 0 - no-clobber | clobber
     * bit 1 - present    | not-present
     * bit 2 - cacheable  | uncacheable
     * bit 3 - read-write | read-only
     */
    enum MappingFlags : uint32_t {
        Clobber     = 1,
        NotPresent  = 2,
        Uncacheable = 4,
        ReadOnly    = 8,
    };

    virtual void initState(ThreadContext* tc) = 0;

    // for DPRINTF compatibility
    const std::string name() const { return _name; }

    Addr pageAlign(Addr a)  { return (a & ~offsetMask); }
    Addr pageOffset(Addr a) { return (a &  offsetMask); }

    /**
     * Maps a virtual memory region to a physical memory region.
     * @param vaddr The starting virtual address of the region.
     * @param paddr The starting physical address where the region is mapped.
     * @param size The length of the region.
     * @param flags Generic mapping flags that can be set by or-ing values
     *              from MappingFlags enum.
     */
    virtual void map(Addr vaddr, Addr paddr, int64_t size,
                     uint64_t flags = 0) = 0;
    virtual void remap(Addr vaddr, int64_t size, Addr new_vaddr) = 0;
    virtual void unmap(Addr vaddr, int64_t size) = 0;

    /**
     * Check if any pages in a region are already allocated
     * @param vaddr The starting virtual address of the region.
     * @param size The length of the region.
     * @return True if no pages in the region are mapped.
     */
    virtual bool isUnmapped(Addr vaddr, int64_t size) = 0;

    /**
     * Lookup function
     * @param vaddr The virtual address.
     * @return entry The page table entry corresponding to vaddr.
     */
    virtual bool lookup(Addr vaddr, TheISA::TlbEntry &entry) = 0;

    /**
     * Translate function
     * @param vaddr The virtual address.
     * @param paddr Physical address from translation.
     * @return True if translation exists
     */
    bool translate(Addr vaddr, Addr &paddr);

    /**
     * Simplified translate function (just check for translation)
     * @param vaddr The virtual address.
     * @return True if translation exists
     */
    bool translate(Addr vaddr) { Addr dummy; return translate(vaddr, dummy); }

    /**
     * Perform a translation on the memory request, fills in paddr
     * field of req.
     * @param req The memory request.
     */
    Fault translate(RequestPtr req);

    /**
     * Update the page table cache.
     * @param vaddr virtual address (page aligned) to check
     * @param pte page table entry to return
     */
    inline void updateCache(Addr vaddr, TheISA::TlbEntry entry)
    {
        pTableCache[2].entry = pTableCache[1].entry;
        pTableCache[2].vaddr = pTableCache[1].vaddr;
        pTableCache[2].valid = pTableCache[1].valid;

        pTableCache[1].entry = pTableCache[0].entry;
        pTableCache[1].vaddr = pTableCache[0].vaddr;
        pTableCache[1].valid = pTableCache[0].valid;

        pTableCache[0].entry = entry;
        pTableCache[0].vaddr = vaddr;
        pTableCache[0].valid = true;
    }

    /**
     * Erase an entry from the page table cache.
     * @param vaddr virtual address (page aligned) to check
     */
    inline void eraseCacheEntry(Addr vaddr)
    {
        // Invalidate cached entries if necessary
        if (pTableCache[0].valid && pTableCache[0].vaddr == vaddr) {
            pTableCache[0].valid = false;
        } else if (pTableCache[1].valid && pTableCache[1].vaddr == vaddr) {
            pTableCache[1].valid = false;
        } else if (pTableCache[2].valid && pTableCache[2].vaddr == vaddr) {
            pTableCache[2].valid = false;
        }
    }
};

/**
 * Declaration of functional page table.
 */
class FuncPageTable : public PageTableBase
{
  private:
    typedef std::unordered_map<Addr, TheISA::TlbEntry> PTable;
    typedef PTable::iterator PTableItr;
    PTable pTable;

  public:

    FuncPageTable(const std::string &__name, uint64_t _pid,
                  Addr _pageSize = TheISA::PageBytes);

    ~FuncPageTable();

    void initState(ThreadContext* tc) override
    {
    }

    void map(Addr vaddr, Addr paddr, int64_t size,
             uint64_t flags = 0) override;
    void remap(Addr vaddr, int64_t size, Addr new_vaddr) override;
    void unmap(Addr vaddr, int64_t size) override;

    /**
     * Check if any pages in a region are already allocated
     * @param vaddr The starting virtual address of the region.
     * @param size The length of the region.
     * @return True if no pages in the region are mapped.
     */
    bool isUnmapped(Addr vaddr, int64_t size) override;

    /**
     * Lookup function
     * @param vaddr The virtual address.
     * @return entry The page table entry corresponding to vaddr.
     */
    bool lookup(Addr vaddr, TheISA::TlbEntry &entry) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

/**
 * Faux page table class indended to stop the usage of
 * an architectural page table, when there is none defined
 * for a particular ISA.
 */
class NoArchPageTable : public FuncPageTable
{
  public:
    NoArchPageTable(const std::string &__name, uint64_t _pid, System *_sys,
              Addr _pageSize = TheISA::PageBytes) : FuncPageTable(__name, _pid)
    {
        fatal("No architectural page table defined for this ISA.\n");
    }
};

#endif // __MEM_PAGE_TABLE_HH__
