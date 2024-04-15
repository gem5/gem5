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
 */

/**
 * @file
 * Declarations of a non-full system Page Table.
 */

#ifndef __MEM_PAGE_TABLE_HH__
#define __MEM_PAGE_TABLE_HH__

#include <string>
#include <unordered_map>

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base/types.hh"
#include "mem/request.hh"
#include "mem/translation_gen.hh"
#include "sim/serialize.hh"

namespace gem5
{

class ThreadContext;

class EmulationPageTable : public Serializable
{
  public:
    struct Entry
    {
        Addr paddr;
        uint64_t flags;

        Entry(Addr paddr, uint64_t flags) : paddr(paddr), flags(flags) {}

        Entry() {}
    };

  protected:
    typedef std::unordered_map<Addr, Entry> PTable;
    typedef PTable::iterator PTableItr;
    PTable pTable;

    const Addr _pageSize;
    const Addr offsetMask;

    const uint64_t _pid;
    const std::string _name;

  public:
    EmulationPageTable(const std::string &__name, uint64_t _pid,
                       Addr _pageSize)
        : _pageSize(_pageSize),
          offsetMask(mask(floorLog2(_pageSize))),
          _pid(_pid),
          _name(__name),
          shared(false)
    {
        assert(isPowerOf2(_pageSize));
    }

    uint64_t
    pid() const
    {
        return _pid;
    };

    virtual ~EmulationPageTable(){};

    /* generic page table mapping flags
     *              unset | set
     * bit 0 - no-clobber | clobber
     * bit 2 - cacheable  | uncacheable
     * bit 3 - read-write | read-only
     */
    enum MappingFlags : uint32_t
    {
        Clobber = 1,
        Uncacheable = 4,
        ReadOnly = 8,
    };

    // flag which marks the page table as shared among software threads
    bool shared;

    virtual void initState(){};

    // for DPRINTF compatibility
    const std::string
    name() const
    {
        return _name;
    }

    Addr
    pageAlign(Addr a)
    {
        return (a & ~offsetMask);
    }

    Addr
    pageOffset(Addr a)
    {
        return (a & offsetMask);
    }

    // Page size can technically vary based on the virtual address, but we'll
    // ignore that for now.
    Addr
    pageSize()
    {
        return _pageSize;
    }

    /**
     * Maps a virtual memory region to a physical memory region.
     * @param vaddr The starting virtual address of the region.
     * @param paddr The starting physical address where the region is mapped.
     * @param size The length of the region.
     * @param flags Generic mapping flags that can be set by or-ing values
     *              from MappingFlags enum.
     */
    virtual void map(Addr vaddr, Addr paddr, int64_t size, uint64_t flags = 0);
    virtual void remap(Addr vaddr, int64_t size, Addr new_vaddr);
    virtual void unmap(Addr vaddr, int64_t size);

    /**
     * Check if any pages in a region are already allocated
     * @param vaddr The starting virtual address of the region.
     * @param size The length of the region.
     * @return True if no pages in the region are mapped.
     */
    virtual bool isUnmapped(Addr vaddr, int64_t size);

    /**
     * Lookup function
     * @param vaddr The virtual address.
     * @return The page table entry corresponding to vaddr.
     */
    const Entry *lookup(Addr vaddr);

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
    bool
    translate(Addr vaddr)
    {
        Addr dummy;
        return translate(vaddr, dummy);
    }

    class PageTableTranslationGen : public TranslationGen
    {
      private:
        EmulationPageTable *pt;

        void translate(Range &range) const override;

      public:
        PageTableTranslationGen(EmulationPageTable *_pt, Addr vaddr, Addr size)
            : TranslationGen(vaddr, size), pt(_pt)
        {}
    };

    TranslationGenPtr
    translateRange(Addr vaddr, Addr size)
    {
        return TranslationGenPtr(
            new PageTableTranslationGen(this, vaddr, size));
    }

    /**
     * Perform a translation on the memory request, fills in paddr
     * field of req.
     * @param req The memory request.
     */
    Fault translate(const RequestPtr &req);

    /**
     * Dump all items in the pTable, to a concatenation of strings of the form
     *    Addr:Entry;
     */
    const std::string externalize() const;

    void getMappings(std::vector<std::pair<Addr, Addr>> *addr_mappings);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif // __MEM_PAGE_TABLE_HH__
