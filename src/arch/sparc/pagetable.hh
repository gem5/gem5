/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

#ifndef __ARCH_SPARC_PAGETABLE_HH__
#define __ARCH_SPARC_PAGETABLE_HH__

#include "arch/sparc/isa_traits.hh"
#include "config/full_system.hh"

namespace SparcISA
{
struct VAddr
{
    VAddr(Addr a) { panic("not implemented yet."); }
};

class PageTableEntry
{
  public:
    enum EntryType {
      sun4v,
      sun4u,
      invalid
    };

  private:
    uint64_t entry;
    EntryType type;
    uint64_t entry4u;
    bool populated;


  public:
    PageTableEntry() : entry(0), type(invalid), populated(false) {}

    PageTableEntry(uint64_t e, EntryType t = sun4u)
        : entry(e), type(t), populated(true)

    {
        populate(entry, type);
    }

    void populate(uint64_t e, EntryType t = sun4u)
    {
        entry = e;
        type = t;
        populated = true;

        // If we get a sun4v format TTE, turn it into a sun4u
        if (type == sun4u)
            entry4u = entry;
        else {
            uint64_t entry4u = 0;
            entry4u |= entry & ULL(0x8000000000000000);              //valid
            entry4u |= (entry & 0x3) << 61;                     //size[1:0]
            entry4u |= (entry & ULL(0x4000000000000000)) >> 2;       //nfo
            entry4u |= (entry & 0x1000) << 47;                  //ie
            //entry4u |= (entry & 0x3F00000000000000) >> 7;       //soft2
            entry4u |= (entry & 0x4) << 48;                     //size[2]
                                                                //diag?
            entry4u |= (entry & ULL(0x0000FFFFFFFFE000));            //paddr
            entry4u |= (entry & 0x400) >> 5;                    //cp
            entry4u |= (entry & 0x200) >> 5;                    //cv
            entry4u |= (entry & 0x800) >> 8;                    //e
            entry4u |= (entry & 0x100) >> 6;                    //p
            entry4u |= (entry & 0x40) >> 5;                     //w
        }
    }

    void clear()
    {
        populated = false;
    }

    static int pageSizes[6];


    uint64_t operator()() const { assert(populated); return entry4u; }
    const PageTableEntry &operator=(uint64_t e) { populated = true;
                                                  entry4u = e; return *this; }

    const PageTableEntry &operator=(const PageTableEntry &e)
    { populated = true; entry4u = e.entry4u; return *this; }

    bool    valid()    const { return entry4u & ULL(0x8000000000000000) && populated; }
    uint8_t _size()     const { assert(populated);
                               return ((entry4u & 0x6) >> 61) |
                                      ((entry4u & ULL(0x000080000000000)) >> 46); }
    Addr    size()     const { return pageSizes[_size()]; }
    bool    ie()       const { return entry4u >> 59 & 0x1; }
    Addr    pfn()      const { assert(populated);
                               return entry4u >> 13 & ULL(0xFFFFFFFFFF); }
    Addr    paddr()    const { assert(populated);
                               return entry4u & ULL(0x0000FFFFFFFFE000); }
    bool    locked()   const { assert(populated);
                               return entry4u & 0x40; }
    bool    cv()       const { assert(populated);
                               return entry4u & 0x10; }
    bool    cp()       const { assert(populated);
                               return entry4u & 0x20; }
    bool    priv()     const { assert(populated);
                               return entry4u & 0x4; }
    bool    writable() const { assert(populated);
                               return entry4u & 0x2; }
    bool    nofault()  const { assert(populated);
                               return entry4u & ULL(0x1000000000000000); }
    bool    sideffect() const { assert(populated);
                                return entry4u & 0x8; }
};

struct TlbRange {
    Addr va;
    Addr size;
    int contextId;
    int partitionId;
    bool real;

    inline bool operator<(const TlbRange &r2) const
    {
        if (real && !r2.real)
            return true;
        if (!real && r2.real)
            return false;

        if (!real && !r2.real) {
            if (contextId < r2.contextId)
                return true;
            else if (contextId > r2.contextId)
                return false;
        }

        if (partitionId < r2.partitionId)
            return true;
        else if (partitionId > r2.partitionId)
            return false;

        if (va < r2.va)
            return true;
        return false;
    }
    inline bool operator==(const TlbRange &r2) const
    {
        return va == r2.va &&
               size == r2.size &&
               contextId == r2.contextId &&
               partitionId == r2.partitionId &&
               real == r2.real;
    }
};


struct TlbEntry {
    TlbRange range;
    PageTableEntry pte;
    bool used;
    bool valid;

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);

};


}; // namespace SparcISA

#endif // __ARCH_SPARC_PAGE_TABLE_HH__

