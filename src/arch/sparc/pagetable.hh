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
 */

#ifndef __ARCH_SPARC_PAGETABLE_HH__
#define __ARCH_SPARC_PAGETABLE_HH__

#include <cassert>

#include "base/bitfield.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace gem5
{

namespace SparcISA
{

class TteTag
{
  private:
    uint64_t entry;
    bool populated;

  public:
    TteTag() : entry(0), populated(false) {}

    TteTag(uint64_t e) : entry(e), populated(true) {}

    const TteTag &
    operator=(uint64_t e)
    {
        populated = true;
        entry = e;
        return *this;
    }

    bool
    valid() const
    {
        assert(populated);
        return !bits(entry, 62, 62);
    }

    Addr
    va() const
    {
        assert(populated);
        return bits(entry, 41, 0);
    }
};

class PageTableEntry
{
  public:
    enum EntryType
    {
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

    void
    populate(uint64_t e, EntryType t = sun4u)
    {
        entry = e;
        type = t;
        populated = true;

        // If we get a sun4v format TTE, turn it into a sun4u
        if (type == sun4u)
            entry4u = entry;
        else {
            entry4u = 0;
            entry4u |= mbits(entry, 63, 63);      // valid
            entry4u |= bits(entry, 1, 0) << 61;   // size[1:0]
            entry4u |= bits(entry, 62, 62) << 60; // nfo
            entry4u |= bits(entry, 12, 12) << 59; // ie
            entry4u |= bits(entry, 2, 2) << 48;   // size[2]
            entry4u |= mbits(entry, 39, 13);      // paddr
            entry4u |= bits(entry, 61, 61) << 6;
            ;                                    // locked
            entry4u |= bits(entry, 10, 10) << 5; // cp
            entry4u |= bits(entry, 9, 9) << 4;   // cv
            entry4u |= bits(entry, 11, 11) << 3; // e
            entry4u |= bits(entry, 8, 8) << 2;   // p
            entry4u |= bits(entry, 6, 6) << 1;   // w
        }
    }

    void
    clear()
    {
        populated = false;
    }

    static int pageSizes[6];

    uint64_t
    operator()() const
    {
        assert(populated);
        return entry4u;
    }

    const PageTableEntry &
    operator=(uint64_t e)
    {
        populated = true;
        entry4u = e;
        return *this;
    }

    const PageTableEntry &
    operator=(const PageTableEntry &e)
    {
        populated = true;
        entry4u = e.entry4u;
        type = e.type;
        return *this;
    }

    bool
    valid() const
    {
        return bits(entry4u, 63, 63) && populated;
    }

    uint8_t
    _size() const
    {
        assert(populated);
        return bits(entry4u, 62, 61) | bits(entry4u, 48, 48) << 2;
    }

    Addr
    size() const
    {
        assert(_size() < 6);
        return pageSizes[_size()];
    }

    Addr
    sizeMask() const
    {
        return size() - 1;
    }

    bool
    ie() const
    {
        return bits(entry4u, 59, 59);
    }

    Addr
    pfn() const
    {
        assert(populated);
        return bits(entry4u, 39, 13);
    }

    Addr
    paddr() const
    {
        assert(populated);
        return mbits(entry4u, 39, 13);
    }

    bool
    locked() const
    {
        assert(populated);
        return bits(entry4u, 6, 6);
    }

    bool
    cv() const
    {
        assert(populated);
        return bits(entry4u, 4, 4);
    }

    bool
    cp() const
    {
        assert(populated);
        return bits(entry4u, 5, 5);
    }

    bool
    priv() const
    {
        assert(populated);
        return bits(entry4u, 2, 2);
    }

    bool
    writable() const
    {
        assert(populated);
        return bits(entry4u, 1, 1);
    }

    bool
    nofault() const
    {
        assert(populated);
        return bits(entry4u, 60, 60);
    }

    bool
    sideffect() const
    {
        assert(populated);
        return bits(entry4u, 3, 3);
    }

    Addr
    paddrMask() const
    {
        assert(populated);
        return paddr() & ~sizeMask();
    }

    Addr
    translate(Addr vaddr) const
    {
        assert(populated);
        Addr mask = sizeMask();
        return (paddr() & ~mask) | (vaddr & mask);
    }
};

struct TlbRange
{
    Addr va;
    Addr size;
    int contextId;
    int partitionId;
    bool real;

    inline bool
    operator<(const TlbRange &r2) const
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

    inline bool
    operator==(const TlbRange &r2) const
    {
        return va == r2.va && size == r2.size && contextId == r2.contextId &&
               partitionId == r2.partitionId && real == r2.real;
    }
};

struct TlbEntry
{
    TlbEntry() {}

    TlbEntry(Addr asn, Addr vaddr, Addr paddr, bool uncacheable,
             bool read_only)
    {
        uint64_t entry = 0;
        if (!read_only)
            entry |= 1ULL << 1; // Writable
        entry |= 0ULL << 2;     // Available in nonpriveleged mode
        entry |= 0ULL << 3;     // No side effects
        if (!uncacheable) {
            entry |= 1ULL << 4; // Virtually cachable
            entry |= 1ULL << 5; // Physically cachable
        }
        entry |= 0ULL << 6;            // Not locked
        entry |= mbits(paddr, 39, 13); // Physical address
        entry |= 0ULL << 48;           // size = 8k
        entry |= 0uLL << 59;           // Endianness not inverted
        entry |= 0ULL << 60;           // Not no fault only
        entry |= 0ULL << 61;           // size = 8k
        entry |= 1ULL << 63;           // valid
        pte = PageTableEntry(entry);

        range.va = vaddr;
        range.size = 8 * (1 << 10);
        range.contextId = asn;
        range.partitionId = 0;
        range.real = false;

        valid = true;
    }

    TlbRange range;
    PageTableEntry pte;
    bool used;
    bool valid;

    Addr
    pageStart()
    {
        return pte.paddr();
    }

    void
    updateVaddr(Addr new_vaddr)
    {
        range.va = new_vaddr;
    }

    void serialize(CheckpointOut &cp) const;
    void unserialize(CheckpointIn &cp);
};

} // namespace SparcISA
} // namespace gem5

#endif // __ARCH_SPARC_PAGE_TABLE_HH__
