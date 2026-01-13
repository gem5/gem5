/*
 * Copyright (c) 2025 Jason Lowe-Power
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

#ifndef __ARCH_AMDGPU_VEGA_PAGE_WALK_CACHE_HH__
#define __ARCH_AMDGPU_VEGA_PAGE_WALK_CACHE_HH__


#include "arch/amdgpu/vega/pagetable.hh"
#include "base/cache/associative_cache.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/set_associative.hh"
#include "params/VegaPWCIndexingPolicy.hh"

namespace gem5
{


namespace VegaISA
{

// Page walk cache entry
struct PWCEntry : public ReplaceableEntry
{
  public:
    using IndexingPolicy = VegaPWCIndexingPolicy;
    using KeyType = Addr;

    // The data stored in PTE
    PageTableEntry pteEntry;

    // The physical address of the page table entry
    Addr paddr;

    bool valid;

    void
    invalidate()
    {
        valid = false;
    }

    void insert(const KeyType &key) {}
    bool isValid() const { return valid; }

    bool
    match(const KeyType &key) const
    {
        return valid && paddr == key;
    }
};

/**
 * Set associative indexing policy for the page walk cache.
 * This policy differs from the standard set associative policy in that it
 * hard codes the size of the entry to be 8 bytes (1 PTE).
 * Note that other PWC designs may need to extend or modify this policy.
 */
class VegaPWCIndexingPolicy : public BaseIndexingPolicy
{
  protected:
    virtual uint32_t extractSet(const Addr addr) const
    {
        // Extract the set bits from the address
        return (addr >> setShift) & setMask;
    }

  public:
    /**
     * Convenience typedef.
     */
    typedef VegaPWCIndexingPolicyParams Params;

    /**
     * Construct and initialize this policy.
     * Assume that all PTEs are 8 bytes so the set shift is 3 bits.
     */
    VegaPWCIndexingPolicy(const Params &p) :
        BaseIndexingPolicy(p, p.entries, 3)
    {}

    /**
     * Destructor.
     */
    ~VegaPWCIndexingPolicy() {};

    std::vector<ReplaceableEntry*> getPossibleEntries(const Addr &addr) const
                                                                     override
    {
        return sets[extractSet(addr)];
    }

    Addr regenerateAddr(const Addr &tag,
                        const ReplaceableEntry* entry) const override
    {
        panic("PWCSetAssociative::regenerateAddr() not implemented");
        return 0;
    }
};

// Page walk cache
class PageWalkCache : public AssociativeCache<PWCEntry>
{
  public:
    using AssociativeCache<PWCEntry>::AssociativeCache;
    using AssociativeCache<PWCEntry>::accessEntry;

    PWCEntry* accessEntry(const KeyType &key) override
    {
        auto entry = findEntry(key);
        accessEntry(entry);
        return entry;
    }
    PWCEntry* findEntry(const KeyType &key) const override
    {
        for (auto candidate : indexingPolicy->getPossibleEntries(key)) {
        auto entry = static_cast<PWCEntry*>(candidate);
        if (entry->match(key))
            return entry;
        }
        return nullptr;
    }
    void insert(const KeyType &key, const PageTableEntry &pte_entry) {
        PWCEntry *vict = findVictim(key);
        vict->pteEntry = pte_entry;
        vict->paddr = key;
        vict->valid = true;
        insertEntry(key, vict);
    }
};

} // namespace VegaISA
} // namespace gem5

#endif // __ARCH_AMDGPU_VEGA_PAGE_WALK_CACHE_HH__
