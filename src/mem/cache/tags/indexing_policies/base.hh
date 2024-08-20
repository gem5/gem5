/*
 * Copyright (c) 2018 Inria
 * Copyright (c) 2012-2014,2017,2024 Arm Limited
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
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
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
 * Declaration of a common framework for indexing policies.
 */

#ifndef __MEM_CACHE_INDEXING_POLICIES_BASE_HH__
#define __MEM_CACHE_INDEXING_POLICIES_BASE_HH__

#include <vector>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "params/BaseIndexingPolicy.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * A common base class for indexing table locations. Classes that inherit
 * from it determine hash functions that should be applied based on the set
 * and way. These functions are then applied to re-map the original values.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 * @tparam Types the Types template parameter shall contain the following type
 *         traits:
 *           - KeyType = The key/lookup data type
 *           - Params = The indexing policy Param type
 */
template <class Types>
class IndexingPolicyTemplate : public SimObject
{
  protected:
    using KeyType = typename Types::KeyType;
    using Params = typename Types::Params;

    /**
     * The associativity.
     */
    const unsigned assoc;

    /**
     * The number of sets in the cache.
     */
    const uint32_t numSets;

    /**
     * The amount to shift the address to get the set.
     */
    const int setShift;

    /**
     * Mask out all bits that aren't part of the set index.
     */
    const unsigned setMask;

    /**
     * The cache sets.
     */
    std::vector<std::vector<ReplaceableEntry*>> sets;

    /**
     * The amount to shift the address to get the tag.
     */
    const int tagShift;

  public:
    /**
     * Construct and initialize this policy.
     */
    IndexingPolicyTemplate(const Params &p)
      : SimObject(p), assoc(p.assoc),
        numSets(p.size / (p.entry_size * assoc)),
        setShift(floorLog2(p.entry_size)), setMask(numSets - 1), sets(numSets),
        tagShift(setShift + floorLog2(numSets))
    {
        fatal_if(!isPowerOf2(numSets), "# of sets must be non-zero and a power " \
                 "of 2");
        fatal_if(assoc <= 0, "associativity must be greater than zero");

        // Make space for the entries
        for (uint32_t i = 0; i < numSets; ++i) {
            sets[i].resize(assoc);
        }
    }

    /**
     * Destructor.
     */
    ~IndexingPolicyTemplate() {};

    /**
     * Associate a pointer to an entry to its physical counterpart.
     *
     * @param entry The entry pointer.
     * @param index An unique index for the entry.
     */
    void
    setEntry(ReplaceableEntry* entry, const uint64_t index)
    {
        // Calculate set and way from entry index
        const std::lldiv_t div_result = std::div((long long)index, assoc);
        const uint32_t set = div_result.quot;
        const uint32_t way = div_result.rem;

        // Sanity check
        assert(set < numSets);

        // Assign a free pointer
        sets[set][way] = entry;

        // Inform the entry its position
        entry->setPosition(set, way);
    }

    /**
     * Get an entry based on its set and way. All entries must have been set
     * already before calling this function.
     *
     * @param set The set of the desired entry.
     * @param way The way of the desired entry.
     * @return entry The entry pointer.
     */
    ReplaceableEntry*
    getEntry(const uint32_t set, const uint32_t way) const
    {
        return sets[set][way];
    }

    /**
     * Generate the tag from the given address.
     *
     * @param addr The address to get the tag from.
     * @return The tag of the address.
     */
    virtual Addr
    extractTag(const Addr addr) const
    {
        return (addr >> tagShift);
    }


    /**
     * Find all possible entries for insertion and replacement of an address.
     * Should be called immediately before ReplacementPolicy's findVictim()
     * not to break cache resizing.
     *
     * @param addr The addr to a find possible entries for.
     * @return The possible entries.
     */
    virtual std::vector<ReplaceableEntry*> getPossibleEntries(const KeyType &key)
                                                                    const = 0;

    /**
     * Regenerate an entry's address from its tag and assigned indexing bits.
     *
     * @param tag The tag bits.
     * @param entry The entry.
     * @return the entry's original address.
     */
    virtual Addr regenerateAddr(const KeyType &key,
                                const ReplaceableEntry* entry) const = 0;
};

class AddrTypes
{
  public:
    using KeyType = Addr;
    using Params = BaseIndexingPolicyParams;
};

using BaseIndexingPolicy = IndexingPolicyTemplate<AddrTypes>;
template class IndexingPolicyTemplate<AddrTypes>;

/**
 * This helper generates an a tag extractor function object
 * which will be typically used by Replaceable entries indexed
 * with the BaseIndexingPolicy.
 * It allows to "decouple" indexing from tagging. Those entries
 * would call the functor without directly holding a pointer
 * to the indexing policy which should reside in the cache.
 */
static constexpr auto
genTagExtractor(BaseIndexingPolicy *ip)
{
    return [ip] (Addr addr) { return ip->extractTag(addr); };
}

} // namespace gem5

#endif //__MEM_CACHE_INDEXING_POLICIES_BASE_HH__
