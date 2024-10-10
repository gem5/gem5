/*
 * Copyright (c) 2024 Pranith Kumar
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
 */

/**
 * @file
 * Declaration of a BTB entry and BTB indexing policy.
 */

#ifndef __CPU_PRED_BTB_ENTRY_HH__
#define __CPU_PRED_BTB_ENTRY_HH__

#include <vector>

#include "arch/generic/pcstate.hh"
#include "base/intmath.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "params/BTBIndexingPolicy.hh"
#include "params/BTBSetAssociative.hh"

namespace gem5 {

class BTBTagType
{
  public:
    struct KeyType
    {
        Addr address;
        ThreadID tid;
    };
    using Params = BTBIndexingPolicyParams;
};

using BTBIndexingPolicy = IndexingPolicyTemplate<BTBTagType>;
template class IndexingPolicyTemplate<BTBTagType>;

class BTBSetAssociative : public BTBIndexingPolicy
{
  public:
    PARAMS(BTBSetAssociative);
    using KeyType = BTBTagType::KeyType;

    BTBSetAssociative(const Params &p)
        : BTBIndexingPolicy(p, p.num_entries, p.set_shift),
          tagMask(mask(p.tag_bits))
    {
        setNumThreads(p.numThreads);
    }

  protected:
    /**
     * Extract the set index for the instruction PC based on tid.
     */
    uint32_t
    extractSet(const KeyType &key) const
    {
        return ((key.address >> setShift)
                ^ (key.tid << (tagShift - setShift - log2NumThreads)))
            & setMask;
    }

  public:
    /**
     * Find all possible entries for insertion and replacement of an address.
     */
    std::vector<ReplaceableEntry*>
    getPossibleEntries(const KeyType &key) const override
    {
        auto set_idx = extractSet(key);

        assert(set_idx < sets.size());

        return sets[set_idx];
    }

    /**
     * Set number of threads sharing the BTB
     */
    void
    setNumThreads(unsigned num_threads)
    {
        log2NumThreads = log2i(num_threads);
    }

    /**
     * Generate the tag from the given address.
     */
    Addr
    extractTag(const Addr addr) const override
    {
        return (addr >> tagShift) & tagMask;
    }

    Addr regenerateAddr(const KeyType &key,
                        const ReplaceableEntry* entry) const override
    {
        panic("Not implemented!");
        return 0;
    }

  private:
    const uint64_t tagMask;
    unsigned log2NumThreads;
};

namespace branch_prediction
{

class BTBEntry : public ReplaceableEntry
{
  public:
    using IndexingPolicy = gem5::BTBIndexingPolicy;
    using KeyType = gem5::BTBTagType::KeyType;
    using TagExtractor = std::function<Addr(Addr)>;

    /** Default constructor */
    BTBEntry(TagExtractor ext)
        : inst(nullptr), extractTag(ext), valid(false), tag({MaxAddr, -1})
    {}

    /** Update the target and instruction in the BTB entry.
     *  During insertion, only the tag (key) is updated.
     */
    void
    update(const PCStateBase &_target,
           StaticInstPtr _inst)
    {
        set(target, _target);
        inst = _inst;
    }

    /**
     * Checks if the given tag information corresponds to this entry's.
     */
    bool
    match(const KeyType &key) const
    {
        return isValid() && (tag.address == extractTag(key.address))
            && (tag.tid == key.tid);
    }

    /**
     * Insert the block by assigning it a tag and marking it valid. Touches
     * block if it hadn't been touched previously.
     */
    void
    insert(const KeyType &key)
    {
        setValid();
        setTag({extractTag(key.address), key.tid});
    }

    /** Copy constructor */
    BTBEntry(const BTBEntry &other)
    {
        valid      = other.valid;
        tag        = other.tag;
        inst       = other.inst;
        extractTag = other.extractTag;
        set(target, other.target);
    }

    /** Assignment operator */
    BTBEntry& operator=(const BTBEntry &other)
    {
        valid      = other.valid;
        tag        = other.tag;
        inst       = other.inst;
        extractTag = other.extractTag;
        set(target, other.target);

        return *this;
    }

    /**
     * Checks if the entry is valid.
     */
    bool isValid() const { return valid; }

    /**
     * Get tag associated to this block.
     */
    KeyType getTag() const { return tag; }

    /** Invalidate the block. Its contents are no longer valid. */
    void
    invalidate()
    {
        valid = false;
        setTag({MaxAddr, -1});
    }

    /** The entry's target. */
    std::unique_ptr<PCStateBase> target;

    /** Pointer to the static branch inst at this address */
    StaticInstPtr inst;

    std::string
    print() const override
    {
        return csprintf("tag: %#x tid: %d valid: %d | %s", tag.address, tag.tid,
                        isValid(), ReplaceableEntry::print());
    }

  protected:
    /**
     * Set tag associated to this block.
     */
    void setTag(KeyType _tag) { tag = _tag; }

    /** Set valid bit. The block must be invalid beforehand. */
    void
    setValid()
    {
        assert(!isValid());
        valid = true;
    }

  private:
    /** Callback used to extract the tag from the entry */
    TagExtractor extractTag;

    /**
     * Valid bit. The contents of this entry are only valid if this bit is set.
     * @sa invalidate()
     * @sa insert()
     */
    bool valid;

    /** The entry's tag. */
    KeyType tag;
};

} // namespace gem5::branch_prediction
/**
 * This helper generates a tag extractor function object
 * which will be typically used by Replaceable entries indexed
 * with the BaseIndexingPolicy.
 * It allows to "decouple" indexing from tagging. Those entries
 * would call the functor without directly holding a pointer
 * to the indexing policy which should reside in the cache.
 */
static constexpr auto
genTagExtractor(BTBIndexingPolicy *ip)
{
    return [ip] (Addr addr) { return ip->extractTag(addr); };
}

}

#endif //__CPU_PRED_BTB_ENTRY_HH__
