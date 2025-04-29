/**
 * Copyright (c) 2024 Arm Limited
 * All rights reserved
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
 * Copyright (c) 2020 Inria
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

#ifndef __CACHE_TAGGED_ENTRY_HH__
#define __CACHE_TAGGED_ENTRY_HH__

#include <cassert>
#include "debug/CacheAx.hh"

#include "base/cprintf.hh"
#include "base/types.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "params/TaggedIndexingPolicy.hh"
#include "params/TaggedSetAssociative.hh"

namespace gem5
{

class TaggedTypes
{
  public:
    struct KeyType
    {
        Addr address;
        bool secure;
    };
    using Params = TaggedIndexingPolicyParams;
};

using TaggedIndexingPolicy = IndexingPolicyTemplate<TaggedTypes>;
template class IndexingPolicyTemplate<TaggedTypes>;

/**
 * This version of set associative indexing deals with
 * a Lookup structure made of address and secure bit.
 * It extracts the address but discards the secure bit which
 * is used for tagging only
 */
class TaggedSetAssociative : public TaggedIndexingPolicy
{
  protected:
    virtual uint32_t
    extractSet(const KeyType &key) const
    {
        DPRINTF(CacheAx, "[TaggedSetAssociative::extractSet] setShift: %x, setMask: %x\n", setShift, setMask);
        DPRINTF(CacheAx, "[TaggedSetAssociative::extractSet] Extracted set index : %x for address : %x\n", (key.address >> (setShift)) & setMask, key.address);
        return (key.address >> setShift) & setMask;
    }

    uint32_t
    extractSetAx(const Addr addr) const
    {
      
      DPRINTF(CacheAx, "[TaggedSetAssociative::extractSetAx] setShift: %x, setMask: %x\n", setShift, setMask);
      DPRINTF(CacheAx, "[TaggedSetAssociative::extractSetAx] Extracted set index : %x for address : %x\n", (addr >> (setShift + 1)) & setMask, addr);
      return (addr >> (setShift + 1)) & setMask;
    }

  public:
    PARAMS(TaggedSetAssociative);
    TaggedSetAssociative(const Params &p)
      : TaggedIndexingPolicy(p, p.size / p.entry_size, floorLog2(p.entry_size))
    {}

    std::vector<ReplaceableEntry*>
    getPossibleEntries(const KeyType &key) const override
    {
        return sets[extractSet(key)];
    }

    std::vector<ReplaceableEntry*>
    getPossibleEntriesAx(const KeyType &key) const override
    {
        return sets[extractSetAx(key.address)];
    }

    Addr
    regenerateAddr(const KeyType &key,
                   const ReplaceableEntry *entry) const override
    {
        return (key.address << tagShift) | (entry->getSet() << setShift);
    }
};

/**
 * A tagged entry is an entry containing a tag. Each tag is accompanied by a
 * secure bit, which informs whether it belongs to a secure address space.
 * A tagged entry's contents are only relevant if it is marked as valid.
 */
class TaggedEntry : public ReplaceableEntry
{
  public:
    using KeyType = TaggedTypes::KeyType;
    using IndexingPolicy = TaggedIndexingPolicy;
    using TagExtractor = std::function<Addr(Addr)>;

    TaggedEntry()
      : _valid(false), _secure(false), _tag(MaxAddr)
    {}
    ~TaggedEntry() = default;

    void
    registerTagExtractor(TagExtractor ext)
    {
        extractTag = ext;
    }

    /**
     * Checks if the entry is valid.
     *
     * @return True if the entry is valid.
     */
    virtual bool isValid() const { return _valid; }

    /*
     * ECE757:
     *   Check if the first half or second half of this entry is valid.
     *
     * @param first_half True to check first half, false to check second half
     * @return True if the specified half is valid
     */
    virtual bool isHalfValid(bool first_half) const 
    { 
        return first_half ? _firstValidAx : _SecondValidAx; 
    }

    /**
     * Check if this block holds data from the secure memory space.
     *
     * @return True if the block holds data from the secure memory space.
     */
    bool isSecure() const { return _secure; }

    /**
     * Get tag associated to this block.
     *
     * @return The tag value.
     */
    virtual Addr getTag() const { return _tag; }

    /**
     * Checks if the given tag information corresponds to this entry's.
     *
     * @param tag The tag value to compare to.
     * @param is_secure Whether secure bit is set.
     * @return True if the tag information match this entry's.
     */
    bool
    match(const KeyType &key) const
    {
        return isValid() && (getTag() == extractTag(key.address)) &&
            (isSecure() == key.secure);
    }

    /**
     * Insert the block by assigning it a tag and marking it valid. Touches
     * block if it hadn't been touched previously.
     *
     * @param tag The tag value.
     */
    virtual void
    insert(const KeyType &key)
    {
        setValid();
        setTag(extractTag(key.address));
        if (key.secure) {
            setSecure();
        }
    }

    /* ECE757:
     *   Insert approximate data into one half of the block.
     *
     * @param key The tag and secure information
     * @param first_half True to use first half, false to use second half
     */
    virtual void insertAx(const KeyType &key, bool first_half)
    {
        DPRINTF(CacheAx, "taged_entry.hh: Correct version of insertAx(?)\n");
        setValid();
        setTag(extractTag(key.address));
        if (key.secure) {
            setSecure();
        }
        
        // Set only the specified half as valid
        setHalfValid(first_half, !first_half);
    }

    /** Invalidate the block. Its contents are no longer valid. */
    virtual void invalidate()
    {
        _valid = false;

        /*
         * ECE757
         *   Add the invalidation for _firstValidAx and _SecondValidAx.
         */
        _firstValidAx = false;
        _SecondValidAx = false;

        setTag(MaxAddr);
        clearSecure();
    }

    std::string
    print() const override
    {
        return csprintf("tag: %#x secure: %d valid: %d | %s", getTag(),
            isSecure(), isValid(), ReplaceableEntry::print());
    }

  protected:
    /**
     * Set tag associated to this block.
     *
     * @param tag The tag value.
     */
    virtual void setTag(Addr tag) { _tag = tag; }

    /** Set secure bit. */
    virtual void setSecure() { _secure = true; }

    /** Clear secure bit. Should be only used by the invalidation function. */
    void clearSecure() { _secure = false; }

    /** Set valid bit. The block must be invalid beforehand. */
    virtual void
    setValid()
    {
        assert(!isValid());
        _valid = true;
    }

    /** Callback used to extract the tag from the entry */
    TagExtractor extractTag;

  private:
    /**
     * Valid bit. The contents of this entry are only valid if this bit is set.
     * @sa invalidate()
     * @sa insert()
     */
    bool _valid;

    /*
     * ECE757
     *   Add two valid bit for representing if the first or the second block
     *   of the approximate cache block is valid or not
     */
    bool _firstValidAx;
    bool _SecondValidAx;

    /*
     * ECE757
     *   Set the valid bits for the first and/or second half of this entry.
     *   If either half becomes valid, the main valid bit is also set.
     *
     * @param first_half Whether to set the first half valid
     * @param second_half Whether to set the second half valid
     */
    virtual void setHalfValid(bool first_half, bool second_half)
    {
        if (first_half) { _firstValidAx = true; }
        if (second_half) { _SecondValidAx = true; }
        
        // If either half is valid, the main valid (_valid) is set as well
        if (_firstValidAx || _SecondValidAx) {
            _valid = true;
        }
    }

    /**
     * Secure bit. Marks whether this entry refers to an address in the secure
     * memory space. Must always be modified along with the tag.
     */
    bool _secure;

    /** The entry's tag. */
    Addr _tag;
};

/**
 * This helper generates an a tag extractor function object
 * which will be typically used by Replaceable entries indexed
 * with the TaggedIndexingPolicy.
 * It allows to "decouple" indexing from tagging. Those entries
 * would call the functor without directly holding a pointer
 * to the indexing policy which should reside in the cache.
 */
static constexpr auto
genTagExtractor(TaggedIndexingPolicy *ip)
{
    return [ip] (Addr addr) { return ip->extractTag(addr); };
}

} // namespace gem5

#endif//__CACHE_TAGGED_ENTRY_HH__
