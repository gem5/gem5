/**
 * Copyright (c) 2024 - Pranith Kumar
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

#ifndef __BASE_CACHE_CACHE_ENTRY_HH__
#define __BASE_CACHE_CACHE_ENTRY_HH__

#include <cassert>

#include "base/cprintf.hh"
#include "base/types.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"

namespace gem5
{

/**
 * A CacheEntry is an entry containing a tag. A tagged entry's contents
 * are only relevant if it is marked as valid.
 */
class CacheEntry : public ReplaceableEntry
{
  public:
    CacheEntry() = default;
    ~CacheEntry() = default;

    /**
     * Checks if the entry is valid.
     *
     * @return True if the entry is valid.
     */
    virtual bool
    isValid() const
    {
        return valid;
    }

    /**
     * Get tag associated to this block.
     *
     * @return The tag value.
     */
    virtual Addr
    getTag() const
    {
        return tag;
    }

    /**
     * Checks if the given tag information corresponds to this entry's.
     *
     * @param tag The tag value to compare to.
     * @return True if the tag information match this entry's.
     */
    virtual bool
    matchTag(const Addr tag) const
    {
        return isValid() && (getTag() == tag);
    }

    /**
     * Insert the block by assigning it a tag and marking it valid. Touches
     * block if it hadn't been touched previously.
     *
     * @param tag The tag value.
     */
    virtual void
    insert(const Addr tag)
    {
        setValid();
        setTag(tag);
    }

    /** Invalidate the block. Its contents are no longer valid. */
    virtual void
    invalidate()
    {
        valid = false;
        setTag(MaxAddr);
    }

    std::string
    print() const override
    {
        return csprintf("tag: %#x valid: %d | %s", getTag(), isValid(),
                        ReplaceableEntry::print());
    }

  protected:
    /**
     * Set tag associated to this block.
     *
     * @param tag The tag value.
     */
    virtual void
    setTag(Addr _tag)
    {
        tag = _tag;
    }

    /** Set valid bit. The block must be invalid beforehand. */
    virtual void
    setValid()
    {
        assert(!isValid());
        valid = true;
    }

  private:
    /**
     * Valid bit. The contents of this entry are only valid if this bit is set.
     * @sa invalidate()
     * @sa insert()
     */
    bool valid{ false };

    /** The entry's tag. */
    Addr tag{ MaxAddr };
};

} // namespace gem5

#endif //__CACHE_ENTRY_HH__
