/*
 * Copyright (c) 2020 ARM Limited
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
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

/*
 * Common base class for a machine node.
 */

#ifndef __MEM_RUBY_SLICC_INTERFACE_ABSTRACTCACHEENTRY_HH__
#define __MEM_RUBY_SLICC_INTERFACE_ABSTRACTCACHEENTRY_HH__

#include <iostream>

#include "base/logging.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/protocol/AccessPermission.hh"

namespace gem5
{

namespace ruby
{

class RubySystem;

class AbstractCacheEntry : public ReplaceableEntry
{
  private:
    // The last access tick for the cache entry.
    Tick m_last_touch_tick;

  public:
    AbstractCacheEntry();
    virtual ~AbstractCacheEntry() = 0;

    // Get/Set permission of the entry
    AccessPermission getPermission() const;
    void changePermission(AccessPermission new_perm);

    using ReplaceableEntry::print;
    virtual void print(std::ostream& out) const = 0;

    // The methods below are those called by ruby runtime, add when it
    // is absolutely necessary and should all be virtual function.
    [[noreturn]] virtual DataBlock&
    getDataBlk()
    {
        panic("getDataBlk() not implemented!");
    }

    virtual void initBlockSize(int block_size) { };
    virtual void setRubySystem(RubySystem *rs) { };

    int validBlocks;
    virtual int& getNumValidBlocks()
    {
        return validBlocks;
    }

    // Functions for locking and unlocking the cache entry.  These are required
    // for supporting atomic memory accesses.
    void setLocked(int context);
    void clearLocked();
    bool isLocked(int context) const;

    // Address of this block, required by CacheMemory
    Addr m_Address;
    // Holds info whether the address is locked.
    // Required for implementing LL/SC operations.
    int m_locked;

    AccessPermission m_Permission; // Access permission for this
                                   // block, required by CacheMemory

    // Get the last access Tick.
    Tick getLastAccess() { return m_last_touch_tick; }

    // Set the last access Tick.
    void setLastAccess(Tick tick) { m_last_touch_tick = tick; }

    // hardware transactional memory
    void setInHtmReadSet(bool val);
    void setInHtmWriteSet(bool val);
    bool getInHtmReadSet() const;
    bool getInHtmWriteSet() const;
    virtual void invalidateEntry() {}

  private:
    // hardware transactional memory
    bool m_htmInReadSet;
    bool m_htmInWriteSet;
};

inline std::ostream&
operator<<(std::ostream& out, const AbstractCacheEntry& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_SLICC_INTERFACE_ABSTRACTCACHEENTRY_HH__
