/*
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
#include "mem/protocol/AccessPermission.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/slicc_interface/AbstractEntry.hh"

class DataBlock;

class AbstractCacheEntry : public AbstractEntry
{
  public:
    AbstractCacheEntry();
    virtual ~AbstractCacheEntry() = 0;

    // Get/Set permission of the entry
    void changePermission(AccessPermission new_perm);

    // The methods below are those called by ruby runtime, add when it
    // is absolutely necessary and should all be virtual function.
    virtual DataBlock& getDataBlk()
    { panic("getDataBlk() not implemented!"); }

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

    void setSetIndex(uint32_t s) { m_set_index = s; }
    uint32_t getSetIndex() const { return m_set_index; }

    void setWayIndex(uint32_t s) { m_way_index = s; }
    uint32_t getWayIndex() const { return m_way_index; }

    // Address of this block, required by CacheMemory
    Addr m_Address;
    // Holds info whether the address is locked.
    // Required for implementing LL/SC operations.
    int m_locked;

  private:
    // Set and way coordinates of the entry within the cache memory object.
    uint32_t m_set_index;
    uint32_t m_way_index;
};

inline std::ostream&
operator<<(std::ostream& out, const AbstractCacheEntry& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_SLICC_INTERFACE_ABSTRACTCACHEENTRY_HH__
