/*
 * Copyright (c) 2020-2021, 2026 Arm Limited
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
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#ifndef __MEM_RUBY_STRUCTURES_CACHEMEMORY_HH__
#define __MEM_RUBY_STRUCTURES_CACHEMEMORY_HH__

#include <unordered_map>
#include <vector>

#include "base/cache/associative_cache.hh"
#include "base/cache/cache_entry.hh"
#include "base/statistics.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/protocol/CacheRequestType.hh"
#include "mem/ruby/protocol/CacheResourceType.hh"
#include "mem/ruby/slicc_interface/AbstractCacheEntry.hh"
#include "mem/ruby/structures/ALUFreeListArray.hh"
#include "mem/ruby/structures/BankedArray.hh"
#include "mem/ruby/system/CacheRecorder.hh"
#include "params/RubyCache.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace ruby
{

class CacheMemory : public SimObject
{
  public:
    PARAMS(RubyCache);
    CacheMemory(const Params &p);
    ~CacheMemory();

    void init();

    // Public Methods
    // perform a cache access and see if we hit or not.  Return true on a hit.
    bool tryCacheAccess(Addr address, RubyRequestType type,
                        DataBlock*& data_ptr);

    // similar to above, but doesn't require full access check
    bool testCacheAccess(Addr address, RubyRequestType type,
                         DataBlock*& data_ptr);

    // tests to see if an address is present in the cache
    bool isTagPresent(Addr address) const;

    // Returns true if there is:
    //   a) a tag match on this address or there is
    //   b) an unused line in the same cache "way"
    bool cacheAvail(Addr address) const;

    // Returns a NULL entry that acts as a placeholder for invalid lines
    AbstractCacheEntry*
    getNullEntry() const
    {
        return nullptr;
    }

    // find an unused entry and sets the tag appropriate for the address
    AbstractCacheEntry* allocate(Addr address, AbstractCacheEntry* new_entry);
    void allocateVoid(Addr address, AbstractCacheEntry* new_entry)
    {
        allocate(address, new_entry);
    }

    // Explicitly free up this address
    void deallocate(Addr address);

    // Explicitly free up this entry
    void deallocate(AbstractCacheEntry *entry);

    // Flush all entries within the cache.
    // Mainly used for debug purposes
    void flushEntries();

    // Returns with the physical address of the conflicting cache line
    Addr cacheProbe(Addr address) const;

    // looks an address up in the cache
    AbstractCacheEntry* lookup(Addr address);
    const AbstractCacheEntry* lookup(Addr address) const;

    Cycles getTagLatency() const { return tagArray.getLatency(); }
    Cycles getDataLatency() const { return dataArray.getLatency(); }

    // Hook for checkpointing the contents of the cache
    void recordCacheContents(int cntrl, CacheRecorder* tr) const;

    // Set this address to most recently used
    void setMRU(Addr address);
    void setMRU(Addr addr, int occupancy);
    void setMRU(AbstractCacheEntry *entry);

    // Functions for locking and unlocking cache lines corresponding to the
    // provided address.  These are required for supporting atomic memory
    // accesses.  These are to be used when only the address of the cache entry
    // is available.  In case the entry itself is available. use the functions
    // provided by the AbstractCacheEntry class.
    void setLocked (Addr addr, int context);
    void clearLocked (Addr addr);
    void clearLockedAll (int context);
    bool isLocked (Addr addr, int context);

    // Print cache contents
    void print(std::ostream& out) const;
    void printData(std::ostream& out) const;

    bool checkResourceAvailable(CacheResourceType res, Addr addr);
    void recordRequestType(CacheRequestType requestType, Addr addr);

    // hardware transactional memory
    void htmAbortTransaction();
    void htmCommitTransaction();

    void setRubySystem(RubySystem* rs);

  public:
    int getCacheSize() const { return m_cache_size; }
    int getCacheAssoc() const { return m_cache_assoc; }
    int getNumBlocks() const { return m_cache_num_sets * m_cache_assoc; }
    Addr getAddressAtIdx(int idx) const;

  private:
    // convert a Address to its location in the cache
    int64_t addressToCacheSet(Addr address) const;

    // Private copy constructor and assignment operator
    CacheMemory(const CacheMemory& obj);
    CacheMemory& operator=(const CacheMemory& obj);

  private:
    /**
     * RubyCacheSlot provides an adapter between the protocol specific
     * requirements of Ruby cache entries - storage of access
     * time, cache-line addresses, permissions and SLICC fields within
     * derived classes using AbstractCacheEntry. While also meeting
     * the requirements of AssociativeCache using CacheEntry.
     */
    class RubyCacheSlot : public CacheEntry
    {
      public:
        explicit RubyCacheSlot(CacheEntry::TagExtractor tag_extractor)
            : CacheEntry(tag_extractor)
        {}

        bool
        isValid() const override
        {
            return CacheEntry::isValid() && entry != nullptr &&
                   entry->m_Permission != AccessPermission_NotPresent;
        }

        bool
        match(const Addr addr) const override
        {
            return isValid() && entry->m_Address == addr;
        }

        void
        invalidate() override
        {
            CacheEntry::invalidate();
            entry = nullptr;
        }

        AbstractCacheEntry *entry = nullptr;
    };

    /**
     * Ruby Cache needs to perform a couple extra functions to the classic
     * namely cache probing and weighted replacement policy. Therefore we
     * extend associative cache to enable this.
     */
    class RubyCache : public AssociativeCache<RubyCacheSlot>
    {
      public:
        using AssociativeCache<RubyCacheSlot>::AssociativeCache;

        RubyCacheSlot *
        selectVictim(const std::vector<RubyCacheSlot *> &candidates) const;
        void weightedTouch(RubyCacheSlot *entry, int occupancy) const;
    };

    RubyCacheSlot *lookupSlot(Addr address);
    const RubyCacheSlot *lookupSlot(Addr address) const;

    // Data Members (m_prefix)
    bool m_is_instruction_only_cache;

    BankedArray dataArray;
    BankedArray tagArray;
    RubyCache rubyCache;
    ALUFreeListArray atomicALUArray;

    int m_cache_size;
    int m_cache_num_sets;
    int m_cache_num_set_bits;
    int m_cache_assoc;
    int m_start_index_bit;
    bool m_resource_stalls;
    int m_block_size;
    bool m_is_power_of_2_sets;

    /**
     * Set to true when using WeightedLRU replacement policy, otherwise, set to
     * false.
     */
    bool m_use_occupancy;

    RubySystem *m_ruby_system = nullptr;

    Addr
    makeLineAddress(Addr addr) const
    {
        return ruby::makeLineAddress(addr, floorLog2(m_block_size));
    }

    private:
      struct CacheMemoryStats : public statistics::Group
      {
          CacheMemoryStats(statistics::Group *parent);

          statistics::Scalar numDataArrayReads;
          statistics::Scalar numDataArrayWrites;
          statistics::Scalar numTagArrayReads;
          statistics::Scalar numTagArrayWrites;

          statistics::Scalar numTagArrayStalls;
          statistics::Scalar numDataArrayStalls;

          statistics::Scalar numAtomicALUOperations;
          statistics::Scalar numAtomicALUArrayStalls;

          // hardware transactional memory
          statistics::Histogram htmTransCommitReadSet;
          statistics::Histogram htmTransCommitWriteSet;
          statistics::Histogram htmTransAbortReadSet;
          statistics::Histogram htmTransAbortWriteSet;

          statistics::Scalar m_demand_hits;
          statistics::Scalar m_demand_misses;
          statistics::Formula m_demand_accesses;

          statistics::Scalar m_prefetch_hits;
          statistics::Scalar m_prefetch_misses;
          statistics::Formula m_prefetch_accesses;

          statistics::Vector m_accessModeType;
      } cacheMemoryStats;

    public:
      // These function increment the number of demand hits/misses by one
      // each time they are called
      void profileDemandHit();
      void profileDemandMiss();
      void profilePrefetchHit();
      void profilePrefetchMiss();
};

std::ostream& operator<<(std::ostream& out, const CacheMemory& obj);

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_CACHEMEMORY_HH__
