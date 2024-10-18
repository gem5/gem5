/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __CPU_O3_STORE_SET_HH__
#define __CPU_O3_STORE_SET_HH__

#include <list>
#include <map>
#include <utility>
#include <vector>

#include "base/cache/associative_cache.hh"
#include "base/named.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "params/SSITIndexingPolicy.hh"
#include "params/SSITSetAssociative.hh"

namespace gem5
{

class SSITTagTypes
{
  public:
    struct KeyType
    {
        Addr address;
    };

    using Params = SSITIndexingPolicyParams;
};

using SSITIndexingPolicy = IndexingPolicyTemplate<SSITTagTypes>;
template class IndexingPolicyTemplate<SSITTagTypes>;

class SSITSetAssociative : public SSITIndexingPolicy
{
  public:
    PARAMS(SSITSetAssociative);
    using KeyType = SSITTagTypes::KeyType;

    SSITSetAssociative(const Params &p)
        : SSITIndexingPolicy(p, p.num_entries, p.set_shift)
    {
    }

  protected:
    /**
     * Extract the set index for the instruction PC based on tid.
     */
    uint32_t
    extractSet(const KeyType &key) const
    {
        return ((key.address >> setShift) & setMask);
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

    Addr regenerateAddr(const KeyType &key,
                        const ReplaceableEntry* entry) const override
    {
        panic("regenerateAddr() not implemented!");
        return 0;
    }
};

namespace o3
{

struct ltseqnum
{
    bool
    operator()(const InstSeqNum &lhs, const InstSeqNum &rhs) const
    {
        return lhs > rhs;
    }
};


/**
 * Implements a store set predictor for determining if memory
 * instructions are dependent upon each other.  See paper "Memory
 * Dependence Prediction using Store Sets" by Chrysos and Emer.  SSID
 * stands for Store Set ID, SSIT stands for Store Set ID Table, and
 * LFST is Last Fetched Store Table.
 */
class StoreSet : public Named
{
  public:
    typedef Addr SSID;

    class SSITEntry : public CacheEntry
    {
        SSID _ssid;
      public:
        using IndexingPolicy = gem5::SSITIndexingPolicy;
        using KeyType = SSITTagTypes::KeyType;
        using TagExtractor = std::function<Addr(Addr)>;

        SSITEntry(TagExtractor ext) : CacheEntry(ext), _ssid(MaxAddr) {}

        bool match(const KeyType &key) { return match(key.address); }
        void insert(const KeyType &key) { return insert(key.address); }

        void setSSID(SSID id) { _ssid = id; }
        SSID getSSID(void) const { return _ssid; }
      private:
        using CacheEntry::match;
        using CacheEntry::insert;
    };

    /** Default constructor.  init() must be called prior to use. */
    StoreSet() : Named("StoreSets"), SSIT("SSIT") {};

    /** Creates store set predictor with given table sizes. */
    StoreSet(std::string name, uint64_t clear_period,
             size_t SSIT_entries, int SSIT_assoc,
             replacement_policy::Base *replPolicy,
             SSITIndexingPolicy *indexingPolicy, int LFST_size);

    /** Default destructor. */
    ~StoreSet();

    /** Initializes the store set predictor with the given table sizes. */
    void init(uint64_t clear_period,
              size_t SSIT_entries, int SSIT_assoc,
              replacement_policy::Base *_replPolicy,
              SSITIndexingPolicy *_indexingPolicy, int LFST_size);

    /** Records a memory ordering violation between the younger load
     * and the older store. */
    void violation(Addr store_PC, Addr load_PC);

    /** Clears the store set predictor every so often so that all the
     * entries aren't used and stores are constantly predicted as
     * conflicting.
     */
    void checkClear();

    /** Inserts a load into the store set predictor.  This does nothing but
     * is included in case other predictors require a similar function.
     */
    void insertLoad(Addr load_PC, InstSeqNum load_seq_num);

    /** Inserts a store into the store set predictor.  Updates the
     * LFST if the store has a valid SSID. */
    void insertStore(Addr store_PC, InstSeqNum store_seq_num, ThreadID tid);

    /** Checks if the instruction with the given PC is dependent upon
     * any store.  @return Returns the sequence number of the store
     * instruction this PC is dependent upon.  Returns 0 if none.
     */
    InstSeqNum checkInst(Addr PC);

    /** Records this PC/sequence number as issued. */
    void issued(Addr issued_PC, InstSeqNum issued_seq_num, bool is_store);

    /** Squashes for a specific thread until the given sequence number. */
    void squash(InstSeqNum squashed_num, ThreadID tid);

    /** Resets all tables. */
    void clear();

    /** Debug function to dump the contents of the store list. */
    void dump();

  private:
    /** Calculates a Store Set ID based on the PC. */
    inline SSID calcSSID(Addr PC)
    { return ((PC ^ (PC >> 10)) % LFSTSize); }

    /** The Store Set ID Table. */
    AssociativeCache<SSITEntry> SSIT;

    /** Last Fetched Store Table. */
    std::vector<InstSeqNum> LFST;

    /** Bit vector to tell if the LFST has a valid entry. */
    std::vector<bool> validLFST;

    /** Map of stores that have been inserted into the store set, but
     * not yet issued or squashed.
     */
    std::map<InstSeqNum, int, ltseqnum> storeList;

    typedef std::map<InstSeqNum, int, ltseqnum>::iterator SeqNumMapIt;

    /** Number of loads/stores to process before wiping predictor so all
     * entries don't get saturated
     */
    uint64_t clearPeriod;

    /** Store Set ID Table size, in entries. */
    int SSITSize;

    /** Last Fetched Store Table size, in entries. */
    int LFSTSize;

    /** Number of memory operations predicted since last clear of predictor */
    int memOpsPred;
};

} // namespace o3

/**
 * This helper generates a tag extractor function object
 * which will be typically used by Replaceable entries indexed
 * with the SSITIndexingPolicy.
 * It allows to "decouple" indexing from tagging. Those entries
 * would call the functor without directly holding a pointer
 * to the indexing policy which should reside in the cache.
 */
static constexpr auto
genTagExtractor(SSITIndexingPolicy *ip)
{
    return [ip] (Addr addr) { return ip->extractTag(addr); };
}

} // namespace gem5

#endif // __CPU_O3_STORE_SET_HH__
