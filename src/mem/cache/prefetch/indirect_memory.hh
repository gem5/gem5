/**
 * Copyright (c) 2018 Metempsy Technology Consulting
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
 * Implementation of the Indirect Memory Prefetcher
 *
 * References:
 * IMP: Indirect memory prefetcher.
 * Yu, X., Hughes, C. J., Satish, N., & Devadas, S. (2015, December).
 * In Proceedings of the 48th International Symposium on Microarchitecture
 * (pp. 178-190). ACM.
 */

#ifndef __MEM_CACHE_PREFETCH_INDIRECT_MEMORY_HH__
#define __MEM_CACHE_PREFETCH_INDIRECT_MEMORY_HH__

#include <vector>

#include "base/sat_counter.hh"
#include "mem/cache/prefetch/associative_set.hh"
#include "mem/cache/prefetch/queued.hh"

namespace gem5
{

struct IndirectMemoryPrefetcherParams;

namespace prefetch
{

class IndirectMemory : public Queued
{
    /** Maximum number of prefetches generated per event */
    const unsigned int maxPrefetchDistance;
    /** Shift values considered */
    const std::vector<int> shiftValues;
    /** Counter threshold to start prefetching */
    const unsigned int prefetchThreshold;
    /** streamCounter value to trigger the streaming prefetcher */
    const int streamCounterThreshold;
    /** Number of prefetches generated when using the streaming prefetcher */
    const int streamingDistance;

    /** Prefetch Table Entry */
    struct PrefetchTableEntry : public TaggedEntry
    {
        /* Stream table fields */

        /** Accessed address */
        Addr address;
        /** Whether this address is in the secure region */
        bool secure;
        /** Confidence counter of the stream */
        unsigned int streamCounter;

        /* Indirect table fields */

        /** Enable bit of the indirect fields */
        bool enabled;
        /** Current index value */
        int64_t index;
        /** BaseAddr detected */
        Addr baseAddr;
        /** Shift detected */
        int shift;
        /** Confidence counter of the indirect fields */
        SatCounter8 indirectCounter;
        /**
         * This variable is set to indicate that there has been at least one
         * match with the current index value. This information is later used
         * when a new index is updated. If there were no increases in the
         * indirectCounter, the counter is decremented.
         */
        bool increasedIndirectCounter;

        PrefetchTableEntry(unsigned indirect_counter_bits)
            : TaggedEntry(),
              address(0),
              secure(false),
              streamCounter(0),
              enabled(false),
              index(0),
              baseAddr(0),
              shift(0),
              indirectCounter(indirect_counter_bits),
              increasedIndirectCounter(false)
        {}

        void
        invalidate() override
        {
            TaggedEntry::invalidate();
            address = 0;
            secure = false;
            streamCounter = 0;
            enabled = false;
            index = 0;
            baseAddr = 0;
            shift = 0;
            indirectCounter.reset();
            increasedIndirectCounter = false;
        }
    };

    /** Prefetch table */
    AssociativeSet<PrefetchTableEntry> prefetchTable;

    /** Indirect Pattern Detector entrt */
    struct IndirectPatternDetectorEntry : public TaggedEntry
    {
        /** First index */
        int64_t idx1;
        /** Second index */
        int64_t idx2;
        /** Valid bit for the second index */
        bool secondIndexSet;
        /** Number of misses currently recorded */
        int numMisses;
        /**
         * Potential BaseAddr candidates for each recorded miss.
         * The number of candidates per miss is determined by the number of
         * elements in the shiftValues array.
         */
        std::vector<std::vector<Addr>> baseAddr;

        IndirectPatternDetectorEntry(unsigned int num_addresses,
                                     unsigned int num_shifts)
            : TaggedEntry(),
              idx1(0),
              idx2(0),
              secondIndexSet(false),
              numMisses(0),
              baseAddr(num_addresses, std::vector<Addr>(num_shifts))
        {}

        void
        invalidate() override
        {
            TaggedEntry::invalidate();
            idx1 = 0;
            idx2 = 0;
            secondIndexSet = false;
            numMisses = 0;
        }
    };

    /** Indirect Pattern Detector (IPD) table */
    AssociativeSet<IndirectPatternDetectorEntry> ipd;

    /** Entry currently tracking misses */
    IndirectPatternDetectorEntry *ipdEntryTrackingMisses;

    /** Byte order used to access the cache */
    const ByteOrder byteOrder;

    /**
     * Allocate or update an entry in the IPD
     * @param pt_entry Pointer to the associated page table entry
     * @param index Detected first index value
     */
    void allocateOrUpdateIPDEntry(const PrefetchTableEntry *pt_entry,
                                  int64_t index);
    /**
     * Update an IPD entry with a detected miss address, when the first index
     * is being tracked
     * @param miss_addr The address that caused the miss
     */
    void trackMissIndex1(Addr miss_addr);

    /**
     * Update an IPD entry with a detected miss address, when the second index
     * is being tracked
     * @param miss_addr The address that caused the miss
     */
    void trackMissIndex2(Addr miss_addr);

    /**
     * Checks if an access to the cache matches any active PT entry, if so,
     * the indirect confidence counter is incremented
     * @param addr address of the access
     */
    void checkAccessMatchOnActiveEntries(Addr addr);

  public:
    IndirectMemory(const IndirectMemoryPrefetcherParams &p);
    ~IndirectMemory() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;
};

} // namespace prefetch
} // namespace gem5

#endif //__MEM_CACHE_PREFETCH_INDIRECT_MEMORY_HH__
