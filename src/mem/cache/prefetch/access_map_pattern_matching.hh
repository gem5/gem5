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
  * Implementation of the Access Map Pattern Matching Prefetcher
  *
  * References:
  *     Access map pattern matching for high performance data cache prefetch.
  *     Ishii, Y., Inaba, M., & Hiraki, K. (2011).
  *     Journal of Instruction-Level Parallelism, 13, 1-24.
  */

#ifndef __MEM_CACHE_PREFETCH_ACCESS_MAP_PATTERN_MATCHING_HH__
#define __MEM_CACHE_PREFETCH_ACCESS_MAP_PATTERN_MATCHING_HH__

#include "mem/cache/prefetch/associative_set.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/packet.hh"
#include "sim/clocked_object.hh"

struct AccessMapPatternMatchingParams;
struct AMPMPrefetcherParams;

namespace Prefetcher {

class AccessMapPatternMatching : public ClockedObject
{
    /** Cacheline size used by the prefetcher using this object */
    const unsigned blkSize;
    /** Limit the stride checking to -limitStride/+limitStride */
    const unsigned limitStride;
    /** Maximum number of prefetch generated */
    const unsigned startDegree;
    /** Amount of memory covered by a hot zone */
    const uint64_t hotZoneSize;
    /** A prefetch coverage factor bigger than this is considered high */
    const double highCoverageThreshold;
    /** A prefetch coverage factor smaller than this is considered low */
    const double lowCoverageThreshold;
    /** A prefetch accuracy factor bigger than this is considered high */
    const double highAccuracyThreshold;
    /** A prefetch accuracy factor smaller than this is considered low */
    const double lowAccuracyThreshold;
    /** A cache hit ratio bigger than this is considered high */
    const double highCacheHitThreshold;
    /** A cache hit ratio smaller than this is considered low */
    const double lowCacheHitThreshold;
    /** Cycles in an epoch period */
    const Cycles epochCycles;
    /** Off chip memory latency to use for the epoch bandwidth calculation */
    const Tick offChipMemoryLatency;

    /** Data type representing the state of a cacheline in the access map */
    enum AccessMapState
    {
        AM_INIT,
        AM_PREFETCH,
        AM_ACCESS,
        AM_INVALID
    };

    /** AccessMapEntry data type */
    struct AccessMapEntry : public TaggedEntry
    {
        /** vector containing the state of the cachelines in this zone */
        std::vector<AccessMapState> states;

        AccessMapEntry(size_t num_entries)
          : TaggedEntry(), states(num_entries, AM_INIT)
        {
        }

        void
        invalidate() override
        {
            TaggedEntry::invalidate();
            for (auto &entry : states) {
                entry = AM_INIT;
            }
        }
    };
    /** Access map table */
    AssociativeSet<AccessMapEntry> accessMapTable;

    /**
     * Number of good prefetches
     * - State transitions from PREFETCH to ACCESS
     */
    uint64_t numGoodPrefetches;
    /**
     * Number of prefetches issued
     * - State transitions from INIT to PREFETCH
     */
    uint64_t numTotalPrefetches;
    /**
     * Number of raw cache misses
     * - State transitions from INIT or PREFETCH to ACCESS
     */
    uint64_t numRawCacheMisses;
    /**
     * Number of raw cache hits
     * - State transitions from ACCESS to ACCESS
     */
    uint64_t numRawCacheHits;
    /** Current degree */
    unsigned degree;
    /** Current useful degree */
    unsigned usefulDegree;

    /**
     * Given a target cacheline, this function checks if the cachelines
     * that follow the provided stride have been accessed. If so, the line
     * is considered a good candidate.
     * @param states vector containing the states of three contiguous hot zones
     * @param current target block (cacheline)
     * @param stride access stride to obtain the reference cachelines
     * @return true if current is a prefetch candidate
     */
    inline bool checkCandidate(std::vector<AccessMapState> const &states,
                        Addr current, int stride) const
    {
        enum AccessMapState tgt   = states[current - stride];
        enum AccessMapState s     = states[current + stride];
        enum AccessMapState s2    = states[current + 2 * stride];
        enum AccessMapState s2_p1 = states[current + 2 * stride + 1];
        return (tgt != AM_INVALID &&
                ((s == AM_ACCESS && s2 == AM_ACCESS) ||
                (s == AM_ACCESS && s2_p1 == AM_ACCESS)));
    }

    /**
     * Obtain an AccessMapEntry  from the AccessMapTable, if the entry is not
     * found a new one is initialized and inserted.
     * @param am_addr address of the hot zone
     * @param is_secure whether the address belongs to the secure memory area
     * @return the corresponding entry
     */
    AccessMapEntry *getAccessMapEntry(Addr am_addr, bool is_secure);

    /**
     * Updates the state of a block within an AccessMapEntry, also updates
     * the prefetcher metrics.
     * @param entry AccessMapEntry to update
     * @param block cacheline within the hot zone
     * @param state new state
     */
    void setEntryState(AccessMapEntry &entry, Addr block,
        enum AccessMapState state);

    /**
     * This event constitues the epoch of the statistics that keep track of
     * the prefetcher accuracy, when this event triggers, the prefetcher degree
     * is adjusted and the statistics counters are reset.
     */
    void processEpochEvent();
    EventFunctionWrapper epochEvent;

  public:
    AccessMapPatternMatching(const AccessMapPatternMatchingParams* p);
    ~AccessMapPatternMatching() = default;

    void startup() override;
    void calculatePrefetch(const Base::PrefetchInfo &pfi,
        std::vector<Queued::AddrPriority> &addresses);
};

class AMPM : public Queued
{
    AccessMapPatternMatching &ampm;
  public:
    AMPM(const AMPMPrefetcherParams* p);
    ~AMPM() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses) override;
};

} // namespace Prefetcher

#endif//__MEM_CACHE_PREFETCH_ACCESS_MAP_PATTERN_MATCHING_HH__
