/**
 * Copyright (c) 2018 Metempsy Technology Consulting
 * Copyright (c) 2026 The Regents of The Xi'an Jiaotong University
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

#ifndef __MEM_CACHE_PREFETCH_INDIRECT_MEMORY_V2_HH__
#define __MEM_CACHE_PREFETCH_INDIRECT_MEMORY_V2_HH__

#include <deque>
#include <unordered_map>
#include <vector>

#include "base/cache/associative_cache.hh"
#include "base/sat_counter.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/tags/tagged_entry.hh"

namespace gem5
{

struct IndirectMemoryPrefetcherV2Params;

namespace prefetch
{

class IndirectMemoryV2 : public Queued
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
    /** multi-level and multi-way model controllor switch */
    const bool use_multi_way;
    const bool use_multi_level;

    const int maxIndirectWay;
    const int maxIndirectLevel;
    const int maxPending;
    const int maxReadyIndices;

    /** Prefetch Table Entry */
    struct PrefetchTableEntry : public TaggedEntry
    {
        /* Stream table fields */

        /** Accessed address */
        Addr address;
        /** Whether this address is in the secure region */
        bool secure;
        /** Index element size for this stream (Sniper data_length: 4 or 8). */
        unsigned dataSize;
        /** Confidence counter of the stream */
        unsigned int streamCounter;
        /** last address delta observed from this PC stream (±dataSize). */
        int64_t lastDelta;

        /** backoff confidence to avoid trashing ipd*/
        unsigned currHit;
        unsigned numHitNextDetect;

        /* Indirect table fields (Sniper IndirectTableEntry) */

        /** Pattern found (Sniper: indirect_pattern_on). Alone does not issue
         * PF. */
        bool enabled;
        /** Confirmed enough to issue indirect PF (Sniper: prefetch_on). */
        bool prefetchOn;
        /** Current index value */
        int64_t index;
        /** BaseAddr detected */
        Addr baseAddr;
        /** Shift detected */
        int shift;
        /**
         * Sniper indirect_hit_count. Stored in SatCounter; updates follow
         * insertIndex / isIndirectHit, not paper-style "match last index".
         */
        SatCounter8 indirectCounter;
        /** Indices seen but not yet matched by an indirect demand access. */
        static constexpr unsigned MaxUnmatchedIndices = 4;
        std::deque<std::pair<int64_t, bool>> unmatchedIndices;
        /**
         * Sniper curr_prefetch_distance: how far ahead to read B[i+d].
         * Grows by 1 per successful issue opportunity, capped at
         * maxPrefetchDistance; each B[i] issues at most 1–2 indirect PFs.
         */
        unsigned currPrefetchDistance;

        /** Optimization field */
        /** entry type: 1.primary, 2.multi_way or 3.multi_level */
        int type;

        /** entry numbers of children*/
        PrefetchTableEntry *next_way;
        PrefetchTableEntry *next_level;
        int way_depth;
        int level_depth;

        /** parent entry */
        PrefetchTableEntry *prev;

        PrefetchTableEntry(unsigned indirect_counter_bits, TagExtractor ext)
            : TaggedEntry(),
              address(0),
              secure(false),
              dataSize(0),
              streamCounter(0),
              lastDelta(0),
              currHit(0),
              numHitNextDetect(0),
              enabled(false),
              prefetchOn(false),
              index(0),
              baseAddr(0),
              shift(0),
              indirectCounter(indirect_counter_bits),
              unmatchedIndices{},
              currPrefetchDistance(1),
              type(0),
              next_way(nullptr),
              next_level(nullptr),
              way_depth(1),
              level_depth(1),
              prev(nullptr)
        {
            registerTagExtractor(ext);
        }

        void
        invalidate() override
        {
            TaggedEntry::invalidate();
            address = 0;
            secure = false;
            dataSize = 0;
            streamCounter = 0;
            lastDelta = 0;
            currHit = 0;
            numHitNextDetect = 0;
            enabled = false;
            prefetchOn = false;
            index = 0;
            baseAddr = 0;
            shift = 0;
            indirectCounter.reset();
            unmatchedIndices.clear();
            currPrefetchDistance = 1;
            type = 0;

            if (next_way) {
                next_way->invalidate();
            }
            if (next_level) {
                next_level->invalidate();
            }
            if (prev) {
                if (prev->next_way == this) {
                    prev->next_way = nullptr;
                }
                if (prev->next_level == this) {
                    prev->next_level = nullptr;
                }
            }
            next_way = nullptr;
            next_level = nullptr;
            way_depth = 1;
            level_depth = 1;
            prev = nullptr;
        }

        /**
         * Soft stop: turn off issuing but keep (base, shift, enabled) so IPD
         * does not retrain the same pattern (unlike Sniper full clear).
         */
        void
        disablePrefetchOn()
        {
            prefetchOn = false;
            indirectCounter.reset();
            currPrefetchDistance = 1;
        }

        /** Full drop of indirect state (PT invalidate / rare hard reset). */
        void
        clearIndirectPattern()
        {
            enabled = false;
            prefetchOn = false;
            index = 0;
            baseAddr = 0;
            shift = 0;
            indirectCounter.reset();
            unmatchedIndices.clear();
            currPrefetchDistance = 1;
        }
    };
    /** Prefetch table */
    AssociativeCache<PrefetchTableEntry> prefetchTable;

    /** Indirect Pattern Detector entrt */
    struct IndirectPatternDetectorEntry : public TaggedEntry
    {
        /** First index */
        int64_t idx1;
        /** Second index */
        int64_t idx2;
        /** how many index has been set */
        int numIndices;
        /** Number of misses currently recorded */
        int numMisses;

        int detect_type;
        /** Owning PT entry (not the AssociativeCache tag). */
        PrefetchTableEntry *pattern_entry;

        /** track or not */
        // bool ifTrack;
        /**
         * Potential BaseAddr candidates for each recorded miss.
         * The number of candidates per miss is determined by the number of
         * elements in the shiftValues array.
         */
        std::vector<std::vector<Addr>> baseAddr;

        IndirectPatternDetectorEntry(unsigned int num_addresses,
                                     unsigned int num_shifts, TagExtractor ext)
            : TaggedEntry(),
              idx1(0),
              idx2(0),
              numIndices(0),
              numMisses(0),
              detect_type(0),
              pattern_entry(nullptr),
              baseAddr(num_addresses, std::vector<Addr>(num_shifts))
        {
            registerTagExtractor(ext);
        }

        void
        invalidate() override
        {
            TaggedEntry::invalidate();
            idx1 = 0;
            idx2 = 0;
            numIndices = 0;
            numMisses = 0;
            detect_type = 0;
            pattern_entry = nullptr;
        }
    };
    /** Indirect Pattern Detector (IPD) table */
    AssociativeCache<IndirectPatternDetectorEntry> ipd;

    struct PendingIndex
    {
        Addr indexaddr;
        Addr streamPc;
        Addr baseAddr;
        int shift;
        bool secure;
        unsigned dataSize;
        unsigned depth;
    };
    std::unordered_multimap<Addr, PendingIndex> pendingIndices;
    struct ReadyIndirectPrefetch
    {
        Addr target;
        bool secure;
    };
    std::vector<ReadyIndirectPrefetch> readyIndirectPrefetches;

    /** Byte order used to access the cache */
    const ByteOrder byteOrder;

    /** Sniper-style address helpers (support negative shift). */
    static Addr getBaseAddress(Addr address, int64_t index, int shift);
    static Addr getIndirectAddress(Addr base_address, int64_t index,
                                   int shift);

    /**
     * Sniper index-stream: size 4/8 and delta exactly ±size.
     */
    static bool isIndexStreamAccess(unsigned size, int64_t delta);

    /**
     * Sniper insertIndex: enqueue index; if queue already full, slide and
     * penalize indirectCounter / maybe disable prefetchOn.
     */
    void insertIndex(PrefetchTableEntry &entry, int64_t index);

    /**
     * Sniper isIndirectHit: match addr against unmatched_indices, update
     * indirectCounter and prefetchOn. Returns true on hit.
     */
    bool isIndirectHit(PrefetchTableEntry &entry, Addr addr);

    /** Read a 4/8-byte index value from cache; false if unavailable. */
    bool tryReadIndexValue(const CacheAccessor &cache, Addr addr,
                           bool is_secure, unsigned data_size,
                           int64_t &index) const;

    /**
     * Sniper next_level chain: A = f(B), Aval = *A.
     * If do_insert, insertIndex(next_level, Aval).
     * If addresses != nullptr and next_level->prefetchOn, emit C.
     */
    bool processNextLevel(PrefetchTableEntry &pt_entry, int64_t b_index,
                          bool is_secure, const CacheAccessor &cache,
                          bool do_insert,
                          std::vector<AddrPriority> *addresses);

    bool enqueuePendingIndex(const PendingIndex &pending);

    bool enqueueReadyIndirectPrefetch(Addr target, bool secure);
    /**
     * For every pattern-on PT entry, try Sniper-style confirm on this access.
     */
    void checkAccessMatchOnActiveEntries(const PrefetchInfo &pfi,
                                         const CacheAccessor &cache);

    void trackMissIndex1(IndirectPatternDetectorEntry &entry, Addr addr);

    bool trackMissIndex2(IndirectPatternDetectorEntry &entry, Addr addr);

    void establishIndirectPattern(PrefetchTableEntry *pt_entry, int shift,
                                  Addr base);

  public:
    IndirectMemoryV2(const IndirectMemoryPrefetcherV2Params &p);
    ~IndirectMemoryV2() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;

    void notifyFill(const CacheAccessProbeArg &acc) override;
    bool
    allocateEmptyPrefetchTableEntry(
        AssociativeCache<PrefetchTableEntry> &table,
        PrefetchTableEntry *&empty_entry)
    {
        for (auto &e : table) {
            if (!e.isValid()) {
                empty_entry = &e;
                return true;
            }
        }
        return false;
    }

    bool
    allocateEmptyIPDEntry(
        AssociativeCache<IndirectPatternDetectorEntry> &table,
        IndirectPatternDetectorEntry *&empty_entry)
    {
        for (auto &e : table) {
            if (!e.isValid()) {
                empty_entry = &e;
                return true;
            }
        }
        return false;
    }

    /** In-flight IPD trainer for this PT entry and detect_type, if any. */
    IndirectPatternDetectorEntry *
    findIPDEntry(PrefetchTableEntry *pt, int detect_type)
    {
        for (auto &e : ipd) {
            if (e.isValid() && e.pattern_entry == pt &&
                e.detect_type == detect_type) {
                return &e;
            }
        }
        return nullptr;
    }
};

} // namespace prefetch
} // namespace gem5

#endif //__MEM_CACHE_PREFETCH_INDIRECT_MEMORY_HH__
