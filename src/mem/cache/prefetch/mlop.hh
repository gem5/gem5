/*
 * Copyright (c) 2026 Marco Frank, Erik Chao, and Matthew Mosher
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
 * @file
 * Implementation of the 'Multi-Lookahead Offset Prefetcher'
 * References:
 *   Shakerinava, M. et al. (2019, June). Multi-lookahead offset
 *   prefetching. In DPC3 (in conjunction with ISCA 2019).
 *   https://mshakerinava.github.io/papers/mlop-dpc3.pdf
 *
 *   The 4-page paper above omits some implementation details (e.g.
 *   score_threshold). Where this port needed one not covered by the
 *   paper, it was taken from the authors' DPC3 competition source
 *   submission, available alongside the contest's slides at
 *   https://dpc3.compas.cs.stonybrook.edu/?final_programs
 *
 *
 * Scores every candidate offset at every lookahead level (1..N) against
 * an Access Map Table of recently-touched regions, then issues one
 * prefetch per lookahead level using its best-scoring offset, ordered by
 * increasing lookahead so the soonest-needed prefetches are serviced
 * first. The paper places MLOP at the L1 data cache, trained only by
 * L1-D miss streams (see the on_miss/prefetch_on_access params in
 * Prefetcher.py).
 */

#ifndef __MEM_CACHE_PREFETCH_MLOP_HH__
#define __MEM_CACHE_PREFETCH_MLOP_HH__

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "base/cache/associative_cache.hh"
#include "base/types.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/tags/tagged_entry.hh"

namespace gem5
{

struct MLOPPrefetcherParams;

namespace prefetch
{

class MLOP : public Queued
{
  public:
    MLOP(const MLOPPrefetcherParams &p);
    ~MLOP() = default;

    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses,
                           const CacheAccessor &cache) override;

  private:
    /** A lookahead level, in [1, lookaheadLevels]. */
    using LookaheadLevel = unsigned;
    /** A signed displacement from the triggering block, in cache lines. */
    using Offset = int;

    /**
     * scores[L - 1] is an offset's score at lookahead level L. Indexed
     * within offsetTable by offset + maxOffset; the offset itself is
     * never stored, only derived from its position in that table.
     */
    using OffsetEntry = std::vector<uint32_t>;

    /** An Access Map Table entry, tagged by its region's base block. */
    struct AMTEntry : public TaggedEntry
    {
        AMTEntry(TagExtractor ext);

        void invalidate() override;

        uint64_t bitVector = 0;
        /** Most-recent-first access indices, size <= recentDepth */
        std::deque<uint8_t> recent;
    };
    using AMTCache = AssociativeCache<AMTEntry>;

    const unsigned evalPeriod = 0;
    const LookaheadLevel lookaheadLevels = 0;
    const Offset maxOffset = 0;
    const unsigned scoreThreshold = 0;
    const unsigned prefetchDegree = 0;
    const unsigned amtEntries = 0;
    const unsigned bitVectorSize = 0;
    const unsigned recentDepth = 0;
    const Addr regionMask = 0;

    AMTCache amt;

    /** Indexed by offset + maxOffset; the slot at index maxOffset
     * (offset 0) is left unused. */
    std::vector<OffsetEntry> offsetTable;
    /** (lookahead level, offset) pairs selected at the last epoch. */
    std::vector<std::pair<LookaheadLevel, Offset>> bestOffsets;
    unsigned accessCounter = 0;

    void resetScores();
    std::vector<std::pair<LookaheadLevel, Offset>> selectBestOffsets();

    AMTEntry &findOrAllocAmtEntry(Addr block);
    void updateScoresWithAccess(Addr block);
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_MLOP_HH__
