/**
 * Copyright (c) 2026
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
 * Declaration of EMISSARY-aware LRU replacement policy.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_LRU_EMISSARY_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_LRU_EMISSARY_RP_HH__

#include <cstdint>
#include <memory>

#include "mem/cache/cache_blk.hh"
#include "mem/cache/replacement_policies/base.hh"

namespace gem5
{

struct LRUEmissaryRPParams;

namespace replacement_policy
{

class LRUEmissary : public Base
{
  protected:
    struct LRUEmissaryReplData : ReplacementData
    {
        Tick lastTouchTick;
        CacheBlk *blk;

        explicit LRUEmissaryReplData(CacheBlk *blk)
          : lastTouchTick(0), blk(blk)
        {}
    };

  public:
    using Params = LRUEmissaryRPParams;

    int lru_ways;
    int preserve_ways;
    uint64_t last_tick;
    int numSets;
    int numWays;
    uint64_t flush_freq_in_cycles;
    uint32_t max_age;
    TaggedIndexingPolicy *indexingPolicy;

    explicit LRUEmissary(const Params &p);
    ~LRUEmissary() = default;

    void invalidate(
        const std::shared_ptr<ReplacementData>& replacement_data) override;
    void touch(
        const std::shared_ptr<ReplacementData>& replacement_data) const
        override;
    void reset(
        const std::shared_ptr<ReplacementData>& replacement_data) const
        override;
    ReplaceableEntry* getVictim(
        const ReplacementCandidates& candidates) const override;

    std::shared_ptr<ReplacementData> instantiateEntry() override;
    std::shared_ptr<ReplacementData> instantiateEntry(CacheBlk *blk);

    void dumpPreserveHist();
    void checkToFlushPreserveBits();

  private:
    void checkLRU(const std::shared_ptr<ReplacementData>& replacement_data) const;
    void resetAll(const ReplacementCandidates& candidates, bool preservedWays) const;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_LRU_EMISSARY_RP_HH__
