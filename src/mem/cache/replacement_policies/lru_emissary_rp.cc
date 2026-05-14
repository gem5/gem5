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

#include "mem/cache/replacement_policies/lru_emissary_rp.hh"

#include <cassert>
#include <fstream>
#include <map>

#include "base/output.hh"
#include "base/trace.hh"
#include "debug/EMISSARY.hh"
#include "params/LRUEmissaryRP.hh"
#include "sim/cur_tick.hh"
#include "sim/core.hh"

namespace gem5
{

namespace replacement_policy
{

LRUEmissary::LRUEmissary(const Params &p)
    : Base(p),
      lru_ways(p.lru_ways),
      preserve_ways(p.preserve_ways),
      last_tick(0),
      numSets(0),
      numWays(0),
      flush_freq_in_cycles(p.flush_freq_in_cycles),
      max_age(p.max_val),
      indexingPolicy(nullptr)
{
    registerExitCallback([this]() { dumpPreserveHist(); });
}

void
LRUEmissary::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    std::static_pointer_cast<LRUEmissaryReplData>(
        replacement_data)->lastTouchTick = Tick(0);
}

void
LRUEmissary::checkLRU(
    const std::shared_ptr<ReplacementData>& replacement_data) const
{
    auto repl_data = std::static_pointer_cast<LRUEmissaryReplData>(
        replacement_data);
    if (!repl_data->blk || !indexingPolicy || numWays == 0) {
        return;
    }

    CacheBlk *cur_blk = repl_data->blk;
    const auto set = cur_blk->getSet();
    const bool preserve = cur_blk->isPreserve();

    DPRINTF(EMISSARY, "Age of set %d :", set);
    for (int way = 0; way < numWays; way++) {
        auto *entry = indexingPolicy->getEntry(set, way);
        auto *blk = static_cast<CacheBlk*>(entry);
        auto candidate_repl_data =
            std::static_pointer_cast<LRUEmissaryReplData>(blk->replacementData);
        if ((blk->isPreserve() == preserve) &&
            candidate_repl_data->lastTouchTick > 1) {
            candidate_repl_data->lastTouchTick--;
        }
        DPRINTFR(EMISSARY, " %d", candidate_repl_data->lastTouchTick);
        if (blk->isPreserve()) {
            DPRINTFR(EMISSARY, "(P)");
        }
    }
    DPRINTFR(EMISSARY, "\n");
}

void
LRUEmissary::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    const_cast<LRUEmissary*>(this)->checkToFlushPreserveBits();
    std::static_pointer_cast<LRUEmissaryReplData>(
        replacement_data)->lastTouchTick = max_age;
}

void
LRUEmissary::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    auto *non_const_this = const_cast<LRUEmissary*>(this);
    non_const_this->checkToFlushPreserveBits();
    checkLRU(replacement_data);
    std::static_pointer_cast<LRUEmissaryReplData>(
        replacement_data)->lastTouchTick = max_age;
}

void
LRUEmissary::resetAll(
    const ReplacementCandidates& candidates, bool preservedWays) const
{
    for (const auto& candidate : candidates) {
        auto *blk = static_cast<CacheBlk*>(candidate);
        if (preservedWays == blk->isPreserve()) {
            std::static_pointer_cast<LRUEmissaryReplData>(
                candidate->replacementData)->lastTouchTick = 0;
        }
    }
}

ReplaceableEntry*
LRUEmissary::getVictim(const ReplacementCandidates& candidates) const
{
    assert(!candidates.empty());
    const_cast<LRUEmissary*>(this)->checkToFlushPreserveBits();

    ReplaceableEntry *victim_not_preserved = candidates[0];
    ReplaceableEntry *preserved_victim = candidates[0];
    int num_not_preserved = 0;
    int num_preserved = 0;
    bool reset_preserved = true;
    bool reset_non_preserved = true;

    for (const auto& candidate : candidates) {
        auto repl =
            std::static_pointer_cast<LRUEmissaryReplData>(candidate->replacementData);
        if (repl->lastTouchTick == 0) {
            return candidate;
        }

        auto *blk = static_cast<CacheBlk*>(candidate);
        if (blk->isPreserve()) {
            num_preserved++;
            if (num_preserved == 1 ||
                repl->lastTouchTick <
                    std::static_pointer_cast<LRUEmissaryReplData>(
                        preserved_victim->replacementData)->lastTouchTick) {
                preserved_victim = candidate;
                reset_preserved = false;
            }
        } else {
            num_not_preserved++;
            if (num_not_preserved == 1 ||
                repl->lastTouchTick <
                    std::static_pointer_cast<LRUEmissaryReplData>(
                        victim_not_preserved->replacementData)->lastTouchTick) {
                victim_not_preserved = candidate;
                reset_non_preserved = false;
            }
        }
    }

    if (reset_preserved) {
        resetAll(candidates, true);
    }
    if (reset_non_preserved) {
        resetAll(candidates, false);
    }

    if (num_preserved > preserve_ways) {
        return preserved_victim;
    }
    return victim_not_preserved;
}

std::shared_ptr<ReplacementData>
LRUEmissary::instantiateEntry()
{
    return std::make_shared<LRUEmissaryReplData>(nullptr);
}

std::shared_ptr<ReplacementData>
LRUEmissary::instantiateEntry(CacheBlk *blk)
{
    return std::make_shared<LRUEmissaryReplData>(blk);
}

void
LRUEmissary::checkToFlushPreserveBits()
{
    if (!flush_freq_in_cycles || !indexingPolicy) {
        return;
    }

    const uint64_t cur_tick = curTick();
    if (((cur_tick - last_tick) / 500) >= flush_freq_in_cycles) {
        dumpPreserveHist();
        last_tick = cur_tick;
    }
}

void
LRUEmissary::dumpPreserveHist()
{
    if (!indexingPolicy || numSets <= 0 || numWays <= 0) {
        return;
    }

    std::ofstream histOut;
    histOut.open(simout.directory() + "/set_hist.csv", std::fstream::app);

    histOut << curTick() << ",";
    std::map<int, int> preserveCountHist;
    for (int i = 0; i < numWays; i++) {
        preserveCountHist[i] = 0;
    }

    for (int set = 0; set < numSets; set++) {
        int numPreserved = 0;
        for (int way = 0; way < numWays; way++) {
            auto *entry = indexingPolicy->getEntry(set, way);
            auto *blk = static_cast<CacheBlk*>(entry);
            if (blk->isPreserve()) {
                numPreserved++;
            }
            if (!blk->isUsed()) {
                blk->clearPreserve();
            }
            blk->clearUsed();
        }

        if (numPreserved >= preserve_ways) {
            preserveCountHist[preserve_ways]++;
        } else {
            preserveCountHist[numPreserved]++;
        }
    }

    for (int i = 0; i < numWays; i++) {
        histOut << preserveCountHist[i] << ",";
    }
    histOut << "\n";
}

} // namespace replacement_policy
} // namespace gem5
