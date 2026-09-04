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

#include "mem/cache/prefetch/mlop.hh"

#include <algorithm>

#include "base/logging.hh"
#include "debug/HWPrefetch.hh"
#include "params/MLOPPrefetcher.hh"

namespace gem5
{
namespace prefetch
{

MLOP::MLOP(const MLOPPrefetcherParams &p)
    : Queued(p),
      evalPeriod(p.evaluation_period),
      lookaheadLevels(p.lookahead_levels),
      maxOffset(p.max_offset),
      scoreThreshold(p.score_threshold),
      prefetchDegree(p.prefetch_degree),
      amtEntries(p.amt_entries),
      bitVectorSize(p.bit_vector_size),
      recentDepth(lookaheadLevels > 0 ? lookaheadLevels - 1 : 0),
      regionMask(Addr(bitVectorSize - 1)),
      amt(amtEntries)
{
    if (amtEntries == 0) {
        fatal("%s: amt_entries must be > 0\n", name());
    }
    if (lookaheadLevels == 0) {
        fatal("%s: lookahead_levels must be > 0\n", name());
    }
    if (!isPowerOf2(bitVectorSize) || bitVectorSize > 64) {
        fatal("%s: bit_vector_size (%u) must be a power of two no greater "
              "than 64\n",
              name(), bitVectorSize);
    }
    if (maxOffset <= 0) {
        fatal("%s: max_offset must be > 0 (got %d)\n", name(), maxOffset);
    }
    if (maxOffset >= int(bitVectorSize)) {
        fatal("%s: max_offset (%d) must be < bit_vector_size (%u)\n", name(),
              maxOffset, bitVectorSize);
    }
    if (recentDepth > 0 && recentDepth >= bitVectorSize) {
        fatal("%s: lookahead_levels (%u) implies recentDepth=%u, which "
              "must be < bit_vector_size (%u)\n",
              name(), lookaheadLevels, recentDepth, bitVectorSize);
    }

    // Offsets from -maxOffset to +maxOffset, excluding 0.
    for (int o = -maxOffset; o <= maxOffset; o++) {
        if (o == 0) {
            continue;
        }
        OffsetEntry e;
        e.offset = o;
        e.scores.assign(lookaheadLevels, 0);
        offsetTable.emplace(o, std::move(e));
    }
}

void
MLOP::resetScores()
{
    for (auto &kv : offsetTable) {
        std::fill(kv.second.scores.begin(), kv.second.scores.end(), 0);
    }
}

MLOP::AMTEntry &
MLOP::findOrAllocAmtEntry(Addr baseBlock)
{
    int freeIdx = -1;
    int lruIdx = -1;
    uint64_t lruTouch = UINT64_MAX;

    for (unsigned i = 0; i < amt.size(); i++) {
        AMTEntry &e = amt[i];
        if (e.valid && e.baseBlock == baseBlock) {
            e.lastTouch = ++amtClock;
            return e;
        }
        if (!e.valid && freeIdx < 0) {
            freeIdx = (int)i;
        }
        if (e.valid && e.lastTouch < lruTouch) {
            lruTouch = e.lastTouch;
            lruIdx = (int)i;
        }
    }

    AMTEntry &victim = amt[(freeIdx >= 0) ? freeIdx : lruIdx];
    victim.valid = true;
    victim.baseBlock = baseBlock;
    victim.bitVector = 0;
    victim.recent.clear();
    victim.recent.reserve(recentDepth);
    victim.lastTouch = ++amtClock;
    return victim;
}

void
MLOP::updateScoresWithAccess(Addr block)
{
    const Addr baseBlock = block & ~regionMask;
    const unsigned idx = unsigned(block & regionMask);

    AMTEntry &entry = findOrAllocAmtEntry(baseBlock);

    // For each level L, credit every offset that would have predicted
    // this access after excluding the last (L-1) accesses.
    for (unsigned exclude = 0; exclude <= lookaheadLevels - 1; exclude++) {
        uint64_t masked = entry.bitVector;
        for (unsigned r = 0;
             r < std::min<unsigned>(exclude, entry.recent.size()); r++) {
            masked &= ~(1ULL << entry.recent[r]);
        }

        uint64_t bits = masked;
        while (bits) {
            const unsigned j = findLsbSet(bits);
            bits &= (bits - 1);

            const int k = int(idx) - int(j);
            if (k == 0 || k < -maxOffset || k > maxOffset) {
                continue;
            }
            auto it = offsetTable.find(k);
            if (it != offsetTable.end()) {
                it->second.scores[exclude]++;
            }
        }
    }

    if (recentDepth > 0) {
        entry.recent.insert(entry.recent.begin(), uint8_t(idx));
        if (entry.recent.size() > recentDepth) {
            entry.recent.resize(recentDepth);
        }
    }

    entry.bitVector |= (1ULL << idx);
}

void
MLOP::selectBestOffsets()
{
    bestOffsets.clear();

    // Keep every tied top-scorer per level, process longest lookahead
    // first so a claimed offset isn't reissued at a shorter one.
    std::vector<bool> used(2 * maxOffset + 1, false);
    std::vector<std::vector<const OffsetEntry *>> perLevel(lookaheadLevels);

    for (unsigned L = lookaheadLevels; L >= 1; L--) {
        uint32_t bestScore = 0;
        for (auto &kv : offsetTable) {
            bestScore = std::max(bestScore, kv.second.scores[L - 1]);
        }
        if (bestScore < scoreThreshold) {
            continue;
        }

        std::vector<const OffsetEntry *> &winners = perLevel[L - 1];
        for (auto &kv : offsetTable) {
            const OffsetEntry &e = kv.second;
            if (e.scores[L - 1] == bestScore && !used[e.offset + maxOffset]) {
                winners.push_back(&e);
            }
        }
        for (const OffsetEntry *e : winners) {
            used[e->offset + maxOffset] = true;
        }
    }

    // Emit in increasing-lookahead order (L=1 first) so the existing
    // issue-order priority in calculatePrefetch is preserved.
    for (unsigned L = 1; L <= lookaheadLevels; L++) {
        for (const OffsetEntry *e : perLevel[L - 1]) {
            bestOffsets.emplace_back(L, e);
        }
    }
}

void
MLOP::calculatePrefetch(const PrefetchInfo &pfi,
                        std::vector<AddrPriority> &addresses,
                        const CacheAccessor &cache)
{
    const Addr addr = pfi.getAddr();
    const Addr block = addr >> lBlkSize;

    updateScoresWithAccess(block);
    accessCounter++;

    if (accessCounter >= evalPeriod) {
        selectBestOffsets();
        resetScores();
        accessCounter = 0;
    }

    if (bestOffsets.empty()) {
        return;
    }

    // Issue in increasing lookahead order, L=1 first: it needs to arrive
    // soonest, so it gets the highest queue priority.
    unsigned issued = 0;
    for (const auto &p : bestOffsets) {
        if (issued >= prefetchDegree) {
            break;
        }

        const unsigned L = p.first;
        const OffsetEntry *e = p.second;
        const Addr pfAddr = addr + (Addr(e->offset) << lBlkSize);

        if (!samePage(addr, pfAddr)) {
            continue;
        }

        const int32_t prio = int32_t(lookaheadLevels - L);
        addresses.emplace_back(pfAddr, prio);
        issued++;
    }
}

} // namespace prefetch
} // namespace gem5
