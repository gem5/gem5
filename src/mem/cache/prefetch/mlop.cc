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

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/HWPrefetch.hh"
#include "params/MLOPPrefetcher.hh"

namespace gem5
{
namespace prefetch
{

MLOP::AMTEntry::AMTEntry(TagExtractor ext)
{ registerTagExtractor(ext); }

void
MLOP::AMTEntry::invalidate()
{
    TaggedEntry::invalidate();
    bitVector = 0;
    recent.clear();
}

MLOP::MLOP(const MLOPPrefetcherParams &p)
    : Queued(p),
      evalPeriod(p.evaluation_period),
      lookaheadLevels(p.lookahead_levels),
      maxOffset(p.max_offset),
      scoreThreshold(p.score_threshold),
      prefetchDegree(p.degree),
      amtEntries(p.amt_entries),
      bitVectorSize(p.bit_vector_size),
      recentDepth(lookaheadLevels > 0 ? lookaheadLevels - 1 : 0),
      regionMask(Addr(bitVectorSize - 1)),
      amt("AMT", p.amt_entries, p.amt_assoc, p.amt_replacement_policy,
          p.amt_indexing_policy,
          AMTEntry(genTagExtractor(p.amt_indexing_policy))),
      offsetTable(2 * maxOffset + 1, OffsetEntry(lookaheadLevels, 0))
{
    if (amtEntries == 0) {
        fatal("%s: number of Access Map Table entries must be > 0\n", name());
    }
    if (lookaheadLevels == 0) {
        fatal("%s: number of lookahead levels must be > 0\n", name());
    }
    if (!isPowerOf2(bitVectorSize) || bitVectorSize > 64) {
        fatal("%s: bit vector size must be a power of two and no "
              "greater than 64\n",
              name());
    }
    if (maxOffset <= 0) {
        fatal("%s: maximum offset must be > 0\n", name());
    }
    if (maxOffset >= int(bitVectorSize)) {
        fatal("%s: maximum offset must be less than the bit vector size\n",
              name());
    }
    if (recentDepth > 0 && recentDepth >= bitVectorSize) {
        fatal("%s: lookahead levels configuration implies a recent depth that "
              "must be less than the bit vector size\n",
              name());
    }
}

void
MLOP::resetScores()
{
    for (auto &entry : offsetTable) {
        std::fill(entry.begin(), entry.end(), 0);
    }
}

MLOP::AMTEntry &
MLOP::findOrAllocAmtEntry(Addr base_block)
{
    const AMTEntry::KeyType key{base_block, false};

    AMTEntry *entry = amt.findEntry(key);
    if (entry != nullptr) {
        amt.accessEntry(entry);
    } else {
        entry = amt.findVictim(key);
        amt.insertEntry(key, entry);
    }
    return *entry;
}

void
MLOP::updateScoresWithAccess(Addr block)
{
    const Addr base_block = block & ~regionMask;
    const unsigned idx = unsigned(block & regionMask);

    AMTEntry &entry = findOrAllocAmtEntry(base_block);

    // For each level L, credit every offset that would have predicted
    // this access after excluding the last (L-1) accesses. excludeMask
    // gains one more excluded position per level instead of being
    // rebuilt from scratch every iteration.
    uint64_t excludeMask = 0;
    for (unsigned exclude = 0; exclude < lookaheadLevels; exclude++) {
        uint64_t bits = entry.bitVector & ~excludeMask;

        while (bits) {
            const unsigned j = findLsbSet(bits);
            bits &= (bits - 1);

            const Offset k = int(idx) - int(j);
            if (k == 0 || k < -maxOffset || k > maxOffset) {
                continue;
            }
            offsetTable[k + maxOffset][exclude]++;
        }

        if (exclude < entry.recent.size()) {
            excludeMask |= (1ULL << entry.recent[exclude]);
        }
    }

    if (recentDepth > 0) {
        entry.recent.push_front(uint8_t(idx));
        if (entry.recent.size() > recentDepth) {
            entry.recent.pop_back();
        }
    }

    entry.bitVector |= (1ULL << idx);
}

std::vector<std::pair<MLOP::LookaheadLevel, MLOP::Offset>>
MLOP::selectBestOffsets()
{
    std::vector<std::pair<LookaheadLevel, Offset>> selection;

    // Keep every tied top-scorer per level, process longest lookahead
    // first so a claimed offset isn't reissued at a shorter one. Index
    // maxOffset (offset 0) is never a valid candidate and stays unused.
    std::vector<bool> used(offsetTable.size(), false);
    std::vector<std::vector<Offset>> per_level(lookaheadLevels);

    for (LookaheadLevel L = lookaheadLevels; L >= 1; L--) {
        uint32_t best_score = 0;
        for (unsigned i = 0; i < offsetTable.size(); i++) {
            if (i == unsigned(maxOffset)) {
                continue;
            }
            best_score = std::max(best_score, offsetTable[i][L - 1]);
        }
        if (best_score < scoreThreshold) {
            continue;
        }

        std::vector<Offset> &winners = per_level[L - 1];
        for (unsigned i = 0; i < offsetTable.size(); i++) {
            if (i == unsigned(maxOffset)) {
                continue;
            }
            if (offsetTable[i][L - 1] == best_score && !used[i]) {
                winners.push_back(int(i) - maxOffset);
            }
        }
        for (Offset offset : winners) {
            used[offset + maxOffset] = true;
        }
    }

    // Emit in increasing-lookahead order (L=1 first) so the existing
    // issue-order priority in calculatePrefetch is preserved.
    for (LookaheadLevel L = 1; L <= lookaheadLevels; L++) {
        for (Offset offset : per_level[L - 1]) {
            selection.emplace_back(L, offset);
        }
    }

    return selection;
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
        bestOffsets = selectBestOffsets();
        resetScores();
        accessCounter = 0;
    }

    // Issue in increasing lookahead order, L=1 first: it needs to arrive
    // soonest, so it gets the highest queue priority.
    unsigned issued = 0;
    for (const auto &p : bestOffsets) {
        if (issued >= prefetchDegree) {
            break;
        }

        const LookaheadLevel L = p.first;
        const Offset offset = p.second;
        const Addr pf_addr = addr + (Addr(offset) << lBlkSize);

        const int32_t prio = int32_t(lookaheadLevels - L);
        addresses.emplace_back(pf_addr, prio);
        issued++;
    }
}

} // namespace prefetch
} // namespace gem5
