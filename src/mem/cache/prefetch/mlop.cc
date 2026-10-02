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

MLOP::MLOP(const MLOPPrefetcherParams &p)
    : Queued(p),
      eval_period(p.evaluation_period),
      lookahead_levels(p.lookahead_levels),
      max_offset(p.max_offset),
      score_threshold(p.score_threshold),
      prefetch_degree(p.degree),
      amt_entries(p.amt_entries),
      bit_vector_size(p.bit_vector_size),
      recent_depth(lookahead_levels > 0 ? lookahead_levels - 1 : 0),
      region_mask(Addr(bit_vector_size - 1)),
      amt("AMT", p.amt_entries, p.amt_assoc, p.amt_replacement_policy,
          p.amt_indexing_policy,
          AMTEntry(genTagExtractor(p.amt_indexing_policy))),
      offset_table(2 * max_offset + 1,
                   OffsetEntry{std::vector<uint32_t>(lookahead_levels, 0)})
{
    if (amt_entries == 0) {
        fatal("%s: number of Access Map Table entries must be > 0\n", name());
    }
    if (lookahead_levels == 0) {
        fatal("%s: number of lookahead levels must be > 0\n", name());
    }
    if (!isPowerOf2(bit_vector_size) || bit_vector_size > 64) {
        fatal("%s: bit vector size must be a power of two and no "
              "greater than 64\n",
              name());
    }
    if (max_offset <= 0) {
        fatal("%s: maximum offset must be > 0\n", name());
    }
    if (max_offset >= int(bit_vector_size)) {
        fatal("%s: maximum offset must be less than the bit vector size\n",
              name());
    }
    if (recent_depth > 0 && recent_depth >= bit_vector_size) {
        fatal("%s: lookahead levels configuration implies a recent depth that "
              "must be less than the bit vector size\n",
              name());
    }
}

void
MLOP::resetScores()
{
    for (auto &entry : offset_table) {
        std::fill(entry.scores.begin(), entry.scores.end(), 0);
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
        entry->recent.reserve(recent_depth);
    }
    return *entry;
}

void
MLOP::updateScoresWithAccess(Addr block)
{
    const Addr base_block = block & ~region_mask;
    const unsigned idx = unsigned(block & region_mask);

    AMTEntry &entry = findOrAllocAmtEntry(base_block);

    // For each level L, credit every offset that would have predicted
    // this access after excluding the last (L-1) accesses.
    for (unsigned exclude = 0; exclude < lookahead_levels; exclude++) {
        uint64_t bits = entry.bit_vector;
        for (unsigned r = 0;
             r < std::min<unsigned>(exclude, entry.recent.size()); r++) {
            bits &= ~(1ULL << entry.recent[r]);
        }

        while (bits) {
            const unsigned j = findLsbSet(bits);
            bits &= (bits - 1);

            const int k = int(idx) - int(j);
            if (k == 0 || k < -max_offset || k > max_offset) {
                continue;
            }
            offset_table[k + max_offset].scores[exclude]++;
        }
    }

    if (recent_depth > 0) {
        entry.recent.insert(entry.recent.begin(), uint8_t(idx));
        if (entry.recent.size() > recent_depth) {
            entry.recent.resize(recent_depth);
        }
    }

    entry.bit_vector |= (1ULL << idx);
}

std::vector<std::pair<unsigned, int>>
MLOP::selectBestOffsets()
{
    std::vector<std::pair<unsigned, int>> selection;

    // Keep every tied top-scorer per level, process longest lookahead
    // first so a claimed offset isn't reissued at a shorter one. Index
    // max_offset (offset 0) is never a valid candidate and stays unused.
    std::vector<bool> used(offset_table.size(), false);
    std::vector<std::vector<int>> per_level(lookahead_levels);

    for (unsigned L = lookahead_levels; L >= 1; L--) {
        uint32_t best_score = 0;
        for (unsigned i = 0; i < offset_table.size(); i++) {
            if (i == unsigned(max_offset)) {
                continue;
            }
            best_score = std::max(best_score, offset_table[i].scores[L - 1]);
        }
        if (best_score < score_threshold) {
            continue;
        }

        std::vector<int> &winners = per_level[L - 1];
        for (unsigned i = 0; i < offset_table.size(); i++) {
            if (i == unsigned(max_offset)) {
                continue;
            }
            if (offset_table[i].scores[L - 1] == best_score && !used[i]) {
                winners.push_back(int(i) - max_offset);
            }
        }
        for (int offset : winners) {
            used[offset + max_offset] = true;
        }
    }

    // Emit in increasing-lookahead order (L=1 first) so the existing
    // issue-order priority in calculatePrefetch is preserved.
    for (unsigned L = 1; L <= lookahead_levels; L++) {
        for (int offset : per_level[L - 1]) {
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
    access_counter++;

    if (access_counter >= eval_period) {
        best_offsets = selectBestOffsets();
        resetScores();
        access_counter = 0;
    }

    // Issue in increasing lookahead order, L=1 first: it needs to arrive
    // soonest, so it gets the highest queue priority.
    unsigned issued = 0;
    for (const auto &p : best_offsets) {
        if (issued >= prefetch_degree) {
            break;
        }

        const unsigned L = p.first;
        const int offset = p.second;
        const Addr pf_addr = addr + (Addr(offset) << lBlkSize);

        const int32_t prio = int32_t(lookahead_levels - L);
        addresses.emplace_back(pf_addr, prio);
        issued++;
    }
}

} // namespace prefetch
} // namespace gem5
