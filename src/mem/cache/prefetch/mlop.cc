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
    if (lookaheadLevels == 0) {
        fatal("%s: lookahead_levels must be > 0\n", name());
    }
    if (!isPowerOfTwo(bitVectorSize)) {
        fatal("%s: bit_vector_size (%u) must be a power of two\n",
              name(), bitVectorSize);
    }
    if (maxOffset <= 0) {
        fatal("%s: max_offset must be > 0 (got %d)\n", name(), maxOffset);
    }
    if (maxOffset >= int(bitVectorSize)) {
        fatal("%s: max_offset (%d) must be < bit_vector_size (%u)\n",
              name(), maxOffset, bitVectorSize);
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

    // For each lookahead level L, exclude the last (L-1) accesses from
    // the bit-vector, then credit every offset that would have predicted
    // this access from what remains.
    for (unsigned L = 1; L <= lookaheadLevels; L++) {
        uint64_t masked = entry.bitVector;
        const unsigned exclude = (L > 1) ? (L - 1) : 0;
        const unsigned ex = std::min<unsigned>(exclude, entry.recent.size());
        for (unsigned r = 0; r < ex; r++) {
            masked &= ~(1ULL << entry.recent[r]);
        }

        uint64_t bits = masked;
        while (bits) {
            const unsigned j = ctz64(bits);
            bits &= (bits - 1);

            const int k = int(idx) - int(j);
            if (k == 0 || k < -maxOffset || k > maxOffset) {
                continue;
            }
            auto it = offsetTable.find(k);
            if (it != offsetTable.end()) {
                it->second.scores[L - 1]++;
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

    for (unsigned L = 1; L <= lookaheadLevels; L++) {
        uint32_t bestScore = 0;
        const OffsetEntry *bestEntry = nullptr;

        for (auto &kv : offsetTable) {
            const OffsetEntry &e = kv.second;
            const uint32_t s = e.scores[L - 1];
            if (s >= scoreThreshold && s > bestScore) {
                bestScore = s;
                bestEntry = &e;
            }
        }

        if (bestEntry) {
            bestOffsets.emplace_back(L, bestEntry);
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
