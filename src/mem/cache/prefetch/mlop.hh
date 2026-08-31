/**
 * @file
 * Implementation of the 'Multi-Lookahead Offset Prefetcher'
 * Reference:
 *   Shakerinava, M. et al. (2019, June). Multi-lookahead offset
 *   prefetching. In DPC3 (in conjunction with ISCA 2019).
 *   https://mshakerinava.github.io/papers/mlop-dpc3.pdf
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
#include <unordered_map>
#include <utility>
#include <vector>

#include "base/types.hh"
#include "mem/cache/prefetch/queued.hh"

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
    struct OffsetEntry
    {
        int offset;
        /** scores[L - 1] is this offset's score at lookahead level L */
        std::vector<uint32_t> scores;
    };

    struct AMTEntry
    {
        bool valid = false;
        Addr baseBlock = 0;
        uint64_t bitVector = 0;
        /** Most-recent-first access indices, size <= recentDepth */
        std::vector<uint8_t> recent;
        uint64_t lastTouch = 0;
    };

    const unsigned evalPeriod;
    const unsigned lookaheadLevels;
    const int maxOffset;
    const unsigned scoreThreshold;
    const unsigned prefetchDegree;
    const unsigned amtEntries;
    const unsigned bitVectorSize;
    const unsigned recentDepth;
    const Addr regionMask;

    std::vector<AMTEntry> amt;
    uint64_t amtClock = 0;

    std::unordered_map<int, OffsetEntry> offsetTable;
    std::vector<std::pair<unsigned, const OffsetEntry *>> bestOffsets;
    unsigned accessCounter = 0;

    void resetScores();
    void selectBestOffsets();

    AMTEntry &findOrAllocAmtEntry(Addr baseBlock);
    void updateScoresWithAccess(Addr block);

    static inline unsigned
    ctz64(uint64_t x)
    {
        return (unsigned)__builtin_ctzll(x);
    }

    static inline bool
    isPowerOfTwo(unsigned x)
    {
        return x && ((x & (x - 1)) == 0);
    }
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_MLOP_HH__
