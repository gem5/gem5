/**
 * Reuse statistics for a cache (reuse_stats), simulator-only: none of
 * this is hardware state, and nothing here changes timing.
 *
 * Per line, the tick of its last access (fill, demand hit or writeback
 * from above; a CleanEvict is not a use) and its demand hits since the
 * fill. At each demand hit the time since the last access is sampled;
 * when a line leaves the cache it is counted by why it left (replacement,
 * an SRC drop or expiry, anything else) and whether it had any hit.
 *
 * Misses the SRC causes. A shadow copy of the tags, same geometry with
 * LRU replacement and no retention, sees the same accesses: what an SRAM
 * cache would hold. When the SRC drops a line (deadline invalidation,
 * deadline writeback, expiry on access) that the shadow holds, the line
 * becomes a ghost. The ghost dies when the shadow's replacement evicts
 * it. A demand miss to a live ghost is a miss the SRC caused (an SRAM
 * would have hit); the time since the drop is sampled. A miss to a dead
 * ghost would have missed in the SRAM too.
 *
 * Time bins (both histograms): 1 us bins [0,1) ... [63,64) us, then
 * [64,128) ... [2048,4096) us, then >= 4096 us.
 */

#ifndef __MEM_CACHE_RETENTION_REUSE_TRACKER_HH__
#define __MEM_CACHE_RETENTION_REUSE_TRACKER_HH__

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/packet.hh"

namespace gem5
{

class BaseCache;
class CacheBlk;

class ReuseTracker
{
  public:
    /** Why the lines removed from now on leave the cache. */
    enum class Reason { Other, Replacement, Drop };

    ReuseTracker(BaseCache &cache);

    /**
     * An access after its tag lookup (blk: the line found, or nullptr):
     * a demand tag miss to a ghost, and the shadow tags.
     */
    void onAccess(PacketPtr pkt, CacheBlk *blk);

    /** A demand request hit blk. */
    void onDemandHit(CacheBlk *blk);

    /** A writeback from above wrote blk (after onAllocate if new). */
    void onWriteback(CacheBlk *blk);

    /** blk was just allocated (a fill or a writeback). */
    void onAllocate(CacheBlk *blk);

    /** blk is about to be invalidated, for the current reason. */
    void onRemove(CacheBlk *blk);

    /** Set the reason of the next removals (Other when done). */
    void setReason(Reason r) { reason = r; }

    /** Number of time bins. */
    static constexpr unsigned NumBins = 64 + 6 + 1;

  private:
    /** Time bin of an interval in ticks. */
    static unsigned bin(Tick dt);

    unsigned lineIdx(const CacheBlk *blk) const;

    /**
     * The shadow tags see an access to addr in set: a hit updates the
     * LRU order; a miss inserts addr if insert (evicting the LRU way,
     * which kills its ghost).
     */
    void shadowAccess(unsigned set, Addr addr, bool insert);

    /** The shadow tags hold addr in set. */
    bool shadowHolds(unsigned set, Addr addr) const;

    BaseCache &cache;
    const unsigned numSets;
    const unsigned assoc;

    /** Per line (set * assoc + way): last access and demand hits. */
    std::vector<Tick> lastAccess;
    std::vector<uint32_t> hits;

    /** Shadow tags, set * assoc + way. */
    struct ShadowWay
    {
        bool valid = false;
        Addr addr = 0;
        Tick lastUse = 0;
    };
    std::vector<ShadowWay> shadow;

    /** Live ghosts with their drop tick, and dead ghosts. */
    std::unordered_map<Addr, Tick> liveGhosts;
    std::unordered_set<Addr> deadGhosts;

    Reason reason = Reason::Other;

    /** Live ghosts at the last stats reset. */
    uint64_t ghostsAtReset = 0;

    struct ReuseStats : public statistics::Group
    {
        ReuseStats(ReuseTracker &tracker, BaseCache &cache);

        void regStats() override;
        void resetStats() override;

        ReuseTracker &tracker;

        /** Time since the line's last access, at each demand hit. */
        statistics::Vector timeSinceLastAccess;
        /** Lines that left by replacement / with no demand hit. */
        statistics::Scalar linesEvicted;
        statistics::Scalar zeroHitEvictions;
        /** Lines the SRC dropped or found expired / with no hit. */
        statistics::Scalar linesDropped;
        statistics::Scalar zeroHitDrops;
        /** Lines invalidated for any other reason / with no hit. */
        statistics::Scalar linesInvalidatedOther;
        statistics::Scalar zeroHitOther;
        /** SRC drops of lines the shadow holds (ghosts made). */
        statistics::Scalar ghostsCreated;
        /** SRC drops of lines the shadow had evicted already. */
        statistics::Scalar dropsNotInShadow;
        /** Demand misses to a live ghost: misses the SRC caused. */
        statistics::Scalar srcCausedMisses;
        /** Time since the drop at each of those misses. */
        statistics::Vector timeSinceDropAtMiss;
        /** Ghosts the shadow's replacement evicted. */
        statistics::Scalar ghostsKilledByReplacement;
        /** Demand misses to a dead ghost (an SRAM misses too). */
        statistics::Scalar ghostMissesAfterReplacement;
        /** Live ghosts refilled without a demand miss (writeback). */
        statistics::Scalar ghostsRefilledOther;
        /** Live ghosts now, and at the last stats reset. */
        statistics::Value ghostsLive;
        statistics::Value ghostsLiveAtReset;
    } stats;
};

} // namespace gem5

#endif // __MEM_CACHE_RETENTION_REUSE_TRACKER_HH__
