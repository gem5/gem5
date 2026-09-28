/**
 * Selective Retention Cache (SRC) support for the classic cache.
 *
 * A GC-eDRAM data array loses a block's contents DRT ticks after the
 * block was last fully written (filled or refreshed). All SRC state,
 * logic, the refresh daemon and the SRC statistics live in this class.
 *
 * BaseCache owns an instance only when drt > 0. With drt == 0 no
 * instance exists, every hook in BaseCache is skipped, and the cache
 * behaves exactly like stock gem5.
 */

#ifndef __MEM_CACHE_RETENTION_RETENTION_MANAGER_HH__
#define __MEM_CACHE_RETENTION_RETENTION_MANAGER_HH__

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "sim/eventq.hh"

namespace gem5
{

class BaseCache;
class CacheBlk;
struct BaseCacheParams;

class RetentionManager
{
  public:
    RetentionManager(BaseCache &cache, const BaseCacheParams &p);

    /** Start the refresh daemon. Called from BaseCache::startup(). */
    void startup();

    /**
     * Expiry check for the block addressed by a read or write request,
     * done before the tag access. An expired block is written back if
     * dirty (the lazy writeback fallback) and invalidated, so the tag
     * access that follows misses.
     *
     * An expired block with a request already in flight (MSHR or write
     * buffer) is left in place and served as usual; this is counted.
     *
     * @param pkt The request being serviced.
     * @param writebacks List to append the lazy writeback to.
     */
    void expireOnAccess(PacketPtr pkt, PacketList &writebacks);

    /** Bookkeeping for a block returned by the tag access. */
    void onAccessHit(CacheBlk *blk);

    /** A block was allocated and filled: start its retention timer. */
    void onAllocate(CacheBlk *blk);

    /**
     * A block is being evicted. An expired dirty block is still written
     * back (fallback), and counted.
     */
    void onEvict(CacheBlk *blk);

    /** A functional access found blk. Expired data is used; counted. */
    void onFunctional(CacheBlk *blk);

    /** A snoop found blk. Expired data is used; counted. */
    void onSnoop(CacheBlk *blk);

  private:
    /** One daemon step: visit set refreshSetIdx, then the next set. */
    void processRefreshEvent();

    /** Ticks since blk was last fully written. */
    Tick age(const CacheBlk *blk) const;

    /**
     * Record a retention invariant violation: a dirty block was found
     * expired, so its data is lost. Panics if drt_strict is set.
     */
    void violation(const CacheBlk *blk, const char *where);

    BaseCache &cache;

    /** Number of top MRU blocks per set the daemon keeps alive. */
    const unsigned topMRU;

    /** Whether the daemon keeps dirty blocks alive. */
    const bool refreshDirtyDaemon;

    /** Panic on a retention invariant violation instead of counting. */
    const bool strict;

    /** Set the daemon visits next. */
    unsigned refreshSetIdx;

    EventFunctionWrapper refreshEvent;

    struct RetentionStats : public statistics::Group
    {
        RetentionStats(BaseCache &cache);

        /** Accesses that found their block expired and missed. */
        statistics::Scalar retentionMisses;
        /** Lazy writebacks of expired dirty blocks, by a read access. */
        statistics::Scalar lazyWritebacksRead;
        /** Lazy writebacks of expired dirty blocks, by a write access. */
        statistics::Scalar lazyWritebacksWrite;
        /** All lazy writebacks. */
        statistics::Formula lazyWritebacks;
        /** Expired blocks served anyway: a request was in flight. */
        statistics::Scalar expiredInflightHits;
        /** Expired dirty blocks written back on eviction. */
        statistics::Scalar expiredDirtyEvictions;
        /** Expired clean blocks evicted (dead, no data loss). */
        statistics::Scalar expiredCleanEvictions;
        /** Functional accesses that used an expired block. */
        statistics::Scalar expiredFunctional;
        /** Snoops that used an expired block. */
        statistics::Scalar expiredSnoops;
        /** Expired dirty blocks the daemon refreshed (revived). */
        statistics::Scalar daemonRevivedDirty;
        /** Dirty blocks found expired, anywhere (should be 0). */
        statistics::Scalar invariantViolations;
        /** Daemon refreshes done while the cache was not blocked. */
        statistics::Scalar oppRefresh;
        /** Daemon refreshes forced by the age threshold. */
        statistics::Scalar forceRefresh;
        /** All daemon refreshes. */
        statistics::Scalar totalRefreshes;
        /** Expired blocks accessed within 1.2 DRT of their charge. */
        statistics::Scalar expiredHitPot;
        /** Hits on blocks the daemon refreshed. */
        statistics::Scalar refreshedBlockUtility;
    } stats;
};

} // namespace gem5

#endif // __MEM_CACHE_RETENTION_RETENTION_MANAGER_HH__
