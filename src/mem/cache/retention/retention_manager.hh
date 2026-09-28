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
     * Expiry check for the block addressed by pkt, done before the tag
     * access. An expired block is written back if dirty (the lazy
     * writeback fallback) and invalidated.
     *
     * @param pkt The request being serviced.
     * @param writebacks List to append the lazy writeback to.
     * @return true if the block expired and the access must miss.
     */
    bool expireOnAccess(PacketPtr pkt, PacketList &writebacks);

    /** Bookkeeping for a block returned by the tag access. */
    void onAccessHit(CacheBlk *blk);

    /** A block was allocated and filled: start its retention timer. */
    void onAllocate(CacheBlk *blk);

  private:
    /** One daemon step: visit set refreshSetIdx, then the next set. */
    void processRefreshEvent();

    BaseCache &cache;

    /** Number of top MRU blocks per set the daemon keeps alive. */
    const unsigned topMRU;

    /** Whether the daemon keeps dirty blocks alive. */
    const bool refreshDirtyDaemon;

    /** Set the daemon visits next. */
    unsigned refreshSetIdx;

    EventFunctionWrapper refreshEvent;

    struct RetentionStats : public statistics::Group
    {
        RetentionStats(BaseCache &cache);

        /** Accesses that found their block expired. */
        statistics::Scalar retentionMisses;
        /** Expired blocks that were dirty (lazy writebacks). */
        statistics::Scalar dirtyRetentionMisses;
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
