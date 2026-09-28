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
 *
 * Data-array banks. The data array has B banks (refresh_banks); set s
 * lives in bank s mod B. A refresh occupies only its bank, a hit waits
 * only for its own bank, and banks refresh in parallel. One daemon (a
 * single scanner, as with B = 1) visits the sets in order and hands
 * each set's refreshes to that set's bank; it waits only when the bank
 * of the next set is still busy. B = 1 is a single unbanked array.
 *
 * Retention guarantee. The refresh daemon visits one set per step; a
 * step scans the set's tags (refresh_set_scan_cycles) and refreshes the
 * chosen blocks one after the other in the set's bank, each occupying
 * that bank for refresh_line_cycles. Take rounds of B consecutive sets:
 * a round scans B sets and waits only for each bank's refreshes from the
 * previous round, at most k_max * line. So the daemon comes back to a
 * set within
 *
 *   T_sweep = ceil(numSets / B) * (B * scan + k_max * line) cycles,
 *
 * which is numSets * (scan + k_max * line) for B = 1, k_max being the
 * most blocks a step can refresh. The daemon therefore
 * force-refreshes a block whose remaining life is below
 *
 *   T_guard = T_sweep + line + margin * DRT,
 *
 * since it would expire before the next visit. Startup fails if
 * T_guard >= DRT: no refresh schedule can then keep dirty data alive.
 *
 * The same bound, seen from the write side: a store hit that leaves a
 * block dirty with less than T_guard of life left may have arrived just
 * after the daemon passed its set, so the store itself recharges the
 * line (inline refresh: read the line, merge the store, write the whole
 * line back), costing inline_refresh_cycles of extra latency. A store
 * that finds less life left than that read needs is too late and is
 * handled like an expired block.
 */

#ifndef __MEM_CACHE_RETENTION_RETENTION_MANAGER_HH__
#define __MEM_CACHE_RETENTION_RETENTION_MANAGER_HH__

#include <vector>

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

    /**
     * Compute the retention guarantee and start the refresh daemon.
     * Called from BaseCache::startup().
     */
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

    /**
     * Extra hit latency: waiting for a refresh that occupies the data
     * array, plus an inline refresh if this hit triggers one.
     */
    Cycles hitDelay(PacketPtr pkt, CacheBlk *blk);

    /**
     * The whole line of blk was written (allocation, fill): it is fully
     * charged again.
     */
    void onFullLineWrite(CacheBlk *blk);

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

    /** Daemon policy for one valid, unexpired block. */
    void maybeRefresh(CacheBlk *blk);

    /** Recharge blk with a full-line read and write. */
    void refresh(CacheBlk *blk, bool force);

    /** Data-array bank of a set. */
    unsigned bankOf(unsigned set) const { return set % banks; }

    /** Occupy a bank with one full-line refresh. */
    void occupyBank(unsigned bank);

    /** Whether an access of this kind triggers inline refreshes. */
    bool inlineApplies(PacketPtr pkt) const;

    /** Valid blocks of a set, most recently touched first, at most n. */
    std::vector<CacheBlk *> topMRUBlks(unsigned set, unsigned n) const;

    /** No demand access to the bank this cycle and no refresh in it. */
    bool bankIdle(unsigned bank) const;

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

    /** Associativity, the most blocks one daemon step can refresh. */
    const unsigned assoc;

    /** Cycles for the daemon to scan one set's tags. */
    const Cycles scanCycles;

    /** Data-array cycles of one full-line refresh (read + write). */
    const Cycles lineCycles;

    /** Number of data-array banks. */
    const unsigned banks;

    /** Age (fraction of DRT) from which idle-time refreshes start. */
    const double oppAge;

    /** Safety margin of the guarantee, as a fraction of DRT. */
    const double margin;

    /** Store hits near expiry recharge the line. */
    const bool inlineOnWrite;

    /** Read hits near expiry recharge the line. */
    const bool inlineOnRead;

    /** Extra latency of an inline refresh (reading the line first). */
    const Cycles inlineCycles;

    /** Worst-case time for the daemon to come back to a set. */
    Tick tSweep;

    /** Force-refresh a block whose remaining life is below this. */
    Tick tGuard;

    /** Each bank is busy with refreshes until this tick. */
    std::vector<Tick> bankBusyUntil;

    /** Tick of the last demand access to each bank. */
    std::vector<Tick> bankLastAccess;

    /** Tick the current daemon sweep started at set 0. */
    Tick sweepStart;

    /** Set the daemon visits next. */
    unsigned refreshSetIdx;

    EventFunctionWrapper refreshEvent;

    struct RetentionStats : public statistics::Group
    {
        RetentionStats(RetentionManager &mgr, BaseCache &cache);

        void regStats() override;

        RetentionManager &mgr;

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
        /** Daemon refreshes done while the data array was idle. */
        statistics::Scalar oppRefresh;
        /** Daemon refreshes forced by the retention guarantee. */
        statistics::Scalar forceRefresh;
        /** All daemon refreshes (each is one line read + one write). */
        statistics::Scalar totalRefreshes;
        /** Block age at a forced refresh, as a fraction of DRT. */
        statistics::Distribution ageAtForceRefresh;
        /** Hits delayed because a refresh occupied the data array. */
        statistics::Scalar refreshStalledHits;
        /** Cycles hits waited for refreshes. */
        statistics::Scalar refreshStallCycles;
        /** Inline refreshes triggered by a store hit. */
        statistics::Scalar refreshInlineWrite;
        /** Inline refreshes triggered by a read hit. */
        statistics::Scalar refreshInlineRead;
        /** Block age at an inline refresh, as a fraction of DRT. */
        statistics::Distribution ageAtInlineRefresh;
        /** Stores that made a clean block dirty inside T_guard. */
        statistics::Scalar cleanToDirtyInDanger;
        /** Cycles added to hits by inline refreshes. */
        statistics::Scalar inlineRefreshCycles;
        /** Hits with too little life left for an inline refresh. */
        statistics::Scalar inlineTooLate;
        /** Duration of each full daemon sweep, in cycles. */
        statistics::Histogram daemonSweepCycles;
        /** Worst-case sweep time the guarantee assumes, in cycles. */
        statistics::Value tSweepCycles;
        /** Force-refresh threshold on remaining life, in cycles. */
        statistics::Value tGuardCycles;
        /** Expired blocks accessed within 1.2 DRT of their charge. */
        statistics::Scalar expiredHitPot;
        /** Hits on blocks the daemon refreshed. */
        statistics::Scalar refreshedBlockUtility;
        /** Line refreshes (daemon and inline) per bank. */
        statistics::Vector bankRefreshes;
        /** Hits delayed by a refresh, per bank. */
        statistics::Vector bankStalledHits;
        /** Cycles hits waited for refreshes, per bank. */
        statistics::Vector bankStallCycles;
    } stats;
};

} // namespace gem5

#endif // __MEM_CACHE_RETENTION_RETENTION_MANAGER_HH__
