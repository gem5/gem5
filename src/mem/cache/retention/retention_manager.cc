#include "mem/cache/retention/retention_manager.hh"

#include <algorithm>
#include <vector>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Cache.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/tags/base.hh"
#include "params/BaseCache.hh"

namespace gem5
{

RetentionManager::RetentionManager(BaseCache &_cache,
                                   const BaseCacheParams &p)
    : cache(_cache),
      topMRU(p.top_mru),
      refreshDirtyDaemon(p.refresh_dirty_daemon),
      refreshSetIdx(0),
      refreshEvent([this] { processRefreshEvent(); },
                   _cache.name() + ".refreshDaemon"),
      stats(_cache)
{
    fatal_if(!cache.tags->isForgetting(),
             "%s: drt > 0 but the tags do not support retention",
             cache.name());
}

void
RetentionManager::startup()
{
    if (topMRU > 0 || refreshDirtyDaemon) {
        cache.schedule(refreshEvent, cache.clockEdge(Cycles(1)));
    }
}

bool
RetentionManager::expireOnAccess(PacketPtr pkt, PacketList &writebacks)
{
    if (!pkt->isRead()) {
        return false;
    }

    Addr blk_addr = pkt->getBlockAddr(cache.blkSize);
    CacheBlk *blk = cache.tags->findBlock({blk_addr, pkt->isSecure()});

    if (!blk || !cache.tags->isExpired(blk)) {
        return false;
    }

    // The block has a request in flight (O3): leave it alone
    if (cache.mshrQueue.findMatch(blk_addr, pkt->isSecure()) ||
        cache.writeBuffer.findMatch(blk_addr, pkt->isSecure())) {
        return false;
    }

    stats.retentionMisses++;

    if (blk->isSet(CacheBlk::DirtyBit)) {
        stats.dirtyRetentionMisses++;
    }

    Tick age = curTick() - blk->getFullChargeTick();
    Tick drt = cache.tags->getDRT();

    // Died recently: between 100% and 120% of DRT
    if (age >= drt && age < (drt * 1.2)) {
        stats.expiredHitPot++;
    }

    // Lazy writeback of a dirty block, then invalidate and miss
    if (blk->isSet(CacheBlk::DirtyBit)) {
        PacketPtr wb_packet = cache.writebackBlk(blk);
        if (wb_packet) {
            writebacks.push_back(wb_packet);
        }
    }

    cache.invalidateBlock(blk);

    return true;
}

void
RetentionManager::onAccessHit(CacheBlk *blk)
{
    if (blk->getWasOppRefreshed()) {
        stats.refreshedBlockUtility++;
        blk->setWasOppRefreshed(false);
    }
}

void
RetentionManager::onAllocate(CacheBlk *blk)
{
    blk->setFullChargeTick(curTick());
}

void
RetentionManager::processRefreshEvent()
{
    BaseTags *tags = cache.tags;
    bool is_blocked = cache.cpuSidePort.isBlocked() || cache.blocked;

    DPRINTF(Cache, "RefreshDaemon: Set %d | Blocked: %d\n",
            refreshSetIdx, is_blocked);

    // Adaptive thresholds
    double sweep_cycles = double(tags->getNumSets());
    double drt_cycles = double(tags->getDRT()) / double(cache.clockPeriod());

    double force_thresh = 0.95;
    if (drt_cycles > 0.0) {
        // leave a window >= sweep time, plus 2% safety margin
        double needed = 1.0 - (sweep_cycles / drt_cycles) - 0.02;
        force_thresh = std::min(0.95, std::max(0.50, needed));
    }
    double opp_thresh = std::min(0.80, force_thresh - 0.05);

    if (refreshDirtyDaemon) {
        std::vector<CacheBlk *> set_blks = tags->getSetBlks(refreshSetIdx);

        for (CacheBlk *blk : set_blks) {
            if (!(blk && blk->isValid()))
                continue;
            if (!blk->isSet(CacheBlk::DirtyBit))
                continue;

            Tick age = curTick() - blk->getFullChargeTick();
            Tick drt = tags->getDRT();

            bool force_refresh = (age >= drt * force_thresh);
            bool opp_refresh = (!is_blocked && age >= drt * opp_thresh);

            if (force_refresh || opp_refresh) {
                DPRINTF(Cache, "RefreshDaemon: %s dirty block %#lx age %lu\n",
                        force_refresh ? "FORCE" : "OPP", blk->getTag(), age);

                blk->setFullChargeTick(curTick());
                blk->setWasOppRefreshed(true);

                stats.totalRefreshes++;
                if (force_refresh) {
                    stats.forceRefresh++;
                } else {
                    stats.oppRefresh++;
                }
            }
        }
    }

    if (topMRU > 0) {
        std::vector<CacheBlk *> top_mru_blks =
            tags->getNTopMRU(refreshSetIdx, topMRU);
        int rank = 0;

        for (CacheBlk *mru : top_mru_blks) {
            if (!(mru && mru->isValid())) {
                rank++;
                continue;
            }

            // Refreshed by the dirty loop above in this same step
            if (mru->getFullChargeTick() == curTick()) {
                rank++;
                continue;
            }

            Tick age = curTick() - mru->getFullChargeTick();
            Tick drt = tags->getDRT();

            if (!tags->isExpired(mru)) {
                bool force_refresh = (age >= drt * force_thresh);
                bool opp_refresh = (!is_blocked && age >= drt * opp_thresh);

                if (force_refresh || opp_refresh) {
                    DPRINTF(Cache,
                            "RefreshDaemon: %s MRU rank %d block %#lx\n",
                            force_refresh ? "FORCE" : "OPP", rank,
                            mru->getTag());

                    mru->setFullChargeTick(curTick());
                    mru->setWasOppRefreshed(true);

                    stats.totalRefreshes++;
                    if (force_refresh) {
                        stats.forceRefresh++;
                    } else {
                        stats.oppRefresh++;
                    }
                }
            }
            rank++;
        }
    }

    refreshSetIdx = (refreshSetIdx + 1) % tags->getNumSets();
    cache.schedule(refreshEvent, cache.clockEdge(Cycles(1)));
}

RetentionManager::RetentionStats::RetentionStats(BaseCache &cache)
    : statistics::Group(&cache, "retention"),
      ADD_STAT(retentionMisses, statistics::units::Count::get(),
               "accesses that found their block expired"),
      ADD_STAT(dirtyRetentionMisses, statistics::units::Count::get(),
               "expired blocks that were dirty (lazy writebacks)"),
      ADD_STAT(oppRefresh, statistics::units::Count::get(),
               "daemon refreshes done while the cache was not blocked"),
      ADD_STAT(forceRefresh, statistics::units::Count::get(),
               "daemon refreshes forced by the age threshold"),
      ADD_STAT(totalRefreshes, statistics::units::Count::get(),
               "all daemon refreshes"),
      ADD_STAT(expiredHitPot, statistics::units::Count::get(),
               "expired blocks accessed within 1.2 DRT of their charge"),
      ADD_STAT(refreshedBlockUtility, statistics::units::Count::get(),
               "hits on blocks the daemon refreshed")
{
}

} // namespace gem5
