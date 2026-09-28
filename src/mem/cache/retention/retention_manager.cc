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
      strict(p.drt_strict),
      assoc(p.assoc),
      scanCycles(p.refresh_set_scan_cycles),
      lineCycles(p.refresh_line_cycles > 0 ? p.refresh_line_cycles :
                 Cycles(2 * p.data_latency)),
      oppAge(p.refresh_opp_age),
      margin(p.retention_margin),
      inlineOnWrite(p.inline_refresh),
      inlineOnRead(p.inline_refresh_on_read),
      inlineCycles(p.inline_refresh_cycles > 0 ? p.inline_refresh_cycles :
                   Cycles(p.data_latency)),
      tSweep(0),
      tGuard(0),
      dataBusyUntil(0),
      lastAccessTick(0),
      sweepStart(0),
      refreshSetIdx(0),
      refreshEvent([this] { processRefreshEvent(); },
                   _cache.name() + ".refreshDaemon"),
      stats(*this, _cache)
{
    fatal_if(!cache.tags->isForgetting(),
             "%s: drt > 0 but the tags do not support retention",
             cache.name());
    fatal_if(topMRU > assoc, "%s: top_mru (%u) > assoc (%u)",
             cache.name(), topMRU, assoc);
    fatal_if(scanCycles == 0, "%s: refresh_set_scan_cycles must be > 0",
             cache.name());
}

void
RetentionManager::startup()
{
    const Tick drt = cache.tags->getDRT();
    const bool daemon = topMRU > 0 || refreshDirtyDaemon;

    if (!daemon) {
        // No daemon to wait for: inline refreshes just before expiry
        tGuard = cache.cyclesToTicks(lineCycles) +
            Tick(margin * double(drt));
        inform("%s: retention DRT %lu ticks, no refresh daemon, "
               "T_guard %lu", cache.name(), drt, tGuard);
        return;
    }

    const unsigned num_sets = cache.tags->getNumSets();
    const unsigned k_max = refreshDirtyDaemon ? assoc : topMRU;

    tSweep = num_sets *
        cache.cyclesToTicks(Cycles(scanCycles + k_max * lineCycles));
    tGuard = tSweep + cache.cyclesToTicks(lineCycles) +
        Tick(margin * double(drt));

    fatal_if(tGuard >= drt,
             "%s: retention cannot be guaranteed: T_guard = %lu ticks >= "
             "DRT = %lu ticks (T_sweep = %u sets x (%u + %u x %u) cycles). "
             "Use a larger DRT or a faster refresh.",
             cache.name(), tGuard, drt, num_sets, (unsigned)scanCycles,
             k_max, (unsigned)lineCycles);

    inform("%s: retention DRT %lu ticks, T_sweep %lu, T_guard %lu "
           "(force-refresh from %.1f%% of DRT)", cache.name(), drt, tSweep,
           tGuard, 100.0 * double(drt - tGuard) / double(drt));

    sweepStart = curTick();
    cache.schedule(refreshEvent, cache.clockEdge(Cycles(1)));
}

Tick
RetentionManager::age(const CacheBlk *blk) const
{
    return curTick() - blk->getFullChargeTick();
}

void
RetentionManager::violation(const CacheBlk *blk, const char *where)
{
    stats.invariantViolations++;
    panic_if(strict, "%s: dirty block %#lx expired at %s "
             "(age %lu >= drt %lu)", cache.name(), blk->getTag(), where,
             age(blk), cache.tags->getDRT());
}

void
RetentionManager::expireOnAccess(PacketPtr pkt, PacketList &writebacks)
{
    // Only requests that use the stored data. Writebacks carry a full
    // line and replace it.
    if (!(pkt->isRead() || pkt->isWrite()) || pkt->isEviction() ||
        pkt->cmd == MemCmd::WriteClean) {
        return;
    }

    lastAccessTick = curTick();

    Addr blk_addr = pkt->getBlockAddr(cache.blkSize);
    CacheBlk *blk = cache.tags->findBlock({blk_addr, pkt->isSecure()});

    if (!blk) {
        return;
    }

    // An inline refresh first reads the line; with less life left than
    // that read takes, the data is gone by the time it is read
    const Tick drt = cache.tags->getDRT();
    const bool expired = cache.tags->isExpired(blk);
    const bool too_late = !expired && inlineApplies(pkt) &&
        drt - age(blk) < cache.cyclesToTicks(inlineCycles);

    if (!expired && !too_late) {
        return;
    }
    if (too_late) {
        stats.inlineTooLate++;
    }

    const bool dirty = blk->isSet(CacheBlk::DirtyBit);

    // The block has a request in flight (O3): invalidating it would break
    // the MSHR, so it is served as usual (backdoor, counted)
    if (cache.mshrQueue.findMatch(blk_addr, pkt->isSecure()) ||
        cache.writeBuffer.findMatch(blk_addr, pkt->isSecure())) {
        stats.expiredInflightHits++;
        if (dirty) {
            violation(blk, "in-flight access");
        }
        return;
    }

    stats.retentionMisses++;

    // Died recently: between 100% and 120% of DRT
    if (age(blk) < drt * 1.2) {
        stats.expiredHitPot++;
    }

    // Lazy writeback of a dirty block (fallback, counted), then
    // invalidate so the access misses
    if (dirty) {
        violation(blk, "access");
        if (pkt->isWrite()) {
            stats.lazyWritebacksWrite++;
        } else {
            stats.lazyWritebacksRead++;
        }
        PacketPtr wb_packet = cache.writebackBlk(blk);
        if (wb_packet) {
            writebacks.push_back(wb_packet);
        }
    }

    cache.invalidateBlock(blk);
}

void
RetentionManager::onAccessHit(CacheBlk *blk)
{
    blk->setLastTouchTick(curTick());

    if (blk->getWasOppRefreshed()) {
        stats.refreshedBlockUtility++;
        blk->setWasOppRefreshed(false);
    }
}

bool
RetentionManager::inlineApplies(PacketPtr pkt) const
{
    return pkt->isWrite() ? inlineOnWrite : inlineOnRead;
}

Cycles
RetentionManager::hitDelay(PacketPtr pkt, CacheBlk *blk)
{
    Cycles delay(0);

    // A refresh occupies the data array: wait for it
    if (dataBusyUntil > curTick()) {
        delay = cache.ticksToCycles(dataBusyUntil - curTick());
        stats.refreshStalledHits++;
        stats.refreshStallCycles += delay;
    }

    if (!inlineApplies(pkt)) {
        return delay;
    }

    // Too little life left for the daemon to come back in time: this hit
    // recharges the line (read, merge a store, write the whole line)
    const Tick drt = cache.tags->getDRT();
    const Tick a = age(blk);
    if (drt - a >= tGuard) {
        return delay;
    }

    if (pkt->isWrite()) {
        stats.refreshInlineWrite++;
        if (!blk->isSet(CacheBlk::DirtyBit)) {
            stats.cleanToDirtyInDanger++;
        }
    } else {
        stats.refreshInlineRead++;
    }
    stats.ageAtInlineRefresh.sample(double(a) / double(drt));
    stats.inlineRefreshCycles += inlineCycles;

    DPRINTF(Cache, "Inline refresh: %s block %#lx age %lu\n",
            pkt->isWrite() ? "write" : "read", blk->getTag(), a);

    occupyDataArray();
    blk->setFullChargeTick(curTick());

    return delay + inlineCycles;
}

void
RetentionManager::occupyDataArray()
{
    // Full-line read + write; refreshes are serialized on the data array
    Tick start = std::max(curTick(), dataBusyUntil);
    dataBusyUntil = start + cache.cyclesToTicks(lineCycles);
}

void
RetentionManager::onFullLineWrite(CacheBlk *blk)
{
    blk->setFullChargeTick(curTick());
    blk->setLastTouchTick(curTick());
}

void
RetentionManager::onEvict(CacheBlk *blk)
{
    // The temp block is outside the tags and never charged
    if (blk == cache.tempBlock || !blk->isValid() ||
        !cache.tags->isExpired(blk)) {
        return;
    }

    if (blk->isSet(CacheBlk::DirtyBit)) {
        stats.expiredDirtyEvictions++;
        violation(blk, "eviction");
    } else {
        stats.expiredCleanEvictions++;
    }
}

void
RetentionManager::onFunctional(CacheBlk *blk)
{
    if (!blk->isValid() || !cache.tags->isExpired(blk)) {
        return;
    }

    stats.expiredFunctional++;
    if (blk->isSet(CacheBlk::DirtyBit)) {
        violation(blk, "functional access");
    }
}

void
RetentionManager::onSnoop(CacheBlk *blk)
{
    if (!blk->isValid() || !cache.tags->isExpired(blk)) {
        return;
    }

    stats.expiredSnoops++;
    if (blk->isSet(CacheBlk::DirtyBit)) {
        violation(blk, "snoop");
    }
}

bool
RetentionManager::dataIdle() const
{
    return dataBusyUntil <= curTick() &&
        curTick() >= lastAccessTick + cache.clockPeriod();
}

std::vector<CacheBlk *>
RetentionManager::topMRUBlks(unsigned set, unsigned n) const
{
    std::vector<CacheBlk *> blks;
    for (CacheBlk *blk : cache.tags->getSetBlks(set)) {
        if (blk->isValid()) {
            blks.push_back(blk);
        }
    }

    std::stable_sort(blks.begin(), blks.end(),
        [](const CacheBlk *a, const CacheBlk *b) {
            return a->getLastTouchTick() > b->getLastTouchTick();
        });

    if (blks.size() > n) {
        blks.resize(n);
    }
    return blks;
}

void
RetentionManager::refresh(CacheBlk *blk, bool force)
{
    const Tick drt = cache.tags->getDRT();

    DPRINTF(Cache, "RefreshDaemon: %s block %#lx age %lu\n",
            force ? "FORCE" : "OPP", blk->getTag(), age(blk));

    if (force) {
        stats.forceRefresh++;
        stats.ageAtForceRefresh.sample(double(age(blk)) / double(drt));
    } else {
        stats.oppRefresh++;
    }
    stats.totalRefreshes++;

    occupyDataArray();

    blk->setFullChargeTick(curTick());
    blk->setWasOppRefreshed(true);
}

void
RetentionManager::maybeRefresh(CacheBlk *blk)
{
    const Tick drt = cache.tags->getDRT();
    const Tick a = age(blk);

    // Would expire before the daemon comes back: must refresh now
    if (drt - a < tGuard) {
        refresh(blk, true);
    } else if (a >= oppAge * double(drt) && dataIdle()) {
        refresh(blk, false);
    }
}

void
RetentionManager::processRefreshEvent()
{
    BaseTags *tags = cache.tags;

    DPRINTF(Cache, "RefreshDaemon: set %d\n", refreshSetIdx);

    if (refreshDirtyDaemon) {
        for (CacheBlk *blk : tags->getSetBlks(refreshSetIdx)) {
            if (!blk->isValid() || !blk->isSet(CacheBlk::DirtyBit)) {
                continue;
            }

            if (tags->isExpired(blk)) {
                // Refreshing revives lost data (backdoor, counted)
                stats.daemonRevivedDirty++;
                violation(blk, "daemon visit");
                refresh(blk, true);
                continue;
            }

            maybeRefresh(blk);
        }
    }

    if (topMRU > 0) {
        for (CacheBlk *blk : topMRUBlks(refreshSetIdx, topMRU)) {
            // Refreshed by the dirty loop above in this same step
            if (blk->getFullChargeTick() == curTick()) {
                continue;
            }
            // A clean block that already expired has nothing to save
            if (tags->isExpired(blk)) {
                continue;
            }
            maybeRefresh(blk);
        }
    }

    refreshSetIdx = (refreshSetIdx + 1) % tags->getNumSets();
    if (refreshSetIdx == 0) {
        stats.daemonSweepCycles.sample(
            cache.ticksToCycles(curTick() - sweepStart));
        sweepStart = curTick();
    }

    // Next set after scanning this one and finishing its refreshes
    Cycles wait = scanCycles;
    if (dataBusyUntil > curTick()) {
        wait = std::max(wait, cache.ticksToCycles(dataBusyUntil - curTick()));
    }
    cache.schedule(refreshEvent, cache.clockEdge(wait));
}

RetentionManager::RetentionStats::RetentionStats(RetentionManager &_mgr,
                                                 BaseCache &cache)
    : statistics::Group(&cache, "retention"),
      mgr(_mgr),
      ADD_STAT(retentionMisses, statistics::units::Count::get(),
               "accesses that found their block expired and missed"),
      ADD_STAT(lazyWritebacksRead, statistics::units::Count::get(),
               "lazy writebacks of expired dirty blocks, by a read"),
      ADD_STAT(lazyWritebacksWrite, statistics::units::Count::get(),
               "lazy writebacks of expired dirty blocks, by a write"),
      ADD_STAT(lazyWritebacks, statistics::units::Count::get(),
               "all lazy writebacks of expired dirty blocks"),
      ADD_STAT(expiredInflightHits, statistics::units::Count::get(),
               "expired blocks served because a request was in flight"),
      ADD_STAT(expiredDirtyEvictions, statistics::units::Count::get(),
               "expired dirty blocks written back on eviction"),
      ADD_STAT(expiredCleanEvictions, statistics::units::Count::get(),
               "expired clean blocks evicted"),
      ADD_STAT(expiredFunctional, statistics::units::Count::get(),
               "functional accesses that used an expired block"),
      ADD_STAT(expiredSnoops, statistics::units::Count::get(),
               "snoops that used an expired block"),
      ADD_STAT(daemonRevivedDirty, statistics::units::Count::get(),
               "expired dirty blocks refreshed by the daemon"),
      ADD_STAT(invariantViolations, statistics::units::Count::get(),
               "dirty blocks found expired (data lost)"),
      ADD_STAT(oppRefresh, statistics::units::Count::get(),
               "daemon refreshes done while the data array was idle"),
      ADD_STAT(forceRefresh, statistics::units::Count::get(),
               "daemon refreshes forced by the retention guarantee"),
      ADD_STAT(totalRefreshes, statistics::units::Count::get(),
               "all daemon refreshes (one line read + one write each)"),
      ADD_STAT(ageAtForceRefresh, statistics::units::Ratio::get(),
               "block age at a forced refresh, as a fraction of DRT"),
      ADD_STAT(refreshStalledHits, statistics::units::Count::get(),
               "hits delayed because a refresh occupied the data array"),
      ADD_STAT(refreshStallCycles, statistics::units::Cycle::get(),
               "cycles hits waited for refreshes"),
      ADD_STAT(refreshInlineWrite, statistics::units::Count::get(),
               "inline refreshes triggered by a store hit"),
      ADD_STAT(refreshInlineRead, statistics::units::Count::get(),
               "inline refreshes triggered by a read hit"),
      ADD_STAT(ageAtInlineRefresh, statistics::units::Ratio::get(),
               "block age at an inline refresh, as a fraction of DRT"),
      ADD_STAT(cleanToDirtyInDanger, statistics::units::Count::get(),
               "stores that made a clean block dirty inside T_guard"),
      ADD_STAT(inlineRefreshCycles, statistics::units::Cycle::get(),
               "cycles added to hits by inline refreshes"),
      ADD_STAT(inlineTooLate, statistics::units::Count::get(),
               "hits with too little life left for an inline refresh"),
      ADD_STAT(daemonSweepCycles, statistics::units::Cycle::get(),
               "duration of each full daemon sweep"),
      ADD_STAT(tSweepCycles, statistics::units::Cycle::get(),
               "worst-case sweep time the retention guarantee assumes"),
      ADD_STAT(tGuardCycles, statistics::units::Cycle::get(),
               "force-refresh threshold on a block's remaining life"),
      ADD_STAT(expiredHitPot, statistics::units::Count::get(),
               "expired blocks accessed within 1.2 DRT of their charge"),
      ADD_STAT(refreshedBlockUtility, statistics::units::Count::get(),
               "hits on blocks the daemon refreshed")
{
}

void
RetentionManager::RetentionStats::regStats()
{
    statistics::Group::regStats();

    lazyWritebacks = lazyWritebacksRead + lazyWritebacksWrite;
    ageAtForceRefresh.init(0.0, 1.0, 0.02);
    ageAtInlineRefresh.init(0.0, 1.0, 0.02);
    daemonSweepCycles.init(20);
    tSweepCycles.functor([this] {
        return double(mgr.tSweep) / double(mgr.cache.clockPeriod());
    });
    tGuardCycles.functor([this] {
        return double(mgr.tGuard) / double(mgr.cache.clockPeriod());
    });
}

} // namespace gem5
