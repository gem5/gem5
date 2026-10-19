#include "mem/cache/retention/reuse_tracker.hh"

#include <string>

#include "base/logging.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/tags/base.hh"
#include "sim/core.hh"

namespace gem5
{

ReuseTracker::ReuseTracker(BaseCache &_cache, bool idle_gaps,
                           bool fill_stats)
    : cache(_cache),
      numSets(_cache.tags->getNumSets()),
      assoc(_cache.tags->getSetBlks(0).size()),
      lastAccess(numSets * assoc, 0),
      hits(numSets * assoc, 0),
      idleGaps(idle_gaps),
      lastAny(idle_gaps ? numSets * assoc : 0, 0),
      fillStats(fill_stats),
      fillTick(fill_stats ? numSets * assoc : 0, 0),
      fillFlags(fill_stats ? numSets * assoc : 0, 0),
      shadow(numSets * assoc),
      stats(*this, _cache)
{
    if (fillStats)
        fillStatsGroup = std::make_unique<FillStats>(&stats);
}

unsigned
ReuseTracker::bin(Tick dt)
{
    const uint64_t us = dt / sim_clock::as_int::us;
    if (us < 64) {
        return us;
    }
    // [64, 128) us is bin 64, ..., [2048, 4096) us bin 69
    unsigned b = 64;
    for (uint64_t lo = 128; lo <= us && b < NumBins - 1; lo *= 2) {
        b++;
    }
    return b;
}

unsigned
ReuseTracker::lineIdx(const CacheBlk *blk) const
{
    return blk->getSet() * assoc + blk->getWay();
}

bool
ReuseTracker::shadowHolds(unsigned set, Addr addr) const
{
    for (unsigned w = 0; w < assoc; w++) {
        const ShadowWay &s = shadow[set * assoc + w];
        if (s.valid && s.addr == addr) {
            return true;
        }
    }
    return false;
}

void
ReuseTracker::shadowAccess(unsigned set, Addr addr, bool insert)
{
    ShadowWay *victim = nullptr;
    for (unsigned w = 0; w < assoc; w++) {
        ShadowWay &s = shadow[set * assoc + w];
        if (s.valid && s.addr == addr) {
            s.lastUse = curTick();
            return;
        }
        // An empty way first, else the least recently used
        if (!victim || (victim->valid &&
                        (!s.valid || s.lastUse < victim->lastUse))) {
            victim = &s;
        }
    }
    if (!insert) {
        return;
    }

    if (victim->valid) {
        auto it = liveGhosts.find(victim->addr);
        if (it != liveGhosts.end()) {
            liveGhosts.erase(it);
            deadGhosts.insert(victim->addr);
            stats.ghostsKilledByReplacement++;
        }
    }
    victim->valid = true;
    victim->addr = addr;
    victim->lastUse = curTick();
}

void
ReuseTracker::onAccess(PacketPtr pkt, CacheBlk *blk)
{
    if (pkt->req->isCacheMaintenance()) {
        return;
    }

    const Addr addr = pkt->getBlockAddr(cache.blkSize);
    const unsigned set = cache.tags->getSetIndex({addr, pkt->isSecure()});
    const bool demand = pkt->needsResponse();

    // Idle gap of a line this access finds (any kind of access)
    if (idleGaps && blk && blk != cache.tempBlock && blk->isValid()) {
        const unsigned i = lineIdx(blk);
        stats.idleGapAtAccess[bin(curTick() - lastAny[i])]++;
        lastAny[i] = curTick();
        if (pkt->cmd == MemCmd::CleanEvict) {
            stats.idleGapSamplesCleanEvict++;
        } else if (demand) {
            stats.idleGapSamplesDemand++;
        } else {
            stats.idleGapSamplesWriteback++;
        }
    }

    // reuse_fill_stats: a CleanEvict from above uses the line it finds
    if (fillStats && blk && blk != cache.tempBlock && blk->isValid() &&
        pkt->cmd == MemCmd::CleanEvict) {
        fillUse(lineIdx(blk));
    }

    if (demand && !blk) {
        auto it = liveGhosts.find(addr);
        if (it != liveGhosts.end()) {
            stats.srcCausedMisses++;
            stats.timeSinceDropAtMiss[bin(curTick() - it->second)]++;
            liveGhosts.erase(it);
        } else if (deadGhosts.erase(addr)) {
            stats.ghostMissesAfterReplacement++;
        }
    }

    // The SRAM allocates on a demand miss and on a writeback from above;
    // any access that finds the line updates the LRU order
    const bool allocates = demand || pkt->isWriteback() ||
        pkt->cmd == MemCmd::WriteClean;
    shadowAccess(set, addr, allocates);
}

void
ReuseTracker::fillUse(unsigned i)
{
    if ((fillFlags[i] & FillDemand) && !(fillFlags[i] & FillUsed)) {
        fillStatsGroup->fillToFirstUse[bin(curTick() - fillTick[i])]++;
        fillFlags[i] |= FillUsed;
    }
}

void
ReuseTracker::onDemandHit(CacheBlk *blk)
{
    const unsigned i = lineIdx(blk);
    stats.timeSinceLastAccess[bin(curTick() - lastAccess[i])]++;
    lastAccess[i] = curTick();
    hits[i]++;
    if (fillStats && (fillFlags[i] & FillDemand)) {
        if (!(fillFlags[i] & FillHit)) {
            fillStatsGroup->fillToFirstHit[bin(curTick() - fillTick[i])]++;
            fillFlags[i] |= FillHit;
        }
        fillUse(i);
    }
}

void
ReuseTracker::onWriteback(CacheBlk *blk)
{
    lastAccess[lineIdx(blk)] = curTick();
    // A writeback into a demand-filled line uses it (a writeback that
    // allocates the line is not a demand fill: no flags)
    if (fillStats)
        fillUse(lineIdx(blk));
}

void
ReuseTracker::onAllocate(CacheBlk *blk, bool fill)
{
    const unsigned i = lineIdx(blk);
    lastAccess[i] = curTick();
    hits[i] = 0;
    if (idleGaps) {
        lastAny[i] = curTick();
    }
    if (fillStats) {
        fillTick[i] = curTick();
        fillFlags[i] = fill ? FillDemand : 0;
        if (fill)
            fillStatsGroup->demandFills++;
        else
            fillStatsGroup->wbAllocs++;
    }

    const Addr addr = cache.regenerateBlkAddr(blk);
    if (liveGhosts.erase(addr)) {
        stats.ghostsRefilledOther++;
    }
    deadGhosts.erase(addr);
}

void
ReuseTracker::onRemove(CacheBlk *blk)
{
    if (blk == cache.tempBlock || !blk->isValid()) {
        return;
    }

    const bool zero = hits[lineIdx(blk)] == 0;
    if (fillStats) {
        uint8_t &f = fillFlags[lineIdx(blk)];
        if (f & FillDemand) {
            FillStats &fs = *fillStatsGroup;
            if (reason == Reason::Replacement) {
                fs.fillsEvicted++;
                if (!(f & FillHit))
                    fs.fillsEvictedNeverHit++;
                if (!(f & FillUsed))
                    fs.fillsEvictedNeverUsed++;
            } else {
                fs.fillsRemovedOther++;
            }
        }
        f = 0;
    }
    switch (reason) {
      case Reason::Replacement:
        stats.linesEvicted++;
        if (idleGaps) {
            stats.idleGapAtEviction[
                bin(curTick() - lastAny[lineIdx(blk)])]++;
        }
        if (zero) {
            stats.zeroHitEvictions++;
        }
        break;
      case Reason::Drop:
        stats.linesDropped++;
        if (zero) {
            stats.zeroHitDrops++;
        }
        break;
      default:
        stats.linesInvalidatedOther++;
        if (zero) {
            stats.zeroHitOther++;
        }
        break;
    }

    if (reason != Reason::Drop) {
        return;
    }
    const Addr addr = cache.regenerateBlkAddr(blk);
    if (shadowHolds(blk->getSet(), addr)) {
        liveGhosts[addr] = curTick();
        deadGhosts.erase(addr);
        stats.ghostsCreated++;
    } else {
        stats.dropsNotInShadow++;
    }
}

ReuseTracker::ReuseStats::ReuseStats(ReuseTracker &_tracker,
                                     BaseCache &cache)
    : statistics::Group(&cache, "reuse"),
      tracker(_tracker),
      ADD_STAT(timeSinceLastAccess, statistics::units::Count::get(),
               "demand hits by time since the line's last access (fill, "
               "demand hit or writeback); 1 us bins to 64 us, then powers "
               "of two to 4096 us"),
      ADD_STAT(linesEvicted, statistics::units::Count::get(),
               "lines that left by replacement"),
      ADD_STAT(zeroHitEvictions, statistics::units::Count::get(),
               "of those, lines with no demand hit since their fill"),
      ADD_STAT(linesDropped, statistics::units::Count::get(),
               "lines the SRC dropped at their deadline or found expired"),
      ADD_STAT(zeroHitDrops, statistics::units::Count::get(),
               "of those, lines with no demand hit since their fill"),
      ADD_STAT(linesInvalidatedOther, statistics::units::Count::get(),
               "lines invalidated for any other reason"),
      ADD_STAT(zeroHitOther, statistics::units::Count::get(),
               "of those, lines with no demand hit since their fill"),
      ADD_STAT(ghostsCreated, statistics::units::Count::get(),
               "SRC drops of lines an SRAM (shadow tags) still holds"),
      ADD_STAT(dropsNotInShadow, statistics::units::Count::get(),
               "SRC drops of lines the shadow tags had evicted already"),
      ADD_STAT(srcCausedMisses, statistics::units::Count::get(),
               "demand misses to a dropped line the shadow tags still "
               "hold: misses the SRC caused"),
      ADD_STAT(timeSinceDropAtMiss, statistics::units::Count::get(),
               "SRC-caused misses by time since the drop (bins as "
               "timeSinceLastAccess)"),
      ADD_STAT(ghostsKilledByReplacement, statistics::units::Count::get(),
               "dropped lines the shadow tags' replacement evicted"),
      ADD_STAT(ghostMissesAfterReplacement, statistics::units::Count::get(),
               "demand misses to a dropped line after the shadow evicted "
               "it (an SRAM misses too)"),
      ADD_STAT(ghostsRefilledOther, statistics::units::Count::get(),
               "dropped lines refilled without a demand miss (writeback)"),
      ADD_STAT(ghostsLive, statistics::units::Count::get(),
               "dropped lines the shadow tags still hold, now"),
      ADD_STAT(ghostsLiveAtReset, statistics::units::Count::get(),
               "the same, at the last stats reset"),
      ADD_STAT(idleGapAtAccess, statistics::units::Count::get(),
               "accesses that find the line (demand, writeback, CleanEvict) "
               "by time since the line's previous access of any kind (a "
               "fill starts it); bins as timeSinceLastAccess"),
      ADD_STAT(idleGapSamplesDemand, statistics::units::Count::get(),
               "idleGapAtAccess samples from demand accesses"),
      ADD_STAT(idleGapSamplesWriteback, statistics::units::Count::get(),
               "idleGapAtAccess samples from writebacks from above"),
      ADD_STAT(idleGapSamplesCleanEvict, statistics::units::Count::get(),
               "idleGapAtAccess samples from CleanEvicts from above"),
      ADD_STAT(idleGapAtEviction, statistics::units::Count::get(),
               "replacement evictions by time since the line's last access "
               "of any kind; bins as timeSinceLastAccess")
{
}

void
ReuseTracker::ReuseStats::regStats()
{
    statistics::Group::regStats();

    for (statistics::Vector *v : {&timeSinceLastAccess,
                                  &timeSinceDropAtMiss, &idleGapAtAccess,
                                  &idleGapAtEviction}) {
        v->init(NumBins);
        for (unsigned b = 0; b < 64; b++) {
            v->subname(b, std::to_string(b) + "us");
        }
        for (unsigned b = 64; b < NumBins - 1; b++) {
            v->subname(b, std::to_string(1u << (b - 58)) + "us");
        }
        v->subname(NumBins - 1, "ge4096us");
    }
    ghostsLive.functor([this] {
        return double(tracker.liveGhosts.size());
    });
    ghostsLiveAtReset.functor([this] {
        return double(tracker.ghostsAtReset);
    });
}

void
ReuseTracker::ReuseStats::resetStats()
{
    statistics::Group::resetStats();
    tracker.ghostsAtReset = tracker.liveGhosts.size();
}

ReuseTracker::FillStats::FillStats(statistics::Group *parent)
    : statistics::Group(parent, "fill"),
      ADD_STAT(demandFills, statistics::units::Count::get(),
               "allocations by a fill response (demand miss)"),
      ADD_STAT(wbAllocs, statistics::units::Count::get(),
               "allocations by a writeback from above"),
      ADD_STAT(fillToFirstHit, statistics::units::Count::get(),
               "first demand hits after a demand fill, by time since the "
               "fill; bins as timeSinceLastAccess"),
      ADD_STAT(fillToFirstUse, statistics::units::Count::get(),
               "first uses (demand hit, writeback from above, CleanEvict "
               "from above) after a demand fill, by time since the fill"),
      ADD_STAT(fillsEvicted, statistics::units::Count::get(),
               "demand-filled lines that left by replacement"),
      ADD_STAT(fillsEvictedNeverHit, statistics::units::Count::get(),
               "of those, lines with no demand hit since the fill"),
      ADD_STAT(fillsEvictedNeverUsed, statistics::units::Count::get(),
               "of those, lines with no use since the fill"),
      ADD_STAT(fillsRemovedOther, statistics::units::Count::get(),
               "demand-filled lines removed for any other reason")
{
}

void
ReuseTracker::FillStats::regStats()
{
    statistics::Group::regStats();
    for (statistics::Vector *v : {&fillToFirstHit, &fillToFirstUse}) {
        v->init(NumBins);
        for (unsigned b = 0; b < 64; b++) {
            v->subname(b, std::to_string(b) + "us");
        }
        for (unsigned b = 64; b < NumBins - 1; b++) {
            v->subname(b, std::to_string(1u << (b - 58)) + "us");
        }
        v->subname(NumBins - 1, "ge4096us");
    }
}

} // namespace gem5
