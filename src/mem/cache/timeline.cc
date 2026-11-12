#include "mem/cache/timeline.hh"

#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/replacement_policies/lru_rp.hh"
#include "mem/cache/retention/retention_manager.hh"
#include "mem/cache/tags/base.hh"
#include "sim/core.hh"

namespace gem5
{

CacheTimeline::CacheTimeline(BaseCache &c, Cycles p, Cycles live, bool l)
    : cache(c), period(p), liveCycles(live), lru(l),
      event([this] { tick(); }, c.name() + ".timeline", false,
            Event::Stat_Event_Pri)
{
    // dump: before the stats are written (so before a following reset);
    // reset: after the counters were zeroed
    statistics::registerDumpCallback([this] { row("dump"); });
    statistics::registerResetCallback([this] { row("reset"); });
    registerExitCallback([this] { close(); });
}

void
CacheTimeline::start()
{
    out = simout.create(cache.name() + ".timeline.csv.gz", false, false);
    ccprintf(*out->stream(), "tick,event,hits,mshrMisses,wbOut,wbDirtyIn,"
             "wbCleanIn,valid,live,dirty,refreshed,drops,retMiss,tokens\n");
    row("start");
    cache.schedule(event, cache.clockEdge(period));
}

void
CacheTimeline::tick()
{
    row("period");
    cache.schedule(event, cache.clockEdge(period));
}

void
CacheTimeline::row(const char *ev)
{
    if (!out)
        return;
    auto &st = cache.stats;
    const double hits = st.demandHits.total();
    const double mshr = st.demandMshrMisses.total();
    const double wb_out = st.writebacks.total();
    const double wbd = st.cmd[MemCmd::WritebackDirty]->accesses.total();
    const double wbc = st.cmd[MemCmd::WritebackClean]->accesses.total();
    const Tick now = curTick();
    const Tick live_ticks = cache.cyclesToTicks(liveCycles);
    long valid = 0, live = 0, dirty = 0;
    cache.tags->forEachBlk([&](CacheBlk &blk) {
        if (!blk.isValid())
            return;
        valid++;
        if (blk.isSet(CacheBlk::DirtyBit))
            dirty++;
        if (lru && replacement_policy::LRU::lastTouch(
                       blk.replacementData) + live_ticks >= now)
            live++;
    });
    double refreshed = 0, drops = 0, ret_miss = 0, tokens = -1;
    if (cache.retention)
        cache.retention->timelineCounters(refreshed, drops, ret_miss,
                                          tokens);
    ccprintf(*out->stream(), "%d,%s,%.0f,%.0f,%.0f,%.0f,%.0f,%d,%d,%d,"
             "%.0f,%.0f,%.0f,%.0f\n", now, ev, hits, mshr, wb_out, wbd, wbc,
             valid, lru ? live : -1, dirty, refreshed, drops, ret_miss,
             tokens);
}

void
CacheTimeline::close()
{
    if (out) {
        // gem5 runs this (doExitCleanup) before its final atexit stats
        // dump, with nothing simulated in between: the exit row holds the
        // final dump's values
        row("exit");
        out->stream()->flush();
        simout.close(out);
        out = nullptr;
    }
}

} // namespace gem5
