/**
 * Cache timeline (timeline_cycles), simulator-only: nothing here changes
 * timing or state. Every timeline_cycles cycles from startup on, and at
 * every stats dump (before it) and reset (after it), one row is appended to
 * <cache>.timeline.csv.gz:
 *   tick,event,hits,mshrMisses,wbOut,wbDirtyIn,wbCleanIn,valid,live,dirty,
 *   refreshed,drops,retMiss,tokens
 * event: start | period | dump | reset | exit (at simulator exit, before
 * gem5's final atexit stats dump: the same values). The counters are the
 * cache's stats since the last stats reset (demandHits, demandMshrMisses,
 * writebacks, WritebackDirty / WritebackClean accesses) and, with a
 * RetentionManager,
 * the counters the power model uses (lines refreshed of the mode: RA
 * raValidRefreshes, the window modes their linesRefreshed), drops (clean +
 * dirty), retention misses and the tail bucket's tokens (-1: no bucket).
 * valid / dirty: valid (dirty) lines now; live: valid lines touched (LRU
 * replacement data) within the last timeline_live_cycles (-1 unless the
 * replacement policy is LRU).
 */

#ifndef __MEM_CACHE_TIMELINE_HH__
#define __MEM_CACHE_TIMELINE_HH__

#include "base/output.hh"
#include "base/types.hh"
#include "sim/cur_tick.hh"
#include "sim/eventq.hh"

namespace gem5
{

class BaseCache;

class CacheTimeline
{
  public:
    CacheTimeline(BaseCache &cache, Cycles period, Cycles live, bool lru);

    /** First row and the periodic event (BaseCache::startup). */
    void start();

  private:
    void row(const char *event);
    void tick();
    void close();

    BaseCache &cache;
    const Cycles period;
    const Cycles liveCycles;
    const bool lru;
    OutputStream *out = nullptr;
    EventFunctionWrapper event;
};

} // namespace gem5

#endif // __MEM_CACHE_TIMELINE_HH__
