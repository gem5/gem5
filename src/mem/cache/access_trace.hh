/**
 * Text trace of a cache's accesses, fills and evictions (access_trace),
 * simulator-only: nothing here changes timing or state. Written from the
 * first stats reset on (the end of the warmup in the SimPoint runs) to
 * <cache>.access_trace.txt.gz, one line per event:
 *   tick set addr type H|M   for an access from above (type R: a demand
 *                            access that does not need write permission,
 *                            W: one that does, U: an upgrade, WB: a
 *                            writeback, CE: a CleanEvict; H / M: gem5's
 *                            hit or miss for R / W, the line present or
 *                            not before the access for the others)
 *   tick set addr FILL       a fill from below allocated the line
 *   tick set addr WBFILL     a writeback from above allocated the line
 *   tick set addr EVICT      the line left by an eviction
 * addr is the line address (hex).
 */

#ifndef __MEM_CACHE_ACCESS_TRACE_HH__
#define __MEM_CACHE_ACCESS_TRACE_HH__

#include <string>

#include "base/output.hh"
#include "base/types.hh"

namespace gem5
{

class AccessTrace
{
  public:
    explicit AccessTrace(const std::string &cache_name);

    /** Tracing has started (after the first stats reset). */
    bool on() const { return out != nullptr; }

    void access(Tick t, unsigned set, Addr addr, const char *type,
                bool hit);

    void event(Tick t, unsigned set, Addr addr, const char *type);

  private:
    void start();
    void close();

    std::string name;
    OutputStream *out = nullptr;
};

} // namespace gem5

#endif // __MEM_CACHE_ACCESS_TRACE_HH__
