// Generic, owning cache observations. No algorithm-specific cache behavior.
#ifndef __MEM_PREFETCH_FRAMEWORK_EVENTS_HH__
#define __MEM_PREFETCH_FRAMEWORK_EVENTS_HH__
#include <vector>
#include "mem/request.hh"
#include "mem/cache/prefetch/accounting_record.hh"
#include "base/types.hh"
namespace gem5::prefetch {
struct AccessEvent {
    uint64_t id = 0;
    int level = -1;
    Tick tick = 0;
    RequestPtr request; // cloned Request, never a borrowed Packet
    Addr address = 0;
    unsigned size = 0;
    bool hit = false, primaryMiss = false, write = false, secure = false;
};
struct FillEvent {
    int level;
    Tick tick;
    Addr address;
    std::vector<uint8_t> data;
    AccountingPtr origin; // same request identity; owning provenance reference
    // Cache timing facts, independent of public prefetch accounting. Available
    // only for successful timing read fills; readiness includes modeled fill time.
    Tick serviceStart = 0, readyTick = 0;
    bool readService = false;
};
class CacheObserver {
  public:
    virtual ~CacheObserver() = default;
    virtual void cacheAccess(const AccessEvent &) = 0;
    virtual void cacheFill(const FillEvent &) = 0;
};
}
#endif
