// Project-wide provenance, separate from gem5's requestor IDs.
#ifndef __MEM_PREFETCH_ACCOUNTING_RECORD_HH__
#define __MEM_PREFETCH_ACCOUNTING_RECORD_HH__
#include <cstdint>
#include <memory>
namespace gem5 {
class PrefetchAccounting;
namespace prefetch {
enum class Outcome { Pending, Useful, Late, Unused, Aborted };
struct AccountingRecord {
    PrefetchAccounting *owner = nullptr;
    uint64_t id = 0, acceptedEpoch = 0, generatedEpoch = 0;
    uint64_t generatedTick = 0, acceptedTick = 0, readyTick = 0;
    uint64_t demandTick = 0;
    int source = -1, target = -1;
    bool queued = false, accepted = false, filled = false;
    bool demandSeen = false;
    Outcome outcome = Outcome::Pending;
};
using AccountingPtr = std::shared_ptr<AccountingRecord>;
}
}
#endif
