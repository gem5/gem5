#ifndef __MEM_PREFETCH_ACCOUNTING_HH__
#define __MEM_PREFETCH_ACCOUNTING_HH__
#include <array>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include "base/statistics.hh"
#include "mem/cache/prefetch/accounting_record.hh"
#include "mem/packet.hh"
#include "sim/sim_object.hh"

namespace gem5 {
struct PrefetchAccountingParams;
struct PrefetchProvenance : public Extension<Request, PrefetchProvenance> {
    prefetch::AccountingPtr record;
    explicit PrefetchProvenance(prefetch::AccountingPtr r) : record(std::move(r)) {}
    std::unique_ptr<ExtensionBase> clone() const override {
        return std::make_unique<PrefetchProvenance>(record);
    }
};
inline prefetch::AccountingPtr prefetchRecord(const PacketPtr pkt) {
    auto ext = pkt->req->getExtension<PrefetchProvenance>();
    return ext ? ext->record : nullptr;
}
inline bool accountingDemand(const PacketPtr pkt) {
    return !prefetchRecord(pkt) && pkt->isDemand() && !pkt->isEviction() &&
        !pkt->req->isCacheMaintenance() && !pkt->cmd.isSWPrefetch() &&
        !pkt->req->isUncacheable();
}

class PrefetchAccounting : public SimObject {
    using Ptr = prefetch::AccountingPtr;
    struct Metric {
        std::unique_ptr<statistics::Vector> stat;
        std::array<double, 16> values{};
        int size;
    };
    std::map<std::string, Metric> metrics;
    std::unordered_map<uint64_t, std::weak_ptr<prefetch::AccountingRecord>> records;
    uint64_t nextId = 0, epoch = 0;
    const bool trace;
    void metric(const char *name, bool layered = false);
    void finish(const Ptr &, prefetch::Outcome, const char *);
  public:
    explicit PrefetchAccounting(const PrefetchAccountingParams &);
    void beginPhase(const std::string &name);
    void resetStats() override;
    void preDumpStats() override;
    void serialize(CheckpointOut &) const override;
    Ptr generate(int source, int target);
    void count(const std::string &, int source, double value = 1);
    void layer(const std::string &, int source, int level, double value = 1);
    double value(const std::string &, int source) const;
    void queue(const Ptr &);
    void dequeue(const Ptr &);
    void drop(const Ptr &, const char *reason);
    void accept(const Ptr &);
    void fill(const Ptr &, int level, Tick ready);
    void demand(const Ptr &, int level, bool satisfied);
    void remove(const Ptr &, int level, const char *reason);
    void abort(const Ptr &);
    void demandMiss(int level);
};
}
#endif
