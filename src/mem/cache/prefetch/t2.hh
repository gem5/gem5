#ifndef __MEM_CACHE_PREFETCH_T2_HH__
#define __MEM_CACHE_PREFETCH_T2_HH__
#include <functional>
#include <map>
#include <optional>
#include "mem/cache/prefetch/framework.hh"
#include "mem/cache/prefetch/t2_engine.hh"
#include "mem/cache/prefetch/t2_stream.hh"
#include "sim/eventq.hh"
namespace gem5 {
struct T2PrefetcherParams;
namespace prefetch {
class T2Prefetcher : public TPCComponent {
    struct Pending {
        InstructionSource instruction;
        ContextID context = 0;
        Addr pc = 0, ras = 0, address = 0;
        unsigned width = 0, microPC = 0;
        bool rasValid = false, cacheSeen = false;
        Tick cacheTick = 0;
        uint64_t loopGeneration = 0;
    };
    using DynamicKey = std::pair<ContextID, InstSeqNum>;
    struct Entry {
        bool valid = false, pointer = false, lastPrefValid = false;
        uint64_t id = 0, lastUse = 0, loopGeneration = 0;
        Pending key;
        T2DeltaRun run;
        Addr current = 0, desired = 0, lastPref = 0;
        unsigned baseDistance = 1;
        std::optional<SubmissionContext> context;
    };
    const unsigned confirm, reject, provisional, maxDistance, maxPending;
    const unsigned provisionalPerCycle, provisionalQueueLimit;
    const bool trace, serviceLatency;
    const Tick margin;
    T2Loop loop;
    T2Mean latency;
    std::vector<Entry> sit;
    std::map<DynamicKey, Pending> pending;
    uint64_t nextID = 0, useSequence = 0;
    unsigned cursor = 0, provisionalUsed = 0;
    Tick provisionalTick = MaxTick;
    EventFunctionWrapper catchup;
    std::vector<std::function<void(const char *, const T2StreamSnapshot &)>> observers;
    struct Stats : statistics::Group {
        statistics::Scalar cpuLoads, primaryActivations, samples, confirmed, rejected,
            provisionalRequests, candidates, transported, invalidated, evicted,
            replayAttempts, pendingFull, latencySamples, branches, loopSwitches,
            arithmeticDrops, unboundLoads, serviceSamples;
        explicit Stats(statistics::Group *);
    } t2Stats;
    Entry *find(const Pending &);
    const Entry *find(const Pending &) const;
    void invalidate(Entry &, const char *);
    bool ready(const Entry &) const;
    void scheduleWork();
    void emitOne();
    bool nextAddress(const Entry &, Addr &) const;
    void updateTarget(Entry &, const CPULoadEvent &, const SubmissionContext &);
    void syncState(Entry &);
    T2StreamSnapshot snapshot(const Entry &) const;
    void publish(const char *, const Entry &);
  protected:
    void onObserve(const AccessEvent &) override;
    void onFill(const FillEvent &) override;
    void onTrain(const AccessEvent &) override;
    bool canHandle(const AccessEvent &) const override;
    void onPredict(const AccessEvent &, const SubmissionContext &) override {}
    void onInstructionInvalidated(const InstructionSource &) override;
    void onCPULoad(const CPULoadEvent &, const SubmissionContext &) override;
    void onCPUBranch(const CPUBranchEvent &) override;
    void onCPUComplete(const CPUCompleteEvent &) override;
    void onCPURetire(const CPUIdentity &) override;
    void onCPUSquash(InstSeqNum) override;
  public:
    explicit T2Prefetcher(const T2PrefetcherParams &);
    std::optional<T2StreamSnapshot> lookupStream(ContextID, Addr pc, Addr ras,
                                                bool rasValid, unsigned microPC = 0) const;
    bool setPointerSource(T2StreamHandle, bool);
    void addStreamObserver(std::function<void(const char *, const T2StreamSnapshot &)> cb) {
        observers.push_back(std::move(cb));
    }
};
}
}
#endif
