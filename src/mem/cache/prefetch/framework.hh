// Project public component/transport contract. Algorithms live in subclasses.
#ifndef __MEM_PREFETCH_FRAMEWORK_HH__
#define __MEM_PREFETCH_FRAMEWORK_HH__
#include <array>
#include "mem/cache/prefetch/queued.hh"
#include "mem/cache/base.hh"
#include "mem/cache/prefetch/framework_events.hh"
namespace gem5 {
class BaseCache;
struct TPCComponentParams;
struct FrameworkQueueParams;
struct TPCControllerParams;
namespace prefetch {
class TPCController;
class TPCComponent;
struct Candidate {
    Addr address;
    int target;
    int32_t priority = 0;
    bool virtualAddress = false;
};
class SubmissionContext {
    friend class TPCComponent;
    friend class TPCController;
    RequestPtr request;
    const TPCComponent *owner = nullptr;
    explicit SubmissionContext(const AccessEvent &event, const TPCComponent *o)
        : request(std::make_shared<Request>(*event.request)), owner(o) {}
  public:
    SubmissionContext(const SubmissionContext &) = default;
};
enum class SubmissionResult { Transport, ControllerRejected, Throttled };
class CacheView {
    const BaseCache *cache;
  public:
    explicit CacheView(const BaseCache *c) : cache(c) {}
    unsigned blockSize() const;
    bool contains(Addr, bool secure = false) const;
    bool pending(Addr, bool secure = false) const;
};
class TPCComponent : public Base {
    friend class TPCController;
    TPCController *controller = nullptr;
    const std::vector<int> observeLevels;
    enum class Stage { Idle, Observe, Train, Recognize, Predict };
    Stage stage = Stage::Idle;
    void attach(TPCController *);
    // Component instances never own a cache's packet source.
    void notify(const PacketPtr &, const PrefetchInfo &) final;
    void notifyFill(const PacketPtr &) final;
    PacketPtr getPacket() final;
    Tick nextPrefetchReadyTime() const final { return MaxTick; }
    void setCache(BaseCache *) final;
    void regProbeListeners() final {}
  protected:
    virtual void onObserve(const AccessEvent &) {}
    virtual void onTrain(const AccessEvent &) {}
    virtual bool canHandle(const AccessEvent &) const = 0;
    virtual void onPredict(const AccessEvent &, const SubmissionContext &) = 0;
    virtual void onFill(const FillEvent &) {}
    std::vector<SubmissionResult> submitCandidates(
        const SubmissionContext &, const std::vector<Candidate> &);
    CacheView cacheView(int level) const;
  public:
    explicit TPCComponent(const TPCComponentParams &);
    int source() const { return accountingSource; }
    bool observes(int level) const;
    void observe(const AccessEvent &);
    void train(const AccessEvent &);
    bool recognizes(const AccessEvent &);
    void predict(const AccessEvent &);
    void fill(const FillEvent &);
};
class FrameworkQueue : public Queued {
    void calculatePrefetch(const PrefetchInfo &, std::vector<AddrPriority> &) final {}
    void notify(const PacketPtr &, const PrefetchInfo &) final {}
    void regProbeListeners() final {}
    void queueReady() override;
  public:
    explicit FrameworkQueue(const FrameworkQueueParams &);
    void enqueue(const RequestPtr &, const Candidate &, const AccountingPtr &);
    void demand(Addr address, bool secure) { cancelDemand(blockAddress(address), secure); }
    size_t permitted(size_t n) const { return getMaxPermittedPrefetches(n); }
};
class TPCController : public Base, public CacheObserver {
    friend class TPCComponent;
    std::array<BaseCache *, 4> caches{};
    std::vector<FrameworkQueue *> routes;
    bool initialized = false;
    void notify(const PacketPtr &, const PrefetchInfo &) final {}
    void notifyFill(const PacketPtr &) final {}
    void regProbeListeners() override;
    PacketPtr getPacket() final;
    Tick nextPrefetchReadyTime() const final;
    std::vector<SubmissionResult> submit(TPCComponent *, const SubmissionContext &,
                                       const std::vector<Candidate> &);
  protected:
    const std::vector<TPCComponent *> components;
    virtual void onAccess(const AccessEvent &) = 0;
    virtual bool allowCandidate(const TPCComponent &, const Candidate &) const { return true; }
  public:
    explicit TPCController(const TPCControllerParams &);
    void bindCache(int level, BaseCache *);
    void cacheAccess(const AccessEvent &) final;
    void cacheFill(const FillEvent &) final;
    CacheView cacheView(int level) const;
};
}
}
#endif
