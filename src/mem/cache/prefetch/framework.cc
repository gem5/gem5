#include "mem/cache/prefetch/framework.hh"
#include <algorithm>
#include <set>
#include "mem/cache/base.hh"
#include "params/TPCComponent.hh"
#include "params/FrameworkQueue.hh"
#include "params/TPCController.hh"

namespace gem5::prefetch {
unsigned CacheView::blockSize() const { return cache->getBlockSize(); }
bool CacheView::contains(Addr a, bool s) const { return cache->inCache(a, s); }
bool CacheView::pending(Addr a, bool s) const {
    return cache->inMissQueue(a & ~(Addr(blockSize()) - 1), s);
}
TPCComponent::TPCComponent(const TPCComponentParams &p)
    : Base(p), observeLevels(p.observe_levels) {
    fatal_if(accountingSource < 1 || accountingSource > 3,
             "TPC component source must be 1=T2, 2=P1, 3=C1");
    for (int level : observeLevels)
        fatal_if(level < 1 || level > 3, "Invalid observation layer");
}
void TPCComponent::attach(TPCController *c) {
    fatal_if(controller, "Component already belongs to a controller");
    controller = c;
    fatal_if(accounting && accounting != c->accounting, "Accounting object mismatch");
    accounting = c->accounting;
}
void TPCComponent::setCache(BaseCache *) { fatal("Components attach through TPCController"); }
void TPCComponent::notify(const PacketPtr &, const PrefetchInfo &) { fatal("Use component events"); }
void TPCComponent::notifyFill(const PacketPtr &) { fatal("Use component fill event"); }
PacketPtr TPCComponent::getPacket() { fatal("Components cannot issue packets directly"); }
bool TPCComponent::observes(int l) const {
    return std::find(observeLevels.begin(), observeLevels.end(), l) != observeLevels.end();
}
void TPCComponent::observe(const AccessEvent &e) {
    fatal_if(stage != Stage::Idle, "Reentrant component callback");
    stage = Stage::Observe;
    accounting->count("observations", source());
    onObserve(e); stage = Stage::Idle;
}
void TPCComponent::train(const AccessEvent &e) {
    fatal_if(stage != Stage::Idle, "Reentrant component callback");
    stage = Stage::Train;
    accounting->count("trainingEvents", source());
    onTrain(e); stage = Stage::Idle;
}
bool TPCComponent::recognizes(const AccessEvent &e) {
    fatal_if(stage != Stage::Idle, "Reentrant component callback");
    stage = Stage::Recognize;
    bool recognized = canHandle(e);
    stage = Stage::Idle;
    if (recognized) accounting->count("recognitionHits", source());
    return recognized;
}
void TPCComponent::predict(const AccessEvent &e) {
    fatal_if(stage != Stage::Idle || !controller, "Invalid prediction dispatch");
    stage = Stage::Predict;
    accounting->count("predictionCalls", source());
    onPredict(e, SubmissionContext(e, this)); stage = Stage::Idle;
}
void TPCComponent::cpuLoad(const CPULoadEvent &e) {
    fatal_if(stage != Stage::Idle || !controller, "Invalid CPU load dispatch");
    // Real CPU request, validated by the controller; no fabricated cache event.
    stage = Stage::Predict;
    onCPULoad(e, SubmissionContext(e, this));
    stage = Stage::Idle;
}
void TPCComponent::fill(const FillEvent &e) {
    fatal_if(stage != Stage::Idle, "Reentrant fill callback");
    // Fill callbacks may use a previously issued SubmissionContext capability.
    onFill(e);
}
std::vector<SubmissionResult> TPCComponent::submitCandidates(
    const SubmissionContext &context, const std::vector<Candidate> &candidates) {
    fatal_if(!controller || context.owner != this ||
             (stage != Stage::Idle && stage != Stage::Predict),
             "Submission requires a prediction capability; observe/train cannot submit");
    return controller->submit(this, context, candidates);
}
CacheView TPCComponent::cacheView(int level) const {
    fatal_if(!controller, "Component has no controller");
    return controller->cacheView(level);
}
int TPCComponent::instructionState(const InstructionSource &s) const {
    return controller->instructionState(s, source());
}
bool TPCComponent::updateInstructionState(const InstructionSource &s, int expected, int state) {
    return controller->updateInstructionState(s, source(), expected, state);
}
FrameworkQueue::FrameworkQueue(const FrameworkQueueParams &p) : Queued(p) {}
void FrameworkQueue::queueReady() {
    Tick next = nextPrefetchReadyTime();
    if (cache && next != MaxTick)
        cache->schedMemSideSendEvent(std::max(next, clockEdge()));
}
void FrameworkQueue::enqueue(const RequestPtr &req, const Candidate &candidate,
                             const AccountingPtr &record) {
    // A temporary owned read packet supplies PrefetchInfo metadata only.
    Packet packet(req, MemCmd::ReadReq);
    PrefetchInfo original(&packet, req->getPaddr(), true);
    PrefetchInfo info(original, blockAddress(candidate.address));
    info.accountingRecord = record;
    if (!candidate.virtualAddress && !cache->system->isMemAddr(info.getAddr())) {
        accounting->drop(record, "dropAddress"); return;
    }
    // Ready entries can contain either physical predictions or translated VAs.
    // Compare physical candidates with the actual packet PA, and virtual ones
    // only with entries retaining a translation request. Equal numeric VA/PA
    // values alone do not identify the same request.
    auto duplicate = [&](std::list<DeferredPacket> &queue) {
        for (auto &entry : queue) {
            bool match = candidate.virtualAddress ?
                (entry.translationRequest && entry.pfInfo.sameAddr(info)) :
                (entry.pkt && entry.pkt->getAddr() == info.getAddr() &&
                 entry.pfInfo.isSecure() == info.isSecure());
            if (!match) continue;
            statsQueued.pfBufferHit++;
            if (entry.priority < candidate.priority) {
                entry.priority = candidate.priority;
                queue.sort([](const DeferredPacket &a, const DeferredPacket &b) { return a > b; });
                queueReady();
            }
            return true;
        }
        return false;
    };
    if (queueFilter && (duplicate(pfq) || duplicate(pfqMissingTranslation))) {
        accounting->drop(record, "dropQueueDuplicate"); return;
    }
    if (candidate.virtualAddress) {
        if (!mmu || !req->hasContextId() || !req->hasPC()) {
            accounting->drop(record, "dropNoTranslation"); return;
        }
        DeferredPacket deferred(this, info, 0, candidate.priority);
        deferred.setTranslationRequest(createPrefetchRequest(info.getAddr(), info, &packet));
        deferred.tc = cache->system->threads[req->contextId()];
        addToQueue(pfqMissingTranslation, deferred);
        processMissingTranslations(queueSize);
    } else {
        // This address is explicitly physical. Do not infer a VA by adding a
        // physical stride across pages (pages need not be contiguous).
        if (cacheSnoop && (inCache(info.getAddr(), info.isSecure()) ||
                          inMissQueue(info.getAddr(), info.isSecure()))) {
            accounting->drop(record, "dropSnoop"); return;
        }
        DeferredPacket deferred(this, info, 0, candidate.priority);
        deferred.createPkt(info.getAddr(), blkSize, requestorId, tagPrefetch,
                           curTick() + clockPeriod() * latency);
        addToQueue(pfq, deferred);
    }
}
TPCController::TPCController(const TPCControllerParams &p)
    : Base(p), routes(p.routes.begin(), p.routes.end()),
      components(p.components.begin(), p.components.end()) {
    fatal_if(!accounting || routes.size() != 3, "Controller needs accounting and three routes");
    std::set<int> sources;
    for (auto component : components) {
        fatal_if(!sources.insert(component->source()).second, "Duplicate component source");
        component->attach(this);
    }
    for (auto route : routes) {
        fatal_if(route->accounting != accounting, "Route accounting mismatch");
    }
}
void TPCController::bindCache(int level, BaseCache *c) {
    fatal_if(initialized || level < 1 || level > 3 || caches[level], "Invalid hierarchy binding");
    caches[level] = c;
    // L1's route is hidden behind the controller's gem5 packet interface.
    if (level == 1) routes[0]->setCache(c);
}
void TPCController::bindInstructionCache(BaseCache *cache) {
    fatal_if(initialized || instructionCache, "Duplicate instruction cache binding");
    instructionCache = cache;
    cache->addInstructionObserver(this);
}
void TPCController::instructionInvalidated(const InstructionSource &s) {
    for (auto *component : components) component->onInstructionInvalidated(s);
}
void TPCController::regProbeListeners() {
    for (int level = 1; level <= 3; ++level) {
        fatal_if(!caches[level], "All three data cache levels must be bound");
        caches[level]->addPrefetchObserver(this);
    }
    initialized = true;
}
PacketPtr TPCController::getPacket() { return routes[0]->getPacket(); }
Tick TPCController::nextPrefetchReadyTime() const { return routes[0]->nextPrefetchReadyTime(); }
CacheView TPCController::cacheView(int level) const {
    fatal_if(level < 1 || level > 3 || !caches[level], "Unbound cache layer");
    return CacheView(caches[level]);
}
void TPCController::cacheAccess(const AccessEvent &e) {
    routes.at(e.level - 1)->demand(e.address, e.secure);
    accounting->count("accessObservations", e.level);
    if (e.primaryMiss) accounting->count("primaryMissObservations", e.level);
    if (!e.hit && !e.primaryMiss) accounting->count("mergedMissObservations", e.level);
    for (auto component : components)
        if (component->observes(e.level)) component->observe(e);
    onAccess(e);
}
void TPCController::cacheFill(const FillEvent &e) {
    for (auto component : components)
        if (component->observes(e.level)) component->fill(e);
}
std::vector<SubmissionResult> TPCController::submit(TPCComponent *component,
    const SubmissionContext &context, const std::vector<Candidate> &candidates) {
    fatal_if(!initialized || context.owner != component, "Invalid submission capability");
    // Validate the entire batch before generating any records.
    for (const auto &candidate : candidates)
        fatal_if(candidate.target < 1 || candidate.target > 3, "Invalid candidate target");
    std::array<size_t, 3> counts{}, used{}, allowed{};
    for (const auto &candidate : candidates) counts[candidate.target - 1]++;
    for (int i = 0; i < 3; ++i) allowed[i] = routes[i]->permitted(counts[i]);
    std::vector<SubmissionResult> result;
    for (const auto &candidate : candidates) {
        auto record = accounting->generate(component->source(), candidate.target);
        if (!validSource(instructionSource(context.request)) ||
            !allowCandidate(*component, candidate)) {
            accounting->drop(record, "dropController");
            result.push_back(SubmissionResult::ControllerRejected); continue;
        }
        int index = candidate.target - 1;
        if (used[index]++ >= allowed[index]) {
            accounting->drop(record, "dropThrottle");
            result.push_back(SubmissionResult::Throttled); continue;
        } else {
            routes[index]->enqueue(context.request, candidate, record);
        }
        result.push_back(SubmissionResult::Transport);
    }
    return result;
}
}

namespace gem5::prefetch {
void TPCController::cpuLoad(const CPULoadEvent &e) {
    if (!e.request || !e.request->hasPC() || !e.request->hasContextId() ||
        !validSource(instructionSource(e.request))) return;
    for (auto *c : components) if (c->source() == 1) c->cpuLoad(e);
}
void TPCController::cpuBranch(const CPUBranchEvent &e) {
    for (auto *c : components) c->onCPUBranch(e);
}
void TPCController::cpuComplete(const CPUCompleteEvent &e) {
    for (auto *c : components) c->onCPUComplete(e);
}
void TPCController::cpuRetire(const CPUIdentity &e) {
    for (auto *c : components) c->onCPURetire(e);
}
void TPCController::cpuSquash(InstSeqNum boundary) {
    for (auto *c : components) c->onCPUSquash(boundary);
}
}

namespace gem5::prefetch {
size_t TPCComponent::queuedCandidates(int level) const {
    fatal_if(!controller, "Component has no controller");
    return controller->queuedCandidates(level);
}
size_t TPCController::queuedCandidates(int level) const {
    fatal_if(level<1 || level>3, "Invalid candidate queue layer");
    return routes.at(level-1)->occupancy();
}
}
