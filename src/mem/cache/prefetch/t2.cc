#include "mem/cache/prefetch/t2.hh"
#include <algorithm>
#include "params/T2Prefetcher.hh"
namespace gem5::prefetch {
T2Prefetcher::Stats::Stats(statistics::Group *p) : statistics::Group(p, "t2"),
    ADD_STAT(cpuLoads, statistics::units::Count::get(), "Eligible CPU load attempts"),
    ADD_STAT(primaryActivations, statistics::units::Count::get(), "Primary miss activations"),
    ADD_STAT(samples, statistics::units::Count::get(), "Byte delta samples"),
    ADD_STAT(confirmed, statistics::units::Count::get(), "Confirmed strides"),
    ADD_STAT(rejected, statistics::units::Count::get(), "Non-strided classifications"),
    ADD_STAT(provisionalRequests, statistics::units::Count::get(), "Provisional requests"),
    ADD_STAT(candidates, statistics::units::Count::get(), "Submitted L1D candidates"),
    ADD_STAT(transported, statistics::units::Count::get(), "Candidates passed to transport"),
    ADD_STAT(invalidated, statistics::units::Count::get(), "Invalidated streams"),
    ADD_STAT(evicted, statistics::units::Count::get(), "SIT replacements"),
    ADD_STAT(replayAttempts, statistics::units::Count::get(), "Suppressed repeat attempts"),
    ADD_STAT(pendingFull, statistics::units::Count::get(), "Pending identity capacity skips"),
    ADD_STAT(latencySamples, statistics::units::Count::get(), "Selected timing-estimator samples"),
    ADD_STAT(branches, statistics::units::Count::get(), "Committed branch observations"),
    ADD_STAT(loopSwitches, statistics::units::Count::get(), "Loop register switches"),
    ADD_STAT(arithmeticDrops, statistics::units::Count::get(), "Overflow/address rejections"),
    ADD_STAT(unboundLoads, statistics::units::Count::get(), "Loads before loop recognition"),
    ADD_STAT(serviceSamples, statistics::units::Count::get(), "Full L1 read-miss service samples") {}
T2Prefetcher::T2Prefetcher(const T2PrefetcherParams &p)
    : TPCComponent(p), confirm(p.confirm_deltas), reject(p.reject_changes),
      provisional(p.provisional_deltas), maxDistance(p.max_distance),
      maxPending(p.max_pending), provisionalPerCycle(p.provisional_per_cycle),
      provisionalQueueLimit(p.provisional_queue_limit),
      trace(p.trace), serviceLatency(p.latency_mode=="service"),
      margin(clockPeriod()*p.margin_cycles),
      loop(p.nlpct_entries, p.min_segment_size, p.timing_window),
      latency(p.timing_window), sit(p.sit_entries),
      catchup([this] { emitOne(); }, name()+".catchup"), t2Stats(this)
{
    fatal_if(p.latency_mode!="service" && p.latency_mode!="demand", "Invalid T2 latency_mode");
    fatal_if(source()!=1 || !p.sit_entries || !p.nlpct_entries ||
        !p.timing_window || !maxPending || !confirm || !reject ||
        !provisional || provisional>confirm || !maxDistance || maxDistance>1024 ||
        !provisionalPerCycle || !provisionalQueueLimit || p.observe_levels.size()!=1 || p.observe_levels[0]!=1,
        "Invalid T2 parameters");
}
const T2Prefetcher::Entry *T2Prefetcher::find(const Pending &p) const {
    for (const auto &e : sit)
        if (e.valid && e.key.instruction==p.instruction && e.key.pc==p.pc &&
            e.key.context==p.context && e.key.width==p.width && e.key.microPC==p.microPC && e.key.ras==p.ras && e.key.rasValid==p.rasValid)
            return &e;
    return nullptr;
}
T2Prefetcher::Entry *T2Prefetcher::find(const Pending &p) {
    return const_cast<Entry *>(static_cast<const T2Prefetcher *>(this)->find(p));
}
T2StreamSnapshot T2Prefetcher::snapshot(const Entry &e) const {
    T2StreamSnapshot s;
    s.handle = {e.id, e.id}; s.instruction=e.key.instruction;
    s.context=e.key.context; s.pc=e.key.pc; s.ras=e.key.ras; s.rasValid=e.key.rasValid;
    s.modifiedPC=s.pc^(s.rasValid?s.ras:0); s.lastAddress=e.run.last;
    s.pointerSource=e.pointer; s.loopGeneration=e.loopGeneration;
    s.loopPeriod=loop.loopPeriod(); s.stride=e.run.delta;
    s.width=e.key.width; s.microPC=e.key.microPC; s.state=e.run.state; s.baseDistance=e.baseDistance;
    s.effectiveDistance=e.baseDistance*(e.pointer?2:1);
    return s;
}
void T2Prefetcher::publish(const char *kind, const Entry &e) {
    const auto s=snapshot(e);
    for (const auto &cb : observers) cb(kind,s);
}
void T2Prefetcher::invalidate(Entry &e, const char *reason) {
    if (!e.valid) return;
    e.valid=false; e.context.reset(); t2Stats.invalidated++;
    // Revoke the handle before notifying consumers. Never touch a new residency.
    int state=instructionState(e.key.instruction);
    if (state>=0) updateInstructionState(e.key.instruction,state,0);
    publish(reason,e);
}
bool T2Prefetcher::ready(const Entry &e) const {
    return e.valid && e.run.deltaValid && e.run.delta!=0 && loop.recognized() &&
        e.loopGeneration==loop.generation && instructionState(e.key.instruction)>=0 &&
        (e.run.state==T2DeltaRun::Strided ||
         (e.run.state==T2DeltaRun::Observation && e.run.equal>=provisional));
}
void T2Prefetcher::syncState(Entry &e) {
    int current=instructionState(e.key.instruction);
    if (current<0) { invalidate(e,"invalidated"); return; }
    updateInstructionState(e.key.instruction,current,e.run.state);
}
void T2Prefetcher::onCPULoad(const CPULoadEvent &event, const SubmissionContext &context) {
    t2Stats.cpuLoads++;
    DynamicKey id{event.context,event.sequence};
    auto previous=pending.find(id);
    if (previous!=pending.end()) {
        t2Stats.replayAttempts++;
        if (previous->second.address!=event.address)
            if (auto *e=find(previous->second)) invalidate(*e,"replay");
        return;
    }
    if (pending.size()>=maxPending) { t2Stats.pendingFull++; return; }
    Pending p;
    p.instruction=instructionSource(event.request); p.context=event.context;
    p.pc=event.request->getPC(); p.ras=event.ras; p.rasValid=event.rasValid;
    p.address=event.address; p.width=event.size; p.microPC=event.microPC;
    p.loopGeneration=loop.recognized()?loop.generation:0;
    pending.emplace(id,p);
    auto *e=find(p);
    if (!e) return; // Only a classified primary cache miss may allocate the SIT.
    e->lastUse=++useSequence;
    if (!p.loopGeneration) { e->run.last=p.address; t2Stats.unboundLoads++; return; }
    if (e->loopGeneration!=p.loopGeneration) {
        // Bind observation after the ROB has recognized its loop.
        e->loopGeneration=p.loopGeneration; e->run=T2DeltaRun{};
        e->run.last=p.address; e->lastPrefValid=false; syncState(*e); return;
    }
    auto old=e->run.state;
    auto oldDelta=e->run.delta;
    bool wasDeltaValid=e->run.deltaValid;
    if (!e->run.sample(p.address,confirm,reject)) {
        t2Stats.arithmeticDrops++; invalidate(*e,"overflow"); return;
    }
    t2Stats.samples++;
    if (wasDeltaValid && oldDelta!=e->run.delta) {
        e->lastPrefValid=false; e->context.reset();
    }
    if (old!=e->run.state) {
        syncState(*e);
        if (e->run.state==T2DeltaRun::Strided) {
            t2Stats.confirmed++; publish("recognized",*e);
        } else { t2Stats.rejected++; e->context.reset(); publish("rejected",*e); }
        if (trace) inform("T2_STATE tick=%llu pc=%#x ras=%#x state=%u stride=%lld equal=%u changing=%u",
            curTick(),p.pc,p.ras,unsigned(e->run.state),e->run.delta,e->run.equal,e->run.changing);
    }
    if (ready(*e)) updateTarget(*e,event,context);
}
void T2Prefetcher::onObserve(const AccessEvent &e) {
    if (e.level!=1 || e.write || !e.request->hasInstSeqNum() || !e.request->hasContextId()) return;
    auto it=pending.find({e.request->contextId(),e.request->getReqInstSeqNum()});
    if (it!=pending.end() && !it->second.cacheSeen) {
        it->second.cacheSeen=true; it->second.cacheTick=e.tick;
    }
}
void T2Prefetcher::onTrain(const AccessEvent &event) {
    if (!event.primaryMiss || event.write || !event.request->hasInstSeqNum()) return;
    auto it=pending.find({event.request->contextId(),event.request->getReqInstSeqNum()});
    if (it==pending.end() || find(it->second) || instructionState(it->second.instruction)<0) return;
    auto slot=std::find_if(sit.begin(),sit.end(),[](const Entry &e) { return !e.valid; });
    if (slot==sit.end()) {
        slot=std::min_element(sit.begin(),sit.end(),[](const Entry &a,const Entry &b) {
            return a.lastUse<b.lastUse;
        });
        invalidate(*slot,"evicted"); t2Stats.evicted++;
    }
    *slot=Entry{}; slot->valid=true; slot->id=++nextID;
    slot->key=it->second; slot->run.last=it->second.address;
    slot->loopGeneration=it->second.loopGeneration; slot->lastUse=++useSequence;
    syncState(*slot); t2Stats.primaryActivations++;
    if (trace) inform("T2_ACTIVATE tick=%llu pc=%#x ras=%#x va=%#x seq=%llu",
        curTick(),slot->key.pc,slot->key.ras,slot->run.last,event.request->getReqInstSeqNum());
}
bool T2Prefetcher::canHandle(const AccessEvent &event) const {
    if (event.write || !event.request->hasInstSeqNum() || !event.request->hasContextId()) return false;
    auto it=pending.find({event.request->contextId(),event.request->getReqInstSeqNum()});
    if (it==pending.end()) return false;
    auto *e=find(it->second);
    return e && ready(*e); // Ownership remains true even with no new candidate.
}
void T2Prefetcher::updateTarget(Entry &e, const CPULoadEvent &event,
                               const SubmissionContext &context) {
    e.baseDistance=t2Distance(latency.value(),loop.loopPeriod(),margin,maxDistance);
    unsigned effective=e.baseDistance*(e.pointer?2:1);
    if (trace) inform("T2_TARGET tick=%llu pc=%#x va=%#x latency=%llu period=%llu margin=%llu distance=%u",
        curTick(),e.key.pc,event.address,latency.value(),loop.loopPeriod(),margin,effective);
    e.current=event.address;
    if (!t2Target(e.current,e.run.delta,effective,e.desired)) {
        t2Stats.arithmeticDrops++; e.context.reset(); return;
    }
    e.context=context;
    scheduleWork();
    if (e.run.state==T2DeltaRun::Strided && !observers.empty()) publish("updated",e);
}
bool T2Prefetcher::nextAddress(const Entry &e, Addr &address) const {
    if (!ready(e) || !e.context) return false;
    Addr base=e.current;
    if (e.lastPrefValid && ((e.run.delta>0 && e.lastPref>base) ||
                           (e.run.delta<0 && e.lastPref<base))) base=e.lastPref;
    if (!t2Target(base,e.run.delta,1,address)) return false;
    return e.run.delta>0 ? address<=e.desired : address>=e.desired;
}
void T2Prefetcher::scheduleWork() {
    if (catchup.scheduled()) return;
    Addr next;
    for (const auto &e : sit) if (nextAddress(e,next)) {
        schedule(catchup,clockEdge(Cycles(1))); return;
    }
}
void T2Prefetcher::emitOne() {
    if (provisionalTick!=curTick()) { provisionalTick=curTick(); provisionalUsed=0; }
    for (unsigned k=0;k<sit.size();++k) {
        auto &e=sit[cursor++%sit.size()];
        Addr address;
        if (!nextAddress(e,address)) continue;
        bool early=e.run.state==T2DeltaRun::Observation;
        if (early && (provisionalUsed>=provisionalPerCycle ||
                      queuedCandidates(1)>=provisionalQueueLimit)) continue;
        // Skip zero-value transport: line already covers the actual demand.
        auto line=cacheView(1).blockSize();
        if (address/line!=e.current/line) {
            auto result=submitCandidates(*e.context,{{address,1,early?0:1,true}});
            t2Stats.candidates++;
            if (result[0]==SubmissionResult::Transport) t2Stats.transported++;
            if (early) { ++provisionalUsed; t2Stats.provisionalRequests++; }
            if (trace) inform("T2_CANDIDATE tick=%llu pc=%#x va=%#x stride=%lld distance=%u state=%u",
                curTick(),e.key.pc,address,e.run.delta,e.baseDistance*(e.pointer?2:1),unsigned(e.run.state));
        }
        e.lastPref=address; e.lastPrefValid=true;
        break; // At most one catch-up address step per component cycle.
    }
    scheduleWork();
}
void T2Prefetcher::onCPUBranch(const CPUBranchEvent &e) {
    t2Stats.branches++;
    if (loop.branch(e.pc,e.target,e.taken,e.tick)) {
        t2Stats.loopSwitches++;
        for (auto &entry:sit) invalidate(entry,"loop_switch");
        if (catchup.scheduled()) deschedule(catchup);
    }
}
void T2Prefetcher::onFill(const FillEvent &e) {
    // Global cache-service estimate: one sample per completed read MSHR, not
    // one per merged demand. No accounting counters or stream state are read.
    if (serviceLatency && e.level==1 && e.readService && e.readyTick>e.serviceStart)
        if (latency.add(e.readyTick-e.serviceStart)) {
            t2Stats.serviceSamples++; t2Stats.latencySamples++;
        }
}
void T2Prefetcher::onCPUComplete(const CPUCompleteEvent &e) {
    if (serviceLatency) return;
    auto it=pending.find({e.context,e.sequence});
    if (it!=pending.end() && e.success && it->second.cacheSeen && e.tick>it->second.cacheTick)
        if (latency.add(e.tick-it->second.cacheTick)) t2Stats.latencySamples++;
}
void T2Prefetcher::onCPURetire(const CPUIdentity &e) { pending.erase({e.context,e.sequence}); }
void T2Prefetcher::onCPUSquash(InstSeqNum boundary) {
    for (auto it=pending.begin();it!=pending.end();)
        if (it->first.second>boundary) it=pending.erase(it); else ++it;
    // Pending issue-derived work is conservative: a squash cancels all catch-up,
    // but does not pretend to undo already submitted requests or predictor state.
    for (auto &e:sit) e.context.reset();
    if (catchup.scheduled()) deschedule(catchup);
}
void T2Prefetcher::onInstructionInvalidated(const InstructionSource &s) {
    for (auto &e:sit) if (e.valid && e.key.instruction.sameLine(s)) invalidate(e,"invalidated");
    for (auto it=pending.begin();it!=pending.end();)
        if (it->second.instruction.sameLine(s)) it=pending.erase(it); else ++it;
}
std::optional<T2StreamSnapshot> T2Prefetcher::lookupStream(ContextID context, Addr pc,
                                                        Addr ras, bool valid, unsigned microPC) const {
    for (const auto &e:sit)
        if (e.valid && e.key.context==context && e.key.pc==pc && e.key.ras==ras &&
            e.key.rasValid==valid && e.key.microPC==microPC && e.run.state==T2DeltaRun::Strided && ready(e)) return snapshot(e);
    return std::nullopt;
}
bool T2Prefetcher::setPointerSource(T2StreamHandle h, bool pointer) {
    for (auto &e:sit) if (e.valid && e.id==h.id && e.id==h.generation) {
        if (instructionState(e.key.instruction)<0 || e.run.state!=T2DeltaRun::Strided) return false;
        if (e.pointer!=pointer) { e.pointer=pointer; publish("updated",e); }
        return true;
    }
    return false;
}
}
