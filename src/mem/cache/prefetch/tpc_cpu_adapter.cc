#include "mem/cache/prefetch/tpc_cpu_adapter.hh"
#include <functional>
#include <type_traits>
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/lsq.hh"
#include "cpu/pred/bpred_unit.hh"
#include "params/TPCCPUAdapter.hh"
namespace gem5::prefetch {
template<class Arg> class CPUFactListener : public ProbeListenerArgBase<Arg> {
    std::function<void(const Arg &)> callback;
  public:
    CPUFactListener(ProbeManager *p,const char *n,std::function<void(const Arg &)> c)
        : ProbeListenerArgBase<Arg>(p,n),callback(std::move(c)) {}
    void notify(const Arg &a) override { callback(a); }
};
TPCCPUAdapter::TPCCPUAdapter(const TPCCPUAdapterParams &p)
    : SimObject(p),cpu(p.cpu),controller(p.controller),predictor(p.branch_predictor),
      capacity(p.max_inflight) {
    auto *o3cpu=dynamic_cast<o3::CPU *>(cpu);
    fatal_if(!o3cpu || !controller || !predictor || !capacity,
             "TPC CPU adapter requires O3 CPU/controller/predictor");
}
void TPCCPUAdapter::fetch(const o3::DynInstPtr &i) {
    if (!i->isLoad()) return;
    fatal_if(fetched.size()>=capacity,"TPC CPU context capacity exhausted; increase max_inflight");
    Addr ras=0;
    bool valid=predictor->rasTopSnapshot(i->threadNumber,ras);
    fetched.emplace(i->seqNum,Context{valid,ras});
}
void TPCCPUAdapter::load(const o3::DynInstPtr &i) {
    if (!i->isLoad() || i->isAtomic() || i->isInstPrefetch() ||
        i->isDataPrefetch() || i->isSquashed() || i->getFault()!=NoFault ||
        !i->readPredicate() || !i->readMemAccPredicate() || !i->effAddrValid() ||
        !i->savedRequest) return;
    auto f=fetched.find(i->seqNum);
    if (f==fetched.end()) return;
    auto req=i->savedRequest->mainReq();
    if (!req || req->isUncacheable() || req->isHTMCmd() || req->isPrefetch() ||
        req->isLocalAccess() || req->isCacheMaintenance()) return;
    CPULoadEvent e;
    e.context=i->contextId(); e.sequence=i->seqNum; e.tick=curTick();
    e.request=std::make_shared<Request>(*req);
    e.request->setReqInstSeqNum(i->seqNum);
    e.address=i->effAddr; e.size=i->effSize; e.microPC=i->pcState().microPC();
    e.rasValid=f->second.valid; e.ras=f->second.ras;
    controller->cpuLoad(e);
}
void TPCCPUAdapter::commit(const o3::DynInstPtr &i) {
    if (i->isControl() && i->isDirectCtrl() && !i->isCall() && !i->isReturn()) {
        auto target=i->branchTarget();
        CPUBranchEvent e;
        e.context=i->contextId(); e.sequence=i->seqNum; e.tick=curTick();
        e.pc=i->pcState().instAddr(); e.target=target->instAddr();
        e.taken=i->pcState().branching(); controller->cpuBranch(e);
    }
    controller->cpuRetire({i->contextId(),i->seqNum});
    fetched.erase(i->seqNum);
}
void TPCCPUAdapter::complete(const std::pair<o3::DynInstPtr,PacketPtr> &p) {
    const auto &i=p.first;
    if (!i->isLoad()) return;
    CPUCompleteEvent e;
    e.context=i->contextId(); e.sequence=i->seqNum; e.tick=curTick();
    e.success=!i->isSquashed() && i->getFault()==NoFault && !p.second->isError();
    controller->cpuComplete(e);
}
void TPCCPUAdapter::regProbeListeners() {
    auto *o3cpu=dynamic_cast<o3::CPU *>(cpu);
    fatal_if(o3cpu->numContexts()!=1,"TPC CPU adapter currently supports one hardware thread");
    auto *pm=cpu->getProbeManager();
    auto add=[this,pm](const char *n,auto cb,auto *type) {
        using Arg=std::remove_pointer_t<decltype(type)>;
        listeners.emplace_back(std::make_unique<CPUFactListener<Arg>>(pm,n,cb));
    };
    add("Fetch",[this](const o3::DynInstPtr &i){fetch(i);},static_cast<o3::DynInstPtr *>(nullptr));
    add("LoadAddressReady",[this](const o3::DynInstPtr &i){load(i);},static_cast<o3::DynInstPtr *>(nullptr));
    add("Commit",[this](const o3::DynInstPtr &i){commit(i);},static_cast<o3::DynInstPtr *>(nullptr));
    add("Squash",[this](const o3::DynInstPtr &i){
        fetched.erase(i->seqNum); controller->cpuRetire({i->contextId(),i->seqNum});
    },static_cast<o3::DynInstPtr *>(nullptr));
    using Completion=std::pair<o3::DynInstPtr,PacketPtr>;
    add("DataAccessComplete",[this](const Completion &c){complete(c);},static_cast<Completion *>(nullptr));
    using Squash=std::pair<ThreadID,InstSeqNum>;
    add("SquashRange",[this](const Squash &s){
        fetched.erase(fetched.upper_bound(s.second),fetched.end());
        controller->cpuSquash(s.second);
    },static_cast<Squash *>(nullptr));
}
}
