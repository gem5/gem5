#ifndef __MEM_CACHE_PREFETCH_TPC_CPU_ADAPTER_HH__
#define __MEM_CACHE_PREFETCH_TPC_CPU_ADAPTER_HH__
#include <map>
#include <memory>
#include <vector>
#include "cpu/o3/dyn_inst_ptr.hh"
#include "mem/cache/prefetch/framework.hh"
#include "sim/probe/probe.hh"
namespace gem5 {
struct TPCCPUAdapterParams;
namespace branch_prediction { class BPredUnit; }
namespace prefetch {
class TPCCPUAdapter : public SimObject {
    SimObject *cpu;
    TPCController *controller;
    branch_prediction::BPredUnit *predictor;
    const unsigned capacity;
    struct Context { bool valid; Addr ras; };
    std::map<InstSeqNum,Context> fetched;
    std::vector<std::unique_ptr<ProbeListener>> listeners;
    void fetch(const o3::DynInstPtr &);
    void load(const o3::DynInstPtr &);
    void commit(const o3::DynInstPtr &);
    void complete(const std::pair<o3::DynInstPtr, PacketPtr> &);
  public:
    explicit TPCCPUAdapter(const TPCCPUAdapterParams &);
    void regProbeListeners() override;
};
}
}
#endif
