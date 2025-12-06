#ifndef __PCSTALL_DVFS_POLICY_HH__
#define __PCSTALL_DVFS_POLICY_HH__

#include <cstdint>
#include <vector>

#include "gpu-compute/gpu_dvfs_policy.hh"
#include "params/PCSTALLDVFSPolicy.hh"

namespace gem5 {

/**
 * PCSTALL-like DVFS policy for GPUs.
 *
 * Extends STALLDVFSPolicy with a per-CU, per-PC-bucket predictor that
 * learns the "critical fraction" (non-memory-stall work) for each PC
 * bucket and uses that to predict the next epoch's stall/busy split.
 */
class PCSTALLDVFSPolicy : public GPUDVFSPolicy
{
  public:
    using Params = PCSTALLDVFSPolicyParams;

    PCSTALLDVFSPolicy(const Params &p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  private:
    /** Per-CU history at the previous sample to get per-epoch deltas. */
    std::vector<uint64_t> prevCyclesPerCU;
    std::vector<uint64_t> prevMemPerCU;

    /** Number of PC buckets per CU (must match ScoreboardCheckStage). */
    unsigned numPcBuckets;

    /** EMA factor for per-PC sensitivity (0 < alpha <= 1). */
    const double pcAlpha;

    /** Exponent n in ED^nP objective (1 = EDP, 2 = ED^2P, ...). */
    const unsigned edExponent;

    struct PCEntry
    {
        double sensitivity;
        uint64_t lastTotal;
        uint64_t lastCritical;
        bool valid;

        PCEntry();
    };

    /** pcTable[cu][bucket] = per-CU, per-PC-bucket sensitivity entry. */
    std::vector<std::vector<PCEntry>> pcTable;

    DVFSHandler::PerfLevel
    sample(DVFSHandler::PerfLevel currentLevel) override;
};

} // namespace gem5

#endif // __PCSTALL_DVFS_POLICY_HH__
