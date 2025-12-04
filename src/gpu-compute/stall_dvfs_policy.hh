#ifndef __STALL_DVFS_POLICY_HH__
#define __STALL_DVFS_POLICY_HH__

#include <cstdint>
#include <vector>

#include "gpu-compute/gpu_dvfs_policy.hh"
#include "params/STALLDVFSPolicy.hh"

namespace gem5 {

/**
 * Toy STALL-like DVFS policy for GPUs.
 *
 * Uses per-CU instruction and global memory stats over each sampling epoch
 * to decide whether the workload is memory-bound (many stalls) or
 * compute-bound, and steps the DVFS performance level up/down accordingly.
 */
class STALLDVFSPolicy : public GPUDVFSPolicy
{
  public:
    using Params = STALLDVFSPolicyParams;

    STALLDVFSPolicy(const Params &p);
    void serialize(CheckpointOut &cp) const override {
        GPUDVFSPolicy::serialize(cp);
        SERIALIZE_CONTAINER(prevCyclesPerCU);
        SERIALIZE_CONTAINER(prevMemPerCU);
    }

    void unserialize(CheckpointIn &cp) override {
        GPUDVFSPolicy::unserialize(cp);
        UNSERIALIZE_CONTAINER(prevCyclesPerCU);
        UNSERIALIZE_CONTAINER(prevMemPerCU);
    }

  private:

    /**
    Per-CU history at the *previous* sample, to convert cumulative
    statistics into per-epoch deltas.
    */
    std::vector<uint64_t> prevCyclesPerCU;
    std::vector<uint64_t> prevMemPerCU;

    /**
     * @brief Core STALL decision function.
     *
     * Called once per epoch by the GPUDVFSPolicy base class. Uses the STALL
     * algorithm to decide the optimal performance level based on per-CU stats.
     *
     * Specifically, it aggregates per-CU clock and memory stall cycles.
     * This is then evaluated using `optimalPerfLevel()`.
     *
     */
    DVFSHandler::PerfLevel
    sample(DVFSHandler::PerfLevel currentLevel) override;
};

}

#endif // __STALL_DVFS_POLICY_HH__
