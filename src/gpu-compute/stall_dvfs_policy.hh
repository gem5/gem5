#ifndef __STALL_DVFS_POLICY_HH__
#define __STALL_DVFS_POLICY_HH__

#include <cstdint>
#include <vector>

#include "gpu-compute/gpu_dvfs_policy.hh"
#include "params/STALLPolicy.hh"

namespace gem5 {

/**
 * Toy STALL-like DVFS policy for GPUs.
 *
 * Uses per-CU instruction and global memory stats over each sampling epoch
 * to decide whether the workload is memory-bound (many stalls) or
 * compute-bound, and steps the DVFS performance level up/down accordingly.
 */
class STALLPolicy : public GPUDVFSPolicy
{
  public:
    using Params = STALLPolicyParams;

    STALLPolicy(const Params &p);

  private:
    /**
    Per-CU history at the *previous* sample, to convert cumulative
    statistics into per-epoch deltas.
    */
    std::vector<uint64_t> prevInstrPerCU;
    std::vector<uint64_t> prevMemPerCU;
    std::vector<uint64_t> prevCyclesPerCU;

    /**
     * Core STALL decision function.
     *
     * Called once per epoch by the GPUDVFSPolicy base class.  Given the
     * current performance level, it:
     *   1. Reads CU stats and computes deltas since last sample.
     *   2. Derives an IPC and a "stall proxy" (mem ops per instr).
     *   3. Logs a fake energy estimate based on V^2 * f.
     *   4. Chooses the next performance level.
     */
    DVFSHandler::PerfLevel
    sample(DVFSHandler::PerfLevel currentLevel) override;
};

} 

#endif // __STALL_DVFS_POLICY_HH__