#include "gpu-compute/stall_dvfs_policy.hh"

#include "base/trace.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/scoreboard_check_stage.hh"

namespace gem5 {

STALLDVFSPolicy::STALLDVFSPolicy(const Params &p)
    : GPUDVFSPolicy(p)
{

    const int numCUs = shader->n_cu;

    prevMemPerCU.resize(numCUs, 0);
    prevCyclesPerCU.resize(numCUs, 0);

    DPRINTF(GPUDVFSPolicy, "%s: STALLDVFSPolicy constructed for %d CUs\n",
            name(), numCUs);
}

DVFSHandler::PerfLevel
STALLDVFSPolicy::sample(DVFSHandler::PerfLevel currentLevel)
{
    // Aggregate per-CU stats over the last epoch
    uint64_t totalStallCycles = 0;
    uint64_t totalCycles = 0;

    for (int i = 0; i < shader->n_cu; ++i)
    {
        ComputeUnit *cu = shader->cuList[i];

        // Get # cycles that any WF within this CU was executing.
        uint64_t currCycles = cu->stats.totalCycles.value();

        // Get memory stall cycles. These are PER-WF.
        auto& statsStruct = cu->scoreboardCheckStage.getStats();
        uint64_t currMemStallCycles = statsStruct
            .stallCycles[ScoreboardCheckStage::NRDY_WAIT_CNT]
            .value();

        // Calculate scaling factor (The number of WFs that could be active
        // in this CU).
        const int maxActiveWFsPerCU = cu->numVectorALUs *
            cu->shader->n_wf;

        // calculate deltas.
        uint64_t deltaCycles;
        uint64_t deltaMemStallCycles;

        // Detect Reset: If current is less than previous, stats were reset.
        // Delta is just the current value (assuming reset to 0).
        if (currCycles < prevCyclesPerCU[i]) {
            deltaCycles = currCycles;
            deltaMemStallCycles = currMemStallCycles;
        } else {
            deltaCycles = currCycles - prevCyclesPerCU[i];
            deltaMemStallCycles = currMemStallCycles - prevMemPerCU[i];
        }

        // Scale total cycles to be per-active-WF basis.
        uint64_t scaledDeltaCycles =
            deltaCycles * maxActiveWFsPerCU;

        // Aggregate stats.
        totalCycles += scaledDeltaCycles;
        totalStallCycles += deltaMemStallCycles;

        // Update Previous Values
        prevCyclesPerCU[i] = currCycles;
        prevMemPerCU[i] = currMemStallCycles;
    }

    // If no work was done, do nothing.
    if (totalCycles == 0) return currentLevel;


    // from Keramidas et al.,
    // "Interval-Based Models for
    // Run-Time DVFS Orchestration in Superscalar Processors"
    //  t_new = (c * k - ST * k + ST) * T_fmax
    //
    // Since we are measuring data at (most likely) the non-maximum frequency
    // level, we just assume that the current frequency corresponds to k = 1
    // and substitute "fmax" for "fcurrent".
    // The formula still holds because we only care about relative values for
    // objective comparison between levels.
    //
    // Thus, we use:
    // t_new = (c * k - ST * k + ST) * T_fcurr
    // t_new = (c - ST) * (k*T_fcurr) + ST * T_fcurr
    // t_new = totalBusyCycles * T_fnew + totalStallCycles * T_fcurr
    //
    // This equation corresponds exactly to the delayFunction defined
    // in the base class' `optimalPerfLevel()`

    uint64_t totalBusyCycles = (totalCycles > totalStallCycles) ?
        (totalCycles - totalStallCycles) : 0;


    double T_fcurr = 1.0 / frequencyOpps[currentLevel];

    // TODO check units here.

    auto delayFunction =
    [totalBusyCycles, totalStallCycles, T_fcurr](double targetFreq) -> Tick {
        double T_fnew = 1.0 / targetFreq;

        Tick t_new =
            (totalBusyCycles * T_fnew) +
            (totalStallCycles * T_fcurr);

        return t_new;
    };

    auto objectiveFunction = [this](double V, double f, Tick T) -> double {
        return calculateEDNP<2>(V, f, T);
    };

    DVFSHandler::PerfLevel optimalLevel =
    chooseOptimalPerfLevel(delayFunction,objectiveFunction, true);

    return optimalLevel;
}

}
