#include "gpu-compute/stall_dvfs_policy.hh"

#include "base/trace.hh"

namespace gem5 {

STALLPolicy::STALLPolicy(const Params &p)
    : GPUDVFSPolicy(p)
{
    fatal_if(!shader, "%s: shader pointer must be set for STALLPolicy\n",
             name());

    const int numCUs = shader->n_cu;

    prevInstrPerCU.resize(numCUs, 0);
    prevMemPerCU.resize(numCUs, 0);
    prevCyclesPerCU.resize(numCUs, 0);

    DPRINTF(GPUDVFSPolicy, "%s: STALLPolicy constructed for %d CUs\n",
            name(), numCUs);
}

DVFSHandler::PerfLevel
STALLPolicy::sample(DVFSHandler::PerfLevel currentLevel)
{
    /** Aggregate per-epoch deltas across all CUs. */
    double totalInstrDelta  = 0.0;
    double totalMemDelta    = 0.0;
    double totalCycleDelta  = 0.0;

    const int numCUs = shader->n_cu;

    for (int cuIdx = 0; cuIdx < numCUs; ++cuIdx) {
        const auto &stats = statsOfCU(cuIdx);

        /**
        Toy choice of stats:
          numInstrExecuted: number of instructions retired.
          globalReads/globalWrites: off-chip memory traffic.
          totalCycles: total cycles executed.
        */
        uint64_t instr = stats.numInstrExecuted.value();
        uint64_t mem   = stats.globalReads.value() +
                         stats.globalWrites.value();
        uint64_t cyc   = stats.totalCycles.value();

        /** Convert cumulative stats into per-epoch deltas. */
        uint64_t instrDelta = instr - prevInstrPerCU[cuIdx];
        uint64_t memDelta   = mem   - prevMemPerCU[cuIdx];
        uint64_t cycDelta   = cyc   - prevCyclesPerCU[cuIdx];

        prevInstrPerCU[cuIdx]  = instr;
        prevMemPerCU[cuIdx]    = mem;
        prevCyclesPerCU[cuIdx] = cyc;

        totalInstrDelta += instrDelta;
        totalMemDelta   += memDelta;
        totalCycleDelta += cycDelta;
    }

    /** If nothing happened in this epoch, just keep the current level. */
    if (totalInstrDelta <= 0.0 || totalCycleDelta <= 0.0) {
        DPRINTF(GPUDVFSPolicy,
                "%s: STALLPolicy sample: no work this epoch, "
                "staying at level=%u\n",
                name(), currentLevel);
        return currentLevel;
    }

    /** IPC over the last epoch. */
    const double ipcEpoch = totalInstrDelta / totalCycleDelta;


    /**
    Memory-intensity / stall proxy:
    more global mem accesses per instruction => more likely memory-bound.
    */
    const double memPerInstr = totalMemDelta / totalInstrDelta;

    /**
    Heuristic thresholds for the toy policy.
    Tunable, but hard-coded here for simplicity.
    */
    const double memBoundThreshold     = 0.7; // high stall / memory-bound
    const double computeBoundThreshold = 0.3; // low stall / compute-bound

    /** Fake energy estimate using V^2 * f at the current performance level. */
    const double V = voltageOpps[currentLevel];
    const double f = frequencyOpps[currentLevel];
    const double energyProxy = V * V * f;

    DPRINTF(GPUDVFSPolicy,
            "%s: STALLPolicy epoch: instr=%.0f mem=%.0f cycles=%.0f "
            "IPC=%.3f mem/instr=%.3f Eproxy(V^2*f)=%.3f (level=%u)\n",
            name(), totalInstrDelta, totalMemDelta, totalCycleDelta,
            ipcEpoch, memPerInstr, energyProxy, currentLevel);

    DVFSHandler::PerfLevel next = currentLevel;

    /**
    Policy decision:
      Memory-bound  => step DOWN a level to save energy.
      Compute-bound => step UP a level to gain performance.
      Otherwise     => keep the same level.
    */
    if (memPerInstr >= memBoundThreshold && currentLevel > minPerfLevel) {
        --next;
    } else if (memPerInstr <= computeBoundThreshold &&
               currentLevel < maxPerfLevel) {
        ++next;
    }

    if (next != currentLevel) {
        DPRINTF(GPUDVFSPolicy,
                "%s: STALLPolicy changing perf level %u -> %u\n",
                name(), currentLevel, next);
    }

    return next;
}

} 