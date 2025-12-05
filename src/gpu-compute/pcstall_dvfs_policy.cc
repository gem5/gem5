#include "gpu-compute/pcstall_dvfs_policy.hh"

#include "base/trace.hh"
#include "debug/GPUDVFSPolicy.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/scoreboard_check_stage.hh"

namespace gem5 {

PCSTALLDVFSPolicy::PCEntry::PCEntry()
  : sensitivity(0.0), lastTotal(0), lastCritical(0), valid(false)
{
}

PCSTALLDVFSPolicy::PCSTALLDVFSPolicy(const Params &p)
    : GPUDVFSPolicy(p),
      numPcBuckets(p.numPcBuckets),
      pcAlpha(p.pcAlpha),
      edExponent(p.edExponent)
{
    const int numCUs = shader->n_cu;

    prevCyclesPerCU.resize(numCUs, 0);
    prevMemPerCU.resize(numCUs, 0);

    pcTable.resize(numCUs);
    for (int cu = 0; cu < numCUs; ++cu) {
        pcTable[cu].resize(numPcBuckets);
    }

    if (numPcBuckets == 0) {
        warn("%s: PCSTALLDVFSPolicy with numPcBuckets == 0; "
             "PC predictor disabled.\n", name());
    }

    if (pcAlpha <= 0.0 || pcAlpha > 1.0) {
        panic("%s: pcAlpha must be in (0,1], got %f\n",
              name(), pcAlpha);
    }

    DPRINTF(GPUDVFSPolicy,
            "%s: PCSTALLDVFSPolicy constructed for %d CUs, "
            "%u PC buckets, alpha=%f, ED^%uP\n",
            name(), numCUs, numPcBuckets, pcAlpha, edExponent);
}

void
PCSTALLDVFSPolicy::serialize(CheckpointOut &cp) const
{
    GPUDVFSPolicy::serialize(cp);
    SERIALIZE_CONTAINER(prevCyclesPerCU);
    SERIALIZE_CONTAINER(prevMemPerCU);
    // pcTable is not checkpointed; it will relearn after restore.
}

void
PCSTALLDVFSPolicy::unserialize(CheckpointIn &cp)
{
    GPUDVFSPolicy::unserialize(cp);
    UNSERIALIZE_CONTAINER(prevCyclesPerCU);
    UNSERIALIZE_CONTAINER(prevMemPerCU);
    // pcTable remains default-initialized and will relearn.
}

DVFSHandler::PerfLevel
PCSTALLDVFSPolicy::sample(DVFSHandler::PerfLevel currentLevel)
{
    const int numCUs = shader->n_cu;
    if (numCUs == 0) {
        return currentLevel;
    }

    uint64_t totalCycles = 0;
    uint64_t totalStallCycles = 0;

    // For PC-based prediction.
    double weightedCrit = 0.0;
    double weightedTotal = 0.0;

    for (int i = 0; i < numCUs; ++i) {
        ComputeUnit *cu = shader->cuList[i];

        // Cumulative cycles this CU has been active (any WF).
        uint64_t currCycles = cu->stats.totalCycles.value();

        // Cumulative memory stall cycles (NRDY_WAIT_CNT).
        auto &statsStruct = cu->scoreboardCheckStage.getStats();
        uint64_t currMemStallCycles =
            statsStruct.stallCycles[
                ScoreboardCheckStage::NRDY_WAIT_CNT].value();

        const int maxActiveWFsPerCU =
            cu->numVectorALUs * cu->shader->n_wf;

        uint64_t deltaCycles;
        uint64_t deltaMemStallCycles;

        // Detect stats reset (e.g., between kernels).
        if (currCycles < prevCyclesPerCU[i] ||
            currMemStallCycles < prevMemPerCU[i]) {
            deltaCycles         = currCycles;
            deltaMemStallCycles = currMemStallCycles;
        } else {
            deltaCycles         = currCycles        - prevCyclesPerCU[i];
            deltaMemStallCycles = currMemStallCycles - prevMemPerCU[i];
        }

        prevCyclesPerCU[i] = currCycles;
        prevMemPerCU[i] = currMemStallCycles;

        uint64_t scaledDeltaCycles =
            deltaCycles * maxActiveWFsPerCU;

        totalCycles += scaledDeltaCycles;
        totalStallCycles += deltaMemStallCycles;

        // --- PC-based learning for this CU ---
        if (numPcBuckets == 0) {
            continue;
        }

        for (unsigned b = 0; b < numPcBuckets; ++b) {
            uint64_t curTotal =
                statsStruct.pcBucketTotalCycles[b].value();
            uint64_t curCritical =
                statsStruct.pcBucketCriticalCycles[b].value();

            PCEntry &entry = pcTable[i][b];

            uint64_t deltaTotal;
            uint64_t deltaCritical;

            // Detect stats reset for this bucket (either counter can reset).
            if (curTotal < entry.lastTotal ||
                curCritical < entry.lastCritical) {
                deltaTotal    = curTotal;
                deltaCritical = curCritical;
            } else {
                deltaTotal    = curTotal    - entry.lastTotal;
                deltaCritical = curCritical - entry.lastCritical;
            }

            entry.lastTotal    = curTotal;
            entry.lastCritical = curCritical;

            if (deltaTotal == 0) {
                // No contribution from this bucket this epoch.
                continue;
            }

            // Safety: critical cycles should never exceed total cycles.
            if (deltaCritical > deltaTotal) {
                deltaCritical = deltaTotal;
            }

            double obsSens =
                static_cast<double>(deltaCritical) /
                static_cast<double>(deltaTotal);

            // Extra safety clamp [0, 1]
            if (obsSens < 0.0) obsSens = 0.0;
            if (obsSens > 1.0) obsSens = 1.0;

            if (!entry.valid) {
                entry.sensitivity = obsSens;
                entry.valid = true;
            } else {
                entry.sensitivity =
                    (1.0 - pcAlpha) * entry.sensitivity +
                    pcAlpha * obsSens;
            }

            weightedCrit +=
                entry.sensitivity *
                static_cast<double>(deltaTotal);
            weightedTotal += static_cast<double>(deltaTotal);
        }
    }

    // If no work was done, don't change level.
    if (totalCycles == 0) {
        return currentLevel;
    }

    // Baseline STALL split (same as STALLDVFSPolicy).
    uint64_t totalBusyCycles =
        (totalCycles > totalStallCycles) ?
        (totalCycles - totalStallCycles) : 0;

    uint64_t predictedBusyCycles = totalBusyCycles;
    uint64_t predictedStallCycles = totalStallCycles;

    double predCritFrac = -1.0;
    if (weightedTotal > 0.0) {
    predCritFrac = weightedCrit / weightedTotal;

    // Clamp predictor to [0, 1]
    if (predCritFrac < 0.0) {
        predCritFrac = 0.0;
    } else if (predCritFrac > 1.0) {
        predCritFrac = 1.0;
    }

    double critCycles =
        predCritFrac * static_cast<double>(totalCycles);

    // Round to nearest integer
    uint64_t busy =
        static_cast<uint64_t>(critCycles + 0.5);

    // Clamp busy ≤ totalCycles to avoid underflow in stall
    if (busy > totalCycles) {
        DPRINTF(GPUDVFSPolicy,
                "%s: predictedBusy(%lu) > total(%lu); "
                "clamping busy\n",
                name(), busy, totalCycles);
        busy = totalCycles;
    }

    predictedBusyCycles  = busy;
    predictedStallCycles = totalCycles - predictedBusyCycles;
    }

    DPRINTF(GPUDVFSPolicy,
            "%s: totalCycles=%lu totalStall=%lu busyBaseline=%lu "
            "predCritFrac=%.3f predBusy=%lu predStall=%lu\n",
            name(),
            totalCycles,
            totalStallCycles,
            totalBusyCycles,
            predCritFrac,
            predictedBusyCycles,
            predictedStallCycles);

    // STALL-style delay model using predicted busy/stall cycles.
    auto delayFunction =
        [this,
         predictedBusyCycles,
         predictedStallCycles,
         currentLevel](DVFSHandler::PerfLevel level) -> Tick
    {
        Tick T_fcurr = clkPeriodAtPerfLevel(currentLevel);
        Tick T_fnew = clkPeriodAtPerfLevel(level);

        long double t_new =
            static_cast<long double>(predictedBusyCycles) * T_fnew +
            static_cast<long double>(predictedStallCycles) * T_fcurr;

        return static_cast<Tick>(t_new);
    };

    auto objectiveFunction =
        [this](double V, double f, Tick T) -> double
    {
        switch (edExponent) {
          case 1:
            return calculateEDNP<1>(V, f, T);
          case 2:
          default:
            return calculateEDNP<2>(V, f, T);
        }
    };

    DVFSHandler::PerfLevel optimalLevel =
        chooseOptimalPerfLevel(delayFunction,
                               objectiveFunction,
                               true);

    return optimalLevel;
}

} // namespace gem5
