#ifndef __GPU_DVFS_POLICY_HH__
#define __GPU_DVFS_POLICY_HH__

#include <cmath>

#include "debug/GPUDVFSPolicy.hh"
#include "gpu-compute/shader.hh"
#include "params/GPUDVFSPolicy.hh"
#include "sim/clocked_object.hh"
#include "sim/dvfs_handler.hh"
#include "sim/serialize.hh"

namespace gem5 {


class GPUDVFSPolicy : public ClockedObject
{

public:
    using Params = GPUDVFSPolicyParams;

    /**
    Schedule first sample
    */
    void init() override;

    /**
    Checkpointing: preserve state on resume()
    */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    GPUDVFSPolicy(const Params &p);

protected:
    const DVFSHandler::PerfLevel minPerfLevel = 0;
    const DVFSHandler::PerfLevel maxPerfLevel;
    const Shader * const shader;

    virtual DVFSHandler::PerfLevel
    sample(DVFSHandler::PerfLevel currentLevel) = 0;

    template <unsigned int N>
    double calculateEDNP(double V, double f, double T)
    {

        // Energy (E) ~ V^2 * f * T
        // Delay (D) ~ T

        double E = V * V * f * T;
        double D = std::pow(T, N);

        double P = E * D;

        return P;
    }


    int
    numCUs() const {
        return shader->n_cu;
    }

    /**
     * Access the statistics of a specific CU.
     */
    const ComputeUnit::ComputeUnitStats &
    statsOfCU(int cuIdx) {
        fatal_if(cuIdx < 0 || cuIdx >= shader->n_cu,
                 "CU index %d out of range [0, %d)", cuIdx, shader->n_cu);
        return shader->cuList[cuIdx]->stats;
    }

    /**
     * @brief Choose a performance level that optimizes an objective function
     *        given an estimated delay function.
     *
     * @param delayFunction A function that estimates delay (in ticks)
     *                      given a frequency.
     *
     * @param objectiveFunction A function that computes a score given voltage,
     *                          frequency, and estimated delay.
     *                          The score is minimized or maximized
     *                          based on the `minimize` parameter.
     *
     * @param minimize Whether to minimize or maximize the objective function.
     *                 Defaults to true (minimize).
     *
     * @return  `DVFSHandler::PerfLevel`
     */
    DVFSHandler::PerfLevel chooseOptimalPerfLevel(
        std::function<Tick(DVFSHandler::PerfLevel level)> delayFunction,
        std::function<double(double V,double f, Tick t)> objectiveFunction,
        bool minimize = true)
    {
        double bestObjective = minimize ?
            std::numeric_limits<double>::max() :
            std::numeric_limits<double>::lowest();

        DVFSHandler::PerfLevel bestLevel = minPerfLevel;

        for (DVFSHandler::PerfLevel level = minPerfLevel;
            level <= maxPerfLevel;
            ++level)
        {
            Tick estimatedDelay = delayFunction(level);

            double score = objectiveFunction(
                voltageAtPerfLevel(level),
                frequencyAtPerfLevel(level),
                estimatedDelay);

            bool bestScore = minimize ?
                (score < bestObjective) : (score > bestObjective);

            bestObjective = bestScore ? score : bestObjective;
            bestLevel = bestScore ? level : bestLevel;

            DPRINTF(GPUDVFSPolicy, "%s: perfLevel=%u, "
                    "V=%.3f f=%.3fGHz T=%lu score=%f\n",
                    name(), level,
                    voltageAtPerfLevel(level),
                    frequencyAtPerfLevel(level) / 1e9,
                    estimatedDelay,
                    score);
        }

        DPRINTF(GPUDVFSPolicy, "%s: chosen perfLevel=%u with score=%f\n",
                name(), bestLevel, bestObjective);

        return bestLevel;
    }


    double voltageAtPerfLevel(DVFSHandler::PerfLevel level) const
    {
        return dvfsHandler->voltageAtPerfLevel(gpuDomainId, level);
    }

    double frequencyAtPerfLevel(DVFSHandler::PerfLevel level) const
    {
        Tick clkPeriod = dvfsHandler->clkPeriodAtPerfLevel(gpuDomainId, level);
        return static_cast<double>(sim_clock::Frequency) /
               static_cast<double>(clkPeriod);
    }

    Tick clkPeriodAtPerfLevel(DVFSHandler::PerfLevel level) const
    {
        return dvfsHandler->clkPeriodAtPerfLevel(gpuDomainId, level);
    }

private:
    /**
    DVFS Handler to control SrcClockDomain
    */
    DVFSHandler *dvfsHandler;
    DVFSHandler::DomainID gpuDomainId;

    class SampleEvent: public Event
    {
        public:
            SampleEvent(GPUDVFSPolicy *p)
                : Event(Event::Default_Pri), policy(p)
            {}
            void process() override { policy->process(); }
            const char *description() const override
            { return "GPU DVFS sample"; }
        private:
            GPUDVFSPolicy *policy;
    };

    SampleEvent sampleEvent;
    Tick samplingPeriod;

    void scheduleNextSample() {
        if (!sampleEvent.scheduled())
            schedule(sampleEvent,
                clockEdge(ticksToCycles(samplingPeriod)));
    }

    void process() {
        DVFSHandler::PerfLevel newPerfLevel = sample(currentPerfLevel());
        setLevel(newPerfLevel);
        scheduleNextSample();
    }

    void clamp(DVFSHandler::PerfLevel &level) {
        if (level < minPerfLevel)
            level = minPerfLevel;
        else if (level > maxPerfLevel)
            level = maxPerfLevel;
    }

    DVFSHandler::PerfLevel currentPerfLevel() const {
        return dvfsHandler->perfLevel(gpuDomainId);
    }

    void setLevel(DVFSHandler::PerfLevel level) {
        clamp(level);
        if (level != currentPerfLevel())
        {
            dvfsHandler->perfLevel(gpuDomainId, level);
            DPRINTF(GPUDVFSPolicy, "%s: set perfLevel=%u at %lu\n",
                    name(), level, curTick());
        }
    }

};

}
#endif // __GPU_DVFS_POLICY_HH__
