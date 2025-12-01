#ifndef __GPU_DVFS_POLICY_HH__
#define __GPU_DVFS_POLICY_HH__

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
    /**
    OPP(Operating Performance Point) volts and freqs(GHz)
    */
    std::vector<double> voltageOpps;
    std::vector<double> frequencyOpps;

    DVFSHandler::PerfLevel minPerfLevel = 0;
    DVFSHandler::PerfLevel maxPerfLevel;
    const Shader * const shader;

    virtual DVFSHandler::PerfLevel
    sample(DVFSHandler::PerfLevel currentLevel) = 0;

    template<unsigned int N>
    uint calculateEDNP(uint E, uint D) {
        uint P = E;
        for (unsigned i = 0; i < N; ++i) {
            P *= D;
        }
        return P;
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

    DVFSHandler::PerfLevel chooseOptimalPerfLevel(
        std::function<Tick(double f)> delayFunction,
        std::function<double(double V,double f, Tick t)> objectiveFunction,
        bool minimize = true)
    {
        double bestObjective = minimize ?
            std::numeric_limits<double>::max() :
            std::numeric_limits<double>::lowest();

        DVFSHandler::PerfLevel bestLevel = minPerfLevel;

        for (DVFSHandler::PerfLevel level = minPerfLevel;
            level < maxPerfLevel;
            ++level)
        {
            Tick estimatedDelay = delayFunction(frequencyOpps[level]);

            double score = objectiveFunction(
                voltageOpps[level],
                frequencyOpps[level],
                estimatedDelay);

            bool bestScore = minimize ?
                (score < bestObjective) : (score > bestObjective);

            bestObjective = bestScore ? score : bestObjective;
            bestLevel = bestScore ? level : bestLevel;
        }

        return bestLevel;
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

        DPRINTF(GPUDVFSPolicy, "%s: sampling at %lu\n", name(), curTick());

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
