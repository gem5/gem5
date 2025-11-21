#ifndef __GPU_DVFS_POLICY_HH__
#define __GPU_DVFS_POLICY_HH__

#include "debug/GPUDVFSPolicy.hh"
#include "params/GPUDVFSPolicy.hh"
#include "sim/clocked_object.hh"
#include "sim/dvfs_handler.hh"
#include "sim/serialize.hh"

namespace gem5 {


class GPUDVFSPolicy : public ClockedObject
{

public:
    using Params = GPUDVFSPolicyParams;

    void init() override;

    //Checkpointing: preserve state on resume()
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    GPUDVFSPolicy(const Params &p);

protected:
    DVFSHandler::PerfLevel minPerfLevel;
    DVFSHandler::PerfLevel maxPerfLevel;

    std::vector<double> voltageOpps;
    std::vector<double> frequencyOpps;

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

private:
    DVFSHandler *dvfsHandler;
    DVFSHandler::DomainID gpuDomainId; //to control Src

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
