#include "gpu-compute/gpu_dvfs_policy.hh"

namespace gem5 {


class ToyDVFSPolicy : public GPUDVFSPolicy
{
public:
    using Params = ToyDVFSPolicyParams;

    ToyDVFSPolicy(const Params &p)
        : GPUDVFSPolicy(p)
    {}

private:
    DVFSHandler::PerfLevel
    sample(DVFSHandler::PerfLevel currentLevel) override
    {
        // Simple toy policy: iterate over all performance levels
        DVFSHandler::PerfLevel newLevel = currentLevel + 1;
        if (newLevel > maxPerfLevel) {
            newLevel = minPerfLevel;
        }
        return newLevel;
    }

};

}
