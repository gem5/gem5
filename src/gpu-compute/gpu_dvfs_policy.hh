#ifndef __GPU_DVFS_POLICY_HH__
#define __GPU_DVFS_POLICY_HH__

#include "sim/clocked_object.hh"
#include "sim/serialize.hh"

namespace gem5 {


class GPUDVFSPolicy : public ClockedObject
{

public:
    using Params = GPUDVFSPolicyParams;

    void init() override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

private:


};

}
#endif // __GPU_DVFS_POLICY_HH__