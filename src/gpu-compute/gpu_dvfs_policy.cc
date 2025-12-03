#include "gpu-compute/gpu_dvfs_policy.hh"

#include "base/trace.hh"
#include "gpu-compute/shader.hh"

namespace gem5 {

GPUDVFSPolicy::GPUDVFSPolicy(const Params &p)
  : ClockedObject(p),
    maxPerfLevel(p.dvfs_handler->numPerfLevels(p.gpu_domain_id) - 1),
    shader(p.shader),
    dvfsHandler(p.dvfs_handler),
    gpuDomainId(p.gpu_domain_id),
    sampleEvent(this), // TODO fix initialization?
    samplingPeriod(p.sampling_period)
{
    DPRINTF(GPUDVFSPolicy, "%s: Initializing GPU DVFS Policy\n", name());
    fatal_if(!dvfsHandler, "%s: DVFSHandler is required", name());
    fatal_if(!shader, "%s: shader pointer must be set for GPUDVFSPolicy!\n",
             name());

}

void
GPUDVFSPolicy::init()
{
    ClockedObject::init();
    scheduleNextSample();
}


// --- Checkpointing ---
void
GPUDVFSPolicy::serialize(CheckpointOut &cp) const
{
    // TODO implement.
}

void
GPUDVFSPolicy::unserialize(CheckpointIn &cp)
{
    // TODO implement.
}

} // namespace gem5
