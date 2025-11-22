#include "gpu-compute/gpu_dvfs_policy.hh"

#include "base/trace.hh"

namespace gem5 {

GPUDVFSPolicy::GPUDVFSPolicy(const Params &p)
  : ClockedObject(p),
    dvfsHandler(p.dvfs_handler),
    gpuDomainId(p.gpu_domain_id),
    sampleEvent(this), // TODO fix initialization?
    samplingPeriod(p.sampling_period),
    voltageOpps(p.voltage_opps.begin(), p.voltage_opps.end()),
    frequencyOpps(p.frequency_opps.begin(), p.frequency_opps.end()),
    maxPerfLevel(frequencyOpps.size() - 1)
{
    DPRINTF(GPUDVFSPolicy, "%s: Initializing GPU DVFS Policy\n", name());
    fatal_if(!dvfsHandler, "%s: DVFSHandler is required", name());
    // fatal_if(!gpu,         "%s: GPU device pointer is required", name());
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
