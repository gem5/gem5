/*
 * Copyright (c) 2011-2015,2018 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * The GPUDispatcher is the component of the shader that is responsible
 * for creating and dispatching WGs to the compute units. If all WGs in
 * a kernel cannot be dispatched simultaneously, then the dispatcher will
 * keep track of all pending WGs and dispatch them as resources become
 * available.
 */

#ifndef __GPU_COMPUTE_DISPATCHER_HH__
#define __GPU_COMPUTE_DISPATCHER_HH__

#include <queue>
#include <unordered_map>
#include <vector>

#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "dev/hsa/hsa_packet.hh"
#include "params/GPUDispatcher.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class GPUCommandProcessor;
class HSAQueueEntry;
class Shader;
class Wavefront;

class GPUDispatcher : public SimObject
{
  public:
    typedef GPUDispatcherParams Params;

    GPUDispatcher(const Params &p);
    ~GPUDispatcher();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    void setCommandProcessor(GPUCommandProcessor *gpu_cmd_proc);
    void setShader(Shader *new_shader);
    void exec();
    bool isReachingKernelEnd(Wavefront *wf);
    void updateInvCounter(int kern_id, int val=-1);
    bool updateWbCounter(int kern_id, int val=-1);
    int getOutstandingWbs(int kern_id);
    void notifyWgCompl(Wavefront *wf);
    void scheduleDispatch();
    void dispatch(HSAQueueEntry *task);
    HSAQueueEntry* hsaTask(int disp_id);

  private:
    Shader *shader;
    GPUCommandProcessor *gpuCmdProc;
    EventFunctionWrapper tickEvent;
    std::unordered_map<int, HSAQueueEntry*> hsaQueueEntries;
    // list of kernel_ids to launch
    std::queue<int> execIds;
    // list of kernel_ids that have finished
    std::queue<int> doneIds;
    // is there a kernel in execution?
    bool dispatchActive;

  protected:
    struct GPUDispatcherStats : public statistics::Group
    {
        GPUDispatcherStats(statistics::Group *parent);

        statistics::Scalar numKernelLaunched;
        statistics::Scalar cyclesWaitingForDispatch;
    } stats;
};

} // namespace gem5

#endif // __GPU_COMPUTE_DISPATCHER_HH__
