/*
 * Copyright (c) 2014-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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

#ifndef __LOCAL_MEMORY_PIPELINE_HH__
#define __LOCAL_MEMORY_PIPELINE_HH__

#include <queue>
#include <string>

#include "gpu-compute/misc.hh"
#include "params/ComputeUnit.hh"
#include "sim/stats.hh"

/*
 * @file local_memory_pipeline.hh
 *
 * The local memory pipeline issues newly created local memory packets
 * from pipeline to the LDS. This stage also retires previously issued
 * loads and stores that have returned from the LDS.
 */

class ComputeUnit;
class Wavefront;

class LocalMemPipeline
{
  public:
    LocalMemPipeline(const ComputeUnitParams *p, ComputeUnit &cu);
    void exec();
    std::queue<GPUDynInstPtr> &getLMRespFIFO() { return lmReturnedRequests; }

    void issueRequest(GPUDynInstPtr gpuDynInst);


    bool
    isLMRespFIFOWrRdy() const
    {
        return lmReturnedRequests.size() < lmQueueSize;
    }

    bool
    isLMReqFIFOWrRdy(uint32_t pendReqs=0) const
    {
        return (lmIssuedRequests.size() + pendReqs) < lmQueueSize;
    }

    const std::string& name() const { return _name; }
    void regStats();

    void
    incLoadVRFBankConflictCycles(int num_cycles)
    {
        loadVrfBankConflictCycles += num_cycles;
    }

  private:
    ComputeUnit &computeUnit;
    const std::string _name;
    int lmQueueSize;
    Stats::Scalar loadVrfBankConflictCycles;
    // Local Memory Request Fifo: all shared memory requests
    // are issued to this FIFO from the memory pipelines
    std::queue<GPUDynInstPtr> lmIssuedRequests;

    // Local Memory Response Fifo: all responses of shared memory
    // requests are sent to this FIFO from LDS
    std::queue<GPUDynInstPtr> lmReturnedRequests;
};

#endif // __LOCAL_MEMORY_PIPELINE_HH__
