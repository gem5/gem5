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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 *
 * Author: John Kalamatianos, Sooraj Puthoor
 */

#ifndef __GLOBAL_MEMORY_PIPELINE_HH__
#define __GLOBAL_MEMORY_PIPELINE_HH__

#include <queue>
#include <string>

#include "gpu-compute/misc.hh"
#include "params/ComputeUnit.hh"
#include "sim/stats.hh"

/*
 * @file global_memory_pipeline.hh
 *
 * The global memory pipeline issues newly created global memory packets
 * from the pipeline to DTLB. The exec() method of the memory packet issues
 * the packet to the DTLB if there is space available in the return fifo.
 * This stage also retires previously issued loads and stores that have
 * returned from the memory sub-system.
 */

class ComputeUnit;

class GlobalMemPipeline
{
  public:
    GlobalMemPipeline(const ComputeUnitParams *params);
    void init(ComputeUnit *cu);
    void exec();

    template<typename c0, typename c1> void doGmReturn(GPUDynInstPtr m);

    std::queue<GPUDynInstPtr> &getGMReqFIFO() { return gmIssuedRequests; }
    std::queue<GPUDynInstPtr> &getGMStRespFIFO() { return gmReturnedStores; }
    std::queue<GPUDynInstPtr> &getGMLdRespFIFO() { return gmReturnedLoads; }

    bool
    isGMLdRespFIFOWrRdy() const
    {
        return gmReturnedLoads.size() < gmQueueSize;
    }

    bool
    isGMStRespFIFOWrRdy() const
    {
        return gmReturnedStores.size() < gmQueueSize;
    }

    bool
    isGMReqFIFOWrRdy(uint32_t pendReqs=0) const
    {
        return (gmIssuedRequests.size() + pendReqs) < gmQueueSize;
    }

    const std::string &name() const { return _name; }
    void regStats();

  private:
    ComputeUnit *computeUnit;
    std::string _name;
    int gmQueueSize;

    // number of cycles of delaying the update of a VGPR that is the
    // target of a load instruction (or the load component of an atomic)
    // The delay is due to VRF bank conflicts
    Stats::Scalar loadVrfBankConflictCycles;
    // Counters to track the inflight loads and stores
    // so that we can provide the proper backpressure
    // on the number of inflight memory operations.
    int inflightStores;
    int inflightLoads;

    // The size of global memory.
    int globalMemSize;

    // Global Memory Request FIFO: all global memory requests
    // are issued to this FIFO from the memory pipelines
    std::queue<GPUDynInstPtr> gmIssuedRequests;

    // Globa Store Response FIFO: all responses of global memory
    // stores are sent to this FIFO from TCP
    std::queue<GPUDynInstPtr> gmReturnedStores;

    // Global Load Response FIFO: all responses of global memory
    // loads are sent to this FIFO from TCP
    std::queue<GPUDynInstPtr> gmReturnedLoads;
};

#endif // __GLOBAL_MEMORY_PIPELINE_HH__
