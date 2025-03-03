/*
 * Copyright (c) 2014-2015 Advanced Micro Devices, Inc.
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

#ifndef __GLOBAL_MEMORY_PIPELINE_HH__
#define __GLOBAL_MEMORY_PIPELINE_HH__

#include <queue>
#include <string>

#include "base/statistics.hh"
#include "base/stats/group.hh"
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

namespace gem5
{

class ComputeUnit;

class GlobalMemPipeline
{
  public:
    GlobalMemPipeline(const ComputeUnitParams &p, ComputeUnit &cu);
    void init();
    void exec();

    /**
     * Find the next ready response to service. In order to ensure
     * that no waitcnts are violated, we pop the oldest (in program order)
     * response, and only if it is marked as done. This is because waitcnt
     * values expect memory operations to complete and decrement their
     * counter values in program order.
     */
    GPUDynInstPtr getNextReadyResp();

    /**
     * once a memory request is finished we remove it from the
     * buffer.
     */
    void completeRequest(GPUDynInstPtr gpuDynInst);

    /**
     * Issues a request to the pipeline (i.e., enqueue it
     * in the request buffer).
     */
    void issueRequest(GPUDynInstPtr gpuDynInst);

    /**
     * This method handles responses sent to this GM pipeline by the
     * CU. Simply marks the reqeust as done in the ordered buffer to
     * indicate that the requst is finished.
     */
    void handleResponse(GPUDynInstPtr gpuDynInst);

    bool
    isGMReqFIFOWrRdy(uint32_t pendReqs=0) const
    {
        return (gmIssuedRequests.size() + pendReqs) < gmQueueSize;
    }

    const std::string &name() const { return _name; }
    void
    incLoadVRFBankConflictCycles(int num_cycles)
    {
        stats.loadVrfBankConflictCycles += num_cycles;
    }

    bool coalescerReady(GPUDynInstPtr mp) const;
    bool outstandingReqsCheck(GPUDynInstPtr mp) const;

    void acqCoalescerToken(GPUDynInstPtr mp);

    void printProgress();

  private:
    ComputeUnit &computeUnit;
    const std::string _name;
    int gmQueueSize;
    int maxWaveRequests;

    // Counters to track the inflight loads and stores
    // so that we can provide the proper backpressure
    // on the number of inflight memory operations.
    int inflightStores;
    int inflightLoads;

    // The size of global memory.
    int globalMemSize;

    /*
     * This buffer holds the memory responses in order data - the responses
     * are ordered by their unique sequence number, which is monotonically
     * increasing. When a memory request returns its "done" flag is set to
     * true. During each tick the the GM pipeline will check if the oldest
     * request is finished, and if so it will be removed from the queue.
     *
     * key:   memory instruction's sequence ID
     *
     * value: pair holding the instruction pointer and a bool that
     *        is used to indicate whether or not the request has
     *        completed
     */
    std::map<uint64_t, std::pair<GPUDynInstPtr, bool>> gmOrderedRespBuffer;

    // Global Memory Request FIFO: all global memory requests
    // are issued to this FIFO from the memory pipelines
    std::queue<GPUDynInstPtr> gmIssuedRequests;

  protected:
    struct GlobalMemPipelineStats : public statistics::Group
    {
        GlobalMemPipelineStats(statistics::Group *parent);

        // number of cycles of delaying the update of a VGPR that is the
        // target of a load instruction (or the load component of an atomic)
        // The delay is due to VRF bank conflicts
        statistics::Scalar loadVrfBankConflictCycles;
    } stats;
};

} // namespace gem5

#endif // __GLOBAL_MEMORY_PIPELINE_HH__
