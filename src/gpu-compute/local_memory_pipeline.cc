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

#include "gpu-compute/local_memory_pipeline.hh"

#include "debug/GPUMem.hh"
#include "debug/GPUPort.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

LocalMemPipeline::LocalMemPipeline(const ComputeUnitParams &p, ComputeUnit &cu)
    : computeUnit(cu), _name(cu.name() + ".LocalMemPipeline"),
      lmQueueSize(p.local_mem_queue_size), stats(&cu)
{
}

void
LocalMemPipeline::exec()
{
    // apply any returned shared (LDS) memory operations
    GPUDynInstPtr m = !lmReturnedRequests.empty() ?
        lmReturnedRequests.front() : nullptr;

    bool accessVrf = true;
    Wavefront *w = nullptr;

    if ((m) && m->latency.rdy() && (m->isLoad() || m->isAtomicRet())) {
        w = m->wavefront();

        accessVrf = w->computeUnit->vrf[w->simdId]->
            canScheduleWriteOperandsFromLoad(w, m);

    }

    if (!lmReturnedRequests.empty() && m->latency.rdy() && accessVrf &&
        computeUnit.locMemToVrfBus.rdy()
        && (computeUnit.shader->coissue_return
        || computeUnit.vectorSharedMemUnit.rdy())) {

        lmReturnedRequests.pop();
        w = m->wavefront();

        if (m->isFlat() && !m->isMemSync() && !m->isEndOfKernel()
            && m->allLanesZero()) {
            computeUnit.getTokenManager()->recvTokens(1);
        }

        DPRINTF(GPUMem, "CU%d: WF[%d][%d]: Completing local mem instr %s\n",
                m->cu_id, m->simdId, m->wfSlotId, m->disassemble());
        m->completeAcc(m);
        w->decLGKMInstsIssued();

        if (m->isLoad() || m->isAtomicRet()) {
            w->computeUnit->vrf[w->simdId]->
                scheduleWriteOperandsFromLoad(w, m);
        }

        // Decrement outstanding request count
        computeUnit.shader->ScheduleAdd(&w->outstandingReqs, m->time, -1);

        if (m->isStore() || m->isAtomic()) {
            computeUnit.shader->ScheduleAdd(&w->outstandingReqsWrLm,
                                             m->time, -1);
        }

        if (m->isLoad() || m->isAtomic()) {
            computeUnit.shader->ScheduleAdd(&w->outstandingReqsRdLm,
                                             m->time, -1);
        }

        // Mark write bus busy for appropriate amount of time
        computeUnit.locMemToVrfBus.set(m->time);
        if (computeUnit.shader->coissue_return == 0)
            w->computeUnit->vectorSharedMemUnit.set(m->time);
    }

    // If pipeline has executed a local memory instruction
    // execute local memory packet and issue the packets
    // to LDS
    if (!lmIssuedRequests.empty() && lmReturnedRequests.size() < lmQueueSize) {

        GPUDynInstPtr m = lmIssuedRequests.front();

        bool returnVal = computeUnit.sendToLds(m);
        if (!returnVal) {
            DPRINTF(GPUPort, "packet was nack'd and put in retry queue");
        }
        lmIssuedRequests.pop();
    }
}

void
LocalMemPipeline::issueRequest(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    if (gpuDynInst->isLoad()) {
        wf->rdLmReqsInPipe--;
        wf->outstandingReqsRdLm++;
    } else if (gpuDynInst->isStore()) {
        wf->wrLmReqsInPipe--;
        wf->outstandingReqsWrLm++;
    } else {
        // Atomic, both read and write
        wf->rdLmReqsInPipe--;
        wf->outstandingReqsRdLm++;
        wf->wrLmReqsInPipe--;
        wf->outstandingReqsWrLm++;
    }

    wf->outstandingReqs++;
    wf->validateRequestCounters();

    gpuDynInst->setAccessTime(curTick());
    lmIssuedRequests.push(gpuDynInst);
}


LocalMemPipeline::
LocalMemPipelineStats::LocalMemPipelineStats(statistics::Group *parent)
    : statistics::Group(parent, "LocalMemPipeline"),
      ADD_STAT(loadVrfBankConflictCycles, "total number of cycles LDS data "
               "are delayed before updating the VRF")
{
}

} // namespace gem5
