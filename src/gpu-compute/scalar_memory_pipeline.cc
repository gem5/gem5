/*
 * Copyright (c) 2016-2017 Advanced Micro Devices, Inc.
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

#include "gpu-compute/scalar_memory_pipeline.hh"

#include "debug/GPUMem.hh"
#include "debug/GPUReg.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

ScalarMemPipeline::ScalarMemPipeline(const ComputeUnitParams &p,
                                     ComputeUnit &cu)
    : computeUnit(cu), _name(cu.name() + ".ScalarMemPipeline"),
      queueSize(p.scalar_mem_queue_size),
      inflightStores(0), inflightLoads(0)
{
}

void
ScalarMemPipeline::exec()
{
    // afind oldest scalar request whose data has arrived
    GPUDynInstPtr m = !returnedLoads.empty() ? returnedLoads.front() :
        !returnedStores.empty() ? returnedStores.front() : nullptr;

    Wavefront *w = nullptr;

    bool accessSrf = true;
    // check the SRF to see if the operands of a load (or load component
    // of an atomic) are accessible
    if ((m) && (m->isLoad() || m->isAtomicRet())) {
        w = m->wavefront();

        accessSrf =
            w->computeUnit->srf[w->simdId]->
                canScheduleWriteOperandsFromLoad(w, m);
    }

    if ((!returnedStores.empty() || !returnedLoads.empty()) &&
        m->latency.rdy() && computeUnit.scalarMemToSrfBus.rdy() &&
        accessSrf &&
        (computeUnit.shader->coissue_return ||
         computeUnit.scalarMemUnit.rdy())) {

        w = m->wavefront();

        if (m->isLoad() || m->isAtomicRet()) {
            w->computeUnit->srf[w->simdId]->
                scheduleWriteOperandsFromLoad(w, m);
        }

        m->completeAcc(m);
        w->decLGKMInstsIssued();

        if (m->isLoad() || m->isAtomic()) {
            returnedLoads.pop();
            assert(inflightLoads > 0);
            --inflightLoads;
        } else {
            returnedStores.pop();
            assert(inflightStores > 0);
            --inflightStores;
        }

        // Decrement outstanding register count
        computeUnit.shader->ScheduleAdd(&w->outstandingReqs, m->time, -1);

        if (m->isStore() || m->isAtomic()) {
            computeUnit.shader->ScheduleAdd(&w->scalarOutstandingReqsWrGm,
                                             m->time, -1);
        }

        if (m->isLoad() || m->isAtomic()) {
            computeUnit.shader->ScheduleAdd(&w->scalarOutstandingReqsRdGm,
                                             m->time, -1);
        }

        // Mark write bus busy for appropriate amount of time
        computeUnit.scalarMemToSrfBus.set(m->time);
        if (!computeUnit.shader->coissue_return)
            w->computeUnit->scalarMemUnit.set(m->time);
    }

    // If pipeline has executed a global memory instruction
    // execute global memory packets and issue global
    // memory packets to DTLB
    if (!issuedRequests.empty()) {
        GPUDynInstPtr mp = issuedRequests.front();
        if (mp->isLoad() || mp->isAtomic()) {

            if (inflightLoads >= queueSize) {
                return;
            } else {
                ++inflightLoads;
            }
        } else {
            if (inflightStores >= queueSize) {
                return;
            } else {
                ++inflightStores;
            }
        }
        mp->initiateAcc(mp);
        issuedRequests.pop();

        DPRINTF(GPUMem, "CU%d: WF[%d][%d] Popping scalar mem_op\n",
                computeUnit.cu_id, mp->simdId, mp->wfSlotId);
    }
}

void
ScalarMemPipeline::issueRequest(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    if (gpuDynInst->isLoad()) {
        wf->scalarRdGmReqsInPipe--;
        wf->scalarOutstandingReqsRdGm++;
    } else if (gpuDynInst->isStore()) {
        wf->scalarWrGmReqsInPipe--;
        wf->scalarOutstandingReqsWrGm++;
    }

    wf->outstandingReqs++;
    wf->validateRequestCounters();

    issuedRequests.push(gpuDynInst);
}

} // namespace gem5
