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

#include "gpu-compute/global_memory_pipeline.hh"

#include "debug/GPUMem.hh"
#include "debug/GPUReg.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

GlobalMemPipeline::GlobalMemPipeline(const ComputeUnitParams* p) :
    computeUnit(nullptr), gmQueueSize(p->global_mem_queue_size),
    outOfOrderDataDelivery(p->out_of_order_data_delivery), inflightStores(0),
    inflightLoads(0)
{
}

void
GlobalMemPipeline::init(ComputeUnit *cu)
{
    computeUnit = cu;
    globalMemSize = computeUnit->shader->globalMemSize;
    _name = computeUnit->name() + ".GlobalMemPipeline";
}

void
GlobalMemPipeline::exec()
{
    // apply any returned global memory operations
    GPUDynInstPtr m = getNextReadyResp();

    bool accessVrf = true;
    Wavefront *w = nullptr;

    // check the VRF to see if the operands of a load (or load component
    // of an atomic) are accessible
    if ((m) && (m->isLoad() || m->isAtomicRet())) {
        w = m->wavefront();

        accessVrf =
            w->computeUnit->vrf[w->simdId]->
                vrfOperandAccessReady(m->seqNum(), w, m, VrfAccessType::WRITE);
    }

    if (m && m->latency.rdy() && computeUnit->glbMemToVrfBus.rdy() &&
        accessVrf && m->statusBitVector == VectorMask(0) &&
        (computeUnit->shader->coissue_return ||
        computeUnit->wfWait.at(m->pipeId).rdy())) {

        w = m->wavefront();

        m->completeAcc(m);

        completeRequest(m);

        // Decrement outstanding register count
        computeUnit->shader->ScheduleAdd(&w->outstandingReqs, m->time, -1);

        if (m->isStore() || m->isAtomic()) {
            computeUnit->shader->ScheduleAdd(&w->outstandingReqsWrGm,
                                             m->time, -1);
        }

        if (m->isLoad() || m->isAtomic()) {
            computeUnit->shader->ScheduleAdd(&w->outstandingReqsRdGm,
                                             m->time, -1);
        }

        // Mark write bus busy for appropriate amount of time
        computeUnit->glbMemToVrfBus.set(m->time);
        if (!computeUnit->shader->coissue_return)
            w->computeUnit->wfWait.at(m->pipeId).set(m->time);
    }

    // If pipeline has executed a global memory instruction
    // execute global memory packets and issue global
    // memory packets to DTLB
    if (!gmIssuedRequests.empty()) {
        GPUDynInstPtr mp = gmIssuedRequests.front();
        if (mp->isLoad() || mp->isAtomic()) {
            if (inflightLoads >= gmQueueSize) {
                return;
            } else {
                ++inflightLoads;
            }
        } else if (mp->isStore()) {
            if (inflightStores >= gmQueueSize) {
                return;
            } else {
                ++inflightStores;
            }
        }

        mp->initiateAcc(mp);

        if (!outOfOrderDataDelivery && !mp->isMemFence()) {
            /**
             * if we are not in out-of-order data delivery mode
             * then we keep the responses sorted in program order.
             * in order to do so we must reserve an entry in the
             * resp buffer before we issue the request to the mem
             * system. mem fence requests will not be stored here
             * because once they are issued from the GM pipeline,
             * they do not send any response back to it.
             */
            gmOrderedRespBuffer.insert(std::make_pair(mp->seqNum(),
                std::make_pair(mp, false)));
        }

        gmIssuedRequests.pop();

        DPRINTF(GPUMem, "CU%d: WF[%d][%d] Popping 0 mem_op = \n",
                computeUnit->cu_id, mp->simdId, mp->wfSlotId);
    }
}

GPUDynInstPtr
GlobalMemPipeline::getNextReadyResp()
{
    if (outOfOrderDataDelivery) {
        if (!gmReturnedLoads.empty()) {
            return gmReturnedLoads.front();
        } else if (!gmReturnedStores.empty()) {
            return gmReturnedStores.front();
        }
    } else {
        if (!gmOrderedRespBuffer.empty()) {
            auto mem_req = gmOrderedRespBuffer.begin();

            if (mem_req->second.second) {
                return mem_req->second.first;
            }
        }
    }

    return nullptr;
}

void
GlobalMemPipeline::completeRequest(GPUDynInstPtr gpuDynInst)
{
    if (gpuDynInst->isLoad() || gpuDynInst->isAtomic()) {
        assert(inflightLoads > 0);
        --inflightLoads;
    } else if (gpuDynInst->isStore()) {
        assert(inflightStores > 0);
        --inflightStores;
    }

    if (outOfOrderDataDelivery) {
        if (gpuDynInst->isLoad() || gpuDynInst->isAtomic()) {
            assert(!gmReturnedLoads.empty());
            gmReturnedLoads.pop();
        } else if (gpuDynInst->isStore()) {
            assert(!gmReturnedStores.empty());
            gmReturnedStores.pop();
        }
    } else {
        // we should only pop the oldest requst, and it
        // should be marked as done if we are here
        assert(gmOrderedRespBuffer.begin()->first == gpuDynInst->seqNum());
        assert(gmOrderedRespBuffer.begin()->second.first == gpuDynInst);
        assert(gmOrderedRespBuffer.begin()->second.second);
        // remove this instruction from the buffer by its
        // unique seq ID
        gmOrderedRespBuffer.erase(gpuDynInst->seqNum());
    }
}

void
GlobalMemPipeline::issueRequest(GPUDynInstPtr gpuDynInst)
{
    gmIssuedRequests.push(gpuDynInst);
}

void
GlobalMemPipeline::handleResponse(GPUDynInstPtr gpuDynInst)
{
    if (outOfOrderDataDelivery) {
        if (gpuDynInst->isLoad() || gpuDynInst->isAtomic()) {
            assert(isGMLdRespFIFOWrRdy());
            gmReturnedLoads.push(gpuDynInst);
        } else {
            assert(isGMStRespFIFOWrRdy());
            gmReturnedStores.push(gpuDynInst);
        }
    } else {
        auto mem_req = gmOrderedRespBuffer.find(gpuDynInst->seqNum());
        // if we are getting a response for this mem request,
        // then it ought to already be in the ordered response
        // buffer
        assert(mem_req != gmOrderedRespBuffer.end());
        mem_req->second.second = true;
    }
}

void
GlobalMemPipeline::regStats()
{
    loadVrfBankConflictCycles
        .name(name() + ".load_vrf_bank_conflict_cycles")
        .desc("total number of cycles GM data are delayed before updating "
              "the VRF")
        ;
}
