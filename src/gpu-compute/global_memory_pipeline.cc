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

#define __STDC_FORMAT_MACROS
#include <cinttypes>
#include "debug/GPUCoalescer.hh"
#include "debug/GPUMem.hh"
#include "debug/GPUReg.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/global_memory_pipeline.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

GlobalMemPipeline::GlobalMemPipeline(const ComputeUnitParams &p,
                                     ComputeUnit &cu)
    : computeUnit(cu), _name(cu.name() + ".GlobalMemPipeline"),
      gmQueueSize(p.global_mem_queue_size),
      maxWaveRequests(p.max_wave_requests), inflightStores(0),
      inflightLoads(0), stats(&cu)
{
}

void
GlobalMemPipeline::init()
{
    globalMemSize = computeUnit.shader->globalMemSize;
}

bool
GlobalMemPipeline::coalescerReady(GPUDynInstPtr mp) const
{
    // System requests do not need GPU coalescer tokens. Make sure nothing
    // has bypassed the operand gather check stage.
    assert(!mp->isSystemReq());

    // We require one token from the coalescer's uncoalesced table to
    // proceed
    int token_count = 1;

    // Make sure the vector port has tokens. There is a single pool
    // of tokens so only one port in the vector port needs to be checked.
    // Lane 0 is chosen arbirarily.
    DPRINTF(GPUCoalescer, "Checking for %d tokens\n", token_count);
    if (!mp->computeUnit()->getTokenManager()->haveTokens(token_count)) {
        DPRINTF(GPUCoalescer, "Stalling inst because coalsr is busy!\n");
        return false;
    }

    return true;
}

void
GlobalMemPipeline::acqCoalescerToken(GPUDynInstPtr mp)
{
    // We require one token from the coalescer's uncoalesced table to
    // proceed
    int token_count = 1;

    DPRINTF(GPUCoalescer, "Acquiring %d token(s)\n", token_count);
    assert(mp->computeUnit()->getTokenManager()->haveTokens(token_count));
    mp->computeUnit()->getTokenManager()->acquireTokens(token_count);
}

bool
GlobalMemPipeline::outstandingReqsCheck(GPUDynInstPtr mp) const
{
    // Ensure we haven't exceeded the maximum number of vmem requests
    // for this wavefront
    if ((mp->wavefront()->outstandingReqsRdGm
         + mp->wavefront()->outstandingReqsWrGm) >= maxWaveRequests) {
        return false;
    }

    return true;
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
    if (m && (m->isLoad() || m->isAtomicRet())) {
        w = m->wavefront();

        accessVrf = w->computeUnit->vrf[w->simdId]->
            canScheduleWriteOperandsFromLoad(w, m);

    }

    if (m && m->latency.rdy() && computeUnit.glbMemToVrfBus.rdy() &&
        accessVrf && (computeUnit.shader->coissue_return ||
        computeUnit.vectorGlobalMemUnit.rdy())) {

        w = m->wavefront();

        DPRINTF(GPUMem, "CU%d: WF[%d][%d]: Completing global mem instr %s\n",
                m->cu_id, m->simdId, m->wfSlotId, m->disassemble());
        m->completeAcc(m);
        if (m->isFlat()) {
            w->decLGKMInstsIssued();
        }
        w->decVMemInstsIssued();

        if (m->isLoad() || m->isAtomicRet()) {
            w->computeUnit->vrf[w->simdId]->
            scheduleWriteOperandsFromLoad(w, m);
        }

        completeRequest(m);

        Tick accessTime = curTick() - m->getAccessTime();

        // Decrement outstanding requests count
        computeUnit.shader->ScheduleAdd(&w->outstandingReqs, m->time, -1);
        if (m->isStore() || m->isAtomic() || m->isMemSync()) {
            computeUnit.shader->sampleStore(accessTime);
            computeUnit.shader->ScheduleAdd(&w->outstandingReqsWrGm,
                                             m->time, -1);
        }

        if (m->isLoad() || m->isAtomic() || m->isMemSync()) {
            computeUnit.shader->sampleLoad(accessTime);
            computeUnit.shader->ScheduleAdd(&w->outstandingReqsRdGm,
                                             m->time, -1);
        }

        w->validateRequestCounters();

        // Generate stats for round-trip time for vectory memory insts
        // going all the way to memory and stats for individual cache
        // blocks generated by the instruction.
        m->profileRoundTripTime(curTick(), InstMemoryHop::Complete);
        computeUnit.shader->sampleInstRoundTrip(m->getRoundTripTime());
        computeUnit.shader->sampleLineRoundTrip(m->getLineAddressTime());

        // Mark write bus busy for appropriate amount of time
        computeUnit.glbMemToVrfBus.set(m->time);
        if (!computeUnit.shader->coissue_return)
            w->computeUnit->vectorGlobalMemUnit.set(m->time);
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

        DPRINTF(GPUCoalescer, "initiateAcc for %s seqNum %d\n",
                mp->disassemble(), mp->seqNum());
        mp->initiateAcc(mp);

        if (mp->isStore() && mp->isGlobalSeg()) {
            mp->wavefront()->decExpInstsIssued();
        }

        if (((mp->isMemSync() && !mp->isEndOfKernel()) || !mp->isMemSync())) {
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

        if (!mp->isMemSync() && !mp->isEndOfKernel() && mp->allLanesZero()) {
            /**
            * Memory accesses instructions that do not generate any memory
            * requests (such as out-of-bounds buffer acceses where all lanes
            * are out of bounds) will not trigger a callback to complete the
            * request, so we need to mark it as completed as soon as it is
            * issued.  Note this this will still insert an entry in the
            * ordered return FIFO such that waitcnt is still resolved
            * correctly.
            */
            handleResponse(mp);
            computeUnit.getTokenManager()->recvTokens(1);
        }

        gmIssuedRequests.pop();

        DPRINTF(GPUMem, "CU%d: WF[%d][%d] Popping 0 mem_op = \n",
                computeUnit.cu_id, mp->simdId, mp->wfSlotId);
    }
}

GPUDynInstPtr
GlobalMemPipeline::getNextReadyResp()
{
    if (!gmOrderedRespBuffer.empty()) {
        auto mem_req = gmOrderedRespBuffer.begin();

        if (mem_req->second.second) {
            return mem_req->second.first;
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

    // we should only pop the oldest requst, and it
    // should be marked as done if we are here
    assert(gmOrderedRespBuffer.begin()->first == gpuDynInst->seqNum());
    assert(gmOrderedRespBuffer.begin()->second.first == gpuDynInst);
    assert(gmOrderedRespBuffer.begin()->second.second);
    // remove this instruction from the buffer by its
    // unique seq ID
    gmOrderedRespBuffer.erase(gpuDynInst->seqNum());
}

void
GlobalMemPipeline::issueRequest(GPUDynInstPtr gpuDynInst)
{
    Wavefront *wf = gpuDynInst->wavefront();
    if (gpuDynInst->isLoad()) {
        wf->rdGmReqsInPipe--;
        wf->outstandingReqsRdGm++;
    } else if (gpuDynInst->isStore()) {
        wf->wrGmReqsInPipe--;
        wf->outstandingReqsWrGm++;
    } else {
        // Atomic, both read and write
        wf->rdGmReqsInPipe--;
        wf->outstandingReqsRdGm++;
        wf->wrGmReqsInPipe--;
        wf->outstandingReqsWrGm++;
    }

    wf->outstandingReqs++;
    wf->validateRequestCounters();

    gpuDynInst->setAccessTime(curTick());
    gpuDynInst->profileRoundTripTime(curTick(), InstMemoryHop::Initiate);
    gmIssuedRequests.push(gpuDynInst);
}

void
GlobalMemPipeline::handleResponse(GPUDynInstPtr gpuDynInst)
{
    auto mem_req = gmOrderedRespBuffer.find(gpuDynInst->seqNum());
    // if we are getting a response for this mem request,
    // then it ought to already be in the ordered response
    // buffer
    assert(mem_req != gmOrderedRespBuffer.end());
    mem_req->second.second = true;
}

GlobalMemPipeline::
GlobalMemPipelineStats::GlobalMemPipelineStats(statistics::Group *parent)
    : statistics::Group(parent, "GlobalMemPipeline"),
      ADD_STAT(loadVrfBankConflictCycles, "total number of cycles GM data "
               "are delayed before updating the VRF")
{
}

} // namespace gem5
