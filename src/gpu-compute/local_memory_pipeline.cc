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
 * Author: Sooraj Puthoor
 */

#include "gpu-compute/local_memory_pipeline.hh"

#include "debug/GPUPort.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

LocalMemPipeline::LocalMemPipeline(const ComputeUnitParams* p) :
    computeUnit(nullptr), lmQueueSize(p->local_mem_queue_size)
{
}

void
LocalMemPipeline::init(ComputeUnit *cu)
{
    computeUnit = cu;
    _name = computeUnit->name() + ".LocalMemPipeline";
}

void
LocalMemPipeline::exec()
{
    // apply any returned shared (LDS) memory operations
    GPUDynInstPtr m = !lmReturnedRequests.empty() ?
        lmReturnedRequests.front() : nullptr;

    bool accessVrf = true;
    if ((m) && (m->m_op==Enums::MO_LD || MO_A(m->m_op))) {
        Wavefront *w = computeUnit->wfList[m->simdId][m->wfSlotId];

        accessVrf =
            w->computeUnit->vrf[m->simdId]->
            vrfOperandAccessReady(m->seqNum(), w, m,
                                  VrfAccessType::WRITE);
    }

    if (!lmReturnedRequests.empty() && m->latency.rdy() && accessVrf &&
        computeUnit->locMemToVrfBus.rdy() && (computeUnit->shader->coissue_return
                 || computeUnit->wfWait.at(m->pipeId).rdy())) {
        if (m->v_type == VT_32 && m->m_type == Enums::M_U8)
            doSmReturn<uint32_t, uint8_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_U16)
            doSmReturn<uint32_t, uint16_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_U32)
            doSmReturn<uint32_t, uint32_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_S8)
            doSmReturn<int32_t, int8_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_S16)
            doSmReturn<int32_t, int16_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_S32)
            doSmReturn<int32_t, int32_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_F16)
            doSmReturn<float, Float16>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_F32)
            doSmReturn<float, float>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_U8)
            doSmReturn<uint64_t, uint8_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_U16)
            doSmReturn<uint64_t, uint16_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_U32)
            doSmReturn<uint64_t, uint32_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_U64)
            doSmReturn<uint64_t, uint64_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_S8)
            doSmReturn<int64_t, int8_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_S16)
            doSmReturn<int64_t, int16_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_S32)
            doSmReturn<int64_t, int32_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_S64)
            doSmReturn<int64_t, int64_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_F16)
            doSmReturn<double, Float16>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_F32)
            doSmReturn<double, float>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_F64)
            doSmReturn<double, double>(m);
    }

    // If pipeline has executed a local memory instruction
    // execute local memory packet and issue the packets
    // to LDS
    if (!lmIssuedRequests.empty() && lmReturnedRequests.size() < lmQueueSize) {

        GPUDynInstPtr m = lmIssuedRequests.front();

        bool returnVal = computeUnit->sendToLds(m);
        if (!returnVal) {
            DPRINTF(GPUPort, "packet was nack'd and put in retry queue");
        }
        lmIssuedRequests.pop();
    }
}

template<typename c0, typename c1>
void
LocalMemPipeline::doSmReturn(GPUDynInstPtr m)
{
    lmReturnedRequests.pop();
    Wavefront *w = computeUnit->wfList[m->simdId][m->wfSlotId];

    // Return data to registers
    if (m->m_op == Enums::MO_LD || MO_A(m->m_op)) {
        std::vector<uint32_t> regVec;
        for (int k = 0; k < m->n_reg; ++k) {
            int dst = m->dst_reg+k;

            if (m->n_reg > MAX_REGS_FOR_NON_VEC_MEM_INST)
                dst = m->dst_reg_vec[k];
            // virtual->physical VGPR mapping
            int physVgpr = w->remap(dst,sizeof(c0),1);
            // save the physical VGPR index
            regVec.push_back(physVgpr);
            c1 *p1 = &((c1*)m->d_data)[k * VSZ];

            for (int i = 0; i < VSZ; ++i) {
                if (m->exec_mask[i]) {
                    // write the value into the physical VGPR. This is a purely
                    // functional operation. No timing is modeled.
                    w->computeUnit->vrf[w->simdId]->write<c0>(physVgpr,
                                                                *p1, i);
                }
                ++p1;
            }
        }

        // Schedule the write operation of the load data on the VRF. This simply
        // models the timing aspect of the VRF write operation. It does not
        // modify the physical VGPR.
        loadVrfBankConflictCycles +=
            w->computeUnit->vrf[w->simdId]->exec(m->seqNum(), w,
                                                 regVec, sizeof(c0), m->time);
    }

    // Decrement outstanding request count
    computeUnit->shader->ScheduleAdd(&w->outstanding_reqs, m->time, -1);

    if (m->m_op == Enums::MO_ST || MO_A(m->m_op) || MO_ANR(m->m_op)
        || MO_H(m->m_op)) {
        computeUnit->shader->ScheduleAdd(&w->outstanding_reqs_wr_lm,
                                         m->time, -1);
    }

    if (m->m_op == Enums::MO_LD || MO_A(m->m_op) || MO_ANR(m->m_op)) {
        computeUnit->shader->ScheduleAdd(&w->outstanding_reqs_rd_lm,
                                         m->time, -1);
    }

    // Mark write bus busy for appropriate amount of time
    computeUnit->locMemToVrfBus.set(m->time);
    if (computeUnit->shader->coissue_return == 0)
        w->computeUnit->wfWait.at(m->pipeId).set(m->time);
}

void
LocalMemPipeline::regStats()
{
    loadVrfBankConflictCycles
        .name(name() + ".load_vrf_bank_conflict_cycles")
        .desc("total number of cycles LDS data are delayed before updating "
              "the VRF")
        ;
}
