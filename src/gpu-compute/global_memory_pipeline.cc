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
    inflightStores(0), inflightLoads(0)
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
    GPUDynInstPtr m = !gmReturnedLoads.empty() ? gmReturnedLoads.front() :
        !gmReturnedStores.empty() ? gmReturnedStores.front() : nullptr;

    bool accessVrf = true;
    // check the VRF to see if the operands of a load (or load component
    // of an atomic) are accessible
    if ((m) && (m->m_op==Enums::MO_LD || MO_A(m->m_op))) {
        Wavefront *w = computeUnit->wfList[m->simdId][m->wfSlotId];

        accessVrf =
            w->computeUnit->vrf[m->simdId]->
            vrfOperandAccessReady(m->seqNum(), w, m,
                                  VrfAccessType::WRITE);
    }

    if ((!gmReturnedStores.empty() || !gmReturnedLoads.empty()) &&
        m->latency.rdy() && computeUnit->glbMemToVrfBus.rdy() &&
        accessVrf && m->statusBitVector == VectorMask(0) &&
        (computeUnit->shader->coissue_return ||
         computeUnit->wfWait.at(m->pipeId).rdy())) {

        if (m->v_type == VT_32 && m->m_type == Enums::M_U8)
            doGmReturn<uint32_t, uint8_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_U16)
            doGmReturn<uint32_t, uint16_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_U32)
            doGmReturn<uint32_t, uint32_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_S8)
            doGmReturn<int32_t, int8_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_S16)
            doGmReturn<int32_t, int16_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_S32)
            doGmReturn<int32_t, int32_t>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_F16)
            doGmReturn<float, Float16>(m);
        else if (m->v_type == VT_32 && m->m_type == Enums::M_F32)
            doGmReturn<float, float>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_U8)
            doGmReturn<uint64_t, uint8_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_U16)
            doGmReturn<uint64_t, uint16_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_U32)
            doGmReturn<uint64_t, uint32_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_U64)
            doGmReturn<uint64_t, uint64_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_S8)
            doGmReturn<int64_t, int8_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_S16)
            doGmReturn<int64_t, int16_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_S32)
            doGmReturn<int64_t, int32_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_S64)
            doGmReturn<int64_t, int64_t>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_F16)
            doGmReturn<double, Float16>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_F32)
            doGmReturn<double, float>(m);
        else if (m->v_type == VT_64 && m->m_type == Enums::M_F64)
            doGmReturn<double, double>(m);
    }

    // If pipeline has executed a global memory instruction
    // execute global memory packets and issue global
    // memory packets to DTLB
    if (!gmIssuedRequests.empty()) {
        GPUDynInstPtr mp = gmIssuedRequests.front();
        if (mp->m_op == Enums::MO_LD ||
            (mp->m_op >= Enums::MO_AAND && mp->m_op <= Enums::MO_AMIN) ||
            (mp->m_op >= Enums::MO_ANRAND && mp->m_op <= Enums::MO_ANRMIN)) {

            if (inflightLoads >= gmQueueSize) {
                return;
            } else {
                ++inflightLoads;
            }
        } else {
            if (inflightStores >= gmQueueSize) {
                return;
            } else if (mp->m_op == Enums::MO_ST) {
                ++inflightStores;
            }
        }

        mp->initiateAcc(mp);
        gmIssuedRequests.pop();

        DPRINTF(GPUMem, "CU%d: WF[%d][%d] Popping 0 mem_op = %s\n",
                computeUnit->cu_id, mp->simdId, mp->wfSlotId,
                Enums::MemOpTypeStrings[mp->m_op]);
    }
}

template<typename c0, typename c1>
void
GlobalMemPipeline::doGmReturn(GPUDynInstPtr m)
{
    Wavefront *w = computeUnit->wfList[m->simdId][m->wfSlotId];

    // Return data to registers
    if (m->m_op == Enums::MO_LD || MO_A(m->m_op) || MO_ANR(m->m_op)) {
        gmReturnedLoads.pop();
        assert(inflightLoads > 0);
        --inflightLoads;

        if (m->m_op == Enums::MO_LD || MO_A(m->m_op)) {
            std::vector<uint32_t> regVec;
            // iterate over number of destination register operands since
            // this is a load or atomic operation
            for (int k = 0; k < m->n_reg; ++k) {
                assert((sizeof(c1) * m->n_reg) <= MAX_WIDTH_FOR_MEM_INST);
                int dst = m->dst_reg + k;

                if (m->n_reg > MAX_REGS_FOR_NON_VEC_MEM_INST)
                    dst = m->dst_reg_vec[k];
                // virtual->physical VGPR mapping
                int physVgpr = w->remap(dst, sizeof(c0), 1);
                // save the physical VGPR index
                regVec.push_back(physVgpr);
                c1 *p1 = &((c1*)m->d_data)[k * VSZ];

                for (int i = 0; i < VSZ; ++i) {
                    if (m->exec_mask[i]) {
                        DPRINTF(GPUReg, "CU%d, WF[%d][%d], lane %d: "
                                "$%s%d <- %d global ld done (src = wavefront "
                                "ld inst)\n", w->computeUnit->cu_id, w->simdId,
                                w->wfSlotId, i, sizeof(c0) == 4 ? "s" : "d",
                                dst, *p1);
                        // write the value into the physical VGPR. This is a
                        // purely functional operation. No timing is modeled.
                        w->computeUnit->vrf[w->simdId]->write<c0>(physVgpr,
                                                                    *p1, i);
                    }
                    ++p1;
                }
            }

            // Schedule the write operation of the load data on the VRF.
            // This simply models the timing aspect of the VRF write operation.
            // It does not modify the physical VGPR.
            loadVrfBankConflictCycles +=
                w->computeUnit->vrf[w->simdId]->exec(m->seqNum(),
                                                     w, regVec, sizeof(c0),
                                                     m->time);
        }
    } else {
        gmReturnedStores.pop();
        assert(inflightStores > 0);
        --inflightStores;
    }

    // Decrement outstanding register count
    computeUnit->shader->ScheduleAdd(&w->outstanding_reqs, m->time, -1);

    if (m->m_op == Enums::MO_ST || MO_A(m->m_op) || MO_ANR(m->m_op) ||
        MO_H(m->m_op)) {
        computeUnit->shader->ScheduleAdd(&w->outstanding_reqs_wr_gm, m->time,
                                         -1);
    }

    if (m->m_op == Enums::MO_LD || MO_A(m->m_op) || MO_ANR(m->m_op)) {
        computeUnit->shader->ScheduleAdd(&w->outstanding_reqs_rd_gm, m->time,
                                         -1);
    }

    // Mark write bus busy for appropriate amount of time
    computeUnit->glbMemToVrfBus.set(m->time);
    if (!computeUnit->shader->coissue_return)
        w->computeUnit->wfWait.at(m->pipeId).set(m->time);
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
