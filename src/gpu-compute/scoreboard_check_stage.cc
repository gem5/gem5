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

#include "gpu-compute/scoreboard_check_stage.hh"

#include "debug/GPUExec.hh"
#include "debug/GPUSched.hh"
#include "debug/GPUSync.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"
#include "params/ComputeUnit.hh"

ScoreboardCheckStage::ScoreboardCheckStage(const ComputeUnitParams *p,
                                           ComputeUnit &cu,
                                           ScoreboardCheckToSchedule
                                           &to_schedule)
    : computeUnit(cu), toSchedule(to_schedule),
      _name(cu.name() + ".ScoreboardCheckStage")
{
}

ScoreboardCheckStage::~ScoreboardCheckStage()
{
}

void
ScoreboardCheckStage::collectStatistics(nonrdytype_e rdyStatus)
{
    panic_if(rdyStatus == NRDY_ILLEGAL || rdyStatus >= NRDY_CONDITIONS,
             "Instruction ready status %d is illegal!!!", rdyStatus);
    stallCycles[rdyStatus]++;
}

// Return true if this wavefront is ready
// to execute an instruction of the specified type.
// It also returns the reason (in rdyStatus) if the instruction is not
// ready. Finally it sets the execution resource type (in exesResType)
// of the instruction, only if it ready.
bool
ScoreboardCheckStage::ready(Wavefront *w, nonrdytype_e *rdyStatus,
                            int *exeResType, int wfSlot)
{
    /**
     * The waitCnt checks have to be done BEFORE checking for Instruction
     * buffer empty condition. Otherwise, it will result into a deadlock if
     * the last instruction in the Instruction buffer is a waitCnt: after
     * executing the waitCnt, the Instruction buffer would be empty and the
     * ready check logic will exit BEFORE checking for wait counters being
     * satisfied.
     */

    // waitCnt instruction has been dispatched or executed: next
    // instruction should be blocked until waitCnts are satisfied.
    if (w->getStatus() == Wavefront::S_WAITCNT) {
        if (!w->waitCntsSatisfied()) {
            *rdyStatus = NRDY_WAIT_CNT;
            return false;
        }
    }

    // Is the wave waiting at a barrier. Check this condition BEFORE checking
    // for instruction buffer occupancy to avoid a deadlock when the barrier is
    // the last instruction in the instruction buffer.
    if (w->getStatus() == Wavefront::S_BARRIER) {
        assert(w->hasBarrier());
        int bar_id = w->barrierId();
        if (!computeUnit.allAtBarrier(bar_id)) {
            DPRINTF(GPUSync, "CU[%d] WF[%d][%d] Wave[%d] - Stalled at "
                    "barrier Id%d. %d waves remain.\n", w->computeUnit->cu_id,
                    w->simdId, w->wfSlotId, w->wfDynId, bar_id,
                    w->computeUnit->numYetToReachBarrier(bar_id));
            // Are all threads at barrier?
            *rdyStatus = NRDY_BARRIER_WAIT;
            return false;
        }
        DPRINTF(GPUSync, "CU[%d] WF[%d][%d] Wave[%d] - All waves at barrier "
                "Id%d. Resetting barrier resources.\n", w->computeUnit->cu_id,
                w->simdId, w->wfSlotId, w->wfDynId, bar_id);
        computeUnit.resetBarrier(bar_id);
        computeUnit.releaseWFsFromBarrier(bar_id);
    }

    // Check WF status: it has to be running
    if (w->getStatus() == Wavefront::S_STOPPED ||
        w->getStatus() == Wavefront::S_RETURNING ||
        w->getStatus() == Wavefront::S_STALLED) {
        *rdyStatus = NRDY_WF_STOP;
        return false;
    }

    // is the Instruction buffer empty
    if ( w->instructionBuffer.empty()) {
        *rdyStatus = NRDY_IB_EMPTY;
        return false;
    }

    // Check next instruction from instruction buffer
    GPUDynInstPtr ii = w->nextInstr();
    // Only instruction in the instruction buffer has been dispatched.
    // No need to check it again for readiness
    if (!ii) {
        *rdyStatus = NRDY_IB_EMPTY;
        return false;
    }

    // The following code is very error prone and the entire process for
    // checking readiness will be fixed eventually.  In the meantime, let's
    // make sure that we do not silently let an instruction type slip
    // through this logic and always return not ready.
    if (!(ii->isBarrier() || ii->isNop() || ii->isReturn() || ii->isBranch() ||
         ii->isALU() || ii->isLoad() || ii->isStore() || ii->isAtomic() ||
         ii->isEndOfKernel() || ii->isMemSync() || ii->isFlat())) {
        panic("next instruction: %s is of unknown type\n", ii->disassemble());
    }

    DPRINTF(GPUExec, "CU%d: WF[%d][%d]: Checking Ready for Inst : %s\n",
            computeUnit.cu_id, w->simdId, w->wfSlotId, ii->disassemble());

    // Non-scalar (i.e., vector) instructions may use VGPRs
    if (!ii->isScalar()) {
        if (!computeUnit.vrf[w->simdId]->operandsReady(w, ii)) {
            *rdyStatus = NRDY_VGPR_NRDY;
            return false;
        }
    }
    // Scalar and non-scalar instructions may use SGPR
    if (!computeUnit.srf[w->simdId]->operandsReady(w, ii)) {
        *rdyStatus = NRDY_SGPR_NRDY;
        return false;
    }

    // The hardware implicitly executes S_WAITCNT 0 before executing
    // the S_ENDPGM instruction. Implementing this implicit S_WAITCNT.
    // isEndOfKernel() is used to identify the S_ENDPGM instruction
    // On identifying it, we do the following:
    // 1. Wait for all older instruction to execute
    // 2. Once all the older instruction are executed, we add a wait
    //    count for the executed instruction(s) to complete.
    if (ii->isEndOfKernel()) {
        // Waiting for older instruction to execute
        if (w->instructionBuffer.front()->seqNum() != ii->seqNum()) {
            *rdyStatus = NRDY_WAIT_CNT;
            return false;
        }
        // Older instructions have executed, adding implicit wait count
        w->setStatus(Wavefront::S_WAITCNT);
        w->setWaitCnts(0, 0, 0);
        if (!w->waitCntsSatisfied()) {
            *rdyStatus = NRDY_WAIT_CNT;
            return false;
        }
    }
    DPRINTF(GPUExec, "CU%d: WF[%d][%d]: Ready Inst : %s\n", computeUnit.cu_id,
            w->simdId, w->wfSlotId, ii->disassemble());
    *exeResType = mapWaveToExeUnit(w);
    *rdyStatus = INST_RDY;
    return true;
}

int
ScoreboardCheckStage::mapWaveToExeUnit(Wavefront *w)
{
    GPUDynInstPtr ii = w->nextInstr();
    assert(ii);
    if (ii->isFlat()) {
        /**
         * NOTE: Flat memory ops requires both GM and LM resources.
         * The simulator models consumption of both GM and LM
         * resources in the schedule stage. At instruction execution time,
         * after the aperture check is performed, only the GM or LM pipe
         * is actually reserved by the timing model. The GM unit is returned
         * here since Flat ops occupy the GM slot in the ready and dispatch
         * lists. They also consume the LM slot in the dispatch list.
         */
        return w->globalMem;
    } else if (ii->isLocalMem()) {
        return w->localMem;
    } else if (ii->isGlobalMem()) {
        if (!ii->isScalar()) {
            return w->globalMem;
        } else {
            return w->scalarMem;
        }
    } else if (ii->isBranch() ||
               ii->isALU() ||
               (ii->isKernArgSeg() && ii->isLoad()) ||
               ii->isArgSeg() ||
               ii->isReturn() ||
               ii->isEndOfKernel() ||
               ii->isNop() ||
               ii->isBarrier()) {
        if (!ii->isScalar()) {
            return w->simdId;
        } else {
            return w->scalarAluGlobalIdx;
        }
    }
    panic("%s: unmapped to an execution resource", ii->disassemble());
    return computeUnit.numExeUnits();
}

void
ScoreboardCheckStage::exec()
{
    /**
     * Reset the ready list for all execution units; ready list will be
     * constructed every cycle because resource availability may change.
     */
    toSchedule.reset();

    // Iterate over all WF slots across all SIMDs.
    for (int simdId = 0; simdId < computeUnit.numVectorALUs; ++simdId) {
        for (int wfSlot = 0; wfSlot < computeUnit.shader->n_wf; ++wfSlot) {
            // reset the ready status of each wavefront
            Wavefront *curWave = computeUnit.wfList[simdId][wfSlot];
            nonrdytype_e rdyStatus = NRDY_ILLEGAL;
            int exeResType = -1;
            // check WF readiness: If the WF's oldest
            // instruction is ready to issue then add the WF to the ready list
            if (ready(curWave, &rdyStatus, &exeResType, wfSlot)) {
                assert(curWave->simdId == simdId);
                DPRINTF(GPUSched,
                        "Adding to readyList[%d]: SIMD[%d] WV[%d]: %d: %s\n",
                        exeResType,
                        curWave->simdId, curWave->wfDynId,
                        curWave->nextInstr()->seqNum(),
                        curWave->nextInstr()->disassemble());
                toSchedule.markWFReady(curWave, exeResType);
            }
            collectStatistics(rdyStatus);
        }
    }
}

void
ScoreboardCheckStage::regStats()
{
    stallCycles
        .init(NRDY_CONDITIONS)
        .name(name() + ".stall_cycles")
        .desc("number of cycles wave stalled in SCB")
        ;
    stallCycles.subname(NRDY_WF_STOP, csprintf("WFStop"));
    stallCycles.subname(NRDY_IB_EMPTY, csprintf("IBEmpty"));
    stallCycles.subname(NRDY_WAIT_CNT, csprintf("WaitCnt"));
    stallCycles.subname(NRDY_BARRIER_WAIT, csprintf("BarrierWait"));
    stallCycles.subname(NRDY_VGPR_NRDY, csprintf("VgprBusy"));
    stallCycles.subname(NRDY_SGPR_NRDY, csprintf("SgprBusy"));
    stallCycles.subname(INST_RDY, csprintf("InstrReady"));
}
