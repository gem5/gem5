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

#include "gpu-compute/schedule_stage.hh"

#include <unordered_set>

#include "base/compiler.hh"
#include "debug/GPUSched.hh"
#include "debug/GPUVRF.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

ScheduleStage::ScheduleStage(const ComputeUnitParams &p, ComputeUnit &cu,
                             ScoreboardCheckToSchedule &from_scoreboard_check,
                             ScheduleToExecute &to_execute)
    : computeUnit(cu), fromScoreboardCheck(from_scoreboard_check),
      toExecute(to_execute),
      _name(cu.name() + ".ScheduleStage"),
      vectorAluRdy(false), scalarAluRdy(false), scalarMemBusRdy(false),
      scalarMemIssueRdy(false), glbMemBusRdy(false), glbMemIssueRdy(false),
      locMemBusRdy(false), locMemIssueRdy(false), stats(&cu, cu.numExeUnits())
{
    for (int j = 0; j < cu.numExeUnits(); ++j) {
        scheduler.emplace_back(p);
    }
    wavesInSch.clear();
    schList.resize(cu.numExeUnits());
    for (auto &dq : schList) {
        dq.clear();
    }
}

ScheduleStage::~ScheduleStage()
{
    scheduler.clear();
    wavesInSch.clear();
    schList.clear();
}

void
ScheduleStage::init()
{

    fatal_if(scheduler.size() != fromScoreboardCheck.numReadyLists(),
             "Scheduler should have same number of entries as CU's readyList");
    for (int j = 0; j < computeUnit.numExeUnits(); ++j) {
        scheduler[j].bindList(&fromScoreboardCheck.readyWFs(j));
    }

    assert(computeUnit.numVectorGlobalMemUnits == 1);
    assert(computeUnit.numVectorSharedMemUnits == 1);
}

void
ScheduleStage::exec()
{
    toExecute.reset();

    // Update readyList
    for (int j = 0; j < computeUnit.numExeUnits(); ++j) {
        /**
         * Remove any wave that already has an instruction present in SCH
         * waiting for RF reads to complete. This prevents out of order
         * execution within a wave.
         */
        fromScoreboardCheck.updateReadyList(j);
        for (auto wIt = fromScoreboardCheck.readyWFs(j).begin();
             wIt != fromScoreboardCheck.readyWFs(j).end();) {
            if (wavesInSch.find((*wIt)->wfDynId) != wavesInSch.end()) {
                *wIt = nullptr;
                wIt = fromScoreboardCheck.readyWFs(j).erase(wIt);
            } else {
                wIt++;
            }
        }
    }

    // Attempt to add another wave for each EXE type to schList queues
    // VMEM resources are iterated first, effectively giving priority
    // to VMEM over VALU for scheduling read of operands to the RFs.
    // Scalar Memory are iterated after VMEM

    // Iterate VMEM and SMEM
    int firstMemUnit = computeUnit.firstMemUnit();
    int lastMemUnit = computeUnit.lastMemUnit();
    for (int j = firstMemUnit; j <= lastMemUnit; j++) {
        int readyListSize = fromScoreboardCheck.readyWFs(j).size();
        // If no wave is ready to be scheduled on the execution resource
        // then skip scheduling for this execution resource
        if (!readyListSize) {
            stats.rdyListEmpty[j]++;
            continue;
        }
        stats.rdyListNotEmpty[j]++;

        // Pick a wave and attempt to add it to schList
        Wavefront *wf = scheduler[j].chooseWave();
        GPUDynInstPtr &gpu_dyn_inst = wf->instructionBuffer.front();
        assert(gpu_dyn_inst);
        if (!addToSchList(j, gpu_dyn_inst)) {
            // For waves not added to schList, increment count of cycles
            // this wave spends in SCH stage.
            wf->stats.schCycles++;
            stats.addToSchListStalls[j]++;
        } else {
            if (gpu_dyn_inst->isScalar() || gpu_dyn_inst->isGroupSeg()) {
                wf->incLGKMInstsIssued();
            } else {
                wf->incVMemInstsIssued();
                if (gpu_dyn_inst->isFlat()) {
                    wf->incLGKMInstsIssued();
                }
            }
            if (gpu_dyn_inst->isStore() && gpu_dyn_inst->isGlobalSeg()) {
                wf->incExpInstsIssued();
            }
        }
    }

    // Iterate everything else
    for (int j = 0; j < computeUnit.numExeUnits(); ++j) {
        // skip the VMEM resources
        if (j >= firstMemUnit && j <= lastMemUnit) {
            continue;
        }
        int readyListSize = fromScoreboardCheck.readyWFs(j).size();
        // If no wave is ready to be scheduled on the execution resource
        // then skip scheduling for this execution resource
        if (!readyListSize) {
            stats.rdyListEmpty[j]++;
            continue;
        }
        stats.rdyListNotEmpty[j]++;

        // Pick a wave and attempt to add it to schList
        Wavefront *wf = scheduler[j].chooseWave();
        GPUDynInstPtr &gpu_dyn_inst = wf->instructionBuffer.front();
        assert(gpu_dyn_inst);
        if (!addToSchList(j, gpu_dyn_inst)) {
            // For waves not added to schList, increment count of cycles
            // this wave spends in SCH stage.
            wf->stats.schCycles++;
            stats.addToSchListStalls[j]++;
        }
    }

    // At this point, the schList queue per EXE type may contain
    // multiple waves, in order of age (oldest to youngest).
    // Wave may be in RFBUSY, indicating they are waiting for registers
    // to be read, or in RFREADY, indicating they are candidates for
    // the dispatchList and execution

    // Iterate schList queues and check if any of the waves have finished
    // reading their operands, moving those waves to RFREADY status
    checkRfOperandReadComplete();

    // Fill the dispatch list with the oldest wave of each EXE type that
    // is ready to execute
    // Wave is picked if status in schList is RFREADY and it passes resource
    // ready checks similar to those currently in SCB
    fillDispatchList();

    // Resource arbitration on waves in dispatchList
    // Losing waves are re-inserted to the schList at a location determined
    // by wave age

    // Arbitrate access to the VRF->LDS bus
    arbitrateVrfToLdsBus();

    // Schedule write operations to the register files
    scheduleRfDestOperands();

    // Lastly, reserve resources for waves that are ready to execute.
    reserveResources();
}

void
ScheduleStage::doDispatchListTransition(int unitId, DISPATCH_STATUS s,
                                        const GPUDynInstPtr &gpu_dyn_inst)
{
    toExecute.dispatchTransition(gpu_dyn_inst, unitId, s);
}

void
ScheduleStage::doDispatchListTransition(int unitId, DISPATCH_STATUS s)
{
    toExecute.dispatchTransition(unitId, s);
}

bool
ScheduleStage::schedRfWrites(int exeType, const GPUDynInstPtr &gpu_dyn_inst)
{
    assert(gpu_dyn_inst);
    Wavefront *wf = gpu_dyn_inst->wavefront();
    bool accessVrfWr = true;
    if (!gpu_dyn_inst->isScalar()) {
        accessVrfWr = computeUnit.vrf[wf->simdId]
            ->canScheduleWriteOperands(wf, gpu_dyn_inst);
    }
    bool accessSrfWr = computeUnit.srf[wf->simdId]
        ->canScheduleWriteOperands(wf, gpu_dyn_inst);
    bool accessRf = accessVrfWr && accessSrfWr;
    if (accessRf) {
        if (!gpu_dyn_inst->isScalar()) {
            computeUnit.vrf[wf->simdId]->scheduleWriteOperands(wf,
                                                               gpu_dyn_inst);
        }
        computeUnit.srf[wf->simdId]->scheduleWriteOperands(wf, gpu_dyn_inst);
        return true;
    } else {
        stats.rfAccessStalls[SCH_RF_ACCESS_NRDY]++;
        if (!accessSrfWr) {
            stats.rfAccessStalls[SCH_SRF_WR_ACCESS_NRDY]++;
        }
        if (!accessVrfWr) {
            stats.rfAccessStalls[SCH_VRF_WR_ACCESS_NRDY]++;
        }

        // Increment stall counts for WF
        wf->stats.schStalls++;
        wf->stats.schRfAccessStalls++;
    }
    return false;
}

void
ScheduleStage::scheduleRfDestOperands()
{
    for (int j = 0; j < computeUnit.numExeUnits(); ++j) {
        if (toExecute.dispatchStatus(j) == EMPTY ||
            toExecute.dispatchStatus(j) == SKIP) {
            continue;
        }

        // get the wave on dispatch list and attempt to allocate write
        // resources in the RFs
        const GPUDynInstPtr &gpu_dyn_inst = toExecute.readyInst(j);
        assert(gpu_dyn_inst);
        Wavefront *wf = gpu_dyn_inst->wavefront();
        if (!schedRfWrites(j, gpu_dyn_inst)) {
            reinsertToSchList(j, gpu_dyn_inst);
            doDispatchListTransition(j, EMPTY);
            // if this is a flat inst, also transition the LM pipe to empty
            // Note: since FLAT/LM arbitration occurs before scheduling
            // destination operands to the RFs, it is possible that a LM
            // instruction lost arbitration, but would have been able to
            // pass the RF destination operand check here, and execute
            // instead of the FLAT.
            if (wf->instructionBuffer.front()->isFlat()) {
                assert(toExecute.dispatchStatus(wf->localMem)
                       == SKIP);
                doDispatchListTransition(wf->localMem, EMPTY);
            }
        }
    }
}

bool
ScheduleStage::addToSchList(int exeType, const GPUDynInstPtr &gpu_dyn_inst)
{
    // Attempt to add the wave to the schList if the VRF can support the
    // wave's next instruction
    assert(gpu_dyn_inst);
    Wavefront *wf = gpu_dyn_inst->wavefront();
    bool accessVrf = true;
    if (!gpu_dyn_inst->isScalar()) {
        accessVrf = computeUnit.vrf[wf->simdId]
            ->canScheduleReadOperands(wf, gpu_dyn_inst);
    }
    bool accessSrf = computeUnit.srf[wf->simdId]
        ->canScheduleReadOperands(wf, gpu_dyn_inst);
    // If RFs can support instruction, add to schList in RFBUSY state,
    // place wave in wavesInSch and pipeMap, and schedule Rd/Wr operands
    // to the VRF
    bool accessRf = accessVrf && accessSrf;
    if (accessRf) {
        DPRINTF(GPUSched, "schList[%d]: Adding: SIMD[%d] WV[%d]: %d: %s\n",
                exeType, wf->simdId, wf->wfDynId,
                gpu_dyn_inst->seqNum(), gpu_dyn_inst->disassemble());

        computeUnit.insertInPipeMap(wf);
        wavesInSch.emplace(wf->wfDynId);
        schList.at(exeType).push_back(std::make_pair(gpu_dyn_inst, RFBUSY));
        if (wf->isOldestInstBarrier() && wf->hasBarrier()) {
            wf->setStatus(Wavefront::S_BARRIER);
        }
        if (wf->isOldestInstWaitcnt()) {
            wf->setStatus(Wavefront::S_WAITCNT);
        }
        if (wf->isOldestInstSleep()) {
            wf->setStatus(Wavefront::S_STALLED_SLEEP);
        }
        if (!gpu_dyn_inst->isScalar()) {
            computeUnit.vrf[wf->simdId]
                ->scheduleReadOperands(wf, gpu_dyn_inst);
        }
        computeUnit.srf[wf->simdId]->scheduleReadOperands(wf, gpu_dyn_inst);

        DPRINTF(GPUSched, "schList[%d]: Added: SIMD[%d] WV[%d]: %d: %s\n",
                exeType, wf->simdId, wf->wfDynId,
                gpu_dyn_inst->seqNum(), gpu_dyn_inst->disassemble());
        return true;
    } else {
        // Number of stall cycles due to RF access denied
        stats.rfAccessStalls[SCH_RF_ACCESS_NRDY]++;
        // Count number of denials due to each reason
        // Multiple items may contribute to the denied request
        if (!accessVrf) {
            stats.rfAccessStalls[SCH_VRF_RD_ACCESS_NRDY]++;
        }
        if (!accessSrf) {
            stats.rfAccessStalls[SCH_SRF_RD_ACCESS_NRDY]++;
        }

        // Increment stall counts for WF
        wf->stats.schStalls++;
        wf->stats.schRfAccessStalls++;
        DPRINTF(GPUSched, "schList[%d]: Could not add: "
                "SIMD[%d] WV[%d]: %d: %s\n",
                exeType, wf->simdId, wf->wfDynId,
                gpu_dyn_inst->seqNum(), gpu_dyn_inst->disassemble());
    }
    return false;
}

void
ScheduleStage::reinsertToSchList(int exeType,
                                 const GPUDynInstPtr &gpu_dyn_inst)
{
    // Insert wave w into schList for specified exeType.
    // Wave is inserted in age order, with oldest wave being at the
    // front of the schList
    assert(gpu_dyn_inst);
    auto schIter = schList.at(exeType).begin();
    while (schIter != schList.at(exeType).end()
           && schIter->first->wfDynId < gpu_dyn_inst->wfDynId) {
        schIter++;
    }
    schList.at(exeType).insert(schIter, std::make_pair(gpu_dyn_inst, RFREADY));
}

void
ScheduleStage::checkMemResources()
{
    // Check for resource availability in the next cycle
    scalarMemBusRdy = false;
    scalarMemIssueRdy = false;
    // check if there is a SRF->Global Memory bus available and
    if (computeUnit.srfToScalarMemPipeBus.rdy(Cycles(1))) {
        scalarMemBusRdy = true;
    }
    // check if we can issue a scalar memory instruction
    if (computeUnit.scalarMemUnit.rdy(Cycles(1))) {
        scalarMemIssueRdy = true;
    }

    glbMemBusRdy = false;
    glbMemIssueRdy = false;
    // check if there is a VRF->Global Memory bus available
    if (computeUnit.vrfToGlobalMemPipeBus.rdy(Cycles(1))) {
        glbMemBusRdy = true;
    }
    // check if we can issue a Global memory instruction
    if (computeUnit.vectorGlobalMemUnit.rdy(Cycles(1))) {
        glbMemIssueRdy = true;
    }

    locMemBusRdy = false;
    locMemIssueRdy = false;
    // check if there is a VRF->LDS bus available
    if (computeUnit.vrfToLocalMemPipeBus.rdy(Cycles(1))) {
        locMemBusRdy = true;
    }
    // check if we can issue a LDS instruction
    if (computeUnit.vectorSharedMemUnit.rdy(Cycles(1))) {
        locMemIssueRdy = true;
    }
}

bool
ScheduleStage::dispatchReady(const GPUDynInstPtr &gpu_dyn_inst)
{
    assert(gpu_dyn_inst);
    Wavefront *wf = gpu_dyn_inst->wavefront();
    vectorAluRdy = false;
    scalarAluRdy = false;
    // check for available vector/scalar ALUs in the next cycle
    if (computeUnit.vectorALUs[wf->simdId].rdy(Cycles(1))) {
        vectorAluRdy = true;
    }
    if (computeUnit.scalarALUs[wf->scalarAlu].rdy(Cycles(1))) {
        scalarAluRdy = true;
    }

    if (gpu_dyn_inst->isNop()) {
        // S_NOP requires SALU. V_NOP requires VALU.
        // TODO: Scalar NOP does not require SALU in hardware,
        // and is executed out of IB directly.
        if (gpu_dyn_inst->isScalar() && !scalarAluRdy) {
            stats.dispNrdyStalls[SCH_SCALAR_ALU_NRDY]++;
            return false;
        } else if (!gpu_dyn_inst->isScalar() && !vectorAluRdy) {
            stats.dispNrdyStalls[SCH_VECTOR_ALU_NRDY]++;
            return false;
        }
    } else if (gpu_dyn_inst->isEndOfKernel()) {
        // EndPgm instruction
        if (gpu_dyn_inst->isScalar() && !scalarAluRdy) {
            stats.dispNrdyStalls[SCH_SCALAR_ALU_NRDY]++;
            return false;
        }
    } else if (gpu_dyn_inst->isBarrier() || gpu_dyn_inst->isBranch()
               || gpu_dyn_inst->isALU()) {
        // Barrier, Branch, or ALU instruction
        if (gpu_dyn_inst->isScalar() && !scalarAluRdy) {
            stats.dispNrdyStalls[SCH_SCALAR_ALU_NRDY]++;
            return false;
        } else if (!gpu_dyn_inst->isScalar() && !vectorAluRdy) {
            stats.dispNrdyStalls[SCH_VECTOR_ALU_NRDY]++;
            return false;
        }
    } else if (!gpu_dyn_inst->isScalar() && gpu_dyn_inst->isGlobalMem()) {
        // Vector Global Memory instruction
        bool rdy = true;
        if (!glbMemIssueRdy) {
            rdy = false;
            stats.dispNrdyStalls[SCH_VECTOR_MEM_ISSUE_NRDY]++;
        }
        if (!glbMemBusRdy) {
            rdy = false;
            stats.dispNrdyStalls[SCH_VECTOR_MEM_BUS_BUSY_NRDY]++;
        }
        if (!computeUnit.globalMemoryPipe.coalescerReady(gpu_dyn_inst)) {
            rdy = false;
            stats.dispNrdyStalls[SCH_VECTOR_MEM_COALESCER_NRDY]++;
        }
        if (!computeUnit.globalMemoryPipe.outstandingReqsCheck(gpu_dyn_inst)) {
            rdy = false;
            stats.dispNrdyStalls[SCH_VECTOR_MEM_REQS_NRDY]++;
        }
        if (!rdy) {
            return false;
        }
    } else if (gpu_dyn_inst->isScalar() && gpu_dyn_inst->isGlobalMem()) {
        // Scalar Global Memory instruction
        bool rdy = true;
        if (!scalarMemIssueRdy) {
            rdy = false;
            stats.dispNrdyStalls[SCH_SCALAR_MEM_ISSUE_NRDY]++;
        }
        if (!scalarMemBusRdy) {
            rdy = false;
            stats.dispNrdyStalls[SCH_SCALAR_MEM_BUS_BUSY_NRDY]++;
        }
        if (!computeUnit.scalarMemoryPipe
            .isGMReqFIFOWrRdy(wf->scalarRdGmReqsInPipe
            + wf->scalarWrGmReqsInPipe))
        {
            rdy = false;
            stats.dispNrdyStalls[SCH_SCALAR_MEM_FIFO_NRDY]++;
        }
        if (!rdy) {
            return false;
        }
    } else if (!gpu_dyn_inst->isScalar() && gpu_dyn_inst->isLocalMem()) {
        // Vector Local Memory instruction
        bool rdy = true;
        if (!locMemIssueRdy) {
            rdy = false;
            stats.dispNrdyStalls[SCH_LOCAL_MEM_ISSUE_NRDY]++;
        }
        if (!locMemBusRdy) {
            rdy = false;
            stats.dispNrdyStalls[SCH_LOCAL_MEM_BUS_BUSY_NRDY]++;
        }
        if (!computeUnit.localMemoryPipe.
                isLMReqFIFOWrRdy(wf->rdLmReqsInPipe + wf->wrLmReqsInPipe)) {
            rdy = false;
            stats.dispNrdyStalls[SCH_LOCAL_MEM_FIFO_NRDY]++;
        }
        if (!rdy) {
            return false;
        }
    } else if (!gpu_dyn_inst->isScalar() && gpu_dyn_inst->isFlat()) {
        // Vector Flat memory instruction
        bool rdy = true;
        if (!glbMemIssueRdy || !locMemIssueRdy) {
            rdy = false;
            stats.dispNrdyStalls[SCH_FLAT_MEM_ISSUE_NRDY]++;
        }
        if (!glbMemBusRdy || !locMemBusRdy) {
            rdy = false;
            stats.dispNrdyStalls[SCH_FLAT_MEM_BUS_BUSY_NRDY]++;
        }
        if (!computeUnit.globalMemoryPipe.coalescerReady(gpu_dyn_inst)) {
            rdy = false;
            stats.dispNrdyStalls[SCH_FLAT_MEM_COALESCER_NRDY]++;
        }
        if (!computeUnit.globalMemoryPipe.outstandingReqsCheck(gpu_dyn_inst)) {
            rdy = false;
            stats.dispNrdyStalls[SCH_FLAT_MEM_REQS_NRDY]++;
        }
        if (!computeUnit.localMemoryPipe.
                isLMReqFIFOWrRdy(wf->rdLmReqsInPipe + wf->wrLmReqsInPipe)) {
            rdy = false;
            stats.dispNrdyStalls[SCH_FLAT_MEM_FIFO_NRDY]++;
        }
        if (!rdy) {
            return false;
        }
    } else {
        panic("%s: unknown instr checked for readiness",
              gpu_dyn_inst->disassemble());
        return false;
    }
    stats.dispNrdyStalls[SCH_RDY]++;
    return true;
}

void
ScheduleStage::fillDispatchList()
{
    // update execution resource status
    checkMemResources();
    // iterate execution resources
    for (int j = 0; j < computeUnit.numExeUnits(); j++) {
        assert(toExecute.dispatchStatus(j) == EMPTY);

        // iterate waves in schList to pick one for dispatch
        auto schIter = schList.at(j).begin();
        bool dispatched = false;
        while (schIter != schList.at(j).end()) {
            // only attempt to dispatch if status is RFREADY
            if (schIter->second == RFREADY) {
                // Check if this wave is ready for dispatch
                bool dispRdy = dispatchReady(schIter->first);
                if (!dispatched && dispRdy) {
                    // No other wave has been dispatched for this exe
                    // resource, and this wave is ready. Place this wave
                    // on dispatchList and make it ready for execution
                    // next cycle.

                    // Acquire a coalescer token if it is a global mem
                    // operation.
                    GPUDynInstPtr mp = schIter->first;
                    if (!mp->isMemSync() && !mp->isScalar() &&
                        (mp->isGlobalMem() || mp->isFlat())) {
                        computeUnit.globalMemoryPipe.acqCoalescerToken(mp);
                    }

                    // Set instruction's exec_mask if it's a mem operation
                    if (mp->isMemRef()) {
                        mp->exec_mask = mp->wavefront()->execMask();
                    }

                    doDispatchListTransition(j, EXREADY, schIter->first);
                    DPRINTF(GPUSched, "dispatchList[%d]: fillDispatchList: "
                            "EMPTY->EXREADY\n", j);
                    schIter->first = nullptr;
                    schIter = schList.at(j).erase(schIter);
                    dispatched = true;
                } else {
                    // Either another wave has been dispatched, or this wave
                    // was not ready, so it is stalled this cycle
                    schIter->first->wavefront()->stats.schStalls++;
                    if (!dispRdy) {
                        // not ready for dispatch, increment stall stat
                        schIter->first->wavefront()->stats.schResourceStalls++;
                    }
                    // Examine next wave for this resource
                    schIter++;
                }
            } else {
                // Wave not in RFREADY, try next wave
                schIter++;
            }
        }

        // Increment stall count if no wave sent to dispatchList for
        // current execution resource
        if (!dispatched) {
            stats.schListToDispListStalls[j]++;
        } else {
            stats.schListToDispList[j]++;
        }
    }
}

void
ScheduleStage::arbitrateVrfToLdsBus()
{
    // Arbitrate the VRF->GM and VRF->LDS buses for Flat memory ops
    // Note: a Flat instruction in GFx8 reserves both VRF->Glb memory bus
    // and a VRF->LDS bus. In GFx9, this is not the case.

    // iterate the GM pipelines
    for (int i = 0; i < computeUnit.numVectorGlobalMemUnits; i++) {
        // get the GM pipe index in the dispatchList
        int gm_exe_unit = computeUnit.firstMemUnit() + i;
        // get the wave in the dispatchList
        GPUDynInstPtr &gpu_dyn_inst
            = toExecute.readyInst(gm_exe_unit);
        // If the WF is valid, ready to execute, and the instruction
        // is a flat access, arbitrate with the WF's assigned LM pipe
        if (gpu_dyn_inst && toExecute.dispatchStatus(gm_exe_unit)
            == EXREADY && gpu_dyn_inst->isFlat()) {
            Wavefront *wf = gpu_dyn_inst->wavefront();
            // If the associated LM pipe also has a wave selected, block
            // that wave and let the Flat instruction issue. The WF in the
            // LM pipe is added back to the schList for consideration next
            // cycle.
            if (toExecute.dispatchStatus(wf->localMem) == EXREADY) {
                reinsertToSchList(wf->localMem, toExecute
                                  .readyInst(wf->localMem));
                // Increment stall stats for LDS-VRF arbitration
                stats.ldsBusArbStalls++;
                toExecute.readyInst(wf->localMem)
                    ->wavefront()->stats.schLdsArbStalls++;
            }
            // With arbitration of LM pipe complete, transition the
            // LM pipe to SKIP state in the dispatchList to inform EX stage
            // that a Flat instruction is executing next cycle
            doDispatchListTransition(wf->localMem, SKIP, gpu_dyn_inst);
            DPRINTF(GPUSched, "dispatchList[%d]: arbVrfLds: "
                    "EXREADY->SKIP\n", wf->localMem);
        }
    }
}

void
ScheduleStage::checkRfOperandReadComplete()
{
    // Iterate the schList queues and check if operand reads
    // have completed in the RFs. If so, mark the wave as ready for
    // selection for dispatchList
    for (int j = 0; j < computeUnit.numExeUnits(); ++j) {
        for (auto &p : schList.at(j)) {
            const GPUDynInstPtr &gpu_dyn_inst = p.first;
            assert(gpu_dyn_inst);
            Wavefront *wf = gpu_dyn_inst->wavefront();

            // Increment the number of cycles the wave spends in the
            // SCH stage, since this loop visits every wave in SCH.
            wf->stats.schCycles++;

            bool vrfRdy = true;
            if (!gpu_dyn_inst->isScalar()) {
                vrfRdy = computeUnit.vrf[wf->simdId]
                    ->operandReadComplete(wf, gpu_dyn_inst);
            }
            bool srfRdy = computeUnit.srf[wf->simdId]
                ->operandReadComplete(wf, gpu_dyn_inst);
            bool operandsReady = vrfRdy && srfRdy;
            if (operandsReady) {
                DPRINTF(GPUSched, "schList[%d]: WV[%d] operands ready for: "
                        "%d: %s\n", j, wf->wfDynId, gpu_dyn_inst->seqNum(),
                        gpu_dyn_inst->disassemble());
                DPRINTF(GPUSched, "schList[%d]: WV[%d] RFBUSY->RFREADY\n",
                        j, wf->wfDynId);
                p.second = RFREADY;
            } else {
                DPRINTF(GPUSched, "schList[%d]: WV[%d] operands not ready "
                        "for: %d: %s\n", j, wf->wfDynId,
                        gpu_dyn_inst->seqNum(), gpu_dyn_inst->disassemble());

                // operands not ready yet, increment SCH stage stats
                // aggregate to all wavefronts on the CU
                p.second = RFBUSY;

                // Increment stall stats
                wf->stats.schStalls++;
                wf->stats.schOpdNrdyStalls++;

                stats.opdNrdyStalls[SCH_RF_OPD_NRDY]++;
                if (!vrfRdy) {
                    stats.opdNrdyStalls[SCH_VRF_OPD_NRDY]++;
                }
                if (!srfRdy) {
                    stats.opdNrdyStalls[SCH_SRF_OPD_NRDY]++;
                }
            }
        }
    }
}

void
ScheduleStage::reserveResources()
{
    std::vector<bool> exeUnitReservations;
    exeUnitReservations.resize(computeUnit.numExeUnits(), false);

    for (int j = 0; j < computeUnit.numExeUnits(); ++j) {
        GPUDynInstPtr &gpu_dyn_inst = toExecute.readyInst(j);
        if (gpu_dyn_inst) {
            DISPATCH_STATUS s = toExecute.dispatchStatus(j);
            Wavefront *wf = gpu_dyn_inst->wavefront();
            if (s == EMPTY) {
                continue;
            } else if (s == EXREADY) {
                // Wave is ready for execution
                std::vector<int> execUnitIds = wf->reserveResources();

                if (!gpu_dyn_inst->isScalar()) {
                    computeUnit.vrf[wf->simdId]
                        ->dispatchInstruction(gpu_dyn_inst);
                }
                computeUnit.srf[wf->simdId]->dispatchInstruction(gpu_dyn_inst);

                std::stringstream ss;
                for (auto id : execUnitIds) {
                    ss << id << " ";
                }
                DPRINTF(GPUSched, "dispatchList[%d]: SIMD[%d] WV[%d]: %d: %s"
                        "    Reserving ExeRes[ %s]\n",
                        j, wf->simdId, wf->wfDynId, gpu_dyn_inst->seqNum(),
                        gpu_dyn_inst->disassemble(), ss.str());
                // mark the resources as reserved for this cycle
                for (auto execUnitId : execUnitIds) {
                    panic_if(exeUnitReservations.at(execUnitId),
                             "Execution unit %d is reserved!!!\n"
                             "SIMD[%d] WV[%d]: %d: %s",
                             execUnitId, wf->simdId, wf->wfDynId,
                             gpu_dyn_inst->seqNum(),
                             gpu_dyn_inst->disassemble());
                    exeUnitReservations.at(execUnitId) = true;
                }

                // If wavefront::reserveResources reserved multiple resources,
                // then we're executing a flat memory instruction. This means
                // that we've reserved a global and local memory unit. Thus,
                // we need to mark the latter execution unit as not available.
                if (execUnitIds.size() > 1) {
                    [[maybe_unused]] int lm_exec_unit = wf->localMem;
                    assert(toExecute.dispatchStatus(lm_exec_unit)
                           == SKIP);
                }
            } else if (s == SKIP) {
                // Shared Memory pipe reserved for FLAT instruction.
                // Verify the GM pipe for this wave is ready to execute
                // and the wave in the GM pipe is the same as the wave
                // in the LM pipe
                [[maybe_unused]] int gm_exec_unit = wf->globalMem;
                assert(wf->wfDynId == toExecute
                       .readyInst(gm_exec_unit)->wfDynId);
                assert(toExecute.dispatchStatus(gm_exec_unit)
                       == EXREADY);
            }
        }
    }
}

void
ScheduleStage::deleteFromSch(Wavefront *w)
{
    wavesInSch.erase(w->wfDynId);
}

ScheduleStage::ScheduleStageStats::ScheduleStageStats(
    statistics::Group *parent, int num_exec_units)
    : statistics::Group(parent, "ScheduleStage"),
      ADD_STAT(rdyListEmpty ,"number of cycles no wave on ready list per "
               "execution resource"),
      ADD_STAT(rdyListNotEmpty, "number of cycles one or more wave on ready "
               "list per execution resource"),
      ADD_STAT(addToSchListStalls, "number of cycles a wave is not added to "
               "schList per execution resource when ready list is not empty"),
      ADD_STAT(schListToDispList, "number of cycles a wave is added to "
               "dispatchList per execution resource"),
      ADD_STAT(schListToDispListStalls, "number of cycles no wave is added to"
               " dispatchList per execution resource"),
      ADD_STAT(rfAccessStalls, "number of stalls due to RF access denied"),
      ADD_STAT(ldsBusArbStalls, "number of stalls due to VRF->LDS bus "
               "conflicts"),
      ADD_STAT(opdNrdyStalls, "number of stalls in SCH due to operands not "
               "ready"),
      ADD_STAT(dispNrdyStalls, "number of stalls in SCH due to resource not "
               "ready")
{
    rdyListNotEmpty.init(num_exec_units);
    rdyListEmpty.init(num_exec_units);
    addToSchListStalls.init(num_exec_units);
    schListToDispList.init(num_exec_units);
    schListToDispListStalls.init(num_exec_units);
    opdNrdyStalls.init(SCH_RF_OPD_NRDY_CONDITIONS);
    dispNrdyStalls.init(SCH_NRDY_CONDITIONS);
    rfAccessStalls.init(SCH_RF_ACCESS_NRDY_CONDITIONS);

    opdNrdyStalls.subname(SCH_VRF_OPD_NRDY, csprintf("VRF"));
    opdNrdyStalls.subname(SCH_SRF_OPD_NRDY, csprintf("SRF"));
    opdNrdyStalls.subname(SCH_RF_OPD_NRDY, csprintf("RF"));

    dispNrdyStalls.subname(SCH_SCALAR_ALU_NRDY, csprintf("ScalarAlu"));
    dispNrdyStalls.subname(SCH_VECTOR_ALU_NRDY, csprintf("VectorAlu"));
    dispNrdyStalls.subname(SCH_VECTOR_MEM_ISSUE_NRDY,
                                  csprintf("VectorMemIssue"));
    dispNrdyStalls.subname(SCH_VECTOR_MEM_BUS_BUSY_NRDY,
                                  csprintf("VectorMemBusBusy"));
    dispNrdyStalls.subname(SCH_VECTOR_MEM_COALESCER_NRDY,
                                  csprintf("VectorMemCoalescer"));
    dispNrdyStalls.subname(SCH_CEDE_SIMD_NRDY, csprintf("CedeSimd"));
    dispNrdyStalls.subname(SCH_SCALAR_MEM_ISSUE_NRDY,
                                  csprintf("ScalarMemIssue"));
    dispNrdyStalls.subname(SCH_SCALAR_MEM_BUS_BUSY_NRDY,
                                  csprintf("ScalarMemBusBusy"));
    dispNrdyStalls.subname(SCH_SCALAR_MEM_FIFO_NRDY,
                                  csprintf("ScalarMemFIFO"));
    dispNrdyStalls.subname(SCH_LOCAL_MEM_ISSUE_NRDY,
                                  csprintf("LocalMemIssue"));
    dispNrdyStalls.subname(SCH_LOCAL_MEM_BUS_BUSY_NRDY,
                                  csprintf("LocalMemBusBusy"));
    dispNrdyStalls.subname(SCH_LOCAL_MEM_FIFO_NRDY,
                                  csprintf("LocalMemFIFO"));
    dispNrdyStalls.subname(SCH_FLAT_MEM_ISSUE_NRDY,
                                  csprintf("FlatMemIssue"));
    dispNrdyStalls.subname(SCH_FLAT_MEM_BUS_BUSY_NRDY,
                                  csprintf("FlatMemBusBusy"));
    dispNrdyStalls.subname(SCH_FLAT_MEM_COALESCER_NRDY,
                                  csprintf("FlatMemCoalescer"));
    dispNrdyStalls.subname(SCH_FLAT_MEM_FIFO_NRDY,
                                  csprintf("FlatMemFIFO"));
    dispNrdyStalls.subname(SCH_RDY, csprintf("Ready"));

    rfAccessStalls.subname(SCH_VRF_RD_ACCESS_NRDY, csprintf("VrfRd"));
    rfAccessStalls.subname(SCH_VRF_WR_ACCESS_NRDY, csprintf("VrfWr"));
    rfAccessStalls.subname(SCH_SRF_RD_ACCESS_NRDY, csprintf("SrfRd"));
    rfAccessStalls.subname(SCH_SRF_WR_ACCESS_NRDY, csprintf("SrfWr"));
    rfAccessStalls.subname(SCH_RF_ACCESS_NRDY, csprintf("Any"));
}

} // namespace gem5
