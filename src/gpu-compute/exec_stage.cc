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
 *
 * Authors: John Kalamatianos,
 *          Sooraj Puthoor
 */

#include "gpu-compute/exec_stage.hh"

#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/wavefront.hh"

ExecStage::ExecStage(const ComputeUnitParams *p) : numSIMDs(p->num_SIMDs),
    numMemUnits(p->num_global_mem_pipes + p->num_shared_mem_pipes),
    vectorAluInstAvail(nullptr), glbMemInstAvail(nullptr),
    shrMemInstAvail(nullptr), lastTimeInstExecuted(false),
    thisTimeInstExecuted(false), instrExecuted (false),
    executionResourcesUsed(0)
{
    numTransActiveIdle = 0;
    idle_dur = 0;
}

void
ExecStage::init(ComputeUnit *cu)
{
    computeUnit = cu;
    _name = computeUnit->name() + ".ExecStage";
    dispatchList = &computeUnit->dispatchList;
    vectorAluInstAvail = &(computeUnit->vectorAluInstAvail);
    glbMemInstAvail= &(computeUnit->glbMemInstAvail);
    shrMemInstAvail= &(computeUnit->shrMemInstAvail);
    idle_dur = 0;
}

void
ExecStage::collectStatistics(enum STAT_STATUS stage, int unitId) {
    if (stage == IdleExec) {
        // count cycles of no vector ALU instruction executed
        // even if one was the oldest in a WV of that vector SIMD unit
        if (computeUnit->isVecAlu(unitId) && vectorAluInstAvail->at(unitId)) {
            numCyclesWithNoInstrTypeIssued[unitId]++;
        }

        // count cycles of no global memory (vector) instruction executed
        // even if one was the oldest in a WV of that vector SIMD unit
        if (computeUnit->isGlbMem(unitId) && *glbMemInstAvail > 0) {
            numCyclesWithNoInstrTypeIssued[unitId]++;
            (*glbMemInstAvail)--;
        }

        // count cycles of no shared memory (vector) instruction executed
        // even if one was the oldest in a WV of that vector SIMD unit
        if (computeUnit->isShrMem(unitId) && *shrMemInstAvail > 0) {
            numCyclesWithNoInstrTypeIssued[unitId]++;
            (*shrMemInstAvail)--;
        }
    } else if (stage == BusyExec) {
        // count the number of cycles an instruction to a specific unit
        // was issued
        numCyclesWithInstrTypeIssued[unitId]++;
        thisTimeInstExecuted = true;
        instrExecuted = true;
        ++executionResourcesUsed;
    } else if (stage == PostExec) {
        // count the number of transitions from active to idle
        if (lastTimeInstExecuted && !thisTimeInstExecuted) {
            ++numTransActiveIdle;
        }

        if (!lastTimeInstExecuted && thisTimeInstExecuted) {
            idleDur.sample(idle_dur);
            idle_dur = 0;
        } else if (!thisTimeInstExecuted) {
            idle_dur++;
        }

        lastTimeInstExecuted = thisTimeInstExecuted;
        // track the number of cycles we either issued one vector instruction
        // or issued no instructions at all
        if (instrExecuted) {
            numCyclesWithInstrIssued++;
        } else {
            numCyclesWithNoIssue++;
        }

        spc.sample(executionResourcesUsed);
    }
}

void
ExecStage::initStatistics()
{
    instrExecuted = false;
    executionResourcesUsed = 0;
    thisTimeInstExecuted = false;
}

void
ExecStage::exec()
{
    initStatistics();

    for (int unitId = 0; unitId < (numSIMDs + numMemUnits); ++unitId) {
         // if dispatch list for this execution resource is empty,
         // skip this execution resource this cycle
         if (dispatchList->at(unitId).second == EMPTY) {
             collectStatistics(IdleExec, unitId);
             continue;
         }

         collectStatistics(BusyExec, unitId);
         // execute an instruction for the WF
         dispatchList->at(unitId).first->exec();
         // clear the dispatch list entry
         dispatchList->at(unitId).second = EMPTY;
         dispatchList->at(unitId).first = (Wavefront*)nullptr;
    }

    collectStatistics(PostExec, 0);
}

void
ExecStage::regStats()
{
    numTransActiveIdle
       .name(name() + ".num_transitions_active_to_idle")
       .desc("number of CU transitions from active to idle")
        ;

    numCyclesWithNoIssue
        .name(name() + ".num_cycles_with_no_issue")
        .desc("number of cycles the CU issues nothing")
        ;

    numCyclesWithInstrIssued
        .name(name() + ".num_cycles_with_instr_issued")
        .desc("number of cycles the CU issued at least one instruction")
        ;

    spc
        .init(0, numSIMDs + numMemUnits, 1)
        .name(name() + ".spc")
        .desc("Execution units active per cycle (Exec unit=SIMD,MemPipe)")
        ;

    idleDur
        .init(0,75,5)
        .name(name() + ".idle_duration_in_cycles")
        .desc("duration of idle periods in cycles")
        ;

    numCyclesWithInstrTypeIssued
        .init(numSIMDs + numMemUnits)
        .name(name() + ".num_cycles_with_instrtype_issue")
        .desc("Number of cycles at least one instruction of specific type "
              "issued")
        ;

    numCyclesWithNoInstrTypeIssued
        .init(numSIMDs + numMemUnits)
       .name(name() + ".num_cycles_with_instr_type_no_issue")
       .desc("Number of cycles no instruction of specific type issued")
       ;

    for (int i = 0; i < numSIMDs; ++i) {
        numCyclesWithInstrTypeIssued.subname(i, csprintf("ALU%d",i));
        numCyclesWithNoInstrTypeIssued.subname(i, csprintf("ALU%d",i));
    }

    numCyclesWithInstrTypeIssued.subname(numSIMDs, csprintf("GM"));
    numCyclesWithNoInstrTypeIssued.subname(numSIMDs, csprintf("GM"));
    numCyclesWithInstrTypeIssued.subname(numSIMDs + 1, csprintf("LM"));
    numCyclesWithNoInstrTypeIssued.subname(numSIMDs + 1, csprintf("LM"));
}
