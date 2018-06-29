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

#include "gpu-compute/exec_stage.hh"

#include <sstream>

#include "base/trace.hh"
#include "debug/GPUSched.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

ExecStage::ExecStage(const ComputeUnitParams *p, ComputeUnit *cu)
    : computeUnit(cu), lastTimeInstExecuted(false),
      thisTimeInstExecuted(false), instrExecuted (false),
      executionResourcesUsed(0), _name(cu->name() + ".ExecStage")

{
    numTransActiveIdle = 0;
    idle_dur = 0;
}

void
ExecStage::init()
{
    dispatchList = &computeUnit->dispatchList;
    idle_dur = 0;
}

void
ExecStage::collectStatistics(enum STAT_STATUS stage, int unitId) {
    if (stage == IdleExec) {
        // count cycles when no instruction to a specific execution resource
        // is executed
        numCyclesWithNoInstrTypeIssued[unitId]++;
    } else if (stage == BusyExec) {
        // count the number of cycles an instruction to a specific execution
        // resource type was issued
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
        // track the number of cycles we either issued at least
        // instruction or issued no instructions at all
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

std::string
ExecStage::dispStatusToStr(int i)
{
    std::string s("INVALID");
    switch (i) {
    case EMPTY:
        s = "EMPTY";
        break;
    case SKIP:
        s = "SKIP";
        break;
    case EXREADY:
        s = "EXREADY";
        break;
    }
    return s;
}

void
ExecStage::dumpDispList()
{
    std::stringstream ss;
    bool empty = true;
    for (int i = 0; i < computeUnit->numExeUnits(); i++) {
        DISPATCH_STATUS s = dispatchList->at(i).second;
        ss << i << ": " << dispStatusToStr(s);
        if (s != EMPTY) {
            empty = false;
            Wavefront *w = dispatchList->at(i).first;
            ss << " SIMD[" << w->simdId << "] WV[" << w->wfDynId << "]: ";
            ss << (w->instructionBuffer.front())->seqNum() << ": ";
            ss << (w->instructionBuffer.front())->disassemble();
        }
        ss << "\n";
    }
    if (!empty) {
        DPRINTF(GPUSched, "Dispatch List:\n%s", ss.str());
    }
}

void
ExecStage::exec()
{
    initStatistics();
    if (Debug::GPUSched) {
        dumpDispList();
    }
    for (int unitId = 0; unitId < computeUnit->numExeUnits(); ++unitId) {
        DISPATCH_STATUS s = dispatchList->at(unitId).second;
        switch (s) {
        case EMPTY:
            // Do not execute if empty, waiting for VRF reads,
            // or LM tied to GM waiting for VRF reads
            collectStatistics(IdleExec, unitId);
            break;
        case EXREADY:
        {
            collectStatistics(BusyExec, unitId);
            Wavefront *w = dispatchList->at(unitId).first;
            DPRINTF(GPUSched, "Exec[%d]: SIMD[%d] WV[%d]: %s\n",
                    unitId, w->simdId, w->wfDynId,
                    (w->instructionBuffer.front())->disassemble());
            DPRINTF(GPUSched, "dispatchList[%d] EXREADY->EMPTY\n", unitId);
            dispatchList->at(unitId).first->exec();
            (computeUnit->scheduleStage).deleteFromSch(w);
            dispatchList->at(unitId).second = EMPTY;
            dispatchList->at(unitId).first->freeResources();
            dispatchList->at(unitId).first = nullptr;
            break;
        }
        case SKIP:
            collectStatistics(BusyExec, unitId);
            DPRINTF(GPUSched, "dispatchList[%d] SKIP->EMPTY\n", unitId);
            dispatchList->at(unitId).second = EMPTY;
            dispatchList->at(unitId).first->freeResources();
            dispatchList->at(unitId).first = nullptr;
            break;
        default:
            panic("Unknown dispatch status in exec()\n");
        }
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
        .init(0, computeUnit->numExeUnits(), 1)
        .name(name() + ".spc")
        .desc("Execution units active per cycle (Exec unit=SIMD,MemPipe)")
        ;

    idleDur
        .init(0,75,5)
        .name(name() + ".idle_duration_in_cycles")
        .desc("duration of idle periods in cycles")
        ;

    numCyclesWithInstrTypeIssued
        .init(computeUnit->numExeUnits())
        .name(name() + ".num_cycles_issue_exec_rsrc")
        .desc("Number of cycles at least one instruction issued to "
              "execution resource type")
        ;

    numCyclesWithNoInstrTypeIssued
        .init(computeUnit->numExeUnits())
       .name(name() + ".num_cycles_no_issue_exec_rsrc")
       .desc("Number of clks no instructions issued to execution "
             "resource type")
       ;

    int c = 0;
    for (int i = 0; i < computeUnit->numVectorALUs; i++,c++) {
        std::string s = "VectorALU" + std::to_string(i);
        numCyclesWithNoInstrTypeIssued.subname(c, s);
        numCyclesWithInstrTypeIssued.subname(c, s);
    }
    for (int i = 0; i < computeUnit->numScalarALUs; i++,c++) {
        std::string s = "ScalarALU" + std::to_string(i);
        numCyclesWithNoInstrTypeIssued.subname(c, s);
        numCyclesWithInstrTypeIssued.subname(c, s);
    }
    numCyclesWithNoInstrTypeIssued.subname(c, "VectorMemPipe");
    numCyclesWithInstrTypeIssued.subname(c++, "VectorMemPipe");

    numCyclesWithNoInstrTypeIssued.subname(c, "SharedMemPipe");
    numCyclesWithInstrTypeIssued.subname(c++, "SharedMemPipe");

    numCyclesWithNoInstrTypeIssued.subname(c, "ScalarMemPipe");
    numCyclesWithInstrTypeIssued.subname(c++, "ScalarMemPipe");
}
