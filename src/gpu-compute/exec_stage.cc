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

#include "gpu-compute/exec_stage.hh"

#include <sstream>

#include "base/trace.hh"
#include "debug/GPUSched.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

ExecStage::ExecStage(const ComputeUnitParams &p, ComputeUnit &cu,
                     ScheduleToExecute &from_schedule)
    : computeUnit(cu), fromSchedule(from_schedule),
      lastTimeInstExecuted(false),
      thisTimeInstExecuted(false), instrExecuted (false),
      executionResourcesUsed(0), _name(cu.name() + ".ExecStage"),
      stats(&cu)

{
    stats.numTransActiveIdle = 0;
    idle_dur = 0;
}

void
ExecStage::init()
{
    idle_dur = 0;
}

void
ExecStage::collectStatistics(enum STAT_STATUS stage, int unitId) {
    if (stage == IdleExec) {
        // count cycles when no instruction to a specific execution resource
        // is executed
        stats.numCyclesWithNoInstrTypeIssued[unitId]++;
    } else if (stage == BusyExec) {
        // count the number of cycles an instruction to a specific execution
        // resource type was issued
        stats.numCyclesWithInstrTypeIssued[unitId]++;
        thisTimeInstExecuted = true;
        instrExecuted = true;
        ++executionResourcesUsed;
    } else if (stage == PostExec) {
        // count the number of transitions from active to idle
        if (lastTimeInstExecuted && !thisTimeInstExecuted) {
            ++stats.numTransActiveIdle;
        }

        if (!lastTimeInstExecuted && thisTimeInstExecuted) {
            stats.idleDur.sample(idle_dur);
            idle_dur = 0;
        } else if (!thisTimeInstExecuted) {
            idle_dur++;
        }

        lastTimeInstExecuted = thisTimeInstExecuted;
        // track the number of cycles we either issued at least
        // instruction or issued no instructions at all
        if (instrExecuted) {
            stats.numCyclesWithInstrIssued++;
        } else {
            stats.numCyclesWithNoIssue++;
        }
        stats.spc.sample(executionResourcesUsed);
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
    for (int i = 0; i < computeUnit.numExeUnits(); i++) {
        DISPATCH_STATUS s = fromSchedule.dispatchStatus(i);
        ss << i << ": " << dispStatusToStr(s);
        if (s != EMPTY) {
            empty = false;
            GPUDynInstPtr &gpu_dyn_inst = fromSchedule.readyInst(i);
            Wavefront *wf = gpu_dyn_inst->wavefront();
            ss << " SIMD[" << wf->simdId << "] WV[" << wf->wfDynId << "]: ";
            ss << (wf->instructionBuffer.front())->seqNum() << ": ";
            ss << (wf->instructionBuffer.front())->disassemble();
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
    if (debug::GPUSched) {
        dumpDispList();
    }
    for (int unitId = 0; unitId < computeUnit.numExeUnits(); ++unitId) {
        DISPATCH_STATUS s = fromSchedule.dispatchStatus(unitId);
        switch (s) {
          case EMPTY:
            // Do not execute if empty, waiting for VRF reads,
            // or LM tied to GM waiting for VRF reads
            collectStatistics(IdleExec, unitId);
            break;
          case EXREADY:
            {
                collectStatistics(BusyExec, unitId);
                GPUDynInstPtr &gpu_dyn_inst = fromSchedule.readyInst(unitId);
                assert(gpu_dyn_inst);
                Wavefront *wf = gpu_dyn_inst->wavefront();
                DPRINTF(GPUSched, "Exec[%d]: SIMD[%d] WV[%d]: %s\n",
                        unitId, wf->simdId, wf->wfDynId,
                        gpu_dyn_inst->disassemble());
                DPRINTF(GPUSched, "dispatchList[%d] EXREADY->EMPTY\n", unitId);
                wf->exec();
                (computeUnit.scheduleStage).deleteFromSch(wf);
                fromSchedule.dispatchTransition(unitId, EMPTY);
                wf->freeResources();
                break;
            }
          case SKIP:
            {
                collectStatistics(BusyExec, unitId);
                GPUDynInstPtr &gpu_dyn_inst = fromSchedule.readyInst(unitId);
                assert(gpu_dyn_inst);
                Wavefront *wf = gpu_dyn_inst->wavefront();
                DPRINTF(GPUSched, "dispatchList[%d] SKIP->EMPTY\n", unitId);
                fromSchedule.dispatchTransition(unitId, EMPTY);
                wf->freeResources();
                break;
            }
          default:
            panic("Unknown dispatch status in exec()\n");
        }
    }

    collectStatistics(PostExec, 0);
}

ExecStage::ExecStageStats::ExecStageStats(statistics::Group *parent)
    : statistics::Group(parent, "ExecStage"),
      ADD_STAT(numTransActiveIdle,
               "number of CU transitions from active to idle"),
      ADD_STAT(numCyclesWithNoIssue, "number of cycles the CU issues nothing"),
      ADD_STAT(numCyclesWithInstrIssued,
               "number of cycles the CU issued at least one instruction"),
      ADD_STAT(spc,
               "Execution units active per cycle (Exec unit=SIMD,MemPipe)"),
      ADD_STAT(idleDur, "duration of idle periods in cycles"),
      ADD_STAT(numCyclesWithInstrTypeIssued, "Number of cycles at least one "
               "instruction issued to execution resource type"),
      ADD_STAT(numCyclesWithNoInstrTypeIssued, "Number of clks no instructions"
               " issued to execution resource type")
{
    ComputeUnit *compute_unit = static_cast<ComputeUnit*>(parent);

    spc.init(0, compute_unit->numExeUnits(), 1);
    idleDur.init(0, 75, 5);
    numCyclesWithInstrTypeIssued.init(compute_unit->numExeUnits());
    numCyclesWithNoInstrTypeIssued.init(compute_unit->numExeUnits());

    int c = 0;
    for (int i = 0; i < compute_unit->numVectorALUs; i++,c++) {
        std::string s = "VectorALU" + std::to_string(i);
        numCyclesWithNoInstrTypeIssued.subname(c, s);
        numCyclesWithInstrTypeIssued.subname(c, s);
    }
    for (int i = 0; i < compute_unit->numScalarALUs; i++,c++) {
        std::string s = "ScalarALU" + std::to_string(i);
        numCyclesWithNoInstrTypeIssued.subname(c, s);
        numCyclesWithInstrTypeIssued.subname(c, s);
    }
    numCyclesWithNoInstrTypeIssued.subname(c, "VectorMemPipe");
    numCyclesWithInstrTypeIssued.subname(c++, "VectorMemPipe");

    numCyclesWithNoInstrTypeIssued.subname(c, "SharedMemPipe");
    numCyclesWithInstrTypeIssued.subname(c++, "SharedMemPipe");
}

} // namespace gem5
