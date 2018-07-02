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

#ifndef __SCHEDULE_STAGE_HH__
#define __SCHEDULE_STAGE_HH__

#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gpu-compute/exec_stage.hh"
#include "gpu-compute/misc.hh"
#include "gpu-compute/scheduler.hh"

// Schedule or execution arbitration stage.
// From the pool of ready waves in the ready list,
// one wave is selected for each execution resource.
// The selection is made based on a scheduling policy

class ComputeUnit;
class ScheduleToExecute;
class ScoreboardCheckToSchedule;
class Wavefront;

struct ComputeUnitParams;

class ScheduleStage
{
  public:
    ScheduleStage(const ComputeUnitParams *p, ComputeUnit &cu,
                  ScoreboardCheckToSchedule &from_scoreboard_check,
                  ScheduleToExecute &to_execute);
    ~ScheduleStage();
    void init();
    void exec();

    // Stats related variables and methods
    const std::string& name() const { return _name; }
    enum SchNonRdyType {
        SCH_SCALAR_ALU_NRDY,
        SCH_VECTOR_ALU_NRDY,
        SCH_VECTOR_MEM_ISSUE_NRDY,
        SCH_VECTOR_MEM_BUS_BUSY_NRDY,
        SCH_VECTOR_MEM_COALESCER_NRDY,
        SCH_VECTOR_MEM_REQS_NRDY,
        SCH_CEDE_SIMD_NRDY,
        SCH_SCALAR_MEM_ISSUE_NRDY,
        SCH_SCALAR_MEM_BUS_BUSY_NRDY,
        SCH_SCALAR_MEM_FIFO_NRDY,
        SCH_LOCAL_MEM_ISSUE_NRDY,
        SCH_LOCAL_MEM_BUS_BUSY_NRDY,
        SCH_LOCAL_MEM_FIFO_NRDY,
        SCH_FLAT_MEM_ISSUE_NRDY,
        SCH_FLAT_MEM_BUS_BUSY_NRDY,
        SCH_FLAT_MEM_COALESCER_NRDY,
        SCH_FLAT_MEM_REQS_NRDY,
        SCH_FLAT_MEM_FIFO_NRDY,
        SCH_RDY,
        SCH_NRDY_CONDITIONS
    };
    enum schopdnonrdytype_e {
        SCH_VRF_OPD_NRDY,
        SCH_SRF_OPD_NRDY,
        SCH_RF_OPD_NRDY,
        SCH_RF_OPD_NRDY_CONDITIONS
    };
    enum schrfaccessnonrdytype_e {
        SCH_VRF_RD_ACCESS_NRDY,
        SCH_VRF_WR_ACCESS_NRDY,
        SCH_SRF_RD_ACCESS_NRDY,
        SCH_SRF_WR_ACCESS_NRDY,
        SCH_RF_ACCESS_NRDY,
        SCH_RF_ACCESS_NRDY_CONDITIONS
    };

    void regStats();

    // Called by ExecStage to inform SCH of instruction execution
    void deleteFromSch(Wavefront *w);

    // Schedule List status
    enum SCH_STATUS
    {
        RFBUSY = 0, // RF busy reading operands
        RFREADY, // ready for exec
    };

  private:
    ComputeUnit &computeUnit;
    ScoreboardCheckToSchedule &fromScoreboardCheck;
    ScheduleToExecute &toExecute;

    // Each execution resource will have its own
    // scheduler and a dispatch list
    std::vector<Scheduler> scheduler;

    // Stats

    // Number of cycles with empty (or not empty) readyList, per execution
    // resource, when the CU is active (not sleeping)
    Stats::Vector rdyListEmpty;
    Stats::Vector rdyListNotEmpty;

    // Number of cycles, per execution resource, when at least one wave
    // was on the readyList and picked by scheduler, but was unable to be
    // added to the schList, when the CU is active (not sleeping)
    Stats::Vector addToSchListStalls;

    // Number of cycles, per execution resource, when a wave is selected
    // as candidate for dispatchList from schList
    // Note: may be arbitrated off dispatchList (e.g., LDS arbitration)
    Stats::Vector schListToDispList;

    // Per execution resource stat, incremented once per cycle if no wave
    // was selected as candidate for dispatch and moved to dispatchList
    Stats::Vector schListToDispListStalls;

    // Number of times a wave is selected by the scheduler but cannot
    // be added to the schList due to register files not being able to
    // support reads or writes of operands. RF_ACCESS_NRDY condition is always
    // incremented if at least one read/write not supported, other
    // conditions are incremented independently from each other.
    Stats::Vector rfAccessStalls;

    // Number of times a wave is executing FLAT instruction and
    // forces another wave occupying its required local memory resource
    // to be deselected for execution, and placed back on schList
    Stats::Scalar ldsBusArbStalls;

    // Count of times VRF and/or SRF blocks waves on schList from
    // performing RFBUSY->RFREADY transition
    Stats::Vector opdNrdyStalls;

    // Count of times resource required for dispatch is not ready and
    // blocks wave in RFREADY state on schList from potentially moving
    // to dispatchList
    Stats::Vector dispNrdyStalls;

    const std::string _name;

    // called by exec() to add a wave to schList if the RFs can support it
    bool addToSchList(int exeType, const GPUDynInstPtr &gpu_dyn_inst);
    // re-insert a wave to schList if wave lost arbitration
    // wave is inserted such that age order (oldest to youngest) is preserved
    void reinsertToSchList(int exeType, const GPUDynInstPtr &gpu_dyn_inst);
    // check waves in schList to see if RF reads complete
    void checkRfOperandReadComplete();
    // check execution resources for readiness
    bool vectorAluRdy;
    bool scalarAluRdy;
    bool scalarMemBusRdy;
    bool scalarMemIssueRdy;
    bool glbMemBusRdy;
    bool glbMemIssueRdy;
    bool locMemBusRdy;
    bool locMemIssueRdy;
    // check status of memory pipes and RF to Mem buses
    void checkMemResources();
    // resource ready check called by fillDispatchList
    bool dispatchReady(const GPUDynInstPtr &gpu_dyn_inst);
    // pick waves from schList and populate dispatchList with one wave
    // per EXE resource type
    void fillDispatchList();
    // arbitrate Shared Mem Pipe VRF/LDS bus for waves in dispatchList
    void arbitrateVrfToLdsBus();
    // schedule destination operand writes to register files for waves in
    // dispatchList
    void scheduleRfDestOperands();
    // invoked by scheduleRfDestOperands to schedule RF writes for a wave
    bool schedRfWrites(int exeType, const GPUDynInstPtr &gpu_dyn_inst);
    // reserve resources for waves surviving arbitration in dispatchList
    void reserveResources();

    void doDispatchListTransition(int unitId, DISPATCH_STATUS s,
                                  const GPUDynInstPtr &gpu_dyn_inst);
    void doDispatchListTransition(int unitId, DISPATCH_STATUS s);

    // Set tracking wfDynId for each wave present in schedule stage
    // Used to allow only one instruction per wave in schedule
    std::unordered_set<uint64_t> wavesInSch;

    // List of waves (one list per exe resource) that are in schedule
    // stage. Waves are added to this list after selected by scheduler
    // from readyList. Waves are removed from this list and placed on
    // dispatchList when status reaches SCHREADY.
    // Waves are kept ordered by age for each resource, always favoring
    // forward progress for the oldest wave.
    // The maximum number of waves per resource can be determined by either
    // the VRF/SRF availability or limits imposed by paremeters (to be added)
    // of the SCH stage or CU.
    std::vector<std::deque<std::pair<GPUDynInstPtr, SCH_STATUS>>> schList;
};

#endif // __SCHEDULE_STAGE_HH__
