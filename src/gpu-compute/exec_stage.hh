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

#ifndef __EXEC_STAGE_HH__
#define __EXEC_STAGE_HH__

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "base/statistics.hh"
#include "base/stats/group.hh"

namespace gem5
{

class ComputeUnit;
class ScheduleToExecute;
class Wavefront;

struct ComputeUnitParams;

enum STAT_STATUS
{
    IdleExec,
    BusyExec,
    PostExec
};

enum DISPATCH_STATUS
{
    EMPTY = 0, // no wave present in dispatchList slot
    EXREADY, // wave ready for execution
    SKIP, // extra memory resource needed, Shared Mem. only
};

// Execution stage.
// Each execution resource executes the
// wave which is in its dispatch list.
// The schedule stage is responsible for
// adding a wave into each execution resource's
// dispatch list.

class ExecStage
{
  public:
    ExecStage(const ComputeUnitParams &p, ComputeUnit &cu,
              ScheduleToExecute &from_schedule);
    ~ExecStage() { }
    void init();
    void exec();

    std::string dispStatusToStr(int j);
    void dumpDispList();

    const std::string& name() const { return _name; }

  private:
    void collectStatistics(enum STAT_STATUS stage, int unitId);
    void initStatistics();
    ComputeUnit &computeUnit;
    ScheduleToExecute &fromSchedule;

    bool lastTimeInstExecuted;
    bool thisTimeInstExecuted;
    bool instrExecuted;
    int executionResourcesUsed;
    uint64_t idle_dur;
    const std::string _name;

  protected:
    struct ExecStageStats : public statistics::Group
    {
        ExecStageStats(statistics::Group *parent);

        // number of transitions from active to idle
        statistics::Scalar numTransActiveIdle;
        // number of idle cycles
        statistics::Scalar numCyclesWithNoIssue;
        // number of busy cycles
        statistics::Scalar numCyclesWithInstrIssued;
        // SIMDs active per cycle
        statistics::Distribution spc;
        // duration of idle periods in cycles
        statistics::Distribution idleDur;
        // number of cycles during which at least one
        // instruction was issued to an execution resource type
        statistics::Vector numCyclesWithInstrTypeIssued;
        // number of idle cycles during which the scheduler
        // issued no instructions targeting a specific
        // execution resource type
        statistics::Vector numCyclesWithNoInstrTypeIssued;
    } stats;
};

} // namespace gem5

#endif // __EXEC_STAGE_HH__
