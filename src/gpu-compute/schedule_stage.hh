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

#ifndef __SCHEDULE_STAGE_HH__
#define __SCHEDULE_STAGE_HH__

#include <utility>
#include <vector>

#include "gpu-compute/exec_stage.hh"
#include "gpu-compute/scheduler.hh"
#include "gpu-compute/scoreboard_check_stage.hh"

// Schedule or execution arbitration stage.
// From the pool of ready waves in the ready list,
// one wave is selected for each execution resource.
// The selection is made based on a scheduling policy

class ComputeUnit;
class Wavefront;

struct ComputeUnitParams;

class ScheduleStage
{
  public:
    ScheduleStage(const ComputeUnitParams *params);
    ~ScheduleStage();
    void init(ComputeUnit *cu);
    void exec();
    void arbitrate();
    // Stats related variables and methods
    std::string name() { return _name; }
    void regStats();

  private:
    ComputeUnit *computeUnit;
    uint32_t numSIMDs;
    uint32_t numMemUnits;

    // Each execution resource will have its own
    // scheduler and a dispatch list
    std::vector<Scheduler> scheduler;

    // Stores the status of waves. A READY implies the
    // wave is ready to be scheduled this cycle and
    // is already present in the readyList
    std::vector<std::vector<std::pair<Wavefront*, WAVE_STATUS>>*>
        waveStatusList;

    // List of waves which will be dispatched to
    // each execution resource. A FILLED implies
    // dispatch list is non-empty and
    // execution unit has something to execute
    // this cycle. Currently, the dispatch list of
    // an execution resource can hold only one wave because
    // an execution resource can execute only one wave in a cycle.
    std::vector<std::pair<Wavefront*, DISPATCH_STATUS>> *dispatchList;

    std::string _name;
};

#endif // __SCHEDULE_STAGE_HH__
