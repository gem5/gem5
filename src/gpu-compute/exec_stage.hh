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

#ifndef __EXEC_STAGE_HH__
#define __EXEC_STAGE_HH__

#include <string>
#include <utility>
#include <vector>

#include "sim/stats.hh"

class ComputeUnit;
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
    EMPTY = 0,
    FILLED
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
    ExecStage(const ComputeUnitParams* params);
    ~ExecStage() { }
    void init(ComputeUnit *cu);
    void exec();

    std::string name() { return _name; }
    void regStats();
    // number of idle cycles
    Stats::Scalar numCyclesWithNoIssue;
    // number of busy cycles
    Stats::Scalar numCyclesWithInstrIssued;
    // number of cycles (per execution unit) during which at least one
    // instruction was issued to that unit
    Stats::Vector numCyclesWithInstrTypeIssued;
    // number of idle cycles (per execution unit) during which the unit issued
    // no instruction targeting that unit, even though there is at least one
    // Wavefront with such an instruction as the oldest
    Stats::Vector numCyclesWithNoInstrTypeIssued;
    // SIMDs active per cycle
    Stats::Distribution spc;

  private:
    void collectStatistics(enum STAT_STATUS stage, int unitId);
    void initStatistics();
    ComputeUnit *computeUnit;
    uint32_t numSIMDs;

    // Number of memory execution resources;
    // both global and local memory execution resources in CU
    uint32_t numMemUnits;

    // List of waves which will be dispatched to
    // each execution resource. A FILLED implies
    // dispatch list is non-empty and
    // execution unit has something to execute
    // this cycle. Currently, the dispatch list of
    // an execution resource can hold only one wave because
    // an execution resource can execute only one wave in a cycle.
    // dispatchList is used to communicate between schedule
    // and exec stage
    std::vector<std::pair<Wavefront*, DISPATCH_STATUS>> *dispatchList;
    // flag per vector SIMD unit that is set when there is at least one
    // WV that has a vector ALU instruction as the oldest in its
    // Instruction Buffer
    std::vector<bool> *vectorAluInstAvail;
    int *glbMemInstAvail;
    int *shrMemInstAvail;
    bool lastTimeInstExecuted;
    bool thisTimeInstExecuted;
    bool instrExecuted;
    Stats::Scalar  numTransActiveIdle;
    Stats::Distribution idleDur;
    uint32_t executionResourcesUsed;
    uint64_t idle_dur;
    std::string _name;
};

#endif // __EXEC_STAGE_HH__
