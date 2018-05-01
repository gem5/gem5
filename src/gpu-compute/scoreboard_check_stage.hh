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

#ifndef __SCOREBOARD_CHECK_STAGE_HH__
#define __SCOREBOARD_CHECK_STAGE_HH__

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "sim/stats.hh"

class ComputeUnit;
class Wavefront;

struct ComputeUnitParams;

/*
 * Scoreboard check stage.
 * All wavefronts are analyzed to see if they are ready
 * to be executed this cycle. Both structural and data
 * hazards are considered while marking a wave "ready"
 * for execution. After analysis, the ready waves are
 * added to readyList.
 */
class ScoreboardCheckStage
{
  public:
    enum nonrdytype_e {
        NRDY_ILLEGAL,
        NRDY_WF_STOP,
        NRDY_IB_EMPTY,
        NRDY_WAIT_CNT,
        NRDY_BARRIER_WAIT,
        NRDY_VGPR_NRDY,
        NRDY_SGPR_NRDY,
        INST_RDY,
        NRDY_CONDITIONS
    };

    ScoreboardCheckStage(const ComputeUnitParams* params);
    ~ScoreboardCheckStage();
    void init(ComputeUnit *cu);
    void exec();

    // Stats related variables and methods
    const std::string& name() const { return _name; }
    void regStats();

  private:
    void collectStatistics(nonrdytype_e rdyStatus);
    int mapWaveToExeUnit(Wavefront *w);
    bool ready(Wavefront *w, nonrdytype_e *rdyStatus,
               int *exeResType, int wfSlot);
    ComputeUnit *computeUnit;

    // List of waves which are ready to be scheduled.
    // Each execution resource has a ready list
    std::vector<std::vector<Wavefront*>*> readyList;

    // Stats
    Stats::Vector stallCycles;

    std::string _name;
};

#endif // __SCOREBOARD_CHECK_STAGE_HH__
