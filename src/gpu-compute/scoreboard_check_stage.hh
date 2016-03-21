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

#ifndef __SCOREBOARD_CHECK_STAGE_HH__
#define __SCOREBOARD_CHECK_STAGE_HH__

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

class ComputeUnit;
class Wavefront;

struct ComputeUnitParams;

enum WAVE_STATUS
{
    BLOCKED = 0,
    READY
};

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
    ScoreboardCheckStage(const ComputeUnitParams* params);
    ~ScoreboardCheckStage();
    void init(ComputeUnit *cu);
    void exec();

    // Stats related variables and methods
    const std::string& name() const { return _name; }
    void regStats();

  private:
    void collectStatistics(Wavefront *curWave, int unitId);
    void initStatistics();
    ComputeUnit *computeUnit;
    uint32_t numSIMDs;
    uint32_t numMemUnits;
    uint32_t numShrMemPipes;

    // flag per vector SIMD unit that is set when there is at least one
    // WF that has a vector ALU instruction as the oldest in its
    // Instruction Buffer
    std::vector<bool> *vectorAluInstAvail;
    int lastGlbMemSimd;
    int lastShrMemSimd;

    int *glbMemInstAvail;
    int *shrMemInstAvail;
    // List of waves which are ready to be scheduled.
    // Each execution resource has a ready list
    std::vector<std::vector<Wavefront*>*> readyList;

    // Stores the status of waves. A READY implies the
    // wave is ready to be scheduled this cycle and
    // is already present in the readyList
    std::vector<std::vector<std::pair<Wavefront*, WAVE_STATUS>>*>
        waveStatusList;

    std::string _name;
};

#endif // __SCOREBOARD_CHECK_STAGE_HH__
