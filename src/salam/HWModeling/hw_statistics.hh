/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef __HWMODEL_HW_STATISTICS_HH__
#define __HWMODEL_HW_STATISTICS_HH__

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "params/HWStatistics.hh"
#include "salam/LLVMRead/debug_flags.hh"
#include "sim/sim_object.hh"

using namespace gem5;

// Things here are output only once at end of simulation
struct HW_Params
{
    int run_end;

    void
    reset()
    {
        run_end = 0;
    }
};

// These are outputs that are stored each cycle
struct HW_Cycle_Stats
{
    int cycle;

    int resInFlight;

    int loadInFlight;
    int loadInternal;
    int loadAcitve;
    int loadRawStall;

    int storeInFlight;
    int storeActive;

    int compInFlight;
    int compLaunched;
    int compActive;
    int compFUStall;
    int compCommited;

    void
    reset()
    {
        cycle = 0;
        resInFlight = 0;
        loadInFlight = 0;
        storeInFlight = 0;
        compInFlight = 0;
    }
};

class HWStatistics : public SimObject
{
  private:
    HW_Params hw_params;
    HW_Cycle_Stats current_cycle_stats;
    std::vector<HW_Cycle_Stats>::iterator cycle_buffer;
    std::vector<std::vector<HW_Cycle_Stats>> hw_buffer_list;
    std::vector<std::vector<HW_Cycle_Stats>>::iterator hw_buffer;

    // Make Into SimObjects to pass from config.yml
    bool cycle_tracking = false;
    bool dbg;
    int statBufferSize;
    int statBufferPreDefine;

    // Class Only
    int current_buffer_index = 0;

  public:
    HWStatistics();
    HWStatistics(const HWStatisticsParams &params);
    bool
    use_cycle_tracking()
    {
        return cycle_tracking;
    }

    void print();
    void updateHWStatsCycleStart();
    void updateHWStatsCycleEnd(int curr_cycle);
    void updateBuffer();
    void clearStats();
};

#endif //__HWMODEL_HW_STATISTICS_HH__
