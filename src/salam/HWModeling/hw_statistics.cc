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

#include "hw_statistics.hh"

HWStatistics::HWStatistics(const HWStatisticsParams &params)
    : SimObject(params)
{

    statBufferSize = 10000;
    statBufferPreDefine = 2;
    dbg = false;

    for (int i = 0; i < statBufferPreDefine; i++) {
        std::vector<HW_Cycle_Stats> hw_cycle_buffer;
        hw_cycle_buffer.reserve(statBufferSize);
        hw_buffer_list.push_back(hw_cycle_buffer);
    }
    hw_buffer = hw_buffer_list.begin();
    cycle_buffer = hw_buffer->begin();
    clearStats();
}

void
HWStatistics::updateHWStatsCycleStart()
{
    if (dbg) {
        DPRINTF(SALAM_Debug, "Updating Cycle Statistics Buffer\n");
    }
    (*hw_buffer).insert(cycle_buffer, current_cycle_stats);
    clearStats();
    updateBuffer();
}

void
HWStatistics::clearStats()
{
    if (dbg) {
        DPRINTF(SALAM_Debug, "Clearing Cycle Statistics\n");
    }
    current_cycle_stats.reset();
}

void
HWStatistics::updateHWStatsCycleEnd(int curr_cycle)
{
    if (dbg) {
        DPRINTF(SALAM_Debug, "Updating Cycle Statistics\n");
    }
    current_cycle_stats.cycle = curr_cycle;
}

void
HWStatistics::updateBuffer()
{
    if (dbg) {
        DPRINTF(SALAM_Debug, "Checking Buffer[%i][%i]\n", current_buffer_index,
                hw_buffer_list.at(current_buffer_index).size());
    }
    if (hw_buffer_list.at(current_buffer_index).size() == statBufferSize) {
        current_buffer_index++;
        if (current_buffer_index == statBufferPreDefine) {
            if (dbg) {
                DPRINTF(SALAM_Debug, "Creating New Buffer Window\n");
            }
            std::vector<HW_Cycle_Stats> hw_cycle_buffer;
            hw_cycle_buffer.reserve(statBufferSize);
            hw_buffer_list.push_back(hw_cycle_buffer);
            hw_buffer = hw_buffer_list.end();
            cycle_buffer = hw_buffer->begin();
        } else {
            if (dbg) {
                DPRINTF(SALAM_Debug, "Next Buffer Window\n");
            }
            hw_buffer++;
            cycle_buffer = hw_buffer->begin();
        }
    } else {
        cycle_buffer = hw_buffer->end();
    }
}

void
HWStatistics::print()
{
    if (dbg) {
        DPRINTF(SALAM_Debug, " Buffers: %i\n", (current_buffer_index + 1));
    }
    for (auto buffers : hw_buffer_list) {
        for (auto cycles : buffers) {
            // This loops through the full runtime,
            // starting at cycle 1 to completion
            std::cout << " Cycle: " << cycles.cycle;
        }
    }
}
