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

#include "gpu-compute/of_scheduling_policy.hh"

#include "gpu-compute/wavefront.hh"

Wavefront*
OFSchedulingPolicy::chooseWave()
{
    // Set when policy choose a wave to schedule
    bool waveChosen = false;
    Wavefront *selectedWave = nullptr;
    int selectedWaveID = -1;
    uint32_t selectedPosition = 0;

    for (int position = 0; position < scheduleList->size(); ++position) {
        Wavefront *curWave = scheduleList->at(position);
        uint32_t curWaveID = curWave->wfDynId;

        // Choosed wave with the lowest wave ID
        if (selectedWaveID == -1 || curWaveID < selectedWaveID) {
            waveChosen = true;
            selectedWaveID = curWaveID;
            selectedWave = curWave;
            selectedPosition = position;
        }
    }

    // Check to make sure ready list had atleast one schedulable wave
    if (waveChosen) {
        scheduleList->erase(scheduleList->begin() + selectedPosition);
    } else {
        panic("Empty ready list");
    }

    return selectedWave;
}

void
OFSchedulingPolicy::bindList(std::vector<Wavefront*> *list)
{
    scheduleList = list;
}
