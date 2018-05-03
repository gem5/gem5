/*
 * Copyright (c) 2014-2017 Advanced Micro Devices, Inc.
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
 *
 * Authors: Sooraj Puthoor,
 *          Anthony Gutierrez
 */

#ifndef __GPU_COMPUTE_OF_SCHEDULING_POLICY_HH__
#define __GPU_COMPUTE_OF_SCHEDULING_POLICY_HH__

#include <vector>

#include "gpu-compute/scheduling_policy.hh"
#include "gpu-compute/wavefront.hh"

// oldest first where age is marked by the wave id
class OFSchedulingPolicy final : public __SchedulingPolicy<OFSchedulingPolicy>
{
  public:
    OFSchedulingPolicy()
    {
    }

    static Wavefront*
    __chooseWave(std::vector<Wavefront*> *sched_list)
    {
        panic_if(!sched_list->size(), "OF scheduling policy sched list is "
            "empty.\n");
        // set when policy choose a wave to schedule
        Wavefront *selected_wave(nullptr);
        int selected_wave_id = -1;
        int selected_position = 0;

        for (int position = 0; position < sched_list->size(); ++position) {
            Wavefront *cur_wave = sched_list->at(position);
            int cur_wave_id = cur_wave->wfDynId;

            // Choosed wave with the lowest wave ID
            if (selected_wave_id == -1 || cur_wave_id < selected_wave_id) {
                selected_wave_id = cur_wave_id;
                selected_wave = cur_wave;
                selected_position = position;
            }
        }

        // Check to make sure ready list had at least one schedulable wave
        panic_if(!selected_wave, "No wave found by OF scheduling policy.\n");
        sched_list->erase(sched_list->begin() + selected_position);

        return selected_wave;
    }
};

#endif // __GPU_COMPUTE_OF_SCHEDULING_POLICY_HH__
