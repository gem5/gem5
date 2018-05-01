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
 */

#ifndef __GPU_COMPUTE_RR_SCHEDULING_POLICY_HH__
#define __GPU_COMPUTE_RR_SCHEDULING_POLICY_HH__

#include <vector>

#include "base/logging.hh"
#include "gpu-compute/scheduling_policy.hh"
#include "gpu-compute/wavefront.hh"

// round-robin pick among the list of ready waves
class RRSchedulingPolicy final : public __SchedulingPolicy<RRSchedulingPolicy>
{
  public:
    RRSchedulingPolicy()
    {
    }

    static Wavefront*
    __chooseWave(std::vector<Wavefront*> *sched_list)
    {
        panic_if(!sched_list->size(), "RR scheduling policy sched list is "
            "empty.\n");
        Wavefront *selected_wave(nullptr);

        /**
         * For RR policy, select the wave that is at the front of
         * the list. The selected wave is popped out from the schedule
         * list immediately after selection to avoid starvation. It
         * is the responsibility of the module invoking the RR scheduler
         * to make sure it is scheduling eligible waves are added to the
         * back of the schedule list.
         */
        selected_wave = sched_list->front();
        panic_if(!selected_wave, "No wave found by RR scheduling policy.\n");
        sched_list->erase(sched_list->begin());

        return selected_wave;
    }
};

#endif // __GPU_COMPUTE_RR_SCHEDULING_POLICY_HH__
