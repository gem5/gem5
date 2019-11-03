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

#ifndef __GPU_COMPUTE_SCHEDULING_POLICY_HH__
#define __GPU_COMPUTE_SCHEDULING_POLICY_HH__

#include <vector>

class Wavefront;

/**
 * Interface class for the wave scheduling policy.
 */
class SchedulingPolicy
{
  public:
    SchedulingPolicy() { }
    virtual Wavefront *chooseWave(std::vector<Wavefront*> *sched_list) = 0;
};

/**
 * Intermediate class that derives from the i-face class, and implements
 * its API. It uses the CRTP to take in the actual scheduling policy
 * implementation as a template parameter. This allows us to use a pointer
 * to SchedulingPolicy and instantiate whichever policy we want. The
 * derived policies implement the scheduler arbitration logic using
 * the static member method called __chooseWave();
 */
template<typename Policy>
class __SchedulingPolicy : public SchedulingPolicy
{
  public:
    __SchedulingPolicy() { }

    Wavefront*
    chooseWave(std::vector<Wavefront*> *sched_list) override
    {
        return Policy::__chooseWave(sched_list);
    }
};

#endif // __GPU_COMPUTE_SCHEDULING_POLICY_HH__
