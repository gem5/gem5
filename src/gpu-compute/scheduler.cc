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

#include "gpu-compute/scheduler.hh"

Scheduler::Scheduler(const ComputeUnitParams *p)
{
    if (p->execPolicy  == "OLDEST-FIRST") {
        schedPolicy = SCHED_POLICY::OF_POLICY;
    } else if (p->execPolicy  == "ROUND-ROBIN") {
        schedPolicy = SCHED_POLICY::RR_POLICY;
    } else {
        fatal("Unimplemented scheduling policy");
    }
}

Wavefront*
Scheduler::chooseWave()
{
    if (schedPolicy == SCHED_POLICY::OF_POLICY) {
        return OFSchedPolicy.chooseWave();
    } else if (schedPolicy == SCHED_POLICY::RR_POLICY) {
        return RRSchedPolicy.chooseWave();
    } else {
        fatal("Unimplemented scheduling policy");
    }
}

void
Scheduler::bindList(std::vector<Wavefront*> *list)
{
    if (schedPolicy == SCHED_POLICY::OF_POLICY) {
        OFSchedPolicy.bindList(list);
    } else if (schedPolicy == SCHED_POLICY::RR_POLICY) {
        RRSchedPolicy.bindList(list);
    } else {
        fatal("Unimplemented scheduling policy");
    }
}
