/*
 * Copyright (c) 2024 The Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/probes/inst_tracker.hh"

namespace gem5
{

LocalInstTracker::LocalInstTracker(const LocalInstTrackerParams &params)
    : ProbeListenerObject(params),
      ifListening(params.start_listening),
      globalInstTracker(params.global_inst_tracker)
{
    DPRINTF(InstTracker, "ifListening = %s\n", ifListening ? "true" : "false");
}

void
LocalInstTracker::regProbeListeners()
{
    if (ifListening) {
        if (listeners.empty())
        {
            listeners.push_back(new LocalInstTrackerListener(this,
                                    "RetiredInsts",
                                    &LocalInstTracker::retiredInstsHandler));
            DPRINTF(InstTracker, "Start listening to RetiredInsts\n");
        }
    }
}

void
LocalInstTracker::retiredInstsHandler(const uint64_t& inst)
{
    globalInstTracker->updateAndCheckInstCount(inst);
}

void
LocalInstTracker::stopListening()
{
    ifListening = false;
    for (auto l = listeners.begin(); l != listeners.end(); ++l) {
        delete (*l);
    }
    listeners.clear();
    DPRINTF(InstTracker, "Stopped listening\n");
}


GlobalInstTracker::GlobalInstTracker(const GlobalInstTrackerParams &params)
    : SimObject(params),
      instCount(0),
      instThreshold(params.inst_threshold)
{
    DPRINTF(InstTracker, "instThreshold = %lu\n", instThreshold);
}

void
GlobalInstTracker::updateAndCheckInstCount(const uint64_t& inst)
{
    instCount ++;
    if (instCount >= instThreshold) {
        DPRINTF(InstTracker, "Instruction count reached the threshold\n"
                                "instCount = %lu\n"
                                "instThreshold = %lu\n",
                                instCount, instThreshold);

        // note that when the threshold is reached, the simulation will raise
        // and exit event but it will not reset the instruction counter.
        // user can reset the counter by calling the resetCounter() function
        // in the simulation script.
        exitSimLoopNow("a thread reached the max instruction count");
    }
}

} // namespace gem5
