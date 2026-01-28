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

#ifndef __CPU_PROBES_INST_TRACKER_HH__
#define __CPU_PROBES_INST_TRACKER_HH__

#include <unordered_set>

#include "debug/InstTracker.hh"
#include "params/GlobalInstTracker.hh"
#include "params/LocalInstTracker.hh"
#include "sim/probe/probe_listener_object.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

class LocalInstTracker : public ProbeListenerObject
{
  public:
    LocalInstTracker(const LocalInstTrackerParams &params);

    /** setup the probelistener */
    virtual void regProbeListeners();

    /**
     * this function is called when the ProbePoint "RetiredInsts" is notified
     *
     * @param inst the number of retired instructions. It is usually 1.
     */
    void retiredInstsHandler(const uint64_t& inst);

  private:
    typedef ProbeListenerArg<LocalInstTracker, uint64_t>
        LocalInstTrackerListener;

    /** a boolean variable that determines if the LocalInstTracker is
     * listening to the ProbePoints or not
     */
    bool ifListening;

    /**
     * the pointer to the GlobalInstTracker object. It is used to update the
      * instruction count and check if the instruction count has reached the
      * threshold across all the cores.
     */
    GlobalInstTracker *globalInstTracker;

  public:
    /** stop listening to the ProbePoints */
    void stopListening();

    /** start listening to the ProbePoints */
    void
    startListening()
    {
        ifListening = true;
        regProbeListeners();
    }

};

class GlobalInstTracker : public SimObject
{
  public:
    GlobalInstTracker(const GlobalInstTrackerParams &params);

    /**
     * this function is called by the LocalInstTracker object to update the
      * instruction count and check if the instruction count has reached the
      * threshold across all the cores.
     */
    void updateAndCheckInstCount(const uint64_t& inst);

  private:
    /** the number of instructions that have been executed across all the cores
    */
    uint64_t instCount;

    /**
      * a set of thresholds for the number of instructions that should be
      * executed before the simulation exits
     */
    std::unordered_set<uint64_t> instThresholdSet;

  public:
    void
    addThreshold(uint64_t new_threshold)
    {
        instThresholdSet.insert(new_threshold);
        DPRINTF(InstTracker, "adding the instruction threshold %lu\n",
                                                              new_threshold);
    };

    uint64_t
    getCounter() const
    {
        return instCount;
    };

    void
    resetCounter()
    {
        instCount = 0;
        DPRINTF(InstTracker, "Resetting the instruction counter\n"
                                              "instCount = %lu\n", instCount);
    };

    std::unordered_set<uint64_t>
    getThresholds() const
    {
        return instThresholdSet;
    };

    void
    resetThresholds()
    {
        instThresholdSet.clear();
        DPRINTF(InstTracker, "Resetting the instruction thresholds\n");
    };
};

}

#endif // __CPU_PROBES_INST_TRACKER_HH__
