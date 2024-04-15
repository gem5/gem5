/*
 * Copyright (c) 2022 The Regents of the University of California.
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

#ifndef __CPU_PROBES_PC_COUNT_TRACKER_MANAGER_HH__
#define __CPU_PROBES_PC_COUNT_TRACKER_MANAGER_HH__

#include <unordered_map>
#include <unordered_set>

#include "cpu/base.hh"
#include "params/PcCountTrackerManager.hh"
#include "sim/sim_exit.hh"
#include "debug/PcCountTracker.hh"

namespace gem5
{

class PcCountTrackerManager : public SimObject
{
  public:
    PcCountTrackerManager(const PcCountTrackerManagerParams &params);

    /** this function is called when PcCountTrackerProbeListener finds a target
     * PC
     */
    void checkCount(Addr pc);

  private:
    /** a counter that stores all the target PC addresses and the number
     * of times the target PC has been executed
     */
    std::unordered_map<Addr, int> counter;

    /** a set that stores all the PC Count pairs that should raise an
     * exit event at
     */
    std::unordered_set<PcCountPair, PcCountPair::HashFunction> targetPair;

    /** the current PC Count pair */
    PcCountPair currentPair;

    /** when all the PC Count pairs in the `targetPair` are encountered,
     * and the PCCOUNTTRACK_END exit event is raised, this boolean
     * variable becomes false and is used to stop the `check_count`
     * from functioning. This is default as true.
     */
    bool ifListNotEmpty;

  public:
    /** this function returns the corresponding value of count for the
     * inputted Program Counter address. If the PC address does not
     * exist in the counter, then it returns a -1.
     *
     * @param pc the targeting Program Counter address
     * @return the corresponding value of count for the inputted Program
     * Counter address
     */
    int
    getPcCount(Addr pc) const
    {
        if (counter.find(pc) != counter.end()) {
            return counter.find(pc)->second;
        }
        return -1;
    }

    /** this function returns the current PC Count pair
     *
     * @return current PC Count pair
     */
    PcCountPair
    getCurrentPcCountPair() const
    {
        return currentPair;
    }

    /** this function print all targets
     *
     * @return formatted string that contains all targets
     */
    std::string
    printAllTargets() const
    {
        std::string s;
        for (auto itr = targetPair.begin(); itr != targetPair.end(); ++itr) {
            s += itr->to_string();
            s += "\n";
        }
        return s;
    }
};

} // namespace gem5

#endif // __CPU_PROBES_PC_COUNT_TRACKER_MANAGER_HH__
