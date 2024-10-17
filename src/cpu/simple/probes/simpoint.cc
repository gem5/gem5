/*
 * Copyright (c) 2012-2014 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "cpu/simple/probes/simpoint.hh"

#include "base/output.hh"

namespace gem5
{

SimPoint::SimPoint(const SimPointParams &p)
    : ProbeListenerObject(p),
      intervalSize(p.interval),
      intervalCount(0),
      intervalDrift(0),
      simpointStream(NULL),
      currentBBV(0, 0),
      currentBBVInstCount(0)
{
    simpointStream = simout.create(p.profile_file, false);
    if (!simpointStream)
        fatal("unable to open SimPoint profile_file");
}

SimPoint::~SimPoint()
{
    simout.close(simpointStream);
}

void
SimPoint::init()
{}

void
SimPoint::regProbeListeners()
{
    typedef ProbeListenerArg<SimPoint, std::pair<SimpleThread*,StaticInstPtr>>
        SimPointListener;
    connectListener<SimPointListener>(this, "Commit", &SimPoint::profile);
}

void
SimPoint::profile(const std::pair<SimpleThread*, StaticInstPtr>& p)
{
    SimpleThread* thread = p.first;
    const StaticInstPtr &inst = p.second;

    if (inst->isMicroop() && !inst->isLastMicroop())
        return;

    if (!currentBBVInstCount)
        currentBBV.first = thread->pcState().instAddr();

    ++intervalCount;
    ++currentBBVInstCount;

    // If inst is control inst, assume end of basic block.
    if (inst->isControl()) {
        currentBBV.second = thread->pcState().instAddr();

        auto map_itr = bbMap.find(currentBBV);
        if (map_itr == bbMap.end()){
            // If a new (previously unseen) basic block is found,
            // add a new unique id, record num of insts and insert
            // into bbMap.
            BBInfo info;
            info.id = bbMap.size() + 1;
            info.insts = currentBBVInstCount;
            info.count = currentBBVInstCount;
            bbMap.insert(std::make_pair(currentBBV, info));
        } else {
            // If basic block is seen before, just increment the count by the
            // number of insts in basic block.
            BBInfo& info = map_itr->second;
            info.count += currentBBVInstCount;
        }
        currentBBVInstCount = 0;

        // Reached end of interval if the sum of the current inst count
        // (intervalCount) and the excessive inst count from the previous
        // interval (intervalDrift) is greater than/equal to the interval size.
        if (intervalCount + intervalDrift >= intervalSize) {
            // summarize interval and display BBV info
            std::vector<std::pair<uint64_t, uint64_t> > counts;
            for (auto map_itr = bbMap.begin(); map_itr != bbMap.end();
                    ++map_itr) {
                BBInfo& info = map_itr->second;
                if (info.count != 0) {
                    counts.push_back(std::make_pair(info.id, info.count));
                    info.count = 0;
                }
            }
            std::sort(counts.begin(), counts.end());

            // Print output BBV info
            *simpointStream->stream() << "T";
            for (auto cnt_itr = counts.begin(); cnt_itr != counts.end();
                    ++cnt_itr) {
                *simpointStream->stream() << ":" << cnt_itr->first
                                << ":" << cnt_itr->second << " ";
            }
            *simpointStream->stream() << "\n";

            intervalDrift = (intervalCount + intervalDrift) - intervalSize;
            intervalCount = 0;
        }
    }
}

} // namespace gem5
