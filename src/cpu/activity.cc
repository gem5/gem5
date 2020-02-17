/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "cpu/activity.hh"

#include <string>

#include "cpu/timebuf.hh"
#include "debug/Activity.hh"

using namespace std;

ActivityRecorder::ActivityRecorder(const string &name, int num_stages,
    int longest_latency, int activity)
    : _name(name), activityBuffer(longest_latency, 0),
      longestLatency(longest_latency), activityCount(activity),
      numStages(num_stages)
{
    stageActive = new bool[numStages];
    std::memset(stageActive, 0, numStages);
}

ActivityRecorder::~ActivityRecorder()
{
    delete[] stageActive;
}

void
ActivityRecorder::activity()
{
    // If we've already recorded activity for this cycle, we don't
    // want to increment the count any more.
    if (activityBuffer[0]) {
        return;
    }

    activityBuffer[0] = true;

    ++activityCount;

    DPRINTF(Activity, "Activity: %i\n", activityCount);
}

void
ActivityRecorder::advance()
{
    // If there's a 1 in the slot that is about to be erased once the
    // time buffer advances, then decrement the activityCount.
    if (activityBuffer[-longestLatency]) {
        --activityCount;

        assert(activityCount >= 0);

        DPRINTF(Activity, "Activity: %i\n", activityCount);

        if (activityCount == 0) {
            DPRINTF(Activity, "No activity left!\n");
        }
    }

    activityBuffer.advance();
}

void
ActivityRecorder::activateStage(const int idx)
{
    // Increment the activity count if this stage wasn't already active.
    if (!stageActive[idx]) {
        ++activityCount;

        stageActive[idx] = true;

        DPRINTF(Activity, "Activity: %i\n", activityCount);
    } else {
        DPRINTF(Activity, "Stage %i already active.\n", idx);
    }

//    assert(activityCount < longestLatency + numStages + 1);
}

void
ActivityRecorder::deactivateStage(const int idx)
{
    // Decrement the activity count if this stage was active.
    if (stageActive[idx]) {
        --activityCount;

        stageActive[idx] = false;

        DPRINTF(Activity, "Activity: %i\n", activityCount);
    } else {
        DPRINTF(Activity, "Stage %i already inactive.\n", idx);
    }

    assert(activityCount >= 0);
}

void
ActivityRecorder::reset()
{
    activityCount = 0;
    std::memset(stageActive, 0, numStages);
    for (int i = 0; i < longestLatency + 1; ++i)
        activityBuffer.advance();
}

void
ActivityRecorder::dump()
{
    for (int i = 0; i <= longestLatency; ++i) {
        cprintf("[Idx:%i %i] ", i, activityBuffer[-i]);
    }

    cprintf("\n");

    for (int i = 0; i < numStages; ++i) {
        cprintf("[Stage:%i %i]\n", i, stageActive[i]);
    }

    cprintf("\n");

    cprintf("Activity count: %i\n", activityCount);
}

void
ActivityRecorder::validate()
{
    int count = 0;
    for (int i = 0; i <= longestLatency; ++i) {
        if (activityBuffer[-i]) {
            count++;
        }
    }

    for (int i = 0; i < numStages; ++i) {
        if (stageActive[i]) {
            count++;
        }
    }

    assert(count == activityCount);
}
