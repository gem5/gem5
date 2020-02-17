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

#ifndef __CPU_ACTIVITY_HH__
#define __CPU_ACTIVITY_HH__

#include "base/trace.hh"
#include "cpu/timebuf.hh"

/**
 * ActivityRecorder helper class that informs the CPU if it can switch
 * over to being idle or not.  It works by having a time buffer as
 * long as any time buffer in the CPU, and the CPU and all of its
 * stages inform the ActivityRecorder when they write to any time
 * buffer.  The ActivityRecorder marks a 1 in the "0" slot of the time
 * buffer any time a stage writes to a time buffer, and it advances
 * its time buffer at the same time as all other stages.  The
 * ActivityRecorder also records if a stage has activity to do next
 * cycle.  The recorder keeps a count of these two.  Thus any time the
 * count is non-zero, there is either communication still in flight,
 * or activity that still must be done, meaning that the CPU can not
 * idle.  If count is zero, then the CPU can safely idle as it has no
 * more outstanding work to do.
 */
class ActivityRecorder
{
  public:
    ActivityRecorder(const std::string &name, int num_stages,
                     int longest_latency, int count);
    ~ActivityRecorder();

    /** Records that there is activity this cycle. */
    void activity();

    /** Advances the activity buffer, decrementing the activityCount
     *  if active communication just left the time buffer, and
     *  determining if there is no activity.
     */
    void advance();

    /** Marks a stage as active. */
    void activateStage(const int idx);

    /** Deactivates a stage. */
    void deactivateStage(const int idx);

    /** Returns the activity status of a stage. */
    bool getStageActive(const int idx) const { return stageActive[idx]; }

    /** Returns the number of stages. */
    int getNumStages() const { return numStages; }

    /** Returns how many things are active within the recorder. */
    int getActivityCount() const { return activityCount; }

    /** Sets the count to a starting value.  Can be used to disable
     * the idling option.
     */
    void setActivityCount(int count)
    { activityCount = count; }

    /** Returns if the CPU should be active. */
    bool active() { return activityCount; }

    /** Clears the time buffer and the activity count. */
    void reset();

    /** Debug function to dump the contents of the time buffer. */
    void dump();

    /** Debug function to ensure that the activity count matches the
     * contents of the time buffer.
     */
    void validate();

    const std::string &name() const { return _name; }

  private:
    // provide name() for DPRINTF.
    std::string _name;

    /** Time buffer that tracks if any cycles has active communication
     *  in them.  It should be as long as the longest communication
     *  latency in the system.  Each time any time buffer is written,
     *  the activity buffer should also be written to. The
     *  activityBuffer is advanced along with all the other time
     *  buffers, so it should have a 1 somewhere in it only if there
     *  is active communication in a time buffer.
     */
    TimeBuffer<bool> activityBuffer;

    /** Longest latency time buffer in the CPU. */
    int longestLatency;

    /** Tracks how many stages and cycles of time buffer have
     *  activity. Stages increment this count when they switch to
     *  active, and decrement it when they switch to
     *  inactive. Whenever a cycle that previously had no information
     *  is written in the time buffer, this is incremented. When a
     *  cycle that had information exits the time buffer due to age,
     *  this count is decremented. When the count is 0, there is no
     *  activity in the CPU, and it can be descheduled.
     */
    int activityCount;

    /** Number of stages that can be marked as active or inactive. */
    int numStages;

    /** Records which stages are active/inactive. */
    bool *stageActive;
};

#endif // __CPU_ACTIVITY_HH__
