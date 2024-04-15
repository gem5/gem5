/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
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

// This file will contain default statistics for the simulator that
// don't really belong to a specific simulator object

#include "sim/stat_control.hh"

#include <fstream>
#include <iostream>
#include <list>

#include "base/callback.hh"
#include "base/statistics.hh"
#include "base/time.hh"
#include "sim/global_event.hh"

namespace gem5
{

namespace statistics
{

GlobalEvent *dumpEvent;

void
initSimStats()
{}

/**
 * Event to dump and/or reset the statistics.
 */
class StatEvent : public GlobalEvent
{
  private:
    bool dump;
    bool reset;
    Tick repeat;

  public:
    StatEvent(Tick _when, bool _dump, bool _reset, Tick _repeat)
        : GlobalEvent(_when, Stat_Event_Pri, 0),
          dump(_dump),
          reset(_reset),
          repeat(_repeat)
    {}

    virtual void
    process()
    {
        if (dump)
            statistics::dump();

        if (reset)
            statistics::reset();

        if (repeat) {
            statistics::schedStatEvent(dump, reset, curTick() + repeat,
                                       repeat);
        }
    }

    const char *
    description() const
    {
        return "GlobalStatEvent";
    }
};

void
schedStatEvent(bool dump, bool reset, Tick when, Tick repeat)
{
    // simQuantum is being added to the time when the stats would be
    // dumped so as to ensure that this event happens only after the next
    // sync amongst the event queues.  Asingle event queue simulation
    // should remain unaffected.
    dumpEvent = new StatEvent(when + simQuantum, dump, reset, repeat);
}

void
periodicStatDump(Tick period)
{
    /*
     * If the period is set to 0, then we do not want to dump periodically,
     * thus we deschedule the event. Else, if the period is not 0, but the
     * event has already been scheduled, we need to get rid of the old event
     * before we create a new one, as the old event will no longer be moved
     * forward in the event that we resume from a checkpoint.
     */
    if (dumpEvent != NULL && (period == 0 || dumpEvent->scheduled())) {
        // Event should AutoDelete, so we do not need to free it.
        dumpEvent->deschedule();
    }

    /*
     * If the period is not 0, we schedule the event. If this is called with a
     * period that is less than the current tick, then we shift the first dump
     * by curTick. This ensures that we do not schedule the event is the past.
     */
    if (period != 0) {
        // Schedule the event
        if (period >= curTick()) {
            schedStatEvent(true, true, (Tick)period, (Tick)period);
        } else {
            schedStatEvent(true, true, (Tick)period + curTick(), (Tick)period);
        }
    }
}

void
updateEvents()
{
    /*
     * If the dumpEvent has been scheduled, but is scheduled in the past, then
     * we need to shift the event to be at a valid point in time. Therefore, we
     * shift the event by curTick.
     */
    if (dumpEvent != NULL &&
        (dumpEvent->scheduled() && dumpEvent->when() < curTick())) {
        // shift by curTick() and reschedule
        Tick _when = dumpEvent->when();
        dumpEvent->reschedule(_when + curTick());
    }
}

} // namespace statistics
} // namespace gem5
