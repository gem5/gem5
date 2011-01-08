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
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#include "base/misc.hh"
#include "base/pollevent.hh"
#include "base/types.hh"
#include "sim/async.hh"
#include "sim/eventq.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/simulate.hh"
#include "sim/stat_control.hh"

/** Simulate for num_cycles additional cycles.  If num_cycles is -1
 * (the default), do not limit simulation; some other event must
 * terminate the loop.  Exported to Python via SWIG.
 * @return The SimLoopExitEvent that caused the loop to exit.
 */
SimLoopExitEvent *
simulate(Tick num_cycles)
{
    inform("Entering event queue @ %d.  Starting simulation...\n", curTick());

    if (num_cycles < 0)
        fatal("simulate: num_cycles must be >= 0 (was %d)\n", num_cycles);
    else if (curTick() + num_cycles < 0)  //Overflow
        num_cycles = MaxTick;
    else
        num_cycles = curTick() + num_cycles;

    Event *limit_event =
        new SimLoopExitEvent("simulate() limit reached", 0);
    mainEventQueue.schedule(limit_event, num_cycles);

    while (1) {
        // there should always be at least one event (the SimLoopExitEvent
        // we just scheduled) in the queue
        assert(!mainEventQueue.empty());
        assert(curTick() <= mainEventQueue.nextTick() &&
               "event scheduled in the past");

        // forward current cycle to the time of the first event on the
        // queue
        curTick(mainEventQueue.nextTick());
        Event *exit_event = mainEventQueue.serviceOne();
        if (exit_event != NULL) {
            // hit some kind of exit event; return to Python
            // event must be subclass of SimLoopExitEvent...
            SimLoopExitEvent *se_event;
            se_event = dynamic_cast<SimLoopExitEvent *>(exit_event);

            if (se_event == NULL)
                panic("Bogus exit event class!");

            // if we didn't hit limit_event, delete it
            if (se_event != limit_event) {
                assert(limit_event->scheduled());
                limit_event->squash();
                hack_once("be nice to actually delete the event here");
            }

            return se_event;
        }

        if (async_event) {
            async_event = false;
            if (async_statdump || async_statreset) {
                Stats::schedStatEvent(async_statdump, async_statreset);
                async_statdump = false;
                async_statreset = false;
            }

            if (async_exit) {
                async_exit = false;
                exitSimLoop("user interrupt received");
            }

            if (async_io || async_alarm) {
                async_io = false;
                async_alarm = false;
                pollQueue.service();
            }

            if (async_exception) {
                async_exception = false;
                return NULL;
            }
        }
    }

    // not reached... only exit is return on SimLoopExitEvent
}

