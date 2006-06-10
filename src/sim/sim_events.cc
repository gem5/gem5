/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 */

#include <string>

#include "base/callback.hh"
#include "base/hostinfo.hh"
#include "sim/eventq.hh"
#include "sim/param.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/startup.hh"
#include "sim/stats.hh"

using namespace std;

//
// handle termination event
//
void
SimLoopExitEvent::process()
{
    // if this got scheduled on a different queue (e.g. the committed
    // instruction queue) then make a corresponding event on the main
    // queue.
    if (theQueue() != &mainEventQueue) {
        exitSimLoop(cause, code);
        delete this;
    }

    // otherwise do nothing... the IsExitEvent flag takes care of
    // exiting the simulation loop and returning this object to Python
}


const char *
SimLoopExitEvent::description()
{
    return "simulation loop exit";
}

void
exitSimLoop(Tick when, const std::string &message, int exit_code)
{
    new SimLoopExitEvent(when, message, exit_code);
}

void
exitSimLoop(const std::string &message, int exit_code)
{
    exitSimLoop(curTick, message, exit_code);
}

//
// constructor: automatically schedules at specified time
//
CountedExitEvent::CountedExitEvent(EventQueue *q, const std::string &_cause,
                                   Tick _when, int &_downCounter)
    : Event(q, Sim_Exit_Pri),
      cause(_cause),
      downCounter(_downCounter)
{
    // catch stupid mistakes
    assert(downCounter > 0);

    schedule(_when);
}


//
// handle termination event
//
void
CountedExitEvent::process()
{
    if (--downCounter == 0) {
        exitSimLoop(cause, 0);
    }
}


const char *
CountedExitEvent::description()
{
    return "counted exit";
}

#ifdef CHECK_SWAP_CYCLES
new CheckSwapEvent(&mainEventQueue, CHECK_SWAP_CYCLES);
#endif

void
CheckSwapEvent::process()
{
    /*  Check the amount of free swap space  */
    long swap;

    /*  returns free swap in KBytes  */
    swap = procInfo("/proc/meminfo", "SwapFree:");

    if (swap < 1000)
        ccprintf(cerr, "\a\a\aWarning! Swap space is low (%d)\n", swap);

    if (swap < 100) {
        cerr << "\a\aAborting Simulation! Inadequate swap space!\n\n";
        exitSimLoop("Lack of swap space");
    }

    schedule(curTick + interval);
}

const char *
CheckSwapEvent::description()
{
    return "check swap";
}

//
// handle progress event: print message and reschedule
//
void
ProgressEvent::process()
{
    DPRINTFN("ProgressEvent\n");
    // reschedule for next interval
    schedule(curTick + interval);
}


const char *
ProgressEvent::description()
{
    return "progress message";
}
