/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <sys/types.h>
#include <signal.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "sim/debug.hh"
#include "sim/eventq.hh"
#include "sim/param.hh"
#include "sim/sim_events.hh"

using namespace std;

void
debug_break()
{
    kill(getpid(), SIGTRAP);
}

//
// Debug event: place a breakpoint on the process function and
// schedule the event to break at a particular cycle
//
class DebugBreakEvent : public Event
{
  public:

    DebugBreakEvent(EventQueue *q, Tick _when);

    void process();	// process event
    virtual const char *description();
};

//
// constructor: schedule at specified time
//
DebugBreakEvent::DebugBreakEvent(EventQueue *q, Tick _when)
    : Event(q)
{
    setFlags(AutoDelete);
    schedule(_when, -20000);
}

//
// handle debug event: set debugger breakpoint on this function
//
void
DebugBreakEvent::process()
{
    debug_break();
}


const char *
DebugBreakEvent::description()
{
    return "debug break";
}

//
// Parameter context for global debug options
//
class DebugContext : public ParamContext
{
  public:
    DebugContext(const string &_iniSection)
        : ParamContext(_iniSection) {}
    void checkParams();
};

DebugContext debugParams("debug");

VectorParam<Tick> break_cycles(&debugParams, "break_cycles",
                                 "cycle(s) to create breakpoint events");

void
DebugContext::checkParams()
{
    if (break_cycles.isValid()) {
        vector<Tick> &cycles = break_cycles;

        vector<Tick>::iterator i = cycles.begin();
        vector<Tick>::iterator end = cycles.end();

        for (; i < end; ++i)
            new DebugBreakEvent(&mainEventQueue, *i);
    }
}

//
// handy function to schedule DebugBreakEvent on main event queue
// (callable from debugger)
//
extern "C" void sched_break_cycle(Tick when)
{
    new DebugBreakEvent(&mainEventQueue, when);
}

extern "C" void eventq_dump()
{
    mainEventQueue.dump();
}

