/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#include <string>

#include "base/callback.hh"
#include "base/hostinfo.hh"
#include "sim/eventq.hh"
#include "sim/param.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_init.hh"
#include "sim/stats.hh"

using namespace std;

//
// handle termination event
//
void
SimExitEvent::process()
{
    // This event does not autodelete because exitNow may be called,
    // and the function will never be allowed to finish.
    if (theQueue() == &mainEventQueue) {
        string _cause = cause;
        int _code = code;
        delete this;
        exitNow(_cause, _code);
    } else {
        new SimExitEvent(cause, code);
        delete this;
    }
}


const char *
SimExitEvent::description()
{
    return "simulation termination";
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
        new SimExitEvent(cause, 0);
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
        new SimExitEvent("Lack of swap space");
    }

    schedule(curTick + interval);
}

const char *
CheckSwapEvent::description()
{
    return "check swap";
}


///////////////////////////////////////////////////
//
// Simulation termination parameters
//
///////////////////////////////////////////////////

class TermParamContext : public ParamContext
{
  public:
    TermParamContext(const string &_iniSection)
        : ParamContext(_iniSection) {}
    void checkParams();
};

TermParamContext simTerminationParams("max");

Param<Tick> max_cycle(&simTerminationParams, "cycle",
                        "maximum number of cycles to execute");

void
TermParamContext::checkParams()
{
    // if a max cycle count was specified, put a termination event on
    // the event queue at that point
    if (max_cycle.isValid())
        new SimExitEvent(max_cycle, "reached maximum cycle count");
}

//
// Progress event: print out cycle every so often so we know we're
// making forward progress.
//
class ProgressEvent : public Event
{
  protected:
    Tick interval;

  public:
    ProgressEvent(EventQueue *q, Tick interval);

    void process();	// process event
    virtual const char *description();
};

//
// constructor: schedule at specified time
//
ProgressEvent::ProgressEvent(EventQueue *q, Tick _interval)
    : Event(q), interval(_interval)
{
    schedule(curTick + interval);
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

/////////
//
// Periodic progress message support: print out a message every n
// cycles so we know we're making forward progress.
//
/////////

// Parameter space for execution address tracing options.  Derive
// from ParamContext so we can override checkParams() function.
class ProgressParamContext : public ParamContext
{
  public:
    ProgressParamContext(const string &_iniSection)
        : ParamContext(_iniSection) {}
    void checkParams();
};

ProgressParamContext progessMessageParams("progress");

Param<Tick> progress_interval(&progessMessageParams, "cycle",
                                "cycle interval for progress messages");

namespace {
    struct SetupProgress : public Callback
    {
        Tick interval;
        SetupProgress(Tick tick) : interval(tick) {}

        virtual void process()
        {
            new ProgressEvent(&mainEventQueue, interval);
            delete this;
        }
    };
}

/* check execute options */
void
ProgressParamContext::checkParams()
{
    if (progress_interval.isValid())
        registerInitCallback(new SetupProgress(progress_interval));
}
