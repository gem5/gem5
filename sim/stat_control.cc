/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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

#include <fstream>
#include <iostream>
#include <list>

#include "base/callback.hh"
#include "base/hostinfo.hh"
#include "base/statistics.hh"
#include "base/str.hh"
#include "base/time.hh"
#include "base/stats/output.hh"
#include "cpu/base_cpu.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"
#include "sim/stat_control.hh"
#include "sim/universe.hh"

using namespace std;

Stats::Formula hostInstRate;
Stats::Formula hostTickRate;
Stats::Value hostMemory;
Stats::Value hostSeconds;

Stats::Value simTicks;
Stats::Value simInsts;
Stats::Value simFreq;
Stats::Formula simSeconds;

namespace Stats {

Time statTime(true);
Tick startTick;

class SimTicksReset : public Callback
{
  public:
    void process()
    {
        statTime.set();
        startTick = curTick;
    }
};

double
statElapsedTime()
{
    Time now(true);
    Time elapsed = now - statTime;
    return elapsed();
}

Tick
statElapsedTicks()
{
    return curTick - startTick;
}

SimTicksReset simTicksReset;

void
InitSimStats()
{
    simInsts
        .functor(BaseCPU::numSimulatedInstructions)
        .name("sim_insts")
        .desc("Number of instructions simulated")
        .precision(0)
        .prereq(simInsts)
        ;

    simSeconds
        .name("sim_seconds")
        .desc("Number of seconds simulated")
        ;

    simFreq
        .scalar(ticksPerSecond)
        .name("sim_freq")
        .desc("Frequency of simulated ticks")
        ;

    simTicks
        .functor(statElapsedTicks)
        .name("sim_ticks")
        .desc("Number of ticks simulated")
        ;

    hostInstRate
        .name("host_inst_rate")
        .desc("Simulator instruction rate (inst/s)")
        .precision(0)
        .prereq(simInsts)
        ;

    hostMemory
        .functor(memUsage)
        .name("host_mem_usage")
        .desc("Number of bytes of host memory used")
        .prereq(hostMemory)
        ;

    hostSeconds
        .functor(statElapsedTime)
        .name("host_seconds")
        .desc("Real time elapsed on the host")
        .precision(2)
        ;

    hostTickRate
        .name("host_tick_rate")
        .desc("Simulator tick rate (ticks/s)")
        .precision(0)
        ;

    simSeconds = simTicks / simFreq;
    hostInstRate = simInsts / hostSeconds;
    hostTickRate = simTicks / hostSeconds;

    registerResetCallback(&simTicksReset);
}

class StatEvent : public Event
{
  protected:
    int flags;
    Tick repeat;

  public:
    StatEvent(int _flags, Tick _when, Tick _repeat);
    virtual void process();
    virtual const char *description();
};

StatEvent::StatEvent(int _flags, Tick _when, Tick _repeat)
    : Event(&mainEventQueue, Stat_Event_Pri),
      flags(_flags), repeat(_repeat)
{
    setFlags(AutoDelete);
    schedule(_when);
}

const char *
StatEvent::description()
{
    return "Statistics dump and/or reset";
}

void
StatEvent::process()
{
    if (flags & Stats::Dump)
        DumpNow();

    if (flags & Stats::Reset)
        reset();

    if (repeat)
        schedule(curTick + repeat);
}

list<Output *> OutputList;

void
DumpNow()
{
    list<Output *>::iterator i = OutputList.begin();
    list<Output *>::iterator end = OutputList.end();
    for (; i != end; ++i) {
        Output *output = *i;
        if (!output->valid())
            continue;

        output->output();
    }
}

void
SetupEvent(int flags, Tick when, Tick repeat)
{
    new StatEvent(flags, when, repeat);
}

/* namespace Stats */ }

extern "C" void
debugDumpStats()
{
    Stats::DumpNow();
}

