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
 *          Steve Reinhardt
 */

#include <cstring>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "base/output.hh"
#include "sim/builder.hh"
#include "sim/host.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "sim/root.hh"

using namespace std;

Tick curTick = 0;
ostream *outputStream;
ostream *configStream;

/// The simulated frequency of curTick. (This is only here for a short time)
Tick ticksPerSecond;

namespace Clock {
/// The simulated frequency of curTick. (In ticks per second)
Tick Frequency;

namespace Float {
double s;
double ms;
double us;
double ns;
double ps;

double Hz;
double kHz;
double MHz;
double GHZ;
/* namespace Float */ }

namespace Int {
Tick s;
Tick ms;
Tick us;
Tick ns;
Tick ps;
/* namespace Float */ }

/* namespace Clock */ }


// Dummy Object
class Root : public SimObject
{
  private:
    Tick max_tick;
    Tick progress_interval;

  public:
    Root(const std::string &name, Tick maxtick, Tick pi)
        : SimObject(name), max_tick(maxtick), progress_interval(pi)
    {}

    virtual void startup();
};

void
Root::startup()
{
    if (max_tick != 0)
        new SimExitEvent(curTick + max_tick, "reached maximum cycle count");

    if (progress_interval != 0)
        new ProgressEvent(&mainEventQueue, progress_interval);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Root)

    Param<Tick> clock;
    Param<Tick> max_tick;
    Param<Tick> progress_interval;
    Param<string> output_file;

END_DECLARE_SIM_OBJECT_PARAMS(Root)

BEGIN_INIT_SIM_OBJECT_PARAMS(Root)

    INIT_PARAM(clock, "tick frequency"),
    INIT_PARAM(max_tick, "maximum simulation time"),
    INIT_PARAM(progress_interval, "print a progress message"),
    INIT_PARAM(output_file, "file to dump simulator output to")

END_INIT_SIM_OBJECT_PARAMS(Root)

CREATE_SIM_OBJECT(Root)
{
    static bool created = false;
    if (created)
        panic("only one root object allowed!");

    created = true;

    outputStream = simout.find(output_file);
    Root *root = new Root(getInstanceName(), max_tick, progress_interval);

    using namespace Clock;
    Frequency = clock;
    Float::s = static_cast<double>(Frequency);
    Float::ms = Float::s / 1.0e3;
    Float::us = Float::s / 1.0e6;
    Float::ns = Float::s / 1.0e9;
    Float::ps = Float::s / 1.0e12;

    Float::Hz  = 1.0 / Float::s;
    Float::kHz = 1.0 / Float::ms;
    Float::MHz = 1.0 / Float::us;
    Float::GHZ = 1.0 / Float::ns;

    Int::s  = Frequency;
    Int::ms = Int::s / 1000;
    Int::us = Int::ms / 1000;
    Int::ns = Int::us / 1000;
    Int::ps = Int::ns / 1000;

    return root;
}

REGISTER_SIM_OBJECT("Root", Root)
