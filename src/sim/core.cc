/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "sim/core.hh"

#include <iostream>
#include <string>

#include "base/callback.hh"
#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/output.hh"

namespace gem5
{

namespace sim_clock
{
/// The simulated frequency of curTick(). (In ticks per second)
Tick Frequency;

namespace as_float
{
double s;
double ms;
double us;
double ns;
double ps;

double Hz;
double kHz;
double MHz;
double GHz;
} // namespace as_float

namespace as_int
{
Tick s;
Tick ms;
Tick us;
Tick ns;
Tick ps;
} // namespace as_float

} // namespace sim_clock

namespace {

bool _clockFrequencyFixed = false;

// Default to 1 THz (1 Tick == 1 ps)
Tick _ticksPerSecond = 1e12;

} // anonymous namespace

void
fixClockFrequency()
{
    if (_clockFrequencyFixed)
        return;

    using namespace sim_clock;
    Frequency = _ticksPerSecond;
    as_float::s = static_cast<double>(Frequency);
    as_float::ms = as_float::s / 1.0e3;
    as_float::us = as_float::s / 1.0e6;
    as_float::ns = as_float::s / 1.0e9;
    as_float::ps = as_float::s / 1.0e12;

    as_float::Hz  = 1.0 / as_float::s;
    as_float::kHz = 1.0 / as_float::ms;
    as_float::MHz = 1.0 / as_float::us;
    as_float::GHz = 1.0 / as_float::ns;

    as_int::s  = Frequency;
    as_int::ms = as_int::s / 1000;
    as_int::us = as_int::ms / 1000;
    as_int::ns = as_int::us / 1000;
    as_int::ps = as_int::ns / 1000;

    cprintf("Global frequency set at %d ticks per second\n", _ticksPerSecond);

    _clockFrequencyFixed = true;
}
bool clockFrequencyFixed() { return _clockFrequencyFixed; }

void
setClockFrequency(Tick tps)
{
    panic_if(_clockFrequencyFixed,
            "Global frequency already fixed at %f ticks/s.", _ticksPerSecond);
    _ticksPerSecond = tps;
}
Tick getClockFrequency() { return _ticksPerSecond; }

void
setOutputDir(const std::string &dir)
{
    simout.setDirectory(dir);
}

/**
 * Queue of C++ callbacks to invoke on simulator exit.
 */
inline CallbackQueue &
exitCallbacks()
{
    static CallbackQueue theQueue;
    return theQueue;
}

/**
 * Register an exit callback.
 */
void
registerExitCallback(const std::function<void()> &callback)
{
    exitCallbacks().push_back(callback);
}

/**
 * Do C++ simulator exit processing.  Exported to Python to be invoked
 * when simulator terminates via Python's atexit mechanism.
 */
void
doExitCleanup()
{
    exitCallbacks().process();
    exitCallbacks().clear();

    std::cout.flush();
}

} // namespace gem5
