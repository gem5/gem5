/*
 * Copyright (c) 2026 University of Wisconsin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sim/power/power_model_pyfunc.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PwrIntervalEvent.hh"
#include "sim/clocked_object.hh"
#include "sim/core.hh"

namespace gem5
{

PowerModelPyFunc::PowerModelPyFunc(const Params &p)
    : PowerModelState(p),
      // Take strong references to the PyFunc parameters, which are
      // passed in as borrowed PyObject pointers.
      st_func(pybind11::reinterpret_borrow<pybind11::function>(p.st)),
      dyn_func(pybind11::reinterpret_borrow<pybind11::function>(p.dyn)),
      pwr_interval(p.pwr_interval),
      auto_start(p.auto_start),
      intervalEvent([this] { powerAtInterval(); }, name()),
      stats(this)
{}

void
PowerModelPyFunc::startup()
{
    if (auto_start && pwr_interval > 0) {
        startSampling();
    }
}

double
PowerModelPyFunc::getDynamicPower() const
{
    return dyn_func().cast<double>();
}

double
PowerModelPyFunc::getStaticPower() const
{
    return st_func().cast<double>();
}

void
PowerModelPyFunc::startSampling()
{
    fatal_if(pwr_interval == 0,
             "%s: startSampling() requires pwr_interval > 0.\n", name());
    if (sampling) {
        return;
    }
    sampling = true;
    last_sample_tick = curTick();
    DPRINTF(PwrIntervalEvent, "Starting sampling of %s\n",
            clocked_object->name());
    schedule(intervalEvent,
             curTick() + clocked_object->cyclesToTicks(pwr_interval));
}

void
PowerModelPyFunc::stopSampling()
{
    if (!sampling) {
        return;
    }
    if (intervalEvent.scheduled()) {
        deschedule(intervalEvent);
    }
    sample();
    sampling = false;
    DPRINTF(PwrIntervalEvent, "Stopped sampling of %s\n",
            clocked_object->name());
}

void
PowerModelPyFunc::sample()
{
    sample_duration = curTick() - last_sample_tick;
    if (sample_duration == 0) {
        return;
    }
    last_sample_tick = curTick();

    in_interval = true;
    double dyn = getDynamicPower();
    double st = getStaticPower();
    in_interval = false;

    DPRINTF(PwrIntervalEvent, "Sample at %llu: dur %llu dyn %.17g st %.17g\n",
            curTick(), sample_duration, dyn, st);
    stats.powerDist.sample(dyn + st);
    stats.dynamicPowerDist.sample(dyn);
    stats.staticPowerDist.sample(st);
}

void
PowerModelPyFunc::powerAtInterval()
{
    sample();
    if (sampling) {
        schedule(intervalEvent,
                 curTick() + clocked_object->cyclesToTicks(pwr_interval));
    }
}

double
PowerModelPyFunc::getSampleDurationSeconds() const
{
    return sample_duration / sim_clock::as_float::s;
}

PowerModelPyFunc::PowerModelPyFuncStats::PowerModelPyFuncStats(
    statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(powerDist, statistics::units::Watt::get(),
               "Sampled total power"),
      ADD_STAT(dynamicPowerDist, statistics::units::Watt::get(),
               "Sampled dynamic power"),
      ADD_STAT(staticPowerDist, statistics::units::Watt::get(),
               "Sampled static power")
{
    powerDist.init(2);
    dynamicPowerDist.init(2);
    staticPowerDist.init(2);
}

} // namespace gem5
