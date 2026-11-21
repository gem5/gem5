/*
 * Copyright (c) 2026 - The Unviersity of Wisconsin
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

#include "sim/power/power_model_pyfunc.hh"

#include <algorithm>
#include <ostream>
#include <string>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "debug/PowerLog.hh"
#include "debug/PwrIntervalEvent.hh"
#include "sim/clocked_object.hh"
#include "sim/core.hh"

namespace gem5
{

PowerModelPyFunc::PowerModelPyFunc(const Params &p)
    : PowerModelState(p),
      // Take strong references to the PyFunc parameters, which are
      // passed in as borrowed PyObject pointers.
      st_func(pybind11::reinterpret_borrow<pybind11::object>(p.st)),
      dyn_func(pybind11::reinterpret_borrow<pybind11::object>(p.dyn)),
      on_sample_start(
          pybind11::reinterpret_borrow<pybind11::object>(p.on_sample_start)),
      power_log(pybind11::reinterpret_borrow<pybind11::object>(p.power_log)),
      power_log_file(p.power_log_file),
      pwr_interval(p.pwr_interval),
      auto_start(p.auto_start),
      intervalEvent([this] { powerAtInterval(); }, name()),
      stats(this),
      regions(this, "regions")
{}

void
PowerModelPyFunc::startup()
{
    // This queue runs after the SimObject and legacy stats are reset.
    statistics::registerResetCallback([this] {
        if (sampling || (activeRegion && pwr_interval == 0)) {
            startSampleWindow();
        }
    });
    if (auto_start && pwr_interval > 0) {
        startSampling();
    }
}

double
PowerModelPyFunc::callPy(const pybind11::function &func) const
{
    pybind11::gil_scoped_acquire acquire;
    // An equation may prepare stats even when entered from a direct getter.
    const bool was_preparing = preparing_log;
    preparing_log = true;
    try {
        const double power = func().cast<double>();
        preparing_log = was_preparing;
        return power;
    } catch (...) {
        preparing_log = was_preparing;
        throw;
    }
}

double
PowerModelPyFunc::memoized(const pybind11::function &func,
                           std::optional<Tick> &memo_tick, double &memo) const
{
    // Interval callbacks consume activity deltas and must never be reused.
    if (in_interval) {
        return callPy(func);
    }
    if (memo_tick != curTick()) {
        memo = callPy(func);
        memo_tick = curTick();
    }
    return memo;
}

double
PowerModelPyFunc::getDynamicPower() const
{
    return memoized(dyn_func, dyn_memo_tick, dyn_memo);
}

double
PowerModelPyFunc::getStaticPower() const
{
    return memoized(st_func, st_memo_tick, st_memo);
}

void
PowerModelPyFunc::clearMemo()
{
    dyn_memo_tick.reset();
    st_memo_tick.reset();
}

void
PowerModelPyFunc::resetStats()
{
    PowerModelState::resetStats();
    clearMemo();
}

void
PowerModelPyFunc::setTemperature(Temperature temp)
{
    PowerModelState::setTemperature(temp);
    clearMemo();
}

bool
PowerModelPyFunc::powerLoggingEnabled() const
{
    return !power_log_file.empty() || (TRACING_ON && debug::PowerLog);
}

void
PowerModelPyFunc::logPower(double dyn, double st, double temp_k) const
{
    if (!powerLoggingEnabled()) {
        return;
    }
    pybind11::gil_scoped_acquire acquire;
    const auto label = clocked_object ? clocked_object->name() : name();
    const bool custom_format = !power_log.is_none();
    std::string record;
    if (!custom_format) {
        record = csprintf("%s: dyn %.17g W st %.17g W temp %.2f K", label, dyn,
                          st, temp_k);
    } else {
        pybind11::object text = power_log(curTick(), label, dyn, st, temp_k);
        if (text.is_none()) {
            return;
        }
        if (!pybind11::isinstance<pybind11::str>(text)) {
            throw pybind11::type_error(
                "power_log callback must return str or None");
        }
        record = text.cast<std::string>();
    }

    if (power_log_file.empty()) {
        DPRINTF(PowerLog, "%s\n", record);
        return;
    }
    auto &output = *simout.findOrCreate(power_log_file)->stream();
    if (!custom_format) {
        output << curTick() << ": ";
    }
    output << record << std::endl;
    fatal_if(output.fail(), "%s: failed writing power log '%s'", name(),
             power_log_file);
}

void
PowerModelPyFunc::preDumpStats()
{
    PowerModelState::preDumpStats();
    // Root.preDumpStats() also prepares stats inside Python power equations.
    if (!powerLoggingEnabled() || pwr_interval > 0 || preparing_log ||
        activeRegion || last_log_tick == curTick()) {
        return;
    }
    preparing_log = true;
    try {
        const double dyn = getDynamicPower();
        const double st = getStaticPower();
        logPower(dyn, st, _temp.toKelvin());
        last_log_tick = curTick();
    } catch (...) {
        preparing_log = false;
        throw;
    }
    preparing_log = false;
}

void
PowerModelPyFunc::addRegion(const std::string &region)
{
    constexpr char allowed[] =
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-";
    if (region.empty() ||
        region.find_first_not_of(allowed) != std::string::npos) {
        throw pybind11::value_error(
            "Power region names must use letters, digits, '_' or '-'");
    }
    if (regionStats.find(region) != regionStats.end()) {
        return;
    }

    auto group =
        std::make_unique<PowerModelPyFuncStats>(&regions, region.c_str());
    // A region can be registered after the initial stats enable traversal.
    for (auto *info : group->getStats()) {
        fatal_if(!info->check() || !info->baseCheck(),
                 "%s: invalid power statistics for region '%s'", name(),
                 region);
        info->enable();
    }
    regionStats.emplace(region, std::move(group));
}

void
PowerModelPyFunc::beginRegion(const std::string &region)
{
    if (activeRegion) {
        throw pybind11::value_error("A power region is already active");
    }
    const auto it = regionStats.find(region);
    if (it == regionStats.end()) {
        throw pybind11::value_error("Unregistered power region: " + region);
    }

    stopSampling();
    if (pwr_interval > 0) {
        startSampling();
    } else {
        startSampleWindow();
    }
    activeRegion = it->second.get();
}

void
PowerModelPyFunc::endRegion()
{
    if (!activeRegion) {
        return;
    }
    if (pwr_interval > 0) {
        stopSampling();
    } else {
        sample();
    }
    activeRegion = nullptr;
}

void
PowerModelPyFunc::startSampling()
{
    fatal_if(pwr_interval == 0,
             "%s: startSampling() requires pwr_interval > 0.\n", name());
    if (sampling) {
        return;
    }
    startSampleWindow();
    sampling = true;
    DPRINTF(PwrIntervalEvent, "Starting sampling of %s\n",
            clocked_object->name());
}

void
PowerModelPyFunc::startSampleWindow()
{
    if (!on_sample_start.is_none()) {
        pybind11::gil_scoped_acquire acquire;
        on_sample_start();
    }
    last_sample_tick = curTick();
    if (pwr_interval > 0) {
        reschedule(intervalEvent,
                   last_sample_tick +
                       clocked_object->cyclesToTicks(pwr_interval),
                   true);
    }
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
    if (sampling) {
        stats.sample(dyn, st);
    }
    if (activeRegion) {
        activeRegion->sample(dyn, st);
    }
    logPower(dyn, st, _temp.toKelvin());
    last_log_tick = curTick();
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
    statistics::Group *parent, const char *name)
    : statistics::Group(parent, name),
      retainAcrossReset(name != nullptr),
      ADD_STAT(powerDist, statistics::units::Watt::get(),
               "Sampled total power"),
      ADD_STAT(dynamicPowerDist, statistics::units::Watt::get(),
               "Sampled dynamic power"),
      ADD_STAT(staticPowerDist, statistics::units::Watt::get(),
               "Sampled static power"),
      ADD_STAT(powerMin, statistics::units::Watt::get(),
               "Minimum sampled total power"),
      ADD_STAT(powerMax, statistics::units::Watt::get(),
               "Maximum sampled total power"),
      ADD_STAT(dynamicPowerMin, statistics::units::Watt::get(),
               "Minimum sampled dynamic power"),
      ADD_STAT(dynamicPowerMax, statistics::units::Watt::get(),
               "Maximum sampled dynamic power"),
      ADD_STAT(staticPowerMin, statistics::units::Watt::get(),
               "Minimum sampled static power"),
      ADD_STAT(staticPowerMax, statistics::units::Watt::get(),
               "Maximum sampled static power")
{
    constexpr int buckets = 20;
    powerDist.init(buckets).flags(statistics::nozero);
    dynamicPowerDist.init(buckets).flags(statistics::nozero);
    staticPowerDist.init(buckets).flags(statistics::nozero);
    powerMin.prereq(powerDist);
    powerMax.prereq(powerDist);
    dynamicPowerMin.prereq(dynamicPowerDist);
    dynamicPowerMax.prereq(dynamicPowerDist);
    staticPowerMin.prereq(staticPowerDist);
    staticPowerMax.prereq(staticPowerDist);
}

void
PowerModelPyFunc::PowerModelPyFuncStats::resetStats()
{
    // Guest stats resets must preserve previously recorded kernel samples.
    if (!retainAcrossReset) {
        statistics::Group::resetStats();
    }
}

void
PowerModelPyFunc::PowerModelPyFuncStats::sample(double dyn, double st)
{
    const auto record = [](double power, statistics::Histogram &dist,
                           statistics::Scalar &min, statistics::Scalar &max) {
        if (dist.zero()) {
            min = power;
            max = power;
        } else {
            min = std::min(min.value(), power);
            max = std::max(max.value(), power);
        }
        dist.sample(power);
    };
    record(dyn + st, powerDist, powerMin, powerMax);
    record(dyn, dynamicPowerDist, dynamicPowerMin, dynamicPowerMax);
    record(st, staticPowerDist, staticPowerMin, staticPowerMax);
}

} // namespace gem5
