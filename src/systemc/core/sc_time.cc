/*
 * Copyright 2018 Google, Inc.
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
 * Authors: Gabe Black
 */

#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "python/pybind11/pybind.hh"
#include "systemc/core/python.hh"
#include "systemc/ext/core/sc_time.hh"

namespace sc_core
{

namespace
{

const char *TimeUnitNames[] = {
    [SC_FS] = "fs",
    [SC_PS] = "ps",
    [SC_NS] = "ns",
    [SC_US] = "us",
    [SC_MS] = "ms",
    [SC_SEC] = "s"
};

double TimeUnitScale[] = {
    [SC_FS] = 1.0e-15,
    [SC_PS] = 1.0e-12,
    [SC_NS] = 1.0e-9,
    [SC_US] = 1.0e-6,
    [SC_MS] = 1.0e-3,
    [SC_SEC] = 1.0
};

bool timeFixed = false;
bool pythonReady = false;

struct SetInfo
{
    SetInfo(::sc_core::sc_time *time, double d, ::sc_core::sc_time_unit tu) :
        time(time), d(d), tu(tu)
    {}

    ::sc_core::sc_time *time;
    double d;
    ::sc_core::sc_time_unit tu;
};
std::vector<SetInfo> toSet;

void
setWork(sc_time *time, double d, ::sc_core::sc_time_unit tu)
{
    //XXX Assuming the time resolution is 1ps.
    double scale = TimeUnitScale[tu] / TimeUnitScale[SC_PS];
    // Accellera claims there is a linux bug, and that these next two
    // lines work around them.
    volatile double tmp = d * scale + 0.5;
    *time = sc_time::from_value(static_cast<uint64_t>(tmp));
}

void
fixTime()
{
    auto ticks = pybind11::module::import("m5.ticks");
    auto fix_global_frequency = ticks.attr("fixGlobalFrequency");
    fix_global_frequency();

    for (auto &t: toSet)
        setWork(t.time, t.d, t.tu);
    toSet.clear();
}

void
set(::sc_core::sc_time *time, double d, ::sc_core::sc_time_unit tu)
{
    // Only fix time once.
    if (!timeFixed) {
        timeFixed = true;

        // If we've run, python is working and we haven't fixed time yet.
        if (pythonReady)
            fixTime();
    }
    if (pythonReady) {
        // Time should be working. Set up this sc_time.
        setWork(time, d, tu);
    } else {
        // Time isn't set up yet. Defer setting up this sc_time.
        toSet.emplace_back(time, d, tu);
    }
}

class TimeSetter : public ::sc_gem5::PythonReadyFunc
{
  public:
    TimeSetter() : ::sc_gem5::PythonReadyFunc() {}

    void
    run() override
    {
        // Record that we've run and python/pybind should be usable.
        pythonReady = true;

        // If time is already fixed, let python know.
        if (timeFixed)
            fixTime();
    }
} timeSetter;

} // anonymous namespace

sc_time::sc_time() : val(0) {}

sc_time::sc_time(double d, sc_time_unit tu)
{
    val = 0;
    if (d != 0)
        set(this, d, tu);
}

sc_time::sc_time(const sc_time &t)
{
    val = t.val;
}

sc_time::sc_time(double d, bool scale)
{
    //XXX Assuming the time resolution is 1ps, and the default unit is 1ns.
    set(this, d, scale ? SC_NS : SC_PS);
}

sc_time::sc_time(sc_dt::uint64 v, bool scale)
{
    //XXX Assuming the time resolution is 1ps, and the default unit is 1ns.
    set(this, static_cast<double>(v), scale ? SC_NS : SC_PS);
}

sc_time &
sc_time::operator = (const sc_time &t)
{
    val = t.val;
    return *this;
}

sc_dt::uint64
sc_time::value() const
{
    return val;
}

double
sc_time::to_double() const
{
    return static_cast<double>(val);
}
double
sc_time::to_seconds() const
{
    double d = to_double();
    //XXX Assuming the time resolution is 1ps.
    double scale = TimeUnitScale[SC_PS] / TimeUnitScale[SC_SEC];
    return d * scale;
}

const std::string
sc_time::to_string() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

bool
sc_time::operator == (const sc_time &t) const
{
    return val == t.val;
}

bool
sc_time::operator != (const sc_time &t) const
{
    return val != t.val;
}

bool
sc_time::operator < (const sc_time &t) const
{
    return val < t.val;
}

bool
sc_time::operator <= (const sc_time &t) const
{
    return val <= t.val;
}

bool
sc_time::operator > (const sc_time &t) const
{
    return val > t.val;
}

bool
sc_time::operator >= (const sc_time &t) const
{
    return val >= t.val;
}

sc_time &
sc_time::operator += (const sc_time &t)
{
    val += t.val;
    return *this;
}

sc_time &
sc_time::operator -= (const sc_time &t)
{
    val -= t.val;
    return *this;
}

sc_time &
sc_time::operator *= (double d)
{
    val = static_cast<int64_t>(static_cast<double>(val) * d + 0.5);
    return *this;
}

sc_time &
sc_time::operator /= (double d)
{
    val = static_cast<int64_t>(static_cast<double>(val) / d + 0.5);
    return *this;
}

void
sc_time::print(std::ostream &os) const
{
    if (val == 0) {
        os << "0 s";
    } else {
        //XXX Assuming the time resolution is 1ps.
        sc_time_unit tu = SC_PS;
        uint64_t scaled = val;
        while (tu < SC_SEC && (scaled % 1000) == 0) {
            tu = (sc_time_unit)((int)tu + 1);
            scaled /= 1000;
        }

        os << scaled << ' ' << TimeUnitNames[tu];
    }
}

sc_time
sc_time::from_value(sc_dt::uint64 u)
{
    sc_time t;
    t.val = u;
    return t;
}

sc_time
sc_time::from_seconds(double d)
{
    sc_time t;
    set(&t, d, SC_SEC);
    return t;
}

sc_time
sc_time::from_string(const char *str)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_time();
}

const sc_time
operator + (const sc_time &a, const sc_time &b)
{
    return sc_time::from_value(a.value() + b.value());
}

const sc_time
operator - (const sc_time &a, const sc_time &b)
{
    return sc_time::from_value(a.value() - b.value());
}

const sc_time
operator * (const sc_time &t, double d)
{
    volatile double tmp = static_cast<double>(t.value()) * d + 0.5;
    return sc_time::from_value(static_cast<int64_t>(tmp));
}

const sc_time
operator * (double d, const sc_time &t)
{
    volatile double tmp = d * static_cast<double>(t.value()) + 0.5;
    return sc_time::from_value(static_cast<int64_t>(tmp));
}

const sc_time
operator / (const sc_time &t, double d)
{
    volatile double tmp = static_cast<double>(t.value()) / d + 0.5;
    return sc_time::from_value(static_cast<int64_t>(tmp));
}

double
operator / (const sc_time &t1, const sc_time &t2)
{
    return t1.to_double() / t2.to_double();
}

std::ostream &
operator << (std::ostream &os, const sc_time &t)
{
    t.print(os);
    return os;
}

const sc_time SC_ZERO_TIME;

void
sc_set_time_resolution(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_time
sc_get_time_resolution()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_time();
}

const sc_time &
sc_max_time()
{
    static const sc_time MaxScTime = sc_time::from_value(MaxTick);
    return MaxScTime;
}

void
sc_set_default_time_unit(double, sc_time_unit)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

sc_time
sc_get_default_time_unit()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(sc_time *)nullptr;
}

sc_time_tuple::sc_time_tuple(const sc_time &)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

bool
sc_time_tuple::has_value() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

sc_dt::uint64
sc_time_tuple::value() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0;
}

const char *
sc_time_tuple::unit_symbol() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

double
sc_time_tuple::to_double() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return 0.0;
}

std::string
sc_time_tuple::to_string() const
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return "";
}

} // namespace sc_core
