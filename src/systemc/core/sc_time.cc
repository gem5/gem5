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
 */

#include <cmath>
#include <cstring>
#include <sstream>
#include <vector>

#include "base/types.hh"
#include "sim/core.hh"
#include "systemc/core/time.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_time.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_core
{

namespace
{

void
set(::sc_core::sc_time *time, double d, ::sc_core::sc_time_unit tu)
{
    if (d != 0)
        gem5::fixClockFrequency();

    double scale = sc_gem5::TimeUnitScale[tu] * gem5::sim_clock::as_float::s;
    // Accellera claims there is a linux bug, and that these next two
    // lines work around them.
    volatile double tmp = d * scale + 0.5;
    *time = sc_time::from_value(static_cast<uint64_t>(tmp));
}

double defaultUnit = 1.0e-9;

} // anonymous namespace

sc_time::sc_time() : val(0) {}

sc_time::sc_time(double d, sc_time_unit tu)
{
    val = 0;
    set(this, d, tu);
}

sc_time::sc_time(const sc_time &t) { val = t.val; }

sc_time::sc_time(double d, const char *unit)
{
    sc_time_unit tu;
    for (tu = SC_FS; tu <= SC_SEC; tu = (sc_time_unit)(tu + 1)) {
        if (strcmp(unit, sc_gem5::TimeUnitNames[tu]) == 0 ||
            strcmp(unit, sc_gem5::TimeUnitConstantNames[tu]) == 0) {
            break;
        }
    }

    if (tu > SC_SEC) {
        SC_REPORT_ERROR(SC_ID_TIME_CONVERSION_FAILED_, "invalid unit given");
        val = 0;
        return;
    }
    set(this, d, tu);
}

sc_time::sc_time(double d, bool scale)
{
    double scaler = scale ? defaultUnit : gem5::sim_clock::as_float::Hz;
    set(this, d * scaler, SC_SEC);
}

sc_time::sc_time(sc_dt::uint64 v, bool scale)
{
    double scaler = scale ? defaultUnit : gem5::sim_clock::as_float::Hz;
    set(this, static_cast<double>(v) * scaler, SC_SEC);
}

sc_time &
sc_time::operator=(const sc_time &t)
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
    return to_double() * gem5::sim_clock::as_float::Hz;
}

const std::string
sc_time::to_string() const
{
    std::ostringstream ss;
    print(ss);
    return ss.str();
}

bool
sc_time::operator==(const sc_time &t) const
{
    return val == t.val;
}

bool
sc_time::operator!=(const sc_time &t) const
{
    return val != t.val;
}

bool
sc_time::operator<(const sc_time &t) const
{
    return val < t.val;
}

bool
sc_time::operator<=(const sc_time &t) const
{
    return val <= t.val;
}

bool
sc_time::operator>(const sc_time &t) const
{
    return val > t.val;
}

bool
sc_time::operator>=(const sc_time &t) const
{
    return val >= t.val;
}

sc_time &
sc_time::operator+=(const sc_time &t)
{
    val += t.val;
    return *this;
}

sc_time &
sc_time::operator-=(const sc_time &t)
{
    val -= t.val;
    return *this;
}

sc_time &
sc_time::operator*=(double d)
{
    val = static_cast<int64_t>(static_cast<double>(val) * d + 0.5);
    return *this;
}

sc_time &
sc_time::operator/=(double d)
{
    val = static_cast<int64_t>(static_cast<double>(val) / d + 0.5);
    return *this;
}

void
sc_time::print(std::ostream &os) const
{
    os << sc_time_tuple(*this).to_string();
}

sc_time
sc_time::from_value(sc_dt::uint64 u)
{
    if (u)
        gem5::fixClockFrequency();
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
    char *end = nullptr;

    double d = str ? std::strtod(str, &end) : 0.0;
    if (str == end || d < 0.0) {
        SC_REPORT_ERROR(SC_ID_TIME_CONVERSION_FAILED_, "invalid value given");
        return SC_ZERO_TIME;
    }

    while (*end && std::isspace(*end))
        end++;

    return sc_time(d, end);
}

const sc_time
operator+(const sc_time &a, const sc_time &b)
{
    return sc_time::from_value(a.value() + b.value());
}

const sc_time
operator-(const sc_time &a, const sc_time &b)
{
    return sc_time::from_value(a.value() - b.value());
}

const sc_time
operator*(const sc_time &t, double d)
{
    volatile double tmp = static_cast<double>(t.value()) * d + 0.5;
    return sc_time::from_value(static_cast<int64_t>(tmp));
}

const sc_time
operator*(double d, const sc_time &t)
{
    volatile double tmp = d * static_cast<double>(t.value()) + 0.5;
    return sc_time::from_value(static_cast<int64_t>(tmp));
}

const sc_time
operator/(const sc_time &t, double d)
{
    volatile double tmp = static_cast<double>(t.value()) / d + 0.5;
    return sc_time::from_value(static_cast<int64_t>(tmp));
}

double
operator/(const sc_time &t1, const sc_time &t2)
{
    return t1.to_double() / t2.to_double();
}

std::ostream &
operator<<(std::ostream &os, const sc_time &t)
{
    t.print(os);
    return os;
}

const sc_time SC_ZERO_TIME;

void
sc_set_time_resolution(double d, sc_time_unit tu)
{
    if (d <= 0.0)
        SC_REPORT_ERROR(SC_ID_SET_TIME_RESOLUTION_, "value not positive");

    double dummy;
    if (modf(log10(d), &dummy) != 0.0) {
        SC_REPORT_ERROR(SC_ID_SET_TIME_RESOLUTION_,
                        "value not a power of ten");
    }
    if (sc_is_running())
        SC_REPORT_ERROR(SC_ID_SET_TIME_RESOLUTION_, "simulation running");

    static bool specified = false;
    if (specified)
        SC_REPORT_ERROR(SC_ID_SET_TIME_RESOLUTION_, "already specified");

    // This won't detect the timescale being fixed outside of systemc, but
    // it's at least some protection.
    if (gem5::clockFrequencyFixed()) {
        SC_REPORT_ERROR(SC_ID_SET_TIME_RESOLUTION_,
                        "sc_time object(s) constructed");
    }

    double seconds = d * sc_gem5::TimeUnitScale[tu];
    if (seconds < sc_gem5::TimeUnitScale[SC_FS])
        SC_REPORT_ERROR(SC_ID_SET_TIME_RESOLUTION_, "value smaller than 1 fs");

    if (seconds > defaultUnit) {
        SC_REPORT_WARNING(SC_ID_DEFAULT_TIME_UNIT_CHANGED_, "");
        defaultUnit = seconds;
    }

    // Get rid of fractional parts of d.
    while (d < 1.0 && tu > SC_FS) {
        d *= 1000;
        tu = (sc_time_unit)(tu - 1);
    }

    gem5::Tick ticks_per_second =
        sc_gem5::TimeUnitFrequency[tu] / static_cast<gem5::Tick>(d);
    gem5::setClockFrequency(ticks_per_second);
    specified = true;
}

sc_time
sc_get_time_resolution()
{
    return sc_time::from_value(1);
}

const sc_time &
sc_max_time()
{
    static const sc_time MaxScTime = sc_time::from_value(gem5::MaxTick);
    return MaxScTime;
}

void
sc_set_default_time_unit(double d, sc_time_unit tu)
{
    if (d < 0.0)
        SC_REPORT_ERROR(SC_ID_SET_DEFAULT_TIME_UNIT_, "value not positive");

    double dummy;
    if (modf(log10(d), &dummy) != 0.0) {
        SC_REPORT_ERROR(SC_ID_SET_DEFAULT_TIME_UNIT_,
                        "value not a power of ten");
    }
    if (sc_is_running())
        SC_REPORT_ERROR(SC_ID_SET_DEFAULT_TIME_UNIT_, "simulation running");

    static bool specified = false;
    if (specified) {
        SC_REPORT_ERROR(SC_ID_SET_DEFAULT_TIME_UNIT_, "already specified");
    }
    // This won't detect the timescale being fixed outside of systemc, but
    // it's at least some protection.
    if (gem5::clockFrequencyFixed()) {
        SC_REPORT_ERROR(SC_ID_SET_DEFAULT_TIME_UNIT_,
                        "sc_time object(s) constructed");
    }

    // Normalize d to seconds.
    defaultUnit = d * sc_gem5::TimeUnitScale[tu];
    specified = true;

    double resolution = gem5::sim_clock::as_float::Hz;
    if (resolution == 0.0)
        resolution = sc_gem5::TimeUnitScale[SC_PS];
    if (defaultUnit < resolution) {
        SC_REPORT_ERROR(SC_ID_SET_DEFAULT_TIME_UNIT_,
                        "value smaller than time resolution");
    }
}

sc_time
sc_get_default_time_unit()
{
    return sc_time(defaultUnit, SC_SEC);
}

sc_time_tuple::sc_time_tuple(const sc_time &t)
    : _value(), _unit(SC_SEC), _set(true)
{
    if (!t.value())
        return;

    gem5::Tick frequency = gem5::sim_clock::Frequency;

    // Shrink the frequency by scaling down the time period, ie converting
    // it from cycles per second to cycles per millisecond, etc.
    while (_unit > 1 && (frequency % 1000 == 0)) {
        _unit = (sc_time_unit)((int)_unit - 1);
        frequency /= 1000;
    }

    // Convert the frequency into a period.
    gem5::Tick period;
    if (frequency > 1) {
        _unit = (sc_time_unit)((int)_unit - 1);
        period = 1000 / frequency;
    } else {
        period = frequency;
    }

    // Scale our integer value by the period.
    _value = t.value() * period;

    // Shrink the scaled time value by increasing the size of the units
    // it's measured by, avoiding fractional parts.
    while (_unit < SC_SEC && (_value % 1000) == 0) {
        _unit = (sc_time_unit)((int)_unit + 1);
        _value /= 1000;
    }
}

bool
sc_time_tuple::has_value() const
{
    return _set;
}

sc_dt::uint64
sc_time_tuple::value() const
{
    return _value;
}

const char *
sc_time_tuple::unit_symbol() const
{
    return sc_gem5::TimeUnitNames[_unit];
}

double
sc_time_tuple::to_double() const
{
    return static_cast<double>(_value);
}

std::string
sc_time_tuple::to_string() const
{
    std::ostringstream ss;
    ss << _value << ' ' << unit_symbol();
    return ss.str();
}

} // namespace sc_core
