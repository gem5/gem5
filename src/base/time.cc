/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "base/time.hh"

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "base/logging.hh"
#include "config/use_posix_clock.hh"
#include "sim/core.hh"
#include "sim/serialize.hh"

namespace gem5
{

void
Time::_set(bool monotonic)
{
#if USE_POSIX_CLOCK
    ::clock_gettime(monotonic ? CLOCK_MONOTONIC : CLOCK_REALTIME, &_time);
#else
    timeval tv;
    ::gettimeofday(&tv, NULL);
    operator=(tv);
#endif
}

void
Time::setTick(Tick ticks)
{
    uint64_t secs = ticks / sim_clock::Frequency;
    ticks -= secs * sim_clock::Frequency;
    uint64_t nsecs = static_cast<uint64_t>(ticks * sim_clock::as_float::GHz);
    set(secs, nsecs);
}

Tick
Time::getTick() const
{
    return sec() * sim_clock::Frequency +
           static_cast<uint64_t>(nsec() * sim_clock::as_float::ns);
}

std::string
Time::date(const std::string &format) const
{
    time_t sec = this->sec();
    char buf[256];

    if (format.empty()) {
#ifdef __SUNPRO_CC
        ctime_r(&sec, buf, sizeof(buf));
#else
        ctime_r(&sec, buf);
#endif
        buf[24] = '\0';
        return buf;
    }

    struct tm *tm = localtime(&sec);
    strftime(buf, sizeof(buf), format.c_str(), tm);
    return buf;
}

std::string
Time::time() const
{
    double time = double(*this);
    double secs = fmod(time, 60.0);
    double all_mins = floor(time / 60.0);
    double mins = fmod(all_mins, 60.0);
    double hours = floor(all_mins / 60.0);

    std::stringstream str;

    if (hours > 0.0) {
        if (hours < 10.0)
            str << '0';
        str << hours << ':';
    }

    if (mins > 0.0) {
        if (mins < 10.0)
            str << '0';
        str << mins << ':';
    }

    if (secs < 10.0 && !str.str().empty())
        str << '0';
    str << secs;

    return str.str();
}

void
Time::serialize(const std::string &base, CheckpointOut &cp) const
{
    paramOut(cp, base + ".sec", sec());
    paramOut(cp, base + ".nsec", nsec());
}

void
Time::unserialize(const std::string &base, CheckpointIn &cp)
{
    time_t secs;
    time_t nsecs;
    paramIn(cp, base + ".sec", secs);
    paramIn(cp, base + ".nsec", nsecs);
    sec(secs);
    nsec(nsecs);
}

void
sleep(const Time &time)
{
    timespec ts = time;

#if USE_POSIX_CLOCK
    clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
#else
    nanosleep(&ts, NULL);
#endif
}

time_t
mkutctime(struct tm *time)
{
    // get the current timezone
    char *tz = getenv("TZ");

    // copy the string as the pointer gets invalidated when updating
    // the environment
    if (tz) {
        tz = strdup(tz);
        if (!tz) {
            fatal("Failed to reserve memory for UTC time conversion\n");
        }
    }

    // change to UTC and get the time
    setenv("TZ", "", 1);
    tzset();
    time_t ret = mktime(time);

    // restore the timezone again
    if (tz) {
        setenv("TZ", tz, 1);
        free(tz);
    } else {
        unsetenv("TZ");
    }
    tzset();

    return ret;
}

} // namespace gem5
