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
 *
 * Authors: Nathan Binkert
 */

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <string>

#include "base/time.hh"

using namespace std;

struct _timeval
{
    timeval tv;
};

double
convert(const timeval &tv)
{
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

Time::Time(bool set_now)
{
    time = new _timeval;
    if (set_now)
        set();
}

Time::Time(const timeval &val)
{
    time = new _timeval;
    set(val);
}

Time::Time(const Time &val)
{
    time = new _timeval;
    set(val.get());
}

Time::~Time()
{
    delete time;
}

const timeval &
Time::get() const
{
    return time->tv;
}

void
Time::set()
{
    ::gettimeofday(&time->tv, NULL);
}

void
Time::set(const timeval &tv)
{
    memcpy(&time->tv, &tv, sizeof(timeval));
}

double
Time::operator()() const
{
    return convert(get());
}

string
Time::date(string format) const
{
    const timeval &tv = get();
    time_t sec = tv.tv_sec;
    char buf[256];

    if (format.empty()) {
#ifdef __SUNPRO_CC
        ctime_r(&sec, buf, 256);
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

ostream &
operator<<(ostream &out, const Time &start)
{
    out << start.date();
    return out;
}

Time
operator-(const Time &l, const Time &r)
{
    timeval tv;
    timersub(&l.get(), &r.get(), &tv);
    return tv;
}

const Time Time::start(true);
