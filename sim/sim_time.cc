/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <iostream>

#include "sim/sim_time.hh"

using namespace std;

namespace Time
{
    struct _timeval
    {
        timeval tv;
    };

    double
    convert(const timeval &tv)
    {
        return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
    }

    Start::Start()
    {
        start = new _timeval;
        ::gettimeofday(&start->tv, NULL);
    }

    Start::~Start()
    {
        delete start;
    }

    const timeval &
    Start::get() const
    {
        return start->tv;
    }

    double
    Start::operator()() const
    {
        return convert(get());
    }

    Now::Now()
        : now(0)
    {
    }

    Now::~Now()
    {
        if (now)
            delete now;
    }

    const timeval &
    Now::get() const
    {
        if (!now)
            now = new _timeval;

        ::gettimeofday(&now->tv, NULL);
        return now->tv;
    }

    double
    Now::operator()() const
    {
        return convert(get());
    }


    Elapsed::Elapsed()
        : elapsed(0)
    {}

    Elapsed::~Elapsed()
    {
        if (elapsed)
            delete elapsed;
    }

    const timeval &
    Elapsed::get() const
    {
        if (!elapsed)
            elapsed = new _timeval;

        timersub(&now.get(), &start.get(), &elapsed->tv);
        return elapsed->tv;
    }

    double
    Elapsed::operator()() const
    {
        return convert(get());
    }

    Start start;
    Now now;
    Elapsed elapsed;

    ostream &
    operator<<(ostream &out, const Start &start)
    {
        out << ::ctime(&start.get().tv_sec);
        return out;
    }

    ostream &
    operator<<(ostream &out, const Now &now)
    {
        out << ::ctime(&now.get().tv_sec);
        return out;
    }

    ostream &
    operator<<(ostream &out, const Elapsed &elapsed)
    {
        out << ::ctime(&elapsed.get().tv_sec);
        return out;
    }
}
