/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Andreas Sandberg
 */

#include <csignal>
#include <ctime>

#include "base/misc.hh"
#include "base/trace.hh"
#include "cpu/kvm/timer.hh"
#include "debug/KvmTimer.hh"


PosixKvmTimer::PosixKvmTimer(int signo, clockid_t clockID,
                             float hostFactor, Tick hostFreq)
    : BaseKvmTimer(signo, hostFactor, hostFreq),
      clockID(clockID)
{
    struct sigevent sev;

    // TODO: We should request signal delivery to thread instead of
    // the process here. Unfortunately this seems to be broken, or at
    // least not work as specified in the man page.
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = signo;
    sev.sigev_value.sival_ptr = NULL;
    if (timer_create(clockID, &sev, &timer) == -1)
        panic("timer_create");
}

PosixKvmTimer::~PosixKvmTimer()
{
    timer_delete(timer);
}

void
PosixKvmTimer::arm(Tick ticks)
{
    struct itimerspec ts;
    memset(&ts, 0, sizeof(ts));

    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = 0;
    ts.it_value.tv_sec = hostNs(ticks) / 1000000000ULL;
    ts.it_value.tv_nsec = hostNs(ticks) % 1000000000ULL;

    DPRINTF(KvmTimer, "Arming POSIX timer: %i ticks (%is%ins)\n",
            ticks, ts.it_value.tv_sec, ts.it_value.tv_nsec);

    if (timer_settime(timer, 0, &ts, NULL) == -1)
        panic("PosixKvmTimer: Failed to arm timer\n");
}

void
PosixKvmTimer::disarm()
{
    struct itimerspec ts;
    memset(&ts, 0, sizeof(ts));

    DPRINTF(KvmTimer, "Disarming POSIX timer\n");

    if (timer_settime(timer, 0, &ts, NULL) == -1)
        panic("PosixKvmTimer: Failed to disarm timer\n");
}

Tick
PosixKvmTimer::calcResolution()
{
    struct timespec ts;

    if (clock_getres(clockID, &ts) == -1)
        panic("PosixKvmTimer: Failed to get timer resolution\n");

    Tick resolution(ticksFromHostNs(ts.tv_sec * 1000000000ULL + ts.tv_nsec));

    return resolution;
}


PerfKvmTimer::PerfKvmTimer(PerfKvmCounter &ctr,
                           int signo, float hostFactor, Tick hostFreq)
    : BaseKvmTimer(signo, hostFactor, hostFreq),
      hwOverflow(ctr)
{
    hwOverflow.enableSignals(signo);
}

PerfKvmTimer::~PerfKvmTimer()
{
}

void
PerfKvmTimer::arm(Tick ticks)
{
    hwOverflow.period(hostCycles(ticks));
    hwOverflow.refresh(1);
}

void
PerfKvmTimer::disarm()
{
    hwOverflow.stop();
}

Tick
PerfKvmTimer::calcResolution()
{
    // This is a bit arbitrary, but in practice, we can't really do
    // anything useful in less than ~1000 anyway.
    return ticksFromHostCycles(1000);
}
