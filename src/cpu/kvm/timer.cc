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
 */

#include "cpu/kvm/timer.hh"

#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <ctime>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/KvmTimer.hh"

/* According to timer_create(2), the value SIGEV_THREAD_ID can be used
 * to specify which thread a timer signal gets delivered to. According
 * to the man page, the member sigev_notify_thread is used to specify
 * the TID. This member is currently not defined by default in
 * siginfo.h on x86, so we define it here as a workaround.
 */
#ifndef sigev_notify_thread_id
#define sigev_notify_thread_id     _sigev_un._tid
#endif

static pid_t
sysGettid()
{
    return syscall(__NR_gettid);
}

/**
 * Minimum number of cycles that a host can spend in a KVM call (used
 * to calculate the resolution of some timers).
 *
 * The value of this constant is a bit arbitrary, but in practice, we
 * can't really do anything useful in less than ~1000 cycles.
 */
static const uint64_t MIN_HOST_CYCLES = 1000;

PosixKvmTimer::PosixKvmTimer(int signo, clockid_t clockID,
                             float hostFactor, Tick hostFreq)
    : BaseKvmTimer(signo, hostFactor, hostFreq),
      clockID(clockID)
{
    struct sigevent sev;

    sev.sigev_notify = SIGEV_THREAD_ID;
    sev.sigev_signo = signo;
    sev.sigev_notify_thread_id = sysGettid();
    sev.sigev_value.sival_ptr = NULL;

    while (timer_create(clockID, &sev, &timer) == -1) {
        if (errno != EAGAIN)
            panic("timer_create: %i", errno);
    }
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

    assert(ts.it_value.tv_nsec > 0 || ts.it_value.tv_sec > 0);

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

    const uint64_t res_ns(ts.tv_sec * 1000000000ULL + ts.tv_nsec);
    // We preferrably want ticksFromHostNs() to calculate the the
    // ceiling rather than truncating the value. However, there are
    // other cases where truncating is fine, so we just add 1 here to
    // make sure that the actual resolution is strictly less than what
    // we return. We could get all kinds of nasty behavior if
    // arm(resolution) is called and the resulting time is 0 (which
    // could happen if we truncate the results and the resolution is
    // 1ns).
    const Tick resolution(ticksFromHostNs(res_ns) + 1);
    // It might not make sense to enter into KVM for less than a
    // certain number of host cycles. In some systems (e.g., Linux)
    // the resolution of the timer we use is 1ns (a few cycles on most
    // CPUs), which isn't very useful.
    const Tick min_cycles(ticksFromHostCycles(MIN_HOST_CYCLES));

    return std::max(resolution, min_cycles);
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
    return ticksFromHostCycles(MIN_HOST_CYCLES);
}
