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

#ifndef __CPU_KVM_TIMER_HH__
#define __CPU_KVM_TIMER_HH__

#include <ctime>

#include "cpu/kvm/perfevent.hh"
#include "sim/core.hh"

namespace gem5
{

/**
 * Timer functions to interrupt VM execution after a number of
 * simulation ticks. The timer allows scaling of the host time to take
 * performance differences between the simulated and real CPU into
 * account.
 *
 * The performance scaling factor is ratio between the target's CPI
 * and the host's CPI. It is larger than 1 if the host is faster than
 * the target and lower than 1 if it is slower.
 *
 * When the timer times out, it sends a signal to the thread that
 * started the timer. The signal forces KVM to drop out of the system
 * call that started the guest and hands control to gem5.
 */
class BaseKvmTimer
{
  public:
    /**
     * Setup basic timer functionality shared by all timer
     * implementations.
     *
     * @param signo Signal to deliver
     * @param hostFactor Performance scaling factor
     * @param hostFreq Clock frequency of the host
     */
    BaseKvmTimer(int signo, float hostFactor, Tick hostFreq)
        : signo(signo),
          _resolution(0),
          hostFactor(hostFactor),
          hostFreq(hostFreq){};
    virtual ~BaseKvmTimer(){};

    /**
     * Arm the timer so that it fires after a certain number of ticks.
     *
     * @note A timer implementation is free to convert between
     * simulation ticks and virtualized time using any method it
     * chooses. The accuracy of the timer therefore depends on what it
     * measures, an accurate timer implementation should measure the
     * number of cycles or instructions executed in the guest. If such
     * counters are unavailable, it may fallback to wall clock time.
     *
     * @param ticks Number of ticks until the timer fires
     */
    virtual void arm(Tick ticks) = 0;
    /**
     * Disarm the timer.
     *
     * When this method has returned, the timer may no longer deliver
     * signals upon timeout.
     */
    virtual void disarm() = 0;

    virtual bool
    expired()
    {
        return true;
    }

    /**
     * Determine the resolution of the timer in ticks. This method is
     * mainly used to determine the smallest number of ticks the timer
     * can wait before triggering a signal.
     *
     * @return Minimum number of ticks the timer can resolve
     */
    Tick
    resolution()
    {
        if (_resolution == 0)
            _resolution = calcResolution();
        return _resolution;
    }

    /**
     * Convert cycles executed on the host into Ticks executed in the
     * simulator. Scales the results using the hostFactor to take CPU
     * performance differences into account.
     *
     * @return Host cycles executed in VM converted to simulation ticks
     */
    Tick
    ticksFromHostCycles(uint64_t cycles)
    {
        return cycles * hostFactor * hostFreq;
    }

    /**
     * Convert nanoseconds executed on the host into Ticks executed in
     * the simulator. Scales the results using the hostFactor to take
     * CPU performance differences into account.
     *
     * @return Nanoseconds executed in VM converted to simulation ticks
     */
    Tick
    ticksFromHostNs(uint64_t ns)
    {
        return ns * hostFactor * sim_clock::as_float::ns;
    }

  protected:
    /**
     * Calculate the timer resolution, used by resolution() which
     * caches the result.
     *
     * @return Minimum number of ticks the timer can resolve
     */
    virtual Tick calcResolution() = 0;

    /**
     * Convert a time in simulator ticks to host nanoseconds.
     *
     * @return Simulation ticks converted into nanoseconds on the host
     */
    uint64_t
    hostNs(Tick ticks)
    {
        return ticks / (sim_clock::as_float::ns * hostFactor);
    }

    /**
     * Convert a time in simulator ticks to host cycles
     *
     *
     * @return Simulation ticks converted into CPU cycles on the host
     */
    uint64_t
    hostCycles(Tick ticks)
    {
        return ticks / (hostFreq * hostFactor);
    }

    /** Signal to deliver when the timer times out */
    int signo;

  private:
    /** Cached resolution */
    mutable Tick _resolution;

    /** Performance scaling factor */
    float hostFactor;
    /** Host frequency */
    Tick hostFreq;
};

/**
 * Timer based on standard POSIX timers. The POSIX timer API supports
 * several different clock with different characteristics.
 *
 * @note It might be tempting to use
 * CLOCK_(THREAD|PROCESS)_CPUTIME_ID, however, this clock usually has
 * much lower resolution than the real-time clocks.
 */
class PosixKvmTimer : public BaseKvmTimer
{
  public:
    /**
     * @param signo Signal to deliver
     * @param clockID ID of the clock to use
     * @param hostFactor Performance scaling factor
     * @param hostFreq Clock frequency of the host
     */
    PosixKvmTimer(int signo, clockid_t clockID, float hostFactor,
                  Tick hostFreq);
    ~PosixKvmTimer();

    void arm(Tick ticks) override;
    void disarm() override;
    bool expired() override;

  protected:
    Tick calcResolution() override;

  private:
    clockid_t clockID;
    timer_t timer;
    struct itimerspec prevTimerSpec;
};

/**
 * PerfEvent based timer using the host's CPU cycle counter.
 *
 * @warning There is a known problem in some versions of the PerfEvent
 * API that prevents the counter overflow period from being updated
 * reliably, which might break this timer. See PerfKvmCounter::period()
 * for details.
 */
class PerfKvmTimer : public BaseKvmTimer
{
  public:
    /**
     * Create a timer that uses an existing hardware cycle counter.
     *
     * @note The performance counter must be configured for overflow
     * sampling, which in practice means that it must have a non-zero
     * sample period. The initial sample period is ignored since
     * period will be updated when arm() is called.
     *
     * @param ctr Attached performance counter configured for overflow
     *            reporting.
     * @param signo Signal to deliver
     * @param hostFactor Performance scaling factor
     * @param hostFreq Clock frequency of the host
     */
    PerfKvmTimer(PerfKvmCounter &ctr, int signo, float hostFactor,
                 Tick hostFreq);
    ~PerfKvmTimer();

    void arm(Tick ticks);
    void disarm();

  protected:
    Tick calcResolution();

  private:
    PerfKvmCounter &hwOverflow;
};

} // namespace gem5

#endif
