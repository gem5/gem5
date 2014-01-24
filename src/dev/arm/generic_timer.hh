/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved.
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
 * Authors: Giacomo Gabrielli
 */

#ifndef __DEV_ARM_GENERIC_TIMER_HH__
#define __DEV_ARM_GENERIC_TIMER_HH__

#include "base/bitunion.hh"
#include "params/GenericTimer.hh"
#include "sim/core.hh"
#include "sim/sim_object.hh"

/// @file
/// This module implements the global system counter and the local per-CPU
/// architected timers as specified by the ARM Generic Timer extension (ARM
/// ARM, Issue C, Chapter 17).

class Checkpoint;
class BaseGic;

/// Wrapper around the actual counters and timers of the Generic Timer
/// extension.
class GenericTimer : public SimObject
{
  public:

    /// Global system counter.  It is shared by the architected timers.
    /// @todo: implement memory-mapped controls
    class SystemCounter
    {
      protected:
        /// Counter frequency (as specified by CNTFRQ).
        uint64_t _freq;
        /// Cached copy of the counter period (inverse of the frequency).
        Tick _period;
        /// Tick when the counter was reset.
        Tick _resetTick;

      public:
        /// Ctor.
        SystemCounter()
            : _freq(0), _period(0), _resetTick(0)
        {
            setFreq(0x01800000);
        }

        /// Returns the current value of the physical counter.
        uint64_t value() const
        {
            if (_freq == 0)
                return 0;  // Counter is still off.
            return (curTick() - _resetTick) / _period;
        }

        /// Returns the counter frequency.
        uint64_t freq() const { return _freq; }
        /// Sets the counter frequency.
        /// @param freq frequency in Hz.
        void setFreq(uint32_t freq);

        /// Returns the counter period.
        Tick period() const { return _period; }

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);
    };

    /// Per-CPU architected timer.
    class ArchTimer
    {
      protected:
        /// Control register.
        BitUnion32(ArchTimerCtrl)
            Bitfield<0> enable;
            Bitfield<1> imask;
            Bitfield<2> istatus;
        EndBitUnion(ArchTimerCtrl)

        /// Name of this timer.
        std::string _name;
        /// Pointer to parent class.
        GenericTimer *_parent;
        /// Pointer to the global system counter.
        SystemCounter *_counter;
        /// ID of the CPU this timer is attached to.
        int _cpuNum;
        /// ID of the interrupt to be triggered.
        int _intNum;
        /// Cached value of the control register ({CNTP/CNTHP/CNTV}_CTL).
        ArchTimerCtrl _control;
        /// Programmed limit value for the upcounter ({CNTP/CNTHP/CNTV}_CVAL).
        uint64_t _counterLimit;

        /// Called when the upcounter reaches the programmed value.
        void counterLimitReached();
        EventWrapper<ArchTimer, &ArchTimer::counterLimitReached>
            _counterLimitReachedEvent;

        /// Returns the value of the counter which this timer relies on.
        uint64_t counterValue() const { return _counter->value(); }

      public:
        /// Ctor.
        ArchTimer()
            : _control(0), _counterLimit(0), _counterLimitReachedEvent(this)
        {}

        /// Returns the timer name.
        std::string name() const { return _name; }

        /// Returns the CompareValue view of the timer.
        uint64_t compareValue() const { return _counterLimit; }
        /// Sets the CompareValue view of the timer.
        void setCompareValue(uint64_t val);

        /// Returns the TimerValue view of the timer.
        uint32_t timerValue() const { return _counterLimit - counterValue(); }
        /// Sets the TimerValue view of the timer.
        void setTimerValue(uint32_t val);

        /// Sets the control register.
        uint32_t control() const { return _control; }
        void setControl(uint32_t val);

        virtual void serialize(std::ostream &os);
        virtual void unserialize(Checkpoint *cp, const std::string &section);

        friend class GenericTimer;
    };

  protected:

    static const int CPU_MAX = 8;

    /// Pointer to the GIC, needed to trigger timer interrupts.
    BaseGic *_gic;
    /// System counter.
    SystemCounter _systemCounter;
    /// Per-CPU architected timers.
    // @todo: this would become a 2-dim. array with Security and Virt.
    ArchTimer _archTimers[CPU_MAX];

  public:
    typedef GenericTimerParams Params;
    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    /// Ctor.
    GenericTimer(Params *p);

    /// Returns a pointer to the system counter.
    SystemCounter *getSystemCounter() { return &_systemCounter; }

    /// Returns a pointer to the architected timer for cpu_id.
    ArchTimer *getArchTimer(int cpu_id) { return &_archTimers[cpu_id]; }

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

#endif // __DEV_ARM_GENERIC_TIMER_HH__
