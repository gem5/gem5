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

#include "arch/arm/system.hh"
#include "debug/Checkpoint.hh"
#include "debug/Timer.hh"
#include "dev/arm/base_gic.hh"
#include "dev/arm/generic_timer.hh"

void
GenericTimer::SystemCounter::setFreq(uint32_t freq)
{
    if (_freq != 0) {
        // Altering the frequency after boot shouldn't be done in practice.
        warn_once("The frequency of the system counter has already been set");
    }
    _freq = freq;
    _period = (1.0 / freq) * SimClock::Frequency;
    _resetTick = curTick();
}

void
GenericTimer::SystemCounter::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(_freq);
    SERIALIZE_SCALAR(_period);
    SERIALIZE_SCALAR(_resetTick);
}

void
GenericTimer::SystemCounter::unserialize(Checkpoint *cp,
                                         const std::string &section)
{
    UNSERIALIZE_SCALAR(_freq);
    UNSERIALIZE_SCALAR(_period);
    UNSERIALIZE_SCALAR(_resetTick);
}

void
GenericTimer::ArchTimer::counterLimitReached()
{
    _control.istatus = 1;

    if (!_control.enable)
        return;

    // DPRINTF(Timer, "Counter limit reached\n");

    if (!_control.imask) {
        // DPRINTF(Timer, "Causing interrupt\n");
        _parent->_gic->sendPPInt(_intNum, _cpuNum);
    }
}

void
GenericTimer::ArchTimer::setCompareValue(uint64_t val)
{
    _counterLimit = val;
    if (_counterLimitReachedEvent.scheduled())
        _parent->deschedule(_counterLimitReachedEvent);
    if (counterValue() >= _counterLimit) {
        counterLimitReached();
    } else {
        _control.istatus = 0;
        _parent->schedule(_counterLimitReachedEvent,
             curTick() + (_counterLimit - counterValue()) * _counter->period());
    }
}

void
GenericTimer::ArchTimer::setTimerValue(uint32_t val)
{
    setCompareValue(counterValue() + sext<32>(val));
}

void
GenericTimer::ArchTimer::setControl(uint32_t val)
{
    ArchTimerCtrl new_ctl = val;
    if ((new_ctl.enable && !new_ctl.imask) &&
        !(_control.enable && !_control.imask)) {
        // Re-evalute the timer condition
        if (_counterLimit >= counterValue()) {
            _control.istatus = 1;

            DPRINTF(Timer, "Causing interrupt in control\n");
            //_parent->_gic->sendPPInt(_intNum, _cpuNum);
        }
    }
    _control.enable = new_ctl.enable;
    _control.imask = new_ctl.imask;
}

void
GenericTimer::ArchTimer::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(_cpuNum);
    SERIALIZE_SCALAR(_intNum);
    uint32_t control_serial = _control;
    SERIALIZE_SCALAR(control_serial);
    SERIALIZE_SCALAR(_counterLimit);
    bool event_scheduled = _counterLimitReachedEvent.scheduled();
    SERIALIZE_SCALAR(event_scheduled);
    Tick event_time;
    if (event_scheduled) {
        event_time = _counterLimitReachedEvent.when();
        SERIALIZE_SCALAR(event_time);
    }
}

void
GenericTimer::ArchTimer::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(_cpuNum);
    UNSERIALIZE_SCALAR(_intNum);
    uint32_t control_serial;
    UNSERIALIZE_SCALAR(control_serial);
    _control = control_serial;
    bool event_scheduled;
    UNSERIALIZE_SCALAR(event_scheduled);
    Tick event_time;
    if (event_scheduled) {
        UNSERIALIZE_SCALAR(event_time);
        _parent->schedule(_counterLimitReachedEvent, event_time);
    }
}

GenericTimer::GenericTimer(Params *p)
    : SimObject(p), _gic(p->gic)
{
   for (int i = 0; i < CPU_MAX; ++i) {
        std::stringstream oss;
        oss << name() << ".arch_timer" << i;
        _archTimers[i]._name = oss.str();
        _archTimers[i]._parent = this;
        _archTimers[i]._counter = &_systemCounter;
        _archTimers[i]._cpuNum = i;
        _archTimers[i]._intNum = p->int_num;
   }

   ((ArmSystem *) p->system)->setGenericTimer(this);
}

void
GenericTimer::serialize(std::ostream &os)
{
    nameOut(os, csprintf("%s.sys_counter", name()));
    _systemCounter.serialize(os);
    for (int i = 0; i < CPU_MAX; ++i) {
        nameOut(os, csprintf("%s.arch_timer%d", name(), i));
        _archTimers[i].serialize(os);
    }
}

void
GenericTimer::unserialize(Checkpoint *cp, const std::string &section)
{
    _systemCounter.unserialize(cp, csprintf("%s.sys_counter", section));
    for (int i = 0; i < CPU_MAX; ++i) {
        _archTimers[i].unserialize(cp, csprintf("%s.arch_timer%d", section, i));
    }
}

GenericTimer *
GenericTimerParams::create()
{
    return new GenericTimer(this);
}
