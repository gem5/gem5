/*
 * Copyright (c) 2013, 2015, 2017-2018,2020 ARM Limited
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
 */

#include "dev/arm/generic_timer.hh"

#include <cmath>

#include "arch/arm/system.hh"
#include "arch/arm/utility.hh"
#include "cpu/base.hh"
#include "debug/Timer.hh"
#include "dev/arm/base_gic.hh"
#include "mem/packet_access.hh"
#include "params/GenericTimer.hh"
#include "params/GenericTimerFrame.hh"
#include "params/GenericTimerMem.hh"
#include "params/SystemCounter.hh"

using namespace ArmISA;

SystemCounter::SystemCounter(SystemCounterParams *const p)
    : SimObject(p),
      _enabled(true),
      _value(0),
      _increment(1),
      _freqTable(p->freqs),
      _activeFreqEntry(0),
      _updateTick(0),
      _freqUpdateEvent([this]{ freqUpdateCallback(); }, name()),
      _nextFreqEntry(0)
{
    fatal_if(_freqTable.empty(), "SystemCounter::SystemCounter: Base "
        "frequency not provided\n");
    // Store the table end marker as a 32-bit zero word
    _freqTable.push_back(0);
    fatal_if(_freqTable.size() > MAX_FREQ_ENTRIES,
        "SystemCounter::SystemCounter: Architecture states a maximum of 1004 "
        "frequency table entries, limit surpassed\n");
    // Set the active frequency to be the base
    _freq = _freqTable.front();
    _period = (1.0 / _freq) * SimClock::Frequency;
}

void
SystemCounter::validateCounterRef(SystemCounter *const sys_cnt)
{
    fatal_if(!sys_cnt, "SystemCounter::validateCounterRef: No valid system "
             "counter, can't instantiate system timers\n");
}

void
SystemCounter::enable()
{
    DPRINTF(Timer, "SystemCounter::enable: Counter enabled\n");
    _enabled = true;
    updateTick();
}

void
SystemCounter::disable()
{
    DPRINTF(Timer, "SystemCounter::disable: Counter disabled\n");
    updateValue();
    _enabled = false;
}

uint64_t
SystemCounter::value()
{
    if (_enabled)
        updateValue();
    return _value;
}

void
SystemCounter::updateValue()
{
    uint64_t new_value =
        _value + ((curTick() - _updateTick) / _period) * _increment;
    if (new_value > _value) {
        _value = new_value;
        updateTick();
    }
}

void
SystemCounter::setValue(uint64_t new_value)
{
    if (_enabled)
        warn("Explicit value set with counter enabled, UNKNOWNN result\n");
    _value = new_value;
    updateTick();
    notifyListeners();
}

Tick
SystemCounter::whenValue(uint64_t cur_val, uint64_t target_val) const
{
    Tick when = curTick();
    if (target_val > cur_val) {
        uint64_t num_cycles =
            std::ceil((target_val - cur_val) / ((double) _increment));
        // Take into account current cycle remaining ticks
        Tick rem_ticks = _period - (curTick() % _period);
        if (rem_ticks < _period) {
            when += rem_ticks;
            num_cycles -= 1;
        }
        when += num_cycles * _period;
    }
    return when;
}

Tick
SystemCounter::whenValue(uint64_t target_val)
{
    return whenValue(value(), target_val);
}

void
SystemCounter::updateTick()
{
    _updateTick = curTick() - (curTick() % _period);
}

void
SystemCounter::freqUpdateSchedule(size_t new_freq_entry)
{
    if (new_freq_entry < _freqTable.size()) {
        auto &new_freq = _freqTable[new_freq_entry];
        if (new_freq != _freq) {
            _nextFreqEntry = new_freq_entry;
            // Wait until the value for which the lowest frequency increment
            // is a exact divisor. This covers both high to low and low to
            // high transitions
            uint64_t new_incr = _freqTable[0] / new_freq;
            uint64_t target_val = value();
            target_val += target_val % std::max(_increment, new_incr);
            reschedule(_freqUpdateEvent, whenValue(target_val), true);
        }
    }
}

void
SystemCounter::freqUpdateCallback()
{
    DPRINTF(Timer, "SystemCounter::freqUpdateCallback: Changing counter "
            "frequency\n");
    if (_enabled)
        updateValue();
    _activeFreqEntry = _nextFreqEntry;
    _freq = _freqTable[_activeFreqEntry];
    _increment = _freqTable[0] / _freq;
    _period = (1.0 / _freq) * SimClock::Frequency;
    notifyListeners();
}

void
SystemCounter::registerListener(SystemCounterListener *listener)
{
    _listeners.push_back(listener);
}

void
SystemCounter::notifyListeners() const
{
    for (auto &listener : _listeners)
        listener->notify();
}

void
SystemCounter::serialize(CheckpointOut &cp) const
{
    DPRINTF(Timer, "SystemCounter::serialize: Serializing\n");
    SERIALIZE_SCALAR(_enabled);
    SERIALIZE_SCALAR(_freq);
    SERIALIZE_SCALAR(_value);
    SERIALIZE_SCALAR(_increment);
    SERIALIZE_CONTAINER(_freqTable);
    SERIALIZE_SCALAR(_activeFreqEntry);
    SERIALIZE_SCALAR(_updateTick);
    bool pending_freq_update = _freqUpdateEvent.scheduled();
    SERIALIZE_SCALAR(pending_freq_update);
    if (pending_freq_update) {
        Tick when_freq_update = _freqUpdateEvent.when();
        SERIALIZE_SCALAR(when_freq_update);
    }
    SERIALIZE_SCALAR(_nextFreqEntry);
}

void
SystemCounter::unserialize(CheckpointIn &cp)
{
    DPRINTF(Timer, "SystemCounter::unserialize: Unserializing\n");
    UNSERIALIZE_SCALAR(_enabled);
    UNSERIALIZE_SCALAR(_freq);
    UNSERIALIZE_SCALAR(_value);
    UNSERIALIZE_SCALAR(_increment);
    UNSERIALIZE_CONTAINER(_freqTable);
    UNSERIALIZE_SCALAR(_activeFreqEntry);
    UNSERIALIZE_SCALAR(_updateTick);
    bool pending_freq_update;
    UNSERIALIZE_SCALAR(pending_freq_update);
    if (pending_freq_update) {
        Tick when_freq_update;
        UNSERIALIZE_SCALAR(when_freq_update);
        reschedule(_freqUpdateEvent, when_freq_update, true);
    }
    UNSERIALIZE_SCALAR(_nextFreqEntry);

    _period = (1.0 / _freq) * SimClock::Frequency;
}

ArchTimer::ArchTimer(const std::string &name,
                     SimObject &parent,
                     SystemCounter &sysctr,
                     ArmInterruptPin *interrupt)
    : _name(name), _parent(parent), _systemCounter(sysctr),
      _interrupt(interrupt),
      _control(0), _counterLimit(0), _offset(0),
      _counterLimitReachedEvent([this]{ counterLimitReached(); }, name)
{
    _systemCounter.registerListener(this);
}

void
ArchTimer::counterLimitReached()
{
    if (!_control.enable)
        return;

    DPRINTF(Timer, "Counter limit reached\n");
    _control.istatus = 1;
    if (!_control.imask) {
        if (scheduleEvents()) {
            DPRINTF(Timer, "Causing interrupt\n");
            _interrupt->raise();
        } else {
            DPRINTF(Timer, "Kvm mode; skipping simulated interrupt\n");
        }
    }
}

void
ArchTimer::updateCounter()
{
    if (_counterLimitReachedEvent.scheduled())
        _parent.deschedule(_counterLimitReachedEvent);
    if (value() >= _counterLimit) {
        counterLimitReached();
    } else {
        // Clear the interurpt when timers conditions are not met
        if (_interrupt->active()) {
            DPRINTF(Timer, "Clearing interrupt\n");
            _interrupt->clear();
        }

        _control.istatus = 0;

        if (scheduleEvents()) {
            _parent.schedule(_counterLimitReachedEvent,
                             whenValue(_counterLimit));
        }
    }
}

void
ArchTimer::setCompareValue(uint64_t val)
{
    _counterLimit = val;
    updateCounter();
}

void
ArchTimer::setTimerValue(uint32_t val)
{
    setCompareValue(value() + sext<32>(val));
}

void
ArchTimer::setControl(uint32_t val)
{
    ArchTimerCtrl old_ctl = _control, new_ctl = val;
    _control.enable = new_ctl.enable;
    _control.imask = new_ctl.imask;
    _control.istatus = old_ctl.istatus;
    // Timer unmasked or enabled
    if ((old_ctl.imask && !new_ctl.imask) ||
        (!old_ctl.enable && new_ctl.enable))
        updateCounter();
    // Timer masked or disabled
    else if ((!old_ctl.imask && new_ctl.imask) ||
             (old_ctl.enable && !new_ctl.enable)) {

        if (_interrupt->active()) {
            DPRINTF(Timer, "Clearing interrupt\n");
            // We are clearing the interrupt but we are not
            // setting istatus to 0 as we are doing
            // in the updateCounter.
            // istatus signals that Timer conditions are met.
            // It shouldn't depend on masking.
            // if enable is zero. istatus is unknown.
            _interrupt->clear();
        }
    }
}

void
ArchTimer::setOffset(uint64_t val)
{
    _offset = val;
    updateCounter();
}

uint64_t
ArchTimer::value() const
{
    return _systemCounter.value() - _offset;
}

void
ArchTimer::notify()
{
    updateCounter();
}

void
ArchTimer::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "control_serial", _control);
    SERIALIZE_SCALAR(_counterLimit);
    SERIALIZE_SCALAR(_offset);
}

void
ArchTimer::unserialize(CheckpointIn &cp)
{
    paramIn(cp, "control_serial", _control);
    // We didn't serialize an offset before we added support for the
    // virtual timer. Consider it optional to maintain backwards
    // compatibility.
    if (!UNSERIALIZE_OPT_SCALAR(_offset))
        _offset = 0;

    // We no longer schedule an event here because we may enter KVM
    // emulation.  The event creation is delayed until drainResume().
}

DrainState
ArchTimer::drain()
{
    if (_counterLimitReachedEvent.scheduled())
        _parent.deschedule(_counterLimitReachedEvent);

    return DrainState::Drained;
}

void
ArchTimer::drainResume()
{
    updateCounter();
}

GenericTimer::GenericTimer(GenericTimerParams *const p)
    : SimObject(p),
      systemCounter(*p->counter),
      system(*p->system)
{
    SystemCounter::validateCounterRef(p->counter);
    fatal_if(!p->system, "GenericTimer::GenericTimer: No system specified, "
             "can't instantiate architected timers\n");
    system.setGenericTimer(this);
}

const GenericTimerParams *
GenericTimer::params() const
{
    return dynamic_cast<const GenericTimerParams *>(_params);
}

void
GenericTimer::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "cpu_count", timers.size());

    for (int i = 0; i < timers.size(); ++i) {
        const CoreTimers &core(*timers[i]);
        core.serializeSection(cp, csprintf("pe_implementation%d", i));
    }
}

void
GenericTimer::unserialize(CheckpointIn &cp)
{
    // Try to unserialize the CPU count. Old versions of the timer
    // model assumed a 8 CPUs, so we fall back to that if the field
    // isn't present.
    static const unsigned OLD_CPU_MAX = 8;
    unsigned cpu_count;
    if (!UNSERIALIZE_OPT_SCALAR(cpu_count)) {
        warn("Checkpoint does not contain CPU count, assuming %i CPUs\n",
             OLD_CPU_MAX);
        cpu_count = OLD_CPU_MAX;
    }

    // We cannot assert for equality here because CPU timers are dynamically
    // created on the first miscreg access. Therefore, if we take the checkpoint
    // before any timer registers have been accessed, the number of counters
    // is actually smaller than the total number of CPUs.
    if (cpu_count > system.threads.size()) {
        fatal("The simulated system has been initialized with %d CPUs, "
              "but the Generic Timer checkpoint expects %d CPUs. Consider "
              "restoring the checkpoint specifying %d CPUs.",
              system.threads.size(), cpu_count, cpu_count);
    }

    for (int i = 0; i < cpu_count; ++i) {
        CoreTimers &core(getTimers(i));
        core.unserializeSection(cp, csprintf("pe_implementation%d", i));
    }
}

GenericTimer::CoreTimers &
GenericTimer::getTimers(int cpu_id)
{
    if (cpu_id >= timers.size())
        createTimers(cpu_id + 1);

    return *timers[cpu_id];
}

void
GenericTimer::createTimers(unsigned cpus)
{
    assert(timers.size() < cpus);
    auto p = static_cast<const GenericTimerParams *>(_params);

    const unsigned old_cpu_count(timers.size());
    timers.resize(cpus);
    for (unsigned i = old_cpu_count; i < cpus; ++i) {

        ThreadContext *tc = system.threads[i];

        timers[i].reset(
            new CoreTimers(*this, system, i,
                           p->int_phys_s->get(tc),
                           p->int_phys_ns->get(tc),
                           p->int_virt->get(tc),
                           p->int_hyp->get(tc)));
    }
}

void
GenericTimer::handleStream(CoreTimers::EventStream *ev_stream,
    ArchTimer *timer, RegVal old_cnt_ctl, RegVal cnt_ctl)
{
    uint64_t evnten = bits(cnt_ctl, 2);
    uint64_t old_evnten = bits(old_cnt_ctl, 2);
    uint8_t old_trans_to = ev_stream->transitionTo;
    uint8_t old_trans_bit = ev_stream->transitionBit;
    ev_stream->transitionTo = !bits(cnt_ctl, 3);
    ev_stream->transitionBit = bits(cnt_ctl, 7, 4);
    // Reschedule the Event Stream if enabled and any change in
    // configuration
    if (evnten && ((old_evnten != evnten) ||
        (old_trans_to != ev_stream->transitionTo) ||
        (old_trans_bit != ev_stream->transitionBit))) {

        Tick when = timer->whenValue(
            ev_stream->eventTargetValue(timer->value()));
        reschedule(ev_stream->event, when, true);
    } else if (old_evnten && !evnten) {
        // Event Stream generation disabled
        if (ev_stream->event.scheduled())
            deschedule(ev_stream->event);
    }
}

void
GenericTimer::setMiscReg(int reg, unsigned cpu, RegVal val)
{
    CoreTimers &core(getTimers(cpu));
    ThreadContext *tc = system.threads[cpu];

    switch (reg) {
      case MISCREG_CNTFRQ:
      case MISCREG_CNTFRQ_EL0:
        core.cntfrq = val;
        warn_if(core.cntfrq != systemCounter.freq(), "CNTFRQ configured freq "
                "does not match the system counter freq\n");
        return;
      case MISCREG_CNTKCTL:
      case MISCREG_CNTKCTL_EL1:
      {
        if (ELIsInHost(tc, currEL(tc))) {
            tc->setMiscReg(MISCREG_CNTHCTL_EL2, val);
            return;
        }
        RegVal old_cnt_ctl = core.cntkctl;
        core.cntkctl = val;

        ArchTimer *timer = &core.virt;
        CoreTimers::EventStream *ev_stream = &core.virtEvStream;

        handleStream(ev_stream, timer, old_cnt_ctl, val);
        return;
      }
      case MISCREG_CNTHCTL:
      case MISCREG_CNTHCTL_EL2:
      {
        RegVal old_cnt_ctl = core.cnthctl;
        core.cnthctl = val;

        ArchTimer *timer = &core.physNS;
        CoreTimers::EventStream *ev_stream = &core.physEvStream;

        handleStream(ev_stream, timer, old_cnt_ctl, val);
        return;
      }
      // Physical timer (NS)
      case MISCREG_CNTP_CVAL_NS:
      case MISCREG_CNTP_CVAL_EL0:
        core.physNS.setCompareValue(val);
        return;

      case MISCREG_CNTP_TVAL_NS:
      case MISCREG_CNTP_TVAL_EL0:
        core.physNS.setTimerValue(val);
        return;

      case MISCREG_CNTP_CTL_NS:
      case MISCREG_CNTP_CTL_EL0:
        core.physNS.setControl(val);
        return;

      // Count registers
      case MISCREG_CNTPCT:
      case MISCREG_CNTPCT_EL0:
      case MISCREG_CNTVCT:
      case MISCREG_CNTVCT_EL0:
        warn("Ignoring write to read only count register: %s\n",
             miscRegName[reg]);
        return;

      // Virtual timer
      case MISCREG_CNTVOFF:
      case MISCREG_CNTVOFF_EL2:
        core.virt.setOffset(val);
        return;

      case MISCREG_CNTV_CVAL:
      case MISCREG_CNTV_CVAL_EL0:
        core.virt.setCompareValue(val);
        return;

      case MISCREG_CNTV_TVAL:
      case MISCREG_CNTV_TVAL_EL0:
        core.virt.setTimerValue(val);
        return;

      case MISCREG_CNTV_CTL:
      case MISCREG_CNTV_CTL_EL0:
        core.virt.setControl(val);
        return;

      // Physical timer (S)
      case MISCREG_CNTP_CTL_S:
      case MISCREG_CNTPS_CTL_EL1:
        core.physS.setControl(val);
        return;

      case MISCREG_CNTP_CVAL_S:
      case MISCREG_CNTPS_CVAL_EL1:
        core.physS.setCompareValue(val);
        return;

      case MISCREG_CNTP_TVAL_S:
      case MISCREG_CNTPS_TVAL_EL1:
        core.physS.setTimerValue(val);
        return;

      // Hyp phys. timer, non-secure
      case MISCREG_CNTHP_CTL:
      case MISCREG_CNTHP_CTL_EL2:
        core.hyp.setControl(val);
        return;

      case MISCREG_CNTHP_CVAL:
      case MISCREG_CNTHP_CVAL_EL2:
        core.hyp.setCompareValue(val);
        return;

      case MISCREG_CNTHP_TVAL:
      case MISCREG_CNTHP_TVAL_EL2:
        core.hyp.setTimerValue(val);
        return;

      default:
        warn("Writing to unknown register: %s\n", miscRegName[reg]);
        return;
    }
}


RegVal
GenericTimer::readMiscReg(int reg, unsigned cpu)
{
    CoreTimers &core(getTimers(cpu));

    switch (reg) {
      case MISCREG_CNTFRQ:
      case MISCREG_CNTFRQ_EL0:
        return core.cntfrq;
      case MISCREG_CNTKCTL:
      case MISCREG_CNTKCTL_EL1:
        return core.cntkctl & 0x00000000ffffffff;
      case MISCREG_CNTHCTL:
      case MISCREG_CNTHCTL_EL2:
        return core.cnthctl & 0x00000000ffffffff;
      // Physical timer
      case MISCREG_CNTP_CVAL_NS:
      case MISCREG_CNTP_CVAL_EL0:
        return core.physNS.compareValue();

      case MISCREG_CNTP_TVAL_NS:
      case MISCREG_CNTP_TVAL_EL0:
        return core.physNS.timerValue();

      case MISCREG_CNTP_CTL_EL0:
      case MISCREG_CNTP_CTL_NS:
        return core.physNS.control();

      case MISCREG_CNTPCT:
      case MISCREG_CNTPCT_EL0:
        return core.physNS.value();


      // Virtual timer
      case MISCREG_CNTVCT:
      case MISCREG_CNTVCT_EL0:
        return core.virt.value();

      case MISCREG_CNTVOFF:
      case MISCREG_CNTVOFF_EL2:
        return core.virt.offset();

      case MISCREG_CNTV_CVAL:
      case MISCREG_CNTV_CVAL_EL0:
        return core.virt.compareValue();

      case MISCREG_CNTV_TVAL:
      case MISCREG_CNTV_TVAL_EL0:
        return core.virt.timerValue();

      case MISCREG_CNTV_CTL:
      case MISCREG_CNTV_CTL_EL0:
        return core.virt.control();

      // PL1 phys. timer, secure
      case MISCREG_CNTP_CTL_S:
      case MISCREG_CNTPS_CTL_EL1:
        return core.physS.control();

      case MISCREG_CNTP_CVAL_S:
      case MISCREG_CNTPS_CVAL_EL1:
        return core.physS.compareValue();

      case MISCREG_CNTP_TVAL_S:
      case MISCREG_CNTPS_TVAL_EL1:
        return core.physS.timerValue();

      // HYP phys. timer (NS)
      case MISCREG_CNTHP_CTL:
      case MISCREG_CNTHP_CTL_EL2:
        return core.hyp.control();

      case MISCREG_CNTHP_CVAL:
      case MISCREG_CNTHP_CVAL_EL2:
        return core.hyp.compareValue();

      case MISCREG_CNTHP_TVAL:
      case MISCREG_CNTHP_TVAL_EL2:
        return core.hyp.timerValue();

      default:
        warn("Reading from unknown register: %s\n", miscRegName[reg]);
        return 0;
    }
}

GenericTimer::CoreTimers::CoreTimers(GenericTimer &_parent,
    ArmSystem &system, unsigned cpu,
    ArmInterruptPin *_irqPhysS, ArmInterruptPin *_irqPhysNS,
    ArmInterruptPin *_irqVirt, ArmInterruptPin *_irqHyp)
      : parent(_parent),
        cntfrq(parent.params()->cntfrq),
        threadContext(system.threads[cpu]),
        irqPhysS(_irqPhysS),
        irqPhysNS(_irqPhysNS),
        irqVirt(_irqVirt),
        irqHyp(_irqHyp),
        physS(csprintf("%s.phys_s_timer%d", parent.name(), cpu),
              system, parent, parent.systemCounter,
              _irqPhysS),
        // This should really be phys_timerN, but we are stuck with
        // arch_timer for backwards compatibility.
        physNS(csprintf("%s.arch_timer%d", parent.name(), cpu),
             system, parent, parent.systemCounter,
             _irqPhysNS),
        virt(csprintf("%s.virt_timer%d", parent.name(), cpu),
           system, parent, parent.systemCounter,
           _irqVirt),
        hyp(csprintf("%s.hyp_timer%d", parent.name(), cpu),
           system, parent, parent.systemCounter,
           _irqHyp),
        physEvStream{
           EventFunctionWrapper([this]{ physEventStreamCallback(); },
           csprintf("%s.phys_event_gen%d", parent.name(), cpu)), 0, 0
        },
        virtEvStream{
           EventFunctionWrapper([this]{ virtEventStreamCallback(); },
           csprintf("%s.virt_event_gen%d", parent.name(), cpu)), 0, 0
        }
{
}

void
GenericTimer::CoreTimers::physEventStreamCallback()
{
    eventStreamCallback();
    schedNextEvent(physEvStream, physNS);
}

void
GenericTimer::CoreTimers::virtEventStreamCallback()
{
    eventStreamCallback();
    schedNextEvent(virtEvStream, virt);
}

void
GenericTimer::CoreTimers::eventStreamCallback() const
{
    sendEvent(threadContext);
    threadContext->getCpuPtr()->wakeup(threadContext->threadId());
}

void
GenericTimer::CoreTimers::schedNextEvent(EventStream &ev_stream,
                                         ArchTimer &timer)
{
    parent.reschedule(ev_stream.event, timer.whenValue(
        ev_stream.eventTargetValue(timer.value())), true);
}

void
GenericTimer::CoreTimers::notify()
{
    schedNextEvent(virtEvStream, virt);
    schedNextEvent(physEvStream, physNS);
}

void
GenericTimer::CoreTimers::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(cntfrq);
    SERIALIZE_SCALAR(cntkctl);
    SERIALIZE_SCALAR(cnthctl);

    const bool phys_ev_scheduled = physEvStream.event.scheduled();
    SERIALIZE_SCALAR(phys_ev_scheduled);
    if (phys_ev_scheduled) {
        const Tick phys_ev_when = physEvStream.event.when();
        SERIALIZE_SCALAR(phys_ev_when);
    }
    SERIALIZE_SCALAR(physEvStream.transitionTo);
    SERIALIZE_SCALAR(physEvStream.transitionBit);

    const bool virt_ev_scheduled = virtEvStream.event.scheduled();
    SERIALIZE_SCALAR(virt_ev_scheduled);
    if (virt_ev_scheduled) {
        const Tick virt_ev_when = virtEvStream.event.when();
        SERIALIZE_SCALAR(virt_ev_when);
    }
    SERIALIZE_SCALAR(virtEvStream.transitionTo);
    SERIALIZE_SCALAR(virtEvStream.transitionBit);

    physS.serializeSection(cp, "phys_s_timer");
    physNS.serializeSection(cp, "phys_ns_timer");
    virt.serializeSection(cp, "virt_timer");
    hyp.serializeSection(cp, "hyp_timer");
}

void
GenericTimer::CoreTimers::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(cntfrq);
    UNSERIALIZE_SCALAR(cntkctl);
    UNSERIALIZE_SCALAR(cnthctl);

    bool phys_ev_scheduled;
    UNSERIALIZE_SCALAR(phys_ev_scheduled);
    if (phys_ev_scheduled) {
        Tick phys_ev_when;
        UNSERIALIZE_SCALAR(phys_ev_when);
        parent.reschedule(physEvStream.event, phys_ev_when, true);
    }
    UNSERIALIZE_SCALAR(physEvStream.transitionTo);
    UNSERIALIZE_SCALAR(physEvStream.transitionBit);

    bool virt_ev_scheduled;
    UNSERIALIZE_SCALAR(virt_ev_scheduled);
    if (virt_ev_scheduled) {
        Tick virt_ev_when;
        UNSERIALIZE_SCALAR(virt_ev_when);
        parent.reschedule(virtEvStream.event, virt_ev_when, true);
    }
    UNSERIALIZE_SCALAR(virtEvStream.transitionTo);
    UNSERIALIZE_SCALAR(virtEvStream.transitionBit);

    physS.unserializeSection(cp, "phys_s_timer");
    physNS.unserializeSection(cp, "phys_ns_timer");
    virt.unserializeSection(cp, "virt_timer");
    hyp.unserializeSection(cp, "hyp_timer");
}

void
GenericTimerISA::setMiscReg(int reg, RegVal val)
{
    DPRINTF(Timer, "Setting %s := 0x%x\n", miscRegName[reg], val);
    parent.setMiscReg(reg, cpu, val);
}

RegVal
GenericTimerISA::readMiscReg(int reg)
{
    RegVal value = parent.readMiscReg(reg, cpu);
    DPRINTF(Timer, "Reading %s as 0x%x\n", miscRegName[reg], value);
    return value;
}

GenericTimerFrame::GenericTimerFrame(GenericTimerFrameParams *const p)
    : PioDevice(p),
      timerRange(RangeSize(p->cnt_base, ArmSystem::PageBytes)),
      addrRanges({timerRange}),
      systemCounter(*p->counter),
      physTimer(csprintf("%s.phys_timer", name()),
                *this, systemCounter, p->int_phys->get()),
      virtTimer(csprintf("%s.virt_timer", name()),
                *this, systemCounter,
                p->int_virt->get()),
      accessBits(0x3f),
      system(*dynamic_cast<ArmSystem *>(sys))
{
    SystemCounter::validateCounterRef(p->counter);
    // Expose optional CNTEL0Base register frame
    if (p->cnt_el0_base != MaxAddr) {
        timerEl0Range = RangeSize(p->cnt_el0_base, ArmSystem::PageBytes);
        accessBitsEl0 = 0x303;
        addrRanges.push_back(timerEl0Range);
    }
    for (auto &range : addrRanges)
        GenericTimerMem::validateFrameRange(range);
}

void
GenericTimerFrame::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(accessBits);
    if (hasEl0View())
        SERIALIZE_SCALAR(accessBitsEl0);
    SERIALIZE_SCALAR(nonSecureAccess);

    physTimer.serializeSection(cp, "phys_timer");
    virtTimer.serializeSection(cp, "virt_timer");
}

void
GenericTimerFrame::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(accessBits);
    if (hasEl0View())
        UNSERIALIZE_SCALAR(accessBitsEl0);
    UNSERIALIZE_SCALAR(nonSecureAccess);

    physTimer.unserializeSection(cp, "phys_timer");
    virtTimer.unserializeSection(cp, "virt_timer");
}

uint64_t
GenericTimerFrame::getVirtOffset() const
{
    return virtTimer.offset();
}

void
GenericTimerFrame::setVirtOffset(uint64_t new_offset)
{
    virtTimer.setOffset(new_offset);
}

bool
GenericTimerFrame::hasEl0View() const
{
    return timerEl0Range.valid();
}

uint8_t
GenericTimerFrame::getAccessBits() const
{
    return accessBits;
}

void
GenericTimerFrame::setAccessBits(uint8_t data)
{
    accessBits = data & 0x3f;
}

bool
GenericTimerFrame::hasNonSecureAccess() const
{
    return nonSecureAccess;
}

void
GenericTimerFrame::setNonSecureAccess()
{
    nonSecureAccess = true;
}

bool
GenericTimerFrame::hasReadableVoff() const
{
    return accessBits.rvoff;
}

AddrRangeList
GenericTimerFrame::getAddrRanges() const
{
    return addrRanges;
}

Tick
GenericTimerFrame::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const size_t size = pkt->getSize();
    const bool is_sec = pkt->isSecure();
    panic_if(size != 4 && size != 8,
             "GenericTimerFrame::read: Invalid size %i\n", size);

    bool to_el0 = false;
    uint64_t resp = 0;
    Addr offset = 0;
    if (timerRange.contains(addr)) {
        offset = addr - timerRange.start();
    } else if (hasEl0View() && timerEl0Range.contains(addr)) {
        offset = addr - timerEl0Range.start();
        to_el0 = true;
    } else {
        panic("GenericTimerFrame::read: Invalid address: 0x%x\n", addr);
    }

    resp = timerRead(offset, size, is_sec, to_el0);

    DPRINTF(Timer, "GenericTimerFrame::read: 0x%x<-0x%x(%i) [S = %u]\n", resp,
            addr, size, is_sec);

    pkt->setUintX(resp, ByteOrder::little);
    pkt->makeResponse();
    return 0;
}

Tick
GenericTimerFrame::write(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const size_t size = pkt->getSize();
    const bool is_sec = pkt->isSecure();
    panic_if(size != 4 && size != 8,
             "GenericTimerFrame::write: Invalid size %i\n", size);

    bool to_el0 = false;
    const uint64_t data = pkt->getUintX(ByteOrder::little);
    Addr offset = 0;
    if (timerRange.contains(addr)) {
        offset = addr - timerRange.start();
    } else if (hasEl0View() && timerEl0Range.contains(addr)) {
        offset = addr - timerEl0Range.start();
        to_el0 = true;
    } else {
        panic("GenericTimerFrame::write: Invalid address: 0x%x\n", addr);
    }

    timerWrite(offset, size, data, is_sec, to_el0);

    DPRINTF(Timer, "GenericTimerFrame::write: 0x%x->0x%x(%i) [S = %u]\n", data,
            addr, size, is_sec);

    pkt->makeResponse();
    return 0;
}

uint64_t
GenericTimerFrame::timerRead(Addr addr, size_t size, bool is_sec,
                             bool to_el0) const
{
    if (!GenericTimerMem::validateAccessPerm(system, is_sec) &&
        !nonSecureAccess)
        return 0;

    switch (addr) {
      case TIMER_CNTPCT_LO:
        if (!accessBits.rpct || (to_el0 && !accessBitsEl0.pcten))
            return 0;
        else
            return physTimer.value();

      case TIMER_CNTPCT_HI:
        if (!accessBits.rpct || (to_el0 && !accessBitsEl0.pcten))
            return 0;
        else
            return physTimer.value() >> 32;

      case TIMER_CNTFRQ:
        if ((!accessBits.rfrq) ||
            (to_el0 && (!accessBitsEl0.pcten && !accessBitsEl0.vcten)))
            return 0;
        else
            return systemCounter.freq();

      case TIMER_CNTEL0ACR:
        if (!hasEl0View() || to_el0)
            return 0;
        else
            return accessBitsEl0;

      case TIMER_CNTP_CVAL_LO:
        if (!accessBits.rwpt || (to_el0 && !accessBitsEl0.pten))
            return 0;
        else
            return physTimer.compareValue();

      case TIMER_CNTP_CVAL_HI:
        if (!accessBits.rwpt || (to_el0 && !accessBitsEl0.pten))
            return 0;
        else
            return physTimer.compareValue() >> 32;

      case TIMER_CNTP_TVAL:
        if (!accessBits.rwpt || (to_el0 && !accessBitsEl0.pten))
            return 0;
        else
            return physTimer.timerValue();
      case TIMER_CNTP_CTL:
        if (!accessBits.rwpt || (to_el0 && !accessBitsEl0.pten))
            return 0;
        else
            return physTimer.control();

      case TIMER_CNTVCT_LO:
        if (!accessBits.rvct || (to_el0 && !accessBitsEl0.vcten))
            return 0;
        else
            return virtTimer.value();

      case TIMER_CNTVCT_HI:
        if (!accessBits.rvct || (to_el0 && !accessBitsEl0.vcten))
            return 0;
        else
            return virtTimer.value() >> 32;

      case TIMER_CNTVOFF_LO:
        if (!accessBits.rvoff || (to_el0))
            return 0;
        else
            return virtTimer.offset();

      case TIMER_CNTVOFF_HI:
        if (!accessBits.rvoff || (to_el0))
            return 0;
        else
            return virtTimer.offset() >> 32;

      case TIMER_CNTV_CVAL_LO:
        if (!accessBits.rwvt || (to_el0 && !accessBitsEl0.vten))
            return 0;
        else
            return virtTimer.compareValue();

      case TIMER_CNTV_CVAL_HI:
        if (!accessBits.rwvt || (to_el0 && !accessBitsEl0.vten))
            return 0;
        else
            return virtTimer.compareValue() >> 32;

      case TIMER_CNTV_TVAL:
        if (!accessBits.rwvt || (to_el0 && !accessBitsEl0.vten))
            return 0;
        else
            return virtTimer.timerValue();

      case TIMER_CNTV_CTL:
        if (!accessBits.rwvt || (to_el0 && !accessBitsEl0.vten))
            return 0;
        else
            return virtTimer.control();

      default:
        warn("GenericTimerFrame::timerRead: Unexpected address (0x%x:%i), "
             "assuming RAZ\n", addr, size);
        return 0;
    }
}

void
GenericTimerFrame::timerWrite(Addr addr, size_t size, uint64_t data,
                              bool is_sec, bool to_el0)
{
    if (!GenericTimerMem::validateAccessPerm(system, is_sec) &&
        !nonSecureAccess)
        return;

    switch (addr) {
      case TIMER_CNTPCT_LO ... TIMER_CNTPCT_HI:
        warn("GenericTimerFrame::timerWrite: RO reg (0x%x) [CNTPCT]\n",
             addr);
        return;

      case TIMER_CNTFRQ:
        warn("GenericTimerFrame::timerWrite: RO reg (0x%x) [CNTFRQ]\n",
             addr);
        return;

      case TIMER_CNTEL0ACR:
        if (!hasEl0View() || to_el0)
            return;

        insertBits(accessBitsEl0, 9, 8, data);
        insertBits(accessBitsEl0, 1, 0, data);
        return;

      case TIMER_CNTP_CVAL_LO:
        if ((!accessBits.rwpt) || (to_el0 && !accessBitsEl0.pten))
            return;
        data = size == 4 ? insertBits(physTimer.compareValue(),
                                      31, 0, data) : data;
        physTimer.setCompareValue(data);
        return;

      case TIMER_CNTP_CVAL_HI:
        if ((!accessBits.rwpt) || (to_el0 && !accessBitsEl0.pten))
            return;
        data = insertBits(physTimer.compareValue(), 63, 32, data);
        physTimer.setCompareValue(data);
        return;

      case TIMER_CNTP_TVAL:
        if ((!accessBits.rwpt) || (to_el0 && !accessBitsEl0.pten))
            return;
        physTimer.setTimerValue(data);
        return;

      case TIMER_CNTP_CTL:
        if ((!accessBits.rwpt) || (to_el0 && !accessBitsEl0.pten))
            return;
        physTimer.setControl(data);
        return;

      case TIMER_CNTVCT_LO ... TIMER_CNTVCT_HI:
        warn("GenericTimerFrame::timerWrite: RO reg (0x%x) [CNTVCT]\n",
             addr);
        return;
      case TIMER_CNTVOFF_LO ... TIMER_CNTVOFF_HI:
        warn("GenericTimerFrame::timerWrite: RO reg (0x%x) [CNTVOFF]\n",
             addr);
        return;

      case TIMER_CNTV_CVAL_LO:
        if ((!accessBits.rwvt) || (to_el0 && !accessBitsEl0.vten))
            return;
        data = size == 4 ? insertBits(virtTimer.compareValue(),
                                      31, 0, data) : data;
        virtTimer.setCompareValue(data);
        return;

      case TIMER_CNTV_CVAL_HI:
        if ((!accessBits.rwvt) || (to_el0 && !accessBitsEl0.vten))
            return;
        data = insertBits(virtTimer.compareValue(), 63, 32, data);
        virtTimer.setCompareValue(data);
        return;

      case TIMER_CNTV_TVAL:
        if ((!accessBits.rwvt) || (to_el0 && !accessBitsEl0.vten))
            return;
        virtTimer.setTimerValue(data);
        return;

      case TIMER_CNTV_CTL:
        if ((!accessBits.rwvt) || (to_el0 && !accessBitsEl0.vten))
            return;
        virtTimer.setControl(data);
        return;

      default:
        warn("GenericTimerFrame::timerWrite: Unexpected address (0x%x:%i), "
             "assuming WI\n", addr, size);
    }
}

GenericTimerMem::GenericTimerMem(GenericTimerMemParams *const p)
    : PioDevice(p),
      counterCtrlRange(RangeSize(p->cnt_control_base, ArmSystem::PageBytes)),
      counterStatusRange(RangeSize(p->cnt_read_base, ArmSystem::PageBytes)),
      timerCtrlRange(RangeSize(p->cnt_ctl_base, ArmSystem::PageBytes)),
      cnttidr(0x0),
      addrRanges{counterCtrlRange, counterStatusRange, timerCtrlRange},
      systemCounter(*p->counter),
      frames(p->frames),
      system(*dynamic_cast<ArmSystem *>(sys))
{
    SystemCounter::validateCounterRef(p->counter);
    for (auto &range : addrRanges)
        GenericTimerMem::validateFrameRange(range);
    fatal_if(frames.size() > MAX_TIMER_FRAMES,
        "GenericTimerMem::GenericTimerMem: Architecture states a maximum of "
        "8 memory-mapped timer frames, limit surpassed\n");
    // Initialize CNTTIDR with each frame's features
    for (int i = 0; i < frames.size(); i++) {
        uint32_t features = 0x1;
        features |= 0x2;
        if (frames[i]->hasEl0View())
            features |= 0x4;
        features <<= i * 4;
        replaceBits(cnttidr, (i + 1) * 4 - 1, i * 4, features);
    }
}

void
GenericTimerMem::validateFrameRange(const AddrRange &range)
{
    fatal_if(range.start() % ArmSystem::PageBytes,
             "GenericTimerMem::validateFrameRange: Architecture states each "
             "register frame should be in a separate memory page, specified "
             "range base address [0x%x] is not compliant\n");
}

bool
GenericTimerMem::validateAccessPerm(ArmSystem &sys, bool is_sec)
{
    return !sys.haveSecurity() || is_sec;
}

AddrRangeList
GenericTimerMem::getAddrRanges() const
{
    return addrRanges;
}

Tick
GenericTimerMem::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const size_t size = pkt->getSize();
    const bool is_sec = pkt->isSecure();
    panic_if(size != 4 && size != 8,
             "GenericTimerMem::read: Invalid size %i\n", size);

    uint64_t resp = 0;
    if (counterCtrlRange.contains(addr))
        resp = counterCtrlRead(addr - counterCtrlRange.start(), size, is_sec);
    else if (counterStatusRange.contains(addr))
        resp = counterStatusRead(addr - counterStatusRange.start(), size);
    else if (timerCtrlRange.contains(addr))
        resp = timerCtrlRead(addr - timerCtrlRange.start(), size, is_sec);
    else
        panic("GenericTimerMem::read: Invalid address: 0x%x\n", addr);

    DPRINTF(Timer, "GenericTimerMem::read: 0x%x<-0x%x(%i) [S = %u]\n", resp,
            addr, size, is_sec);

    pkt->setUintX(resp, ByteOrder::little);
    pkt->makeResponse();
    return 0;
}

Tick
GenericTimerMem::write(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const size_t size = pkt->getSize();
    const bool is_sec = pkt->isSecure();
    panic_if(size != 4 && size != 8,
             "GenericTimerMem::write: Invalid size %i\n", size);

    const uint64_t data = pkt->getUintX(ByteOrder::little);
    if (counterCtrlRange.contains(addr))
        counterCtrlWrite(addr - counterCtrlRange.start(), size, data, is_sec);
    else if (counterStatusRange.contains(addr))
        counterStatusWrite(addr - counterStatusRange.start(), size, data);
    else if (timerCtrlRange.contains(addr))
        timerCtrlWrite(addr - timerCtrlRange.start(), size, data, is_sec);
    else
        panic("GenericTimerMem::write: Invalid address: 0x%x\n", addr);

    DPRINTF(Timer, "GenericTimerMem::write: 0x%x->0x%x(%i) [S = %u]\n", data,
            addr, size, is_sec);

    pkt->makeResponse();
    return 0;
}

uint64_t
GenericTimerMem::counterCtrlRead(Addr addr, size_t size, bool is_sec) const
{
    if (!GenericTimerMem::validateAccessPerm(system, is_sec))
        return 0;
    switch (addr) {
      case COUNTER_CTRL_CNTCR:
      {
        CNTCR cntcr = 0;
        cntcr.en = systemCounter.enabled();
        cntcr.fcreq = systemCounter.activeFreqEntry();
        return cntcr;
      }
      case COUNTER_CTRL_CNTSR:
      {
        CNTSR cntsr = 0;
        cntsr.fcack = systemCounter.activeFreqEntry();
        return cntsr;
      }
      case COUNTER_CTRL_CNTCV_LO: return systemCounter.value();
      case COUNTER_CTRL_CNTCV_HI: return systemCounter.value() >> 32;
      case COUNTER_CTRL_CNTSCR: return 0;
      case COUNTER_CTRL_CNTID: return 0;
      default:
      {
        auto &freq_table = systemCounter.freqTable();
        for (int i = 0; i < (freq_table.size() - 1); i++) {
            Addr offset = COUNTER_CTRL_CNTFID + (i * 0x4);
            if (addr == offset)
                return freq_table[i];
        }
        warn("GenericTimerMem::counterCtrlRead: Unexpected address "
             "(0x%x:%i), assuming RAZ\n", addr, size);
        return 0;
      }
    }
}

void
GenericTimerMem::counterCtrlWrite(Addr addr, size_t size, uint64_t data,
                                  bool is_sec)
{
    if (!GenericTimerMem::validateAccessPerm(system, is_sec))
        return;

    switch (addr) {
      case COUNTER_CTRL_CNTCR:
      {
        CNTCR val = data;
        if (!systemCounter.enabled() && val.en)
            systemCounter.enable();
        else if (systemCounter.enabled() && !val.en)
            systemCounter.disable();

        if (val.hdbg)
            warn("GenericTimerMem::counterCtrlWrite: Halt-on-debug is not "
                 "supported\n");
        if (val.scen)
            warn("GenericTimerMem::counterCtrlWrite: Counter Scaling is not "
                 "supported\n");
        if (val.fcreq != systemCounter.activeFreqEntry())
            systemCounter.freqUpdateSchedule(val.fcreq);
        return;
      }

      case COUNTER_CTRL_CNTSR:
        warn("GenericTimerMem::counterCtrlWrite: RO reg (0x%x) [CNTSR]\n",
             addr);
        return;

      case COUNTER_CTRL_CNTCV_LO:
        data = size == 4 ? insertBits(systemCounter.value(), 31, 0, data)
                         : data;
        systemCounter.setValue(data);
        return;

      case COUNTER_CTRL_CNTCV_HI:
        data = insertBits(systemCounter.value(), 63, 32, data);
        systemCounter.setValue(data);
        return;

      case COUNTER_CTRL_CNTSCR:
        return;

      case COUNTER_CTRL_CNTID:
        warn("GenericTimerMem::counterCtrlWrite: RO reg (0x%x) [CNTID]\n",
             addr);
        return;

      default:
      {
        auto &freq_table = systemCounter.freqTable();
        for (int i = 0; i < (freq_table.size() - 1); i++) {
            Addr offset = COUNTER_CTRL_CNTFID + (i * 0x4);
            if (addr == offset) {
                freq_table[i] = data;
                // This is changing the currently selected frequency
                if (i == systemCounter.activeFreqEntry()) {
                    // We've changed the frequency in the table entry,
                    // however the counter will still work with the
                    // current one until transition is completed
                    systemCounter.freqUpdateSchedule(i);
                }
                return;
            }
        }
        warn("GenericTimerMem::counterCtrlWrite: Unexpected address "
             "(0x%x:%i), assuming WI\n", addr, size);
      }
    }
}

uint64_t
GenericTimerMem::counterStatusRead(Addr addr, size_t size) const
{
    switch (addr) {
      case COUNTER_STATUS_CNTCV_LO: return systemCounter.value();
      case COUNTER_STATUS_CNTCV_HI: return systemCounter.value() >> 32;
      default:
        warn("GenericTimerMem::counterStatusRead: Unexpected address "
             "(0x%x:%i), assuming RAZ\n", addr, size);
        return 0;
    }
}

void
GenericTimerMem::counterStatusWrite(Addr addr, size_t size, uint64_t data)
{
    switch (addr) {
      case COUNTER_STATUS_CNTCV_LO ... COUNTER_STATUS_CNTCV_HI:
        warn("GenericTimerMem::counterStatusWrite: RO reg (0x%x) [CNTCV]\n",
             addr);
        return;
      default:
        warn("GenericTimerMem::counterStatusWrite: Unexpected address "
             "(0x%x:%i), assuming WI\n", addr, size);
    }
}

uint64_t
GenericTimerMem::timerCtrlRead(Addr addr, size_t size, bool is_sec) const
{
    switch (addr) {
      case TIMER_CTRL_CNTFRQ:
        if (!GenericTimerMem::validateAccessPerm(system, is_sec)) return 0;
        return systemCounter.freq();
      case TIMER_CTRL_CNTNSAR:
      {
        if (!GenericTimerMem::validateAccessPerm(system, is_sec)) return 0;
        uint32_t cntnsar = 0x0;
        for (int i = 0; i < frames.size(); i++) {
            if (frames[i]->hasNonSecureAccess())
                cntnsar |= 0x1 << i;
        }
        return cntnsar;
      }
      case TIMER_CTRL_CNTTIDR: return cnttidr;
      default:
        for (int i = 0; i < frames.size(); i++) {
            Addr cntacr_off = TIMER_CTRL_CNTACR + (i * 0x4);
            Addr cntvoff_lo_off = TIMER_CTRL_CNTVOFF_LO + (i * 0x4);
            Addr cntvoff_hi_off = TIMER_CTRL_CNTVOFF_HI + (i * 0x4);
            // CNTNSAR.NS determines if CNTACR/CNTVOFF are accessible from
            // normal world
            bool hit = addr == cntacr_off || addr == cntvoff_lo_off ||
                       addr == cntvoff_hi_off;
            bool has_access =
                GenericTimerMem::validateAccessPerm(system, is_sec) ||
                frames[i]->hasNonSecureAccess();
            if (hit && !has_access) return 0;
            if (addr == cntacr_off)
                return frames[i]->getAccessBits();
            if (addr == cntvoff_lo_off || addr == cntvoff_hi_off) {
                return addr == cntvoff_lo_off ? frames[i]->getVirtOffset()
                               : frames[i]->getVirtOffset() >> 32;
            }
        }
        warn("GenericTimerMem::timerCtrlRead: Unexpected address (0x%x:%i), "
             "assuming RAZ\n", addr, size);
        return 0;
    }
}

void
GenericTimerMem::timerCtrlWrite(Addr addr, size_t size, uint64_t data,
                                bool is_sec)
{
    switch (addr) {
      case TIMER_CTRL_CNTFRQ:
        if (!GenericTimerMem::validateAccessPerm(system, is_sec)) return;
        warn_if(data != systemCounter.freq(),
                "GenericTimerMem::timerCtrlWrite: CNTFRQ configured freq "
                "does not match the counter freq, ignoring\n");
        return;
      case TIMER_CTRL_CNTNSAR:
        if (!GenericTimerMem::validateAccessPerm(system, is_sec)) return;
        for (int i = 0; i < frames.size(); i++) {
            // Check if the CNTNSAR.NS bit is set for this frame
            if (data & (0x1 << i))
                frames[i]->setNonSecureAccess();
        }
        return;
      case TIMER_CTRL_CNTTIDR:
        warn("GenericTimerMem::timerCtrlWrite: RO reg (0x%x) [CNTTIDR]\n",
             addr);
        return;
      default:
        for (int i = 0; i < frames.size(); i++) {
            Addr cntacr_off = TIMER_CTRL_CNTACR + (i * 0x4);
            Addr cntvoff_lo_off = TIMER_CTRL_CNTVOFF_LO + (i * 0x4);
            Addr cntvoff_hi_off = TIMER_CTRL_CNTVOFF_HI + (i * 0x4);
            // CNTNSAR.NS determines if CNTACR/CNTVOFF are accessible from
            // normal world
            bool hit = addr == cntacr_off || addr == cntvoff_lo_off ||
                       addr == cntvoff_hi_off;
            bool has_access =
                GenericTimerMem::validateAccessPerm(system, is_sec) ||
                frames[i]->hasNonSecureAccess();
            if (hit && !has_access) return;
            if (addr == cntacr_off) {
                frames[i]->setAccessBits(data);
                return;
            }
            if (addr == cntvoff_lo_off || addr == cntvoff_hi_off) {
                if (addr == cntvoff_lo_off)
                    data = size == 4 ? insertBits(frames[i]->getVirtOffset(),
                                                  31, 0, data) : data;
                else
                    data = insertBits(frames[i]->getVirtOffset(),
                                      63, 32, data);
                frames[i]->setVirtOffset(data);
                return;
            }
        }
        warn("GenericTimerMem::timerCtrlWrite: Unexpected address "
             "(0x%x:%i), assuming WI\n", addr, size);
    }
}

SystemCounter *
SystemCounterParams::create()
{
    return new SystemCounter(this);
}

GenericTimer *
GenericTimerParams::create()
{
    return new GenericTimer(this);
}

GenericTimerFrame *
GenericTimerFrameParams::create()
{
    return new GenericTimerFrame(this);
}

GenericTimerMem *
GenericTimerMemParams::create()
{
    return new GenericTimerMem(this);
}
