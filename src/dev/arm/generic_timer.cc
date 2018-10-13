/*
 * Copyright (c) 2013, 2015, 2017-2018 ARM Limited
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
 *          Andreas Sandberg
 */

#include "dev/arm/generic_timer.hh"

#include "arch/arm/system.hh"
#include "debug/Timer.hh"
#include "dev/arm/base_gic.hh"
#include "mem/packet_access.hh"
#include "params/GenericTimer.hh"
#include "params/GenericTimerMem.hh"

SystemCounter::SystemCounter()
    : _freq(0), _period(0), _resetTick(0), _regCntkctl(0)
{
    setFreq(0x01800000);
}

void
SystemCounter::setFreq(uint32_t freq)
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
SystemCounter::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_regCntkctl);
    SERIALIZE_SCALAR(_regCnthctl);
    SERIALIZE_SCALAR(_freq);
    SERIALIZE_SCALAR(_period);
    SERIALIZE_SCALAR(_resetTick);
}

void
SystemCounter::unserialize(CheckpointIn &cp)
{
    // We didn't handle CNTKCTL in this class before, assume it's zero
    // if it isn't present.
    if (!UNSERIALIZE_OPT_SCALAR(_regCntkctl))
        _regCntkctl = 0;
    if (!UNSERIALIZE_OPT_SCALAR(_regCnthctl))
        _regCnthctl = 0;
    UNSERIALIZE_SCALAR(_freq);
    UNSERIALIZE_SCALAR(_period);
    UNSERIALIZE_SCALAR(_resetTick);
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
}

void
ArchTimer::counterLimitReached()
{
    _control.istatus = 1;

    if (!_control.enable)
        return;

    DPRINTF(Timer, "Counter limit reached\n");
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
        _control.istatus = 0;
        if (scheduleEvents()) {
            const auto period(_systemCounter.period());
            _parent.schedule(_counterLimitReachedEvent,
                 curTick() + (_counterLimit - value()) * period);
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
    ArchTimerCtrl new_ctl = val;
    if ((new_ctl.enable && !new_ctl.imask) &&
        !(_control.enable && !_control.imask)) {
        // Re-evalute the timer condition
        if (_counterLimit >= value()) {
            _control.istatus = 1;

            DPRINTF(Timer, "Causing interrupt in control\n");
            //_interrupt.send();
        }
    }
    _control.enable = new_ctl.enable;
    _control.imask = new_ctl.imask;
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

GenericTimer::GenericTimer(GenericTimerParams *p)
    : ClockedObject(p),
      system(*p->system)
{
    fatal_if(!p->system, "No system specified, can't instantiate timer.\n");
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

    systemCounter.serializeSection(cp, "sys_counter");

    for (int i = 0; i < timers.size(); ++i) {
        const CoreTimers &core(*timers[i]);

        // This should really be phys_timerN, but we are stuck with
        // arch_timer for backwards compatibility.
        core.physNS.serializeSection(cp, csprintf("arch_timer%d", i));
        core.physS.serializeSection(cp, csprintf("phys_s_timer%d", i));
        core.virt.serializeSection(cp, csprintf("virt_timer%d", i));
        core.hyp.serializeSection(cp, csprintf("hyp_timer%d", i));
    }
}

void
GenericTimer::unserialize(CheckpointIn &cp)
{
    systemCounter.unserializeSection(cp, "sys_counter");

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

    for (int i = 0; i < cpu_count; ++i) {
        CoreTimers &core(getTimers(i));
        // This should really be phys_timerN, but we are stuck with
        // arch_timer for backwards compatibility.
        core.physNS.unserializeSection(cp, csprintf("arch_timer%d", i));
        core.physS.unserializeSection(cp, csprintf("phys_s_timer%d", i));
        core.virt.unserializeSection(cp, csprintf("virt_timer%d", i));
        core.hyp.unserializeSection(cp, csprintf("hyp_timer%d", i));
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

        ThreadContext *tc = system.getThreadContext(i);

        timers[i].reset(
            new CoreTimers(*this, system, i,
                           p->int_phys_s->get(tc),
                           p->int_phys_ns->get(tc),
                           p->int_virt->get(tc),
                           p->int_hyp->get(tc)));
    }
}


void
GenericTimer::setMiscReg(int reg, unsigned cpu, RegVal val)
{
    CoreTimers &core(getTimers(cpu));

    switch (reg) {
      case MISCREG_CNTFRQ:
      case MISCREG_CNTFRQ_EL0:
        systemCounter.setFreq(val);
        return;

      case MISCREG_CNTKCTL:
      case MISCREG_CNTKCTL_EL1:
        systemCounter.setKernelControl(val);
        return;

      case MISCREG_CNTHCTL:
      case MISCREG_CNTHCTL_EL2:
        systemCounter.setHypControl(val);
        return;

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
        return systemCounter.freq();

      case MISCREG_CNTKCTL:
      case MISCREG_CNTKCTL_EL1:
        return systemCounter.getKernelControl();

      case MISCREG_CNTHCTL:
      case MISCREG_CNTHCTL_EL2:
        return systemCounter.getHypControl();

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

GenericTimerMem::GenericTimerMem(GenericTimerMemParams *p)
    : PioDevice(p),
      ctrlRange(RangeSize(p->base, TheISA::PageBytes)),
      timerRange(RangeSize(p->base + TheISA::PageBytes, TheISA::PageBytes)),
      addrRanges{ctrlRange, timerRange},
      systemCounter(),
      physTimer(csprintf("%s.phys_timer0", name()),
                *this, systemCounter,
                p->int_phys->get()),
      virtTimer(csprintf("%s.virt_timer0", name()),
                *this, systemCounter,
                p->int_virt->get())
{
}

void
GenericTimerMem::serialize(CheckpointOut &cp) const
{
    paramOut(cp, "timer_count", 1);

    systemCounter.serializeSection(cp, "sys_counter");

    physTimer.serializeSection(cp, "phys_timer0");
    virtTimer.serializeSection(cp, "virt_timer0");
}

void
GenericTimerMem::unserialize(CheckpointIn &cp)
{
    systemCounter.unserializeSection(cp, "sys_counter");

    unsigned timer_count;
    UNSERIALIZE_SCALAR(timer_count);
    // The timer count variable is just here for future versions where
    // we support more than one set of timers.
    if (timer_count != 1)
        panic("Incompatible checkpoint: Only one set of timers supported");

    physTimer.unserializeSection(cp, "phys_timer0");
    virtTimer.unserializeSection(cp, "virt_timer0");
}

Tick
GenericTimerMem::read(PacketPtr pkt)
{
    const unsigned size(pkt->getSize());
    const Addr addr(pkt->getAddr());
    uint64_t value;

    pkt->makeResponse();
    if (ctrlRange.contains(addr)) {
        value = ctrlRead(addr - ctrlRange.start(), size);
    } else if (timerRange.contains(addr)) {
        value = timerRead(addr - timerRange.start(), size);
    } else {
        panic("Invalid address: 0x%x\n", addr);
    }

    DPRINTF(Timer, "Read 0x%x <- 0x%x(%i)\n", value, addr, size);

    if (size == 8) {
        pkt->setLE<uint64_t>(value);
    } else if (size == 4) {
        pkt->setLE<uint32_t>(value);
    } else {
        panic("Unexpected access size: %i\n", size);
    }

    return 0;
}

Tick
GenericTimerMem::write(PacketPtr pkt)
{
    const unsigned size(pkt->getSize());
    if (size != 8 && size != 4)
        panic("Unexpected access size\n");

    const Addr addr(pkt->getAddr());
    const uint64_t value(size == 8 ?
                         pkt->getLE<uint64_t>() : pkt->getLE<uint32_t>());

    DPRINTF(Timer, "Write 0x%x -> 0x%x(%i)\n", value, addr, size);
    if (ctrlRange.contains(addr)) {
        ctrlWrite(addr - ctrlRange.start(), size, value);
    } else if (timerRange.contains(addr)) {
        timerWrite(addr - timerRange.start(), size, value);
    } else {
        panic("Invalid address: 0x%x\n", addr);
    }

    pkt->makeResponse();
    return 0;
}

uint64_t
GenericTimerMem::ctrlRead(Addr addr, size_t size) const
{
    if (size == 4) {
        switch (addr) {
          case CTRL_CNTFRQ:
            return systemCounter.freq();

          case CTRL_CNTTIDR:
            return 0x3; // Frame 0 implemented with virtual timers

          case CTRL_CNTNSAR:
          case CTRL_CNTACR_BASE:
            warn("Reading from unimplemented control register (0x%x)\n", addr);
            return 0;

          case CTRL_CNTVOFF_LO_BASE:
            return virtTimer.offset();

          case CTRL_CNTVOFF_HI_BASE:
            return virtTimer.offset() >> 32;

          default:
            warn("Unexpected address (0x%x:%i), assuming RAZ\n", addr, size);
            return 0;
        }
    } else if (size == 8) {
        switch (addr) {
          case CTRL_CNTVOFF_LO_BASE:
            return virtTimer.offset();

          default:
            warn("Unexpected address (0x%x:%i), assuming RAZ\n", addr, size);
            return 0;
        }
    } else {
        panic("Invalid access size: %i\n", size);
    }
}

void
GenericTimerMem::ctrlWrite(Addr addr, size_t size, uint64_t value)
{
    if (size == 4) {
        switch (addr) {
          case CTRL_CNTFRQ:
          case CTRL_CNTNSAR:
          case CTRL_CNTTIDR:
          case CTRL_CNTACR_BASE:
            warn("Write to unimplemented control register (0x%x)\n", addr);
            return;

          case CTRL_CNTVOFF_LO_BASE:
            virtTimer.setOffset(
                insertBits(virtTimer.offset(), 31, 0, value));
            return;

          case CTRL_CNTVOFF_HI_BASE:
            virtTimer.setOffset(
                insertBits(virtTimer.offset(), 63, 32, value));
            return;

          default:
            warn("Ignoring write to unexpected address (0x%x:%i)\n",
                 addr, size);
            return;
        }
    } else if (size == 8) {
        switch (addr) {
          case CTRL_CNTVOFF_LO_BASE:
            virtTimer.setOffset(value);
            return;

          default:
            warn("Ignoring write to unexpected address (0x%x:%i)\n",
                 addr, size);
            return;
        }
    } else {
        panic("Invalid access size: %i\n", size);
    }
}

uint64_t
GenericTimerMem::timerRead(Addr addr, size_t size) const
{
    if (size == 4) {
        switch (addr) {
          case TIMER_CNTPCT_LO:
            return physTimer.value();

          case TIMER_CNTPCT_HI:
            return physTimer.value() >> 32;

          case TIMER_CNTVCT_LO:
            return virtTimer.value();

          case TIMER_CNTVCT_HI:
            return virtTimer.value() >> 32;

          case TIMER_CNTFRQ:
            return systemCounter.freq();

          case TIMER_CNTEL0ACR:
            warn("Read from unimplemented timer register (0x%x)\n", addr);
            return 0;

          case CTRL_CNTVOFF_LO_BASE:
            return virtTimer.offset();

          case CTRL_CNTVOFF_HI_BASE:
            return virtTimer.offset() >> 32;

          case TIMER_CNTP_CVAL_LO:
            return physTimer.compareValue();

          case TIMER_CNTP_CVAL_HI:
            return physTimer.compareValue() >> 32;

          case TIMER_CNTP_TVAL:
            return physTimer.timerValue();

          case TIMER_CNTP_CTL:
            return physTimer.control();

          case TIMER_CNTV_CVAL_LO:
            return virtTimer.compareValue();

          case TIMER_CNTV_CVAL_HI:
            return virtTimer.compareValue() >> 32;

          case TIMER_CNTV_TVAL:
            return virtTimer.timerValue();

          case TIMER_CNTV_CTL:
            return virtTimer.control();

          default:
            warn("Unexpected address (0x%x:%i), assuming RAZ\n", addr, size);
            return 0;
        }
    } else if (size == 8) {
        switch (addr) {
          case TIMER_CNTPCT_LO:
            return physTimer.value();

          case TIMER_CNTVCT_LO:
            return virtTimer.value();

          case CTRL_CNTVOFF_LO_BASE:
            return virtTimer.offset();

          case TIMER_CNTP_CVAL_LO:
            return physTimer.compareValue();

          case TIMER_CNTV_CVAL_LO:
            return virtTimer.compareValue();

          default:
            warn("Unexpected address (0x%x:%i), assuming RAZ\n", addr, size);
            return 0;
        }
    } else {
        panic("Invalid access size: %i\n", size);
    }
}

void
GenericTimerMem::timerWrite(Addr addr, size_t size, uint64_t value)
{
    if (size == 4) {
        switch (addr) {
          case TIMER_CNTEL0ACR:
            warn("Unimplemented timer register (0x%x)\n", addr);
            return;

          case TIMER_CNTP_CVAL_LO:
            physTimer.setCompareValue(
                insertBits(physTimer.compareValue(), 31, 0, value));
            return;

          case TIMER_CNTP_CVAL_HI:
            physTimer.setCompareValue(
                insertBits(physTimer.compareValue(), 63, 32, value));
            return;

          case TIMER_CNTP_TVAL:
            physTimer.setTimerValue(value);
            return;

          case TIMER_CNTP_CTL:
            physTimer.setControl(value);
            return;

          case TIMER_CNTV_CVAL_LO:
            virtTimer.setCompareValue(
                insertBits(virtTimer.compareValue(), 31, 0, value));
            return;

          case TIMER_CNTV_CVAL_HI:
            virtTimer.setCompareValue(
                insertBits(virtTimer.compareValue(), 63, 32, value));
            return;

          case TIMER_CNTV_TVAL:
            virtTimer.setTimerValue(value);
            return;

          case TIMER_CNTV_CTL:
            virtTimer.setControl(value);
            return;

          default:
            warn("Unexpected address (0x%x:%i), ignoring write\n", addr, size);
            return;
        }
    } else if (size == 8) {
        switch (addr) {
          case TIMER_CNTP_CVAL_LO:
            return physTimer.setCompareValue(value);

          case TIMER_CNTV_CVAL_LO:
            return virtTimer.setCompareValue(value);

          default:
            warn("Unexpected address (0x%x:%i), ignoring write\n", addr, size);
            return;
        }
    } else {
        panic("Invalid access size: %i\n", size);
    }
}

GenericTimer *
GenericTimerParams::create()
{
    return new GenericTimer(this);
}

GenericTimerMem *
GenericTimerMemParams::create()
{
    return new GenericTimerMem(this);
}
