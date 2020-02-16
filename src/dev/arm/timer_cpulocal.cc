/*
 * Copyright (c) 2010-2013,2018 ARM Limited
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

#include "dev/arm/timer_cpulocal.hh"

#include "arch/arm/system.hh"
#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "debug/Timer.hh"
#include "dev/arm/base_gic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

CpuLocalTimer::CpuLocalTimer(Params *p)
    : BasicPioDevice(p, 0x38)
{
}

void
CpuLocalTimer::init()
{
   auto p = params();
   // Initialize the timer registers for each per cpu timer
   for (int i = 0; i < sys->numContexts(); i++) {
        ThreadContext* tc = sys->getThreadContext(i);
        std::stringstream oss;
        oss << name() << ".timer" << i;

        localTimer.emplace_back(
            new Timer(oss.str(), this,
                      p->int_timer->get(tc),
                      p->int_watchdog->get(tc)));
    }

    BasicPioDevice::init();
}

CpuLocalTimer::Timer::Timer(const std::string &timer_name,
                            CpuLocalTimer* _parent,
                            ArmInterruptPin* int_timer,
                            ArmInterruptPin* int_watchdog)
    : _name(timer_name), parent(_parent), intTimer(int_timer),
      intWatchdog(int_watchdog), timerControl(0x0), watchdogControl(0x0),
      rawIntTimer(false), rawIntWatchdog(false),
      rawResetWatchdog(false), watchdogDisableReg(0x0),
      pendingIntTimer(false), pendingIntWatchdog(false),
      timerLoadValue(0x0), watchdogLoadValue(0x0),
      timerZeroEvent([this]{ timerAtZero(); }, name()),
      watchdogZeroEvent([this]{ watchdogAtZero(); }, name())
{
}

Tick
CpuLocalTimer::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;
    ContextID cpu_id = pkt->req->contextId();
    DPRINTF(Timer, "Reading from CpuLocalTimer at offset: %#x\n", daddr);
    assert(cpu_id >= 0);
    assert(cpu_id < localTimer.size());

    if (daddr < Timer::Size)
        localTimer[cpu_id]->read(pkt, daddr);
    else
        panic("Tried to read CpuLocalTimer at offset %#x that doesn't exist\n", daddr);
    pkt->makeAtomicResponse();
    return pioDelay;
}


void
CpuLocalTimer::Timer::read(PacketPtr pkt, Addr daddr)
{
    DPRINTF(Timer, "Reading from CpuLocalTimer at offset: %#x\n", daddr);
    Tick time;

    switch(daddr) {
      case TimerLoadReg:
        pkt->setLE<uint32_t>(timerLoadValue);
        break;
      case TimerCounterReg:
        DPRINTF(Timer, "Event schedule for timer %d, clock=%d, prescale=%d\n",
                timerZeroEvent.when(), parent->clockPeriod(),
                timerControl.prescalar);
        time = timerZeroEvent.when() - curTick();
        time = time / parent->clockPeriod() /
            power(16, timerControl.prescalar);
        DPRINTF(Timer, "-- returning counter at %d\n", time);
        pkt->setLE<uint32_t>(time);
        break;
      case TimerControlReg:
        pkt->setLE<uint32_t>(timerControl);
        break;
      case TimerIntStatusReg:
        pkt->setLE<uint32_t>(rawIntTimer);
        break;
      case WatchdogLoadReg:
        pkt->setLE<uint32_t>(watchdogLoadValue);
        break;
      case WatchdogCounterReg:
        DPRINTF(Timer,
                "Event schedule for watchdog %d, clock=%d, prescale=%d\n",
                watchdogZeroEvent.when(), parent->clockPeriod(),
                watchdogControl.prescalar);
        time = watchdogZeroEvent.when() - curTick();
        time = time / parent->clockPeriod() /
            power(16, watchdogControl.prescalar);
        DPRINTF(Timer, "-- returning counter at %d\n", time);
        pkt->setLE<uint32_t>(time);
        break;
      case WatchdogControlReg:
        pkt->setLE<uint32_t>(watchdogControl);
        break;
      case WatchdogIntStatusReg:
        pkt->setLE<uint32_t>(rawIntWatchdog);
        break;
      case WatchdogResetStatusReg:
        pkt->setLE<uint32_t>(rawResetWatchdog);
        break;
      case WatchdogDisableReg:
        panic("Tried to read from WatchdogDisableRegister\n");
        break;
      default:
        panic("Tried to read CpuLocalTimer at offset %#x\n", daddr);
        break;
    }
}

Tick
CpuLocalTimer::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;
    ContextID cpu_id = pkt->req->contextId();
    DPRINTF(Timer, "Writing to CpuLocalTimer at offset: %#x\n", daddr);
    assert(cpu_id >= 0);
    assert(cpu_id < localTimer.size());

    if (daddr < Timer::Size)
        localTimer[cpu_id]->write(pkt, daddr);
    else
        panic("Tried to write CpuLocalTimer at offset %#x that doesn't exist\n", daddr);
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
CpuLocalTimer::Timer::write(PacketPtr pkt, Addr daddr)
{
    DPRINTF(Timer, "Writing to CpuLocalTimer at offset: %#x\n", daddr);
    bool old_enable;
    bool old_wd_mode;
    uint32_t old_val;

    switch (daddr) {
      case TimerLoadReg:
        // Writing to this register also resets the counter register and
        // starts decrementing if the counter is enabled.
        timerLoadValue = pkt->getLE<uint32_t>();
        restartTimerCounter(timerLoadValue);
        break;
      case TimerCounterReg:
        // Can be written, doesn't start counting unless the timer is enabled
        restartTimerCounter(pkt->getLE<uint32_t>());
        break;
      case TimerControlReg:
        old_enable = timerControl.enable;
        timerControl = pkt->getLE<uint32_t>();
        if ((old_enable == 0) && timerControl.enable)
            restartTimerCounter(timerLoadValue);
        break;
      case TimerIntStatusReg:
        rawIntTimer = false;
        if (pendingIntTimer) {
            pendingIntTimer = false;
            DPRINTF(Timer, "Clearing interrupt\n");
        }
        break;
      case WatchdogLoadReg:
        watchdogLoadValue = pkt->getLE<uint32_t>();
        restartWatchdogCounter(watchdogLoadValue);
        break;
      case WatchdogCounterReg:
        // Can't be written when in watchdog mode, but can in timer mode
        if (!watchdogControl.watchdogMode) {
            restartWatchdogCounter(pkt->getLE<uint32_t>());
        }
        break;
      case WatchdogControlReg:
        old_enable = watchdogControl.enable;
        old_wd_mode = watchdogControl.watchdogMode;
        watchdogControl = pkt->getLE<uint32_t>();
        if ((old_enable == 0) && watchdogControl.enable)
            restartWatchdogCounter(watchdogLoadValue);
        // cannot disable watchdog using control register
        if ((old_wd_mode == 1) && watchdogControl.watchdogMode == 0)
            watchdogControl.watchdogMode = 1;
        break;
      case WatchdogIntStatusReg:
        rawIntWatchdog = false;
        if (pendingIntWatchdog) {
            pendingIntWatchdog = false;
            DPRINTF(Timer, "Clearing watchdog interrupt\n");
        }
        break;
      case WatchdogResetStatusReg:
        rawResetWatchdog = false;
        DPRINTF(Timer, "Clearing watchdog reset flag\n");
        break;
      case WatchdogDisableReg:
        old_val = watchdogDisableReg;
        watchdogDisableReg = pkt->getLE<uint32_t>();
        // if this sequence is observed, turn off watchdog mode
        if (old_val == 0x12345678 && watchdogDisableReg == 0x87654321)
            watchdogControl.watchdogMode = 0;
        break;
      default:
        panic("Tried to write CpuLocalTimer timer at offset %#x\n", daddr);
        break;
    }
}

//XXX: Two functions are needed because the control registers are different types
void
CpuLocalTimer::Timer::restartTimerCounter(uint32_t val)
{
    DPRINTF(Timer, "Resetting timer counter with value %#x\n", val);
    if (!timerControl.enable)
        return;

    Tick time = parent->clockPeriod() * power(16, timerControl.prescalar);
    time *= val;

    if (timerZeroEvent.scheduled()) {
        DPRINTF(Timer, "-- Event was already schedule, de-scheduling\n");
        parent->deschedule(timerZeroEvent);
    }
    parent->schedule(timerZeroEvent, curTick() + time);
    DPRINTF(Timer, "-- Scheduling new event for: %d\n", curTick() + time);
}

void
CpuLocalTimer::Timer::restartWatchdogCounter(uint32_t val)
{
    DPRINTF(Timer, "Resetting watchdog counter with value %#x\n", val);
    if (!watchdogControl.enable)
        return;

    Tick time = parent->clockPeriod() * power(16, watchdogControl.prescalar);
    time *= val;

    if (watchdogZeroEvent.scheduled()) {
        DPRINTF(Timer, "-- Event was already schedule, de-scheduling\n");
        parent->deschedule(watchdogZeroEvent);
    }
    parent->schedule(watchdogZeroEvent, curTick() + time);
    DPRINTF(Timer, "-- Scheduling new event for: %d\n", curTick() + time);
}
//////

void
CpuLocalTimer::Timer::timerAtZero()
{
    if (!timerControl.enable)
        return;

    DPRINTF(Timer, "Timer Counter reached zero\n");

    rawIntTimer = true;
    bool old_pending = pendingIntTimer;
    if (timerControl.intEnable)
        pendingIntTimer = true;
    if (pendingIntTimer && !old_pending) {
        DPRINTF(Timer, "-- Causing interrupt\n");
        intTimer->raise();
    }

    if (!timerControl.autoReload)
        return;
    else
        restartTimerCounter(timerLoadValue);
}

void
CpuLocalTimer::Timer::watchdogAtZero()
{
    if (!watchdogControl.enable)
        return;

    DPRINTF(Timer, "Watchdog Counter reached zero\n");

    rawIntWatchdog = true;
    bool old_pending = pendingIntWatchdog;
    // generates an interrupt only if the watchdog is in timer
    // mode.
    if (watchdogControl.intEnable && !watchdogControl.watchdogMode)
        pendingIntWatchdog = true;
    else if (watchdogControl.watchdogMode) {
        rawResetWatchdog = true;
        fatal("gem5 ARM Model does not support true watchdog operation!\n");
        //XXX: Should we ever support a true watchdog reset?
    }

    if (pendingIntWatchdog && !old_pending) {
        DPRINTF(Timer, "-- Causing interrupt\n");
        intWatchdog->raise();
    }

    if (watchdogControl.watchdogMode)
        return;
    else if (watchdogControl.autoReload)
        restartWatchdogCounter(watchdogLoadValue);
}

void
CpuLocalTimer::Timer::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Arm CpuLocalTimer\n");

    uint32_t timer_control_serial = timerControl;
    uint32_t watchdog_control_serial = watchdogControl;
    SERIALIZE_SCALAR(timer_control_serial);
    SERIALIZE_SCALAR(watchdog_control_serial);

    SERIALIZE_SCALAR(rawIntTimer);
    SERIALIZE_SCALAR(rawIntWatchdog);
    SERIALIZE_SCALAR(rawResetWatchdog);
    SERIALIZE_SCALAR(watchdogDisableReg);
    SERIALIZE_SCALAR(pendingIntTimer);
    SERIALIZE_SCALAR(pendingIntWatchdog);
    SERIALIZE_SCALAR(timerLoadValue);
    SERIALIZE_SCALAR(watchdogLoadValue);

    bool timer_is_in_event = timerZeroEvent.scheduled();
    SERIALIZE_SCALAR(timer_is_in_event);
    bool watchdog_is_in_event = watchdogZeroEvent.scheduled();
    SERIALIZE_SCALAR(watchdog_is_in_event);

    Tick timer_event_time;
    if (timer_is_in_event){
        timer_event_time = timerZeroEvent.when();
        SERIALIZE_SCALAR(timer_event_time);
    }
    Tick watchdog_event_time;
    if (watchdog_is_in_event){
        watchdog_event_time = watchdogZeroEvent.when();
        SERIALIZE_SCALAR(watchdog_event_time);
    }
}

void
CpuLocalTimer::Timer::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm CpuLocalTimer\n");

    uint32_t timer_control_serial;
    UNSERIALIZE_SCALAR(timer_control_serial);
    timerControl = timer_control_serial;
    uint32_t watchdog_control_serial;
    UNSERIALIZE_SCALAR(watchdog_control_serial);
    watchdogControl = watchdog_control_serial;

    UNSERIALIZE_SCALAR(rawIntTimer);
    UNSERIALIZE_SCALAR(rawIntWatchdog);
    UNSERIALIZE_SCALAR(rawResetWatchdog);
    UNSERIALIZE_SCALAR(watchdogDisableReg);
    UNSERIALIZE_SCALAR(pendingIntTimer);
    UNSERIALIZE_SCALAR(pendingIntWatchdog);
    UNSERIALIZE_SCALAR(timerLoadValue);
    UNSERIALIZE_SCALAR(watchdogLoadValue);

    bool timer_is_in_event;
    UNSERIALIZE_SCALAR(timer_is_in_event);
    bool watchdog_is_in_event;
    UNSERIALIZE_SCALAR(watchdog_is_in_event);

    Tick timer_event_time;
    if (timer_is_in_event){
        UNSERIALIZE_SCALAR(timer_event_time);
        parent->schedule(timerZeroEvent, timer_event_time);
    }
    Tick watchdog_event_time;
    if (watchdog_is_in_event) {
        UNSERIALIZE_SCALAR(watchdog_event_time);
        parent->schedule(watchdogZeroEvent, watchdog_event_time);
    }
}



void
CpuLocalTimer::serialize(CheckpointOut &cp) const
{
    for (int i = 0; i < sys->numContexts(); i++)
        localTimer[i]->serializeSection(cp, csprintf("timer%d", i));
}

void
CpuLocalTimer::unserialize(CheckpointIn &cp)
{
    for (int i = 0; i < sys->numContexts(); i++)
        localTimer[i]->unserializeSection(cp, csprintf("timer%d", i));
}

CpuLocalTimer *
CpuLocalTimerParams::create()
{
    return new CpuLocalTimer(this);
}
