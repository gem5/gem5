/*
 * Copyright (c) 2017 Gedare Bloom
 * Copyright (c) 2010 ARM Limited
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

#include "dev/arm/timer_a9global.hh"

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "debug/Timer.hh"
#include "dev/arm/base_gic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

A9GlobalTimer::A9GlobalTimer(Params *p)
    : BasicPioDevice(p, 0x1C), gic(p->gic),
      global_timer(name() + ".globaltimer", this, p->int_num)
{
}

A9GlobalTimer::Timer::Timer(std::string __name, A9GlobalTimer *_parent,
    int int_num)
    : _name(__name), parent(_parent), intNum(int_num), control(0x0),
      rawInt(false), pendingInt(false), autoIncValue(0x0), cmpValEvent(this)
{
}

Tick
A9GlobalTimer::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;

    if (daddr < Timer::Size)
        global_timer.read(pkt, daddr);
    else
        panic("Tried to read A9GlobalTimer at offset %#x that doesn't exist\n",
                daddr);
    pkt->makeAtomicResponse();
    return pioDelay;
}

uint64_t
A9GlobalTimer::Timer::getTimeCounterFromTicks(Tick ticks)
{
  return ticks / parent->clockPeriod() / (control.prescalar + 1) - 1;
}

void
A9GlobalTimer::Timer::read(PacketPtr pkt, Addr daddr)
{
    DPRINTF(Timer, "Reading from A9GlobalTimer at offset: %#x\n", daddr);
    uint64_t time;

    switch(daddr) {
      case CounterRegLow32:
        time = getTimeCounterFromTicks(curTick());
        DPRINTF(Timer, "-- returning lower 32-bits of counter: %u\n", time);
        pkt->setLE<uint32_t>(time);
        break;
      case CounterRegHigh32:
        time = getTimeCounterFromTicks(curTick());
        time >>= 32;
        DPRINTF(Timer, "-- returning upper 32-bits of counter: %u\n", time);
        pkt->setLE<uint32_t>(time);
        break;
      case ControlReg:
        pkt->setLE<uint32_t>(control);
        break;
      case IntStatusReg:
        pkt->setLE<uint32_t>(rawInt);
        break;
      case CmpValRegLow32:
        DPRINTF(Timer, "Event schedule for %d, clock=%d, prescale=%d\n",
                cmpValEvent.when(), parent->clockPeriod(), control.prescalar);
        if (cmpValEvent.scheduled()) {
          time = getTimeCounterFromTicks(cmpValEvent.when() - curTick());
        } else {
          time = 0;
        }
        DPRINTF(Timer, "-- returning lower 32-bits of comparator: %u\n", time);
        pkt->setLE<uint32_t>(time);
        break;
      case CmpValRegHigh32:
        DPRINTF(Timer, "Event schedule for %d, clock=%d, prescale=%d\n",
                cmpValEvent.when(), parent->clockPeriod(), control.prescalar);
        if (cmpValEvent.scheduled()) {
          time = getTimeCounterFromTicks(cmpValEvent.when() - curTick());
          time >>= 32;
        } else {
          time = 0;
        }
        DPRINTF(Timer, "-- returning upper 32-bits of comparator: %u\n", time);
        pkt->setLE<uint32_t>(time);
        break;
      case AutoIncrementReg:
        pkt->setLE<uint32_t>(autoIncValue);
        break;
      default:
        panic("Tried to read A9GlobalTimer at offset %#x\n", daddr);
        break;
    }
    DPRINTF(Timer, "Reading %#x from A9GlobalTimer at offset: %#x\n",
             pkt->getLE<uint32_t>(), daddr);
}

Tick
A9GlobalTimer::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;
    DPRINTF(Timer, "Writing to A9GlobalTimer at offset: %#x\n", daddr);

    warn_once("A9 Global Timer doesn't support banked per-cpu registers\n");

    if (daddr < Timer::Size)
        global_timer.write(pkt, daddr);
    else
        panic("Tried to write A9GlobalTimer at offset %#x doesn't exist\n",
                daddr);
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
A9GlobalTimer::Timer::write(PacketPtr pkt, Addr daddr)
{
    DPRINTF(Timer, "Writing %#x to A9GlobalTimer at offset: %#x\n",
            pkt->getLE<uint32_t>(), daddr);
    switch (daddr) {
     case CounterRegLow32:
     case CounterRegHigh32:
        DPRINTF(Timer, "Ignoring unsupported write to Global Timer Counter\n");
        break;
      case ControlReg:
        bool old_enable;
        bool old_cmpEnable;
        old_enable = control.enable;
        old_cmpEnable = control.cmpEnable;
        control = pkt->getLE<uint32_t>();
        if ((old_enable == 0) && control.enable)
            restartCounter();
        if ((old_cmpEnable == 0) && control.cmpEnable)
            restartCounter();
        break;
      case IntStatusReg:
        /* TODO: should check that '1' was written. */
        rawInt = false;
        if (pendingInt) {
            pendingInt = false;
            DPRINTF(Timer, "Clearing interrupt\n");
            parent->gic->clearInt(intNum);
        }
        break;
      case CmpValRegLow32:
        cmpVal &= 0xFFFFFFFF00000000ULL;
        cmpVal |= (uint64_t)pkt->getLE<uint32_t>();
        break;
      case CmpValRegHigh32:
        cmpVal &= 0x00000000FFFFFFFFULL;
        cmpVal |= ((uint64_t)pkt->getLE<uint32_t>() << 32);
        break;
      case AutoIncrementReg:
        autoIncValue = pkt->getLE<uint32_t>();
        break;
      default:
        panic("Tried to write A9GlobalTimer at offset %#x\n", daddr);
        break;
    }
}

void
A9GlobalTimer::Timer::restartCounter()
{
    if (!control.enable)
        return;
    DPRINTF(Timer, "Restarting counter with value %#x\n", cmpVal);

    Tick time = parent->clockPeriod() * (control.prescalar + 1) * (cmpVal + 1);

    if (time < curTick()) {
        DPRINTF(Timer, "-- Event time %#x < curTick %#x\n", time, curTick());
        return;
    }
    if (cmpValEvent.scheduled()) {
        DPRINTF(Timer, "-- Event was already schedule, de-scheduling\n");
        parent->deschedule(cmpValEvent);
    }
    parent->schedule(cmpValEvent, time);
    DPRINTF(Timer, "-- Scheduling new event for: %d\n", time);
}

void
A9GlobalTimer::Timer::counterAtCmpVal()
{
    if (!control.enable)
        return;

    DPRINTF(Timer, "Counter reached cmpVal\n");

    rawInt = true;
    bool old_pending = pendingInt;
    if (control.intEnable)
        pendingInt = true;
    if (pendingInt && !old_pending) {
        DPRINTF(Timer, "-- Causing interrupt\n");
        parent->gic->sendPPInt(intNum, 0); /* FIXME: cpuNum */
    }

    if (control.autoIncrement == 0) // one-shot
        return;

    cmpVal += (uint64_t)autoIncValue;
    restartCounter();
}

void
A9GlobalTimer::Timer::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Arm A9GlobalTimer\n");

    uint32_t control_serial = control;
    SERIALIZE_SCALAR(control_serial);

    SERIALIZE_SCALAR(rawInt);
    SERIALIZE_SCALAR(pendingInt);
    SERIALIZE_SCALAR(cmpVal);
    SERIALIZE_SCALAR(autoIncValue);

    bool is_in_event = cmpValEvent.scheduled();
    SERIALIZE_SCALAR(is_in_event);

    Tick event_time;
    if (is_in_event){
        event_time = cmpValEvent.when();
        SERIALIZE_SCALAR(event_time);
    }
}

void
A9GlobalTimer::Timer::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm A9GlobalTimer\n");

    uint32_t control_serial;
    UNSERIALIZE_SCALAR(control_serial);
    control = control_serial;

    UNSERIALIZE_SCALAR(rawInt);
    UNSERIALIZE_SCALAR(pendingInt);
    UNSERIALIZE_SCALAR(cmpVal);
    UNSERIALIZE_SCALAR(autoIncValue);

    bool is_in_event;
    UNSERIALIZE_SCALAR(is_in_event);

    Tick event_time;
    if (is_in_event){
        UNSERIALIZE_SCALAR(event_time);
        parent->schedule(cmpValEvent, event_time);
    }
}

void
A9GlobalTimer::serialize(CheckpointOut &cp) const
{
    global_timer.serialize(cp);
}

void
A9GlobalTimer::unserialize(CheckpointIn &cp)
{
    global_timer.unserialize(cp);
}

A9GlobalTimer *
A9GlobalTimerParams::create()
{
    return new A9GlobalTimer(this);
}
