/*
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

#include "dev/arm/timer_sp804.hh"

#include <cassert>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "debug/Timer.hh"
#include "dev/arm/base_gic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

namespace gem5
{

Sp804::Sp804(const Params &p)
    : AmbaPioDevice(p, 0x1000),
      timer0(name() + ".timer0", this, p.int0->get(), p.clock0),
      timer1(name() + ".timer1", this, p.int1->get(), p.clock1)
{}

Sp804::Timer::Timer(std::string __name, Sp804 *_parent,
                    ArmInterruptPin *_interrupt, Tick _clock)
    : _name(__name),
      parent(_parent),
      interrupt(_interrupt),
      clock(_clock),
      control(0x20),
      rawInt(false),
      pendingInt(false),
      loadValue(0xffffffff),
      zeroEvent([this] { counterAtZero(); }, name())
{}

Tick
Sp804::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;
    DPRINTF(Timer, "Reading from DualTimer at offset: %#x\n", daddr);

    if (daddr < Timer::Size)
        timer0.read(pkt, daddr);
    else if ((daddr - Timer::Size) < Timer::Size)
        timer1.read(pkt, daddr - Timer::Size);
    else if (!readId(pkt, ambaId, pioAddr))
        panic("Tried to read SP804 at offset %#x that doesn't exist\n", daddr);
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Sp804::Timer::read(PacketPtr pkt, Addr daddr)
{
    switch (daddr) {
    case LoadReg:
        pkt->setLE<uint32_t>(loadValue);
        break;
    case CurrentReg:
        DPRINTF(Timer, "Event schedule for %d, clock=%d, prescale=%d\n",
                zeroEvent.when(), clock, control.timerPrescale);
        Tick time;
        time = zeroEvent.when() - curTick();
        time = (time / clock) >> (4 * control.timerPrescale);
        DPRINTF(Timer, "-- returning counter at %d\n", time);
        pkt->setLE<uint32_t>(time);
        break;
    case ControlReg:
        pkt->setLE<uint32_t>(control);
        break;
    case RawISR:
        pkt->setLE<uint32_t>(rawInt);
        break;
    case MaskedISR:
        pkt->setLE<uint32_t>(pendingInt);
        break;
    case BGLoad:
        pkt->setLE<uint32_t>(loadValue);
        break;
    default:
        panic("Tried to read SP804 timer at offset %#x\n", daddr);
        break;
    }
    DPRINTF(Timer, "Reading %#x from Timer at offset: %#x\n",
            pkt->getLE<uint32_t>(), daddr);
}

Tick
Sp804::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;
    DPRINTF(Timer, "Writing to DualTimer at offset: %#x\n", daddr);

    if (daddr < Timer::Size)
        timer0.write(pkt, daddr);
    else if ((daddr - Timer::Size) < Timer::Size)
        timer1.write(pkt, daddr - Timer::Size);
    else if (!readId(pkt, ambaId, pioAddr))
        panic("Tried to write SP804 at offset %#x that doesn't exist\n",
              daddr);
    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Sp804::Timer::write(PacketPtr pkt, Addr daddr)
{
    DPRINTF(Timer, "Writing %#x to Timer at offset: %#x\n",
            pkt->getLE<uint32_t>(), daddr);
    switch (daddr) {
    case LoadReg:
        loadValue = pkt->getLE<uint32_t>();
        restartCounter(loadValue);
        break;
    case CurrentReg:
        // Spec says this value can't be written, but linux writes it anyway
        break;
    case ControlReg:
        bool old_enable;
        old_enable = control.timerEnable;
        control = pkt->getLE<uint32_t>();
        if ((old_enable == 0) && control.timerEnable)
            restartCounter(loadValue);
        break;
    case IntClear:
        rawInt = false;
        if (pendingInt) {
            pendingInt = false;
            DPRINTF(Timer, "Clearing interrupt\n");
            interrupt->clear();
        }
        break;
    case BGLoad:
        loadValue = pkt->getLE<uint32_t>();
        break;
    default:
        panic("Tried to write SP804 timer at offset %#x\n", daddr);
        break;
    }
}

void
Sp804::Timer::restartCounter(uint32_t val)
{
    DPRINTF(Timer, "Resetting counter with value %#x\n", val);
    if (!control.timerEnable)
        return;

    Tick time = clock << (4 * control.timerPrescale);
    if (control.timerSize)
        time *= val;
    else
        time *= bits(val, 15, 0);

    if (zeroEvent.scheduled()) {
        DPRINTF(Timer, "-- Event was already schedule, de-scheduling\n");
        parent->deschedule(zeroEvent);
    }
    parent->schedule(zeroEvent, curTick() + time);
    DPRINTF(Timer, "-- Scheduling new event for: %d\n", curTick() + time);
}

void
Sp804::Timer::counterAtZero()
{
    if (!control.timerEnable)
        return;

    DPRINTF(Timer, "Counter reached zero\n");

    rawInt = true;
    bool old_pending = pendingInt;
    if (control.intEnable)
        pendingInt = true;
    if (pendingInt && !old_pending) {
        DPRINTF(Timer, "-- Causing interrupt\n");
        interrupt->raise();
    }

    if (control.oneShot)
        return;

    // Free-running
    if (control.timerMode == 0)
        restartCounter(0xffffffff);
    else
        restartCounter(loadValue);
}

void
Sp804::Timer::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Arm Sp804\n");

    uint32_t control_serial = control;
    SERIALIZE_SCALAR(control_serial);

    SERIALIZE_SCALAR(rawInt);
    SERIALIZE_SCALAR(pendingInt);
    SERIALIZE_SCALAR(loadValue);

    bool is_in_event = zeroEvent.scheduled();
    SERIALIZE_SCALAR(is_in_event);

    Tick event_time;
    if (is_in_event) {
        event_time = zeroEvent.when();
        SERIALIZE_SCALAR(event_time);
    }
}

void
Sp804::Timer::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm Sp804\n");

    uint32_t control_serial;
    UNSERIALIZE_SCALAR(control_serial);
    control = control_serial;

    UNSERIALIZE_SCALAR(rawInt);
    UNSERIALIZE_SCALAR(pendingInt);
    UNSERIALIZE_SCALAR(loadValue);

    bool is_in_event;
    UNSERIALIZE_SCALAR(is_in_event);

    Tick event_time;
    if (is_in_event) {
        UNSERIALIZE_SCALAR(event_time);
        parent->schedule(zeroEvent, event_time);
    }
}

void
Sp804::serialize(CheckpointOut &cp) const
{
    timer0.serializeSection(cp, "timer0");
    timer1.serializeSection(cp, "timer1");
}

void
Sp804::unserialize(CheckpointIn &cp)
{
    timer0.unserializeSection(cp, "timer0");
    timer1.unserializeSection(cp, "timer1");
}

} // namespace gem5
