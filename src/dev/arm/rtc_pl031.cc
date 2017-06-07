/*
 * Copyright (c) 2010-2012 ARM Limited
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
 * Authors: Ali Saidi
 */

#include "dev/arm/rtc_pl031.hh"

#include "base/intmath.hh"
#include "base/time.hh"
#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "debug/Timer.hh"
#include "dev/arm/amba_device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

PL031::PL031(Params *p)
    : AmbaIntDevice(p, 0xfff), timeVal(mkutctime(&p->time)),
      lastWrittenTick(0), loadVal(0), matchVal(0),
      rawInt(false), maskInt(false), pendingInt(false),
      matchEvent([this]{ counterMatch(); }, name())
{
}


Tick
PL031::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;
    uint32_t data;

    DPRINTF(Timer, "Reading from RTC at offset: %#x\n", daddr);

    switch (daddr) {
      case DataReg:
        data = timeVal + ((curTick() - lastWrittenTick) / SimClock::Int::s);
        break;
      case MatchReg:
        data = matchVal;
        break;
      case LoadReg:
        data = loadVal;
        break;
      case ControlReg:
        data = 1; // Always enabled otherwise there is no point
        break;
      case IntMask:
        data = maskInt;
        break;
      case RawISR:
        data = rawInt;
        break;
      case MaskedISR:
        data = pendingInt;
        break;
      default:
        if (readId(pkt, ambaId, pioAddr)) {
            // Hack for variable sized access
            data = pkt->get<uint32_t>();
            break;
        }
        panic("Tried to read PL031 at offset %#x that doesn't exist\n", daddr);
        break;
    }

    switch(pkt->getSize()) {
      case 1:
        pkt->set<uint8_t>(data);
        break;
      case 2:
        pkt->set<uint16_t>(data);
        break;
      case 4:
        pkt->set<uint32_t>(data);
        break;
      default:
        panic("Uart read size too big?\n");
        break;
    }


    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
PL031::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 4);
    Addr daddr = pkt->getAddr() - pioAddr;
    DPRINTF(Timer, "Writing to RTC at offset: %#x\n", daddr);

    switch (daddr) {
      case DataReg:
        break;
      case MatchReg:
        matchVal = pkt->get<uint32_t>();
        resyncMatch();
        break;
      case LoadReg:
        lastWrittenTick = curTick();
        timeVal = pkt->get<uint32_t>();
        loadVal = timeVal;
        resyncMatch();
        break;
      case ControlReg:
        break; // Can't stop when started
      case IntMask:
        maskInt = pkt->get<uint32_t>();
        break;
      case IntClear:
        if (pkt->get<uint32_t>()) {
            rawInt = false;
            pendingInt = false;
        }
        break;
      default:
        if (readId(pkt, ambaId, pioAddr))
            break;
        panic("Tried to read PL031 at offset %#x that doesn't exist\n", daddr);
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
PL031::resyncMatch()
{
    DPRINTF(Timer, "Setting up new match event match=%d time=%d\n", matchVal,
            timeVal);

    uint32_t seconds_until = matchVal - timeVal;
    Tick ticks_until = SimClock::Int::s * seconds_until;

    if (matchEvent.scheduled()) {
        DPRINTF(Timer, "-- Event was already schedule, de-scheduling\n");
        deschedule(matchEvent);
    }
    schedule(matchEvent, curTick() + ticks_until);
    DPRINTF(Timer, "-- Scheduling new event for: %d\n", curTick() + ticks_until);
}

void
PL031::counterMatch()
{
    DPRINTF(Timer, "Counter reached zero\n");

    rawInt = true;
    bool old_pending = pendingInt;
    pendingInt = maskInt & rawInt;
    if (pendingInt && !old_pending) {
        DPRINTF(Timer, "-- Causing interrupt\n");
        gic->sendInt(intNum);
    }
}

void
PL031::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Arm PL031\n");
    SERIALIZE_SCALAR(timeVal);
    SERIALIZE_SCALAR(lastWrittenTick);
    SERIALIZE_SCALAR(loadVal);
    SERIALIZE_SCALAR(matchVal);
    SERIALIZE_SCALAR(rawInt);
    SERIALIZE_SCALAR(maskInt);
    SERIALIZE_SCALAR(pendingInt);

    bool is_in_event = matchEvent.scheduled();
    SERIALIZE_SCALAR(is_in_event);

    Tick event_time;
    if (is_in_event){
        event_time = matchEvent.when();
        SERIALIZE_SCALAR(event_time);
    }
}

void
PL031::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Arm PL031\n");

    UNSERIALIZE_SCALAR(timeVal);
    UNSERIALIZE_SCALAR(lastWrittenTick);
    UNSERIALIZE_SCALAR(loadVal);
    UNSERIALIZE_SCALAR(matchVal);
    UNSERIALIZE_SCALAR(rawInt);
    UNSERIALIZE_SCALAR(maskInt);
    UNSERIALIZE_SCALAR(pendingInt);

    bool is_in_event;
    UNSERIALIZE_SCALAR(is_in_event);

    Tick event_time;
    if (is_in_event){
        UNSERIALIZE_SCALAR(event_time);
        schedule(matchEvent, event_time);
    }
}



PL031 *
PL031Params::create()
{
    return new PL031(this);
}
