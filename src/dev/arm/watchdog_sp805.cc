/*
 * Copyright (c) 2020 ARM Limited
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

#include "dev/arm/watchdog_sp805.hh"

#include "base/logging.hh"
#include "debug/Sp805.hh"
#include "mem/packet_access.hh"
#include "params/Sp805.hh"

Sp805::Sp805(Sp805Params const* params)
    : AmbaIntDevice(params, 0x1000),
      timeoutInterval(0xffffffff),
      timeoutStartTick(MaxTick),
      persistedValue(timeoutInterval),
      enabled(false),
      resetEnabled(false),
      writeAccessEnabled(true),
      integrationTestEnabled(false),
      timeoutEvent([this] { timeoutExpired(); }, name())
{
}

Tick
Sp805::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr() - pioAddr;
    const size_t size = pkt->getSize();
    panic_if(size != 4, "Sp805::read: Invalid size %i\n", size);

    uint64_t resp = 0;
    switch (addr) {
      case WDOGLOAD:
        resp = timeoutInterval;
        break;
      case WDOGVALUE:
        resp = value();
        break;
      case WDOGCONTROL:
        resp = enabled | (resetEnabled << 1);
        break;
      case WDOGINTCLR:
        warn("Sp805::read: WO reg (0x%x) [WDOGINTCLR]\n", addr);
        break;
      case WDOGRIS:
        resp = interrupt->active();
        break;
      case WDOGMIS:
        resp = interrupt->active() && enabled;
        break;
      case WDOGLOCK:
        resp = writeAccessEnabled;
        break;
      case WDOGITCR:
        resp = integrationTestEnabled;
        break;
      case WDOGITOP:
        warn("Sp805::read: WO reg (0x%x) [WDOGITOP]\n", addr);
        break;
      default:
        if (readId(pkt, ambaId, pioAddr))
            resp = pkt->getUintX(ByteOrder::little);
        else
            warn("Sp805::read: Unexpected address (0x%x:%i), assuming RAZ\n",
                 addr, size);
    }

    DPRINTF(Sp805, "Sp805::read: 0x%x<-0x%x(%i)\n", resp, addr, size);

    pkt->setUintX(resp, ByteOrder::little);
    pkt->makeResponse();
    return pioDelay;
}

Tick
Sp805::write(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr() - pioAddr;
    const size_t size = pkt->getSize();
    panic_if(size != 4, "Sp805::write: Invalid size %i\n", size);

    uint64_t data = pkt->getUintX(ByteOrder::little);
    switch (addr) {
      case WDOGLOAD:
        if (writeAccessEnabled) {
            // When WdogLoad is written 0x0, immediately trigger an interrupt
            if (!timeoutInterval)
                sendInt();
            else
                timeoutInterval = data;
            if (enabled)
                restartCounter();
        }
        break;
      case WDOGVALUE:
        warn("Sp805::write: RO reg (0x%x) [WDOGVALUE]\n", addr);
        break;
      case WDOGCONTROL:
        if (writeAccessEnabled) {
            bool was_enabled = enabled;
            enabled = bits(data, 0);
            resetEnabled = bits(data, 1);
            // If watchdog becomes enabled, restart the counter
            if (!was_enabled && enabled)
                restartCounter();
            // If watchdog becomes disabled, stop the counter
            else if (timeoutEvent.scheduled() && !enabled)
                stopCounter();
        }
        break;
      case WDOGINTCLR:
        if (writeAccessEnabled) {
            // Clear the interrupt and restart the counter if enabled
            clearInt();
            if (enabled)
                restartCounter();
        }
        break;
      case WDOGRIS:
        warn("Sp805::write: RO reg (0x%x) [WDOGRIS]\n", addr);
        break;
      case WDOGMIS:
        warn("Sp805::write: RO reg (0x%x) [WDOGMIS]\n", addr);
        break;
      case WDOGLOCK:
        writeAccessEnabled = (data == WDOGLOCK_MAGIC);
        break;
      case WDOGITCR ... WDOGITOP:
        warn("Sp805::write: No support for integration test harness\n");
        break;
      default:
        warn("Sp805::write: Unexpected address (0x%x:%i), assuming WI\n",
             addr, size);
    }

    DPRINTF(Sp805, "Sp805::write: 0x%x->0x%x(%i)\n", data, addr, size);

    pkt->makeResponse();
    return pioDelay;
}

uint32_t
Sp805::value() const
{
    return timeoutEvent.scheduled() ? timeoutInterval -
           ((timeoutEvent.when() - timeoutStartTick) / clockPeriod())
           : persistedValue;
}

void
Sp805::timeoutExpired()
{
    timeoutStartTick = MaxTick;
    sendInt();
    restartCounter();
}

void
Sp805::restartCounter()
{
    reschedule(timeoutEvent, clockEdge(Cycles(timeoutInterval)), true);
    timeoutStartTick = curTick();
}

void
Sp805::stopCounter()
{
    persistedValue = value();
    deschedule(timeoutEvent);
    timeoutStartTick = MaxTick;
}

void
Sp805::sendInt()
{
    // If the previously sent interrupt has not been served,
    // assert system reset if enabled
    if (interrupt->active() && enabled) {
        if (resetEnabled)
            warn("Watchdog timed out, system reset asserted\n");
    } else {
        interrupt->raise();
    }
}

void
Sp805::clearInt()
{
    interrupt->clear();
}

void
Sp805::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(timeoutInterval);
    SERIALIZE_SCALAR(timeoutStartTick);
    SERIALIZE_SCALAR(persistedValue);
    SERIALIZE_SCALAR(enabled);
    SERIALIZE_SCALAR(resetEnabled);
    SERIALIZE_SCALAR(writeAccessEnabled);
    SERIALIZE_SCALAR(integrationTestEnabled);

    bool ev_scheduled = timeoutEvent.scheduled();
    SERIALIZE_SCALAR(ev_scheduled);
    if (ev_scheduled)
        SERIALIZE_SCALAR(timeoutEvent.when());
}

void
Sp805::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(timeoutInterval);
    UNSERIALIZE_SCALAR(timeoutStartTick);
    UNSERIALIZE_SCALAR(persistedValue);
    UNSERIALIZE_SCALAR(enabled);
    UNSERIALIZE_SCALAR(resetEnabled);
    UNSERIALIZE_SCALAR(writeAccessEnabled);
    UNSERIALIZE_SCALAR(integrationTestEnabled);

    bool ev_scheduled;
    UNSERIALIZE_SCALAR(ev_scheduled);
    if (ev_scheduled) {
        Tick when;
        UNSERIALIZE_SCALAR(when);
        reschedule(timeoutEvent, when, true);
    }
}

Sp805 *
Sp805Params::create()
{
    return new Sp805(this);
}
