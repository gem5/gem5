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

#include "dev/arm/watchdog_generic.hh"

#include "dev/arm/base_gic.hh"
#include "params/GenericWatchdog.hh"

namespace gem5
{

GenericWatchdog::GenericWatchdog(const GenericWatchdogParams &p)
    : PioDevice(p),
      timeoutEvent([this]{ timeout(); }, name()),
      controlStatus(0),
      offset(0),
      compare(0),
      iidr(0),
      refreshFrame(p.refresh_start, p.refresh_start + 0x10000),
      controlFrame(p.control_start, p.control_start + 0x10000),
      pioLatency(p.pio_latency),
      cnt(*p.system_counter),
      cntListener(*this),
      ws0(p.ws0->get()),
      ws1(p.ws1->get())
{
    cnt.registerListener(&cntListener);
}

AddrRangeList
GenericWatchdog::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(refreshFrame);
    ranges.push_back(controlFrame);
    return ranges;
}

Tick
GenericWatchdog::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const size_t size = pkt->getSize();
    panic_if(size != 4, "GenericWatchdog::read: Invalid size %i\n", size);

    uint32_t resp = 0;

    if (refreshFrame.contains(addr)) {
        resp = readRefresh(addr);
    } else if (controlFrame.contains(addr)) {
        resp = readControl(addr);
    } else {
        panic("%s unknown address %#x\n", __func__, addr);
    }

    pkt->setUintX(resp, ByteOrder::little);
    pkt->makeResponse();
    return pioLatency;
}

uint32_t
GenericWatchdog::readRefresh(Addr addr)
{
    const auto daddr = static_cast<RefreshOffset>(
        addr - refreshFrame.start());

    switch (daddr) {
      case RefreshOffset::WRR:
        // A read of the refresh register has no effect and returns 0
        return 0;
      case RefreshOffset::W_IIDR:
        return iidr;
      default:
        panic("%s unknown address %#x\n", __func__, addr);
    }
}

uint32_t
GenericWatchdog::readControl(Addr addr)
{
    const auto daddr = static_cast<ControlOffset>(
        addr - controlFrame.start());

    switch (daddr) {
      case ControlOffset::WCS:
        return controlStatus;
      case ControlOffset::WOR:
        return offset;
      case ControlOffset::WCV_LO:
        return bits(compare, 31, 0);
      case ControlOffset::WCV_HI:
        return bits(compare, 63, 32);
      case ControlOffset::W_IIDR:
        return iidr;
      default:
        panic("%s unknown address %#x\n", __func__, addr);
    }
}

Tick
GenericWatchdog::write(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr();
    const size_t size = pkt->getSize();
    panic_if(size != 4, "GenericWatchdog::write: Invalid size %i\n", size);

    uint32_t data = pkt->getUintX(ByteOrder::little);

    if (refreshFrame.contains(addr)) {
        writeRefresh(addr, data);
    } else if (controlFrame.contains(addr)) {
        writeControl(addr, data);
    } else {
        panic("%s unknown address %#x\n", __func__, addr);
    }

    pkt->makeResponse();
    return pioLatency;
}

void
GenericWatchdog::writeRefresh(Addr addr, uint32_t data)
{
    const auto daddr = static_cast<RefreshOffset>(
        addr - refreshFrame.start());

    switch (daddr) {
      case RefreshOffset::WRR:
        explicitRefresh();
        break;
      default:
        panic("%s unknown address %#x\n", __func__, addr);
    }
}

void
GenericWatchdog::writeControl(Addr addr, uint32_t data)
{
    const auto daddr = static_cast<ControlOffset>(
        addr - controlFrame.start());

    switch (daddr) {
      case ControlOffset::WCS:
        controlStatus = data & 0x1;
        explicitRefresh();
        break;
      case ControlOffset::WOR:
        offset = data;
        explicitRefresh();
        break;
      case ControlOffset::WCV_LO:
        compare = insertBits(compare, 31, 0, data);
        break;
      case ControlOffset::WCV_HI:
        compare = insertBits(compare, 63, 32, data);
        break;
      default:
        panic("%s unknown address %#x\n", __func__, addr);
    }
}

void
GenericWatchdog::explicitRefresh()
{
    // Watchdog signals are cleared in case of an explicit refresh
    controlStatus.ws0 = 0;
    controlStatus.ws1 = 0;
    ws0->clear();
    ws1->clear();

    refresh();
}

void
GenericWatchdog::refresh()
{
    // Update compare value
    compare = cnt.value() + offset;

    // Ask the System Counter how long we have to wait until
    // it reaches the new compare value
    Tick timeout_time = cnt.whenValue(compare);

    reschedule(timeoutEvent, timeout_time, true);
}

void
GenericWatchdog::timeout()
{
    if (!controlStatus.enabled)
        return;

    if (!controlStatus.ws0) {
        controlStatus.ws0 = 1;
        ws0->raise();
    } else {
        controlStatus.ws1 = 1;
        ws1->raise();
    }

    refresh();
}

void
GenericWatchdog::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(controlStatus);
    SERIALIZE_SCALAR(offset);
    SERIALIZE_SCALAR(compare);

    bool ev_scheduled = timeoutEvent.scheduled();
    SERIALIZE_SCALAR(ev_scheduled);
    if (ev_scheduled)
        SERIALIZE_SCALAR(timeoutEvent.when());
}

void
GenericWatchdog::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(controlStatus);
    UNSERIALIZE_SCALAR(offset);
    UNSERIALIZE_SCALAR(compare);

    bool ev_scheduled;
    UNSERIALIZE_SCALAR(ev_scheduled);
    if (ev_scheduled) {
        Tick when;
        UNSERIALIZE_SCALAR(when);
        reschedule(timeoutEvent, when, true);
    }
}

} // namespace gem5
