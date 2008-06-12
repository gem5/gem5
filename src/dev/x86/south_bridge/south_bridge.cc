/*
 * Copyright (c) 2008 The Regents of The University of Michigan
 * All rights reserved.
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
 * Authors: Gabe Black
 */

#include "arch/x86/x86_traits.hh"
#include "base/range.hh"
#include "dev/x86/south_bridge/south_bridge.hh"

using namespace X86ISA;

void
SouthBridge::addDevice(X86ISA::SubDevice & sub)
{
    rangeList.push_back(sub.addrRange);
    rangeMap.insert(sub.addrRange, &sub);
}

void
SouthBridge::addressRanges(AddrRangeList &range_list)
{
    range_list = rangeList;
}

Tick
SouthBridge::read(PacketPtr pkt)
{
    RangeMapIt sub =
        rangeMap.find(RangeSize(pkt->getAddr(), 1));
    assert(sub != rangeMap.end());
    return sub->second->read(pkt);
}

Tick
SouthBridge::write(PacketPtr pkt)
{
    RangeMapIt sub =
        rangeMap.find(RangeSize(pkt->getAddr(), 1));
    assert(sub != rangeMap.end());
    return sub->second->write(pkt);
}

SouthBridge::SouthBridge(const Params *p) : PioDevice(p),
    pic1(0x20, 2, p->pio_latency),
    pic2(0xA0, 2, p->pio_latency),
    pit(p->name + ".pit", 0x40, 4, p->pio_latency),
    cmos(0x70, 2, p->pio_latency),
    speaker(0x61, 1, p->pio_latency)
{
    addDevice(pic1);
    addDevice(pic2);
    addDevice(pit);
    addDevice(cmos);
    addDevice(speaker);
}

SouthBridge *
SouthBridgeParams::create()
{
    return new SouthBridge(this);
}
