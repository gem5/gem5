/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 */

/* @file
 * Device module for modelling an ethernet hub
 */

#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "debug/Ethernet.hh"
#include "debug/EthernetData.hh"
#include "dev/etherbus.hh"
#include "dev/etherdump.hh"
#include "dev/etherint.hh"
#include "dev/etherpkt.hh"
#include "params/EtherBus.hh"
#include "sim/core.hh"

using namespace std;

EtherBus::EtherBus(const Params *p)
    : EtherObject(p), ticksPerByte(p->speed), loopback(p->loopback),
      event(this), sender(0), dump(p->dump)
{
}

void
EtherBus::txDone()
{
    devlist_t::iterator i = devlist.begin();
    devlist_t::iterator end = devlist.end();

    DPRINTF(Ethernet, "ethernet packet received: length=%d\n", packet->length);
    DDUMP(EthernetData, packet->data, packet->length);

    while (i != end) {
        if (loopback || *i != sender)
            (*i)->sendPacket(packet);
        ++i;
    }

    sender->sendDone();

    if (dump)
        dump->dump(packet);

    sender = 0;
    packet = 0;
}

EtherInt*
EtherBus::getEthPort(const std::string &if_name, int idx)
{
    panic("Etherbus doesn't work\n");
}

bool
EtherBus::send(EtherInt *sndr, EthPacketPtr &pkt)
{
    if (busy()) {
        DPRINTF(Ethernet, "ethernet packet not sent, bus busy\n", curTick());
        return false;
    }

    DPRINTF(Ethernet, "ethernet packet sent: length=%d\n", pkt->length);
    DDUMP(EthernetData, pkt->data, pkt->length);

    packet = pkt;
    sender = sndr;
    int delay = (int)ceil(((double)pkt->length * ticksPerByte) + 1.0);
    DPRINTF(Ethernet, "scheduling packet: delay=%d, (rate=%f)\n",
            delay, ticksPerByte);
    schedule(event, curTick() + delay);

    return true;
}

EtherBus *
EtherBusParams::create()
{
    return new EtherBus(this);
}
