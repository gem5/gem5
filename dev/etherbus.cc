/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
 */

/* @file
 * Device module for modelling an ethernet hub
 */

#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "dev/etherbus.hh"
#include "dev/etherdump.hh"
#include "dev/etherint.hh"
#include "dev/etherpkt.hh"
#include "sim/builder.hh"
#include "sim/universe.hh"

using namespace std;

EtherBus::EtherBus(const string &name, double rate, bool loop,
                   EtherDump *packet_dump)
    : SimObject(name), ticks_per_byte(rate), loopback(loop),
      event(&mainEventQueue, this),
      sender(0), dump(packet_dump)
{ }

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

void
EtherBus::reg(EtherInt *dev)
{ devlist.push_back(dev); }

bool
EtherBus::send(EtherInt *sndr, PacketPtr &pkt)
{
    if (busy()) {
        DPRINTF(Ethernet, "ethernet packet not sent, bus busy\n", curTick);
        return false;
    }

    DPRINTF(Ethernet, "ethernet packet sent: length=%d\n", pkt->length);
    DDUMP(EthernetData, pkt->data, pkt->length);

    packet = pkt;
    sender = sndr;
    int delay = (int)ceil(((double)pkt->length * ticks_per_byte) + 1.0);
    DPRINTF(Ethernet, "scheduling packet: delay=%d, (rate=%f)\n",
            delay, ticks_per_byte);
    event.schedule(curTick + delay);

    return true;
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(EtherBus)

    Param<bool> loopback;
    Param<int> speed;
    SimObjectParam<EtherDump *> packet_dump;

END_DECLARE_SIM_OBJECT_PARAMS(EtherBus)

BEGIN_INIT_SIM_OBJECT_PARAMS(EtherBus)

    INIT_PARAM_DFLT(loopback,
                    "send the packet back to the interface from which it came",
                    true),
    INIT_PARAM_DFLT(speed, "bus speed in bits per second", 100000000),
    INIT_PARAM_DFLT(packet_dump, "object to dump network packets to", NULL)

END_INIT_SIM_OBJECT_PARAMS(EtherBus)

CREATE_SIM_OBJECT(EtherBus)
{
    double rate = ((double)ticksPerSecond * 8.0) / (double)speed;
    return new EtherBus(getInstanceName(), rate, loopback, packet_dump);
}

REGISTER_SIM_OBJECT("EtherBus", EtherBus)
