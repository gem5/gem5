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
 * Device module for modelling a fixed bandwidth full duplex ethernet link
 */

#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "dev/etherdump.hh"
#include "dev/etherint.hh"
#include "dev/etherlink.hh"
#include "dev/etherpkt.hh"
#include "sim/builder.hh"
#include "sim/universe.hh"
#include "sim/system.hh"

using namespace std;

EtherLink::EtherLink(const std::string &name, EtherInt *i1, EtherInt *i2,
                     Tick speed, EtherDump *dump)
    : SimObject(name)
{
    double rate = ((double)ticksPerSecond * 8.0) / (double)speed;

    link1 = new Link(name + ".link1", rate, dump);
    link2 = new Link(name + ".link2", rate, dump);

    int1 = new Interface(name + ".int1", link1, link2);
    int2 = new Interface(name + ".int2", link2, link1);

    int1->setPeer(i1);
    i1->setPeer(int1);
    int2->setPeer(i2);
    i2->setPeer(int2);
}

EtherLink::~EtherLink()
{
    delete link1;
    delete link2;

    delete int1;
    delete int2;
}

EtherLink::Interface::Interface(const std::string &name, Link *tx, Link *rx)
    : EtherInt(name), txlink(tx)
{
    tx->setTxInt(this);
    rx->setRxInt(this);
}

EtherLink::Link::Link(const std::string &name, double rate, EtherDump *d)
    : objName(name), txint(NULL), rxint(NULL), ticks_per_byte(rate),
      dump(d), event(&mainEventQueue, this)
{}

void
EtherLink::serialize(ostream &os)
{
    nameOut(os, name() + ".link1");
    link1->serialize(os);
    nameOut(os, name() + ".link2");
    link2->serialize(os);
}

void
EtherLink::unserialize(Checkpoint *cp, const string &section)
{
    link1->unserialize(cp, section + ".link1");
    link2->unserialize(cp, section + ".link2");
}

void
EtherLink::Link::txDone()
{
    if (dump)
        dump->dump(packet);

    DPRINTF(Ethernet, "EtherLink packet received: len=%d\n", packet->length);
    DDUMP(EthernetData, packet->data, packet->length);

    rxint->sendPacket(packet);

    packet = 0;
    assert(!busy());

    txint->sendDone();
}

bool
EtherLink::Link::transmit(PacketPtr &pkt)
{
    if (busy()) {
        DPRINTF(Ethernet, "EtherLink packet not sent, link busy\n");
        return false;
    }

    DPRINTF(Ethernet, "EtherLink packet sent: len=%d\n", pkt->length);
    DDUMP(EthernetData, pkt->data, pkt->length);

    packet = pkt;
    int delay = (int)ceil(((double)pkt->length * ticks_per_byte) + 1.0);
    DPRINTF(Ethernet, "EtherLink scheduling packet: delay=%d, (rate=%f)\n",
            delay, ticks_per_byte);
    event.schedule(curTick + delay);

    return true;
}

void
EtherLink::Link::serialize(ostream &os)
{
    bool packet_exists = packet;
    SERIALIZE_SCALAR(packet_exists);

    bool event_scheduled = event.scheduled();
    SERIALIZE_SCALAR(event_scheduled);
    if (event_scheduled) {
        Tick event_time = event.when();
        SERIALIZE_SCALAR(event_time);
    }

    if (packet_exists) {
        nameOut(os, csprintf("%s.packet", name()));
        packet->serialize(os);
    }
}

void
EtherLink::Link::unserialize(Checkpoint *cp, const string &section)
{
    bool packet_exists;
    UNSERIALIZE_SCALAR(packet_exists);
    if (packet_exists) {
        packet = new EtherPacket;
        packet->unserialize(cp, csprintf("%s.packet", section));
    }

    bool event_scheduled;
    UNSERIALIZE_SCALAR(event_scheduled);
    if (event_scheduled) {
        Tick event_time;
        UNSERIALIZE_SCALAR(event_time);
        event.schedule(event_time);
    }
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(EtherLink)

    SimObjectParam<EtherInt *> interface1;
    SimObjectParam<EtherInt *> interface2;
    Param<Tick> link_speed;
    SimObjectParam<EtherDump *> packet_dump;

END_DECLARE_SIM_OBJECT_PARAMS(EtherLink)

BEGIN_INIT_SIM_OBJECT_PARAMS(EtherLink)

    INIT_PARAM(interface1, "interface 1"),
    INIT_PARAM(interface2, "interface 2"),
    INIT_PARAM_DFLT(link_speed, "link speed in bits per second", 100000000),
    INIT_PARAM_DFLT(packet_dump, "object to dump network packets to", NULL)

END_INIT_SIM_OBJECT_PARAMS(EtherLink)

CREATE_SIM_OBJECT(EtherLink)
{
    return new EtherLink(getInstanceName(), interface1, interface2, link_speed,
                         packet_dump);
}

REGISTER_SIM_OBJECT("EtherLink", EtherLink)
