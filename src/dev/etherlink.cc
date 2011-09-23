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
 *          Ron Dreslinski
 */

/* @file
 * Device module for modelling a fixed bandwidth full duplex ethernet link
 */

#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/Ethernet.hh"
#include "debug/EthernetData.hh"
#include "dev/etherdump.hh"
#include "dev/etherint.hh"
#include "dev/etherlink.hh"
#include "dev/etherpkt.hh"
#include "params/EtherLink.hh"
#include "sim/core.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

using namespace std;

EtherLink::EtherLink(const Params *p)
    : EtherObject(p)
{
    link[0] = new Link(name() + ".link0", this, 0, p->speed,
                       p->delay, p->delay_var, p->dump);
    link[1] = new Link(name() + ".link1", this, 1, p->speed,
                       p->delay, p->delay_var, p->dump);

    interface[0] = new Interface(name() + ".int0", link[0], link[1]);
    interface[1] = new Interface(name() + ".int1", link[1], link[0]);
}


EtherLink::~EtherLink()
{
    delete link[0];
    delete link[1];

    delete interface[0];
    delete interface[1];
}

EtherInt*
EtherLink::getEthPort(const std::string &if_name, int idx)
{
    Interface *i;
    if (if_name == "int0")
        i = interface[0];
    else if (if_name == "int1")
        i = interface[1];
    else
        return NULL;
    if (i->getPeer())
        panic("interface already connected to\n");

    return i;
}


EtherLink::Interface::Interface(const string &name, Link *tx, Link *rx)
    : EtherInt(name), txlink(tx)
{
    tx->setTxInt(this);
    rx->setRxInt(this);
}

EtherLink::Link::Link(const string &name, EtherLink *p, int num,
                      double rate, Tick delay, Tick delay_var, EtherDump *d)
    : objName(name), parent(p), number(num), txint(NULL), rxint(NULL),
      ticksPerByte(rate), linkDelay(delay), delayVar(delay_var), dump(d),
      doneEvent(this)
{ }

void
EtherLink::serialize(ostream &os)
{
    link[0]->serialize("link0", os);
    link[1]->serialize("link1", os);
}

void
EtherLink::unserialize(Checkpoint *cp, const string &section)
{
    link[0]->unserialize("link0", cp, section);
    link[1]->unserialize("link1", cp, section);
}

void
EtherLink::Link::txComplete(EthPacketPtr packet)
{
    DPRINTF(Ethernet, "packet received: len=%d\n", packet->length);
    DDUMP(EthernetData, packet->data, packet->length);
    rxint->sendPacket(packet);
}

class LinkDelayEvent : public Event
{
  protected:
    EtherLink::Link *link;
    EthPacketPtr packet;

  public:
    // non-scheduling version for createForUnserialize()
    LinkDelayEvent();
    LinkDelayEvent(EtherLink::Link *link, EthPacketPtr pkt);

    void process();

    virtual void serialize(ostream &os);
    virtual void unserialize(Checkpoint *cp, const string &section);
    static Serializable *createForUnserialize(Checkpoint *cp,
                                              const string &section);
};

void
EtherLink::Link::txDone()
{
    if (dump)
        dump->dump(packet);

    if (linkDelay > 0) {
        DPRINTF(Ethernet, "packet delayed: delay=%d\n", linkDelay);
        Event *event = new LinkDelayEvent(this, packet);
        parent->schedule(event, curTick() + linkDelay);
    } else {
        txComplete(packet);
    }

    packet = 0;
    assert(!busy());

    txint->sendDone();
}

bool
EtherLink::Link::transmit(EthPacketPtr pkt)
{
    if (busy()) {
        DPRINTF(Ethernet, "packet not sent, link busy\n");
        return false;
    }

    DPRINTF(Ethernet, "packet sent: len=%d\n", pkt->length);
    DDUMP(EthernetData, pkt->data, pkt->length);

    packet = pkt;
    Tick delay = (Tick)ceil(((double)pkt->length * ticksPerByte) + 1.0);
    if (delayVar != 0)
        delay += random_mt.random<Tick>(0, delayVar);

    DPRINTF(Ethernet, "scheduling packet: delay=%d, (rate=%f)\n",
            delay, ticksPerByte);
    parent->schedule(doneEvent, curTick() + delay);

    return true;
}

void
EtherLink::Link::serialize(const string &base, ostream &os)
{
    bool packet_exists = packet;
    paramOut(os, base + ".packet_exists", packet_exists);
    if (packet_exists)
        packet->serialize(base + ".packet", os);

    bool event_scheduled = doneEvent.scheduled();
    paramOut(os, base + ".event_scheduled", event_scheduled);
    if (event_scheduled) {
        Tick event_time = doneEvent.when();
        paramOut(os, base + ".event_time", event_time);
    }

}

void
EtherLink::Link::unserialize(const string &base, Checkpoint *cp,
                             const string &section)
{
    bool packet_exists;
    paramIn(cp, section, base + ".packet_exists", packet_exists);
    if (packet_exists) {
        packet = new EthPacketData(16384);
        packet->unserialize(base + ".packet", cp, section);
    }

    bool event_scheduled;
    paramIn(cp, section, base + ".event_scheduled", event_scheduled);
    if (event_scheduled) {
        Tick event_time;
        paramIn(cp, section, base + ".event_time", event_time);
        parent->schedule(doneEvent, event_time);
    }
}

LinkDelayEvent::LinkDelayEvent()
    : Event(Default_Pri, AutoSerialize | AutoDelete), link(NULL)
{
}

LinkDelayEvent::LinkDelayEvent(EtherLink::Link *l, EthPacketPtr p)
    : Event(Default_Pri, AutoSerialize | AutoDelete), link(l), packet(p)
{
}

void
LinkDelayEvent::process()
{
    link->txComplete(packet);
}

void
LinkDelayEvent::serialize(ostream &os)
{
    paramOut(os, "type", string("LinkDelayEvent"));
    Event::serialize(os);

    EtherLink *parent = link->parent;
    bool number = link->number;
    SERIALIZE_OBJPTR(parent);
    SERIALIZE_SCALAR(number);

    packet->serialize("packet", os);
}


void
LinkDelayEvent::unserialize(Checkpoint *cp, const string &section)
{
    Event::unserialize(cp, section);

    EtherLink *parent;
    bool number;
    UNSERIALIZE_OBJPTR(parent);
    UNSERIALIZE_SCALAR(number);

    link = parent->link[number];

    packet = new EthPacketData(16384);
    packet->unserialize("packet", cp, section);
}


Serializable *
LinkDelayEvent::createForUnserialize(Checkpoint *cp, const string &section)
{
    return new LinkDelayEvent();
}

REGISTER_SERIALIZEABLE("LinkDelayEvent", LinkDelayEvent)

EtherLink *
EtherLinkParams::create()
{
    return new EtherLink(this);
}
