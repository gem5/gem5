/*
 * Copyright (c) 2015 ARM Limited
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

#include "dev/net/etherlink.hh"

#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/Ethernet.hh"
#include "debug/EthernetData.hh"
#include "dev/net/etherdump.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
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
      doneEvent([this]{ txDone(); }, name),
      txQueueEvent([this]{ processTxQueue(); }, name)
{ }

void
EtherLink::serialize(CheckpointOut &cp) const
{
    link[0]->serialize("link0", cp);
    link[1]->serialize("link1", cp);
}

void
EtherLink::unserialize(CheckpointIn &cp)
{
    link[0]->unserialize("link0", cp);
    link[1]->unserialize("link1", cp);
}

void
EtherLink::Link::txComplete(EthPacketPtr packet)
{
    DPRINTF(Ethernet, "packet received: len=%d\n", packet->length);
    DDUMP(EthernetData, packet->data, packet->length);
    rxint->sendPacket(packet);
}

void
EtherLink::Link::txDone()
{
    if (dump)
        dump->dump(packet);

    if (linkDelay > 0) {
        DPRINTF(Ethernet, "packet delayed: delay=%d\n", linkDelay);
        txQueue.emplace_back(std::make_pair(curTick() + linkDelay, packet));
        if (!txQueueEvent.scheduled())
            parent->schedule(txQueueEvent, txQueue.front().first);
    } else {
        assert(txQueue.empty());
        txComplete(packet);
    }

    packet = 0;
    assert(!busy());

    txint->sendDone();
}

void
EtherLink::Link::processTxQueue()
{
    auto cur(txQueue.front());
    txQueue.pop_front();

    // Schedule a new event to process the next packet in the queue.
    if (!txQueue.empty()) {
        auto next(txQueue.front());
        assert(next.first > curTick());
        parent->schedule(txQueueEvent, next.first);
    }

    assert(cur.first == curTick());
    txComplete(cur.second);
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
    Tick delay = (Tick)ceil(((double)pkt->simLength * ticksPerByte) + 1.0);
    if (delayVar != 0)
        delay += random_mt.random<Tick>(0, delayVar);

    DPRINTF(Ethernet, "scheduling packet: delay=%d, (rate=%f)\n",
            delay, ticksPerByte);
    parent->schedule(doneEvent, curTick() + delay);

    return true;
}

void
EtherLink::Link::serialize(const string &base, CheckpointOut &cp) const
{
    bool packet_exists = packet != nullptr;
    paramOut(cp, base + ".packet_exists", packet_exists);
    if (packet_exists)
        packet->serialize(base + ".packet", cp);

    bool event_scheduled = doneEvent.scheduled();
    paramOut(cp, base + ".event_scheduled", event_scheduled);
    if (event_scheduled) {
        Tick event_time = doneEvent.when();
        paramOut(cp, base + ".event_time", event_time);
    }

    const size_t tx_queue_size(txQueue.size());
    paramOut(cp, base + ".tx_queue_size", tx_queue_size);
    unsigned idx(0);
    for (const auto &pe : txQueue) {
        paramOut(cp, csprintf("%s.txQueue[%i].tick", base, idx), pe.first);
        pe.second->serialize(csprintf("%s.txQueue[%i].packet", base, idx), cp);

        ++idx;
    }
}

void
EtherLink::Link::unserialize(const string &base, CheckpointIn &cp)
{
    bool packet_exists;
    paramIn(cp, base + ".packet_exists", packet_exists);
    if (packet_exists) {
        packet = make_shared<EthPacketData>();
        packet->unserialize(base + ".packet", cp);
    }

    bool event_scheduled;
    paramIn(cp, base + ".event_scheduled", event_scheduled);
    if (event_scheduled) {
        Tick event_time;
        paramIn(cp, base + ".event_time", event_time);
        parent->schedule(doneEvent, event_time);
    }

    size_t tx_queue_size;
    if (optParamIn(cp, base + ".tx_queue_size", tx_queue_size)) {
        for (size_t idx = 0; idx < tx_queue_size; ++idx) {
            Tick tick;
            EthPacketPtr delayed_packet = make_shared<EthPacketData>();

            paramIn(cp, csprintf("%s.txQueue[%i].tick", base, idx), tick);
            delayed_packet->unserialize(
                csprintf("%s.txQueue[%i].packet", base, idx), cp);

            fatal_if(!txQueue.empty() && txQueue.back().first > tick,
                     "Invalid txQueue packet order in EtherLink!\n");
            txQueue.emplace_back(std::make_pair(tick, delayed_packet));
        }

        if (!txQueue.empty())
            parent->schedule(txQueueEvent, txQueue.front().first);
    } else {
        // We can't reliably convert in-flight packets from old
        // checkpoints. In fact, gem5 hasn't been able to load these
        // packets for at least two years before the format change.
        warn("Old-style EtherLink serialization format detected, "
             "in-flight packets may have been dropped.\n");
    }
}

EtherLink *
EtherLinkParams::create()
{
    return new EtherLink(this);
}
