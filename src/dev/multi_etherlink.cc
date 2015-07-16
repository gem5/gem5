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
 * Authors: Gabor Dozsa
 */

/* @file
 * Device module for a full duplex ethernet link for multi gem5 simulations.
 */

#include "dev/multi_etherlink.hh"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/EthernetData.hh"
#include "debug/MultiEthernet.hh"
#include "debug/MultiEthernetPkt.hh"
#include "dev/etherdump.hh"
#include "dev/etherint.hh"
#include "dev/etherlink.hh"
#include "dev/etherobject.hh"
#include "dev/etherpkt.hh"
#include "dev/multi_iface.hh"
#include "dev/tcp_iface.hh"
#include "params/EtherLink.hh"
#include "sim/core.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

using namespace std;

MultiEtherLink::MultiEtherLink(const Params *p)
    : EtherObject(p)
{
    DPRINTF(MultiEthernet,"MultiEtherLink::MultiEtherLink() "
            "link delay:%llu\n", p->delay);

    txLink = new TxLink(name() + ".link0", this, p->speed, p->delay_var,
                        p->dump);
    rxLink = new RxLink(name() + ".link1", this, p->delay, p->dump);

    // create the multi (TCP) interface to talk to the peer gem5 processes.
    multiIface = new TCPIface(p->server_name, p->server_port, p->multi_rank,
                              p->sync_start, p->sync_repeat, this);

    localIface = new LocalIface(name() + ".int0", txLink, rxLink, multiIface);
}

MultiEtherLink::~MultiEtherLink()
{
    delete txLink;
    delete rxLink;
    delete localIface;
    delete multiIface;
}

EtherInt*
MultiEtherLink::getEthPort(const std::string &if_name, int idx)
{
    if (if_name != "int0") {
        return nullptr;
    } else {
        panic_if(localIface->getPeer(), "interface already connected to");
    }
    return localIface;
}

void MultiEtherLink::memWriteback()
{
    DPRINTF(MultiEthernet,"MultiEtherLink::memWriteback() called\n");
    multiIface->drainDone();
}

void
MultiEtherLink::serialize(CheckpointOut &cp) const
{
    multiIface->serialize("multiIface", cp);
    txLink->serialize("txLink", cp);
    rxLink->serialize("rxLink", cp);
}

void
MultiEtherLink::unserialize(CheckpointIn &cp)
{
    multiIface->unserialize("multiIface", cp);
    txLink->unserialize("txLink", cp);
    rxLink->unserialize("rxLink", cp);
}

void
MultiEtherLink::init()
{
    DPRINTF(MultiEthernet,"MultiEtherLink::init() called\n");
    multiIface->initRandom();
}

void
MultiEtherLink::startup()
{
    DPRINTF(MultiEthernet,"MultiEtherLink::startup() called\n");
    multiIface->startPeriodicSync();
}

void
MultiEtherLink::RxLink::setMultiInt(MultiIface *m)
{
    assert(!multiIface);
    multiIface = m;
    // Spawn a new receiver thread that will process messages
    // coming in from peer gem5 processes.
    // The receive thread will also schedule a (receive) doneEvent
    // for each incoming data packet.
    multiIface->spawnRecvThread(&doneEvent, linkDelay);
}

void
MultiEtherLink::RxLink::rxDone()
{
    assert(!busy());

    // retrieve the packet that triggered the receive done event
    packet = multiIface->packetIn();

    if (dump)
        dump->dump(packet);

    DPRINTF(MultiEthernetPkt, "MultiEtherLink::MultiLink::rxDone() "
            "packet received: len=%d\n", packet->length);
    DDUMP(EthernetData, packet->data, packet->length);

    localIface->sendPacket(packet);

    packet = nullptr;
}

void
MultiEtherLink::TxLink::txDone()
{
    if (dump)
        dump->dump(packet);

    packet = nullptr;
    assert(!busy());

    localIface->sendDone();
}

bool
MultiEtherLink::TxLink::transmit(EthPacketPtr pkt)
{
    if (busy()) {
        DPRINTF(MultiEthernet, "packet not sent, link busy\n");
        return false;
    }

    packet = pkt;
    Tick delay = (Tick)ceil(((double)pkt->length * ticksPerByte) + 1.0);
    if (delayVar != 0)
        delay += random_mt.random<Tick>(0, delayVar);

    // send the packet to the peers
    assert(multiIface);
    multiIface->packetOut(pkt, delay);

    // schedule the send done event
    parent->schedule(doneEvent, curTick() + delay);

    return true;
}

void
MultiEtherLink::Link::serialize(const string &base, CheckpointOut &cp) const
{
    bool packet_exists = (packet != nullptr);
    paramOut(cp, base + ".packet_exists", packet_exists);
    if (packet_exists)
        packet->serialize(base + ".packet", cp);

    bool event_scheduled = event->scheduled();
    paramOut(cp, base + ".event_scheduled", event_scheduled);
    if (event_scheduled) {
        Tick event_time = event->when();
        paramOut(cp, base + ".event_time", event_time);
    }
}

void
MultiEtherLink::Link::unserialize(const string &base, CheckpointIn &cp)
{
    bool packet_exists;
    paramIn(cp, base + ".packet_exists", packet_exists);
    if (packet_exists) {
        packet = make_shared<EthPacketData>(16384);
        packet->unserialize(base + ".packet", cp);
    }

    bool event_scheduled;
    paramIn(cp, base + ".event_scheduled", event_scheduled);
    if (event_scheduled) {
        Tick event_time;
        paramIn(cp, base + ".event_time", event_time);
        parent->schedule(*event, event_time);
    }
}

MultiEtherLink::LocalIface::LocalIface(const std::string &name,
                                       TxLink *tx,
                                       RxLink *rx,
                                       MultiIface *m) :
    EtherInt(name), txLink(tx)
{
    tx->setLocalInt(this);
    rx->setLocalInt(this);
    tx->setMultiInt(m);
    rx->setMultiInt(m);
}

MultiEtherLink *
MultiEtherLinkParams::create()
{
    return new MultiEtherLink(this);
}


