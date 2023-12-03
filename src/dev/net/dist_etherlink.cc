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
 */

/* @file
 * Device module for a full duplex ethernet link for dist gem5 simulations.
 */

#include "dev/net/dist_etherlink.hh"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/DistEthernet.hh"
#include "debug/DistEthernetPkt.hh"
#include "debug/EthernetData.hh"
#include "dev/net/dist_iface.hh"
#include "dev/net/etherdump.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherlink.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/tcp_iface.hh"
#include "params/EtherLink.hh"
#include "sim/cur_tick.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

namespace gem5
{

DistEtherLink::DistEtherLink(const Params &p)
    : SimObject(p), linkDelay(p.delay)
{
    DPRINTF(DistEthernet,
            "DistEtherLink::DistEtherLink() "
            "link delay:%llu ticksPerByte:%f\n",
            p.delay, p.speed);

    txLink = new TxLink(name() + ".link0", this, p.speed, p.delay_var, p.dump);
    rxLink = new RxLink(name() + ".link1", this, p.delay, p.dump);

    Tick sync_repeat;
    if (p.sync_repeat != 0) {
        if (p.sync_repeat != p.delay)
            warn("DistEtherLink(): sync_repeat is %lu and linkdelay is %lu",
                 p.sync_repeat, p.delay);
        sync_repeat = p.sync_repeat;
    } else {
        sync_repeat = p.delay;
    }

    // create the dist (TCP) interface to talk to the peer gem5 processes.
    distIface = new TCPIface(
        p.server_name, p.server_port, p.dist_rank, p.dist_size, p.sync_start,
        sync_repeat, this, p.dist_sync_on_pseudo_op, p.is_switch, p.num_nodes);

    localIface = new LocalIface(name() + ".int0", txLink, rxLink, distIface);
}

DistEtherLink::~DistEtherLink()
{
    delete txLink;
    delete rxLink;
    delete localIface;
    delete distIface;
}

Port &
DistEtherLink::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "int0")
        return *localIface;
    return SimObject::getPort(if_name, idx);
}

void
DistEtherLink::serialize(CheckpointOut &cp) const
{
    distIface->serializeSection(cp, "distIface");
    txLink->serializeSection(cp, "txLink");
    rxLink->serializeSection(cp, "rxLink");
}

void
DistEtherLink::unserialize(CheckpointIn &cp)
{
    distIface->unserializeSection(cp, "distIface");
    txLink->unserializeSection(cp, "txLink");
    rxLink->unserializeSection(cp, "rxLink");
}

void
DistEtherLink::init()
{
    DPRINTF(DistEthernet, "DistEtherLink::init() called\n");
    distIface->init(rxLink->doneEvent(), linkDelay);
}

void
DistEtherLink::startup()
{
    DPRINTF(DistEthernet, "DistEtherLink::startup() called\n");
    distIface->startup();
}

void
DistEtherLink::RxLink::setDistInt(DistIface *m)
{
    assert(!distIface);
    distIface = m;
}

void
DistEtherLink::RxLink::rxDone()
{
    assert(!busy());

    // retrieve the packet that triggered the receive done event
    packet = distIface->packetIn();

    if (dump)
        dump->dump(packet);

    DPRINTF(DistEthernetPkt,
            "DistEtherLink::DistLink::rxDone() "
            "packet received: len=%d\n",
            packet->length);
    DDUMP(EthernetData, packet->data, packet->length);

    localIface->sendPacket(packet);

    packet = nullptr;
}

void
DistEtherLink::TxLink::txDone()
{
    if (dump)
        dump->dump(packet);

    packet = nullptr;
    assert(!busy());

    localIface->sendDone();
}

bool
DistEtherLink::TxLink::transmit(EthPacketPtr pkt)
{
    if (busy()) {
        DPRINTF(DistEthernet, "packet not sent, link busy\n");
        return false;
    }

    packet = pkt;
    Tick delay = (Tick)ceil(((double)pkt->simLength * ticksPerByte) + 1.0);
    if (delayVar != 0)
        delay += random_mt.random<Tick>(0, delayVar);

    // send the packet to the peers
    assert(distIface);
    distIface->packetOut(pkt, delay);

    // schedule the send done event
    parent->schedule(doneEvent, curTick() + delay);

    return true;
}

void
DistEtherLink::Link::serialize(CheckpointOut &cp) const
{
    bool packet_exists = (packet != nullptr);
    SERIALIZE_SCALAR(packet_exists);
    if (packet_exists)
        packet->serialize("packet", cp);

    bool event_scheduled = event->scheduled();
    SERIALIZE_SCALAR(event_scheduled);
    if (event_scheduled) {
        Tick event_time = event->when();
        SERIALIZE_SCALAR(event_time);
    }
}

void
DistEtherLink::Link::unserialize(CheckpointIn &cp)
{
    bool packet_exists;
    UNSERIALIZE_SCALAR(packet_exists);
    if (packet_exists) {
        packet = std::make_shared<EthPacketData>();
        packet->unserialize("packet", cp);
    }

    bool event_scheduled;
    UNSERIALIZE_SCALAR(event_scheduled);
    if (event_scheduled) {
        Tick event_time;
        UNSERIALIZE_SCALAR(event_time);
        parent->schedule(*event, event_time);
    }
}

DistEtherLink::LocalIface::LocalIface(const std::string &name, TxLink *tx,
                                      RxLink *rx, DistIface *m)
    : EtherInt(name), txLink(tx)
{
    tx->setLocalInt(this);
    rx->setLocalInt(this);
    tx->setDistInt(m);
    rx->setDistInt(m);
}

} // namespace gem5
