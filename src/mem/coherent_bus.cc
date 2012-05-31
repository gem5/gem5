/*
 * Copyright (c) 2011-2012 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 *          Andreas Hansson
 *          William Wang
 */

/**
 * @file
 * Definition of a bus object.
 */

#include "base/misc.hh"
#include "base/trace.hh"
#include "debug/BusAddrRanges.hh"
#include "debug/CoherentBus.hh"
#include "mem/coherent_bus.hh"

CoherentBus::CoherentBus(const CoherentBusParams *p)
    : BaseBus(p)
{
    // create the ports based on the size of the master and slave
    // vector ports, and the presence of the default port, the ports
    // are enumerated starting from zero
    for (int i = 0; i < p->port_master_connection_count; ++i) {
        std::string portName = csprintf("%s-p%d", name(), i);
        MasterPort* bp = new CoherentBusMasterPort(portName, *this, i);
        masterPorts.push_back(bp);
    }

    // see if we have a default slave device connected and if so add
    // our corresponding master port
    if (p->port_default_connection_count) {
        defaultPortID = masterPorts.size();
        std::string portName = csprintf("%s-default", name());
        MasterPort* bp = new CoherentBusMasterPort(portName, *this,
                                                   defaultPortID);
        masterPorts.push_back(bp);
    }

    // create the slave ports, once again starting at zero
    for (int i = 0; i < p->port_slave_connection_count; ++i) {
        std::string portName = csprintf("%s-p%d", name(), i);
        SlavePort* bp = new CoherentBusSlavePort(portName, *this, i);
        slavePorts.push_back(bp);
    }

    clearPortCache();
}

void
CoherentBus::init()
{
    // iterate over our slave ports and determine which of our
    // neighbouring master ports are snooping and add them as snoopers
    for (SlavePortConstIter p = slavePorts.begin(); p != slavePorts.end();
         ++p) {
        if ((*p)->getMasterPort().isSnooping()) {
            DPRINTF(BusAddrRanges, "Adding snooping master %s\n",
                    (*p)->getMasterPort().name());
            snoopPorts.push_back(*p);
        }
    }

    if (snoopPorts.empty())
        warn("CoherentBus %s has no snooping ports attached!\n", name());
}

bool
CoherentBus::recvTimingReq(PacketPtr pkt, PortID slave_port_id)
{
    // determine the source port based on the id
    SlavePort *src_port = slavePorts[slave_port_id];

    // test if the bus should be considered occupied for the current
    // port, and exclude express snoops from the check
    if (!pkt->isExpressSnoop() && isOccupied(src_port)) {
        DPRINTF(CoherentBus, "recvTimingReq: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(CoherentBus, "recvTimingReq: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    // set the source port for routing of the response
    pkt->setSrc(slave_port_id);

    Tick headerFinishTime = pkt->isExpressSnoop() ? 0 : calcPacketTiming(pkt);
    Tick packetFinishTime = pkt->isExpressSnoop() ? 0 : pkt->finishTime;

    // uncacheable requests need never be snooped
    if (!pkt->req->isUncacheable()) {
        // the packet is a memory-mapped request and should be
        // broadcasted to our snoopers but the source
        forwardTiming(pkt, slave_port_id);
    }

    // remember if we add an outstanding req so we can undo it if
    // necessary, if the packet needs a response, we should add it
    // as outstanding and express snoops never fail so there is
    // not need to worry about them
    bool add_outstanding = !pkt->isExpressSnoop() && pkt->needsResponse();

    // keep track that we have an outstanding request packet
    // matching this request, this is used by the coherency
    // mechanism in determining what to do with snoop responses
    // (in recvTimingSnoop)
    if (add_outstanding) {
        // we should never have an exsiting request outstanding
        assert(outstandingReq.find(pkt->req) == outstandingReq.end());
        outstandingReq.insert(pkt->req);
    }

    // since it is a normal request, determine the destination
    // based on the address and attempt to send the packet
    bool success = masterPorts[findPort(pkt->getAddr())]->sendTimingReq(pkt);

    if (!success)  {
        // inhibited packets should never be forced to retry
        assert(!pkt->memInhibitAsserted());

        // if it was added as outstanding and the send failed, then
        // erase it again
        if (add_outstanding)
            outstandingReq.erase(pkt->req);

        DPRINTF(CoherentBus, "recvTimingReq: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

        addToRetryList(src_port);
        occupyBus(headerFinishTime);

        return false;
    }

    succeededTiming(packetFinishTime);

    return true;
}

bool
CoherentBus::recvTimingResp(PacketPtr pkt, PortID master_port_id)
{
    // determine the source port based on the id
    MasterPort *src_port = masterPorts[master_port_id];

    // test if the bus should be considered occupied for the current
    // port
    if (isOccupied(src_port)) {
        DPRINTF(CoherentBus, "recvTimingResp: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(CoherentBus, "recvTimingResp: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    calcPacketTiming(pkt);
    Tick packetFinishTime = pkt->finishTime;

    // the packet is a normal response to a request that we should
    // have seen passing through the bus
    assert(outstandingReq.find(pkt->req) != outstandingReq.end());

    // remove it as outstanding
    outstandingReq.erase(pkt->req);

    // send the packet to the destination through one of our slave
    // ports, as determined by the destination field
    bool success M5_VAR_USED = slavePorts[pkt->getDest()]->sendTimingResp(pkt);

    // currently it is illegal to block responses... can lead to
    // deadlock
    assert(success);

    succeededTiming(packetFinishTime);

    return true;
}

void
CoherentBus::recvTimingSnoopReq(PacketPtr pkt, PortID master_port_id)
{
    DPRINTF(CoherentBus, "recvTimingSnoopReq: src %s %s 0x%x\n",
            masterPorts[master_port_id]->name(), pkt->cmdString(),
            pkt->getAddr());

    // we should only see express snoops from caches
    assert(pkt->isExpressSnoop());

    // set the source port for routing of the response
    pkt->setSrc(master_port_id);

    // forward to all snoopers
    forwardTiming(pkt, InvalidPortID);

    // a snoop request came from a connected slave device (one of
    // our master ports), and if it is not coming from the slave
    // device responsible for the address range something is
    // wrong, hence there is nothing further to do as the packet
    // would be going back to where it came from
    assert(master_port_id == findPort(pkt->getAddr()));

    // this is an express snoop and is never forced to retry
    assert(!inRetry);
}

bool
CoherentBus::recvTimingSnoopResp(PacketPtr pkt, PortID slave_port_id)
{
    // determine the source port based on the id
    SlavePort* src_port = slavePorts[slave_port_id];

    // test if the bus should be considered occupied for the current
    // port
    if (isOccupied(src_port)) {
        DPRINTF(CoherentBus, "recvTimingSnoopResp: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(CoherentBus, "recvTimingSnoop: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    // get the destination from the packet
    PortID dest = pkt->getDest();

    // responses are never express snoops
    assert(!pkt->isExpressSnoop());

    calcPacketTiming(pkt);
    Tick packetFinishTime = pkt->finishTime;

    // determine if the response is from a snoop request we
    // created as the result of a normal request (in which case it
    // should be in the outstandingReq), or if we merely forwarded
    // someone else's snoop request
    if (outstandingReq.find(pkt->req) == outstandingReq.end()) {
        // this is a snoop response to a snoop request we
        // forwarded, e.g. coming from the L1 and going to the L2
        // this should be forwarded as a snoop response
        bool success M5_VAR_USED = masterPorts[dest]->sendTimingSnoopResp(pkt);
        assert(success);
    } else {
        // we got a snoop response on one of our slave ports,
        // i.e. from a coherent master connected to the bus, and
        // since we created the snoop request as part of
        // recvTiming, this should now be a normal response again
        outstandingReq.erase(pkt->req);

        // this is a snoop response from a coherent master, with a
        // destination field set on its way through the bus as
        // request, hence it should never go back to where the
        // snoop response came from, but instead to where the
        // original request came from
        assert(slave_port_id != dest);

        // as a normal response, it should go back to a master
        // through one of our slave ports
        bool success M5_VAR_USED = slavePorts[dest]->sendTimingResp(pkt);

        // currently it is illegal to block responses... can lead
        // to deadlock
        assert(success);
    }

    succeededTiming(packetFinishTime);

    return true;
}


void
CoherentBus::forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id)
{
    for (SlavePortIter s = snoopPorts.begin(); s != snoopPorts.end(); ++s) {
        SlavePort *p = *s;
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (exclude_slave_port_id == InvalidPortID ||
            p->getId() != exclude_slave_port_id) {
            // cache is not allowed to refuse snoop
            p->sendTimingSnoopReq(pkt);
        }
    }
}

Tick
CoherentBus::recvAtomic(PacketPtr pkt, PortID slave_port_id)
{
    DPRINTF(CoherentBus, "recvAtomic: packet src %s addr 0x%x cmd %s\n",
            slavePorts[slave_port_id]->name(), pkt->getAddr(),
            pkt->cmdString());

    MemCmd snoop_response_cmd = MemCmd::InvalidCmd;
    Tick snoop_response_latency = 0;

    // uncacheable requests need never be snooped
    if (!pkt->req->isUncacheable()) {
        // forward to all snoopers but the source
        std::pair<MemCmd, Tick> snoop_result =
            forwardAtomic(pkt, slave_port_id);
        snoop_response_cmd = snoop_result.first;
        snoop_response_latency = snoop_result.second;
    }

    // even if we had a snoop response, we must continue and also
    // perform the actual request at the destination
    PortID dest_id = findPort(pkt->getAddr());

    // forward the request to the appropriate destination
    Tick response_latency = masterPorts[dest_id]->sendAtomic(pkt);

    // if we got a response from a snooper, restore it here
    if (snoop_response_cmd != MemCmd::InvalidCmd) {
        // no one else should have responded
        assert(!pkt->isResponse());
        pkt->cmd = snoop_response_cmd;
        response_latency = snoop_response_latency;
    }

    pkt->finishTime = curTick() + response_latency;
    return response_latency;
}

Tick
CoherentBus::recvAtomicSnoop(PacketPtr pkt, PortID master_port_id)
{
    DPRINTF(CoherentBus, "recvAtomicSnoop: packet src %s addr 0x%x cmd %s\n",
            masterPorts[master_port_id]->name(), pkt->getAddr(),
            pkt->cmdString());

    // forward to all snoopers
    std::pair<MemCmd, Tick> snoop_result =
        forwardAtomic(pkt, InvalidPortID);
    MemCmd snoop_response_cmd = snoop_result.first;
    Tick snoop_response_latency = snoop_result.second;

    if (snoop_response_cmd != MemCmd::InvalidCmd)
        pkt->cmd = snoop_response_cmd;

    pkt->finishTime = curTick() + snoop_response_latency;
    return snoop_response_latency;
}

std::pair<MemCmd, Tick>
CoherentBus::forwardAtomic(PacketPtr pkt, PortID exclude_slave_port_id)
{
    // the packet may be changed on snoops, record the original
    // command to enable us to restore it between snoops so that
    // additional snoops can take place properly
    MemCmd orig_cmd = pkt->cmd;
    MemCmd snoop_response_cmd = MemCmd::InvalidCmd;
    Tick snoop_response_latency = 0;

    for (SlavePortIter s = snoopPorts.begin(); s != snoopPorts.end(); ++s) {
        SlavePort *p = *s;
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (exclude_slave_port_id == InvalidPortID ||
            p->getId() != exclude_slave_port_id) {
            Tick latency = p->sendAtomicSnoop(pkt);
            // in contrast to a functional access, we have to keep on
            // going as all snoopers must be updated even if we get a
            // response
            if (pkt->isResponse()) {
                // response from snoop agent
                assert(pkt->cmd != orig_cmd);
                assert(pkt->memInhibitAsserted());
                // should only happen once
                assert(snoop_response_cmd == MemCmd::InvalidCmd);
                // save response state
                snoop_response_cmd = pkt->cmd;
                snoop_response_latency = latency;
                // restore original packet state for remaining snoopers
                pkt->cmd = orig_cmd;
            }
        }
    }

    // the packet is restored as part of the loop and any potential
    // snoop response is part of the returned pair
    return std::make_pair(snoop_response_cmd, snoop_response_latency);
}

void
CoherentBus::recvFunctional(PacketPtr pkt, PortID slave_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(CoherentBus,
                "recvFunctional: packet src %s addr 0x%x cmd %s\n",
                slavePorts[slave_port_id]->name(), pkt->getAddr(),
                pkt->cmdString());
    }

    // uncacheable requests need never be snooped
    if (!pkt->req->isUncacheable()) {
        // forward to all snoopers but the source
        forwardFunctional(pkt, slave_port_id);
    }

    // there is no need to continue if the snooping has found what we
    // were looking for and the packet is already a response
    if (!pkt->isResponse()) {
        PortID dest_id = findPort(pkt->getAddr());

        masterPorts[dest_id]->sendFunctional(pkt);
    }
}

void
CoherentBus::recvFunctionalSnoop(PacketPtr pkt, PortID master_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(CoherentBus,
                "recvFunctionalSnoop: packet src %s addr 0x%x cmd %s\n",
                masterPorts[master_port_id]->name(), pkt->getAddr(),
                pkt->cmdString());
    }

    // forward to all snoopers
    forwardFunctional(pkt, InvalidPortID);
}

void
CoherentBus::forwardFunctional(PacketPtr pkt, PortID exclude_slave_port_id)
{
    for (SlavePortIter s = snoopPorts.begin(); s != snoopPorts.end(); ++s) {
        SlavePort *p = *s;
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (exclude_slave_port_id == InvalidPortID ||
            p->getId() != exclude_slave_port_id)
            p->sendFunctionalSnoop(pkt);

        // if we get a response we are done
        if (pkt->isResponse()) {
            break;
        }
    }
}

CoherentBus *
CoherentBusParams::create()
{
    return new CoherentBus(this);
}
