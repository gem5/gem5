/*
 * Copyright (c) 2011-2014 ARM Limited
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
 * Definition of a crossbar object.
 */

#include "base/misc.hh"
#include "base/trace.hh"
#include "debug/AddrRanges.hh"
#include "debug/CoherentXBar.hh"
#include "mem/coherent_xbar.hh"
#include "sim/system.hh"

CoherentXBar::CoherentXBar(const CoherentXBarParams *p)
    : BaseXBar(p), system(p->system), snoopFilter(p->snoop_filter)
{
    // create the ports based on the size of the master and slave
    // vector ports, and the presence of the default port, the ports
    // are enumerated starting from zero
    for (int i = 0; i < p->port_master_connection_count; ++i) {
        std::string portName = csprintf("%s.master[%d]", name(), i);
        MasterPort* bp = new CoherentXBarMasterPort(portName, *this, i);
        masterPorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this,
                                         csprintf(".reqLayer%d", i)));
        snoopLayers.push_back(new SnoopLayer(*bp, *this,
                                             csprintf(".snoopLayer%d", i)));
    }

    // see if we have a default slave device connected and if so add
    // our corresponding master port
    if (p->port_default_connection_count) {
        defaultPortID = masterPorts.size();
        std::string portName = name() + ".default";
        MasterPort* bp = new CoherentXBarMasterPort(portName, *this,
                                                   defaultPortID);
        masterPorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this, csprintf(".reqLayer%d",
                                             defaultPortID)));
        snoopLayers.push_back(new SnoopLayer(*bp, *this,
                                             csprintf(".snoopLayer%d",
                                                      defaultPortID)));
    }

    // create the slave ports, once again starting at zero
    for (int i = 0; i < p->port_slave_connection_count; ++i) {
        std::string portName = csprintf("%s.slave[%d]", name(), i);
        SlavePort* bp = new CoherentXBarSlavePort(portName, *this, i);
        slavePorts.push_back(bp);
        respLayers.push_back(new RespLayer(*bp, *this,
                                           csprintf(".respLayer%d", i)));
        snoopRespPorts.push_back(new SnoopRespPort(*bp, *this));
    }

    if (snoopFilter)
        snoopFilter->setSlavePorts(slavePorts);

    clearPortCache();
}

CoherentXBar::~CoherentXBar()
{
    for (auto l: reqLayers)
        delete l;
    for (auto l: respLayers)
        delete l;
    for (auto l: snoopLayers)
        delete l;
    for (auto p: snoopRespPorts)
        delete p;
}

void
CoherentXBar::init()
{
    // the base class is responsible for determining the block size
    BaseXBar::init();

    // iterate over our slave ports and determine which of our
    // neighbouring master ports are snooping and add them as snoopers
    for (const auto& p: slavePorts) {
        // check if the connected master port is snooping
        if (p->isSnooping()) {
            DPRINTF(AddrRanges, "Adding snooping master %s\n",
                    p->getMasterPort().name());
            snoopPorts.push_back(p);
        }
    }

    if (snoopPorts.empty())
        warn("CoherentXBar %s has no snooping ports attached!\n", name());
}

bool
CoherentXBar::recvTimingReq(PacketPtr pkt, PortID slave_port_id)
{
    // determine the source port based on the id
    SlavePort *src_port = slavePorts[slave_port_id];

    // remember if the packet is an express snoop
    bool is_express_snoop = pkt->isExpressSnoop();

    // determine the destination based on the address
    PortID master_port_id = findPort(pkt->getAddr());

    // test if the crossbar should be considered occupied for the current
    // port, and exclude express snoops from the check
    if (!is_express_snoop && !reqLayers[master_port_id]->tryTiming(src_port)) {
        DPRINTF(CoherentXBar, "recvTimingReq: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(CoherentXBar, "recvTimingReq: src %s %s expr %d 0x%x\n",
            src_port->name(), pkt->cmdString(), is_express_snoop,
            pkt->getAddr());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // set the source port for routing of the response
    pkt->setSrc(slave_port_id);

    calcPacketTiming(pkt);
    Tick packetFinishTime = pkt->lastWordDelay + curTick();

    // uncacheable requests need never be snooped
    if (!pkt->req->isUncacheable() && !system->bypassCaches()) {
        // the packet is a memory-mapped request and should be
        // broadcasted to our snoopers but the source
        if (snoopFilter) {
            // check with the snoop filter where to forward this packet
            auto sf_res = snoopFilter->lookupRequest(pkt, *src_port);
            packetFinishTime += sf_res.second * clockPeriod();
            DPRINTF(CoherentXBar, "recvTimingReq: src %s %s 0x%x"\
                    " SF size: %i lat: %i\n", src_port->name(),
                    pkt->cmdString(), pkt->getAddr(), sf_res.first.size(),
                    sf_res.second);
            forwardTiming(pkt, slave_port_id, sf_res.first);
        } else {
            forwardTiming(pkt, slave_port_id);
        }
    }

    // remember if we add an outstanding req so we can undo it if
    // necessary, if the packet needs a response, we should add it
    // as outstanding and express snoops never fail so there is
    // not need to worry about them
    bool add_outstanding = !is_express_snoop && pkt->needsResponse();

    // keep track that we have an outstanding request packet
    // matching this request, this is used by the coherency
    // mechanism in determining what to do with snoop responses
    // (in recvTimingSnoop)
    if (add_outstanding) {
        // we should never have an exsiting request outstanding
        assert(outstandingReq.find(pkt->req) == outstandingReq.end());
        outstandingReq.insert(pkt->req);
    }

    // Note: Cannot create a copy of the full packet, here.
    MemCmd orig_cmd(pkt->cmd);

    // since it is a normal request, attempt to send the packet
    bool success = masterPorts[master_port_id]->sendTimingReq(pkt);

    if (snoopFilter && !pkt->req->isUncacheable()
        && !system->bypassCaches()) {
        // The packet may already be overwritten by the sendTimingReq function.
        // The snoop filter needs to see the original request *and* the return
        // status of the send operation, so we need to recreate the original
        // request.  Atomic mode does not have the issue, as there the send
        // operation and the response happen instantaneously and don't need two
        // phase tracking.
        MemCmd tmp_cmd(pkt->cmd);
        pkt->cmd = orig_cmd;
        // Let the snoop filter know about the success of the send operation
        snoopFilter->updateRequest(pkt, *src_port, !success);
        pkt->cmd = tmp_cmd;
    }

    // if this is an express snoop, we are done at this point
    if (is_express_snoop) {
        assert(success);
        snoops++;
    } else {
        // for normal requests, check if successful
        if (!success)  {
            // inhibited packets should never be forced to retry
            assert(!pkt->memInhibitAsserted());

            // if it was added as outstanding and the send failed, then
            // erase it again
            if (add_outstanding)
                outstandingReq.erase(pkt->req);

            // undo the calculation so we can check for 0 again
            pkt->firstWordDelay = pkt->lastWordDelay = 0;

            DPRINTF(CoherentXBar, "recvTimingReq: src %s %s 0x%x RETRY\n",
                    src_port->name(), pkt->cmdString(), pkt->getAddr());

            // update the layer state and schedule an idle event
            reqLayers[master_port_id]->failedTiming(src_port,
                                                    clockEdge(headerCycles));
        } else {
            // update the layer state and schedule an idle event
            reqLayers[master_port_id]->succeededTiming(packetFinishTime);
        }
    }

    // stats updates only consider packets that were successfully sent
    if (success) {
        pktCount[slave_port_id][master_port_id]++;
        pktSize[slave_port_id][master_port_id] += pkt_size;
        transDist[pkt_cmd]++;
    }

    return success;
}

bool
CoherentXBar::recvTimingResp(PacketPtr pkt, PortID master_port_id)
{
    // determine the source port based on the id
    MasterPort *src_port = masterPorts[master_port_id];

    // determine the destination based on what is stored in the packet
    PortID slave_port_id = pkt->getDest();

    // test if the crossbar should be considered occupied for the
    // current port
    if (!respLayers[slave_port_id]->tryTiming(src_port)) {
        DPRINTF(CoherentXBar, "recvTimingResp: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(CoherentXBar, "recvTimingResp: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    calcPacketTiming(pkt);
    Tick packetFinishTime = pkt->lastWordDelay + curTick();

    // the packet is a normal response to a request that we should
    // have seen passing through the crossbar
    assert(outstandingReq.find(pkt->req) != outstandingReq.end());

    if (snoopFilter && !pkt->req->isUncacheable() && !system->bypassCaches()) {
        // let the snoop filter inspect the response and update its state
        snoopFilter->updateResponse(pkt, *slavePorts[slave_port_id]);
    }

    // remove it as outstanding
    outstandingReq.erase(pkt->req);

    // send the packet through the destination slave port
    bool success M5_VAR_USED = slavePorts[slave_port_id]->sendTimingResp(pkt);

    // currently it is illegal to block responses... can lead to
    // deadlock
    assert(success);

    respLayers[slave_port_id]->succeededTiming(packetFinishTime);

    // stats updates
    pktCount[slave_port_id][master_port_id]++;
    pktSize[slave_port_id][master_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}

void
CoherentXBar::recvTimingSnoopReq(PacketPtr pkt, PortID master_port_id)
{
    DPRINTF(CoherentXBar, "recvTimingSnoopReq: src %s %s 0x%x\n",
            masterPorts[master_port_id]->name(), pkt->cmdString(),
            pkt->getAddr());

    // update stats here as we know the forwarding will succeed
    transDist[pkt->cmdToIndex()]++;
    snoops++;

    // we should only see express snoops from caches
    assert(pkt->isExpressSnoop());

    // set the source port for routing of the response
    pkt->setSrc(master_port_id);

    if (snoopFilter) {
        // let the Snoop Filter work its magic and guide probing
        auto sf_res = snoopFilter->lookupSnoop(pkt);
        // No timing here: packetFinishTime += sf_res.second * clockPeriod();
        DPRINTF(CoherentXBar, "recvTimingSnoopReq: src %s %s 0x%x"\
                " SF size: %i lat: %i\n", masterPorts[master_port_id]->name(),
                pkt->cmdString(), pkt->getAddr(), sf_res.first.size(),
                sf_res.second);

        // forward to all snoopers
        forwardTiming(pkt, InvalidPortID, sf_res.first);
    } else {
        forwardTiming(pkt, InvalidPortID);
    }

    // a snoop request came from a connected slave device (one of
    // our master ports), and if it is not coming from the slave
    // device responsible for the address range something is
    // wrong, hence there is nothing further to do as the packet
    // would be going back to where it came from
    assert(master_port_id == findPort(pkt->getAddr()));
}

bool
CoherentXBar::recvTimingSnoopResp(PacketPtr pkt, PortID slave_port_id)
{
    // determine the source port based on the id
    SlavePort* src_port = slavePorts[slave_port_id];

    // get the destination from the packet
    PortID dest_port_id = pkt->getDest();

    // determine if the response is from a snoop request we
    // created as the result of a normal request (in which case it
    // should be in the outstandingReq), or if we merely forwarded
    // someone else's snoop request
    bool forwardAsSnoop = outstandingReq.find(pkt->req) ==
        outstandingReq.end();

    // test if the crossbar should be considered occupied for the
    // current port, note that the check is bypassed if the response
    // is being passed on as a normal response since this is occupying
    // the response layer rather than the snoop response layer
    if (forwardAsSnoop) {
        if (!snoopLayers[dest_port_id]->tryTiming(src_port)) {
            DPRINTF(CoherentXBar, "recvTimingSnoopResp: src %s %s 0x%x BUSY\n",
                    src_port->name(), pkt->cmdString(), pkt->getAddr());
            return false;
        }
    } else {
        // get the master port that mirrors this slave port internally
        MasterPort* snoop_port = snoopRespPorts[slave_port_id];
        if (!respLayers[dest_port_id]->tryTiming(snoop_port)) {
            DPRINTF(CoherentXBar, "recvTimingSnoopResp: src %s %s 0x%x BUSY\n",
                    snoop_port->name(), pkt->cmdString(), pkt->getAddr());
            return false;
        }
    }

    DPRINTF(CoherentXBar, "recvTimingSnoopResp: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // responses are never express snoops
    assert(!pkt->isExpressSnoop());

    calcPacketTiming(pkt);
    Tick packetFinishTime = pkt->lastWordDelay + curTick();

    // forward it either as a snoop response or a normal response
    if (forwardAsSnoop) {
        // this is a snoop response to a snoop request we forwarded,
        // e.g. coming from the L1 and going to the L2, and it should
        // be forwarded as a snoop response

        if (snoopFilter) {
            // update the probe filter so that it can properly track the line
            snoopFilter->updateSnoopForward(pkt, *slavePorts[slave_port_id],
                                            *masterPorts[dest_port_id]);
        }

        bool success M5_VAR_USED =
            masterPorts[dest_port_id]->sendTimingSnoopResp(pkt);
        pktCount[slave_port_id][dest_port_id]++;
        pktSize[slave_port_id][dest_port_id] += pkt_size;
        assert(success);

        snoopLayers[dest_port_id]->succeededTiming(packetFinishTime);
    } else {
        // we got a snoop response on one of our slave ports,
        // i.e. from a coherent master connected to the crossbar, and
        // since we created the snoop request as part of recvTiming,
        // this should now be a normal response again
        outstandingReq.erase(pkt->req);

        // this is a snoop response from a coherent master, with a
        // destination field set on its way through the crossbar as
        // request, hence it should never go back to where the snoop
        // response came from, but instead to where the original
        // request came from
        assert(slave_port_id != dest_port_id);

        if (snoopFilter) {
            // update the probe filter so that it can properly track the line
            snoopFilter->updateSnoopResponse(pkt, *slavePorts[slave_port_id],
                                    *slavePorts[dest_port_id]);
        }

        DPRINTF(CoherentXBar, "recvTimingSnoopResp: src %s %s 0x%x"\
                " FWD RESP\n", src_port->name(), pkt->cmdString(),
                pkt->getAddr());

        // as a normal response, it should go back to a master through
        // one of our slave ports, at this point we are ignoring the
        // fact that the response layer could be busy and do not touch
        // its state
        bool success M5_VAR_USED =
            slavePorts[dest_port_id]->sendTimingResp(pkt);

        // @todo Put the response in an internal FIFO and pass it on
        // to the response layer from there

        // currently it is illegal to block responses... can lead
        // to deadlock
        assert(success);

        respLayers[dest_port_id]->succeededTiming(packetFinishTime);
    }

    // stats updates
    transDist[pkt_cmd]++;
    snoops++;

    return true;
}


void
CoherentXBar::forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id,
                           const std::vector<SlavePort*>& dests)
{
    DPRINTF(CoherentXBar, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    // snoops should only happen if the system isn't bypassing caches
    assert(!system->bypassCaches());

    unsigned fanout = 0;

    for (const auto& p: dests) {
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (exclude_slave_port_id == InvalidPortID ||
            p->getId() != exclude_slave_port_id) {
            // cache is not allowed to refuse snoop
            p->sendTimingSnoopReq(pkt);
            fanout++;
        }
    }

    // Stats for fanout of this forward operation
    snoopFanout.sample(fanout);
}

void
CoherentXBar::recvRetry(PortID master_port_id)
{
    // responses and snoop responses never block on forwarding them,
    // so the retry will always be coming from a port to which we
    // tried to forward a request
    reqLayers[master_port_id]->recvRetry();
}

Tick
CoherentXBar::recvAtomic(PacketPtr pkt, PortID slave_port_id)
{
    DPRINTF(CoherentXBar, "recvAtomic: packet src %s addr 0x%x cmd %s\n",
            slavePorts[slave_port_id]->name(), pkt->getAddr(),
            pkt->cmdString());

    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    MemCmd snoop_response_cmd = MemCmd::InvalidCmd;
    Tick snoop_response_latency = 0;

    // uncacheable requests need never be snooped
    if (!pkt->req->isUncacheable() && !system->bypassCaches()) {
        // forward to all snoopers but the source
        std::pair<MemCmd, Tick> snoop_result;
        if (snoopFilter) {
            // check with the snoop filter where to forward this packet
            auto sf_res =
                snoopFilter->lookupRequest(pkt, *slavePorts[slave_port_id]);
            snoop_response_latency += sf_res.second * clockPeriod();
            DPRINTF(CoherentXBar, "%s: src %s %s 0x%x"\
                    " SF size: %i lat: %i\n", __func__,
                    slavePorts[slave_port_id]->name(), pkt->cmdString(),
                    pkt->getAddr(), sf_res.first.size(), sf_res.second);
            snoop_result = forwardAtomic(pkt, slave_port_id, InvalidPortID,
                                         sf_res.first);
        } else {
            snoop_result = forwardAtomic(pkt, slave_port_id);
        }
        snoop_response_cmd = snoop_result.first;
        snoop_response_latency += snoop_result.second;
    }

    // even if we had a snoop response, we must continue and also
    // perform the actual request at the destination
    PortID master_port_id = findPort(pkt->getAddr());

    // stats updates for the request
    pktCount[slave_port_id][master_port_id]++;
    pktSize[slave_port_id][master_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    // forward the request to the appropriate destination
    Tick response_latency = masterPorts[master_port_id]->sendAtomic(pkt);

    // Lower levels have replied, tell the snoop filter
    if (snoopFilter && !pkt->req->isUncacheable() && !system->bypassCaches() &&
        pkt->isResponse()) {
        snoopFilter->updateResponse(pkt, *slavePorts[slave_port_id]);
    }

    // if we got a response from a snooper, restore it here
    if (snoop_response_cmd != MemCmd::InvalidCmd) {
        // no one else should have responded
        assert(!pkt->isResponse());
        pkt->cmd = snoop_response_cmd;
        response_latency = snoop_response_latency;
    }

    // add the response data
    if (pkt->isResponse()) {
        pkt_size = pkt->hasData() ? pkt->getSize() : 0;
        pkt_cmd = pkt->cmdToIndex();

        // stats updates
        pktCount[slave_port_id][master_port_id]++;
        pktSize[slave_port_id][master_port_id] += pkt_size;
        transDist[pkt_cmd]++;
    }

    // @todo: Not setting first-word time
    pkt->lastWordDelay = response_latency;
    return response_latency;
}

Tick
CoherentXBar::recvAtomicSnoop(PacketPtr pkt, PortID master_port_id)
{
    DPRINTF(CoherentXBar, "recvAtomicSnoop: packet src %s addr 0x%x cmd %s\n",
            masterPorts[master_port_id]->name(), pkt->getAddr(),
            pkt->cmdString());

    // add the request snoop data
    snoops++;

    // forward to all snoopers
    std::pair<MemCmd, Tick> snoop_result;
    Tick snoop_response_latency = 0;
    if (snoopFilter) {
        auto sf_res = snoopFilter->lookupSnoop(pkt);
        snoop_response_latency += sf_res.second * clockPeriod();
        DPRINTF(CoherentXBar, "%s: src %s %s 0x%x SF size: %i lat: %i\n",
                __func__, masterPorts[master_port_id]->name(), pkt->cmdString(),
                pkt->getAddr(), sf_res.first.size(), sf_res.second);
        snoop_result = forwardAtomic(pkt, InvalidPortID, master_port_id,
                                     sf_res.first);
    } else {
        snoop_result = forwardAtomic(pkt, InvalidPortID);
    }
    MemCmd snoop_response_cmd = snoop_result.first;
    snoop_response_latency += snoop_result.second;

    if (snoop_response_cmd != MemCmd::InvalidCmd)
        pkt->cmd = snoop_response_cmd;

    // add the response snoop data
    if (pkt->isResponse()) {
        snoops++;
    }

    // @todo: Not setting first-word time
    pkt->lastWordDelay = snoop_response_latency;
    return snoop_response_latency;
}

std::pair<MemCmd, Tick>
CoherentXBar::forwardAtomic(PacketPtr pkt, PortID exclude_slave_port_id,
                           PortID source_master_port_id,
                           const std::vector<SlavePort*>& dests)
{
    // the packet may be changed on snoops, record the original
    // command to enable us to restore it between snoops so that
    // additional snoops can take place properly
    MemCmd orig_cmd = pkt->cmd;
    MemCmd snoop_response_cmd = MemCmd::InvalidCmd;
    Tick snoop_response_latency = 0;

    // snoops should only happen if the system isn't bypassing caches
    assert(!system->bypassCaches());

    unsigned fanout = 0;

    for (const auto& p: dests) {
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (exclude_slave_port_id != InvalidPortID &&
            p->getId() == exclude_slave_port_id)
            continue;

        Tick latency = p->sendAtomicSnoop(pkt);
        fanout++;

        // in contrast to a functional access, we have to keep on
        // going as all snoopers must be updated even if we get a
        // response
        if (!pkt->isResponse())
            continue;

        // response from snoop agent
        assert(pkt->cmd != orig_cmd);
        assert(pkt->memInhibitAsserted());
        // should only happen once
        assert(snoop_response_cmd == MemCmd::InvalidCmd);
        // save response state
        snoop_response_cmd = pkt->cmd;
        snoop_response_latency = latency;

        if (snoopFilter) {
            // Handle responses by the snoopers and differentiate between
            // responses to requests from above and snoops from below
            if (source_master_port_id != InvalidPortID) {
                // Getting a response for a snoop from below
                assert(exclude_slave_port_id == InvalidPortID);
                snoopFilter->updateSnoopForward(pkt, *p,
                             *masterPorts[source_master_port_id]);
            } else {
                // Getting a response for a request from above
                assert(source_master_port_id == InvalidPortID);
                snoopFilter->updateSnoopResponse(pkt, *p,
                             *slavePorts[exclude_slave_port_id]);
            }
        }
        // restore original packet state for remaining snoopers
        pkt->cmd = orig_cmd;
    }

    // Stats for fanout
    snoopFanout.sample(fanout);

    // the packet is restored as part of the loop and any potential
    // snoop response is part of the returned pair
    return std::make_pair(snoop_response_cmd, snoop_response_latency);
}

void
CoherentXBar::recvFunctional(PacketPtr pkt, PortID slave_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(CoherentXBar,
                "recvFunctional: packet src %s addr 0x%x cmd %s\n",
                slavePorts[slave_port_id]->name(), pkt->getAddr(),
                pkt->cmdString());
    }

    // uncacheable requests need never be snooped
    if (!pkt->req->isUncacheable() && !system->bypassCaches()) {
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
CoherentXBar::recvFunctionalSnoop(PacketPtr pkt, PortID master_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(CoherentXBar,
                "recvFunctionalSnoop: packet src %s addr 0x%x cmd %s\n",
                masterPorts[master_port_id]->name(), pkt->getAddr(),
                pkt->cmdString());
    }

    // forward to all snoopers
    forwardFunctional(pkt, InvalidPortID);
}

void
CoherentXBar::forwardFunctional(PacketPtr pkt, PortID exclude_slave_port_id)
{
    // snoops should only happen if the system isn't bypassing caches
    assert(!system->bypassCaches());

    for (const auto& p: snoopPorts) {
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

unsigned int
CoherentXBar::drain(DrainManager *dm)
{
    // sum up the individual layers
    unsigned int total = 0;
    for (auto l: reqLayers)
        total += l->drain(dm);
    for (auto l: respLayers)
        total += l->drain(dm);
    for (auto l: snoopLayers)
        total += l->drain(dm);
    return total;
}

void
CoherentXBar::regStats()
{
    // register the stats of the base class and our layers
    BaseXBar::regStats();
    for (auto l: reqLayers)
        l->regStats();
    for (auto l: respLayers)
        l->regStats();
    for (auto l: snoopLayers)
        l->regStats();

    snoops
        .name(name() + ".snoops")
        .desc("Total snoops (count)")
    ;

    snoopFanout
        .init(0, snoopPorts.size(), 1)
        .name(name() + ".snoop_fanout")
        .desc("Request fanout histogram")
    ;
}

CoherentXBar *
CoherentXBarParams::create()
{
    return new CoherentXBar(this);
}
