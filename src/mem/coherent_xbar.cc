/*
 * Copyright (c) 2011-2015 ARM Limited
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
    : BaseXBar(p), system(p->system), snoopFilter(p->snoop_filter),
      snoopResponseLatency(p->snoop_response_latency)
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
        snoopLayers.push_back(new SnoopRespLayer(*bp, *this,
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
        snoopLayers.push_back(new SnoopRespLayer(*bp, *this,
                                                 csprintf(".snoopLayer%d",
                                                          defaultPortID)));
    }

    // create the slave ports, once again starting at zero
    for (int i = 0; i < p->port_slave_connection_count; ++i) {
        std::string portName = csprintf("%s.slave[%d]", name(), i);
        QueuedSlavePort* bp = new CoherentXBarSlavePort(portName, *this, i);
        slavePorts.push_back(bp);
        respLayers.push_back(new RespLayer(*bp, *this,
                                           csprintf(".respLayer%d", i)));
        snoopRespPorts.push_back(new SnoopRespPort(*bp, *this));
    }

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

    // inform the snoop filter about the slave ports so it can create
    // its own internal representation
    if (snoopFilter)
        snoopFilter->setSlavePorts(slavePorts);
}

bool
CoherentXBar::recvTimingReq(PacketPtr pkt, PortID slave_port_id)
{
    // @todo temporary hack to deal with memory corruption issue until
    // 4-phase transactions are complete
    for (int x = 0; x < pendingDelete.size(); x++)
        delete pendingDelete[x];
    pendingDelete.clear();

    // determine the source port based on the id
    SlavePort *src_port = slavePorts[slave_port_id];

    // remember if the packet is an express snoop
    bool is_express_snoop = pkt->isExpressSnoop();
    bool is_inhibited = pkt->memInhibitAsserted();
    // for normal requests, going downstream, the express snoop flag
    // and the inhibited flag should always be the same
    assert(is_express_snoop == is_inhibited);

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

    // store the old header delay so we can restore it if needed
    Tick old_header_delay = pkt->headerDelay;

    // a request sees the frontend and forward latency
    Tick xbar_delay = (frontendLatency + forwardLatency) * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    if (!system->bypassCaches()) {
        assert(pkt->snoopDelay == 0);

        // the packet is a memory-mapped request and should be
        // broadcasted to our snoopers but the source
        if (snoopFilter) {
            // check with the snoop filter where to forward this packet
            auto sf_res = snoopFilter->lookupRequest(pkt, *src_port);
            // the time required by a packet to be delivered through
            // the xbar has to be charged also with to lookup latency
            // of the snoop filter
            pkt->headerDelay += sf_res.second * clockPeriod();
            DPRINTF(CoherentXBar, "recvTimingReq: src %s %s 0x%x"\
                    " SF size: %i lat: %i\n", src_port->name(),
                    pkt->cmdString(), pkt->getAddr(), sf_res.first.size(),
                    sf_res.second);
            forwardTiming(pkt, slave_port_id, sf_res.first);
        } else {
            forwardTiming(pkt, slave_port_id);
        }

        // add the snoop delay to our header delay, and then reset it
        pkt->headerDelay += pkt->snoopDelay;
        pkt->snoopDelay = 0;
    }

    // forwardTiming snooped into peer caches of the sender, and if
    // this is a clean evict, but the packet is found in a cache, do
    // not forward it
    if (pkt->cmd == MemCmd::CleanEvict && pkt->isBlockCached()) {
        DPRINTF(CoherentXBar, "recvTimingReq: Clean evict 0x%x still cached, "
                "not forwarding\n", pkt->getAddr());

        // update the layer state and schedule an idle event
        reqLayers[master_port_id]->succeededTiming(packetFinishTime);
        pendingDelete.push_back(pkt);
        return true;
    }

    // remember if the packet will generate a snoop response
    const bool expect_snoop_resp = !is_inhibited && pkt->memInhibitAsserted();
    const bool expect_response = pkt->needsResponse() &&
        !pkt->memInhibitAsserted();

    // since it is a normal request, attempt to send the packet
    bool success = masterPorts[master_port_id]->sendTimingReq(pkt);

    if (snoopFilter && !system->bypassCaches()) {
        // Let the snoop filter know about the success of the send operation
        snoopFilter->finishRequest(!success, pkt);
    }

    // check if we were successful in sending the packet onwards
    if (!success)  {
        // express snoops and inhibited packets should never be forced
        // to retry
        assert(!is_express_snoop);
        assert(!pkt->memInhibitAsserted());

        // restore the header delay
        pkt->headerDelay = old_header_delay;

        DPRINTF(CoherentXBar, "recvTimingReq: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

        // update the layer state and schedule an idle event
        reqLayers[master_port_id]->failedTiming(src_port,
                                                clockEdge(Cycles(1)));
    } else {
        // express snoops currently bypass the crossbar state entirely
        if (!is_express_snoop) {
            // if this particular request will generate a snoop
            // response
            if (expect_snoop_resp) {
                // we should never have an exsiting request outstanding
                assert(outstandingSnoop.find(pkt->req) ==
                       outstandingSnoop.end());
                outstandingSnoop.insert(pkt->req);

                // basic sanity check on the outstanding snoops
                panic_if(outstandingSnoop.size() > 512,
                         "Outstanding snoop requests exceeded 512\n");
            }

            // remember where to route the normal response to
            if (expect_response || expect_snoop_resp) {
                assert(routeTo.find(pkt->req) == routeTo.end());
                routeTo[pkt->req] = slave_port_id;

                panic_if(routeTo.size() > 512,
                         "Routing table exceeds 512 packets\n");
            }

            // update the layer state and schedule an idle event
            reqLayers[master_port_id]->succeededTiming(packetFinishTime);
        }

        // stats updates only consider packets that were successfully sent
        pktCount[slave_port_id][master_port_id]++;
        pktSize[slave_port_id][master_port_id] += pkt_size;
        transDist[pkt_cmd]++;

        if (is_express_snoop)
            snoops++;
    }

    return success;
}

bool
CoherentXBar::recvTimingResp(PacketPtr pkt, PortID master_port_id)
{
    // determine the source port based on the id
    MasterPort *src_port = masterPorts[master_port_id];

    // determine the destination
    const auto route_lookup = routeTo.find(pkt->req);
    assert(route_lookup != routeTo.end());
    const PortID slave_port_id = route_lookup->second;
    assert(slave_port_id != InvalidPortID);
    assert(slave_port_id < respLayers.size());

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

    // a response sees the response latency
    Tick xbar_delay = responseLatency * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    if (snoopFilter && !system->bypassCaches()) {
        // let the snoop filter inspect the response and update its state
        snoopFilter->updateResponse(pkt, *slavePorts[slave_port_id]);
    }

    // send the packet through the destination slave port and pay for
    // any outstanding header delay
    Tick latency = pkt->headerDelay;
    pkt->headerDelay = 0;
    slavePorts[slave_port_id]->schedTimingResp(pkt, curTick() + latency);

    // remove the request from the routing table
    routeTo.erase(route_lookup);

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

    // set the packet header and payload delay, for now use forward latency
    // @todo Assess the choice of latency further
    calcPacketTiming(pkt, forwardLatency * clockPeriod());

    // remeber if the packet is inhibited so we can see if it changes
    const bool is_inhibited = pkt->memInhibitAsserted();

    assert(pkt->snoopDelay == 0);

    if (snoopFilter) {
        // let the Snoop Filter work its magic and guide probing
        auto sf_res = snoopFilter->lookupSnoop(pkt);
        // the time required by a packet to be delivered through
        // the xbar has to be charged also with to lookup latency
        // of the snoop filter
        pkt->headerDelay += sf_res.second * clockPeriod();
        DPRINTF(CoherentXBar, "recvTimingSnoopReq: src %s %s 0x%x"\
                " SF size: %i lat: %i\n", masterPorts[master_port_id]->name(),
                pkt->cmdString(), pkt->getAddr(), sf_res.first.size(),
                sf_res.second);

        // forward to all snoopers
        forwardTiming(pkt, InvalidPortID, sf_res.first);
    } else {
        forwardTiming(pkt, InvalidPortID);
    }

    // add the snoop delay to our header delay, and then reset it
    pkt->headerDelay += pkt->snoopDelay;
    pkt->snoopDelay = 0;

    // if we can expect a response, remember how to route it
    if (!is_inhibited && pkt->memInhibitAsserted()) {
        assert(routeTo.find(pkt->req) == routeTo.end());
        routeTo[pkt->req] = master_port_id;
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

    // get the destination
    const auto route_lookup = routeTo.find(pkt->req);
    assert(route_lookup != routeTo.end());
    const PortID dest_port_id = route_lookup->second;
    assert(dest_port_id != InvalidPortID);

    // determine if the response is from a snoop request we
    // created as the result of a normal request (in which case it
    // should be in the outstandingSnoop), or if we merely forwarded
    // someone else's snoop request
    const bool forwardAsSnoop = outstandingSnoop.find(pkt->req) ==
        outstandingSnoop.end();

    // test if the crossbar should be considered occupied for the
    // current port, note that the check is bypassed if the response
    // is being passed on as a normal response since this is occupying
    // the response layer rather than the snoop response layer
    if (forwardAsSnoop) {
        assert(dest_port_id < snoopLayers.size());
        if (!snoopLayers[dest_port_id]->tryTiming(src_port)) {
            DPRINTF(CoherentXBar, "recvTimingSnoopResp: src %s %s 0x%x BUSY\n",
                    src_port->name(), pkt->cmdString(), pkt->getAddr());
            return false;
        }
    } else {
        // get the master port that mirrors this slave port internally
        MasterPort* snoop_port = snoopRespPorts[slave_port_id];
        assert(dest_port_id < respLayers.size());
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

    // a snoop response sees the snoop response latency, and if it is
    // forwarded as a normal response, the response latency
    Tick xbar_delay =
        (forwardAsSnoop ? snoopResponseLatency : responseLatency) *
        clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

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
        outstandingSnoop.erase(pkt->req);

        // this is a snoop response from a coherent master, hence it
        // should never go back to where the snoop response came from,
        // but instead to where the original request came from
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
        // one of our slave ports, we also pay for any outstanding
        // header latency
        Tick latency = pkt->headerDelay;
        pkt->headerDelay = 0;
        slavePorts[dest_port_id]->schedTimingResp(pkt, curTick() + latency);

        respLayers[dest_port_id]->succeededTiming(packetFinishTime);
    }

    // remove the request from the routing table
    routeTo.erase(route_lookup);

    // stats updates
    transDist[pkt_cmd]++;
    snoops++;

    return true;
}


void
CoherentXBar::forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id,
                           const std::vector<QueuedSlavePort*>& dests)
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
CoherentXBar::recvReqRetry(PortID master_port_id)
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

    if (!system->bypassCaches()) {
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

            // let the snoop filter know about the success of the send
            // operation, and do it even before sending it onwards to
            // avoid situations where atomic upward snoops sneak in
            // between and change the filter state
            snoopFilter->finishRequest(false, pkt);

            snoop_result = forwardAtomic(pkt, slave_port_id, InvalidPortID,
                                         sf_res.first);
        } else {
            snoop_result = forwardAtomic(pkt, slave_port_id);
        }
        snoop_response_cmd = snoop_result.first;
        snoop_response_latency += snoop_result.second;
    }

    // forwardAtomic snooped into peer caches of the sender, and if
    // this is a clean evict, but the packet is found in a cache, do
    // not forward it
    if (pkt->cmd == MemCmd::CleanEvict && pkt->isBlockCached()) {
        DPRINTF(CoherentXBar, "recvAtomic: Clean evict 0x%x still cached, "
                "not forwarding\n", pkt->getAddr());
        return 0;
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

    // if lower levels have replied, tell the snoop filter
    if (!system->bypassCaches() && snoopFilter && pkt->isResponse()) {
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

    // @todo: Not setting header time
    pkt->payloadDelay = response_latency;
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

    // @todo: Not setting header time
    pkt->payloadDelay = snoop_response_latency;
    return snoop_response_latency;
}

std::pair<MemCmd, Tick>
CoherentXBar::forwardAtomic(PacketPtr pkt, PortID exclude_slave_port_id,
                           PortID source_master_port_id,
                           const std::vector<QueuedSlavePort*>& dests)
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

    if (!system->bypassCaches()) {
        // forward to all snoopers but the source
        forwardFunctional(pkt, slave_port_id);
    }

    // there is no need to continue if the snooping has found what we
    // were looking for and the packet is already a response
    if (!pkt->isResponse()) {
        // since our slave ports are queued ports we need to check them as well
        for (const auto& p : slavePorts) {
            // if we find a response that has the data, then the
            // downstream caches/memories may be out of date, so simply stop
            // here
            if (p->checkFunctional(pkt)) {
                if (pkt->needsResponse())
                    pkt->makeResponse();
                return;
            }
        }

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
