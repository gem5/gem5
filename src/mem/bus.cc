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
#include "debug/Bus.hh"
#include "debug/BusAddrRanges.hh"
#include "mem/bus.hh"

Bus::Bus(const BusParams *p)
    : MemObject(p), clock(p->clock),
      headerCycles(p->header_cycles), width(p->width), tickNextIdle(0),
      drainEvent(NULL), busIdleEvent(this), inRetry(false),
      defaultPortID(InvalidPortID),
      useDefaultRange(p->use_default_range),
      defaultBlockSize(p->block_size),
      cachedBlockSize(0), cachedBlockSizeValid(false)
{
    //width, clock period, and header cycles must be positive
    if (width <= 0)
        fatal("Bus width must be positive\n");
    if (clock <= 0)
        fatal("Bus clock period must be positive\n");
    if (headerCycles <= 0)
        fatal("Number of header cycles must be positive\n");

    // create the ports based on the size of the master and slave
    // vector ports, and the presence of the default port, the ports
    // are enumerated starting from zero
    for (int i = 0; i < p->port_master_connection_count; ++i) {
        std::string portName = csprintf("%s-p%d", name(), i);
        MasterPort* bp = new BusMasterPort(portName, this, i);
        masterPorts.push_back(bp);
    }

    // see if we have a default slave device connected and if so add
    // our corresponding master port
    if (p->port_default_connection_count) {
        defaultPortID = masterPorts.size();
        std::string portName = csprintf("%s-default", name());
        MasterPort* bp = new BusMasterPort(portName, this, defaultPortID);
        masterPorts.push_back(bp);
    }

    // create the slave ports, once again starting at zero
    for (int i = 0; i < p->port_slave_connection_count; ++i) {
        std::string portName = csprintf("%s-p%d", name(), i);
        SlavePort* bp = new BusSlavePort(portName, this, i);
        slavePorts.push_back(bp);
    }

    clearPortCache();
}

MasterPort &
Bus::getMasterPort(const std::string &if_name, int idx)
{
    if (if_name == "master" && idx < masterPorts.size()) {
        // the master port index translates directly to the vector position
        return *masterPorts[idx];
    } else  if (if_name == "default") {
        return *masterPorts[defaultPortID];
    } else {
        return MemObject::getMasterPort(if_name, idx);
    }
}

SlavePort &
Bus::getSlavePort(const std::string &if_name, int idx)
{
    if (if_name == "slave" && idx < slavePorts.size()) {
        // the slave port index translates directly to the vector position
        return *slavePorts[idx];
    } else {
        return MemObject::getSlavePort(if_name, idx);
    }
}

void
Bus::init()
{
    // iterate over our slave ports and determine which of our
    // neighbouring master ports are snooping and add them as snoopers
    for (SlavePortConstIter p = slavePorts.begin(); p != slavePorts.end();
         ++p) {
        if ((*p)->getMasterPort().isSnooping()) {
            DPRINTF(BusAddrRanges, "Adding snooping neighbour %s\n",
                    (*p)->getMasterPort().name());
            snoopPorts.push_back(*p);
        }
    }
}

Tick
Bus::calcPacketTiming(PacketPtr pkt)
{
    // determine the current time rounded to the closest following
    // clock edge
    Tick now = curTick();
    if (now % clock != 0) {
        now = ((now / clock) + 1) * clock;
    }

    Tick headerTime = now + headerCycles * clock;

    // The packet will be sent. Figure out how long it occupies the bus, and
    // how much of that time is for the first "word", aka bus width.
    int numCycles = 0;
    if (pkt->hasData()) {
        // If a packet has data, it needs ceil(size/width) cycles to send it
        int dataSize = pkt->getSize();
        numCycles += dataSize/width;
        if (dataSize % width)
            numCycles++;
    }

    // The first word will be delivered after the current tick, the delivery
    // of the address if any, and one bus cycle to deliver the data
    pkt->firstWordTime = headerTime + clock;

    pkt->finishTime = headerTime + numCycles * clock;

    return headerTime;
}

void Bus::occupyBus(Tick until)
{
    if (until == 0) {
        // shortcut for express snoop packets
        return;
    }

    tickNextIdle = until;
    reschedule(busIdleEvent, tickNextIdle, true);

    DPRINTF(Bus, "The bus is now occupied from tick %d to %d\n",
            curTick(), tickNextIdle);
}

bool
Bus::isOccupied(Port* port)
{
    // first we see if the next idle tick is in the future, next the
    // bus is considered occupied if there are ports on the retry list
    // and we are not in a retry with the current port
    if (tickNextIdle > curTick() ||
        (!retryList.empty() && !(inRetry && port == retryList.front()))) {
        addToRetryList(port);
        return true;
    }
    return false;
}

bool
Bus::recvTimingReq(PacketPtr pkt, PortID slave_port_id)
{
    // determine the source port based on the id
    SlavePort *src_port = slavePorts[slave_port_id];

    // test if the bus should be considered occupied for the current
    // port, and exclude express snoops from the check
    if (!pkt->isExpressSnoop() && isOccupied(src_port)) {
        DPRINTF(Bus, "recvTimingReq: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(Bus, "recvTimingReq: src %s %s 0x%x\n",
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

        DPRINTF(Bus, "recvTimingReq: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

        addToRetryList(src_port);
        occupyBus(headerFinishTime);

        return false;
    }

    succeededTiming(packetFinishTime);

    return true;
}

bool
Bus::recvTimingResp(PacketPtr pkt, PortID master_port_id)
{
    // determine the source port based on the id
    MasterPort *src_port = masterPorts[master_port_id];

    // test if the bus should be considered occupied for the current
    // port
    if (isOccupied(src_port)) {
        DPRINTF(Bus, "recvTimingResp: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(Bus, "recvTimingResp: src %s %s 0x%x\n",
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
Bus::recvTimingSnoopReq(PacketPtr pkt, PortID master_port_id)
{
    DPRINTF(Bus, "recvTimingSnoopReq: src %s %s 0x%x\n",
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
Bus::recvTimingSnoopResp(PacketPtr pkt, PortID slave_port_id)
{
    // determine the source port based on the id
    SlavePort* src_port = slavePorts[slave_port_id];

    // test if the bus should be considered occupied for the current
    // port
    if (isOccupied(src_port)) {
        DPRINTF(Bus, "recvTimingSnoopResp: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(Bus, "recvTimingSnoop: src %s %s 0x%x\n",
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
Bus::succeededTiming(Tick busy_time)
{
    // occupy the bus accordingly
    occupyBus(busy_time);

    // if a retrying port succeeded, also take it off the retry list
    if (inRetry) {
        DPRINTF(Bus, "Remove retry from list %s\n",
                retryList.front()->name());
        retryList.pop_front();
        inRetry = false;
    }
}

void
Bus::forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id)
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

void
Bus::releaseBus()
{
    // releasing the bus means we should now be idle
    assert(curTick() >= tickNextIdle);

    // bus is now idle, so if someone is waiting we can retry
    if (!retryList.empty()) {
        // note that we block (return false on recvTiming) both
        // because the bus is busy and because the destination is
        // busy, and in the latter case the bus may be released before
        // we see a retry from the destination
        retryWaiting();
    }

    //If we weren't able to drain before, we might be able to now.
    if (drainEvent && retryList.empty() && curTick() >= tickNextIdle) {
        drainEvent->process();
        // Clear the drain event once we're done with it.
        drainEvent = NULL;
    }
}

void
Bus::retryWaiting()
{
    // this should never be called with an empty retry list
    assert(!retryList.empty());

    // send a retry to the port at the head of the retry list
    inRetry = true;

    // note that we might have blocked on the receiving port being
    // busy (rather than the bus itself) and now call retry before the
    // destination called retry on the bus
    retryList.front()->sendRetry();

    // If inRetry is still true, sendTiming wasn't called in zero time
    // (e.g. the cache does this)
    if (inRetry) {
        retryList.pop_front();
        inRetry = false;

        //Bring tickNextIdle up to the present
        while (tickNextIdle < curTick())
            tickNextIdle += clock;

        //Burn a cycle for the missed grant.
        tickNextIdle += clock;

        reschedule(busIdleEvent, tickNextIdle, true);
    }
}

void
Bus::recvRetry()
{
    // we got a retry from a peer that we tried to send something to
    // and failed, but we sent it on the account of someone else, and
    // that source port should be on our retry list, however if the
    // bus is released before this happens and the retry (from the bus
    // point of view) is successful then this no longer holds and we
    // could in fact have an empty retry list
    if (retryList.empty())
        return;

    // if the bus isn't busy
    if (curTick() >= tickNextIdle) {
        // note that we do not care who told us to retry at the moment, we
        // merely let the first one on the retry list go
        retryWaiting();
    }
}

PortID
Bus::findPort(Addr addr)
{
    /* An interval tree would be a better way to do this. --ali. */
    PortID dest_id = checkPortCache(addr);
    if (dest_id != InvalidPortID)
        return dest_id;

    // Check normal port ranges
    PortIter i = portMap.find(RangeSize(addr,1));
    if (i != portMap.end()) {
        dest_id = i->second;
        updatePortCache(dest_id, i->first.start, i->first.end);
        return dest_id;
    }

    // Check if this matches the default range
    if (useDefaultRange) {
        AddrRangeIter a_end = defaultRange.end();
        for (AddrRangeIter i = defaultRange.begin(); i != a_end; i++) {
            if (*i == addr) {
                DPRINTF(Bus, "  found addr %#llx on default\n", addr);
                return defaultPortID;
            }
        }
    } else if (defaultPortID != InvalidPortID) {
        DPRINTF(Bus, "Unable to find destination for addr %#llx, "
                "will use default port\n", addr);
        return defaultPortID;
    }

    // we should use the range for the default port and it did not
    // match, or the default port is not set
    fatal("Unable to find destination for addr %#llx on bus %s\n", addr,
          name());
}

Tick
Bus::recvAtomic(PacketPtr pkt, PortID slave_port_id)
{
    DPRINTF(Bus, "recvAtomic: packet src %s addr 0x%x cmd %s\n",
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
Bus::recvAtomicSnoop(PacketPtr pkt, PortID master_port_id)
{
    DPRINTF(Bus, "recvAtomicSnoop: packet src %s addr 0x%x cmd %s\n",
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
Bus::forwardAtomic(PacketPtr pkt, PortID exclude_slave_port_id)
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
Bus::recvFunctional(PacketPtr pkt, PortID slave_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(Bus,
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
Bus::recvFunctionalSnoop(PacketPtr pkt, PortID master_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(Bus,
                "recvFunctionalSnoop: packet src %s addr 0x%x cmd %s\n",
                masterPorts[master_port_id]->name(), pkt->getAddr(),
                pkt->cmdString());
    }

    // forward to all snoopers
    forwardFunctional(pkt, InvalidPortID);
}

void
Bus::forwardFunctional(PacketPtr pkt, PortID exclude_slave_port_id)
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

/** Function called by the port when the bus is receiving a range change.*/
void
Bus::recvRangeChange(PortID master_port_id)
{
    AddrRangeList ranges;
    AddrRangeIter iter;

    if (inRecvRangeChange.count(master_port_id))
        return;
    inRecvRangeChange.insert(master_port_id);

    DPRINTF(BusAddrRanges, "received RangeChange from device id %d\n",
            master_port_id);

    clearPortCache();
    if (master_port_id == defaultPortID) {
        defaultRange.clear();
        // Only try to update these ranges if the user set a default responder.
        if (useDefaultRange) {
            AddrRangeList ranges =
                masterPorts[master_port_id]->getSlavePort().getAddrRanges();
            for(iter = ranges.begin(); iter != ranges.end(); iter++) {
                defaultRange.push_back(*iter);
                DPRINTF(BusAddrRanges, "Adding range %#llx - %#llx for default range\n",
                        iter->start, iter->end);
            }
        }
    } else {

        assert(master_port_id < masterPorts.size() && master_port_id >= 0);
        MasterPort *port = masterPorts[master_port_id];

        // Clean out any previously existent ids
        for (PortIter portIter = portMap.begin();
             portIter != portMap.end(); ) {
            if (portIter->second == master_port_id)
                portMap.erase(portIter++);
            else
                portIter++;
        }

        ranges = port->getSlavePort().getAddrRanges();

        for (iter = ranges.begin(); iter != ranges.end(); iter++) {
            DPRINTF(BusAddrRanges, "Adding range %#llx - %#llx for id %d\n",
                    iter->start, iter->end, master_port_id);
            if (portMap.insert(*iter, master_port_id) == portMap.end()) {
                PortID conflict_id = portMap.find(*iter)->second;
                fatal("%s has two ports with same range:\n\t%s\n\t%s\n",
                      name(),
                      masterPorts[master_port_id]->getSlavePort().name(),
                      masterPorts[conflict_id]->getSlavePort().name());
            }
        }
    }
    DPRINTF(BusAddrRanges, "port list has %d entries\n", portMap.size());

    // tell all our neighbouring master ports that our address range
    // has changed
    for (SlavePortConstIter p = slavePorts.begin(); p != slavePorts.end();
         ++p)
        (*p)->sendRangeChange();

    inRecvRangeChange.erase(master_port_id);
}

AddrRangeList
Bus::getAddrRanges()
{
    AddrRangeList ranges;

    DPRINTF(BusAddrRanges, "received address range request, returning:\n");

    for (AddrRangeIter dflt_iter = defaultRange.begin();
         dflt_iter != defaultRange.end(); dflt_iter++) {
        ranges.push_back(*dflt_iter);
        DPRINTF(BusAddrRanges, "  -- Dflt: %#llx : %#llx\n",dflt_iter->start,
                dflt_iter->end);
    }
    for (PortIter portIter = portMap.begin();
         portIter != portMap.end(); portIter++) {
        bool subset = false;
        for (AddrRangeIter dflt_iter = defaultRange.begin();
             dflt_iter != defaultRange.end(); dflt_iter++) {
            if ((portIter->first.start < dflt_iter->start &&
                portIter->first.end >= dflt_iter->start) ||
               (portIter->first.start < dflt_iter->end &&
                portIter->first.end >= dflt_iter->end))
                fatal("Devices can not set ranges that itersect the default set\
                        but are not a subset of the default set.\n");
            if (portIter->first.start >= dflt_iter->start &&
                portIter->first.end <= dflt_iter->end) {
                subset = true;
                DPRINTF(BusAddrRanges, "  -- %#llx : %#llx is a SUBSET\n",
                    portIter->first.start, portIter->first.end);
            }
        }
        if (!subset) {
            ranges.push_back(portIter->first);
            DPRINTF(BusAddrRanges, "  -- %#llx : %#llx\n",
                    portIter->first.start, portIter->first.end);
        }
    }

    return ranges;
}

bool
Bus::isSnooping() const
{
    // in essence, answer the question if there are snooping ports
    return !snoopPorts.empty();
}

unsigned
Bus::findBlockSize()
{
    if (cachedBlockSizeValid)
        return cachedBlockSize;

    unsigned max_bs = 0;

    PortIter p_end = portMap.end();
    for (PortIter p_iter = portMap.begin(); p_iter != p_end; p_iter++) {
        unsigned tmp_bs = masterPorts[p_iter->second]->peerBlockSize();
        if (tmp_bs > max_bs)
            max_bs = tmp_bs;
    }

    for (SlavePortConstIter s = snoopPorts.begin(); s != snoopPorts.end();
         ++s) {
        unsigned tmp_bs = (*s)->peerBlockSize();
        if (tmp_bs > max_bs)
            max_bs = tmp_bs;
    }
    if (max_bs == 0)
        max_bs = defaultBlockSize;

    if (max_bs != 64)
        warn_once("Blocksize found to not be 64... hmm... probably not.\n");
    cachedBlockSize = max_bs;
    cachedBlockSizeValid = true;
    return max_bs;
}


unsigned int
Bus::drain(Event * de)
{
    //We should check that we're not "doing" anything, and that noone is
    //waiting. We might be idle but have someone waiting if the device we
    //contacted for a retry didn't actually retry.
    if (!retryList.empty() || (curTick() < tickNextIdle &&
                               busIdleEvent.scheduled())) {
        drainEvent = de;
        return 1;
    }
    return 0;
}

void
Bus::startup()
{
    if (tickNextIdle < curTick())
        tickNextIdle = (curTick() / clock) * clock + clock;
}

Bus *
BusParams::create()
{
    return new Bus(this);
}
