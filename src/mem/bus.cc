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
      nbrMasterPorts(p->port_master_connection_count),
      defaultPortId(INVALID_PORT_ID), useDefaultRange(p->use_default_range),
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
    // vector ports, and the presence of the default master

    // id used to index into master and slave ports, that currently
    // has holes to be able to use the id to index into either
    int id = 0;
    for (int i = 0; i < p->port_master_connection_count; ++i) {
        std::string portName = csprintf("%s-p%d", name(), id);
        BusMasterPort* bp = new BusMasterPort(portName, this, id);
        masterPorts.push_back(bp);
        slavePorts.push_back(NULL);
        ++id;
    }

    // see if we have a default master connected and if so add the
    // port
    if (p->port_default_connection_count) {
        defaultPortId = id;
        std::string portName = csprintf("%s-default", name());
        BusMasterPort* bp = new BusMasterPort(portName, this, id);
        masterPorts.push_back(bp);
        slavePorts.push_back(NULL);
        ++id;
        // this is an additional master port
        ++nbrMasterPorts;
    }

    // note that the first slave port is now stored on index
    // nbrMasterPorts in the vector
    for (int i = 0; i < p->port_slave_connection_count; ++i) {
        std::string portName = csprintf("%s-p%d", name(), id);
        BusSlavePort* bp = new BusSlavePort(portName, this, id);
        masterPorts.push_back(NULL);
        slavePorts.push_back(bp);
        ++id;
    }

    clearPortCache();
}

MasterPort &
Bus::getMasterPort(const std::string &if_name, int idx)
{
    if (if_name == "master") {
        // the master index translates directly to the interfaces
        // vector as they are stored first
        return *masterPorts[idx];
    } else  if (if_name == "default") {
        return *masterPorts[defaultPortId];
    } else {
        return MemObject::getMasterPort(if_name, idx);
    }
}

SlavePort &
Bus::getSlavePort(const std::string &if_name, int idx)
{
    if (if_name == "slave") {
        return *slavePorts[nbrMasterPorts + idx];
    } else {
        return MemObject::getSlavePort(if_name, idx);
    }
}

void
Bus::init()
{
    std::vector<BusSlavePort*>::iterator intIter;

    // iterate over our interfaces and determine which of our neighbours
    // are snooping and add them as snoopers
    for (intIter = slavePorts.begin(); intIter != slavePorts.end();
         intIter++) {
        // since there are holes in the vector, check for NULL
        if (*intIter != NULL) {
            if ((*intIter)->getMasterPort().isSnooping()) {
                DPRINTF(BusAddrRanges, "Adding snooping neighbour %s\n",
                        (*intIter)->getMasterPort().name());
                snoopPorts.push_back(*intIter);
            }
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

/** Function called by the port when the bus is receiving a Timing
 * transaction.*/
bool
Bus::recvTiming(PacketPtr pkt)
{
    // called for both requests and responses

    // get the source id and port
    Packet::NodeID src_id = pkt->getSrc();

    // determine the source port based on the id
    Port *src_port = slavePorts[src_id] ?
        (Port*) slavePorts[src_id] : (Port*) masterPorts[src_id];

    // If the bus is busy, or other devices are in line ahead of the current
    // one, put this device on the retry list.
    if (!pkt->isExpressSnoop() &&
        (tickNextIdle > curTick() ||
         (!retryList.empty() && (!inRetry || src_port != retryList.front()))))
    {
        addToRetryList(src_port);
        DPRINTF(Bus, "recvTiming: src %d dst %d %s 0x%x BUSY\n",
                src_id, pkt->getDest(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(Bus, "recvTiming: src %d dst %d %s 0x%x\n",
            src_id, pkt->getDest(), pkt->cmdString(), pkt->getAddr());

    Tick headerFinishTime = pkt->isExpressSnoop() ? 0 : calcPacketTiming(pkt);
    Tick packetFinishTime = pkt->isExpressSnoop() ? 0 : pkt->finishTime;

    Packet::NodeID dest = pkt->getDest();
    int dest_id;
    Port *dest_port;

    if (pkt->isRequest()) {
        // the packet is a memory-mapped request and should be broadcasted to
        // our snoopers
        assert(dest == Packet::Broadcast);

        SnoopIter s_end = snoopPorts.end();
        for (SnoopIter s_iter = snoopPorts.begin(); s_iter != s_end; s_iter++) {
            BusSlavePort *p = *s_iter;
            // we got this request from a snooping master
            // (corresponding to our own slave port that is also in
            // snoopPorts) and should not send it back to where it
            // came from
            if (p->getId() != src_id) {
                // cache is not allowed to refuse snoop
                bool success M5_VAR_USED = p->sendTiming(pkt);
                assert(success);
            }
        }

        // since it is a request, similar to functional and atomic,
        // determine the destination based on the address and forward
        // through the corresponding master port
        dest_id = findPort(pkt->getAddr());
        dest_port = masterPorts[dest_id];
    } else {
        // the packet is a response, and it should always go back to
        // the port determined by the destination field
        dest_id = dest;
        assert(dest_id != src_id); // catch infinite loops
        dest_port = slavePorts[dest_id] ?
            (Port*) slavePorts[dest_id] : (Port*) masterPorts[dest_id];

            // a normal response from the memory system (i.e. from a
            // connected slave) should always go back to the master
            // that issued it through one of our slave ports, however
            // if this is a snoop response it could go either way, for
            // example, it could be coming from a slave port
            // connecting an L1 with a coherent master and another L1
            // coherent master (one of our slave ports), or coming
            // from the L1 and going to the L2 slave port (through one
            // of our master ports)
    }

    assert(dest_port != NULL);

    // if this is a snoop from a slave (corresponding to our own
    // master), i.e. the memory side of the bus, then do not send it
    // back to where it came from
    if (dest_id != src_id) {
        // send to actual target
        if (!dest_port->sendTiming(pkt))  {
            // Packet not successfully sent. Leave or put it on the retry list.
            // illegal to block responses... can lead to deadlock
            assert(!pkt->isResponse());
            // It's also illegal to force a transaction to retry after
            // someone else has committed to respond.
            assert(!pkt->memInhibitAsserted());
            DPRINTF(Bus, "recvTiming: src %d dst %d %s 0x%x TGT RETRY\n",
                    src_id, pkt->getDest(), pkt->cmdString(), pkt->getAddr());
            addToRetryList(src_port);
            occupyBus(headerFinishTime);
            return false;
        }
        // send OK, fall through... pkt may have been deleted by
        // target at this point, so it should *not* be referenced
        // again.  We'll set it to NULL here just to be safe.
        pkt = NULL;
    }

    occupyBus(packetFinishTime);

    // Packet was successfully sent.
    // Also take care of retries
    if (inRetry) {
        DPRINTF(Bus, "Remove retry from list %d\n", src_id);
        retryList.pop_front();
        inRetry = false;
    }
    return true;
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
Bus::recvRetry(int id)
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

int
Bus::findPort(Addr addr)
{
    /* An interval tree would be a better way to do this. --ali. */
    int dest_id;

    dest_id = checkPortCache(addr);
    if (dest_id != INVALID_PORT_ID)
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
                return defaultPortId;
            }
        }
    } else if (defaultPortId != INVALID_PORT_ID) {
        DPRINTF(Bus, "Unable to find destination for addr %#llx, "
                "will use default port\n", addr);
        return defaultPortId;
    }

    // we should use the range for the default port and it did not
    // match, or the default port is not set
    fatal("Unable to find destination for addr %#llx on bus %s\n", addr,
          name());
}


/** Function called by the port when the bus is receiving a Atomic
 * transaction.*/
Tick
Bus::recvAtomic(PacketPtr pkt)
{
    DPRINTF(Bus, "recvAtomic: packet src %d dest %d addr 0x%x cmd %s\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString());

    // we should always see a request routed based on the address
    assert(pkt->getDest() == Packet::Broadcast);
    assert(pkt->isRequest());

    // the packet may be changed by another bus on snoops, record the
    // source id here
    Packet::NodeID src_id = pkt->getSrc();

    // record the original command to enable us to restore it between
    // snoops so that additional snoops can take place properly
    MemCmd orig_cmd = pkt->cmd;
    MemCmd snoop_response_cmd = MemCmd::InvalidCmd;
    Tick snoop_response_latency = 0;

    SnoopIter s_end = snoopPorts.end();
    for (SnoopIter s_iter = snoopPorts.begin(); s_iter != s_end; s_iter++) {
        BusSlavePort *p = *s_iter;
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (p->getId() != src_id) {
            Tick latency = p->sendAtomic(pkt);
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
                pkt->setSrc(src_id);
                pkt->setDest(Packet::Broadcast);
            }
        }
    }

    // even if we had a snoop response, we must continue and also
    // perform the actual request at the destination
    int dest_id = findPort(pkt->getAddr());

    Tick response_latency = 0;

    // if this is a snoop from a slave (corresponding to our own
    // master), i.e. the memory side of the bus, then do not send it
    // back to where it came from
    if (dest_id != src_id) {
        response_latency = masterPorts[dest_id]->sendAtomic(pkt);
    }

    // if we got a response from a snooper, restore it here
    if (snoop_response_cmd != MemCmd::InvalidCmd) {
        // no one else should have responded
        assert(!pkt->isResponse());
        assert(pkt->cmd == orig_cmd);
        pkt->cmd = snoop_response_cmd;
        response_latency = snoop_response_latency;
    }

    // why do we have this packet field and the return value both???
    pkt->finishTime = curTick() + response_latency;
    return response_latency;
}

/** Function called by the port when the bus is receiving a Functional
 * transaction.*/
void
Bus::recvFunctional(PacketPtr pkt)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(Bus,
                "recvFunctional: packet src %d dest %d addr 0x%x cmd %s\n",
                pkt->getSrc(), pkt->getDest(), pkt->getAddr(),
                pkt->cmdString());
    }

    // we should always see a request routed based on the address
    assert(pkt->getDest() == Packet::Broadcast);
    assert(pkt->isRequest());

    // the packet may be changed by another bus on snoops, record the
    // source id here
    Packet::NodeID src_id = pkt->getSrc();

    SnoopIter s_end = snoopPorts.end();
    for (SnoopIter s_iter = snoopPorts.begin(); s_iter != s_end; s_iter++) {
        BusSlavePort *p = *s_iter;
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (p->getId() != src_id) {
            p->sendFunctional(pkt);

            // if we get a response we are done
            if (pkt->isResponse()) {
                break;
            }
        }
    }

    // there is no need to continue if the snooping has found what we
    // were looking for and the packet is already a response
    if (!pkt->isResponse()) {
        int dest_id = findPort(pkt->getAddr());

        // if this is a snoop from a slave (corresponding to our own
        // master), i.e. the memory side of the bus, then do not send
        // it back to where it came from,
        if (dest_id != src_id) {
            masterPorts[dest_id]->sendFunctional(pkt);
        }
    }
}

/** Function called by the port when the bus is receiving a range change.*/
void
Bus::recvRangeChange(int id)
{
    AddrRangeList ranges;
    AddrRangeIter iter;

    if (inRecvRangeChange.count(id))
        return;
    inRecvRangeChange.insert(id);

    DPRINTF(BusAddrRanges, "received RangeChange from device id %d\n", id);

    clearPortCache();
    if (id == defaultPortId) {
        defaultRange.clear();
        // Only try to update these ranges if the user set a default responder.
        if (useDefaultRange) {
            AddrRangeList ranges =
                masterPorts[id]->getSlavePort().getAddrRanges();
            for(iter = ranges.begin(); iter != ranges.end(); iter++) {
                defaultRange.push_back(*iter);
                DPRINTF(BusAddrRanges, "Adding range %#llx - %#llx for default range\n",
                        iter->start, iter->end);
            }
        }
    } else {

        assert(id < masterPorts.size() && id >= 0);
        BusMasterPort *port = masterPorts[id];

        // Clean out any previously existent ids
        for (PortIter portIter = portMap.begin();
             portIter != portMap.end(); ) {
            if (portIter->second == id)
                portMap.erase(portIter++);
            else
                portIter++;
        }

        ranges = port->getSlavePort().getAddrRanges();

        for (iter = ranges.begin(); iter != ranges.end(); iter++) {
            DPRINTF(BusAddrRanges, "Adding range %#llx - %#llx for id %d\n",
                    iter->start, iter->end, id);
            if (portMap.insert(*iter, id) == portMap.end()) {
                int conflict_id = portMap.find(*iter)->second;
                fatal("%s has two ports with same range:\n\t%s\n\t%s\n",
                      name(), masterPorts[id]->getSlavePort().name(),
                      masterPorts[conflict_id]->getSlavePort().name());
            }
        }
    }
    DPRINTF(BusAddrRanges, "port list has %d entries\n", portMap.size());

    // tell all our peers that our address range has changed.
    // Don't tell the device that caused this change, it already knows
    std::vector<BusSlavePort*>::const_iterator intIter;

    for (intIter = slavePorts.begin(); intIter != slavePorts.end(); intIter++)
        if (*intIter != NULL && (*intIter)->getId() != id)
            (*intIter)->sendRangeChange();

    inRecvRangeChange.erase(id);
}

AddrRangeList
Bus::getAddrRanges(int id)
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
        if (portIter->second != id && !subset) {
            ranges.push_back(portIter->first);
            DPRINTF(BusAddrRanges, "  -- %#llx : %#llx\n",
                    portIter->first.start, portIter->first.end);
        }
    }

    return ranges;
}

bool
Bus::isSnooping(int id) const
{
    // in essence, answer the question if there are snooping ports
    return !snoopPorts.empty();
}

unsigned
Bus::findBlockSize(int id)
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
    SnoopIter s_end = snoopPorts.end();
    for (SnoopIter s_iter = snoopPorts.begin(); s_iter != s_end; s_iter++) {
        unsigned tmp_bs = (*s_iter)->peerBlockSize();
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
