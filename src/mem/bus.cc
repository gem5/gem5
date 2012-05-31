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

BaseBus::BaseBus(const BaseBusParams *p)
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
}

BaseBus::~BaseBus()
{
    for (MasterPortIter m = masterPorts.begin(); m != masterPorts.end();
         ++m) {
        delete *m;
    }

    for (SlavePortIter s = slavePorts.begin(); s != slavePorts.end();
         ++s) {
        delete *s;
    }
}

MasterPort &
BaseBus::getMasterPort(const std::string &if_name, int idx)
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
BaseBus::getSlavePort(const std::string &if_name, int idx)
{
    if (if_name == "slave" && idx < slavePorts.size()) {
        // the slave port index translates directly to the vector position
        return *slavePorts[idx];
    } else {
        return MemObject::getSlavePort(if_name, idx);
    }
}

Tick
BaseBus::calcPacketTiming(PacketPtr pkt)
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

void BaseBus::occupyBus(Tick until)
{
    if (until == 0) {
        // shortcut for express snoop packets
        return;
    }

    tickNextIdle = until;
    reschedule(busIdleEvent, tickNextIdle, true);

    DPRINTF(BaseBus, "The bus is now occupied from tick %d to %d\n",
            curTick(), tickNextIdle);
}

bool
BaseBus::isOccupied(Port* port)
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

void
BaseBus::succeededTiming(Tick busy_time)
{
    // occupy the bus accordingly
    occupyBus(busy_time);

    // if a retrying port succeeded, also take it off the retry list
    if (inRetry) {
        DPRINTF(BaseBus, "Remove retry from list %s\n",
                retryList.front()->name());
        retryList.pop_front();
        inRetry = false;
    }
}

void
BaseBus::releaseBus()
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
BaseBus::retryWaiting()
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
BaseBus::recvRetry()
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
BaseBus::findPort(Addr addr)
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
                DPRINTF(BusAddrRanges, "  found addr %#llx on default\n",
                        addr);
                return defaultPortID;
            }
        }
    } else if (defaultPortID != InvalidPortID) {
        DPRINTF(BusAddrRanges, "Unable to find destination for addr %#llx, "
                "will use default port\n", addr);
        return defaultPortID;
    }

    // we should use the range for the default port and it did not
    // match, or the default port is not set
    fatal("Unable to find destination for addr %#llx on bus %s\n", addr,
          name());
}

/** Function called by the port when the bus is receiving a range change.*/
void
BaseBus::recvRangeChange(PortID master_port_id)
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
BaseBus::getAddrRanges()
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

unsigned
BaseBus::findBlockSize()
{
    if (cachedBlockSizeValid)
        return cachedBlockSize;

    unsigned max_bs = 0;

    for (MasterPortConstIter m = masterPorts.begin(); m != masterPorts.end();
         ++m) {
        unsigned tmp_bs = (*m)->peerBlockSize();
        if (tmp_bs > max_bs)
            max_bs = tmp_bs;
    }

    for (SlavePortConstIter s = slavePorts.begin(); s != slavePorts.end();
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
BaseBus::drain(Event * de)
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
BaseBus::startup()
{
    if (tickNextIdle < curTick())
        tickNextIdle = (curTick() / clock) * clock + clock;
}
