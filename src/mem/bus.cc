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

#include "base/intmath.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "debug/Bus.hh"
#include "debug/BusAddrRanges.hh"
#include "mem/bus.hh"

BaseBus::BaseBus(const BaseBusParams *p)
    : MemObject(p), clock(p->clock),
      headerCycles(p->header_cycles), width(p->width),
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
    Tick now = divCeil(curTick(), clock) * clock;

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

template <typename PortClass>
BaseBus::Layer<PortClass>::Layer(BaseBus& _bus, const std::string& _name,
                                 Tick _clock) :
    bus(_bus), _name(_name), state(IDLE), clock(_clock), drainEvent(NULL),
    releaseEvent(this)
{
}

template <typename PortClass>
void BaseBus::Layer<PortClass>::occupyLayer(Tick until)
{
    // ensure the state is busy or in retry and never idle at this
    // point, as the bus should transition from idle as soon as it has
    // decided to forward the packet to prevent any follow-on calls to
    // sendTiming seeing an unoccupied bus
    assert(state != IDLE);

    // note that we do not change the bus state here, if we are going
    // from idle to busy it is handled by tryTiming, and if we
    // are in retry we should remain in retry such that
    // succeededTiming still sees the accurate state

    // until should never be 0 as express snoops never occupy the bus
    assert(until != 0);
    bus.schedule(releaseEvent, until);

    DPRINTF(BaseBus, "The bus is now busy from tick %d to %d\n",
            curTick(), until);
}

template <typename PortClass>
bool
BaseBus::Layer<PortClass>::tryTiming(PortClass* port)
{
    // first we see if the bus is busy, next we check if we are in a
    // retry with a port other than the current one
    if (state == BUSY || (state == RETRY && port != retryList.front())) {
        // put the port at the end of the retry list
        retryList.push_back(port);
        return false;
    }

    // update the state which is shared for request, response and
    // snoop responses, if we were idle we are now busy, if we are in
    // a retry, then do not change
    if (state == IDLE)
        state = BUSY;

    return true;
}

template <typename PortClass>
void
BaseBus::Layer<PortClass>::succeededTiming(Tick busy_time)
{
    // if a retrying port succeeded, also take it off the retry list
    if (state == RETRY) {
        DPRINTF(BaseBus, "Remove retry from list %s\n",
                retryList.front()->name());
        retryList.pop_front();
        state = BUSY;
    }

    // we should either have gone from idle to busy in the
    // tryTiming test, or just gone from a retry to busy
    assert(state == BUSY);

    // occupy the bus accordingly
    occupyLayer(busy_time);
}

template <typename PortClass>
void
BaseBus::Layer<PortClass>::failedTiming(PortClass* port, Tick busy_time)
{
    // if we are not in a retry, i.e. busy (but never idle), or we are
    // in a retry but not for the current port, then add the port at
    // the end of the retry list
    if (state != RETRY || port != retryList.front()) {
        retryList.push_back(port);
    }

    // even if we retried the current one and did not succeed,
    // we are no longer retrying but instead busy
    state = BUSY;

    // occupy the bus accordingly
    occupyLayer(busy_time);
}

template <typename PortClass>
void
BaseBus::Layer<PortClass>::releaseLayer()
{
    // releasing the bus means we should now be idle
    assert(state == BUSY);
    assert(!releaseEvent.scheduled());

    // update the state
    state = IDLE;

    // bus is now idle, so if someone is waiting we can retry
    if (!retryList.empty()) {
        // note that we block (return false on recvTiming) both
        // because the bus is busy and because the destination is
        // busy, and in the latter case the bus may be released before
        // we see a retry from the destination
        retryWaiting();
    } else if (drainEvent) {
        //If we weren't able to drain before, do it now.
        drainEvent->process();
        // Clear the drain event once we're done with it.
        drainEvent = NULL;
    }
}

template <typename PortClass>
void
BaseBus::Layer<PortClass>::retryWaiting()
{
    // this should never be called with an empty retry list
    assert(!retryList.empty());

    // we always go to retrying from idle
    assert(state == IDLE);

    // update the state which is shared for request, response and
    // snoop responses
    state = RETRY;

    // note that we might have blocked on the receiving port being
    // busy (rather than the bus itself) and now call retry before the
    // destination called retry on the bus
    retryList.front()->sendRetry();

    // If the bus is still in the retry state, sendTiming wasn't
    // called in zero time (e.g. the cache does this)
    if (state == RETRY) {
        retryList.pop_front();

        //Burn a cycle for the missed grant.

        // update the state which is shared for request, response and
        // snoop responses
        state = BUSY;

        // determine the current time rounded to the closest following
        // clock edge
        Tick now = divCeil(curTick(), clock) * clock;

        occupyLayer(now + clock);
    }
}

template <typename PortClass>
void
BaseBus::Layer<PortClass>::recvRetry()
{
    // we got a retry from a peer that we tried to send something to
    // and failed, but we sent it on the account of someone else, and
    // that source port should be on our retry list, however if the
    // bus layer is released before this happens and the retry (from
    // the bus point of view) is successful then this no longer holds
    // and we could in fact have an empty retry list
    if (retryList.empty())
        return;

    // if the bus layer is idle
    if (state == IDLE) {
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
    PortMapConstIter i = portMap.find(RangeSize(addr,1));
    if (i != portMap.end()) {
        dest_id = i->second;
        updatePortCache(dest_id, i->first.start, i->first.end);
        return dest_id;
    }

    // Check if this matches the default range
    if (useDefaultRange) {
        AddrRangeConstIter a_end = defaultRange.end();
        for (AddrRangeConstIter i = defaultRange.begin(); i != a_end; i++) {
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
            // get the address ranges of the connected slave port
            AddrRangeList ranges =
                masterPorts[master_port_id]->getAddrRanges();
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
        for (PortMapIter portIter = portMap.begin();
             portIter != portMap.end(); ) {
            if (portIter->second == master_port_id)
                portMap.erase(portIter++);
            else
                portIter++;
        }

        // get the address ranges of the connected slave port
        ranges = port->getAddrRanges();

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
BaseBus::getAddrRanges() const
{
    AddrRangeList ranges;

    DPRINTF(BusAddrRanges, "received address range request, returning:\n");

    for (AddrRangeConstIter dflt_iter = defaultRange.begin();
         dflt_iter != defaultRange.end(); dflt_iter++) {
        ranges.push_back(*dflt_iter);
        DPRINTF(BusAddrRanges, "  -- Dflt: %#llx : %#llx\n",dflt_iter->start,
                dflt_iter->end);
    }
    for (PortMapConstIter portIter = portMap.begin();
         portIter != portMap.end(); portIter++) {
        bool subset = false;
        for (AddrRangeConstIter dflt_iter = defaultRange.begin();
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

template <typename PortClass>
unsigned int
BaseBus::Layer<PortClass>::drain(Event * de)
{
    //We should check that we're not "doing" anything, and that noone is
    //waiting. We might be idle but have someone waiting if the device we
    //contacted for a retry didn't actually retry.
    if (!retryList.empty() || state != IDLE) {
        drainEvent = de;
        return 1;
    }
    return 0;
}

/**
 * Bus layer template instantiations. Could be removed with _impl.hh
 * file, but since there are only two given options (MasterPort and
 * SlavePort) it seems a bit excessive at this point.
 */
template class BaseBus::Layer<SlavePort>;
template class BaseBus::Layer<MasterPort>;
