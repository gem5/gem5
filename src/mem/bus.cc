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
#include "debug/Drain.hh"
#include "mem/bus.hh"

BaseBus::BaseBus(const BaseBusParams *p)
    : MemObject(p),
      headerCycles(p->header_cycles), width(p->width),
      gotAddrRanges(p->port_default_connection_count +
                          p->port_master_connection_count, false),
      gotAllAddrRanges(false), defaultPortID(InvalidPortID),
      useDefaultRange(p->use_default_range),
      blockSize(p->block_size)
{}

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

void
BaseBus::init()
{
    // determine the maximum peer block size, look at both the
    // connected master and slave modules
    uint32_t peer_block_size = 0;

    for (MasterPortConstIter m = masterPorts.begin(); m != masterPorts.end();
         ++m) {
        peer_block_size = std::max((*m)->peerBlockSize(), peer_block_size);
    }

    for (SlavePortConstIter s = slavePorts.begin(); s != slavePorts.end();
         ++s) {
        peer_block_size = std::max((*s)->peerBlockSize(), peer_block_size);
    }

    // if the peers do not have a block size, use the default value
    // set through the bus parameters
    if (peer_block_size != 0)
        blockSize = peer_block_size;

    // check if the block size is a value known to work
    if (!(blockSize == 16 || blockSize == 32 || blockSize == 64 ||
          blockSize == 128))
        warn_once("Block size is neither 16, 32, 64 or 128 bytes.\n");
}

BaseMasterPort &
BaseBus::getMasterPort(const std::string &if_name, PortID idx)
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

BaseSlavePort &
BaseBus::getSlavePort(const std::string &if_name, PortID idx)
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
    Tick now = nextCycle();

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
    Drainable(),
    bus(_bus), _name(_name), state(IDLE), clock(_clock), drainManager(NULL),
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
    } else if (drainManager) {
        DPRINTF(Drain, "Bus done draining, signaling drain manager\n");
        //If we weren't able to drain before, do it now.
        drainManager->signalDrainDone();
        // Clear the drain event once we're done with it.
        drainManager = NULL;
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
        Tick now = bus.nextCycle();

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
    // we should never see any address lookups before we've got the
    // ranges of all connected slave modules
    assert(gotAllAddrRanges);

    // Check the cache
    PortID dest_id = checkPortCache(addr);
    if (dest_id != InvalidPortID)
        return dest_id;

    // Check the address map interval tree
    PortMapConstIter i = portMap.find(addr);
    if (i != portMap.end()) {
        dest_id = i->second;
        updatePortCache(dest_id, i->first);
        return dest_id;
    }

    // Check if this matches the default range
    if (useDefaultRange) {
        if (defaultRange.contains(addr)) {
            DPRINTF(BusAddrRanges, "  found addr %#llx on default\n",
                    addr);
            return defaultPortID;
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
    DPRINTF(BusAddrRanges, "Received range change from slave port %s\n",
            masterPorts[master_port_id]->getSlavePort().name());

    // remember that we got a range from this master port and thus the
    // connected slave module
    gotAddrRanges[master_port_id] = true;

    // update the global flag
    if (!gotAllAddrRanges) {
        // take a logical AND of all the ports and see if we got
        // ranges from everyone
        gotAllAddrRanges = true;
        std::vector<bool>::const_iterator r = gotAddrRanges.begin();
        while (gotAllAddrRanges &&  r != gotAddrRanges.end()) {
            gotAllAddrRanges &= *r++;
        }
        if (gotAllAddrRanges)
            DPRINTF(BusAddrRanges, "Got address ranges from all slaves\n");
    }

    // note that we could get the range from the default port at any
    // point in time, and we cannot assume that the default range is
    // set before the other ones are, so we do additional checks once
    // all ranges are provided
    if (master_port_id == defaultPortID) {
        // only update if we are indeed checking ranges for the
        // default port since the port might not have a valid range
        // otherwise
        if (useDefaultRange) {
            AddrRangeList ranges = masterPorts[master_port_id]->getAddrRanges();

            if (ranges.size() != 1)
                fatal("Bus %s may only have a single default range",
                      name());

            defaultRange = ranges.front();
        }
    } else {
        // the ports are allowed to update their address ranges
        // dynamically, so remove any existing entries
        if (gotAddrRanges[master_port_id]) {
            for (PortMapIter p = portMap.begin(); p != portMap.end(); ) {
                if (p->second == master_port_id)
                    // erasing invalidates the iterator, so advance it
                    // before the deletion takes place
                    portMap.erase(p++);
                else
                    p++;
            }
        }

        AddrRangeList ranges = masterPorts[master_port_id]->getAddrRanges();

        for (AddrRangeConstIter r = ranges.begin(); r != ranges.end(); ++r) {
            DPRINTF(BusAddrRanges, "Adding range %s for id %d\n",
                    r->to_string(), master_port_id);
            if (portMap.insert(*r, master_port_id) == portMap.end()) {
                PortID conflict_id = portMap.find(*r)->second;
                fatal("%s has two ports with same range:\n\t%s\n\t%s\n",
                      name(),
                      masterPorts[master_port_id]->getSlavePort().name(),
                      masterPorts[conflict_id]->getSlavePort().name());
            }
        }
    }

    // if we have received ranges from all our neighbouring slave
    // modules, go ahead and tell our connected master modules in
    // turn, this effectively assumes a tree structure of the system
    if (gotAllAddrRanges) {
        // also check that no range partially overlaps with the
        // default range, this has to be done after all ranges are set
        // as there are no guarantees for when the default range is
        // update with respect to the other ones
        if (useDefaultRange) {
            for (PortID port_id = 0; port_id < masterPorts.size(); ++port_id) {
                if (port_id == defaultPortID) {
                    if (!gotAddrRanges[port_id])
                        fatal("Bus %s uses default range, but none provided",
                              name());
                } else {
                    AddrRangeList ranges =
                        masterPorts[port_id]->getAddrRanges();

                    for (AddrRangeConstIter r = ranges.begin();
                         r != ranges.end(); ++r) {
                        // see if the new range is partially
                        // overlapping the default range
                        if (r->intersects(defaultRange) &&
                            !r->isSubset(defaultRange))
                            fatal("Range %s intersects the " \
                                  "default range of %s but is not a " \
                                  "subset\n", r->to_string(), name());
                    }
                }
            }
        }

        // tell all our neighbouring master ports that our address
        // ranges have changed
        for (SlavePortConstIter s = slavePorts.begin(); s != slavePorts.end();
             ++s)
            (*s)->sendRangeChange();
    }

    clearPortCache();
}

AddrRangeList
BaseBus::getAddrRanges() const
{
    // we should never be asked without first having sent a range
    // change, and the latter is only done once we have all the ranges
    // of the connected devices
    assert(gotAllAddrRanges);

    // at the moment, this never happens, as there are no cycles in
    // the range queries and no devices on the master side of a bus
    // (CPU, cache, bridge etc) actually care about the ranges of the
    // ports they are connected to

    DPRINTF(BusAddrRanges, "Received address range request, returning:\n");

    // start out with the default range
    AddrRangeList ranges;
    if (useDefaultRange) {
        ranges.push_back(defaultRange);
        DPRINTF(BusAddrRanges, "  -- Default %s\n", defaultRange.to_string());
    }

    // add any range that is not a subset of the default range
    for (PortMapConstIter p = portMap.begin(); p != portMap.end(); ++p) {
        if (useDefaultRange && p->first.isSubset(defaultRange)) {
            DPRINTF(BusAddrRanges, "  -- %s is a subset of default\n",
                    p->first.to_string());
        } else {
            ranges.push_back(p->first);
            DPRINTF(BusAddrRanges, "  -- %s\n", p->first.to_string());
        }
    }

    return ranges;
}

unsigned
BaseBus::deviceBlockSize() const
{
    return blockSize;
}

template <typename PortClass>
unsigned int
BaseBus::Layer<PortClass>::drain(DrainManager *dm)
{
    //We should check that we're not "doing" anything, and that noone is
    //waiting. We might be idle but have someone waiting if the device we
    //contacted for a retry didn't actually retry.
    if (!retryList.empty() || state != IDLE) {
        DPRINTF(Drain, "Bus not drained\n");
        drainManager = dm;
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
