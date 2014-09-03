/*
 * Copyright (c) 2011-2013 ARM Limited
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
      useDefaultRange(p->use_default_range)
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

void
BaseBus::calcPacketTiming(PacketPtr pkt)
{
    // the bus will be called at a time that is not necessarily
    // coinciding with its own clock, so start by determining how long
    // until the next clock edge (could be zero)
    Tick offset = clockEdge() - curTick();

    // determine how many cycles are needed to send the data
    unsigned dataCycles = pkt->hasData() ? divCeil(pkt->getSize(), width) : 0;

    // before setting the bus delay fields of the packet, ensure that
    // the delay from any previous bus has been accounted for
    if (pkt->busFirstWordDelay != 0 || pkt->busLastWordDelay != 0)
        panic("Packet %s already has bus delay (%d, %d) that should be "
              "accounted for.\n", pkt->cmdString(), pkt->busFirstWordDelay,
              pkt->busLastWordDelay);

    // The first word will be delivered on the cycle after the header.
    pkt->busFirstWordDelay = (headerCycles + 1) * clockPeriod() + offset;

    // Note that currently busLastWordDelay can be smaller than
    // busFirstWordDelay if the packet has no data
    pkt->busLastWordDelay = (headerCycles + dataCycles) * clockPeriod() +
        offset;
}

template <typename SrcType, typename DstType>
BaseBus::Layer<SrcType,DstType>::Layer(DstType& _port, BaseBus& _bus,
                                       const std::string& _name) :
    port(_port), bus(_bus), _name(_name), state(IDLE), drainManager(NULL),
    waitingForPeer(NULL), releaseEvent(this)
{
}

template <typename SrcType, typename DstType>
void BaseBus::Layer<SrcType,DstType>::occupyLayer(Tick until)
{
    // ensure the state is busy at this point, as the bus should
    // transition from idle as soon as it has decided to forward the
    // packet to prevent any follow-on calls to sendTiming seeing an
    // unoccupied bus
    assert(state == BUSY);

    // until should never be 0 as express snoops never occupy the bus
    assert(until != 0);
    bus.schedule(releaseEvent, until);

    // account for the occupied ticks
    occupancy += until - curTick();

    DPRINTF(BaseBus, "The bus is now busy from tick %d to %d\n",
            curTick(), until);
}

template <typename SrcType, typename DstType>
bool
BaseBus::Layer<SrcType,DstType>::tryTiming(SrcType* src_port)
{
    // if we are in the retry state, we will not see anything but the
    // retrying port (or in the case of the snoop ports the snoop
    // response port that mirrors the actual slave port) as we leave
    // this state again in zero time if the peer does not immediately
    // call the bus when receiving the retry

    // first we see if the layer is busy, next we check if the
    // destination port is already engaged in a transaction waiting
    // for a retry from the peer
    if (state == BUSY || waitingForPeer != NULL) {
        // the port should not be waiting already
        assert(std::find(waitingForLayer.begin(), waitingForLayer.end(),
                         src_port) == waitingForLayer.end());

        // put the port at the end of the retry list waiting for the
        // layer to be freed up (and in the case of a busy peer, for
        // that transaction to go through, and then the bus to free
        // up)
        waitingForLayer.push_back(src_port);
        return false;
    }

    // update the state to busy
    state = BUSY;

    return true;
}

template <typename SrcType, typename DstType>
void
BaseBus::Layer<SrcType,DstType>::succeededTiming(Tick busy_time)
{
    // we should have gone from idle or retry to busy in the tryTiming
    // test
    assert(state == BUSY);

    // occupy the bus accordingly
    occupyLayer(busy_time);
}

template <typename SrcType, typename DstType>
void
BaseBus::Layer<SrcType,DstType>::failedTiming(SrcType* src_port,
                                              Tick busy_time)
{
    // ensure no one got in between and tried to send something to
    // this port
    assert(waitingForPeer == NULL);

    // if the source port is the current retrying one or not, we have
    // failed in forwarding and should track that we are now waiting
    // for the peer to send a retry
    waitingForPeer = src_port;

    // we should have gone from idle or retry to busy in the tryTiming
    // test
    assert(state == BUSY);

    // occupy the bus accordingly
    occupyLayer(busy_time);
}

template <typename SrcType, typename DstType>
void
BaseBus::Layer<SrcType,DstType>::releaseLayer()
{
    // releasing the bus means we should now be idle
    assert(state == BUSY);
    assert(!releaseEvent.scheduled());

    // update the state
    state = IDLE;

    // bus layer is now idle, so if someone is waiting we can retry
    if (!waitingForLayer.empty()) {
        // there is no point in sending a retry if someone is still
        // waiting for the peer
        if (waitingForPeer == NULL)
            retryWaiting();
    } else if (waitingForPeer == NULL && drainManager) {
        DPRINTF(Drain, "Bus done draining, signaling drain manager\n");
        //If we weren't able to drain before, do it now.
        drainManager->signalDrainDone();
        // Clear the drain event once we're done with it.
        drainManager = NULL;
    }
}

template <typename SrcType, typename DstType>
void
BaseBus::Layer<SrcType,DstType>::retryWaiting()
{
    // this should never be called with no one waiting
    assert(!waitingForLayer.empty());

    // we always go to retrying from idle
    assert(state == IDLE);

    // update the state
    state = RETRY;

    // set the retrying port to the front of the retry list and pop it
    // off the list
    SrcType* retryingPort = waitingForLayer.front();
    waitingForLayer.pop_front();

    // tell the port to retry, which in some cases ends up calling the
    // bus
    retryingPort->sendRetry();

    // If the bus is still in the retry state, sendTiming wasn't
    // called in zero time (e.g. the cache does this), burn a cycle
    if (state == RETRY) {
        // update the state to busy and reset the retrying port, we
        // have done our bit and sent the retry
        state = BUSY;

        // occupy the bus layer until the next cycle ends
        occupyLayer(bus.clockEdge(Cycles(1)));
    }
}

template <typename SrcType, typename DstType>
void
BaseBus::Layer<SrcType,DstType>::recvRetry()
{
    // we should never get a retry without having failed to forward
    // something to this port
    assert(waitingForPeer != NULL);

    // add the port where the failed packet originated to the front of
    // the waiting ports for the layer, this allows us to call retry
    // on the port immediately if the bus layer is idle
    waitingForLayer.push_front(waitingForPeer);

    // we are no longer waiting for the peer
    waitingForPeer = NULL;

    // if the bus layer is idle, retry this port straight away, if we
    // are busy, then simply let the port wait for its turn
    if (state == IDLE) {
        retryWaiting();
    } else {
        assert(state == BUSY);
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
        DPRINTF(BusAddrRanges, "Aggregating bus ranges\n");
        busRanges.clear();

        // start out with the default range
        if (useDefaultRange) {
            if (!gotAddrRanges[defaultPortID])
                fatal("Bus %s uses default range, but none provided",
                      name());

            busRanges.push_back(defaultRange);
            DPRINTF(BusAddrRanges, "-- Adding default %s\n",
                    defaultRange.to_string());
        }

        // merge all interleaved ranges and add any range that is not
        // a subset of the default range
        std::vector<AddrRange> intlv_ranges;
        for (AddrRangeMap<PortID>::const_iterator r = portMap.begin();
             r != portMap.end(); ++r) {
            // if the range is interleaved then save it for now
            if (r->first.interleaved()) {
                // if we already got interleaved ranges that are not
                // part of the same range, then first do a merge
                // before we add the new one
                if (!intlv_ranges.empty() &&
                    !intlv_ranges.back().mergesWith(r->first)) {
                    DPRINTF(BusAddrRanges, "-- Merging range from %d ranges\n",
                            intlv_ranges.size());
                    AddrRange merged_range(intlv_ranges);
                    // next decide if we keep the merged range or not
                    if (!(useDefaultRange &&
                          merged_range.isSubset(defaultRange))) {
                        busRanges.push_back(merged_range);
                        DPRINTF(BusAddrRanges, "-- Adding merged range %s\n",
                                merged_range.to_string());
                    }
                    intlv_ranges.clear();
                }
                intlv_ranges.push_back(r->first);
            } else {
                // keep the current range if not a subset of the default
                if (!(useDefaultRange &&
                      r->first.isSubset(defaultRange))) {
                    busRanges.push_back(r->first);
                    DPRINTF(BusAddrRanges, "-- Adding range %s\n",
                            r->first.to_string());
                }
            }
        }

        // if there is still interleaved ranges waiting to be merged,
        // go ahead and do it
        if (!intlv_ranges.empty()) {
            DPRINTF(BusAddrRanges, "-- Merging range from %d ranges\n",
                    intlv_ranges.size());
            AddrRange merged_range(intlv_ranges);
            if (!(useDefaultRange && merged_range.isSubset(defaultRange))) {
                busRanges.push_back(merged_range);
                DPRINTF(BusAddrRanges, "-- Adding merged range %s\n",
                        merged_range.to_string());
            }
        }

        // also check that no range partially overlaps with the
        // default range, this has to be done after all ranges are set
        // as there are no guarantees for when the default range is
        // update with respect to the other ones
        if (useDefaultRange) {
            for (AddrRangeConstIter r = busRanges.begin();
                 r != busRanges.end(); ++r) {
                // see if the new range is partially
                // overlapping the default range
                if (r->intersects(defaultRange) &&
                    !r->isSubset(defaultRange))
                    fatal("Range %s intersects the "                    \
                          "default range of %s but is not a "           \
                          "subset\n", r->to_string(), name());
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

    DPRINTF(BusAddrRanges, "Received address range request\n");

    return busRanges;
}

void
BaseBus::regStats()
{
    using namespace Stats;

    transDist
        .init(MemCmd::NUM_MEM_CMDS)
        .name(name() + ".trans_dist")
        .desc("Transaction distribution")
        .flags(nozero);

    // get the string representation of the commands
    for (int i = 0; i < MemCmd::NUM_MEM_CMDS; i++) {
        MemCmd cmd(i);
        const std::string &cstr = cmd.toString();
        transDist.subname(i, cstr);
    }

    pktCount
        .init(slavePorts.size(), masterPorts.size())
        .name(name() + ".pkt_count")
        .desc("Packet count per connected master and slave (bytes)")
        .flags(total | nozero | nonan);

    totPktSize
        .init(slavePorts.size(), masterPorts.size())
        .name(name() + ".tot_pkt_size")
        .desc("Cumulative packet size per connected master and slave (bytes)")
        .flags(total | nozero | nonan);

    // both the packet count and total size are two-dimensional
    // vectors, indexed by slave port id and master port id, thus the
    // neighbouring master and slave, they do not differentiate what
    // came from the master and was forwarded to the slave (requests
    // and snoop responses) and what came from the slave and was
    // forwarded to the master (responses and snoop requests)
    for (int i = 0; i < slavePorts.size(); i++) {
        pktCount.subname(i, slavePorts[i]->getMasterPort().name());
        totPktSize.subname(i, slavePorts[i]->getMasterPort().name());
        for (int j = 0; j < masterPorts.size(); j++) {
            pktCount.ysubname(j, masterPorts[j]->getSlavePort().name());
            totPktSize.ysubname(j, masterPorts[j]->getSlavePort().name());
        }
    }
}

template <typename SrcType, typename DstType>
unsigned int
BaseBus::Layer<SrcType,DstType>::drain(DrainManager *dm)
{
    //We should check that we're not "doing" anything, and that noone is
    //waiting. We might be idle but have someone waiting if the device we
    //contacted for a retry didn't actually retry.
    if (state != IDLE) {
        DPRINTF(Drain, "Bus not drained\n");
        drainManager = dm;
        return 1;
    }
    return 0;
}

template <typename SrcType, typename DstType>
void
BaseBus::Layer<SrcType,DstType>::regStats()
{
    using namespace Stats;

    occupancy
        .name(name() + ".occupancy")
        .desc("Layer occupancy (ticks)")
        .flags(nozero);

    utilization
        .name(name() + ".utilization")
        .desc("Layer utilization (%)")
        .precision(1)
        .flags(nozero);

    utilization = 100 * occupancy / simTicks;
}

/**
 * Bus layer template instantiations. Could be removed with _impl.hh
 * file, but since there are only two given options (MasterPort and
 * SlavePort) it seems a bit excessive at this point.
 */
template class BaseBus::Layer<SlavePort,MasterPort>;
template class BaseBus::Layer<MasterPort,SlavePort>;
