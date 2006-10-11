/*
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
 */

/**
 * @file
 * Definition of a bus object.
 */


#include "base/misc.hh"
#include "base/trace.hh"
#include "mem/bus.hh"
#include "sim/builder.hh"

Port *
Bus::getPort(const std::string &if_name, int idx)
{
    if (if_name == "default")
        if (defaultPort == NULL) {
            defaultPort = new BusPort(csprintf("%s-default",name()), this,
                    defaultId);
            return defaultPort;
        } else
            fatal("Default port already set\n");

    // if_name ignored?  forced to be empty?
    int id = interfaces.size();
    BusPort *bp = new BusPort(csprintf("%s-p%d", name(), id), this, id);
    interfaces.push_back(bp);
    return bp;
}

/** Get the ranges of anyone other buses that we are connected to. */
void
Bus::init()
{
    std::vector<Port*>::iterator intIter;

    for (intIter = interfaces.begin(); intIter != interfaces.end(); intIter++)
        (*intIter)->sendStatusChange(Port::RangeChange);
}

Bus::BusFreeEvent::BusFreeEvent(Bus *_bus) : Event(&mainEventQueue), bus(_bus)
{}

void Bus::BusFreeEvent::process()
{
    bus->recvRetry(-1);
}

const char * Bus::BusFreeEvent::description()
{
    return "bus became available";
}

/** Function called by the port when the bus is receiving a Timing
 * transaction.*/
bool
Bus::recvTiming(Packet *pkt)
{
    Port *port;
    DPRINTF(Bus, "recvTiming: packet src %d dest %d addr 0x%x cmd %s\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString());

    Port *pktPort = interfaces[pkt->getSrc()];

    // If the bus is busy, or other devices are in line ahead of the current
    // one, put this device on the retry list.
    if (tickNextIdle > curTick ||
            (retryList.size() && (!inRetry || pktPort != retryList.front()))) {
        DPRINTF(Bus, "Adding RETRY for %i\n", pktPort);
        addToRetryList(pktPort);
        return false;
    }

    short dest = pkt->getDest();
    if (dest == Packet::Broadcast) {
        if (timingSnoop(pkt)) {
            pkt->flags |= SNOOP_COMMIT;
            bool success = timingSnoop(pkt);
            assert(success);
            if (pkt->flags & SATISFIED) {
                //Cache-Cache transfer occuring
                if (inRetry) {
                    DPRINTF(Bus, "Removing RETRY %i\n", retryList.front());
                    retryList.pop_front();
                    inRetry = false;
                }
                return true;
            }
            port = findPort(pkt->getAddr(), pkt->getSrc());
        } else {
            //Snoop didn't succeed
            DPRINTF(Bus, "Snoop caused adding to RETRY list %i\n", pktPort);
            addToRetryList(pktPort);
            return false;
        }
    } else {
        assert(dest >= 0 && dest < interfaces.size());
        assert(dest != pkt->getSrc()); // catch infinite loops
        port = interfaces[dest];
    }

    //Bring tickNextIdle up to the present tick
    //There is some potential ambiguity where a cycle starts, which might make
    //a difference when devices are acting right around a cycle boundary. Using
    //a < allows things which happen exactly on a cycle boundary to take up only
    //the following cycle. Anthing that happens later will have to "wait" for
    //the end of that cycle, and then start using the bus after that.
    while (tickNextIdle < curTick)
        tickNextIdle += clock;

    // The packet will be sent. Figure out how long it occupies the bus, and
    // how much of that time is for the first "word", aka bus width.
    int numCycles = 0;
    // Requests need one cycle to send an address
    if (pkt->isRequest())
        numCycles++;
    else if (pkt->isResponse() || pkt->hasData()) {
        // If a packet has data, it needs ceil(size/width) cycles to send it
        // We're using the "adding instead of dividing" trick again here
        if (pkt->hasData()) {
            int dataSize = pkt->getSize();
            for (int transmitted = 0; transmitted < dataSize;
                    transmitted += width) {
                numCycles++;
            }
        } else {
            // If the packet didn't have data, it must have been a response.
            // Those use the bus for one cycle to send their data.
            numCycles++;
        }
    }

    // The first word will be delivered after the current tick, the delivery
    // of the address if any, and one bus cycle to deliver the data
    pkt->firstWordTime =
        tickNextIdle +
        pkt->isRequest() ? clock : 0 +
        clock;

    //Advance it numCycles bus cycles.
    //XXX Should this use the repeated addition trick as well?
    tickNextIdle += (numCycles * clock);
    if (!busIdle.scheduled()) {
        busIdle.schedule(tickNextIdle);
    } else {
        busIdle.reschedule(tickNextIdle);
    }
    DPRINTF(Bus, "The bus is now occupied from tick %d to %d\n",
            curTick, tickNextIdle);

    // The bus will become idle once the current packet is delivered.
    pkt->finishTime = tickNextIdle;

    if (port->sendTiming(pkt))  {
        // Packet was successfully sent. Return true.
        // Also take care of retries
        if (inRetry) {
            DPRINTF(Bus, "Remove retry from list %i\n", retryList.front());
            retryList.pop_front();
            inRetry = false;
        }
        return true;
    }

    // Packet not successfully sent. Leave or put it on the retry list.
    DPRINTF(Bus, "Adding a retry to RETRY list %i\n", pktPort);
    addToRetryList(pktPort);
    return false;
}

void
Bus::recvRetry(int id)
{
    DPRINTF(Bus, "Received a retry\n");
    // If there's anything waiting...
    if (retryList.size()) {
        //retryingPort = retryList.front();
        inRetry = true;
        DPRINTF(Bus, "Sending a retry\n");
        retryList.front()->sendRetry();
        // If inRetry is still true, sendTiming wasn't called
        if (inRetry)
            panic("Port %s didn't call sendTiming in it's recvRetry\n",\
                    retryList.front()->getPeer()->name());
        //assert(!inRetry);
    }
}

Port *
Bus::findPort(Addr addr, int id)
{
    /* An interval tree would be a better way to do this. --ali. */
    int dest_id = -1;
    int i = 0;
    bool found = false;
    AddrRangeIter iter;

    while (i < portList.size() && !found)
    {
        if (portList[i].range == addr) {
            dest_id = portList[i].portId;
            found = true;
            DPRINTF(Bus, "  found addr %#llx on device %d\n", addr, dest_id);
        }
        i++;
    }

    // Check if this matches the default range
    if (dest_id == -1) {
        for (iter = defaultRange.begin(); iter != defaultRange.end(); iter++) {
            if (*iter == addr) {
                DPRINTF(Bus, "  found addr %#llx on default\n", addr);
                return defaultPort;
            }
        }
        panic("Unable to find destination for addr: %#llx", addr);
    }


    // we shouldn't be sending this back to where it came from
    assert(dest_id != id);

    return interfaces[dest_id];
}

std::vector<int>
Bus::findSnoopPorts(Addr addr, int id)
{
    int i = 0;
    AddrRangeIter iter;
    std::vector<int> ports;

    while (i < portSnoopList.size())
    {
        if (portSnoopList[i].range == addr && portSnoopList[i].portId != id) {
            //Careful  to not overlap ranges
            //or snoop will be called more than once on the port
            ports.push_back(portSnoopList[i].portId);
//            DPRINTF(Bus, "  found snoop addr %#llx on device%d\n", addr,
//                    portSnoopList[i].portId);
        }
        i++;
    }
    return ports;
}

void
Bus::atomicSnoop(Packet *pkt)
{
    std::vector<int> ports = findSnoopPorts(pkt->getAddr(), pkt->getSrc());

    while (!ports.empty())
    {
        interfaces[ports.back()]->sendAtomic(pkt);
        ports.pop_back();
    }
}

void
Bus::functionalSnoop(Packet *pkt)
{
    std::vector<int> ports = findSnoopPorts(pkt->getAddr(), pkt->getSrc());

    while (!ports.empty())
    {
        interfaces[ports.back()]->sendFunctional(pkt);
        ports.pop_back();
    }
}

bool
Bus::timingSnoop(Packet *pkt)
{
    std::vector<int> ports = findSnoopPorts(pkt->getAddr(), pkt->getSrc());
    bool success = true;

    while (!ports.empty() && success)
    {
        success = interfaces[ports.back()]->sendTiming(pkt);
        ports.pop_back();
    }

    return success;
}


/** Function called by the port when the bus is receiving a Atomic
 * transaction.*/
Tick
Bus::recvAtomic(Packet *pkt)
{
    DPRINTF(Bus, "recvAtomic: packet src %d dest %d addr 0x%x cmd %s\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString());
    assert(pkt->getDest() == Packet::Broadcast);
    atomicSnoop(pkt);
    return findPort(pkt->getAddr(), pkt->getSrc())->sendAtomic(pkt);
}

/** Function called by the port when the bus is receiving a Functional
 * transaction.*/
void
Bus::recvFunctional(Packet *pkt)
{
    DPRINTF(Bus, "recvFunctional: packet src %d dest %d addr 0x%x cmd %s\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString());
    assert(pkt->getDest() == Packet::Broadcast);
    functionalSnoop(pkt);
    findPort(pkt->getAddr(), pkt->getSrc())->sendFunctional(pkt);
}

/** Function called by the port when the bus is receiving a status change.*/
void
Bus::recvStatusChange(Port::Status status, int id)
{
    AddrRangeList ranges;
    AddrRangeList snoops;
    int x;
    AddrRangeIter iter;

    assert(status == Port::RangeChange &&
           "The other statuses need to be implemented.");

    DPRINTF(BusAddrRanges, "received RangeChange from device id %d\n", id);

    if (id == defaultId) {
        defaultRange.clear();
        defaultPort->getPeerAddressRanges(ranges, snoops);
        assert(snoops.size() == 0);
        for(iter = ranges.begin(); iter != ranges.end(); iter++) {
            defaultRange.push_back(*iter);
            DPRINTF(BusAddrRanges, "Adding range %#llx - %#llx for default range\n",
                    iter->start, iter->end);
        }
    } else {

        assert((id < interfaces.size() && id >= 0) || id == -1);
        Port *port = interfaces[id];
        std::vector<DevMap>::iterator portIter;
        std::vector<DevMap>::iterator snoopIter;

        // Clean out any previously existent ids
        for (portIter = portList.begin(); portIter != portList.end(); ) {
            if (portIter->portId == id)
                portIter = portList.erase(portIter);
            else
                portIter++;
        }

        for (snoopIter = portSnoopList.begin(); snoopIter != portSnoopList.end(); ) {
            if (snoopIter->portId == id)
                snoopIter = portSnoopList.erase(snoopIter);
            else
                snoopIter++;
        }

        port->getPeerAddressRanges(ranges, snoops);

        for(iter = snoops.begin(); iter != snoops.end(); iter++) {
            DevMap dm;
            dm.portId = id;
            dm.range = *iter;

            DPRINTF(BusAddrRanges, "Adding snoop range %#llx - %#llx for id %d\n",
                    dm.range.start, dm.range.end, id);
            portSnoopList.push_back(dm);
        }

        for(iter = ranges.begin(); iter != ranges.end(); iter++) {
            DevMap dm;
            dm.portId = id;
            dm.range = *iter;

            DPRINTF(BusAddrRanges, "Adding range %#llx - %#llx for id %d\n",
                    dm.range.start, dm.range.end, id);
            portList.push_back(dm);
        }
    }
    DPRINTF(MMU, "port list has %d entries\n", portList.size());

    // tell all our peers that our address range has changed.
    // Don't tell the device that caused this change, it already knows
    for (x = 0; x < interfaces.size(); x++)
        if (x != id)
            interfaces[x]->sendStatusChange(Port::RangeChange);

    if (id != defaultId && defaultPort)
        defaultPort->sendStatusChange(Port::RangeChange);
}

void
Bus::addressRanges(AddrRangeList &resp, AddrRangeList &snoop, int id)
{
    std::vector<DevMap>::iterator portIter;
    AddrRangeIter dflt_iter;
    bool subset;

    resp.clear();
    snoop.clear();

    DPRINTF(BusAddrRanges, "received address range request, returning:\n");

    for (dflt_iter = defaultRange.begin(); dflt_iter != defaultRange.end();
            dflt_iter++) {
        resp.push_back(*dflt_iter);
        DPRINTF(BusAddrRanges, "  -- %#llx : %#llx\n",dflt_iter->start,
                dflt_iter->end);
    }
    for (portIter = portList.begin(); portIter != portList.end(); portIter++) {
        subset = false;
        for (dflt_iter = defaultRange.begin(); dflt_iter != defaultRange.end();
                dflt_iter++) {
            if ((portIter->range.start < dflt_iter->start &&
                portIter->range.end >= dflt_iter->start) ||
               (portIter->range.start < dflt_iter->end &&
                portIter->range.end >= dflt_iter->end))
                fatal("Devices can not set ranges that itersect the default set\
                        but are not a subset of the default set.\n");
            if (portIter->range.start >= dflt_iter->start &&
                portIter->range.end <= dflt_iter->end) {
                subset = true;
                DPRINTF(BusAddrRanges, "  -- %#llx : %#llx is a SUBSET\n",
                    portIter->range.start, portIter->range.end);
            }
        }
        if (portIter->portId != id && !subset) {
            resp.push_back(portIter->range);
            DPRINTF(BusAddrRanges, "  -- %#llx : %#llx\n",
                    portIter->range.start, portIter->range.end);
        }
    }
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Bus)

    Param<int> bus_id;
    Param<int> clock;
    Param<int> width;

END_DECLARE_SIM_OBJECT_PARAMS(Bus)

BEGIN_INIT_SIM_OBJECT_PARAMS(Bus)
    INIT_PARAM(bus_id, "a globally unique bus id"),
    INIT_PARAM(clock, "bus clock speed"),
    INIT_PARAM(width, "width of the bus (bits)")
END_INIT_SIM_OBJECT_PARAMS(Bus)

CREATE_SIM_OBJECT(Bus)
{
    return new Bus(getInstanceName(), bus_id, clock, width);
}

REGISTER_SIM_OBJECT("Bus", Bus)
