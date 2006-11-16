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
    if (if_name == "default") {
        if (defaultPort == NULL) {
            defaultPort = new BusPort(csprintf("%s-default",name()), this,
                                      defaultId);
            return defaultPort;
        } else
            fatal("Default port already set\n");
    }

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
    std::vector<BusPort*>::iterator intIter;

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

void Bus::occupyBus(PacketPtr pkt)
{
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
}

/** Function called by the port when the bus is receiving a Timing
 * transaction.*/
bool
Bus::recvTiming(PacketPtr pkt)
{
    Port *port;
    DPRINTF(Bus, "recvTiming: packet src %d dest %d addr 0x%x cmd %s\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString());

    BusPort *pktPort;
    if (pkt->getSrc() == defaultId)
        pktPort = defaultPort;
    else pktPort = interfaces[pkt->getSrc()];

    // If the bus is busy, or other devices are in line ahead of the current
    // one, put this device on the retry list.
    if (tickNextIdle > curTick ||
            (retryList.size() && (!inRetry || pktPort != retryList.front()))) {
        addToRetryList(pktPort);
        return false;
    }

    short dest = pkt->getDest();
    if (dest == Packet::Broadcast) {
        port = findPort(pkt->getAddr(), pkt->getSrc());
        if (timingSnoop(pkt, port ? port : interfaces[pkt->getSrc()])) {
            bool success;

            pkt->flags |= SNOOP_COMMIT;
            success = timingSnoop(pkt, port ? port : interfaces[pkt->getSrc()]);
            assert(success);

            if (pkt->flags & SATISFIED) {
                //Cache-Cache transfer occuring
                if (inRetry) {
                    retryList.front()->onRetryList(false);
                    retryList.pop_front();
                    inRetry = false;
                }
                occupyBus(pkt);
                return true;
            }
        } else {
            //Snoop didn't succeed
            DPRINTF(Bus, "Adding a retry to RETRY list %i\n", pktPort);
            addToRetryList(pktPort);
            return false;
        }
    } else {
        assert(dest >= 0 && dest < interfaces.size());
        assert(dest != pkt->getSrc()); // catch infinite loops
        port = interfaces[dest];
    }

    occupyBus(pkt);

    if (port) {
        if (port->sendTiming(pkt))  {
            // Packet was successfully sent. Return true.
            // Also take care of retries
            if (inRetry) {
                DPRINTF(Bus, "Remove retry from list %i\n", retryList.front());
                retryList.front()->onRetryList(false);
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
    else {
        //Forwarding up from responder, just return true;
        return true;
    }
}

void
Bus::recvRetry(int id)
{
    DPRINTF(Bus, "Received a retry\n");
    // If there's anything waiting, and the bus isn't busy...
    if (retryList.size() && curTick >= tickNextIdle) {
        //retryingPort = retryList.front();
        inRetry = true;
        DPRINTF(Bus, "Sending a retry\n");
        retryList.front()->sendRetry();
        // If inRetry is still true, sendTiming wasn't called
        if (inRetry)
        {
            retryList.front()->onRetryList(false);
            retryList.pop_front();
            inRetry = false;

            //Bring tickNextIdle up to the present
            while (tickNextIdle < curTick)
                tickNextIdle += clock;

            //Burn a cycle for the missed grant.
            tickNextIdle += clock;

            if (!busIdle.scheduled()) {
                busIdle.schedule(tickNextIdle);
            } else {
                busIdle.reschedule(tickNextIdle);
            }
        }
    }
    //If we weren't able to drain before, we might be able to now.
    if (drainEvent && retryList.size() == 0 && curTick >= tickNextIdle) {
        drainEvent->process();
        // Clear the drain event once we're done with it.
        drainEvent = NULL;
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

        if (responderSet) {
            panic("Unable to find destination for addr (user set default "
                  "responder): %#llx", addr);
        } else {
            DPRINTF(Bus, "Unable to find destination for addr: %#llx, will use "
                    "default port", addr);

            return defaultPort;
        }
    }


    // we shouldn't be sending this back to where it came from
    // only on a functional access and then we should terminate
    // the cyclical call.
    if (dest_id == id)
        return 0;

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

            //@todo Fix this hack because ranges are overlapping
            //need to make sure we dont't create overlapping ranges
            bool hack_overlap = false;
            int size = ports.size();
            for (int j=0; j < size; j++) {
                if (ports[j] == portSnoopList[i].portId)
                    hack_overlap = true;
            }

            if (!hack_overlap)
                ports.push_back(portSnoopList[i].portId);
//            DPRINTF(Bus, "  found snoop addr %#llx on device%d\n", addr,
//                    portSnoopList[i].portId);
        }
        i++;
    }
    return ports;
}

Tick
Bus::atomicSnoop(PacketPtr pkt, Port *responder)
{
    std::vector<int> ports = findSnoopPorts(pkt->getAddr(), pkt->getSrc());
    Tick response_time = 0;

    while (!ports.empty())
    {
        if (interfaces[ports.back()] != responder) {
            Tick response = interfaces[ports.back()]->sendAtomic(pkt);
            if (response) {
                assert(!response_time);  //Multiple responders
                response_time = response;
            }
        }
        ports.pop_back();
    }
    return response_time;
}

void
Bus::functionalSnoop(PacketPtr pkt, Port *responder)
{
    std::vector<int> ports = findSnoopPorts(pkt->getAddr(), pkt->getSrc());

    //The packet may be changed by another bus on snoops, restore the id after each
    int id = pkt->getSrc();
    while (!ports.empty() && pkt->result != Packet::Success)
    {
        if (interfaces[ports.back()] != responder)
            interfaces[ports.back()]->sendFunctional(pkt);
        ports.pop_back();
        pkt->setSrc(id);
    }
}

bool
Bus::timingSnoop(PacketPtr pkt, Port* responder)
{
    std::vector<int> ports = findSnoopPorts(pkt->getAddr(), pkt->getSrc());
    bool success = true;

    while (!ports.empty() && success)
    {
        if (interfaces[ports.back()] != responder) //Don't call if responder also, once will do
            success = interfaces[ports.back()]->sendTiming(pkt);
        ports.pop_back();
    }

    return success;
}


/** Function called by the port when the bus is receiving a Atomic
 * transaction.*/
Tick
Bus::recvAtomic(PacketPtr pkt)
{
    DPRINTF(Bus, "recvAtomic: packet src %d dest %d addr 0x%x cmd %s\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString());
    assert(pkt->getDest() == Packet::Broadcast);
    pkt->flags |= SNOOP_COMMIT;

    // Assume one bus cycle in order to get through.  This may have
    // some clock skew issues yet again...
    pkt->finishTime = curTick + clock;

    Port *port = findPort(pkt->getAddr(), pkt->getSrc());
    Tick snoopTime = atomicSnoop(pkt, port ? port : interfaces[pkt->getSrc()]);

    if (snoopTime)
        return snoopTime;  //Snoop satisfies it
    else if (port)
        return port->sendAtomic(pkt);
    else
        return 0;
}

/** Function called by the port when the bus is receiving a Functional
 * transaction.*/
void
Bus::recvFunctional(PacketPtr pkt)
{
    DPRINTF(Bus, "recvFunctional: packet src %d dest %d addr 0x%x cmd %s\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString());
    assert(pkt->getDest() == Packet::Broadcast);
    pkt->flags |= SNOOP_COMMIT;

    Port* port = findPort(pkt->getAddr(), pkt->getSrc());
    functionalSnoop(pkt, port ? port : interfaces[pkt->getSrc()]);

    // If the snooping found what we were looking for, we're done.
    if (pkt->result != Packet::Success && port) {
        port->sendFunctional(pkt);
    }
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
        // Only try to update these ranges if the user set a default responder.
        if (responderSet) {
            defaultPort->getPeerAddressRanges(ranges, snoops);
            assert(snoops.size() == 0);
            for(iter = ranges.begin(); iter != ranges.end(); iter++) {
                defaultRange.push_back(*iter);
                DPRINTF(BusAddrRanges, "Adding range %#llx - %#llx for default range\n",
                        iter->start, iter->end);
            }
        }
    } else {

        assert((id < interfaces.size() && id >= 0) || id == defaultId);
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

            //@todo, make sure we don't overlap ranges
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
        DPRINTF(BusAddrRanges, "  -- Dflt: %#llx : %#llx\n",dflt_iter->start,
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

    for (portIter = portSnoopList.begin();
         portIter != portSnoopList.end(); portIter++)
    {
        if (portIter->portId != id) {
            snoop.push_back(portIter->range);
            DPRINTF(BusAddrRanges, "  -- Snoop: %#llx : %#llx\n",
                    portIter->range.start, portIter->range.end);
            //@todo We need to properly insert snoop ranges
            //not overlapping the ranges (multiple)
        }
    }
}

unsigned int
Bus::drain(Event * de)
{
    //We should check that we're not "doing" anything, and that noone is
    //waiting. We might be idle but have someone waiting if the device we
    //contacted for a retry didn't actually retry.
    if (curTick >= tickNextIdle && retryList.size() == 0) {
        return 0;
    } else {
        drainEvent = de;
        return 1;
    }
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Bus)

    Param<int> bus_id;
    Param<int> clock;
    Param<int> width;
    Param<bool> responder_set;

END_DECLARE_SIM_OBJECT_PARAMS(Bus)

BEGIN_INIT_SIM_OBJECT_PARAMS(Bus)
    INIT_PARAM(bus_id, "a globally unique bus id"),
    INIT_PARAM(clock, "bus clock speed"),
    INIT_PARAM(width, "width of the bus (bits)"),
    INIT_PARAM(responder_set, "Is a default responder set by the user")
END_INIT_SIM_OBJECT_PARAMS(Bus)

CREATE_SIM_OBJECT(Bus)
{
    return new Bus(getInstanceName(), bus_id, clock, width, responder_set);
}

REGISTER_SIM_OBJECT("Bus", Bus)
