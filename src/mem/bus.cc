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

#include <algorithm>
#include <limits>

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
            cachedBlockSizeValid = false;
            return defaultPort;
        } else
            fatal("Default port already set\n");
    }
    int id;
    if (if_name == "functional") {
        if (!funcPort) {
            id = maxId++;
            funcPort = new BusPort(csprintf("%s-p%d-func", name(), id), this, id);
            funcPortId = id;
            interfaces[id] = funcPort;
        }
        return funcPort;
    }

    // if_name ignored?  forced to be empty?
    id = maxId++;
    assert(maxId < std::numeric_limits<typeof(maxId)>::max());
    BusPort *bp = new BusPort(csprintf("%s-p%d", name(), id), this, id);
    interfaces[id] = bp;
    cachedBlockSizeValid = false;
    return bp;
}

void
Bus::deletePortRefs(Port *p)
{

    BusPort *bp =  dynamic_cast<BusPort*>(p);
    if (bp == NULL)
        panic("Couldn't convert Port* to BusPort*\n");
    // If this is our one functional port
    if (funcPort == bp)
        return;
    interfaces.erase(bp->getId());
    delete bp;
}

/** Get the ranges of anyone other buses that we are connected to. */
void
Bus::init()
{
    m5::hash_map<short,BusPort*>::iterator intIter;

    for (intIter = interfaces.begin(); intIter != interfaces.end(); intIter++)
        intIter->second->sendStatusChange(Port::RangeChange);
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
    DPRINTF(Bus, "recvTiming: packet src %d dest %d addr 0x%x cmd %s result %d\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString(),
            pkt->result);

    BusPort *pktPort;
    if (pkt->getSrc() == defaultId)
        pktPort = defaultPort;
    else pktPort = interfaces[pkt->getSrc()];

    // If the bus is busy, or other devices are in line ahead of the current
    // one, put this device on the retry list.
    if (tickNextIdle > curTick ||
        (retryList.size() && (!inRetry || pktPort != retryList.front())))
    {
        addToRetryList(pktPort);
        DPRINTF(Bus, "recvTiming: Bus is busy, returning false\n");
        return false;
    }

    short dest = pkt->getDest();

    // Make sure to clear the snoop commit flag so it doesn't think an
    // access has been handled twice.
    if (dest == Packet::Broadcast) {
        port = findPort(pkt->getAddr(), pkt->getSrc());
        timingSnoop(pkt, port ? port : interfaces[pkt->getSrc()]);

        if (pkt->memInhibitAsserted()) {
            //Cache-Cache transfer occuring
            if (inRetry) {
                retryList.front()->onRetryList(false);
                retryList.pop_front();
                inRetry = false;
            }
            occupyBus(pkt);
            DPRINTF(Bus, "recvTiming: Packet sucessfully sent\n");
            return true;
        }
    } else {
        assert(dest >= 0 && dest < maxId);
        assert(dest != pkt->getSrc()); // catch infinite loops
        port = interfaces[dest];
    }

    occupyBus(pkt);

    if (port) {
        if (port->sendTiming(pkt))  {
            // Packet was successfully sent. Return true.
            // Also take care of retries
            if (inRetry) {
                DPRINTF(Bus, "Remove retry from list %d\n",
                        retryList.front()->getId());
                retryList.front()->onRetryList(false);
                retryList.pop_front();
                inRetry = false;
            }
            return true;
        }

        // Packet not successfully sent. Leave or put it on the retry list.
        DPRINTF(Bus, "Adding2 a retry to RETRY list %d\n",
                pktPort->getId());
        addToRetryList(pktPort);
        return false;
    }
    else {
        //Forwarding up from responder, just return true;
        DPRINTF(Bus, "recvTiming: can we be here?\n");
        return true;
    }
}

void
Bus::recvRetry(int id)
{
    DPRINTF(Bus, "Received a retry from %s\n", id == -1 ? "self" : interfaces[id]->getPeer()->name());
    // If there's anything waiting, and the bus isn't busy...
    if (retryList.size() && curTick >= tickNextIdle) {
        //retryingPort = retryList.front();
        inRetry = true;
        DPRINTF(Bus, "Sending a retry to %s\n", retryList.front()->getPeer()->name());
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

            busIdle.reschedule(tickNextIdle, true);
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

    PortIter i = portMap.find(RangeSize(addr,1));
    if (i != portMap.end())
        dest_id = i->second;

    // Check if this matches the default range
    if (dest_id == -1) {
        for (AddrRangeIter iter = defaultRange.begin();
             iter != defaultRange.end(); iter++) {
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
    // do the snoop access and then we should terminate
    // the cyclical call.
    if (dest_id == id)
        return 0;

    return interfaces[dest_id];
}

void
Bus::functionalSnoop(PacketPtr pkt, Port *responder)
{
    // The packet may be changed by another bus on snoops, restore the
    // id after each
    int src_id = pkt->getSrc();

    for (SnoopIter s_iter = snoopPorts.begin();
         s_iter != snoopPorts.end();
         s_iter++) {
        BusPort *p = *s_iter;
        if (p != responder && p->getId() != src_id) {
            p->sendFunctional(pkt);
        }
        if (pkt->result == Packet::Success) {
            break;
        }
        pkt->setSrc(src_id);
    }
}

bool
Bus::timingSnoop(PacketPtr pkt, Port* responder)
{
    for (SnoopIter s_iter = snoopPorts.begin();
         s_iter != snoopPorts.end();
         s_iter++) {
        BusPort *p = *s_iter;
        if (p != responder && p->getId() != pkt->getSrc()) {
            bool success = p->sendTiming(pkt);
            if (!success)
                return false;
        }
    }

    return true;
}


/** Function called by the port when the bus is receiving a Atomic
 * transaction.*/
Tick
Bus::recvAtomic(PacketPtr pkt)
{
    DPRINTF(Bus, "recvAtomic: packet src %d dest %d addr 0x%x cmd %s\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString());
    assert(pkt->getDest() == Packet::Broadcast);

    // Variables for recording original command and snoop response (if
    // any)... if a snooper respondes, we will need to restore
    // original command so that additional snoops can take place
    // properly
    MemCmd orig_cmd = pkt->cmd;
    Packet::Result response_result = Packet::Unknown;
    MemCmd response_cmd = MemCmd::InvalidCmd;

    Port *target_port = findPort(pkt->getAddr(), pkt->getSrc());

    SnoopIter s_end = snoopPorts.end();
    for (SnoopIter s_iter = snoopPorts.begin(); s_iter != s_end; s_iter++) {
        BusPort *p = *s_iter;
        // same port should not have both target addresses and snooping
        assert(p != target_port);
        if (p->getId() != pkt->getSrc()) {
            p->sendAtomic(pkt);
            if (pkt->result != Packet::Unknown) {
                // response from snoop agent
                assert(pkt->cmd != orig_cmd);
                assert(pkt->memInhibitAsserted());
                assert(pkt->isResponse());
                // should only happen once
                assert(response_result == Packet::Unknown);
                assert(response_cmd == MemCmd::InvalidCmd);
                // save response state
                response_result = pkt->result;
                response_cmd = pkt->cmd;
                // restore original packet state for remaining snoopers
                pkt->cmd = orig_cmd;
                pkt->result = Packet::Unknown;
            }
        }
    }

    Tick response_time = target_port->sendAtomic(pkt);

    // if we got a response from a snooper, restore it here
    if (response_result != Packet::Unknown) {
        assert(response_cmd != MemCmd::InvalidCmd);
        // no one else should have responded
        assert(pkt->result == Packet::Unknown);
        assert(pkt->cmd == orig_cmd);
        pkt->cmd = response_cmd;
        pkt->result = response_result;
    }

    // why do we have this packet field and the return value both???
    pkt->finishTime = std::max(response_time, curTick + clock);
    return pkt->finishTime;
}

/** Function called by the port when the bus is receiving a Functional
 * transaction.*/
void
Bus::recvFunctional(PacketPtr pkt)
{
    DPRINTF(Bus, "recvFunctional: packet src %d dest %d addr 0x%x cmd %s\n",
            pkt->getSrc(), pkt->getDest(), pkt->getAddr(), pkt->cmdString());
    assert(pkt->getDest() == Packet::Broadcast);

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
    bool snoops;
    AddrRangeIter iter;

    assert(status == Port::RangeChange &&
           "The other statuses need to be implemented.");

    DPRINTF(BusAddrRanges, "received RangeChange from device id %d\n", id);

    if (id == defaultId) {
        defaultRange.clear();
        // Only try to update these ranges if the user set a default responder.
        if (responderSet) {
            defaultPort->getPeerAddressRanges(ranges, snoops);
            assert(snoops == false);
            for(iter = ranges.begin(); iter != ranges.end(); iter++) {
                defaultRange.push_back(*iter);
                DPRINTF(BusAddrRanges, "Adding range %#llx - %#llx for default range\n",
                        iter->start, iter->end);
            }
        }
    } else {

        assert((id < maxId && id >= 0) || id == defaultId);
        BusPort *port = interfaces[id];

        // Clean out any previously existent ids
        for (PortIter portIter = portMap.begin();
             portIter != portMap.end(); ) {
            if (portIter->second == id)
                portMap.erase(portIter++);
            else
                portIter++;
        }

        for (SnoopIter s_iter = snoopPorts.begin();
             s_iter != snoopPorts.end(); ) {
            if ((*s_iter)->getId() == id)
                s_iter = snoopPorts.erase(s_iter);
            else
                s_iter++;
        }

        port->getPeerAddressRanges(ranges, snoops);

        if (snoops) {
            DPRINTF(BusAddrRanges, "Adding id %d to snoop list\n", id);
            snoopPorts.push_back(port);
        }

        for (iter = ranges.begin(); iter != ranges.end(); iter++) {
            DPRINTF(BusAddrRanges, "Adding range %#llx - %#llx for id %d\n",
                    iter->start, iter->end, id);
            if (portMap.insert(*iter, id) == portMap.end())
                panic("Two devices with same range\n");

        }
    }
    DPRINTF(MMU, "port list has %d entries\n", portMap.size());

    // tell all our peers that our address range has changed.
    // Don't tell the device that caused this change, it already knows
    m5::hash_map<short,BusPort*>::iterator intIter;

    for (intIter = interfaces.begin(); intIter != interfaces.end(); intIter++)
        if (intIter->first != id && intIter->first != funcPortId)
            intIter->second->sendStatusChange(Port::RangeChange);

    if (id != defaultId && defaultPort)
        defaultPort->sendStatusChange(Port::RangeChange);
}

void
Bus::addressRanges(AddrRangeList &resp, bool &snoop, int id)
{
    resp.clear();
    snoop = false;

    DPRINTF(BusAddrRanges, "received address range request, returning:\n");

    for (AddrRangeIter dflt_iter = defaultRange.begin();
         dflt_iter != defaultRange.end(); dflt_iter++) {
        resp.push_back(*dflt_iter);
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
            resp.push_back(portIter->first);
            DPRINTF(BusAddrRanges, "  -- %#llx : %#llx\n",
                    portIter->first.start, portIter->first.end);
        }
    }

    for (SnoopIter s_iter = snoopPorts.begin(); s_iter != snoopPorts.end();
         s_iter++) {
        if ((*s_iter)->getId() != id) {
            snoop = true;
            break;
        }
    }
}

int
Bus::findBlockSize(int id)
{
    if (cachedBlockSizeValid)
        return cachedBlockSize;

    int max_bs = -1;

    for (PortIter portIter = portMap.begin();
         portIter != portMap.end(); portIter++) {
        int tmp_bs = interfaces[portIter->second]->peerBlockSize();
        if (tmp_bs > max_bs)
            max_bs = tmp_bs;
    }
    for (SnoopIter s_iter = snoopPorts.begin();
         s_iter != snoopPorts.end(); s_iter++) {
        int tmp_bs = (*s_iter)->peerBlockSize();
        if (tmp_bs > max_bs)
            max_bs = tmp_bs;
    }
    if (max_bs <= 0)
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
    if (curTick >= tickNextIdle && retryList.size() == 0) {
        return 0;
    } else {
        drainEvent = de;
        return 1;
    }
}

void
Bus::startup()
{
    if (tickNextIdle < curTick)
        tickNextIdle = (curTick / clock) * clock + clock;
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Bus)

    Param<int> bus_id;
    Param<int> clock;
    Param<int> width;
    Param<bool> responder_set;
    Param<int> block_size;

END_DECLARE_SIM_OBJECT_PARAMS(Bus)

BEGIN_INIT_SIM_OBJECT_PARAMS(Bus)
    INIT_PARAM(bus_id, "a globally unique bus id"),
    INIT_PARAM(clock, "bus clock speed"),
    INIT_PARAM(width, "width of the bus (bits)"),
    INIT_PARAM(responder_set, "Is a default responder set by the user"),
    INIT_PARAM(block_size, "Default blocksize if no device has one")
END_INIT_SIM_OBJECT_PARAMS(Bus)

CREATE_SIM_OBJECT(Bus)
{
    return new Bus(getInstanceName(), bus_id, clock, width, responder_set,
            block_size);
}

REGISTER_SIM_OBJECT("Bus", Bus)
