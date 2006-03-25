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
 */

/**
 * @file Definition of a bus object.
 */


#include "bus.hh"
#include "sim/builder.hh"

/** Function called by the port when the bus is recieving a Timing
 * transaction.*/
bool
Bus::recvTiming(Packet &pkt, int id)
{

    panic("I need to be implemented, but not right now.");
}

Port *
Bus::findPort(Addr addr, int id)
{
    /* An interval tree would be a better way to do this. --ali. */
    int dest_id = -1;
    int i = 0;
    bool found = false;

    while (i < portList.size() && !found)
    {
        if (portList[i].range == addr) {
            dest_id = portList[i].portId;
            found = true;
        }
    }
    assert(dest_id != -1 && "Unable to find destination");
    // we shouldn't be sending this back to where it came from
    assert(dest_id != id);

    return interfaces[dest_id];
}

/** Function called by the port when the bus is recieving a Atomic
 * transaction.*/
Tick
Bus::recvAtomic(Packet &pkt, int id)
{
    return findPort(pkt.addr, id)->sendAtomic(pkt);
}

/** Function called by the port when the bus is recieving a Functional
 * transaction.*/
void
Bus::recvFunctional(Packet &pkt, int id)
{
    findPort(pkt.addr, id)->sendFunctional(pkt);
}

/** Function called by the port when the bus is recieving a status change.*/
void
Bus::recvStatusChange(Port::Status status, int id)
{
    assert(status == Port:: RangeChange &&
           "The other statuses need to be implemented.");
    Port *port = interfaces[id];
    AddrRangeList ranges;
    bool owner;

    port->getPeerAddressRanges(ranges, owner);
    // not dealing with snooping yet either
    assert(owner == true);
    // or multiple ranges
    assert(ranges.size() == 1);
    DevMap dm;
    dm.portId = id;
    dm.range = ranges.front();

    portList.push_back(dm);
}

void
Bus::BusPort::addressRanges(AddrRangeList &range_list, bool &owner)
{
    panic("I'm not implemented.\n");
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Bus)

    Param<int> bus_id;

END_DECLARE_SIM_OBJECT_PARAMS(Bus)

BEGIN_INIT_SIM_OBJECT_PARAMS(Bus)
    INIT_PARAM(bus_id, "junk bus id")
END_INIT_SIM_OBJECT_PARAMS(PhysicalMemory)

CREATE_SIM_OBJECT(Bus)
{
    return new Bus(getInstanceName());
}

REGISTER_SIM_OBJECT("Bus", Bus)
