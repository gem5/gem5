/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * @file Decleration of a bus object.
 */

#ifndef __MEM_BUS_HH__
#define __MEM_BUS_HH__

#include <string>
#include <list>
#include <inttypes.h>

#include "base/range.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"

class Bus : public MemObject
{
    struct DevMap {
        int portId;
        Range<Addr> range;
    };
    std::vector<DevMap> portList;


    /** Function called by the port when the bus is recieving a Timing
        transaction.*/
    bool recvTiming(Packet &pkt, int id);

    /** Function called by the port when the bus is recieving a Atomic
        transaction.*/
    Tick recvAtomic(Packet &pkt, int id);

    /** Function called by the port when the bus is recieving a Functional
        transaction.*/
    void recvFunctional(Packet &pkt, int id);

    /** Function called by the port when the bus is recieving a status change.*/
    void recvStatusChange(Port::Status status, int id);

    /** Find which port connected to this bus (if any) should be given a packet
     * with this address.
     * @param addr Address to find port for.
     * @param id Id of the port this packet was received from (to prevent
     *             loops)
     * @return pointer to port that the packet should be sent out of.
     */
    Port *
    Bus::findPort(Addr addr, int id);

    /** Decleration of the buses port type, one will be instantiated for each
        of the interfaces connecting to the bus. */
    class BusPort : public Port
    {
        /** A pointer to the bus to which this port belongs. */
        Bus *bus;

        /** A id to keep track of the intercafe ID this port is connected to. */
        int id;

      public:

        /** Constructor for the BusPort.*/
        BusPort(Bus *_bus, int _id)
            : bus(_bus), id(_id)
        { }

      protected:

        /** When reciving a timing request from the peer port (at id),
            pass it to the bus. */
        virtual bool recvTiming(Packet &pkt)
        { return bus->recvTiming(pkt, id); }

        /** When reciving a Atomic requestfrom the peer port (at id),
            pass it to the bus. */
        virtual Tick recvAtomic(Packet &pkt)
        { return bus->recvAtomic(pkt, id); }

        /** When reciving a Functional requestfrom the peer port (at id),
            pass it to the bus. */
        virtual void recvFunctional(Packet &pkt)
        { bus->recvFunctional(pkt, id); }

        /** When reciving a status changefrom the peer port (at id),
            pass it to the bus. */
        virtual void recvStatusChange(Status status)
        { bus->recvStatusChange(status, id); }

        // This should return all the 'owned' addresses that are
        // downstream from this bus, yes?  That is, the union of all
        // the 'owned' address ranges of all the other interfaces on
        // this bus...
        virtual void addressRanges(AddrRangeList &resp, AddrRangeList &snoop);

        // Hack to make translating port work without changes
        virtual int deviceBlockSize() { return 32; }

    };

    /** An array of pointers to the peer port interfaces
        connected to this bus.*/
    std::vector<Port*> interfaces;

  public:

    /** A function used to return the port associated with this bus object. */
    virtual Port *getPort(const std::string &if_name)
    {
        // if_name ignored?  forced to be empty?
        int id = interfaces.size();
        interfaces.push_back(new BusPort(this, id));
        return interfaces.back();
    }
    Bus(const std::string &n)
        : MemObject(n)  {}

};

#endif //__MEM_BUS_HH__
