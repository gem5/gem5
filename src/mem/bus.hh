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
 *
 * Authors: Ron Dreslinski
 *          Ali Saidi
 */

/**
 * @file
 * Declaration of a bus object.
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
    /** a globally unique id for this bus. */
    int busId;

    static const int defaultId = -1;

    struct DevMap {
        int portId;
        Range<Addr> range;
    };
    std::vector<DevMap> portList;
    AddrRangeList defaultRange;
    std::vector<DevMap> portSnoopList;

    std::vector<int> snoopCallbacks;


    /** Function called by the port when the bus is recieving a Timing
      transaction.*/
    bool recvTiming(Packet *pkt);

    /** Function called by the port when the bus is recieving a Atomic
      transaction.*/
    Tick recvAtomic(Packet *pkt);

    /** Function called by the port when the bus is recieving a Functional
        transaction.*/
    void recvFunctional(Packet *pkt);

    /** Timing function called by port when it is once again able to process
     * requests. */
    void recvRetry(int id);

    /** Function called by the port when the bus is recieving a status change.*/
    void recvStatusChange(Port::Status status, int id);

    /** Find which port connected to this bus (if any) should be given a packet
     * with this address.
     * @param addr Address to find port for.
     * @param id Id of the port this packet was received from (to prevent
     *             loops)
     * @return pointer to port that the packet should be sent out of.
     */
    Port *findPort(Addr addr, int id);

    /** Find all ports with a matching snoop range, except src port.  Keep in mind
     * that the ranges shouldn't overlap or you will get a double snoop to the same
     * interface.and the cache will assert out.
     * @param addr Address to find snoop prts for.
     * @param id Id of the src port of the request to avoid calling snoop on src
     * @return vector of IDs to snoop on
     */
    std::vector<int> findSnoopPorts(Addr addr, int id);

    /** Snoop all relevant ports atomicly. */
    void atomicSnoop(Packet *pkt);

    /** Snoop for NACK and Blocked in phase 1
     * @return True if succeds.
     */
    bool timingSnoopPhase1(Packet *pkt);

    /** @todo Don't need to commit all snoops just those that need it
     *(register somehow). */
    /** Commit all snoops now that we know if any of them would have blocked.
     */
    void timingSnoopPhase2(Packet *pkt);

    /** Process address range request.
     * @param resp addresses that we can respond to
     * @param snoop addresses that we would like to snoop
     * @param id ide of the busport that made the request.
     */
    void addressRanges(AddrRangeList &resp, AddrRangeList &snoop, int id);


    /** Declaration of the buses port type, one will be instantiated for each
        of the interfaces connecting to the bus. */
    class BusPort : public Port
    {
        /** A pointer to the bus to which this port belongs. */
        Bus *bus;

        /** A id to keep track of the intercafe ID this port is connected to. */
        int id;

      public:

        /** Constructor for the BusPort.*/
        BusPort(const std::string &_name, Bus *_bus, int _id)
            : Port(_name), bus(_bus), id(_id)
        { }

      protected:

        /** When reciving a timing request from the peer port (at id),
            pass it to the bus. */
        virtual bool recvTiming(Packet *pkt)
        { pkt->setSrc(id); return bus->recvTiming(pkt); }

        /** When reciving a Atomic requestfrom the peer port (at id),
            pass it to the bus. */
        virtual Tick recvAtomic(Packet *pkt)
        { pkt->setSrc(id); return bus->recvAtomic(pkt); }

        /** When reciving a Functional requestfrom the peer port (at id),
            pass it to the bus. */
        virtual void recvFunctional(Packet *pkt)
        { pkt->setSrc(id); bus->recvFunctional(pkt); }

        /** When reciving a status changefrom the peer port (at id),
            pass it to the bus. */
        virtual void recvStatusChange(Status status)
        { bus->recvStatusChange(status, id); }

        /** When reciving a retry from the peer port (at id),
            pass it to the bus. */
        virtual void recvRetry()
        { bus->recvRetry(id); }

        // This should return all the 'owned' addresses that are
        // downstream from this bus, yes?  That is, the union of all
        // the 'owned' address ranges of all the other interfaces on
        // this bus...
        virtual void getDeviceAddressRanges(AddrRangeList &resp,
                                            AddrRangeList &snoop)
        { bus->addressRanges(resp, snoop, id); }

        // Hack to make translating port work without changes
        virtual int deviceBlockSize() { return 32; }

    };

    /** An array of pointers to the peer port interfaces
        connected to this bus.*/
    std::vector<Port*> interfaces;

    /** An array of pointers to ports that retry should be called on because the
     * original send failed for whatever reason.*/
    std::list<Port*> retryList;

    /** Port that handles requests that don't match any of the interfaces.*/
    Port *defaultPort;

  public:

    /** A function used to return the port associated with this bus object. */
    virtual Port *getPort(const std::string &if_name, int idx = -1);

    virtual void init();

    Bus(const std::string &n, int bus_id)
        : MemObject(n), busId(bus_id), defaultPort(NULL)  {}

};

#endif //__MEM_BUS_HH__
