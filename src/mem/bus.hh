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
#include "base/range_map.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"

class Bus : public MemObject
{
    /** a globally unique id for this bus. */
    int busId;
    /** the clock speed for the bus */
    int clock;
    /** the width of the bus in bytes */
    int width;
    /** the next tick at which the bus will be idle */
    Tick tickNextIdle;

    Event * drainEvent;

    static const int defaultId = -3; //Make it unique from Broadcast

    struct DevMap {
        int portId;
        Range<Addr> range;
    };
    range_map<Addr, int> portMap;
    AddrRangeList defaultRange;
    std::vector<DevMap> portSnoopList;

    /** Function called by the port when the bus is recieving a Timing
      transaction.*/
    bool recvTiming(PacketPtr pkt);

    /** Function called by the port when the bus is recieving a Atomic
      transaction.*/
    Tick recvAtomic(PacketPtr pkt);

    /** Function called by the port when the bus is recieving a Functional
        transaction.*/
    void recvFunctional(PacketPtr pkt);

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
    Tick atomicSnoop(PacketPtr pkt, Port* responder);

    /** Snoop all relevant ports functionally. */
    void functionalSnoop(PacketPtr pkt, Port *responder);

    /** Call snoop on caches, be sure to set SNOOP_COMMIT bit if you want
     * the snoop to happen
     * @return True if succeds.
     */
    bool timingSnoop(PacketPtr pkt, Port *responder);

    /** Process address range request.
     * @param resp addresses that we can respond to
     * @param snoop addresses that we would like to snoop
     * @param id ide of the busport that made the request.
     */
    void addressRanges(AddrRangeList &resp, AddrRangeList &snoop, int id);

    /** Occupy the bus with transmitting the packet pkt */
    void occupyBus(PacketPtr pkt);

    /** Declaration of the buses port type, one will be instantiated for each
        of the interfaces connecting to the bus. */
    class BusPort : public Port
    {
        bool _onRetryList;

        /** A pointer to the bus to which this port belongs. */
        Bus *bus;

        /** A id to keep track of the intercafe ID this port is connected to. */
        int id;

      public:

        /** Constructor for the BusPort.*/
        BusPort(const std::string &_name, Bus *_bus, int _id)
            : Port(_name, _bus), _onRetryList(false), bus(_bus), id(_id)
        { }

        bool onRetryList()
        { return _onRetryList; }

        void onRetryList(bool newVal)
        { _onRetryList = newVal; }

        int getId() { return id; }

      protected:

        /** When reciving a timing request from the peer port (at id),
            pass it to the bus. */
        virtual bool recvTiming(PacketPtr pkt)
        { pkt->setSrc(id); return bus->recvTiming(pkt); }

        /** When reciving a Atomic requestfrom the peer port (at id),
            pass it to the bus. */
        virtual Tick recvAtomic(PacketPtr pkt)
        { pkt->setSrc(id); return bus->recvAtomic(pkt); }

        /** When reciving a Functional requestfrom the peer port (at id),
            pass it to the bus. */
        virtual void recvFunctional(PacketPtr pkt)
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

    class BusFreeEvent : public Event
    {
        Bus * bus;

      public:
        BusFreeEvent(Bus * _bus);
        void process();
        const char *description();
    };

    BusFreeEvent busIdle;

    bool inRetry;

    /** An array of pointers to the peer port interfaces
        connected to this bus.*/
    std::vector<BusPort*> interfaces;

    /** An array of pointers to ports that retry should be called on because the
     * original send failed for whatever reason.*/
    std::list<BusPort*> retryList;

    void addToRetryList(BusPort * port)
    {
        if (!inRetry) {
            // The device wasn't retrying a packet, or wasn't at an appropriate
            // time.
            assert(!port->onRetryList());
            port->onRetryList(true);
            retryList.push_back(port);
        } else {
            if (port->onRetryList()) {
                // The device was retrying a packet. It didn't work, so we'll leave
                // it at the head of the retry list.
                assert(port == retryList.front());
                inRetry = false;
            }
            else {
                port->onRetryList(true);
                retryList.push_back(port);
            }
        }
    }

    /** Port that handles requests that don't match any of the interfaces.*/
    BusPort *defaultPort;

    /** Has the user specified their own default responder? */
    bool responderSet;

  public:

    /** A function used to return the port associated with this bus object. */
    virtual Port *getPort(const std::string &if_name, int idx = -1);

    virtual void init();

    unsigned int drain(Event *de);

    Bus(const std::string &n, int bus_id, int _clock, int _width,
        bool responder_set)
        : MemObject(n), busId(bus_id), clock(_clock), width(_width),
          tickNextIdle(0), drainEvent(NULL), busIdle(this), inRetry(false),
          defaultPort(NULL), responderSet(responder_set)
    {
        //Both the width and clock period must be positive
        if (width <= 0)
            fatal("Bus width must be positive\n");
        if (clock <= 0)
            fatal("Bus clock period must be positive\n");
    }

};

#endif //__MEM_BUS_HH__
