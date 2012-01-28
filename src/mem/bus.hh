/*
 * Copyright (c) 2011 ARM Limited
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
 *          Andreas Hansson
 */

/**
 * @file
 * Declaration of a bus object.
 */

#ifndef __MEM_BUS_HH__
#define __MEM_BUS_HH__

#include <list>
#include <set>
#include <string>

#include "base/hashmap.hh"
#include "base/range.hh"
#include "base/range_map.hh"
#include "base/types.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "params/Bus.hh"
#include "sim/eventq.hh"

class Bus : public MemObject
{
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

        int getId() const { return id; }

        /**
         * Determine if this port should be considered a snooper. This
         * is determined by the bus.
         *
         * @return a boolean that is true if this port is snooping
         */
        virtual bool isSnooping()
        { return bus->isSnooping(id); }

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

        /** When reciving a range change from the peer port (at id),
            pass it to the bus. */
        virtual void recvRangeChange()
        { bus->recvRangeChange(id); }

        /** When reciving a retry from the peer port (at id),
            pass it to the bus. */
        virtual void recvRetry()
        { bus->recvRetry(id); }

        // This should return all the 'owned' addresses that are
        // downstream from this bus, yes?  That is, the union of all
        // the 'owned' address ranges of all the other interfaces on
        // this bus...
        virtual AddrRangeList getAddrRanges()
        { return bus->getAddrRanges(id); }

        // Ask the bus to ask everyone on the bus what their block size is and
        // take the max of it. This might need to be changed a bit if we ever
        // support multiple block sizes.
        virtual unsigned deviceBlockSize() const
        { return bus->findBlockSize(id); }

    };

    class BusFreeEvent : public Event
    {
        Bus * bus;

      public:
        BusFreeEvent(Bus * _bus);
        void process();
        const char *description() const;
    };

    /** a globally unique id for this bus. */
    int busId;
    /** the clock speed for the bus */
    int clock;
    /** cycles of overhead per transaction */
    int headerCycles;
    /** the width of the bus in bytes */
    int width;
    /** the next tick at which the bus will be idle */
    Tick tickNextIdle;

    Event * drainEvent;

    typedef range_map<Addr,int>::iterator PortIter;
    range_map<Addr, int> portMap;

    AddrRangeList defaultRange;

    typedef std::vector<BusPort*>::iterator SnoopIter;
    std::vector<BusPort*> snoopPorts;

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

    /** Function called by the port when the bus is recieving a range change.*/
    void recvRangeChange(int id);

    /** Find which port connected to this bus (if any) should be given a packet
     * with this address.
     * @param addr Address to find port for.
     * @return id of port that the packet should be sent out of.
     */
    int findPort(Addr addr);

    // Cache for the findPort function storing recently used ports from portMap
    struct PortCache {
        bool valid;
        int  id;
        Addr start;
        Addr end;
    };

    PortCache portCache[3];

    // Checks the cache and returns the id of the port that has the requested
    // address within its range
    inline int checkPortCache(Addr addr) {
        if (portCache[0].valid && addr >= portCache[0].start &&
            addr < portCache[0].end) {
            return portCache[0].id;
        }
        if (portCache[1].valid && addr >= portCache[1].start &&
                   addr < portCache[1].end) {
            return portCache[1].id;
        }
        if (portCache[2].valid && addr >= portCache[2].start &&
            addr < portCache[2].end) {
            return portCache[2].id;
        }

        return -1;
    }

    // Clears the earliest entry of the cache and inserts a new port entry
    inline void updatePortCache(short id, Addr start, Addr end) {
        portCache[2].valid = portCache[1].valid;
        portCache[2].id    = portCache[1].id;
        portCache[2].start = portCache[1].start;
        portCache[2].end   = portCache[1].end;

        portCache[1].valid = portCache[0].valid;
        portCache[1].id    = portCache[0].id;
        portCache[1].start = portCache[0].start;
        portCache[1].end   = portCache[0].end;

        portCache[0].valid = true;
        portCache[0].id    = id;
        portCache[0].start = start;
        portCache[0].end   = end;
    }

    // Clears the cache. Needs to be called in constructor.
    inline void clearPortCache() {
        portCache[2].valid = false;
        portCache[1].valid = false;
        portCache[0].valid = false;
    }

    /**
     * Return the address ranges this port is responsible for.
     *
     * @param id id of the bus port that made the request
     *
     * @return a list of non-overlapping address ranges
     */
    AddrRangeList getAddrRanges(int id);

    /**
     * Determine if the bus port is snooping or not.
     *
     * @param id id of the bus port that made the request
     *
     * @return a boolean indicating if this port is snooping or not
     */
    bool isSnooping(int id);

    /** Calculate the timing parameters for the packet.  Updates the
     * firstWordTime and finishTime fields of the packet object.
     * Returns the tick at which the packet header is completed (which
     * will be all that is sent if the target rejects the packet).
     */
    Tick calcPacketTiming(PacketPtr pkt);

    /** Occupy the bus until until */
    void occupyBus(Tick until);

    /** Ask everyone on the bus what their size is
     * @param id id of the busport that made the request
     * @return the max of all the sizes
     */
    unsigned findBlockSize(int id);

    BusFreeEvent busIdle;

    bool inRetry;
    std::set<int> inRecvRangeChange;

    /** An ordered vector of pointers to the peer port interfaces
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
    short defaultPortId;

    /** If true, use address range provided by default device.  Any
       address not handled by another port and not in default device's
       range will cause a fatal error.  If false, just send all
       addresses not handled by another port to default device. */
    bool useDefaultRange;

    unsigned defaultBlockSize;
    unsigned cachedBlockSize;
    bool cachedBlockSizeValid;

  public:

    /** A function used to return the port associated with this bus object. */
    virtual Port *getPort(const std::string &if_name, int idx = -1);

    virtual void init();
    virtual void startup();

    unsigned int drain(Event *de);

    Bus(const BusParams *p);
};

#endif //__MEM_BUS_HH__
