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
 *          William Wang
 */

/**
 * @file
 * Declaration of an abstract bus base class.
 */

#ifndef __MEM_BUS_HH__
#define __MEM_BUS_HH__

#include <list>
#include <set>

#include "base/range.hh"
#include "base/range_map.hh"
#include "base/types.hh"
#include "mem/mem_object.hh"
#include "params/BaseBus.hh"

/**
 * The base bus contains the common elements of the non-coherent and
 * coherent bus. It is an abstract class that does not have any of the
 * functionality relating to the actual reception and transmission of
 * packets, as this is left for the subclasses.
 *
 * The BaseBus is responsible for the basic flow control (busy or
 * not), the administration of retries, and the address decoding.
 */
class BaseBus : public MemObject
{

  protected:

    /**
     * We declare an enum to track the state of the bus. The starting
     * point is an idle state where the bus is waiting for a packet to
     * arrive. Upon arrival, the bus transitions to the busy state,
     * where it remains either until the packet transfer is done, or
     * the header time is spent. Once the bus leaves the busy state,
     * it can either go back to idle, if no packets have arrived while
     * it was busy, or the bus goes on to retry the first port on the
     * retryList. A similar transition takes place from idle to retry
     * if the bus receives a retry from one of its connected
     * ports. The retry state lasts until the port in questions calls
     * sendTiming and returns control to the bus, or goes to a busy
     * state if the port does not immediately react to the retry by
     * calling sendTiming.
     */
    enum State { IDLE, BUSY, RETRY };

    /** track the state of the bus */
    State state;

    /** the clock speed for the bus */
    int clock;
    /** cycles of overhead per transaction */
    int headerCycles;
    /** the width of the bus in bytes */
    int width;

    Event * drainEvent;

    typedef range_map<Addr, PortID>::iterator PortMapIter;
    typedef range_map<Addr, PortID>::const_iterator PortMapConstIter;
    range_map<Addr, PortID> portMap;

    AddrRangeList defaultRange;

    /**
     * Determine if the bus accepts a packet from a specific port. If
     * not, the port in question is also added to the retry list. In
     * either case the state of the bus is updated accordingly.
     *
     * @param port Source port on the bus presenting the packet
     *
     * @return True if the bus accepts the packet
     */
    bool tryTiming(Port* port);

    /**
     * Deal with a destination port accepting a packet by potentially
     * removing the source port from the retry list (if retrying) and
     * occupying the bus accordingly.
     *
     * @param busy_time Time to spend as a result of a successful send
     */
    void succeededTiming(Tick busy_time);

    /**
     * Deal with a destination port not accepting a packet by
     * potentially adding the source port to the retry list (if
     * not already at the front) and occupying the bus accordingly.
     *
     * @param busy_time Time to spend as a result of a failed send
     */
    void failedTiming(SlavePort* port, Tick busy_time);

    /** Timing function called by port when it is once again able to process
     * requests. */
    void recvRetry();

    /**
     * Function called by the port when the bus is recieving a range change.
     *
     * @param master_port_id id of the port that received the change
     */
    void recvRangeChange(PortID master_port_id);

    /** Find which port connected to this bus (if any) should be given a packet
     * with this address.
     * @param addr Address to find port for.
     * @return id of port that the packet should be sent out of.
     */
    PortID findPort(Addr addr);

    // Cache for the findPort function storing recently used ports from portMap
    struct PortCache {
        bool valid;
        PortID id;
        Addr start;
        Addr end;
    };

    PortCache portCache[3];

    // Checks the cache and returns the id of the port that has the requested
    // address within its range
    inline PortID checkPortCache(Addr addr) {
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

        return InvalidPortID;
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
     * Return the address ranges the bus is responsible for.
     *
     * @return a list of non-overlapping address ranges
     */
    AddrRangeList getAddrRanges() const;

    /** Calculate the timing parameters for the packet.  Updates the
     * firstWordTime and finishTime fields of the packet object.
     * Returns the tick at which the packet header is completed (which
     * will be all that is sent if the target rejects the packet).
     */
    Tick calcPacketTiming(PacketPtr pkt);

    /** Occupy the bus until until */
    void occupyBus(Tick until);

    /**
     * Release the bus after being occupied and return to an idle
     * state where we proceed to send a retry to any potential waiting
     * port, or drain if asked to do so.
     */
    void releaseBus();

    /**
     * Send a retry to the port at the head of the retryList. The
     * caller must ensure that the list is not empty.
     */
    void retryWaiting();

    /**
     * Ask everyone on the bus what their size is
     *
     * @return the max of all the sizes
     */
    unsigned findBlockSize();

    // event used to schedule a release of the bus
    EventWrapper<BaseBus, &BaseBus::releaseBus> busIdleEvent;

    std::set<PortID> inRecvRangeChange;

    /** The master and slave ports of the bus */
    std::vector<SlavePort*> slavePorts;
    std::vector<MasterPort*> masterPorts;

    /** Convenience typedefs. */
    typedef std::vector<SlavePort*>::iterator SlavePortIter;
    typedef std::vector<MasterPort*>::iterator MasterPortIter;
    typedef std::vector<SlavePort*>::const_iterator SlavePortConstIter;
    typedef std::vector<MasterPort*>::const_iterator MasterPortConstIter;

    /** An array of pointers to ports that retry should be called on because the
     * original send failed for whatever reason.*/
    std::list<Port*> retryList;

    /** Port that handles requests that don't match any of the interfaces.*/
    PortID defaultPortID;

    /** If true, use address range provided by default device.  Any
       address not handled by another port and not in default device's
       range will cause a fatal error.  If false, just send all
       addresses not handled by another port to default device. */
    bool useDefaultRange;

    unsigned defaultBlockSize;
    unsigned cachedBlockSize;
    bool cachedBlockSizeValid;

    BaseBus(const BaseBusParams *p);

    virtual ~BaseBus();

  public:

    /** A function used to return the port associated with this bus object. */
    virtual MasterPort& getMasterPort(const std::string& if_name, int idx = -1);
    virtual SlavePort& getSlavePort(const std::string& if_name, int idx = -1);

    unsigned int drain(Event *de);

};

#endif //__MEM_BUS_HH__
