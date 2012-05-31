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
 * Declaration of a coherent bus.
 */

#ifndef __MEM_COHERENT_BUS_HH__
#define __MEM_COHERENT_BUS_HH__

#include "mem/bus.hh"
#include "params/CoherentBus.hh"

/**
 * A coherent bus connects a number of (potentially) snooping masters
 * and slaves, and routes the request and response packets based on
 * the address, and also forwards all requests to the snoopers and
 * deals with the snoop responses.
 *
 * The coherent bus can be used as a template for modelling QPI,
* HyperTransport, ACE and coherent OCP buses, and is typically used
 * for the L1-to-L2 buses and as the main system interconnect.
 */
class CoherentBus : public BaseBus
{

  protected:

    /**
     * Declaration of the coherent bus slave port type, one will be
     * instantiated for each of the master ports connecting to the
     * bus.
     */
    class CoherentBusSlavePort : public SlavePort
    {

      private:

        /** A reference to the bus to which this port belongs. */
        CoherentBus &bus;

      public:

        CoherentBusSlavePort(const std::string &_name,
                             CoherentBus &_bus, PortID _id)
            : SlavePort(_name, &_bus, _id), bus(_bus)
        { }

      protected:

        /**
         * When receiving a timing request, pass it to the bus.
         */
        virtual bool recvTimingReq(PacketPtr pkt)
        { return bus.recvTimingReq(pkt, id); }

        /**
         * When receiving a timing snoop response, pass it to the bus.
         */
        virtual bool recvTimingSnoopResp(PacketPtr pkt)
        { return bus.recvTimingSnoopResp(pkt, id); }

        /**
         * When receiving an atomic request, pass it to the bus.
         */
        virtual Tick recvAtomic(PacketPtr pkt)
        { return bus.recvAtomic(pkt, id); }

        /**
         * When receiving a functional request, pass it to the bus.
         */
        virtual void recvFunctional(PacketPtr pkt)
        { bus.recvFunctional(pkt, id); }

        /**
         * When receiving a retry, pass it to the bus.
         */
        virtual void recvRetry()
        { panic("Bus slave ports always succeed and should never retry.\n"); }

        /**
         * Return the union of all adress ranges seen by this bus.
         */
        virtual AddrRangeList getAddrRanges()
        { return bus.getAddrRanges(); }

        /**
         * Get the maximum block size as seen by the bus.
         */
        virtual unsigned deviceBlockSize() const
        { return bus.findBlockSize(); }

    };

    /**
     * Declaration of the coherent bus master port type, one will be
     * instantiated for each of the slave interfaces connecting to the
     * bus.
     */
    class CoherentBusMasterPort : public MasterPort
    {
      private:
        /** A reference to the bus to which this port belongs. */
        CoherentBus &bus;

      public:

        CoherentBusMasterPort(const std::string &_name,
                              CoherentBus &_bus, PortID _id)
            : MasterPort(_name, &_bus, _id), bus(_bus)
        { }

      protected:

        /**
         * Determine if this port should be considered a snooper. For
         * a coherent bus master port this is always true.
         *
         * @return a boolean that is true if this port is snooping
         */
        virtual bool isSnooping() const
        { return true; }

        /**
         * When receiving a timing response, pass it to the bus.
         */
        virtual bool recvTimingResp(PacketPtr pkt)
        { return bus.recvTimingResp(pkt, id); }

        /**
         * When receiving a timing snoop request, pass it to the bus.
         */
        virtual void recvTimingSnoopReq(PacketPtr pkt)
        { return bus.recvTimingSnoopReq(pkt, id); }

        /**
         * When receiving an atomic snoop request, pass it to the bus.
         */
        virtual Tick recvAtomicSnoop(PacketPtr pkt)
        { return bus.recvAtomicSnoop(pkt, id); }

        /**
         * When receiving a functional snoop request, pass it to the bus.
         */
        virtual void recvFunctionalSnoop(PacketPtr pkt)
        { bus.recvFunctionalSnoop(pkt, id); }

        /** When reciving a range change from the peer port (at id),
            pass it to the bus. */
        virtual void recvRangeChange()
        { bus.recvRangeChange(id); }

        /** When reciving a retry from the peer port (at id),
            pass it to the bus. */
        virtual void recvRetry()
        { bus.recvRetry(); }

        // Ask the bus to ask everyone on the bus what their block size is and
        // take the max of it. This might need to be changed a bit if we ever
        // support multiple block sizes.
        virtual unsigned deviceBlockSize() const
        { return bus.findBlockSize(); }

    };

    std::vector<SlavePort*> snoopPorts;

    /**
     * Store the outstanding requests so we can determine which ones
     * we generated and which ones were merely forwarded. This is used
     * in the coherent bus when coherency responses come back.
     */
    std::set<RequestPtr> outstandingReq;

    /** Function called by the port when the bus is recieving a Timing
      request packet.*/
    bool recvTimingReq(PacketPtr pkt, PortID slave_port_id);

    /** Function called by the port when the bus is recieving a Timing
      response packet.*/
    bool recvTimingResp(PacketPtr pkt, PortID master_port_id);

    /** Function called by the port when the bus is recieving a timing
        snoop request.*/
    void recvTimingSnoopReq(PacketPtr pkt, PortID master_port_id);

    /** Function called by the port when the bus is recieving a timing
        snoop response.*/
    bool recvTimingSnoopResp(PacketPtr pkt, PortID slave_port_id);

    /**
     * Forward a timing packet to our snoopers, potentially excluding
     * one of the connected coherent masters to avoid sending a packet
     * back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     */
    void forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id);

    /** Function called by the port when the bus is recieving a Atomic
      transaction.*/
    Tick recvAtomic(PacketPtr pkt, PortID slave_port_id);

    /** Function called by the port when the bus is recieving an
        atomic snoop transaction.*/
    Tick recvAtomicSnoop(PacketPtr pkt, PortID master_port_id);

    /**
     * Forward an atomic packet to our snoopers, potentially excluding
     * one of the connected coherent masters to avoid sending a packet
     * back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     *
     * @return a pair containing the snoop response and snoop latency
     */
    std::pair<MemCmd, Tick> forwardAtomic(PacketPtr pkt,
                                          PortID exclude_slave_port_id);

    /** Function called by the port when the bus is recieving a Functional
        transaction.*/
    void recvFunctional(PacketPtr pkt, PortID slave_port_id);

    /** Function called by the port when the bus is recieving a functional
        snoop transaction.*/
    void recvFunctionalSnoop(PacketPtr pkt, PortID master_port_id);

    /**
     * Forward a functional packet to our snoopers, potentially
     * excluding one of the connected coherent masters to avoid
     * sending a packet back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     */
    void forwardFunctional(PacketPtr pkt, PortID exclude_slave_port_id);

  public:

    virtual void init();

    CoherentBus(const CoherentBusParams *p);
};

#endif //__MEM_COHERENT_BUS_HH__
