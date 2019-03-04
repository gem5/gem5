/*
 * Copyright (c) 2011-2015, 2017, 2019 ARM Limited
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
 * Declaration of a coherent crossbar.
 */

#ifndef __MEM_COHERENT_XBAR_HH__
#define __MEM_COHERENT_XBAR_HH__

#include <unordered_map>
#include <unordered_set>

#include "mem/snoop_filter.hh"
#include "mem/xbar.hh"
#include "params/CoherentXBar.hh"

/**
 * A coherent crossbar connects a number of (potentially) snooping
 * masters and slaves, and routes the request and response packets
 * based on the address, and also forwards all requests to the
 * snoopers and deals with the snoop responses.
 *
 * The coherent crossbar can be used as a template for modelling QPI,
 * HyperTransport, ACE and coherent OCP buses, and is typically used
 * for the L1-to-L2 buses and as the main system interconnect.  @sa
 * \ref gem5MemorySystem "gem5 Memory System"
 */
class CoherentXBar : public BaseXBar
{

  protected:

    /**
     * Declare the layers of this crossbar, one vector for requests,
     * one for responses, and one for snoop responses
     */
    std::vector<ReqLayer*> reqLayers;
    std::vector<RespLayer*> respLayers;
    std::vector<SnoopRespLayer*> snoopLayers;

    /**
     * Declaration of the coherent crossbar slave port type, one will
     * be instantiated for each of the master ports connecting to the
     * crossbar.
     */
    class CoherentXBarSlavePort : public QueuedSlavePort
    {

      private:

        /** A reference to the crossbar to which this port belongs. */
        CoherentXBar &xbar;

        /** A normal packet queue used to store responses. */
        RespPacketQueue queue;

      public:

        CoherentXBarSlavePort(const std::string &_name,
                             CoherentXBar &_xbar, PortID _id)
            : QueuedSlavePort(_name, &_xbar, queue, _id), xbar(_xbar),
              queue(_xbar, *this)
        { }

      protected:

        bool
        recvTimingReq(PacketPtr pkt) override
        {
            return xbar.recvTimingReq(pkt, id);
        }

        bool
        recvTimingSnoopResp(PacketPtr pkt) override
        {
            return xbar.recvTimingSnoopResp(pkt, id);
        }

        Tick
        recvAtomic(PacketPtr pkt) override
        {
            return xbar.recvAtomicBackdoor(pkt, id);
        }

        Tick
        recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor) override
        {
            return xbar.recvAtomicBackdoor(pkt, id, &backdoor);
        }

        void
        recvFunctional(PacketPtr pkt) override
        {
            xbar.recvFunctional(pkt, id);
        }

        AddrRangeList
        getAddrRanges() const override
        {
            return xbar.getAddrRanges();
        }

    };

    /**
     * Declaration of the coherent crossbar master port type, one will be
     * instantiated for each of the slave interfaces connecting to the
     * crossbar.
     */
    class CoherentXBarMasterPort : public MasterPort
    {
      private:
        /** A reference to the crossbar to which this port belongs. */
        CoherentXBar &xbar;

      public:

        CoherentXBarMasterPort(const std::string &_name,
                              CoherentXBar &_xbar, PortID _id)
            : MasterPort(_name, &_xbar, _id), xbar(_xbar)
        { }

      protected:

        /**
         * Determine if this port should be considered a snooper. For
         * a coherent crossbar master port this is always true.
         *
         * @return a boolean that is true if this port is snooping
         */
        bool isSnooping() const override { return true; }

        bool
        recvTimingResp(PacketPtr pkt) override
        {
            return xbar.recvTimingResp(pkt, id);
        }

        void
        recvTimingSnoopReq(PacketPtr pkt) override
        {
            return xbar.recvTimingSnoopReq(pkt, id);
        }

        Tick
        recvAtomicSnoop(PacketPtr pkt) override
        {
            return xbar.recvAtomicSnoop(pkt, id);
        }

        void
        recvFunctionalSnoop(PacketPtr pkt) override
        {
            xbar.recvFunctionalSnoop(pkt, id);
        }

        void recvRangeChange() override { xbar.recvRangeChange(id); }
        void recvReqRetry() override { xbar.recvReqRetry(id); }

    };

    /**
     * Internal class to bridge between an incoming snoop response
     * from a slave port and forwarding it through an outgoing slave
     * port. It is effectively a dangling master port.
     */
    class SnoopRespPort : public MasterPort
    {

      private:

        /** The port which we mirror internally. */
        QueuedSlavePort& slavePort;

      public:

        /**
         * Create a snoop response port that mirrors a given slave port.
         */
        SnoopRespPort(QueuedSlavePort& slave_port, CoherentXBar& _xbar) :
            MasterPort(slave_port.name() + ".snoopRespPort", &_xbar),
            slavePort(slave_port) { }

        /**
         * Override the sending of retries and pass them on through
         * the mirrored slave port.
         */
        void
        sendRetryResp() override
        {
            // forward it as a snoop response retry
            slavePort.sendRetrySnoopResp();
        }

        void
        recvReqRetry() override
        {
            panic("SnoopRespPort should never see retry");
        }

        bool
        recvTimingResp(PacketPtr pkt) override
        {
            panic("SnoopRespPort should never see timing response");
        }

    };

    std::vector<SnoopRespPort*> snoopRespPorts;

    std::vector<QueuedSlavePort*> snoopPorts;

    /**
     * Store the outstanding requests that we are expecting snoop
     * responses from so we can determine which snoop responses we
     * generated and which ones were merely forwarded.
     */
    std::unordered_set<RequestPtr> outstandingSnoop;

    /**
     * Store the outstanding cache maintenance that we are expecting
     * snoop responses from so we can determine when we received all
     * snoop responses and if any of the agents satisfied the request.
     */
    std::unordered_map<PacketId, PacketPtr> outstandingCMO;

    /**
     * Keep a pointer to the system to be allow to querying memory system
     * properties.
     */
    System *system;

    /** A snoop filter that tracks cache line residency and can restrict the
      * broadcast needed for probes.  NULL denotes an absent filter. */
    SnoopFilter *snoopFilter;

    /** Cycles of snoop response latency.*/
    const Cycles snoopResponseLatency;

    /** Maximum number of outstading snoops sanity check*/
    const unsigned int maxOutstandingSnoopCheck;

    /** Maximum routing table size sanity check*/
    const unsigned int maxRoutingTableSizeCheck;

    /** Is this crossbar the point of coherency? **/
    const bool pointOfCoherency;

    /** Is this crossbar the point of unification? **/
    const bool pointOfUnification;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

    bool recvTimingReq(PacketPtr pkt, PortID slave_port_id);
    bool recvTimingResp(PacketPtr pkt, PortID master_port_id);
    void recvTimingSnoopReq(PacketPtr pkt, PortID master_port_id);
    bool recvTimingSnoopResp(PacketPtr pkt, PortID slave_port_id);
    void recvReqRetry(PortID master_port_id);

    /**
     * Forward a timing packet to our snoopers, potentially excluding
     * one of the connected coherent masters to avoid sending a packet
     * back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     */
    void
    forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id)
    {
        forwardTiming(pkt, exclude_slave_port_id, snoopPorts);
    }

    /**
     * Forward a timing packet to a selected list of snoopers, potentially
     * excluding one of the connected coherent masters to avoid sending a packet
     * back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     * @param dests Vector of destination ports for the forwarded pkt
     */
    void forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id,
                       const std::vector<QueuedSlavePort*>& dests);

    Tick recvAtomicBackdoor(PacketPtr pkt, PortID slave_port_id,
                            MemBackdoorPtr *backdoor=nullptr);
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
    std::pair<MemCmd, Tick>
    forwardAtomic(PacketPtr pkt, PortID exclude_slave_port_id)
    {
        return forwardAtomic(pkt, exclude_slave_port_id, InvalidPortID,
                             snoopPorts);
    }

    /**
     * Forward an atomic packet to a selected list of snoopers, potentially
     * excluding one of the connected coherent masters to avoid sending a packet
     * back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     * @param source_master_port_id Id of the master port for snoops from below
     * @param dests Vector of destination ports for the forwarded pkt
     *
     * @return a pair containing the snoop response and snoop latency
     */
    std::pair<MemCmd, Tick> forwardAtomic(PacketPtr pkt,
                                          PortID exclude_slave_port_id,
                                          PortID source_master_port_id,
                                          const std::vector<QueuedSlavePort*>&
                                          dests);

    /** Function called by the port when the crossbar is recieving a Functional
        transaction.*/
    void recvFunctional(PacketPtr pkt, PortID slave_port_id);

    /** Function called by the port when the crossbar is recieving a functional
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

    /**
     * Determine if the crossbar should sink the packet, as opposed to
     * forwarding it, or responding.
     */
    bool sinkPacket(const PacketPtr pkt) const;

    /**
     * Determine if the crossbar should forward the packet, as opposed to
     * responding to it.
     */
    bool forwardPacket(const PacketPtr pkt);

    /**
     * Determine if the packet's destination is the memory below
     *
     * The memory below is the destination for a cache mainteance
     * operation to the Point of Coherence/Unification if this is the
     * Point of Coherence/Unification.
     *
     * @param pkt The processed packet
     *
     * @return Whether the memory below is the destination for the packet
     */
    bool
    isDestination(const PacketPtr pkt) const
    {
        return (pkt->req->isToPOC() && pointOfCoherency) ||
            (pkt->req->isToPOU() && pointOfUnification);
    }

    Stats::Scalar snoops;
    Stats::Scalar snoopTraffic;
    Stats::Distribution snoopFanout;

  public:

    virtual void init();

    CoherentXBar(const CoherentXBarParams *p);

    virtual ~CoherentXBar();

    virtual void regStats();
};

#endif //__MEM_COHERENT_XBAR_HH__
