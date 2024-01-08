/*
 * Copyright (c) 2011-2013 ARM Limited
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
 * @file
 * Declaration of a memory-mapped bridge that connects a requestor
 * and a responder through a request and response queue.
 */

#ifndef __MEM_BRIDGE_HH__
#define __MEM_BRIDGE_HH__

#include <deque>

#include "base/types.hh"
#include "mem/port.hh"
#include "params/Bridge.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * A bridge is used to interface two different crossbars (or in general a
 * memory-mapped requestor and responder), with buffering for requests and
 * responses. The bridge has a fixed delay for packets passing through
 * it and responds to a fixed set of address ranges.
 *
 * The bridge comprises a response port and a request port, that buffer
 * outgoing responses and requests respectively. Buffer space is
 * reserved when a request arrives, also reserving response space
 * before forwarding the request. If there is no space present, then
 * the bridge will delay accepting the packet until space becomes
 * available.
 */
class Bridge : public ClockedObject
{
  protected:

    /**
     * A deferred packet stores a packet along with its scheduled
     * transmission time
     */
    class DeferredPacket
    {

      public:

        const Tick tick;
        const PacketPtr pkt;

        DeferredPacket(PacketPtr _pkt, Tick _tick) : tick(_tick), pkt(_pkt)
        { }
    };

    // Forward declaration to allow the response port to have a pointer
    class BridgeRequestPort;

    /**
     * The port on the side that receives requests and sends
     * responses. The response port has a set of address ranges that it
     * is responsible for. The response port also has a buffer for the
     * responses not yet sent.
     */
    class BridgeResponsePort : public ResponsePort
    {

      private:

        /** The bridge to which this port belongs. */
        Bridge& bridge;

        /**
         * Request port on the other side of the bridge.
         */
        BridgeRequestPort& memSidePort;
        BridgeRequestPort& shadowPort;
        bool& is_main_cache;

        /** Minimum request delay though this bridge. */
        const Cycles delay;

        /** Address ranges to pass through the bridge */
        const AddrRangeList ranges;

        /**
         * Response packet queue. Response packets are held in this
         * queue for a specified delay to model the processing delay
         * of the bridge. We use a deque as we need to iterate over
         * the items for functional accesses.
         */
        std::deque<DeferredPacket> transmitList;

        /** Counter to track the outstanding responses. */
        unsigned int outstandingResponses;

        /** If we should send a retry when space becomes available. */
        bool retryReq;

        /** Max queue size for reserved responses. */
        unsigned int respQueueLimit;

        /**
         * Upstream caches need this packet until true is returned, so
         * hold it for deletion until a subsequent call
         */
        std::unique_ptr<Packet> pendingDelete;

        /**
         * Is this side blocked from accepting new response packets.
         *
         * @return true if the reserved space has reached the set limit
         */
        bool respQueueFull() const;

        /**
         * Handle send event, scheduled when the packet at the head of
         * the response queue is ready to transmit (for timing
         * accesses only).
         */
        void trySendTiming();

        /** Send event for the response queue. */
        EventFunctionWrapper sendEvent;

      public:

        /**
         * Constructor for the BridgeResponsePort.
         *
         * @param _name the port name including the owner
         * @param _bridge the structural owner
         * @param _memSidePort the request port on the other
         *                       side of the bridge
         * @param _delay the delay in cycles from receiving to sending
         * @param _resp_limit the size of the response queue
         * @param _ranges a number of address ranges to forward
         */
        BridgeResponsePort(const std::string& _name, Bridge& _bridge,
                        BridgeRequestPort& _memSidePort, BridgeRequestPort& _shadowPort, bool& _main_cache, Cycles _delay,
                        int _resp_limit, std::vector<AddrRange> _ranges);

        /**
         * Queue a response packet to be sent out later and also schedule
         * a send if necessary.
         *
         * @param pkt a response to send out after a delay
         * @param when tick when response packet should be sent
         */
        void schedTimingResp(PacketPtr pkt, Tick when);

        /**
         * Retry any stalled request that we have failed to accept at
         * an earlier point in time. This call will do nothing if no
         * request is waiting.
         */
        void retryStalledReq();

      protected:

        /** When receiving a timing request from the peer port,
            pass it to the bridge. */
        bool recvTimingReq(PacketPtr pkt) override;

        /** When receiving a retry request from the peer port,
            pass it to the bridge. */
        void recvRespRetry() override;

        /** When receiving an Atomic request from the peer port,
            pass it to the bridge. */
        Tick recvAtomic(PacketPtr pkt) override;

        /** When receiving an Atomic backdoor request from the peer port,
            pass it to the bridge. */
        Tick recvAtomicBackdoor(
            PacketPtr pkt, MemBackdoorPtr &backdoor) override;


        /** When receiving a Functional request from the peer port,
            pass it to the bridge. */
        void recvFunctional(PacketPtr pkt) override;

        /** When receiving a Functional backdoor request from the peer port,
            pass it to the bridge. */
        void recvMemBackdoorReq(
            const MemBackdoorReq &req, MemBackdoorPtr &backdoor) override;


        /** When receiving a address range request the peer port,
            pass it to the bridge. */
        AddrRangeList getAddrRanges() const override;
    };


    /**
     * Port on the side that forwards requests and receives
     * responses. The request port has a buffer for the requests not
     * yet sent.
     */
    class BridgeRequestPort : public RequestPort
    {

      private:

        /** The bridge to which this port belongs. */
        Bridge& bridge;

        /**
         * The response port on the other side of the bridge.
         */
        BridgeResponsePort& cpuSidePort;

        /** Minimum delay though this bridge. */
        const Cycles delay;

        /**
         * Request packet queue. Request packets are held in this
         * queue for a specified delay to model the processing delay
         * of the bridge.  We use a deque as we need to iterate over
         * the items for functional accesses.
         */
        std::deque<DeferredPacket> transmitList;

        /** Max queue size for request packets */
        const unsigned int reqQueueLimit;

        /**
         * Handle send event, scheduled when the packet at the head of
         * the outbound queue is ready to transmit (for timing
         * accesses only).
         */
        void trySendTiming();

        /** Send event for the request queue. */
        EventFunctionWrapper sendEvent;

      public:

        /**
         * Constructor for the BridgeRequestPort.
         *
         * @param _name the port name including the owner
         * @param _bridge the structural owner
         * @param _cpuSidePort the response port on the other side of
         * the bridge
         * @param _delay the delay in cycles from receiving to sending
         * @param _req_limit the size of the request queue
         */
        BridgeRequestPort(const std::string& _name, Bridge& _bridge,
                         BridgeResponsePort& _cpuSidePort, Cycles _delay,
                         int _req_limit);

        /**
         * Is this side blocked from accepting new request packets.
         *
         * @return true if the occupied space has reached the set limit
         */
        bool reqQueueFull() const;

        /**
         * Queue a request packet to be sent out later and also schedule
         * a send if necessary.
         *
         * @param pkt a request to send out after a delay
         * @param when tick when response packet should be sent
         */
        void schedTimingReq(PacketPtr pkt, Tick when);

        /**
         * Check a functional request against the packets in our
         * request queue.
         *
         * @param pkt packet to check against
         *
         * @return true if we find a match
         */
        bool trySatisfyFunctional(PacketPtr pkt);

      protected:

        /** When receiving a timing request from the peer port,
            pass it to the bridge. */
        bool recvTimingResp(PacketPtr pkt) override;

        /** When receiving a retry request from the peer port,
            pass it to the bridge. */
        void recvReqRetry() override;
    };

    /** Response port of the bridge. */
    BridgeResponsePort cpuSidePort;

    /** Request port of the bridge. */
    BridgeRequestPort memSidePort;

    /** additional request port of the bridge*/
    BridgeRequestPort shadowPort;

    bool is_main;

  public:

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    void init() override;

    typedef BridgeParams Params;

    Bridge(const Params &p);
};

} // namespace gem5

#endif //__MEM_BRIDGE_HH__
