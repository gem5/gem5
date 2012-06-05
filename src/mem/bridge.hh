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
 *          Steve Reinhardt
 *          Andreas Hansson
 */

/**
 * @file
 * Declaration of a memory-mapped bus bridge that connects a master
 * and a slave through a request and response queue.
 */

#ifndef __MEM_BRIDGE_HH__
#define __MEM_BRIDGE_HH__

#include <list>
#include <queue>
#include <string>

#include "base/types.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/Bridge.hh"
#include "sim/eventq.hh"

/**
 * A bridge is used to interface two different busses (or in general a
 * memory-mapped master and slave), with buffering for requests and
 * responses. The bridge has a fixed delay for packets passing through
 * it and responds to a fixed set of address ranges.
 *
 * The bridge comprises a slave port and a master port, that buffer
 * outgoing responses and requests respectively. Buffer space is
 * reserved when a request arrives, also reserving response space
 * before forwarding the request. An incoming request is always
 * accepted (recvTiming returns true), but is potentially NACKed if
 * there is no request space or response space.
 */
class Bridge : public MemObject
{
  protected:

    /**
     * A bridge request state stores packets along with their sender
     * state and original source. It has enough information to also
     * restore the response once it comes back to the bridge.
     */
    class RequestState : public Packet::SenderState
    {

      public:

        Packet::SenderState *origSenderState;
        PortID origSrc;

        RequestState(PacketPtr _pkt)
            : origSenderState(_pkt->senderState),
              origSrc(_pkt->getSrc())
        { }

        void fixResponse(PacketPtr pkt)
        {
            assert(pkt->senderState == this);
            pkt->setDest(origSrc);
            pkt->senderState = origSenderState;
        }
    };

    /**
     * A deferred request stores a packet along with its scheduled
     * transmission time, and whether we can expect to see a response
     * or not.
     */
    class DeferredRequest
    {

      public:

        Tick ready;
        PacketPtr pkt;
        bool expectResponse;

        DeferredRequest(PacketPtr _pkt, Tick t)
            : ready(t), pkt(_pkt), expectResponse(_pkt->needsResponse())
        { }
    };

    /**
     * A deferred response stores a packet along with its scheduled
     * transmission time. It also contains information of whether the
     * bridge NACKed the packet to be able to correctly maintain
     * counters of outstanding responses.
     */
    class DeferredResponse {

      public:

        Tick ready;
        PacketPtr pkt;
        bool nackedHere;

        DeferredResponse(PacketPtr _pkt, Tick t, bool nack = false)
            : ready(t), pkt(_pkt), nackedHere(nack)
        { }
    };

    // Forward declaration to allow the slave port to have a pointer
    class BridgeMasterPort;

    /**
     * The port on the side that receives requests and sends
     * responses. The slave port has a set of address ranges that it
     * is responsible for. The slave port also has a buffer for the
     * responses not yet sent.
     */
    class BridgeSlavePort : public SlavePort
    {

      private:

        /** A pointer to the bridge to which this port belongs. */
        Bridge *bridge;

        /**
         * Master port on the other side of the bridge
         * (connected to the other bus).
         */
        BridgeMasterPort& masterPort;

        /** Minimum request delay though this bridge. */
        Tick delay;

        /** Min delay to respond with a nack. */
        Tick nackDelay;

        /** Address ranges to pass through the bridge */
        AddrRangeList ranges;

        /**
         * Response packet queue. Response packets are held in this
         * queue for a specified delay to model the processing delay
         * of the bridge.
         */
        std::list<DeferredResponse> responseQueue;

        /** Counter to track the outstanding responses. */
        unsigned int outstandingResponses;

        /** If we're waiting for a retry to happen. */
        bool inRetry;

        /** Max queue size for reserved responses. */
        unsigned int respQueueLimit;

        /**
         * Is this side blocked from accepting new response packets.
         *
         * @return true if the reserved space has reached the set limit
         */
        bool respQueueFull();

        /**
         * Turn the request packet into a NACK response and put it in
         * the response queue and schedule its transmission.
         *
         * @param pkt the request packet to NACK
         */
        void nackRequest(PacketPtr pkt);

        /**
         * Handle send event, scheduled when the packet at the head of
         * the response queue is ready to transmit (for timing
         * accesses only).
         */
        void trySend();

        /**
         * Private class for scheduling sending of responses from the
         * response queue.
         */
        class SendEvent : public Event
        {
            BridgeSlavePort& port;

          public:
            SendEvent(BridgeSlavePort& p) : port(p) {}
            virtual void process() { port.trySend(); }
            virtual const char *description() const { return "bridge send"; }
        };

        /** Send event for the response queue. */
        SendEvent sendEvent;

      public:

        /**
         * Constructor for the BridgeSlavePort.
         *
         * @param _name the port name including the owner
         * @param _bridge the structural owner
         * @param _masterPort the master port on the other side of the bridge
         * @param _delay the delay from seeing a response to sending it
         * @param _nack_delay the delay from a NACK to sending the response
         * @param _resp_limit the size of the response queue
         * @param _ranges a number of address ranges to forward
         */
        BridgeSlavePort(const std::string &_name, Bridge *_bridge,
                        BridgeMasterPort& _masterPort, int _delay,
                        int _nack_delay, int _resp_limit,
                        std::vector<Range<Addr> > _ranges);

        /**
         * Queue a response packet to be sent out later and also schedule
         * a send if necessary.
         *
         * @param pkt a response to send out after a delay
         */
        void queueForSendTiming(PacketPtr pkt);

      protected:

        /** When receiving a timing request from the peer port,
            pass it to the bridge. */
        virtual bool recvTimingReq(PacketPtr pkt);

        /** When receiving a retry request from the peer port,
            pass it to the bridge. */
        virtual void recvRetry();

        /** When receiving a Atomic requestfrom the peer port,
            pass it to the bridge. */
        virtual Tick recvAtomic(PacketPtr pkt);

        /** When receiving a Functional request from the peer port,
            pass it to the bridge. */
        virtual void recvFunctional(PacketPtr pkt);

        /** When receiving a address range request the peer port,
            pass it to the bridge. */
        virtual AddrRangeList getAddrRanges();
    };


    /**
     * Port on the side that forwards requests and receives
     * responses. The master port has a buffer for the requests not
     * yet sent.
     */
    class BridgeMasterPort : public MasterPort
    {

      private:

        /** A pointer to the bridge to which this port belongs. */
        Bridge* bridge;

        /**
         * Pointer to the slave port on the other side of the bridge
         * (connected to the other bus).
         */
        BridgeSlavePort& slavePort;

        /** Minimum delay though this bridge. */
        Tick delay;

        /**
         * Request packet queue. Request packets are held in this
         * queue for a specified delay to model the processing delay
         * of the bridge.
         */
        std::list<DeferredRequest> requestQueue;

        /** If we're waiting for a retry to happen. */
        bool inRetry;

        /** Max queue size for request packets */
        unsigned int reqQueueLimit;

        /**
         * Handle send event, scheduled when the packet at the head of
         * the outbound queue is ready to transmit (for timing
         * accesses only).
         */
        void trySend();

        /**
         * Private class for scheduling sending of requests from the
         * request queue.
         */
        class SendEvent : public Event
        {
            BridgeMasterPort& port;

          public:
            SendEvent(BridgeMasterPort& p) : port(p) {}
            virtual void process() { port.trySend(); }
            virtual const char *description() const { return "bridge send"; }
        };

        /** Send event for the request queue. */
        SendEvent sendEvent;

      public:

        /**
         * Constructor for the BridgeMasterPort.
         *
         * @param _name the port name including the owner
         * @param _bridge the structural owner
         * @param _slavePort the slave port on the other side of the bridge
         * @param _delay the delay from seeing a request to sending it
         * @param _req_limit the size of the request queue
         */
        BridgeMasterPort(const std::string &_name, Bridge *_bridge,
                         BridgeSlavePort& _slavePort, int _delay,
                         int _req_limit);

        /**
         * Is this side blocked from accepting new request packets.
         *
         * @return true if the occupied space has reached the set limit
         */
        bool reqQueueFull();

        /**
         * Queue a request packet to be sent out later and also schedule
         * a send if necessary.
         *
         * @param pkt a request to send out after a delay
         */
        void queueForSendTiming(PacketPtr pkt);

        /**
         * Check a functional request against the packets in our
         * request queue.
         *
         * @param pkt packet to check against
         *
         * @return true if we find a match
         */
        bool checkFunctional(PacketPtr pkt);

      protected:

        /** When receiving a timing request from the peer port,
            pass it to the bridge. */
        virtual bool recvTimingResp(PacketPtr pkt);

        /** When receiving a retry request from the peer port,
            pass it to the bridge. */
        virtual void recvRetry();
    };

    /** Slave port of the bridge. */
    BridgeSlavePort slavePort;

    /** Master port of the bridge. */
    BridgeMasterPort masterPort;

    /** If this bridge should acknowledge writes. */
    bool ackWrites;

  public:
    typedef BridgeParams Params;

  protected:
    Params *_params;

  public:
    const Params *params() const { return _params; }

    virtual MasterPort& getMasterPort(const std::string& if_name,
                                      int idx = -1);
    virtual SlavePort& getSlavePort(const std::string& if_name, int idx = -1);

    virtual void init();

    Bridge(Params *p);
};

#endif //__MEM_BUS_HH__
