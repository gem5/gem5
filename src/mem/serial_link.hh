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
 * Copyright (c) 2015 The University of Bologna
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
 * Declaration of the SerialLink Class, modeling Hybrid-Memory-Cube's serial
 * interface.
 */

#ifndef __MEM_SERIAL_LINK_HH__
#define __MEM_SERIAL_LINK_HH__

#include <deque>

#include "base/types.hh"
#include "mem/port.hh"
#include "params/SerialLink.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * SerialLink is a simple variation of the Bridge class, with the ability to
 * account for the latency of packet serialization. We assume that the
 * serializer component at the transmitter side does not need to receive the
 * whole packet to start the serialization. But the deserializer waits for the
 * complete packet to check its integrity first.
 */
class SerialLink : public ClockedObject
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

        DeferredPacket(PacketPtr _pkt, Tick _tick) : tick(_tick), pkt(_pkt) {}
    };

    // Forward declaration to allow the CPU-side port to have a pointer
    class SerialLinkRequestPort;

    /**
     * The port on the side that receives requests and sends
     * responses. The CPU-side port has a set of address ranges that it
     * is responsible for. The CPU-side port also has a buffer for the
     * responses not yet sent.
     */
    class SerialLinkResponsePort : public ResponsePort
    {
      private:
        /** The serial_link to which this port belongs. */
        SerialLink &serial_link;

        /**
         * Request port on the other side of the serial_link.
         */
        SerialLinkRequestPort &mem_side_port;

        /** Minimum request delay though this serial_link. */
        const Cycles delay;

        /** Address ranges to pass through the serial_link */
        const AddrRangeList ranges;

        /**
         * Response packet queue. Response packets are held in this
         * queue for a specified delay to model the processing delay
         * of the serial_link. We use a deque as we need to iterate over
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
         * Constructor for the SerialLinkResponsePort.
         *
         * @param _name the port name including the owner
         * @param _serial_link the structural owner
         * @param _mem_side_port the memory-side port on the other side of the
         * serial_link
         * @param _delay the delay in cycles from receiving to sending
         * @param _resp_limit the size of the response queue
         * @param _ranges a number of address ranges to forward
         */
        SerialLinkResponsePort(const std::string &_name,
                               SerialLink &_serial_link,
                               SerialLinkRequestPort &_mem_side_port,
                               Cycles _delay, int _resp_limit,
                               const std::vector<AddrRange> &_ranges);

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
            pass it to the serial_link. */
        bool recvTimingReq(PacketPtr pkt);

        /** When receiving a retry request from the peer port,
            pass it to the serial_link. */
        void recvRespRetry();

        /** When receiving a Atomic requestfrom the peer port,
            pass it to the serial_link. */
        Tick recvAtomic(PacketPtr pkt);

        /** When receiving a Functional request from the peer port,
            pass it to the serial_link. */
        void recvFunctional(PacketPtr pkt);

        /** When receiving a address range request the peer port,
            pass it to the serial_link. */
        AddrRangeList getAddrRanges() const;
    };

    /**
     * Port on the side that forwards requests and receives
     * responses. The memory-side port has a buffer for the requests not
     * yet sent.
     */
    class SerialLinkRequestPort : public RequestPort
    {
      private:
        /** The serial_link to which this port belongs. */
        SerialLink &serial_link;

        /**
         * The response (CPU-side port) port on the other side of
         * the serial_link.
         */
        SerialLinkResponsePort &cpu_side_port;

        /** Minimum delay though this serial_link. */
        const Cycles delay;

        /**
         * Request packet queue. Request packets are held in this
         * queue for a specified delay to model the processing delay
         * of the serial_link.  We use a deque as we need to iterate over
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
         * Constructor for the SerialLinkRequestPort.
         *
         * @param _name the port name including the owner
         * @param _serial_link the structural owner
         * @param _cpu_side_port the CPU-side port on the other
         * side of the serial_link
         * @param _delay the delay in cycles from receiving to sending
         * @param _req_limit the size of the request queue
         */
        SerialLinkRequestPort(const std::string &_name,
                              SerialLink &_serial_link,
                              SerialLinkResponsePort &_cpu_side_port,
                              Cycles _delay, int _req_limit);

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
            pass it to the serial_link. */
        bool recvTimingResp(PacketPtr pkt);

        /** When receiving a retry request from the peer port,
            pass it to the serial_link. */
        void recvReqRetry();
    };

    /** Response port of the serial_link. */
    SerialLinkResponsePort cpu_side_port;

    /** Request port of the serial_link. */
    SerialLinkRequestPort mem_side_port;

    /** Number of parallel lanes in this serial link */
    unsigned num_lanes;

    /** Speed of each link (Gb/s) in this serial link */
    uint64_t link_speed;

  public:
    Port &getPort(const std::string &if_name, PortID idx = InvalidPortID);

    virtual void init();

    typedef SerialLinkParams Params;

    SerialLink(const SerialLinkParams &p);
};

} // namespace gem5

#endif //__MEM_SERIAL_LINK_HH__
