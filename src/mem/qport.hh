/*
 * Copyright (c) 2012,2015 ARM Limited
 * All rights reserved.
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

#ifndef __MEM_QPORT_HH__
#define __MEM_QPORT_HH__

/**
 * @file
 * Declaration of the queued port.
 */

#include "mem/packet_queue.hh"
#include "mem/port.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * A queued port is a port that has an infinite queue for outgoing
 * packets and thus decouples the module that wants to send
 * request/responses from the flow control (retry mechanism) of the
 * port. A queued port can be used by both a requestor and a responder. The
 * queue is a parameter to allow tailoring of the queue implementation
 * (used in the cache).
 */
class QueuedResponsePort : public ResponsePort
{

  protected:

    /** Packet queue used to store outgoing responses. */
    RespPacketQueue &respQueue;

    void recvRespRetry() { respQueue.retry(); }

  public:

    /**
     * Create a QueuedPort with a given name, owner, and a supplied
     * implementation of a packet queue. The external definition of
     * the queue enables e.g. the cache to implement a specific queue
     * behaviuor in a subclass, and provide the latter to the
     * QueuePort constructor.
     */
    QueuedResponsePort(const std::string& name, SimObject* owner,
                    RespPacketQueue &resp_queue, PortID id = InvalidPortID) :
        ResponsePort(name, owner, id), respQueue(resp_queue)
    { }

    virtual ~QueuedResponsePort() { }

    /**
     * Schedule the sending of a timing response.
     *
     * @param pkt Packet to send
     * @param when Absolute time (in ticks) to send packet
     */
    void schedTimingResp(PacketPtr pkt, Tick when)
    { respQueue.schedSendTiming(pkt, when); }

    /** Check the list of buffered packets against the supplied
     * functional request. */
    bool trySatisfyFunctional(PacketPtr pkt)
    { return respQueue.trySatisfyFunctional(pkt); }
};

/**
 * The QueuedRequestPort combines two queues, a request queue and a
 * snoop response queue, that both share the same port. The flow
 * control for requests and snoop responses are completely
 * independent, and so each queue manages its own flow control
 * (retries).
 */
class QueuedRequestPort : public RequestPort
{

  protected:

    /** Packet queue used to store outgoing requests. */
    ReqPacketQueue &reqQueue;

    /** Packet queue used to store outgoing snoop responses. */
    SnoopRespPacketQueue &snoopRespQueue;

    void recvReqRetry() { reqQueue.retry(); }

    void recvRetrySnoopResp() { snoopRespQueue.retry(); }

  public:

    /**
     * Create a QueuedPort with a given name, owner, and a supplied
     * implementation of two packet queues. The external definition of
     * the queues enables e.g. the cache to implement a specific queue
     * behaviuor in a subclass, and provide the latter to the
     * QueuePort constructor.
     */
    QueuedRequestPort(const std::string& name, SimObject* owner,
                     ReqPacketQueue &req_queue,
                     SnoopRespPacketQueue &snoop_resp_queue,
                     PortID id = InvalidPortID) :
        RequestPort(name, owner, id), reqQueue(req_queue),
        snoopRespQueue(snoop_resp_queue)
    { }

    virtual ~QueuedRequestPort() { }

    /**
     * Schedule the sending of a timing request.
     *
     * @param pkt Packet to send
     * @param when Absolute time (in ticks) to send packet
     */
    void schedTimingReq(PacketPtr pkt, Tick when)
    { reqQueue.schedSendTiming(pkt, when); }

    /**
     * Schedule the sending of a timing snoop response.
     *
     * @param pkt Packet to send
     * @param when Absolute time (in ticks) to send packet
     */
    void schedTimingSnoopResp(PacketPtr pkt, Tick when)
    { snoopRespQueue.schedSendTiming(pkt, when); }

    /** Check the list of buffered packets against the supplied
     * functional request. */
    bool trySatisfyFunctional(PacketPtr pkt)
    {
        return reqQueue.trySatisfyFunctional(pkt) ||
            snoopRespQueue.trySatisfyFunctional(pkt);
    }
};

} // namespace gem5

#endif // __MEM_QPORT_HH__
