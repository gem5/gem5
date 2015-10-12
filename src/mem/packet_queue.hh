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
 *          Andreas Hansson
 */

#ifndef __MEM_PACKET_QUEUE_HH__
#define __MEM_PACKET_QUEUE_HH__

/**
 * @file
 * Declaration of a simple PacketQueue that is associated with
 * a port on which it attempts to send packets according to the time
 * stamp given to them at insertion. The packet queue is responsible
 * for the flow control of the port.
 */

#include <list>

#include "mem/port.hh"
#include "sim/drain.hh"
#include "sim/eventq_impl.hh"

/**
 * A packet queue is a class that holds deferred packets and later
 * sends them using the associated slave port or master port.
 */
class PacketQueue : public Drainable
{
  private:
    /** A deferred packet, buffered to transmit later. */
    class DeferredPacket {
      public:
        Tick tick;      ///< The tick when the packet is ready to transmit
        PacketPtr pkt;  ///< Pointer to the packet to transmit
        DeferredPacket(Tick t, PacketPtr p)
            : tick(t), pkt(p)
        {}
    };

    typedef std::list<DeferredPacket> DeferredPacketList;

    /** A list of outgoing packets. */
    DeferredPacketList transmitList;

    /** The manager which is used for the event queue */
    EventManager& em;

    /** Used to schedule sending of deferred packets. */
    void processSendEvent();

    /** Event used to call processSendEvent. */
    EventWrapper<PacketQueue, &PacketQueue::processSendEvent> sendEvent;

  protected:

    /** Label to use for print request packets label stack. */
    const std::string label;

    /** Remember whether we're awaiting a retry. */
    bool waitingOnRetry;

    /** Check whether we have a packet ready to go on the transmit list. */
    bool deferredPacketReady() const
    { return !transmitList.empty() && transmitList.front().tick <= curTick(); }

    /**
     * Attempt to send a packet. Note that a subclass of the
     * PacketQueue can override this method and thus change the
     * behaviour (as done by the cache for the request queue). The
     * default implementation sends the head of the transmit list. The
     * caller must guarantee that the list is non-empty and that the
     * head packet is scheduled for curTick() (or earlier).
     */
    virtual void sendDeferredPacket();

    /**
     * Send a packet using the appropriate method for the specific
     * subclass (reuest, response or snoop response).
     */
    virtual bool sendTiming(PacketPtr pkt) = 0;

    /**
     * Create a packet queue, linked to an event manager, and a label
     * that will be used for functional print request packets.
     *
     * @param _em Event manager used for scheduling this queue
     * @param _label Label to push on the label stack for print request packets
     */
    PacketQueue(EventManager& _em, const std::string& _label);

    /**
     * Virtual desctructor since the class may be used as a base class.
     */
    virtual ~PacketQueue();

  public:

    /**
     * Provide a name to simplify debugging.
     *
     * @return A complete name, appended to module and port
     */
    virtual const std::string name() const = 0;

    /**
     * Get the size of the queue.
     */
    size_t size() const { return transmitList.size(); }

    /**
     * Get the next packet ready time.
     */
    Tick deferredPacketReadyTime() const
    { return transmitList.empty() ? MaxTick : transmitList.front().tick; }

    /**
     * Check if a packets address exists in the queue.
     */
    bool hasAddr(Addr addr) const;

    /** Check the list of buffered packets against the supplied
     * functional request. */
    bool checkFunctional(PacketPtr pkt);

    /**
     * Schedule a send event if we are not already waiting for a
     * retry. If the requested time is before an already scheduled
     * send event, the event will be rescheduled. If MaxTick is
     * passed, no event is scheduled. Instead, if we are idle and
     * asked to drain then check and signal drained.
     *
     * @param when time to schedule an event
     */
    void schedSendEvent(Tick when);

    /**
     * Add a packet to the transmit list, and schedule a send event.
     *
     * @param pkt Packet to send
     * @param when Absolute time (in ticks) to send packet
     * @param force_order Do not reorder packets despite timing, but keep them
     *                    in insertion order.
     */
    void schedSendTiming(PacketPtr pkt, Tick when, bool force_order = false);

    /**
     * Retry sending a packet from the queue. Note that this is not
     * necessarily the same packet if something has been added with an
     * earlier time stamp.
     */
    void retry();

    DrainState drain() override;
};

class ReqPacketQueue : public PacketQueue
{

  protected:

    MasterPort& masterPort;

  public:

    /**
     * Create a request packet queue, linked to an event manager, a
     * master port, and a label that will be used for functional print
     * request packets.
     *
     * @param _em Event manager used for scheduling this queue
     * @param _masterPort Master port used to send the packets
     * @param _label Label to push on the label stack for print request packets
     */
    ReqPacketQueue(EventManager& _em, MasterPort& _masterPort,
                   const std::string _label = "ReqPacketQueue");

    virtual ~ReqPacketQueue() { }

    const std::string name() const
    { return masterPort.name() + "-" + label; }

    bool sendTiming(PacketPtr pkt);

};

class SnoopRespPacketQueue : public PacketQueue
{

  protected:

    MasterPort& masterPort;

  public:

    /**
     * Create a snoop response packet queue, linked to an event
     * manager, a master port, and a label that will be used for
     * functional print request packets.
     *
     * @param _em Event manager used for scheduling this queue
     * @param _masterPort Master port used to send the packets
     * @param _label Label to push on the label stack for print request packets
     */
    SnoopRespPacketQueue(EventManager& _em, MasterPort& _masterPort,
                         const std::string _label = "SnoopRespPacketQueue");

    virtual ~SnoopRespPacketQueue() { }

    const std::string name() const
    { return masterPort.name() + "-" + label; }

    bool sendTiming(PacketPtr pkt);

};

class RespPacketQueue : public PacketQueue
{

  protected:

    SlavePort& slavePort;

  public:

    /**
     * Create a response packet queue, linked to an event manager, a
     * slave port, and a label that will be used for functional print
     * request packets.
     *
     * @param _em Event manager used for scheduling this queue
     * @param _slavePort Slave port used to send the packets
     * @param _label Label to push on the label stack for print request packets
     */
    RespPacketQueue(EventManager& _em, SlavePort& _slavePort,
                     const std::string _label = "RespPacketQueue");

    virtual ~RespPacketQueue() { }

    const std::string name() const
    { return slavePort.name() + "-" + label; }

    bool sendTiming(PacketPtr pkt);

};

#endif // __MEM_PACKET_QUEUE_HH__
