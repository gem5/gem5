/*
 * Copyright (c) 2012 ARM Limited
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
 * for the flow control of the port, but relies on the module
 * notifying the queue when a transfer ends.
 */

#include <list>

#include "mem/port.hh"
#include "sim/eventq.hh"

/**
 * A packet queue is a class that holds deferred packets and later
 * sends them using the associated slave port or master port.
 */
class PacketQueue
{
  private:
    /** A deferred packet, buffered to transmit later. */
    class DeferredPacket {
      public:
        Tick tick;      ///< The tick when the packet is ready to transmit
        PacketPtr pkt;  ///< Pointer to the packet to transmit
        bool sendAsSnoop; ///< Should it be sent as a snoop or not
        DeferredPacket(Tick t, PacketPtr p, bool send_as_snoop)
            : tick(t), pkt(p), sendAsSnoop(send_as_snoop)
        {}
    };

    typedef std::list<DeferredPacket> DeferredPacketList;
    typedef std::list<DeferredPacket>::iterator DeferredPacketIterator;

    /** A list of outgoing timing response packets that haven't been
     * serviced yet. */
    DeferredPacketList transmitList;

    /** The manager which is used for the event queue */
    EventManager& em;

    /** This function attempts to send deferred packets.  Scheduled to
     * be called in the future via SendEvent. */
    void processSendEvent();

    /**
     * Event used to call processSendEvent.
     **/
    EventWrapper<PacketQueue, &PacketQueue::processSendEvent> sendEvent;

    /** If we need to drain, keep the drain event around until we're done
     * here.*/
    Event *drainEvent;

  protected:

    /** Label to use for print request packets label stack. */
    const std::string label;

    /** Remember whether we're awaiting a retry from the bus. */
    bool waitingOnRetry;

    /** Check whether we have a packet ready to go on the transmit list. */
    bool deferredPacketReady()
    { return !transmitList.empty() && transmitList.front().tick <= curTick(); }

    Tick deferredPacketReadyTime()
    { return transmitList.empty() ? MaxTick : transmitList.front().tick; }

    /**
     * Attempt to send the packet at the head of the transmit
     * list. Caller must guarantee that the list is non-empty and that
     * the head packet is scheduled for curTick() (or earlier). Note
     * that a subclass of the PacketQueue can override this method and
     * thus change the behaviour (as done by the cache).
     */
    virtual void sendDeferredPacket();

    /**
     * Attempt to send the packet at the front of the transmit list,
     * and set waitingOnRetry accordingly. The packet is temporarily
     * taken off the list, but put back at the front if not
     * successfully sent.
     */
    void trySendTiming();

    /**
     *
     */
    virtual bool sendTiming(PacketPtr pkt, bool send_as_snoop) = 0;

    /**
     * Based on the transmit list, or the provided time, schedule a
     * send event if there are packets to send. If we are idle and
     * asked to drain then do so.
     *
     * @param time an alternative time for the next send event
     */
    void scheduleSend(Tick time = MaxTick);

    /**
     * Simple ports are generally used as slave ports (i.e. the
     * respond to requests) and thus do not expect to receive any
     * range changes (as the neighbouring port has a master role and
     * do not have any address ranges. A subclass can override the
     * default behaviuor if needed.
     */
    virtual void recvRangeChange() { }

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

    /** Check the list of buffered packets against the supplied
     * functional request. */
    bool checkFunctional(PacketPtr pkt);

    /**
     * Schedule a send even if not already waiting for a retry. If the
     * requested time is before an already scheduled send event it
     * will be rescheduled.
     *
     * @param when
     */
    void schedSendEvent(Tick when);

    /**
     * Add a packet to the transmit list, and ensure that a
     * processSendEvent is called in the future.
     *
     * @param pkt Packet to send
     * @param when Absolute time (in ticks) to send packet
     * @param send_as_snoop Send the packet as a snoop or not
     */
    void schedSendTiming(PacketPtr pkt, Tick when, bool send_as_snoop = false);

    /**
     * Used by a port to notify the queue that a retry was received
     * and that the queue can proceed and retry sending the packet
     * that caused the wait.
     */
    void retry();

    /**
     * Hook for draining the packet queue.
     *
     * @param de An event which is used to signal back to the caller
     * @return A number indicating how many times process will be called
     */
    unsigned int drain(Event *de);
};

class MasterPacketQueue : public PacketQueue
{

  protected:

    MasterPort& masterPort;

  public:

    /**
     * Create a master packet queue, linked to an event manager, a
     * master port, and a label that will be used for functional print
     * request packets.
     *
     * @param _em Event manager used for scheduling this queue
     * @param _masterPort Master port used to send the packets
     * @param _label Label to push on the label stack for print request packets
     */
    MasterPacketQueue(EventManager& _em, MasterPort& _masterPort,
                      const std::string _label = "MasterPacketQueue");

    virtual ~MasterPacketQueue() { }

    const std::string name() const
    { return masterPort.name() + "-" + label; }

    bool sendTiming(PacketPtr pkt, bool send_as_snoop);
};

class SlavePacketQueue : public PacketQueue
{

  protected:

    SlavePort& slavePort;

  public:

    /**
     * Create a slave packet queue, linked to an event manager, a
     * slave port, and a label that will be used for functional print
     * request packets.
     *
     * @param _em Event manager used for scheduling this queue
     * @param _slavePort Slave port used to send the packets
     * @param _label Label to push on the label stack for print request packets
     */
    SlavePacketQueue(EventManager& _em, SlavePort& _slavePort,
                     const std::string _label = "SlavePacketQueue");

    virtual ~SlavePacketQueue() { }

    const std::string name() const
    { return slavePort.name() + "-" + label; }

    bool sendTiming(PacketPtr pkt, bool send_as_snoop);

};

#endif // __MEM_PACKET_QUEUE_HH__
