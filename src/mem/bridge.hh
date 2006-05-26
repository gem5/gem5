/*
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
 * @file Decleration of a simple bus bridge object with no buffering
 */

#ifndef __MEM_BRIDGE_HH__
#define __MEM_BRIDGE_HH__

#include <string>
#include <list>
#include <inttypes.h>
#include <queue>


#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "sim/eventq.hh"

class Bridge : public MemObject
{
  public:
    enum Side
    {
        SideA,
        SideB
    };

  protected:
    /** Function called by the port when the bus is recieving a Timing
        transaction.*/
    bool recvTiming(Packet *pkt, Side id);

    /** Function called by the port when the bus is recieving a Atomic
        transaction.*/
    Tick recvAtomic(Packet *pkt, Side id);

    /** Function called by the port when the bus is recieving a Functional
        transaction.*/
    void recvFunctional(Packet *pkt, Side id);

    /** Function called by the port when the bus is recieving a status change.*/
    void recvStatusChange(Port::Status status, Side id);

    /** Process address range request.
     * @param resp addresses that we can respond to
     * @param snoop addresses that we would like to snoop
     * @param id ide of the busport that made the request.
     */
    void addressRanges(AddrRangeList &resp, AddrRangeList &snoop, Side id);


    /** Event that the SendEvent calls when it fires. This code must reschedule
     * the send event as required. */
    void timerEvent();

    /** Decleration of the buses port type, one will be instantiated for each
        of the interfaces connecting to the bus. */
    class BridgePort : public Port
    {
        /** A pointer to the bus to which this port belongs. */
        Bridge *bridge;

        /** A id to keep track of the intercafe ID this port is connected to. */
        Bridge::Side side;

      public:

        /** Constructor for the BusPort.*/
        BridgePort(Bridge *_bridge, Side _side)
            : Port(""), bridge(_bridge), side(_side)
        { }

        int numQueued() { return outbound.size(); }

      protected:
        /** Data this is waiting to be transmitted. */
        std::list<std::pair<Packet*, Tick> > outbound;

        void sendPkt(Packet *pkt);
        void sendPkt(std::pair<Packet*, Tick> p);

        /** When reciving a timing request from the peer port,
            pass it to the bridge. */
        virtual bool recvTiming(Packet *pkt)
        { return bridge->recvTiming(pkt, side); }

        /** When reciving a retry request from the peer port,
            pass it to the bridge. */
        virtual Packet* recvRetry();

        /** When reciving a Atomic requestfrom the peer port,
            pass it to the bridge. */
        virtual Tick recvAtomic(Packet *pkt)
        { return bridge->recvAtomic(pkt, side); }

        /** When reciving a Functional request from the peer port,
            pass it to the bridge. */
        virtual void recvFunctional(Packet *pkt)
        { bridge->recvFunctional(pkt, side); }

        /** When reciving a status changefrom the peer port,
            pass it to the bridge. */
        virtual void recvStatusChange(Status status)
        { bridge->recvStatusChange(status, side); }

        /** When reciving a address range request the peer port,
            pass it to the bridge. */
        virtual void getDeviceAddressRanges(AddrRangeList &resp, AddrRangeList &snoop)
        { bridge->addressRanges(resp, snoop, side); }

        friend class Bridge;
    };

    class SendEvent : public Event
    {
        Bridge *bridge;

        SendEvent(Bridge *b)
            : Event(&mainEventQueue), bridge(b) {}

        virtual void process() { bridge->timerEvent(); }

        virtual const char *description() { return "bridge delay event"; }
        friend class Bridge;
    };

    SendEvent sendEvent;

    /** Sides of the bus bridges. */
    BridgePort* sideA;
    BridgePort* sideB;

    /** inbound queues on both sides. */
    std::list<std::pair<Packet*, Tick> > inboundA;
    std::list<std::pair<Packet*, Tick> > inboundB;

    /** The size of the queue for data coming into side a */
    int queueSizeA;
    int queueSizeB;

    /* if the side is blocked or not. */
    bool blockedA;
    bool blockedB;

    /** Miminum delay though this bridge. */
    Tick delay;

    /** If this bridge should acknowledge writes. */
    bool ackWrites;

  public:

    /** A function used to return the port associated with this bus object. */
    virtual Port *getPort(const std::string &if_name)
    {
        if (if_name == "side_a") {
            if (sideA != NULL)
                panic("bridge side a already connected to.");
            sideA = new BridgePort(this, SideA);
            return sideA;
        } else if (if_name == "side_b") {
            if (sideB != NULL)
                panic("bridge side b already connected to.");
            sideB = new BridgePort(this, SideB);
            return sideB;
        } else
            return NULL;
    }

    virtual void init();

    Bridge(const std::string &n, int qsa, int qsb, Tick _delay, int write_ack)
        : MemObject(n), sendEvent(this), sideA(NULL), sideB(NULL),
          queueSizeA(qsa), queueSizeB(qsb), blockedA(false), blockedB(false),
          delay(_delay), ackWrites(write_ack)
          {}

    /** Check if the port should block/unblock after recieving/sending a packet.
     * */
    void blockCheck(Side id);

    friend class Bridge::SendEvent;

};

#endif //__MEM_BUS_HH__
