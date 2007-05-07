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
 *
 * Authors: Ali Saidi
 *          Steve Reinhardt
 */

/**
 * @file
 * Declaration of a simple bus bridge object with no buffering
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
  protected:
    /** Declaration of the buses port type, one will be instantiated for each
        of the interfaces connecting to the bus. */
    class BridgePort : public Port
    {
        /** A pointer to the bridge to which this port belongs. */
        Bridge *bridge;

        /**
         * Pointer to the port on the other side of the bridge
         * (connected to the other bus).
         */
        BridgePort *otherPort;

        /** Minimum delay though this bridge. */
        Tick delay;

        bool fixPartialWrite;

        class PacketBuffer : public Packet::SenderState {

          public:
            Tick ready;
            PacketPtr pkt;
            Packet::SenderState *origSenderState;
            short origSrc;
            bool expectResponse;

            bool partialWriteFixed;
            PacketPtr oldPkt;

            PacketBuffer(PacketPtr _pkt, Tick t)
                : ready(t), pkt(_pkt),
                  origSenderState(_pkt->senderState), origSrc(_pkt->getSrc()),
                  expectResponse(_pkt->needsResponse()), partialWriteFixed(false)
            {
                if (!pkt->isResponse())
                    pkt->senderState = this;
            }

            void fixResponse(PacketPtr pkt)
            {
                assert(pkt->senderState == this);
                pkt->setDest(origSrc);
                pkt->senderState = origSenderState;
                if (partialWriteFixed)
                    delete oldPkt;
            }

            void partialWriteFix(Port *port)
            {
                assert(!partialWriteFixed);
                assert(expectResponse);

                int pbs = port->peerBlockSize();
                partialWriteFixed = true;
                PacketDataPtr data;

                data = new uint8_t[pbs];
                PacketPtr funcPkt = new Packet(pkt->req, MemCmd::ReadReq,
                        Packet::Broadcast, pbs);

                funcPkt->dataStatic(data);
                port->sendFunctional(funcPkt);
                assert(funcPkt->result == Packet::Success);
                delete funcPkt;

                oldPkt = pkt;
                memcpy(data + oldPkt->getOffset(pbs), pkt->getPtr<uint8_t>(),
                        pkt->getSize());
                pkt = new Packet(oldPkt->req, MemCmd::WriteInvalidateReq,
                        Packet::Broadcast, pbs);
                pkt->dataDynamicArray(data);
                pkt->senderState = oldPkt->senderState;
            }

            void undoPartialWriteFix()
            {
                if (!partialWriteFixed)
                    return;
                delete pkt;
                pkt = oldPkt;
                partialWriteFixed = false;
            }

        };

        /**
         * Outbound packet queue.  Packets are held in this queue for a
         * specified delay to model the processing delay of the
         * bridge.
         */
        std::list<PacketBuffer*> sendQueue;

        int outstandingResponses;

        /** Max queue size for outbound packets */
        int queueLimit;

        /**
         * Is this side blocked from accepting outbound packets?
         */
        bool queueFull() { return (sendQueue.size() == queueLimit); }

        bool queueForSendTiming(PacketPtr pkt);

        void finishSend(PacketBuffer *buf);

        /**
         * Handle send event, scheduled when the packet at the head of
         * the outbound queue is ready to transmit (for timing
         * accesses only).
         */
        void trySend();

        class SendEvent : public Event
        {
            BridgePort *port;

          public:
            SendEvent(BridgePort *p)
                : Event(&mainEventQueue), port(p) {}

            virtual void process() { port->trySend(); }

            virtual const char *description() { return "bridge send event"; }
        };

        SendEvent sendEvent;

      public:

        /** Constructor for the BusPort.*/
        BridgePort(const std::string &_name,
                   Bridge *_bridge, BridgePort *_otherPort,
                   int _delay, int _queueLimit, bool fix_partial_write);

      protected:

        /** When receiving a timing request from the peer port,
            pass it to the bridge. */
        virtual bool recvTiming(PacketPtr pkt);

        /** When receiving a retry request from the peer port,
            pass it to the bridge. */
        virtual void recvRetry();

        /** When receiving a Atomic requestfrom the peer port,
            pass it to the bridge. */
        virtual Tick recvAtomic(PacketPtr pkt);

        /** When receiving a Functional request from the peer port,
            pass it to the bridge. */
        virtual void recvFunctional(PacketPtr pkt);

        /** When receiving a status changefrom the peer port,
            pass it to the bridge. */
        virtual void recvStatusChange(Status status);

        /** When receiving a address range request the peer port,
            pass it to the bridge. */
        virtual void getDeviceAddressRanges(AddrRangeList &resp,
                                            AddrRangeList &snoop);
    };

    BridgePort portA, portB;

    /** If this bridge should acknowledge writes. */
    bool ackWrites;

  public:

    /** A function used to return the port associated with this bus object. */
    virtual Port *getPort(const std::string &if_name, int idx = -1);

    virtual void init();

    Bridge(const std::string &n, int qsa, int qsb, Tick _delay, int write_ack,
            bool fix_partial_write_a, bool fix_partial_write_b);
};

#endif //__MEM_BUS_HH__
