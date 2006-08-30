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
 */

/**
 * @file
 * Implement a port which adds simple support of a sendTiming() function that
 * takes a delay. In this way the * device can immediatly call
 * sendTiming(pkt, time) after processing a request and the request will be
 * handled by the port even if the port bus the device connects to is blocked.
 */

/** recvTiming and drain should be implemented something like this when this
 * class is used.

bool
PioPort::recvTiming(Packet *pkt)
{
    if (pkt->result == Packet::Nacked) {
        resendNacked(pkt);
    } else {
        Tick latency = device->recvAtomic(pkt);
        // turn packet around to go back to requester
        pkt->makeTimingResponse();
        sendTiming(pkt, latency);
    }
    return true;
}

PioDevice::drain(Event *de)
{
    unsigned int count;
    count = SimpleTimingPort->drain(de);
    if (count)
        changeState(Draining);
    else
        changeState(Drained);
    return count;
}
*/

#ifndef __MEM_TPORT_HH__
#define __MEM_TPORT_HH__

#include "mem/port.hh"
#include "sim/eventq.hh"
#include <list>
#include <string>

class SimpleTimingPort : public Port
{
  protected:
    /** A list of outgoing timing response packets that haven't been
     * serviced yet. */
    std::list<Packet*> transmitList;
    /**
     * This class is used to implemented sendTiming() with a delay. When
     * a delay is requested a new event is created. When the event time
     * expires it attempts to send the packet. If it cannot, the packet
     * is pushed onto the transmit list to be sent when recvRetry() is
     * called. */
    class SendEvent : public Event
    {
        SimpleTimingPort *port;
        Packet *packet;

      public:
        SendEvent(SimpleTimingPort *p, Packet *pkt, Tick t)
            : Event(&mainEventQueue), port(p), packet(pkt)
        { setFlags(AutoDelete); schedule(curTick + t); }

        virtual void process();

        virtual const char *description()
        { return "Future scheduled sendTiming event"; }
    };


    /** Number of timing requests that are emulating the device timing before
     * attempting to end up on the bus.
     */
    int outTiming;

    /** If we need to drain, keep the drain event around until we're done
     * here.*/
    Event *drainEvent;

    /** Schedule a sendTiming() event to be called in the future. */
    void sendTiming(Packet *pkt, Tick time)
    { outTiming++; new SimpleTimingPort::SendEvent(this, pkt, time); }

    /** This function is notification that the device should attempt to send a
     * packet again. */
    virtual void recvRetry();

    void resendNacked(Packet *pkt);
  public:

    SimpleTimingPort(std::string pname)
        : Port(pname), outTiming(0), drainEvent(NULL)
    {}

    unsigned int drain(Event *de);
};

#endif // __MEM_TPORT_HH__
