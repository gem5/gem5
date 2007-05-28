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

#ifndef __MEM_TPORT_HH__
#define __MEM_TPORT_HH__

/**
 * @file
 *
 * Declaration of SimpleTimingPort.
 */

#include "mem/port.hh"
#include "sim/eventq.hh"
#include <list>
#include <string>

/**
 * A simple port for interfacing objects that basically have only
 * functional memory behavior (e.g. I/O devices) to the memory system.
 * Both timing and functional accesses are implemented in terms of
 * atomic accesses.  A derived port class thus only needs to provide
 * recvAtomic() to support all memory access modes.
 *
 * The tricky part is handling recvTiming(), where the response must
 * be scheduled separately via a later call to sendTiming().  This
 * feature is handled by scheduling an internal event that calls
 * sendTiming() after a delay, and optionally rescheduling the
 * response if it is nacked.
 */
class SimpleTimingPort : public Port
{
  protected:
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
    typedef std::list<DeferredPacket>::iterator DeferredPacketIterator;

    /** A list of outgoing timing response packets that haven't been
     * serviced yet. */
    DeferredPacketList transmitList;

    /** This function attempts to send deferred packets.  Scheduled to
     * be called in the future via SendEvent. */
    void processSendEvent();

    /**
     * This class is used to implemented sendTiming() with a delay. When
     * a delay is requested a the event is scheduled if it isn't already.
     * When the event time expires it attempts to send the packet.
     * If it cannot, the packet sent when recvRetry() is called.
     **/
    typedef EventWrapper<SimpleTimingPort, &SimpleTimingPort::processSendEvent>
            SendEvent;

    Event *sendEvent;

    /** If we need to drain, keep the drain event around until we're done
     * here.*/
    Event *drainEvent;

    /** Check the list of buffered packets against the supplied
     * functional request. */
    void checkFunctional(PacketPtr funcPkt);

    /** Schedule a sendTiming() event to be called in the future.
     * @param pkt packet to send
     * @param time increment from now (in ticks) to send packet
     */
    void sendTiming(PacketPtr pkt, Tick time);

    /** This function is notification that the device should attempt to send a
     * packet again. */
    virtual void recvRetry();

    /** Implemented using recvAtomic(). */
    void recvFunctional(PacketPtr pkt);

    /** Implemented using recvAtomic(). */
    bool recvTiming(PacketPtr pkt);

    /**
     * Simple ports generally don't care about any status
     * changes... can always override this in cases where that's not
     * true. */
    virtual void recvStatusChange(Status status) { }


  public:

    SimpleTimingPort(std::string pname, MemObject *_owner = NULL)
        : Port(pname, _owner),
          sendEvent(new SendEvent(this)),
          drainEvent(NULL)
    {}

    ~SimpleTimingPort() { delete sendEvent; }

    /** Hook for draining timing accesses from the system.  The
     * associated SimObject's drain() functions should be implemented
     * something like this when this class is used:
     \code
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
     \endcode
    */
    unsigned int drain(Event *de);
};

#endif // __MEM_TPORT_HH__
