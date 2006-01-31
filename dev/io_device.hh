/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __DEV_IO_DEVICE_HH__
#define __DEV_IO_DEVICE_HH__

#include "base/chunk_generator.hh"
#include "mem/port.hh"
#include "sim/eventq.hh"

class Bus;
class Platform;
class PioDevice;

class PioPort : public Port
{
  protected:
    PioDevice *device;

    virtual bool recvTiming(Packet &pkt);

    virtual Tick recvAtomic(Packet &pkt)
    { return device->recvAtomic(pkt) };

    virtual void recvFunctional(Packet &pkt)
    { device->recvAtomic(pkt) };

    virtual void recvAddressRangeQuery(std::list<Range<Addr> > &range_list,
                                                 bool &owner)
    { device->addressRanges(range_list, owner); }

    void sendTiming(Packet &pkt, Tick time)
    { new SendEvent(this, pkt, time); }

    class SendEvent : public Event
    {
        PioPort *port;
        Packet packet;

        SendEvent(PioPort *p, Packet &pkt, Tick t)
            : Event(&mainEventQueue), packet(pkt)
        { schedule(curTick + t); }

        virtual void process();

        virtual const char *description()
        { return "Future scheduled sendTiming event"; }

        friend class PioPort;
    }

  public:
    PioPort(PioDevice *dev)
        : device(dev)
    { }

};

class DmaPort : public Port
{
  protected:
    PioDevice *device;
    std::list<Packet*> transmitList;


    virtual bool recvTiming(Packet &pkt)
    { completionEvent->schedule(curTick+1); completionEvent = NULL; }
    virtual Tick recvAtomic(Packet &pkt)
    { panic("dma port shouldn't be used for pio access."); }
    virtual void recvFunctional(Packet &pkt)
    { panic("dma port shouldn't be used for pio access."); }

    virtual void recvStatusChange(Status status)
    { ; }

    virtual Packet *recvRetry()
    { return transmitList.pop_front();  }

    virtual void recvAddressRangeQuery(std::list<Range<Addr> > &range_list,
                                                 bool &owner)
    { range_list.clear(); owner = true; }

    void dmaAction(Memory::Command cmd, DmaPort port, Addr addr, int size,
              Event *event, uint8_t *data = NULL);

    void sendDma(Packet &pkt);

    virtual Packet *recvRetry()
    { return transmitList.pop_front();  }

    class SendEvent : public Event
    {
        PioPort *port;
        Packet packet;

        SendEvent(PioPort *p, Packet &pkt, Tick t)
            : Event(&mainEventQueue), packet(pkt)
        { schedule(curTick + t); }

        virtual void process();

        virtual const char *description()
        { return "Future scheduled sendTiming event"; }

        friend class PioPort;
    }
  public:
    DmaPort(DmaDevice *dev)
        : device(dev)
    { }


};


class PioDevice : public SimObject
{
  protected:

    Platform *platform;

    PioPort *pioPort;

    virtual void addressRanges(std::list<Range<Addr> > &range_list,
                                   bool &owner) = 0;

    virtual read(Packet &pkt) = 0;

    virtual write(Packet &pkt) = 0;

    Tick recvAtomic(Packet &pkt)
    { return pkt->cmd == Read ? this->read(pkt) : this->write(pkt); }

  public:
    PioDevice(const std::string &name, Platform *p);

    virtual ~PioDevice();

    virtual Port *getPort(std::string if_name)
    {
        if (if_name == "pio")
            return pioPort;
        else
            return NULL;
    }
};

class DmaDevice : public PioDevice
{
  protected:
    DmaPort *dmaPort;

  public:
    DmaDevice(const std::string &name, Platform *p);
    virtual ~DmaDevice();

    virtual Port *getPort(std::string if_name)
    {
        if (if_name == "pio")
            return pioPort;
        else if (if_name = "dma")
            return dmaPort;
        else
            return NULL;
    }
};




#endif // __DEV_IO_DEVICE_HH__
