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
 *
 * Authors: Ali Saidi
 *          Nathan Binkert
 */

#ifndef __DEV_IO_DEVICE_HH__
#define __DEV_IO_DEVICE_HH__

#include "base/chunk_generator.hh"
#include "mem/mem_object.hh"
#include "mem/packet_impl.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

class Platform;
class PioDevice;
class DmaDevice;
class System;

/**
 * The PioPort class is a programmed i/o port that all devices that are
 * sensitive to an address range use. The port takes all the memory
 * access types and roles them into one read() and write() call that the device
 * must respond to. The device must also provide the addressRanges() function
 * with which it returns the address ranges it is interested in. An extra
 * sendTiming() function is implemented which takes an delay. In this way the
 * device can immediatly call sendTiming(pkt, time) after processing a request
 * and the request will be handled by the port even if the port bus the device
 * connects to is blocked.
 */
class PioPort : public Port
{
  protected:
    /** The device that this port serves. */
    PioDevice *device;

    /** The platform that device/port are in. This is used to select which mode
     * we are currently operating in. */
    Platform *platform;

    /** A list of outgoing timing response packets that haven't been serviced
     * yet. */
    std::list<Packet*> transmitList;

    /** The current status of the peer(bus) that we are connected to. */
    Status peerStatus;

    virtual bool recvTiming(Packet *pkt);

    virtual Tick recvAtomic(Packet *pkt);

    virtual void recvFunctional(Packet *pkt) ;

    virtual void recvStatusChange(Status status)
    { peerStatus = status; }

    virtual void getDeviceAddressRanges(AddrRangeList &resp, AddrRangeList &snoop);

    /**
     * This class is used to implemented sendTiming() with a delay. When a delay
     * is requested a new event is created. When the event time expires it
     * attempts to send the packet. If it cannot, the packet is pushed onto the
     * transmit list to be sent when recvRetry() is called. */
    class SendEvent : public Event
    {
        PioPort *port;
        Packet *packet;

        SendEvent(PioPort *p, Packet *pkt, Tick t)
            : Event(&mainEventQueue), port(p), packet(pkt)
        { schedule(curTick + t); }

        virtual void process();

        virtual const char *description()
        { return "Future scheduled sendTiming event"; }

        friend class PioPort;
    };

    /** Schedule a sendTiming() event to be called in the future. */
    void sendTiming(Packet *pkt, Tick time)
    { new PioPort::SendEvent(this, pkt, time); }

    /** This function is notification that the device should attempt to send a
     * packet again. */
    virtual void recvRetry();

  public:
    PioPort(PioDevice *dev, Platform *p);

  friend class PioPort::SendEvent;
};


struct DmaReqState : public Packet::SenderState
{
    /** Event to call on the device when this transaction (all packets)
     * complete. */
    Event *completionEvent;

    /** Where we came from for some sanity checking. */
    Port *outPort;

    /** Total number of bytes that this transaction involves. */
    Addr totBytes;

    /** Number of bytes that have been acked for this transaction. */
    Addr numBytes;

    bool final;
    DmaReqState(Event *ce, Port *p, Addr tb)
        : completionEvent(ce), outPort(p), totBytes(tb), numBytes(0)
    {}
};

class DmaPort : public Port
{
  protected:
    DmaDevice *device;
    std::list<Packet*> transmitList;

    /** The platform that device/port are in. This is used to select which mode
     * we are currently operating in. */
    Platform *platform;

    /** Number of outstanding packets the dma port has. */
    int pendingCount;

    virtual bool recvTiming(Packet *pkt);
    virtual Tick recvAtomic(Packet *pkt)
    { panic("dma port shouldn't be used for pio access."); }
    virtual void recvFunctional(Packet *pkt)
    { panic("dma port shouldn't be used for pio access."); }

    virtual void recvStatusChange(Status status)
    { ; }

    virtual void recvRetry() ;

    virtual void getDeviceAddressRanges(AddrRangeList &resp, AddrRangeList &snoop)
    { resp.clear(); snoop.clear(); }

    void sendDma(Packet *pkt, bool front = false);

  public:
    DmaPort(DmaDevice *dev, Platform *p);

    void dmaAction(Packet::Command cmd, Addr addr, int size, Event *event,
                   uint8_t *data = NULL);

    bool dmaPending() { return pendingCount > 0; }

};

/**
 * This device is the base class which all devices senstive to an address range
 * inherit from. There are three pure virtual functions which all devices must
 * implement addressRanges(), read(), and write(). The magic do choose which
 * mode we are in, etc is handled by the PioPort so the device doesn't have to
 * bother.
 */

class PioDevice : public MemObject
{
  protected:

    /** The platform we are in. This is used to decide what type of memory
     * transaction we should perform. */
    Platform *platform;

    /** The pioPort that handles the requests for us and provides us requests
     * that it sees. */
    PioPort *pioPort;

    virtual void addressRanges(AddrRangeList &range_list) = 0;

    /** As far as the devices are concerned they only accept atomic transactions
     * which are converted to either a write or a read. */
    Tick recvAtomic(Packet *pkt)
    { return pkt->isRead() ? this->read(pkt) : this->write(pkt); }

    /** Pure virtual function that the device must implement. Called when a read
     * command is recieved by the port.
     * @param pkt Packet describing this request
     * @return number of ticks it took to complete
     */
    virtual Tick read(Packet *pkt) = 0;

    /** Pure virtual function that the device must implement. Called when a
     * write command is recieved by the port.
     * @param pkt Packet describing this request
     * @return number of ticks it took to complete
     */
    virtual Tick write(Packet *pkt) = 0;

  public:
    /** Params struct which is extended through each device based on the
     * parameters it needs. Since we are re-writing everything, we might as well
     * start from the bottom this time. */

    struct Params
    {
        std::string name;
        Platform *platform;
        System *system;
    };

  protected:
    Params *_params;

  public:
    const Params *params() const { return _params; }

    PioDevice(Params *p)
              : MemObject(p->name),  platform(p->platform), pioPort(NULL),
                _params(p)
              {}

    virtual ~PioDevice();

    virtual void init();

    virtual Port *getPort(const std::string &if_name, int idx = -1)
    {
        if (if_name == "pio") {
            if (pioPort != NULL)
                panic("pio port already connected to.");
            pioPort = new PioPort(this, params()->platform);
            return pioPort;
        } else
            return NULL;
    }
    friend class PioPort;

};

class BasicPioDevice : public PioDevice
{
  public:
    struct Params :  public PioDevice::Params
    {
        Addr pio_addr;
        Tick pio_delay;
    };

  protected:
    /** Address that the device listens to. */
    Addr pioAddr;

    /** Size that the device's address range. */
    Addr pioSize;

    /** Delay that the device experinces on an access. */
    Tick pioDelay;

  public:
    BasicPioDevice(Params *p)
        : PioDevice(p), pioAddr(p->pio_addr), pioSize(0), pioDelay(p->pio_delay)
    {}

    /** return the address ranges that this device responds to.
     * @params range_list range list to populate with ranges
     */
    void addressRanges(AddrRangeList &range_list);

};

class DmaDevice : public PioDevice
{
  protected:
    DmaPort *dmaPort;

  public:
    DmaDevice(Params *p);
    virtual ~DmaDevice();

    void dmaWrite(Addr addr, int size, Event *event, uint8_t *data)
    { dmaPort->dmaAction(Packet::WriteReq, addr, size, event, data) ; }

    void dmaRead(Addr addr, int size, Event *event, uint8_t *data = NULL)
    { dmaPort->dmaAction(Packet::ReadReq, addr, size, event, data); }

    bool dmaPending() { return dmaPort->dmaPending(); }

    virtual Port *getPort(const std::string &if_name, int idx = -1)
    {
        if (if_name == "pio") {
            if (pioPort != NULL)
                panic("pio port already connected to.");
            pioPort = new PioPort(this, params()->platform);
            return pioPort;
        } else if (if_name == "dma") {
            if (dmaPort != NULL)
                panic("dma port already connected to.");
            dmaPort = new DmaPort(this, params()->platform);
            return dmaPort;
        } else
            return NULL;
    }

    friend class DmaPort;
};


#endif // __DEV_IO_DEVICE_HH__
