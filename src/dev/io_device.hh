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

#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/tport.hh"
#include "params/BasicPioDevice.hh"
#include "params/DmaDevice.hh"
#include "params/PioDevice.hh"
#include "sim/sim_object.hh"

class Event;
class Platform;
class PioDevice;
class DmaDevice;
class System;

/**
 * The PioPort class is a programmed i/o port that all devices that are
 * sensitive to an address range use. The port takes all the memory
 * access types and roles them into one read() and write() call that the device
 * must respond to. The device must also provide the addressRanges() function
 * with which it returns the address ranges it is interested in.
 */
class PioPort : public SimpleTimingPort
{
  protected:
    /** The device that this port serves. */
    PioDevice *device;

    virtual Tick recvAtomic(PacketPtr pkt);

    virtual void getDeviceAddressRanges(AddrRangeList &resp,
                                        bool &snoop);

  public:

    PioPort(PioDevice *dev, System *s, std::string pname = "-pioport");
};


class DmaPort : public Port
{
  protected:
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

        DmaReqState(Event *ce, Port *p, Addr tb)
            : completionEvent(ce), outPort(p), totBytes(tb), numBytes(0)
        {}
    };

    DmaDevice *device;
    std::list<PacketPtr> transmitList;

    /** The system that device/port are in. This is used to select which mode
     * we are currently operating in. */
    System *sys;

    /** Number of outstanding packets the dma port has. */
    int pendingCount;

    /** If a dmaAction is in progress. */
    int actionInProgress;

    /** If we need to drain, keep the drain event around until we're done
     * here.*/
    Event *drainEvent;

    /** time to wait between sending another packet, increases as NACKs are
     * recived, decreases as responses are recived. */
    Tick backoffTime;

    /** If the port is currently waiting for a retry before it can send whatever
     * it is that it's sending. */
    bool inRetry;

    virtual bool recvTiming(PacketPtr pkt);
    virtual Tick recvAtomic(PacketPtr pkt)
    { panic("dma port shouldn't be used for pio access."); M5_DUMMY_RETURN }
    virtual void recvFunctional(PacketPtr pkt)
    { panic("dma port shouldn't be used for pio access."); }

    virtual void recvStatusChange(Status status)
    { ; }

    virtual void recvRetry() ;

    virtual void getDeviceAddressRanges(AddrRangeList &resp,
                                        bool &snoop)
    { resp.clear(); snoop = false; }

    void queueDma(PacketPtr pkt, bool front = false);
    void sendDma();

    /** event to give us a kick every time we backoff time is reached. */
    EventWrapper<DmaPort, &DmaPort::sendDma> backoffEvent;

  public:
    DmaPort(DmaDevice *dev, System *s);

    void dmaAction(Packet::Command cmd, Addr addr, int size, Event *event,
                   uint8_t *data = NULL);

    bool dmaPending() { return pendingCount > 0; }

    int cacheBlockSize() { return peerBlockSize(); }
    unsigned int drain(Event *de);
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

    System *sys;

    /** The pioPort that handles the requests for us and provides us requests
     * that it sees. */
    PioPort *pioPort;

    virtual void addressRanges(AddrRangeList &range_list) = 0;

    /** Pure virtual function that the device must implement. Called
     * when a read command is recieved by the port.
     * @param pkt Packet describing this request
     * @return number of ticks it took to complete
     */
    virtual Tick read(PacketPtr pkt) = 0;

    /** Pure virtual function that the device must implement. Called when a
     * write command is recieved by the port.
     * @param pkt Packet describing this request
     * @return number of ticks it took to complete
     */
    virtual Tick write(PacketPtr pkt) = 0;

  public:
    typedef PioDeviceParams Params;
    PioDevice(const Params *p);
    virtual ~PioDevice();

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    virtual void init();

    virtual unsigned int drain(Event *de);

    virtual Port *getPort(const std::string &if_name, int idx = -1)
    {
        if (if_name == "pio") {
            if (pioPort != NULL)
                panic("pio port already connected to.");
            pioPort = new PioPort(this, sys);
            return pioPort;
        } else
            return NULL;
    }
    friend class PioPort;

};

class BasicPioDevice : public PioDevice
{
  protected:
    /** Address that the device listens to. */
    Addr pioAddr;

    /** Size that the device's address range. */
    Addr pioSize;

    /** Delay that the device experinces on an access. */
    Tick pioDelay;

  public:
    typedef BasicPioDeviceParams Params;
    BasicPioDevice(const Params *p);

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    /** return the address ranges that this device responds to.
     * @param range_list range list to populate with ranges
     */
    void addressRanges(AddrRangeList &range_list);

};

class DmaDevice : public PioDevice
{
   protected:
    DmaPort *dmaPort;
    Tick minBackoffDelay;
    Tick maxBackoffDelay;

  public:
    typedef DmaDeviceParams Params;
    DmaDevice(const Params *p);
    virtual ~DmaDevice();

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    void dmaWrite(Addr addr, int size, Event *event, uint8_t *data)
    {
        dmaPort->dmaAction(MemCmd::WriteInvalidateReq,
                           addr, size, event, data);
    }

    void dmaRead(Addr addr, int size, Event *event, uint8_t *data)
    {
        dmaPort->dmaAction(MemCmd::ReadReq, addr, size, event, data);
    }

    bool dmaPending() { return dmaPort->dmaPending(); }

    virtual unsigned int drain(Event *de);

    int cacheBlockSize() { return dmaPort->cacheBlockSize(); }

    virtual Port *getPort(const std::string &if_name, int idx = -1)
    {
        if (if_name == "pio") {
            if (pioPort != NULL)
                panic("pio port already connected to.");
            pioPort = new PioPort(this, sys);
            return pioPort;
        } else if (if_name == "dma") {
            if (dmaPort != NULL)
                panic("dma port already connected to.");
            dmaPort = new DmaPort(this, sys);
            return dmaPort;
        } else
            return NULL;
    }

    friend class DmaPort;
};


#endif // __DEV_IO_DEVICE_HH__
