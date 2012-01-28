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

#include "base/fast_alloc.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/tport.hh"
#include "params/BasicPioDevice.hh"
#include "params/DmaDevice.hh"
#include "params/PioDevice.hh"
#include "sim/sim_object.hh"

class Event;
class PioDevice;
class DmaDevice;
class System;

/**
 * The PioPort class is a programmed i/o port that all devices that are
 * sensitive to an address range use. The port takes all the memory
 * access types and roles them into one read() and write() call that the device
 * must respond to. The device must also provide getAddrRanges() function
 * with which it returns the address ranges it is interested in.
 */
class PioPort : public SimpleTimingPort
{
  protected:
    /** The device that this port serves. */
    PioDevice *device;

    virtual Tick recvAtomic(PacketPtr pkt);

    virtual AddrRangeList getAddrRanges();

  public:

    PioPort(PioDevice *dev, System *s, std::string pname = "-pioport");
};


class DmaPort : public Port
{
  protected:
    struct DmaReqState : public Packet::SenderState, public FastAlloc
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

        /** Amount to delay completion of dma by */
        Tick delay;


        DmaReqState(Event *ce, Port *p, Addr tb, Tick _delay)
            : completionEvent(ce), outPort(p), totBytes(tb), numBytes(0),
              delay(_delay)
        {}
    };

    MemObject *device;
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

    /** Minimum time that device should back off for after failed sendTiming */
    Tick minBackoffDelay;

    /** Maximum time that device should back off for after failed sendTiming */
    Tick maxBackoffDelay;

    /** If the port is currently waiting for a retry before it can send whatever
     * it is that it's sending. */
    bool inRetry;

    /** Port accesses a cache which requires snooping */
    bool recvSnoops;

    virtual bool recvTiming(PacketPtr pkt);
    virtual Tick recvAtomic(PacketPtr pkt)
    {
        if (recvSnoops) return 0;

        panic("dma port shouldn't be used for pio access."); M5_DUMMY_RETURN
    }
    virtual void recvFunctional(PacketPtr pkt)
    {
        if (recvSnoops) return;

        panic("dma port shouldn't be used for pio access.");
    }

    virtual void recvRangeChange()
    {
        // DMA port is a master with a single slave so there is no choice and
        // thus no need to worry about any address changes
    }

    virtual void recvRetry() ;

    virtual bool isSnooping()
    { return recvSnoops; }

    void queueDma(PacketPtr pkt, bool front = false);
    void sendDma();

    /** event to give us a kick every time we backoff time is reached. */
    EventWrapper<DmaPort, &DmaPort::sendDma> backoffEvent;

  public:
    DmaPort(MemObject *dev, System *s, Tick min_backoff, Tick max_backoff,
            bool recv_snoops = false);

    void dmaAction(Packet::Command cmd, Addr addr, int size, Event *event,
                   uint8_t *data, Tick delay, Request::Flags flag = 0);

    bool dmaPending() { return pendingCount > 0; }

    unsigned cacheBlockSize() const { return peerBlockSize(); }
    unsigned int drain(Event *de);
};

/**
 * This device is the base class which all devices senstive to an address range
 * inherit from. There are three pure virtual functions which all devices must
 * implement getAddrRanges(), read(), and write(). The magic do choose which
 * mode we are in, etc is handled by the PioPort so the device doesn't have to
 * bother.
 */
class PioDevice : public MemObject
{
  protected:
    System *sys;

    /** The pioPort that handles the requests for us and provides us requests
     * that it sees. */
    PioPort *pioPort;

    /**
     * Every PIO device is obliged to provide an implementation that
     * returns the address ranges the device responds to.
     *
     * @return a list of non-overlapping address ranges
     */
    virtual AddrRangeList getAddrRanges() = 0;

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

    virtual Port *getPort(const std::string &if_name, int idx = -1);

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

    /**
     * Determine the address ranges that this device responds to.
     *
     * @return a list of non-overlapping address ranges
     */
    virtual AddrRangeList getAddrRanges();

};

class DmaDevice : public PioDevice
{
   protected:
    DmaPort *dmaPort;

  public:
    typedef DmaDeviceParams Params;
    DmaDevice(const Params *p);
    virtual ~DmaDevice();

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    void dmaWrite(Addr addr, int size, Event *event, uint8_t *data, Tick delay = 0)
    {
        dmaPort->dmaAction(MemCmd::WriteReq, addr, size, event, data, delay);
    }

    void dmaRead(Addr addr, int size, Event *event, uint8_t *data, Tick delay = 0)
    {
        dmaPort->dmaAction(MemCmd::ReadReq, addr, size, event, data, delay);
    }

    bool dmaPending() { return dmaPort->dmaPending(); }

    virtual unsigned int drain(Event *de);

    unsigned cacheBlockSize() const { return dmaPort->cacheBlockSize(); }

    virtual Port *getPort(const std::string &if_name, int idx = -1);

    friend class DmaPort;
};


#endif // __DEV_IO_DEVICE_HH__
