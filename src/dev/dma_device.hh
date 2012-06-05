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

#ifndef __DEV_DMA_DEVICE_HH__
#define __DEV_DMA_DEVICE_HH__

#include "dev/io_device.hh"
#include "params/DmaDevice.hh"

class DmaPort : public MasterPort
{
  protected:
    struct DmaReqState : public Packet::SenderState
    {
        /** Event to call on the device when this transaction (all packets)
         * complete. */
        Event *completionEvent;

        /** Total number of bytes that this transaction involves. */
        Addr totBytes;

        /** Number of bytes that have been acked for this transaction. */
        Addr numBytes;

        /** Amount to delay completion of dma by */
        Tick delay;

        DmaReqState(Event *ce, Addr tb, Tick _delay)
            : completionEvent(ce), totBytes(tb), numBytes(0), delay(_delay)
        {}
    };

    MemObject *device;
    std::list<PacketPtr> transmitList;

    /** The system that device/port are in. This is used to select which mode
     * we are currently operating in. */
    System *sys;

    /** Id for all requests */
    MasterID masterId;

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

    virtual bool recvTimingResp(PacketPtr pkt);

    virtual void recvRetry() ;

    void queueDma(PacketPtr pkt, bool front = false);
    void sendDma();

    /** event to give us a kick every time we backoff time is reached. */
    EventWrapper<DmaPort, &DmaPort::sendDma> backoffEvent;

  public:
    DmaPort(MemObject *dev, System *s, Tick min_backoff, Tick max_backoff);

    void dmaAction(Packet::Command cmd, Addr addr, int size, Event *event,
                   uint8_t *data, Tick delay, Request::Flags flag = 0);

    bool dmaPending() { return pendingCount > 0; }

    unsigned cacheBlockSize() const { return peerBlockSize(); }
    unsigned int drain(Event *de);
};

class DmaDevice : public PioDevice
{
   protected:
    DmaPort dmaPort;

  public:
    typedef DmaDeviceParams Params;
    DmaDevice(const Params *p);
    virtual ~DmaDevice();

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    void dmaWrite(Addr addr, int size, Event *event, uint8_t *data,
                  Tick delay = 0)
    {
        dmaPort.dmaAction(MemCmd::WriteReq, addr, size, event, data, delay);
    }

    void dmaRead(Addr addr, int size, Event *event, uint8_t *data,
                 Tick delay = 0)
    {
        dmaPort.dmaAction(MemCmd::ReadReq, addr, size, event, data, delay);
    }

    bool dmaPending() { return dmaPort.dmaPending(); }

    virtual void init();

    virtual unsigned int drain(Event *de);

    unsigned cacheBlockSize() const { return dmaPort.cacheBlockSize(); }

    virtual MasterPort &getMasterPort(const std::string &if_name,
                                      int idx = -1);

    friend class DmaPort;
};

#endif // __DEV_DMA_DEVICE_HH__
