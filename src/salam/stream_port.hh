/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SALAM_STREAM_PORT_HH__
#define __SALAM_STREAM_PORT_HH__

#include "mem/port.hh"
#include "mem/tport.hh"

using namespace gem5;

/** Forward declaration **/
class StreamRequestPort;

/**
 * StreamResponsePort is a specialization of a SimpleTimingPort meant to enable
 * functionality similar to the request port in the AXI-Stream specification.
 * This serves only as a base class.
 */
class StreamResponsePort : public SimpleTimingPort
{
    friend class StreamRequestPort;

  private:
  protected:
    virtual bool tvalid(PacketPtr pkt) = 0;
    virtual bool tvalid(size_t len, bool isRead) = 0;
    Tick
    recvAtomic(PacketPtr pkt) override
    {
        Tick t = 0;
        return t;
    }
    AddrRangeList
    getAddrRanges() const override
    {
        AddrRangeList range;
        return range;
    }

  public:
    StreamResponsePort(const std::string &name, SimObject *owner)
        : SimpleTimingPort(name, owner)
    {}
};

/**
 * Templated StreamResponsePort that functions similarly to the pio port
 * on PioDevices.
 */
template <class Device> class StreamResponsePortT : public StreamResponsePort
{
    friend class StreamRequestPort;

  private:
    bool isBusy;
    bool retryReq;

    EventFunctionWrapper releaseEvent;

    void
    release()
    {
        assert(isBusy);
        isBusy = false;
        if (retryReq) {
            retryReq = false;
            sendRetryReq();
        }
    }

  protected:
    Device *device;

    virtual bool
    tvalid(PacketPtr pkt)
    {
        return device->tvalid(pkt);
    }
    virtual bool
    tvalid(size_t len, bool isRead)
    {
        return device->tvalid(len, isRead);
    }

    bool
    recvTimingReq(PacketPtr pkt) override
    {
        // we should not get a new request after committing to retry the
        // current one, but unfortunately the CPU violates this rule, so
        // simply ignore it for now
        if (retryReq) {
            return false;
        }
        // if we are busy with a read or write, remember that we have to
        // retry
        if (isBusy) {
            retryReq = true;
            return false;
        }
        // Make sure that the transfer is valid
        if (!tvalid(pkt)) {
            return false;
        }
        // the SimpleTimingPort should not be used anywhere where there is
        // a need to deal with snoop responses and their flow control
        // requirements
        if (pkt->cacheResponding()) {
            panic("SimpleTimingPort should never see packets with the "
                  "cacheResponding flag set\n");
        }

        Tick duration = pkt->getSize() * device->getBandwidth();

        if (duration != 0) {
            device->schedule(releaseEvent, curTick() + duration);
            isBusy = true;
        }

        bool needsResponse = pkt->needsResponse();

        Tick latency = recvAtomic(pkt);

        // turn packet around to go back to requester if response expected
        if (needsResponse) {
            // recvAtomic() should already have turned packet into
            // atomic response
            assert(pkt->isResponse());
            schedTimingResp(pkt, curTick() + latency);
        } else {
            // queue the packet for deletion
            pendingDelete.reset(pkt);
        }
        return true;
    }

    Tick
    recvAtomic(PacketPtr pkt) override
    {
        Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
        pkt->headerDelay = pkt->payloadDelay = 0;
        const Tick delay =
            pkt->isRead() ? device->streamRead(pkt) : device->streamWrite(pkt);
        assert(pkt->isResponse() || pkt->isError());
        return delay + receive_delay;
    }

    AddrRangeList
    getAddrRanges() const override
    {
        return device->getStreamAddrRanges();
    }

  public:
    StreamResponsePortT(Device *dev)
        : StreamResponsePort(dev->name() + ".stream", dev),
          isBusy(false),
          retryReq(false),
          releaseEvent([this] { release(); }, dev->name()),
          device(dev)
    {}
    virtual ~StreamResponsePortT() {}
};

/**
 * A StreamRequestPort is a specialization of a RequestPort, meant to enable
 * functionality similar to the request port in the AXI-Stream specification.
 * A StreamRequestPort is able to check the valid signal on a corresponding
 * StreamResponsePort before initiating a transfer. Otherwise it functions like
 * a standard RequestPort.
 */
class StreamRequestPort : public RequestPort
{
  private:
    StreamResponsePort *_stream_resp;

  protected:
    //
  public:
    StreamRequestPort(const std::string &name, SimObject *_owner,
                      PortID id = InvalidPortID);
    virtual ~StreamRequestPort();

    /**
     * Bind this request port to a response port. This also does the
     * mirror action and binds the response port to the request port.
     * If the response port is a stream response, also binds the tvalid
     * signal.
     */
    void bind(Port &peer) override;

    /**
     * Unbind this request port and the associated response port.
     */
    void unbind() override;

    /**
     * If the response port is a stream response port, then check if it can
     * service a request of size 'len'
     */
    bool
    streamValid(PacketPtr pkt)
    {
        if (_stream_resp) {
            return _stream_resp->tvalid(pkt);
        }
        return true;
    }
    bool
    streamValid(size_t len, bool isRead)
    {
        if (_stream_resp) {
            return _stream_resp->tvalid(len, isRead);
        }
        return true;
    }
};

template <class Device> class StatusPort : public SimpleTimingPort
{
  protected:
    Device *device;
    bool read; // Port reads from the stream

    Tick
    recvAtomic(PacketPtr pkt) override
    {
        // Technically the packet only reaches us after the header delay,
        // and typically we also need to deserialise any payload.
        Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
        pkt->headerDelay = pkt->payloadDelay = 0;

        const Tick delay = device->status(pkt, read);
        assert(pkt->isResponse() || pkt->isError());
        return delay + receive_delay;
    }

    AddrRangeList
    getAddrRanges() const override
    {
        return device->getStatusAddrRanges();
    }

  public:
    StatusPort(Device *dev, bool _read = true)
        : SimpleTimingPort(dev->name() + ".status", dev),
          device(dev),
          read(_read)
    {}
};

#endif //__SALAM_STREAM_PORT_HH__
