/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2008 The Regents of The University of Michigan
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

#ifndef __DEV_X86_INTDEV_HH__
#define __DEV_X86_INTDEV_HH__

#include <cassert>
#include <functional>
#include <string>

#include "base/cast.hh"
#include "mem/tport.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace X86ISA
{

template <class Device>
class IntResponsePort : public SimpleTimingPort
{
    Device *device;

  public:
    IntResponsePort(const std::string &_name, SimObject *_parent, Device *dev)
        : SimpleTimingPort(_name, _parent), device(dev)
    {}

    AddrRangeList
    getAddrRanges() const
    {
        return device->getIntAddrRange();
    }

    Tick
    recvAtomic(PacketPtr pkt)
    {
        panic_if(pkt->cmd != MemCmd::WriteReq,
                 "%s received unexpected command %s from %s.\n", name(),
                 pkt->cmd.toString(), getPeer());
        pkt->headerDelay = pkt->payloadDelay = 0;
        return device->recvMessage(pkt);
    }
};

template <class T>
PacketPtr
buildIntPacket(Addr addr, T payload)
{
    RequestPtr req = std::make_shared<Request>(
        addr, sizeof(T), Request::UNCACHEABLE, Request::intRequestorId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setRaw<T>(payload);
    return pkt;
}

template <class Device>
class IntRequestPort : public QueuedRequestPort
{
  private:
    ReqPacketQueue reqQueue;
    SnoopRespPacketQueue snoopRespQueue;

    Device *device;
    Tick latency;

    typedef std::function<void(PacketPtr)> OnCompletionFunc;

    struct OnCompletion : public Packet::SenderState
    {
        OnCompletionFunc func;

        OnCompletion(OnCompletionFunc _func) : func(_func) {}
    };

    // If nothing extra needs to happen, just clean up the packet.
    static void
    defaultOnCompletion(PacketPtr pkt)
    {
        delete pkt;
    }

  public:
    IntRequestPort(const std::string &_name, SimObject *_parent, Device *dev,
                   Tick _latency)
        : QueuedRequestPort(_name, reqQueue, snoopRespQueue),
          reqQueue(*_parent, *this),
          snoopRespQueue(*_parent, *this),
          device(dev),
          latency(_latency)
    {}

    bool
    recvTimingResp(PacketPtr pkt) override
    {
        assert(pkt->isResponse());
        auto *oc = safe_cast<OnCompletion *>(pkt->popSenderState());
        oc->func(pkt);
        delete oc;
        return true;
    }

    void
    sendMessage(PacketPtr pkt, bool timing,
                OnCompletionFunc func = defaultOnCompletion)
    {
        if (timing) {
            pkt->pushSenderState(new OnCompletion(func));
            schedTimingReq(pkt, curTick() + latency);
            // The target handles cleaning up the packet in timing mode.
        } else {
            // ignore the latency involved in the atomic transaction
            sendAtomic(pkt);
            func(pkt);
        }
    }
};

} // namespace X86ISA
} // namespace gem5

#endif //__DEV_X86_INTDEV_HH__
