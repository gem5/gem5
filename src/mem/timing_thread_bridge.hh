/*
 * Copyright (c) 2026 ZhangJian
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

#ifndef __MEM_TIMING_THREAD_BRIDGE_HH__
#define __MEM_TIMING_THREAD_BRIDGE_HH__

#include <atomic>
#include <deque>
#include <functional>

#include "base/types.hh"
#include "mem/port.hh"
#include "params/TimingThreadBridge.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class TimingThreadBridge : public ClockedObject
{
  protected:
    class BridgeResponsePort : public ResponsePort
    {
      private:
        TimingThreadBridge &bridge;
        const AddrRangeList ranges;

      public:
        BridgeResponsePort(const std::string &name, TimingThreadBridge &b,
                           std::vector<AddrRange> _ranges)
            : ResponsePort(name), bridge(b),
              ranges(_ranges.begin(), _ranges.end())
        {
        }

      protected:
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        AddrRangeList getAddrRanges() const override;
    };

    class BridgeRequestPort : public RequestPort
    {
      private:
        TimingThreadBridge &bridge;

      public:
        BridgeRequestPort(const std::string &name, TimingThreadBridge &b)
            : RequestPort(name), bridge(b)
        {
        }

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
    };

    BridgeResponsePort inPort;
    BridgeRequestPort outPort;

    const Cycles delay;
    const int peerQueueIdx;

    std::deque<PacketPtr> reqQueue;
    std::atomic<int> reqFreeSlots;

    std::deque<PacketPtr> respQueue;
    int respFreeSlots;
    bool retryReq;

    std::atomic<bool> reqNotifyPending;

    void scheduleOnPeer(std::function<void()> fn, Tick when);
    void enqueueReq(PacketPtr pkt);
    void enqueueResp(PacketPtr pkt);
    void trySendReq();
    void trySendResp();
    void notifyReqSpace();
    void drainReqNotify();

  public:
    typedef TimingThreadBridgeParams Params;

    TimingThreadBridge(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    void init() override;
};

} // namespace gem5

#endif // __MEM_TIMING_THREAD_BRIDGE_HH__
