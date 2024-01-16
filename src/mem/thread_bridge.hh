/*
 * Copyright 2022 Google, LLC.
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

#ifndef __MEM_THREAD_BRIDGE_HH__
#define __MEM_THREAD_BRIDGE_HH__

#include "mem/port.hh"
#include "params/ThreadBridge.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ThreadBridge : public SimObject
{
  public:
    explicit ThreadBridge(const ThreadBridgeParams &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  private:
    class IncomingPort : public ResponsePort
    {
      public:
        IncomingPort(const std::string &name, ThreadBridge &device);
        AddrRangeList getAddrRanges() const override;

        // TimingResponseProtocol
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;

        // AtomicResponseProtocol
        Tick recvAtomicBackdoor(PacketPtr pkt,
                                MemBackdoorPtr &backdoor) override;
        Tick recvAtomic(PacketPtr pkt) override;

        // FunctionalResponseProtocol
        void recvFunctional(PacketPtr pkt) override;

      private:
        ThreadBridge &device_;
    };

    class OutgoingPort : public RequestPort
    {
      public:
        OutgoingPort(const std::string &name, ThreadBridge &device);
        void recvRangeChange() override;

        // TimingRequestProtocol
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;

      private:
        ThreadBridge &device_;
    };

    IncomingPort in_port_;
    OutgoingPort out_port_;
};

}  // namespace gem5
#endif  // __MEM_THREAD_BRIDGE_HH__
