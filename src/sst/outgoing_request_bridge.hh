// Copyright (c) 2021 The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __SST_OUTGOING_REQUEST_BRIDGE_HH__
#define __SST_OUTGOING_REQUEST_BRIDGE_HH__

#include <utility>
#include <vector>

#include "mem/port.hh"
#include "params/OutgoingRequestBridge.hh"
#include "sim/sim_object.hh"
#include "sst/sst_responder_interface.hh"

/**
 * -  OutgoingRequestBridge acts as a SimObject owning pointers to both a gem5
 * OutgoingRequestPort and an SST port (via SSTResponderInterface). This bridge
 * will forward gem5 packets from the gem5 port to the SST interface. Responses
 * from SST will be handle by OutgoingRequestPort itself. Note: the bridge
 * should be decoupled from the SST libraries so that it'll be
 * SST-version-independent. Thus, there's no translation between a gem5 packet
 * and SST Response here.
 *
 *  - OutgoingRequestPort is a specialized ResponsePort working with
 * OutgoingRequestBridge.
 */

namespace gem5
{

class OutgoingRequestBridge: public SimObject
{
  public:
    class OutgoingRequestPort: public ResponsePort
    {
      private:
        OutgoingRequestBridge* owner;
      public:
        OutgoingRequestPort(const std::string &name_,
                            OutgoingRequestBridge* owner_);
        ~OutgoingRequestPort();
        Tick recvAtomic(PacketPtr pkt);
        void recvFunctional(PacketPtr pkt);
        bool recvTimingReq(PacketPtr pkt);
        void recvRespRetry();
        AddrRangeList getAddrRanges() const;
    };

  public:
    // a gem5 ResponsePort
    OutgoingRequestPort outgoingPort;
    // pointer to the corresponding SST responder
    SSTResponderInterface* sstResponder;
    // this vector holds the initialization data sent by gem5
    std::vector<std::pair<Addr, std::vector<uint8_t>>> initData;

    AddrRangeList physicalAddressRanges;

  public:
    OutgoingRequestBridge(const OutgoingRequestBridgeParams &params);
    ~OutgoingRequestBridge();

    // Required to let the OutgoingRequestPort to send range change request.
    void init();

    // Returns the range of addresses that the ports will handle.
    // Currently, it will return the range of [0x80000000, inf), which is
    // specific to RISCV (SiFive's HiFive boards).
    AddrRangeList getAddrRanges() const;

    // Required to return a port during gem5 instantiate phase.
    Port & getPort(const std::string &if_name, PortID idx);

    // Returns the buffered data for initialization. This is necessary as
    // when gem5 sends functional requests to memory for initialization,
    // the connection in SST Memory Hierarchy has not been constructed yet.
    std::vector<std::pair<Addr, std::vector<uint8_t>>> getInitData() const;

    // gem5 Component (from SST) will call this function to let set the
    // bridge's corresponding SSTResponderSubComponent (which implemented
    // SSTResponderInterface). I.e., this will connect this bridge to the
    // corresponding port in SST.
    void setResponder(SSTResponderInterface* responder);

    // This function is called when SST wants to sent a timing response to gem5
    bool sendTimingResp(PacketPtr pkt);

    // This function is called when SST sends response having an invalidate .
    void sendTimingSnoopReq(PacketPtr pkt);

    // This function is called when gem5 wants to send a non-timing request
    // to SST. Should only be called during the SST construction phase, i.e.
    // not at the simulation time.
    void handleRecvFunctional(PacketPtr pkt);
};

}; // namespace gem5

#endif //__SST_OUTGOING_REQUEST_BRIDGE_HH__
