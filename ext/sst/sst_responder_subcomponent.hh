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

#ifndef __SST_RESPONDER_SUBCOMPONENT_HH__
#define __SST_RESPONDER_SUBCOMPONENT_HH__

#define TRACING_ON 0

#include <string>
#include <vector>
#include <unordered_map>
#include <queue>

#include <sst/core/sst_config.h>
#include <sst/core/component.h>

#include <sst/core/simulation.h>
#include <sst/core/interfaces/stringEvent.h>
#include <sst/core/interfaces/simpleMem.h>

#include <sst/core/eli/elementinfo.h>
#include <sst/core/link.h>

// from gem5
#include <sim/sim_object.hh>
#include <sst/outgoing_request_bridge.hh>
#include <sim/root.hh>
#include <sst/sst_responder_interface.hh>

#include "translator.hh"
#include "sst_responder.hh"

class SSTResponderSubComponent: public SST::SubComponent
{
  private:
    gem5::OutgoingRequestBridge* responseReceiver;
    gem5::SSTResponderInterface* sstResponder;

    SST::Interfaces::SimpleMem* memoryInterface;
    SST::TimeConverter* timeConverter;
    SST::Output* output;
    std::queue<gem5::PacketPtr> responseQueue;

    std::vector<SST::Interfaces::SimpleMem::Request*> initRequests;

    std::string gem5SimObjectName;
    std::string memSize;

  public:
    SSTResponderSubComponent(SST::ComponentId_t id, SST::Params& params);
    ~SSTResponderSubComponent();

    void init(unsigned phase);
    void setTimeConverter(SST::TimeConverter* tc);
    void setOutputStream(SST::Output* output_);

    void setResponseReceiver(gem5::OutgoingRequestBridge* gem5_bridge);
    void portEventHandler(SST::Interfaces::SimpleMem::Request* request);

    bool blocked();
    void setup();

    // return true if the SimObject could be found
    bool findCorrespondingSimObject(gem5::Root* gem5_root);

    bool handleTimingReq(SST::Interfaces::SimpleMem::Request* request);
    void handleRecvRespRetry();
    void handleRecvFunctional(gem5::PacketPtr pkt);
    void handleSwapReqResponse(SST::Interfaces::SimpleMem::Request* request);

    TPacketMap sstRequestIdToPacketMap;

  public: // register the component to SST
    SST_ELI_REGISTER_SUBCOMPONENT_API(SSTResponderSubComponent);
    SST_ELI_REGISTER_SUBCOMPONENT_DERIVED(
        SSTResponderSubComponent,
        "gem5", // SST will look for libgem5.so
        "gem5Bridge",
        SST_ELI_ELEMENT_VERSION(1, 0, 0),
        "Initialize gem5 and link SST's ports to gem5's ports",
        SSTResponderSubComponent
    )

    SST_ELI_DOCUMENT_SUBCOMPONENT_SLOTS(
        {"memory", "Interface to the memory subsystem", \
         "SST::Interfaces::SimpleMem"}
    )

    SST_ELI_DOCUMENT_PORTS(
        {"port", "Handling mem events", {"memHierarchy.MemEvent", ""}}
    )

    SST_ELI_DOCUMENT_PARAMS(
        {"response_receiver_name", \
         "Name of the SimObject receiving the responses"}
    )

};

#endif // __SST_RESPONDER_SUBCOMPONENT_HH__
