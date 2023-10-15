// Copyright (c) 2021-2023 The Regents of the University of California
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

#include "sst_responder_subcomponent.hh"

#include <cassert>
#include <sstream>
#include <iomanip>

#ifdef fatal  // gem5 sets this
#undef fatal
#endif

SSTResponderSubComponent::SSTResponderSubComponent(SST::ComponentId_t id,
                                                   SST::Params& params)
    : SubComponent(id)
{
    sstResponder = new SSTResponder(this);
    gem5SimObjectName = params.find<std::string>("response_receiver_name", "");
    memSize = params.find<std::string>("mem_size", "8GiB");
    if (gem5SimObjectName == "")
        assert(false && "The response_receiver_name must be specified");
}

SSTResponderSubComponent::~SSTResponderSubComponent()
{
    delete sstResponder;
}

void
SSTResponderSubComponent::setTimeConverter(SST::TimeConverter* tc)
{
    timeConverter = tc;

    // Get the memory interface
    SST::Params interface_params;
    // This is how you tell the interface the name of the port it should use
    interface_params.insert("port", "port");
    interface_params.insert("mem_size", memSize.c_str());
    // Loads a “memHierarchy.memInterface” into index 0 of the “memory” slot
    // SHARE_PORTS means the interface can use our port as if it were its own
    // INSERT_STATS means the interface will inherit our statistic
    //   configuration (e.g., if ours are enabled, the interface’s will be too)
    memoryInterface = loadAnonymousSubComponent<SST::Interfaces::StandardMem>(
        "memHierarchy.standardInterface", "memory", 0,
        SST::ComponentInfo::SHARE_PORTS | SST::ComponentInfo::INSERT_STATS,
        interface_params, timeConverter,
        new SST::Interfaces::StandardMem::Handler<SSTResponderSubComponent>(
            this, &SSTResponderSubComponent::portEventHandler)
    );
    assert(memoryInterface != NULL);
}

void
SSTResponderSubComponent::setOutputStream(SST::Output* output_)
{
    output = output_;
}

void
SSTResponderSubComponent::setResponseReceiver(
    gem5::OutgoingRequestBridge* gem5_bridge)
{
    responseReceiver = gem5_bridge;
    responseReceiver->setResponder(sstResponder);
}

bool
SSTResponderSubComponent::handleTimingReq(
    SST::Interfaces::StandardMem::Request* request)
{
    memoryInterface->send(request);
    return true;
}

void
SSTResponderSubComponent::init(unsigned phase)
{
    if (phase == 1) {
        for (auto p: responseReceiver->getInitData()) {
            gem5::Addr addr = p.first;
            std::vector<uint8_t> data = p.second;
            SST::Interfaces::StandardMem::Request* request = \
                new SST::Interfaces::StandardMem::Write(
                    addr, data.size(), data);
            memoryInterface->sendUntimedData(request);
        }
    }
    memoryInterface->init(phase);
}

void
SSTResponderSubComponent::setup()
{
}

bool
SSTResponderSubComponent::findCorrespondingSimObject(gem5::Root* gem5_root)
{
    gem5::OutgoingRequestBridge* receiver = \
        dynamic_cast<gem5::OutgoingRequestBridge*>(
            gem5_root->find(gem5SimObjectName.c_str()));
    setResponseReceiver(receiver);
    return receiver != NULL;
}

void
SSTResponderSubComponent::handleSwapReqResponse(
    SST::Interfaces::StandardMem::Request* request)
{
    // get the data, then,
    //     1. send a response to gem5 with the original data
    //     2. send a write to memory with atomic op applied

    SST::Interfaces::StandardMem::Request::id_t request_id = request->getID();
    TPacketMap::iterator it = sstRequestIdToPacketMap.find(request_id);
    assert(it != sstRequestIdToPacketMap.end());
    std::vector<uint8_t> data = \
        dynamic_cast<SST::Interfaces::StandardMem::ReadResp*>(request)->data;

    // step 1
    gem5::PacketPtr pkt = it->second;
    pkt->setData(
        dynamic_cast<SST::Interfaces::StandardMem::ReadResp*>(
            request)->data.data()
    );
    pkt->makeAtomicResponse();
    pkt->headerDelay = pkt->payloadDelay = 0;
    if (blocked() || !responseReceiver->sendTimingResp(pkt))
        responseQueue.push(pkt);

    // step 2
    (*(pkt->getAtomicOp()))(data.data()); // apply the atomic op
    // This is a Write. Need to use the Write visitor class. But the original
    // request is a read response. Therefore, we need to find the address and
    // the data size and then call Write.
    SST::Interfaces::StandardMem::Addr addr = \
        dynamic_cast<SST::Interfaces::StandardMem::ReadResp*>(request)->pAddr;
    auto data_size = data.size();
    // Create the Write request here.
    SST::Interfaces::StandardMem::Request* write_request = \
        new SST::Interfaces::StandardMem::Write(addr, data_size, data);
    // F_LOCKED flag in SimpleMem was changed to ReadLock and WriteUnlock
    // visitor classes. This has to be addressed in the future. The boot test
    // works without using ReadLock and WriteUnlock classes.
    memoryInterface->send(write_request);

    delete request;
}

void
SSTResponderSubComponent::portEventHandler(
    SST::Interfaces::StandardMem::Request* request)
{
    // Expect to handle an SST response
    SST::Interfaces::StandardMem::Request::id_t request_id = request->getID();

    TPacketMap::iterator it = sstRequestIdToPacketMap.find(request_id);

    // replying to a prior request
    if (it != sstRequestIdToPacketMap.end()) {
        gem5::PacketPtr pkt = it->second; // the packet that needs response

        // Responding to a SwapReq requires a special handler
        //     1. send a response to gem5 with the original data
        //     2. send a write to memory with atomic op applied
        if ((gem5::MemCmd::Command)pkt->cmd.toInt() == gem5::MemCmd::SwapReq) {
            handleSwapReqResponse(request);
            return;
        }

        sstRequestIdToPacketMap.erase(it);

        Translator::inplaceSSTRequestToGem5PacketPtr(pkt, request);

        if (blocked() || !(responseReceiver->sendTimingResp(pkt))) {
            responseQueue.push(pkt);
        }
    } else {
        // we can handle unexpected invalidates, but nothing else.
        if (SST::Interfaces::StandardMem::Read* test =
                dynamic_cast<SST::Interfaces::StandardMem::Read*>(request)) {
            return;
        }
        else if (SST::Interfaces::StandardMem::WriteResp* test =
                dynamic_cast<SST::Interfaces::StandardMem::WriteResp*>(
                request)) {
            return;
        }
        // for Snoop/no response needed
        // presently no consideration for masterId, packet type, flags...
        gem5::RequestPtr req = std::make_shared<gem5::Request>(
            dynamic_cast<SST::Interfaces::StandardMem::FlushAddr*>(
                request)->pAddr,
            dynamic_cast<SST::Interfaces::StandardMem::FlushAddr*>(
                request)->size, 0, 0);

        gem5::PacketPtr pkt = new gem5::Packet(
            req, gem5::MemCmd::InvalidateReq);

        // Clear out bus delay notifications
        pkt->headerDelay = pkt->payloadDelay = 0;

        responseReceiver->sendTimingSnoopReq(pkt);
    }

    delete request;
}

void
SSTResponderSubComponent::handleRecvRespRetry()
{
    while (blocked() &&
           responseReceiver->sendTimingResp(responseQueue.front()))
        responseQueue.pop();
}

void
SSTResponderSubComponent::handleRecvFunctional(gem5::PacketPtr pkt)
{
}

bool
SSTResponderSubComponent::blocked()
{
    return !(responseQueue.empty());
}
