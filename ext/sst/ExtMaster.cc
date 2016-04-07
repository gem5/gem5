// Copyright (c) 2015 ARM Limited
// All rights reserved.
// 
// The license below extends only to copyright in the software and shall
// not be construed as granting a license to any other intellectual
// property including but not limited to intellectual property relating
// to a hardware implementation of the functionality of the software
// licensed hereunder.  You may use the software subject to the license
// terms below provided that you ensure that this notice is replicated
// unmodified and in its entirety in all distributions of the software,
// modified or unmodified, in source code or in binary form.
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

// Copyright 2009-2014 Sandia Coporation.  Under the terms
// of Contract DE-AC04-94AL85000 with Sandia Corporation, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2009-2014, Sandia Corporation
// All rights reserved.
//
// For license information, see the LICENSE file in the current directory.

#include "gem5.hh"

#ifdef fatal  // gem5 sets this
#undef fatal
#endif

#include <sst_config.h>

#include <mem/packet.hh>

#include <sst/core/component.h>
#include <sst/core/params.h>
#include <sst/core/link.h>
#include <sst/elements/memHierarchy/memNIC.h>

using namespace SST;
using namespace SST::gem5;
using namespace SST::MemHierarchy;

ExtMaster::ExtMaster(gem5Component *g, Output &o, ::ExternalMaster& p,
        std::string &n) :
    Port(n, p), out(o), port(p), simPhase(CONSTRUCTION),
    gem5(g), name(n)
{
    Params _p; // will be ignored
    nic = dynamic_cast<MemNIC*>(gem5->loadModuleWithComponent("memHierarchy.memNIC", g, _p));

    MemNIC::ComponentInfo ci;
    ci.num_vcs = 1;
    ci.link_port = "network";
    ci.link_bandwidth = "16GB/s";
    ci.link_inbuf_size = "1KB";
    ci.link_outbuf_size = "1KB";
    ci.network_addr = 0; // hard coded at the moment
    ci.type = MemNIC::TypeDirectoryCtrl;
    nic->moduleInit(ci, new Event::Handler<ExtMaster>
                                          (this, &ExtMaster::handleEvent));
}

void
ExtMaster::init(unsigned phase)
{
    simPhase = INIT;

    if (phase == 0) {
        assert(nic);
        for (auto range : getAddrRanges()) {
            MemNIC::ComponentTypeInfo ti;
            ti.rangeStart = range.start();
            ti.rangeEnd = range.end();
            ti.interleaveSize = 0;
            ti.interleaveStep = 0;
            nic->addTypeInfo(ti);
            ranges.insert(range);
        }
    }

    nic->init(phase);
}

void
ExtMaster::setup(void)
{
    nic->setup();

    simPhase = RUN;
}

void
ExtMaster::finish(void)
{
    nic->finish();
}

void
ExtMaster::clock(void)
{
    nic->clock();
}

void
ExtMaster::handleEvent(SST::Event* event)
{
    if (simPhase == CONSTRUCTION) {
        out.fatal(CALL_INFO, 1, "received Event during Construction phase\n");
    }

    MemEvent *ev = dynamic_cast<MemEvent*>(event);
    if (!ev) {
        out.fatal(CALL_INFO, 1, "Can't handle non-MemEvent Event's\n");
    }

    Command cmdI = ev->getCmd(); // command in - SST
    MemCmd::Command cmdO;        // command out - gem5
    bool data = false;

    switch (cmdI) {
        case GetS:      cmdO = MemCmd::ReadReq;                break;
        case GetX:      cmdO = MemCmd::WriteReq;  data = true; break;
        case GetSEx:
        case PutS:
        case PutM:
        case PutE:
        case PutX:
        case PutXE:
        case Inv:
        case FetchInv:
        case FetchInvX:

        case NACK:

        case NULLCMD:
        case GetSResp:
        case GetXResp:
        case FetchResp:
        case FetchXResp:
            out.fatal(CALL_INFO, 1, "Don't know how to convert "
                                    "SST command %s to gem5\n",
                      CommandString[cmdI]);
    }

    Request::FlagsType flags = 0;
    if (ev->queryFlag(MemEvent::F_LOCKED))
        flags |= Request::LOCKED_RMW;
    if (ev->queryFlag(MemEvent::F_NONCACHEABLE))
        flags |= Request::UNCACHEABLE;
    if (ev->isLoadLink()) {
        assert(cmdI == GetS);
        cmdO = MemCmd::LoadLockedReq;
    } else if (ev->isStoreConditional()) {
        assert(cmdI == GetX);
        cmdO = MemCmd::StoreCondReq;
    }

    auto req = new Request(ev->getAddr(), ev->getSize(), flags, 0);
    req->setContext(ev->getGroupId());

    auto pkt = new Packet(req, cmdO);
    pkt->allocate();
    if (data) {
        pkt->setData(ev->getPayload().data());
    }
    pkt->pushSenderState(new SenderState(ev));

    if (blocked() || !sendTimingReq(pkt))
        sendQ.push_back(pkt);
}

bool
ExtMaster::recvTimingResp(PacketPtr pkt) {
    if (simPhase == INIT) {
        out.fatal(CALL_INFO, 1, "not prepared to handle INIT-phase traffic\n");
    }

    // get original SST packet from gem5 SenderState
    auto senderState = dynamic_cast<SenderState*>(pkt->popSenderState());
    if (!senderState)
        out.fatal(CALL_INFO, 1, "gem5 senderState corrupt\n");

    // make (new) response packet, discard (old) original request
    MemEvent* ev = senderState->event;
    delete senderState;

    MemEvent* resp = ev->makeResponse();
    delete ev;

    // copy the payload and then destroy gem5 packet
    resp->setPayload(pkt->getSize(), pkt->getPtr<uint8_t>());
    delete pkt->req;
    delete pkt;

    nic->send(resp);
    return true;
}

void
ExtMaster::recvReqRetry() {
    while (blocked() && sendTimingReq(sendQ.front())) {
        sendQ.pop_front();
    }
}

void
ExtMaster::recvRangeChange() {
    for (auto range : getAddrRanges()) {
        if (ranges.find(range) == ranges.end()) { // i.e. if not found,
            MemNIC::ComponentTypeInfo ti;      // indicating a new range.
            ti.rangeStart = range.start();
            ti.rangeEnd = range.end();
            ti.interleaveSize = 0;
            ti.interleaveStep = 0;
            nic->addTypeInfo(ti);
            ranges.insert(range);
        }
    }
}
