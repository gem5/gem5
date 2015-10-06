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

#include <sst_config.h>
#include <sst/core/serialization.h>

#include <sst/core/params.h>
#include <sst/core/output.h>
#include <sst/core/link.h>

#ifdef fatal  // gem5 sets this
#undef fatal
#endif

using namespace SST;
using namespace SST::gem5;
using namespace SST::MemHierarchy;

ExtSlave::ExtSlave(gem5Component *g5c, Output &out,
        ::ExternalSlave& port, std::string &name) :
    Port(name, port),
    comp(g5c), out(out), simPhase(CONSTRUCTION), initPackets(NULL),
    link(comp->configureLink(name, new Event::Handler<ExtSlave>(this,
                                              &ExtSlave::handleEvent)))
{
    if (!link) {
        out.fatal(CALL_INFO, 1, "Failed to configure link %s\n", name.c_str());
    }
}

void ExtSlave::init(unsigned phase)
{
    simPhase = INIT;
    if (initPackets) {
        while (!initPackets->empty()) {
            link->sendInitData(initPackets->front());
            initPackets->pop_front();
        }
        delete initPackets;
        initPackets = NULL;
    }
}

void
ExtSlave::recvFunctional(PacketPtr pkt)
{
    if (simPhase == CONSTRUCTION) {
        if (initPackets == NULL) {
            initPackets = new std::list<MemEvent*>;
        }
        ::MemCmd::Command pktCmd = (::MemCmd::Command)pkt->cmd.toInt();
        assert(pktCmd == ::MemCmd::WriteReq || pktCmd == ::MemCmd::Writeback);
        Addr a = pkt->getAddr();
        MemEvent* ev = new MemEvent(comp, a, a, GetX);
        ev->setPayload(pkt->getSize(), pkt->getPtr<uint8_t>());
        initPackets->push_back(ev);
    } else {
        panic("Functional accesses not allowed after construction phase");
    }
}

bool
ExtSlave::recvTimingReq(PacketPtr pkt)
{
    Command cmd;
    switch ((::MemCmd::Command)pkt->cmd.toInt()) {
    case ::MemCmd::HardPFReq:
    case ::MemCmd::SoftPFReq:
    case ::MemCmd::LoadLockedReq:
    case ::MemCmd::ReadExReq:
    case ::MemCmd::ReadReq:       cmd = GetS;   break;
    case ::MemCmd::StoreCondReq:
    case ::MemCmd::WriteReq:      cmd = GetX;   break;
    default:
        out.fatal(CALL_INFO, 1, "Don't know how to convert gem5 packet "
                  "command %s to SST\n", pkt->cmd.toString().c_str());
    }

    auto ev = new MemEvent(comp, pkt->getAddr(), pkt->getAddr(), cmd);
    ev->setPayload(pkt->getSize(), pkt->getPtr<uint8_t>());
    if ((::MemCmd::Command)pkt->cmd.toInt() == ::MemCmd::LoadLockedReq)
        ev->setLoadLink();
    else if ((::MemCmd::Command)pkt->cmd.toInt() == ::MemCmd::StoreCondReq)
        ev->setStoreConditional();

    if (pkt->req->isLockedRMW())      ev->setFlag(MemEvent::F_LOCKED);
    if (pkt->req->isUncacheable()) ev->setFlag(MemEvent::F_NONCACHEABLE);
    if (pkt->req->hasContextId())  ev->setGroupId(pkt->req->contextId());
// Prefetches not working with SST; it maybe be dropping them, treating them
// as not deserving of responses, or something else -- not sure yet.
//  ev->setPrefetchFlag(pkt->req->isPrefetch());

    if (simPhase == INIT) {
        link->sendInitData(ev);
        delete pkt->req;
        delete pkt;
    } else {
        if (pkt->needsResponse()) {
            PacketMap[ev->getID()] = pkt;
        }
        link->send(ev);
    }
    return true;
}


void
ExtSlave::handleEvent(Event* ev)
{
    MemEvent* event = dynamic_cast<MemEvent*>(ev);
    if (!event) {
        out.fatal(CALL_INFO, 1, "ExtSlave handleEvent received non-MemEvent\n");
        delete ev;
        return;
    }
    Event::id_type id = event->getID();

    PacketMap_t::iterator mi = PacketMap.find(id);
    if (mi != PacketMap.end()) { // replying to prior request
        PacketPtr pkt = mi->second;
        PacketMap.erase(mi);

        pkt->makeResponse();  // Convert to a response packet
        pkt->setData(event->getPayload().data());

        // Resolve the success of Store Conditionals
        if (pkt->isLLSC() && pkt->isWrite()) {
            pkt->req->setExtraData(event->isAtomic());
        }

        // Clear out bus delay notifications
        pkt->headerDelay = pkt->payloadDelay = 0;

        if (blocked() || !sendTimingResp(pkt)) {
            respQ.push_back(pkt);
        }
    } else { // we can handle unexpected invalidates, but nothing else.
        Command cmd = event->getCmd();
        assert(cmd == Inv);

        // make Req/Pkt for Snoop/no response needed
        // presently no consideration for masterId, packet type, flags...
        RequestPtr req = new Request(event->getAddr(), event->getSize(), 0, 0);
        auto pkt = new Packet(req, ::MemCmd::InvalidateReq);

        // Clear out bus delay notifications
        pkt->headerDelay = pkt->payloadDelay = 0;

        sendTimingSnoopReq(pkt);
    }
    delete event;
}

void
ExtSlave::recvRespRetry()
{
    while (blocked() && sendTimingResp(respQ.front())) {
        respQ.pop_front();
    }
}
