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

#ifndef __TRANSLATOR_H__
#define __TRANSLATOR_H__

#include <sst/core/simulation.h>
#include <sst/core/interfaces/stringEvent.h>
#include <sst/core/interfaces/simpleMem.h>
#include <sst/elements/memHierarchy/memEvent.h>
#include <sst/elements/memHierarchy/memTypes.h>
#include <sst/elements/memHierarchy/util.h>

typedef std::unordered_map<SST::Interfaces::SimpleMem::Request::id_t,
                           gem5::PacketPtr> TPacketMap;

namespace Translator
{

inline SST::Interfaces::SimpleMem::Request*
gem5RequestToSSTRequest(gem5::PacketPtr pkt,
                        TPacketMap& sst_request_id_to_packet_map)
{
    SST::Interfaces::SimpleMem::Request::Command cmd;
    switch ((gem5::MemCmd::Command)pkt->cmd.toInt()) {
        case gem5::MemCmd::HardPFReq:
        case gem5::MemCmd::SoftPFReq:
        case gem5::MemCmd::SoftPFExReq:
        case gem5::MemCmd::LoadLockedReq:
        case gem5::MemCmd::ReadExReq:
        case gem5::MemCmd::ReadReq:
        case gem5::MemCmd::SwapReq:
            cmd = SST::Interfaces::SimpleMem::Request::Command::Read;
            break;
        case gem5::MemCmd::StoreCondReq:
        case gem5::MemCmd::WriteReq:
            cmd = SST::Interfaces::SimpleMem::Request::Command::Write;
            break;
        case gem5::MemCmd::CleanInvalidReq:
        case gem5::MemCmd::InvalidateReq:
            cmd = SST::Interfaces::SimpleMem::Request::Command::FlushLineInv;
            break;
        case gem5::MemCmd::CleanSharedReq:
            cmd = SST::Interfaces::SimpleMem::Request::Command::FlushLine;
            break;
        default:
            panic("Unable to convert gem5 packet: %s\n", pkt->cmd.toString());
    }

    SST::Interfaces::SimpleMem::Addr addr = pkt->getAddr();

    uint8_t* data_ptr = pkt->getPtr<uint8_t>();
    auto data_size = pkt->getSize();
    std::vector<uint8_t> data = std::vector<uint8_t>(
        data_ptr, data_ptr + data_size
    );

    SST::Interfaces::SimpleMem::Request* request = \
        new SST::Interfaces::SimpleMem::Request(
            cmd, addr, data_size, data
        );

    if ((gem5::MemCmd::Command)pkt->cmd.toInt() == gem5::MemCmd::LoadLockedReq
        || (gem5::MemCmd::Command)pkt->cmd.toInt() == gem5::MemCmd::SwapReq
        || pkt->req->isLockedRMW()) {
        request->setMemFlags(
            SST::Interfaces::SimpleMem::Request::Flags::F_LOCKED);
    } else if ((gem5::MemCmd::Command)pkt->cmd.toInt() == \
              gem5::MemCmd::StoreCondReq) {
        request->setMemFlags(
            SST::Interfaces::SimpleMem::Request::Flags::F_LLSC);
    }

    if (pkt->req->isUncacheable()) {
        request->setFlags(
            SST::Interfaces::SimpleMem::Request::Flags::F_NONCACHEABLE);
    }

    if (pkt->needsResponse())
        sst_request_id_to_packet_map[request->id] = pkt;

    return request;
}

inline void
inplaceSSTRequestToGem5PacketPtr(gem5::PacketPtr pkt,
                                 SST::Interfaces::SimpleMem::Request* request)
{
    pkt->makeResponse();

    // Resolve the success of Store Conditionals
    if (pkt->isLLSC() && pkt->isWrite()) {
        // SC interprets ExtraData == 1 as the store was successful
        pkt->req->setExtraData(1);
    }

    pkt->setData(request->data.data());

    // Clear out bus delay notifications
    pkt->headerDelay = pkt->payloadDelay = 0;

}

}; // namespace Translator

#endif // __TRANSLATOR_H__
