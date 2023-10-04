// Copyright (c) 2023 The Regents of the University of California
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

#include <sst/core/interfaces/stdMem.h>
#include <sst/core/interfaces/stringEvent.h>
#include <sst/elements/memHierarchy/memEvent.h>
#include <sst/elements/memHierarchy/memTypes.h>
#include <sst/elements/memHierarchy/util.h>

typedef std::unordered_map<SST::Interfaces::StandardMem::Request::id_t,
                           gem5::PacketPtr> TPacketMap;

namespace Translator
{

inline SST::Interfaces::StandardMem::Request*
gem5RequestToSSTRequest(gem5::PacketPtr pkt,
                        TPacketMap& sst_request_id_to_packet_map)
{
    // Listing all the different SST Memory commands.
    enum sst_standard_mem_commands
    {
        Read,
        ReadResp,
        Write,
        WriteResp,
        FlushAddr,
        FlushResp,
        ReadLock,
        WriteUnlock,
        LoadLink,
        StoreConditional,
        MoveData,
        CustomReq,
        CustomResp,
        InvNotify

    };
    // SST's standard memory class has visitor classes for all the different
    // types of memory commands. Request class now does not have a command
    // variable. Instead for different types of request, we now need to
    // dynamically cast the class object. I'm using an extra variable to map
    // the type of command for SST.
    int sst_command_type = -1;
    // StandardMem only has one cache flush class with an option to flush or
    // flush and invalidate an address. By default, this is set to true so that
    // it corresponds to ge,::MemCmd::InvalidateReq
    bool flush_addr_flag = true;
    switch ((gem5::MemCmd::Command)pkt->cmd.toInt()) {
        case gem5::MemCmd::HardPFReq:
        case gem5::MemCmd::SoftPFReq:
        case gem5::MemCmd::SoftPFExReq:
        case gem5::MemCmd::LoadLockedReq:
        case gem5::MemCmd::ReadExReq:
        case gem5::MemCmd::ReadCleanReq:
        case gem5::MemCmd::ReadSharedReq:
        case gem5::MemCmd::ReadReq:
        case gem5::MemCmd::SwapReq:
            sst_command_type = Read;
            break;
        case gem5::MemCmd::StoreCondReq:
        case gem5::MemCmd::WritebackDirty:
        case gem5::MemCmd::WritebackClean:
        case gem5::MemCmd::WriteReq:
            sst_command_type = Write;
            break;
        case gem5::MemCmd::CleanInvalidReq:
        case gem5::MemCmd::InvalidateReq:
            sst_command_type = FlushAddr;
            break;
        case gem5::MemCmd::CleanSharedReq:
            sst_command_type = FlushAddr;
            flush_addr_flag = false;
            break;
        default:
            panic("Unable to convert gem5 packet: %s\n", pkt->cmd.toString());
    }

    SST::Interfaces::StandardMem::Addr addr = pkt->getAddr();
    auto data_size = pkt->getSize();
    std::vector<uint8_t> data;
    // Need to make sure that the command type is a Write to retrive the data
    // data_ptr.
    if (sst_command_type == Write) {
        uint8_t* data_ptr = pkt->getPtr<uint8_t>();
        data = std::vector<uint8_t>(data_ptr, data_ptr + data_size);

    }
    // Now convert a sst StandardMem request.
    SST::Interfaces::StandardMem::Request* request = nullptr;
    // find the corresponding memory command type.
    switch(sst_command_type) {
        case Read:
            request = new SST::Interfaces::StandardMem::Read(addr, data_size);
            break;
        case Write:
            request =
                new SST::Interfaces::StandardMem::Write(addr, data_size, data);
            break;
        case FlushAddr: {
            // StandardMem::FlushAddr has a invoking variable called `depth`
            // which defines the number of cache levels to invalidate. Ideally
            // this has to be input from the SST config, however in
            // implementation I'm hardcoding this value to 2.
            int cache_depth = 2;
            request =
                new SST::Interfaces::StandardMem::FlushAddr(
                    addr, data_size, flush_addr_flag, cache_depth);
            break;
        }
        default:
            panic("Unable to translate command %d to Request class!",
                sst_command_type);
    }

    if ((gem5::MemCmd::Command)pkt->cmd.toInt() == gem5::MemCmd::LoadLockedReq
        || (gem5::MemCmd::Command)pkt->cmd.toInt() == gem5::MemCmd::SwapReq
        || pkt->req->isLockedRMW()) {
        // F_LOCKED is deprecated. Therefore I'm skipping this flag for the
        // StandardMem request.
    } else if ((gem5::MemCmd::Command)pkt->cmd.toInt() ==
              gem5::MemCmd::StoreCondReq) {
        // F_LLSC is deprecated. Therefore I'm skipping this flag for the
        // StandardMem request.
    }

    if (pkt->req->isUncacheable()) {
        request->setFlag(
            SST::Interfaces::StandardMem::Request::Flag::F_NONCACHEABLE);
    }

    if (pkt->needsResponse())
        sst_request_id_to_packet_map[request->getID()] = pkt;

    return request;
}

inline void
inplaceSSTRequestToGem5PacketPtr(gem5::PacketPtr pkt,
                                SST::Interfaces::StandardMem::Request* request)
{
    pkt->makeResponse();

    // Resolve the success of Store Conditionals
    if (pkt->isLLSC() && pkt->isWrite()) {
        // SC interprets ExtraData == 1 as the store was successful
        pkt->req->setExtraData(1);
    }
    // If there is data in the request, send it back. Only ReadResp requests
    // have data associated with it. Other packets does not need to be casted.
    if (!pkt->isWrite()) {
        // Need to verify whether the packet is a ReadResp, otherwise the
        // program will try to incorrectly cast the request object.
        if (SST::Interfaces::StandardMem::ReadResp* test =
            dynamic_cast<SST::Interfaces::StandardMem::ReadResp*>(request)) {
            pkt->setData(dynamic_cast<SST::Interfaces::StandardMem::ReadResp*>(
                request)->data.data()
            );
        }
    }

    // Clear out bus delay notifications
    pkt->headerDelay = pkt->payloadDelay = 0;

}

}; // namespace Translator

#endif // __TRANSLATOR_H__
