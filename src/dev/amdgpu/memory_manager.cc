/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "dev/amdgpu/memory_manager.hh"

#include <memory>

#include "base/chunk_generator.hh"
#include "debug/AMDGPUMem.hh"
#include "params/AMDGPUMemoryManager.hh"
#include "sim/system.hh"

namespace gem5
{

AMDGPUMemoryManager::AMDGPUMemoryManager(const AMDGPUMemoryManagerParams &p)
    : ClockedObject(p), _gpuMemPort(csprintf("%s-port", name()), *this),
      cacheLineSize(p.cache_line_size),
      _requestorId(p.system->getRequestorId(this))
{
}

void
AMDGPUMemoryManager::writeRequest(Addr addr, uint8_t *data, int size,
                               Request::Flags flag, Event *callback)
{
    assert(data);

    // Requests may return out of order, so we should track how many chunks
    // are outstanding and if the last chunk was sent. Give each status struct
    // a unique ID so that DMAs to the same address may occur at the same time
    requestStatus.emplace(std::piecewise_construct,
                          std::forward_as_tuple(requestId), std::tuple<>{});

    DPRINTF(AMDGPUMem, "Created status for write request %ld\n", requestId);

    ChunkGenerator gen(addr, size, cacheLineSize);
    for (; !gen.done(); gen.next()) {
        RequestPtr req = std::make_shared<Request>(gen.addr(), gen.size(),
                                                   flag, _requestorId);

        PacketPtr pkt = Packet::createWrite(req);
        uint8_t *dataPtr = new uint8_t[gen.size()];
        std::memcpy(dataPtr, data + (gen.complete()/sizeof(uint8_t)),
                    gen.size());
        pkt->dataDynamic<uint8_t>(dataPtr);

        pkt->pushSenderState(
                new GPUMemPort::SenderState(callback, addr, requestId));
        requestStatus.at(requestId).outstandingChunks++;
        if (gen.last()) {
            requestStatus.at(requestId).sentLastChunk = true;
        }

        if (!_gpuMemPort.sendTimingReq(pkt)) {
            DPRINTF(AMDGPUMem, "Request to %#lx needs retry\n", gen.addr());
            _gpuMemPort.retries.push_back(pkt);
        } else {
            DPRINTF(AMDGPUMem, "Write request to %#lx sent\n", gen.addr());
        }
    }

    requestId++;
}

void
AMDGPUMemoryManager::readRequest(Addr addr, uint8_t *data, int size,
                                 Request::Flags flag, Event *callback)
{
    assert(data);
    uint8_t *dataPtr = data;

    // Requests may return out of order, so we should track how many chunks
    // are outstanding and if the last chunk was sent. Give each status struct
    // a unique ID so that DMAs to the same address may occur at the same time
    requestStatus.emplace(std::piecewise_construct,
                          std::forward_as_tuple(requestId), std::tuple<>{});

    DPRINTF(AMDGPUMem, "Created status for read request %ld\n", requestId);

    ChunkGenerator gen(addr, size, cacheLineSize);
    for (; !gen.done(); gen.next()) {
        RequestPtr req = std::make_shared<Request>(gen.addr(), gen.size(),
                                                   flag, _requestorId);

        PacketPtr pkt = Packet::createRead(req);
        pkt->dataStatic<uint8_t>(dataPtr);
        dataPtr += gen.size();

        pkt->pushSenderState(
                new GPUMemPort::SenderState(callback, addr, requestId));
        requestStatus.at(requestId).outstandingChunks++;
        if (gen.last()) {
            requestStatus.at(requestId).sentLastChunk = true;
        }

        if (!_gpuMemPort.sendTimingReq(pkt)) {
            DPRINTF(AMDGPUMem, "Request to %#lx needs retry\n", gen.addr());
            _gpuMemPort.retries.push_back(pkt);
        } else {
            DPRINTF(AMDGPUMem, "Read request to %#lx sent\n", gen.addr());
        }
    }

    requestId++;
}

bool
AMDGPUMemoryManager::GPUMemPort::recvTimingResp(PacketPtr pkt)
{
    // Retrieve sender state
    [[maybe_unused]] SenderState *sender_state =
        safe_cast<SenderState*>(pkt->senderState);

    // Check if all chunks have completed, the last chunk was sent, and there
    // is a callback, call the callback now.
    assert(gpu_mem.requestStatus.count(sender_state->_requestId));
    auto& status = gpu_mem.requestStatus.at(sender_state->_requestId);

    assert(status.outstandingChunks != 0);
    status.outstandingChunks--;
    DPRINTF(AMDGPUMem, "Received Response for %#x. %d chunks remain, sent "
            "last = %d, requestId = %ld\n", sender_state->_addr,
            status.outstandingChunks, status.sentLastChunk,
            sender_state->_requestId);

    if (!status.outstandingChunks && status.sentLastChunk) {
        // Call and free the callback if there is one
        if (sender_state->_callback) {
            DPRINTF(AMDGPUMem, "Calling callback for request %ld\n",
                    sender_state->_requestId);
            sender_state->_callback->process();
            delete sender_state->_callback;
        }
        DPRINTF(AMDGPUMem, "Deleting status for request %ld\n",
                sender_state->_requestId);
        gpu_mem.requestStatus.erase(sender_state->_requestId);
    }

    delete pkt->senderState;
    delete pkt;
    return true;
}

void
AMDGPUMemoryManager::GPUMemPort::recvReqRetry()
{
    for (const auto &pkt : retries) {
        if (!sendTimingReq(pkt)) {
            break;
        } else {
            DPRINTF(AMDGPUMem, "Retry for %#lx sent\n", pkt->getAddr());
            retries.pop_front();
        }
    }
}

} // namespace gem5
