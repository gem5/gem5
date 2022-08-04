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
    : ClockedObject(p), _gpuMemPort(csprintf("%s-port", name()), this),
      cacheLineSize(p.system->cacheLineSize()),
      _requestorId(p.system->getRequestorId(this))
{
}

void
AMDGPUMemoryManager::writeRequest(Addr addr, uint8_t *data, int size,
                               Request::Flags flag, Event *callback)
{
    assert(data);

    ChunkGenerator gen(addr, size, cacheLineSize);
    for (; !gen.done(); gen.next()) {
        RequestPtr req = std::make_shared<Request>(gen.addr(), gen.size(),
                                                   flag, _requestorId);

        PacketPtr pkt = Packet::createWrite(req);
        uint8_t *dataPtr = new uint8_t[gen.size()];
        std::memcpy(dataPtr, data + (gen.complete()/sizeof(uint8_t)),
                    gen.size());
        pkt->dataDynamic<uint8_t>(dataPtr);

        // We only want to issue the callback on the last request completing.
        if (gen.last()) {
            pkt->pushSenderState(new GPUMemPort::SenderState(callback, addr));
        } else {
            pkt->pushSenderState(new GPUMemPort::SenderState(nullptr, addr));
        }

        if (!_gpuMemPort.sendTimingReq(pkt)) {
            DPRINTF(AMDGPUMem, "Request to %#lx needs retry\n", gen.addr());
            _gpuMemPort.retries.push_back(pkt);
        } else {
            DPRINTF(AMDGPUMem, "Write request to %#lx sent\n", gen.addr());
        }
    }
}

bool
AMDGPUMemoryManager::GPUMemPort::recvTimingResp(PacketPtr pkt)
{
    // Retrieve sender state
    [[maybe_unused]] SenderState *sender_state =
        safe_cast<SenderState*>(pkt->senderState);

    DPRINTF(AMDGPUMem, "Recveived Response for %#x\n", sender_state->_addr);

    // Check if there is a callback event and if so call it
    if (sender_state->_callback) {
        sender_state->_callback->process();
        delete sender_state->_callback;
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
