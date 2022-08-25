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
 */

#ifndef __DEV_AMDGPU_MEMORY_MANAGER_HH__
#define __DEV_AMDGPU_MEMORY_MANAGER_HH__

#include <deque>

#include "base/callback.hh"
#include "mem/port.hh"
#include "params/AMDGPUMemoryManager.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class AMDGPUMemoryManager : public ClockedObject
{
    class GPUMemPort : public MasterPort
    {
        public:
        GPUMemPort(const std::string &_name, AMDGPUMemoryManager *_gpuMemMgr)
            : MasterPort(_name, _gpuMemMgr)
        {
        }

        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;

        struct SenderState : public Packet::SenderState
        {
            SenderState(Event *callback, Addr addr)
                : _callback(callback), _addr(addr)
            {}

            Event *_callback;
            Addr _addr;
        };

        std::deque<PacketPtr> retries;
    };

    GPUMemPort _gpuMemPort;
    const int cacheLineSize;
    const RequestorID _requestorId;

  public:
    AMDGPUMemoryManager(const AMDGPUMemoryManagerParams &p);
    ~AMDGPUMemoryManager() {};

    /**
     * Write size amount of data to device memory at addr using flags and
     * callback.
     *
     * @param addr Device address to write.
     * @param data Pointer to data to write.
     * @param size Number of bytes to write.
     * @param flag Additional request flags for write packets.
     * @param callback Event callback to call after all bytes are written.
     */
    void writeRequest(Addr addr, uint8_t *data, int size,
                      Request::Flags flag = 0, Event *callback = nullptr);

    /**
     * Read size amount of data from device memory at addr using flags and
     * callback.
     *
     * @param addr Device address to read.
     * @param data Pointer to data to read into.
     * @param size Number of bytes to read.
     * @param flag Additional request flags for read packets.
     * @param callback Event callback to call after all bytes are read.
     */
    void readRequest(Addr addr, uint8_t *data, int size,
                     Request::Flags flag = 0, Event *callback = nullptr);

    /**
     * Get the requestorID for the memory manager. This ID is used for all
     * packets which should be routed through the device network.
     *
     * @return requestorID of this object.
     */
    RequestorID getRequestorID() const { return _requestorId; }

    Port &
    getPort(const std::string &if_name, PortID idx) override
    {
        if (if_name == "port") {
            return _gpuMemPort;
        } else {
            return ClockedObject::getPort(if_name, idx);
        }
    }
};

} // namespace gem5

#endif // __DEV_AMDGPU_MEMORY_MANAGER_HH__
