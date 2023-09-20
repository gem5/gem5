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

#ifndef __DEV_GPU_SYSTEM_HUB_HH__
#define __DEV_GPU_SYSTEM_HUB_HH__

#include "dev/dma_device.hh"
#include "params/AMDGPUSystemHub.hh"

namespace gem5
{

class RequestPort;

/**
 * This class handles reads from the system/host memory space from the shader.
 * It is meant to handle requests to memory which translation to system
 * addresses. This can occur in fetch, scalar read/write, or vector memory
 * read/write. It is a very basic interface to convert read packets to DMA
 * requests and respond to the caller using an event.
 */
class AMDGPUSystemHub : public DmaDevice
{
  public:
    AMDGPUSystemHub(const AMDGPUSystemHubParams &p) : DmaDevice(p) { }

    void sendRequest(PacketPtr pkt, Event *callback);
    void dmaResponse(PacketPtr pkt);

    /**
     * Inherited methods.
     */
    Tick write(PacketPtr pkt) override { return 0; }
    Tick read(PacketPtr pkt) override { return 0; }
    AddrRangeList getAddrRanges() const override;

  private:
    typedef std::pair<PacketPtr, Event*> DeferredReq;
    typedef std::list<DeferredReq> DeferredReqList;
    std::unordered_map<Addr, DeferredReqList> outstandingReqs;

    void sendNextRequest(Addr addr, const PacketPtr donePkt);
    void sendDeferredRequest(DeferredReq& deferredReq);

    class ResponseEvent : public Event
    {
        AMDGPUSystemHub &systemHub;
        Event *callback;
        PacketPtr pkt;

      public:
        ResponseEvent(AMDGPUSystemHub& _hub,
                      Event *_callback, PacketPtr _pkt);

        void process();
    };

    class AtomicResponseEvent : public Event
    {
        AMDGPUSystemHub &systemHub;
        Event *callback;
        PacketPtr pkt;

      public:
        AtomicResponseEvent(AMDGPUSystemHub& _hub,
                            Event *_callback, PacketPtr _pkt);

        void process();
    };
};

} // namespace gem5

#endif /* __DEV_GPU_SYSTEM_HUB_HH__ */
