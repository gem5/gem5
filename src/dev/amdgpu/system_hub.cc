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

#include "dev/amdgpu/system_hub.hh"

#include "mem/port.hh"

namespace gem5
{

void
AMDGPUSystemHub::sendRequest(PacketPtr pkt, Event *callback)
{
    ResponseEvent *dmaRespEvent = new ResponseEvent(callback);
    Tick delay = 0;

    if (pkt->isWrite()) {
        dmaWrite(pkt->getAddr(), pkt->getSize(), dmaRespEvent,
                 pkt->getPtr<uint8_t>(), 0, 0, delay);
    } else {
        assert(pkt->isRead());
        dmaRead(pkt->getAddr(), pkt->getSize(), dmaRespEvent,
                pkt->getPtr<uint8_t>(), 0, 0, delay);
    }
}

void
AMDGPUSystemHub::dmaResponse(PacketPtr pkt)
{
}

AMDGPUSystemHub::ResponseEvent::ResponseEvent(Event *_callback)
    : callback(_callback)
{
    // Delete this event after process is called
    setFlags(Event::AutoDelete);
}

void
AMDGPUSystemHub::ResponseEvent::process()
{
    callback->process();
}

AddrRangeList
AMDGPUSystemHub::getAddrRanges() const
{
    AddrRangeList ranges;
    return ranges;
}

} // namespace gem5
