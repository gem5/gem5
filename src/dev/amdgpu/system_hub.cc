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

#include "debug/AMDGPUSystemHub.hh"
#include "mem/packet_access.hh"
#include "mem/port.hh"

namespace gem5
{

void
AMDGPUSystemHub::sendRequest(PacketPtr pkt, Event *callback)
{
    // Some requests, in particular atomics, need to be sent in order
    // to receive the correct values. If there is an atomic in progress
    // we must block it until that request is complete. This is overly
    // conservative and blocks reads/writes but this situation is rare
    // so it should not impact simulated performance.
    DeferredReq this_req(pkt, callback);
    outstandingReqs[pkt->getAddr()].push_back(this_req);

    if (outstandingReqs[pkt->getAddr()].size () > 1) {
        // There is another request in progress, Delay this one.
        DPRINTF(AMDGPUSystemHub, "SystemHub deferring request for %#lx\n",
                pkt->getAddr());
    } else {
        // No other requests, we can send immediately.
        sendDeferredRequest(this_req);
    }
}

void
AMDGPUSystemHub::sendDeferredRequest(DeferredReq& deferredReq)
{
    PacketPtr pkt = deferredReq.first;
    Event *callback = deferredReq.second;
    Tick delay = 0;
    std::string req_type;

    if (pkt->isAtomicOp()) {
        AtomicResponseEvent *atomicRespEvent =
            new AtomicResponseEvent(*this, callback, pkt);

        // First read the value. The response event will do the atomic/write
        // This places the current value in the packet, which is correct since
        // atomics return the value prior to performing the atomic.
        dmaRead(pkt->getAddr(), pkt->getSize(), atomicRespEvent,
                pkt->getPtr<uint8_t>(), 0, 0, delay);

        req_type = "Atomic";
    } else if (pkt->isWrite()) {
        ResponseEvent *dmaRespEvent =
            new ResponseEvent(*this, callback, pkt);

        dmaWrite(pkt->getAddr(), pkt->getSize(), dmaRespEvent,
                 pkt->getPtr<uint8_t>(), 0, 0, delay);

        req_type = "Write";
    } else {
        ResponseEvent *dmaRespEvent =
            new ResponseEvent(*this, callback, pkt);

        assert(pkt->isRead());
        dmaRead(pkt->getAddr(), pkt->getSize(), dmaRespEvent,
                pkt->getPtr<uint8_t>(), 0, 0, delay);

        req_type = "Read";
    }

    DPRINTF(AMDGPUSystemHub, "SystemHub %s request for %#lx size %d\n",
            req_type.c_str(), pkt->getAddr(), pkt->getSize());
}

void
AMDGPUSystemHub::sendNextRequest(Addr addr, const PacketPtr donePkt)
{
    // Remove our request
    assert(outstandingReqs.count(addr));

    [[maybe_unused]] DeferredReq& frontPkt = outstandingReqs[addr].front();
    assert(frontPkt.first == donePkt);

    outstandingReqs[addr].pop_front();

    // If there are no more requests this can be removed from the map.
    // Otherwise issue the next request in the list
    if (outstandingReqs[addr].empty()) {
        DPRINTF(AMDGPUSystemHub, "SystemHub done with packets for addr %#lx\n",
                donePkt->getAddr());

        outstandingReqs.erase(addr);
    } else {
        DeferredReq& nextPkt = outstandingReqs[addr].front();

        DPRINTF(AMDGPUSystemHub, "SystemHub sending deferred request for addr"
                " %#lx size %d\n", nextPkt.first->getAddr(),
                nextPkt.first->getSize());

        sendDeferredRequest(nextPkt);
    }
}

void
AMDGPUSystemHub::dmaResponse(PacketPtr pkt)
{
}

AMDGPUSystemHub::ResponseEvent::ResponseEvent(
        AMDGPUSystemHub& _hub, Event *_callback, PacketPtr _pkt)
    : systemHub(_hub), callback(_callback), pkt(_pkt)
{
    // Delete this event after process is called
    setFlags(Event::AutoDelete);
}

void
AMDGPUSystemHub::ResponseEvent::process()
{
    DPRINTF(AMDGPUSystemHub, "SystemHub response for addr %#lx size %d\n",
            pkt->getAddr(), pkt->getSize());

    systemHub.sendNextRequest(pkt->getAddr(), pkt);

    callback->process();
}

AMDGPUSystemHub::AtomicResponseEvent::AtomicResponseEvent(
        AMDGPUSystemHub& _hub, Event *_callback, PacketPtr _pkt)
    : systemHub(_hub), callback(_callback), pkt(_pkt)
{
    // Delete this event after process is called
    setFlags(Event::AutoDelete);
}

void
AMDGPUSystemHub::AtomicResponseEvent::process()
{
    // Make a second response with the original sender's callback
    ResponseEvent *dmaRespEvent = new ResponseEvent(systemHub, callback, pkt);
    Tick delay = 0;

    // Create a new write packet which will be modifed then written
    RequestPtr write_req =
        std::make_shared<Request>(pkt->getAddr(), pkt->getSize(), 0,
                                  pkt->requestorId());

    PacketPtr write_pkt = Packet::createWrite(write_req);
    uint8_t *write_data = new uint8_t[pkt->getSize()];
    std::memcpy(write_data, pkt->getPtr<uint8_t>(), pkt->getSize());
    write_pkt->dataDynamic(write_data);

    // Perform the atomic on the write packet data. The atomic op is not
    // copied from the original packet, so use the original packet.
    assert(pkt->isAtomicOp());
    (*pkt->getAtomicOp())(write_pkt->getPtr<uint8_t>());

    // Write back the new value. The atomic is not considered done until
    // this packet's response event is triggered.
    systemHub.dmaWrite(write_pkt->getAddr(), write_pkt->getSize(),
        dmaRespEvent, write_pkt->getPtr<uint8_t>(), 0, 0, delay);

    // Atomics from the GPU are at most 64-bit and usually 32-bit.
    // We can take a peek at the data for debugging purposes.
    [[maybe_unused]] uint64_t req_data = 0x12345678;
    if (write_pkt->getSize() == 8) {
        req_data = write_pkt->getLE<uint64_t>();
    } else if (pkt->getSize() == 4) {
        req_data = write_pkt->getLE<uint32_t>();
    }

    DPRINTF(AMDGPUSystemHub, "SystemHub atomic %#lx writing %lx size %d\n",
            write_pkt->getAddr(), req_data, write_pkt->getSize());
}

AddrRangeList
AMDGPUSystemHub::getAddrRanges() const
{
    AddrRangeList ranges;
    return ranges;
}

} // namespace gem5
