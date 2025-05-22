/*
 * Copyright 2023 Google, LLC.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/port_wrapper.hh"

namespace gem5
{

RequestPortWrapper::RequestPortWrapper(const std::string& name, PortID id)
    : RequestPort(name, id)
{
}

void
RequestPortWrapper::recvRangeChange()
{
    if (!recvRangeChangeCb) {
        RequestPort::recvRangeChange();
        return;
    }
    recvRangeChangeCb();
}

bool
RequestPortWrapper::recvTimingResp(PacketPtr packet)
{
    return recvTimingRespCb(packet);
}

void
RequestPortWrapper::recvReqRetry()
{
    recvReqRetryCb();
}

void
RequestPortWrapper::setRangeChangeCallback(RecvReqRetryCallback cb)
{
    recvRangeChangeCb = std::move(cb);
}

void
RequestPortWrapper::setTimingCallbacks(RecvTimingRespCallback resp_cb,
                                       RecvReqRetryCallback retry_cb)
{
    recvTimingRespCb = std::move(resp_cb);
    recvReqRetryCb = std::move(retry_cb);
}

ResponsePortWrapper::ResponsePortWrapper(const std::string& name, PortID id)
    : ResponsePort(name, id)
{
}

AddrRangeList
ResponsePortWrapper::getAddrRanges() const
{
    return getAddrRangesCb();
}

bool
ResponsePortWrapper::recvTimingReq(PacketPtr packet)
{
    return recvTimingReqCb(packet);
}

void
ResponsePortWrapper::recvRespRetry()
{
    recvRespRetryCb();
}

Tick
ResponsePortWrapper::recvAtomic(PacketPtr packet)
{
    return recvAtomicCb(packet);
}

Tick
ResponsePortWrapper::recvAtomicBackdoor(PacketPtr packet,
                                        MemBackdoorPtr& backdoor)
{
    if (!recvAtomicBackdoorCb) {
        return ResponsePort::recvAtomicBackdoor(packet, backdoor);
    }
    return recvAtomicBackdoorCb(packet, backdoor);
}

void
ResponsePortWrapper::recvFunctional(PacketPtr packet)
{
    recvFunctionalCb(packet);
}

void
ResponsePortWrapper::recvMemBackdoorReq(const MemBackdoorReq& req,
                                        MemBackdoorPtr& backdoor)
{
    if (!recvMemBackdoorReqCb) {
        ResponsePort::recvMemBackdoorReq(req, backdoor);
        return;
    }
    recvMemBackdoorReqCb(req, backdoor);
}

void
ResponsePortWrapper::setGetAddrRangesCallback(GetAddrRangesCallback cb)
{
    getAddrRangesCb = std::move(cb);
}

void
ResponsePortWrapper::setTimingCallbacks(RecvTimingReqCallback timing_cb,
                                        RecvRespRetryCallback retry_cb)
{
    recvTimingReqCb = std::move(timing_cb);
    recvRespRetryCb = std::move(retry_cb);
}

void
ResponsePortWrapper::setAtomicCallbacks(RecvAtomicCallback atomic_cb,
                                        RecvAtomicBackdoorCallback backdoor_cb)
{
    recvAtomicCb = std::move(atomic_cb);
    recvAtomicBackdoorCb = std::move(backdoor_cb);
}

void
ResponsePortWrapper::setFunctionalCallbacks(
    RecvFunctionalCallback func_cb, RecvMemBackdoorReqCallback backdoor_cb)
{
    recvFunctionalCb = std::move(func_cb);
    recvMemBackdoorReqCb = std::move(backdoor_cb);
}

}  // namespace gem5
