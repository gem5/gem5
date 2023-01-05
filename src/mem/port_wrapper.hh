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

/**
 * @file
 * PortWrapper Object Declaration.
 *
 * The RequestPortWrapper and ResponsePortWrapper converts inherit-based
 * RequestPort and ResponsePort into callback-based. This help reducing
 * redundant code and increase code reusability in most cases, allowing
 * composition over inheritance pattern.
 *
 * Example usage:
 *
 * class MySimObject : public SimObject
 * {
 *   public:
 *       ResponsePortWrapper inPort;
 *
 *       MySimObject(...) : inPort("in_port", this)... {
 *         inPort.setGetAddrRangesCallback([this]() {
 *           return getRange();
 *         });
 *
 *         inPort.setAtomicCallbacks([this](PacketPtr packet) {
 *           // process the packet
 *           ...
 *           return Tick();
 *         });
 *       }
 *
 *   private:
 *       AddrRangeList getRange() const {...}
 * };
 */

#ifndef __MEM_PORT_WRAPPER_HH__
#define __MEM_PORT_WRAPPER_HH__

#include <functional>

#include "mem/port.hh"

namespace gem5
{

/**
 * The RequestPortWrapper converts inherit-based RequestPort into
 * callback-based.
 */
class RequestPortWrapper : public RequestPort
{
  public:
    using RecvRangeChangeCallback = std::function<void()>;
    // Timing Protocol
    using RecvTimingRespCallback = std::function<bool(PacketPtr)>;
    using RecvReqRetryCallback = std::function<void()>;

    RequestPortWrapper(const std::string& name, SimObject* _owner,
                       PortID id = InvalidPortID);

    void recvRangeChange() override;

    // TimingRequestProtocol
    bool recvTimingResp(PacketPtr) override;
    void recvReqRetry() override;

    void setRangeChangeCallback(RecvReqRetryCallback);
    void setTimingCallbacks(RecvTimingRespCallback, RecvReqRetryCallback);

  private:
    RecvRangeChangeCallback recvRangeChangeCb = nullptr;
    RecvTimingRespCallback recvTimingRespCb = nullptr;
    RecvReqRetryCallback recvReqRetryCb = nullptr;
};

/**
 * The ResponsePortWrapper converts inherit-based ResponsePort into
 * callback-based.
 */
class ResponsePortWrapper : public ResponsePort
{
  public:
    using GetAddrRangesCallback = std::function<AddrRangeList()>;
    // Timing Protocol
    using RecvTimingReqCallback = std::function<bool(PacketPtr)>;
    // Atomic Protocol
    using RecvAtomicCallback = std::function<Tick(PacketPtr)>;
    using RecvAtomicBackdoorCallback =
        std::function<Tick(PacketPtr, MemBackdoorPtr&)>;

    // Functional Protocol
    using RecvFunctionalCallback = std::function<void(PacketPtr)>;
    using RecvMemBackdoorReqCallback =
        std::function<void(const MemBackdoorReq&, MemBackdoorPtr&)>;

    using RecvRespRetryCallback = std::function<void()>;

    ResponsePortWrapper(const std::string& name, SimObject* _owner,
                        PortID id = InvalidPortID);

    AddrRangeList getAddrRanges() const override;

    // TimingResponseProtocol
    bool recvTimingReq(PacketPtr) override;
    void recvRespRetry() override;

    // AtomicResponseProtocol
    Tick recvAtomic(PacketPtr) override;
    Tick recvAtomicBackdoor(PacketPtr, MemBackdoorPtr&) override;

    // FunctionalResponseProtocol
    void recvFunctional(PacketPtr) override;
    void recvMemBackdoorReq(const MemBackdoorReq&, MemBackdoorPtr&) override;

    void setGetAddrRangesCallback(GetAddrRangesCallback);
    void setTimingCallbacks(RecvTimingReqCallback, RecvRespRetryCallback);
    void setAtomicCallbacks(RecvAtomicCallback,
                            RecvAtomicBackdoorCallback = nullptr);
    void setFunctionalCallbacks(RecvFunctionalCallback,
                                RecvMemBackdoorReqCallback = nullptr);

  private:
    GetAddrRangesCallback getAddrRangesCb = nullptr;
    RecvTimingReqCallback recvTimingReqCb = nullptr;
    RecvRespRetryCallback recvRespRetryCb = nullptr;
    RecvAtomicCallback recvAtomicCb = nullptr;
    RecvAtomicBackdoorCallback recvAtomicBackdoorCb = nullptr;
    RecvFunctionalCallback recvFunctionalCb = nullptr;
    RecvMemBackdoorReqCallback recvMemBackdoorReqCb = nullptr;
};

}  // namespace gem5

#endif  //__MEM_PORT_WRAPPER_HH__
