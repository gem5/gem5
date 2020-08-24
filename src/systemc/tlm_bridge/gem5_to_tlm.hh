/*
 * Copyright 2019 Google, Inc.
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
 *
 * Copyright (c) 2015, University of Kaiserslautern
 * Copyright (c) 2016, Dresden University of Technology (TU Dresden)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SYSTEMC_TLM_BRIDGE_GEM5_TO_TLM_HH__
#define __SYSTEMC_TLM_BRIDGE_GEM5_TO_TLM_HH__

#include <string>

#include "mem/port.hh"
#include "params/Gem5ToTlmBridgeBase.hh"
#include "sim/system.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_module_name.hh"
#include "systemc/ext/tlm_core/2/generic_payload/gp.hh"
#include "systemc/ext/tlm_utils/simple_initiator_socket.h"
#include "systemc/tlm_port_wrapper.hh"

namespace sc_gem5
{

tlm::tlm_generic_payload *packet2payload(PacketPtr packet);

class Gem5ToTlmBridgeBase : public sc_core::sc_module
{
  protected:
    using sc_core::sc_module::sc_module;
};

template <unsigned int BITWIDTH>
class Gem5ToTlmBridge : public Gem5ToTlmBridgeBase
{
  private:
    class BridgeResponsePort : public ResponsePort
    {
      protected:
        Gem5ToTlmBridge<BITWIDTH> &bridge;

        AddrRangeList
        getAddrRanges() const override
        {
            return bridge.getAddrRanges();
        }
        Tick
        recvAtomic(PacketPtr pkt) override
        {
            return bridge.recvAtomic(pkt);
        }
        Tick
        recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor) override
        {
            return bridge.recvAtomicBackdoor(pkt, backdoor);
        }
        void
        recvFunctional(PacketPtr pkt) override
        {
            return bridge.recvFunctional(pkt);
        }
        bool
        recvTimingReq(PacketPtr pkt) override
        {
            return bridge.recvTimingReq(pkt);
        }
        bool
        tryTiming(PacketPtr pkt) override
        {
            return bridge.tryTiming(pkt);
        }
        bool
        recvTimingSnoopResp(PacketPtr pkt) override
        {
            return bridge.recvTimingSnoopResp(pkt);
        }
        void recvRespRetry() override { bridge.recvRespRetry(); }

      public:
        BridgeResponsePort(const std::string &name_,
                        Gem5ToTlmBridge<BITWIDTH> &bridge_) :
            ResponsePort(name_, nullptr), bridge(bridge_)
        {}
    };

    BridgeResponsePort bridgeResponsePort;
    tlm_utils::simple_initiator_socket<
        Gem5ToTlmBridge<BITWIDTH>, BITWIDTH> socket;
    sc_gem5::TlmInitiatorWrapper<BITWIDTH> wrapper;

    System *system;

    /**
     * A transaction after BEGIN_REQ has been sent but before END_REQ, which
     * is blocking the request channel (Exlusion Rule, see IEEE1666)
     */
    tlm::tlm_generic_payload *blockingRequest;

    /**
     * Did another gem5 request arrive while currently blocked?
     * This variable is needed when a retry should happen
     */
    bool needToSendRequestRetry;

    /**
     * A response which has been asked to retry by gem5 and so is blocking
     * the response channel
     */
    tlm::tlm_generic_payload *blockingResponse;

    AddrRangeList addrRanges;

  protected:
    void pec(tlm::tlm_generic_payload &trans, const tlm::tlm_phase &phase);

    MemBackdoorPtr getBackdoor(tlm::tlm_generic_payload &trans);
    AddrRangeMap<MemBackdoorPtr> backdoorMap;

    // The gem5 port interface.
    Tick recvAtomic(PacketPtr packet);
    Tick recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor);
    void recvFunctional(PacketPtr packet);
    bool recvTimingReq(PacketPtr packet);
    bool tryTiming(PacketPtr packet);
    bool recvTimingSnoopResp(PacketPtr packet);
    void recvRespRetry();
    void recvFunctionalSnoop(PacketPtr packet);
    AddrRangeList getAddrRanges() const { return addrRanges; }

    // The TLM initiator interface.
    tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload &trans,
                                       tlm::tlm_phase &phase,
                                       sc_core::sc_time &t);
    void invalidate_direct_mem_ptr(
            sc_dt::uint64 start_range, sc_dt::uint64 end_range);

  public:
    ::Port &gem5_getPort(const std::string &if_name, int idx=-1) override;

    typedef Gem5ToTlmBridgeBaseParams Params;
    Gem5ToTlmBridge(Params *p, const sc_core::sc_module_name &mn);

    tlm_utils::simple_initiator_socket<Gem5ToTlmBridge<BITWIDTH>, BITWIDTH> &
    getSocket()
    {
        return socket;
    }

    void before_end_of_elaboration() override;
};

} // namespace sc_gem5

#endif // __SYSTEMC_TLM_BRIDGE_GEM5_TO_TLM_HH__
