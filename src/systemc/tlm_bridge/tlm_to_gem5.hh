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
 *
 * Authors: Gabe Black
 *          Christian Menard
 */

#ifndef __SYSTEMC_TLM_BRIDGE_TLM_TO_GEM5_HH__
#define __SYSTEMC_TLM_BRIDGE_TLM_TO_GEM5_HH__

#include "mem/port.hh"
#include "params/TlmToGem5BridgeBase.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_module_name.hh"
#include "systemc/ext/tlm_core/2/generic_payload/gp.hh"
#include "systemc/ext/tlm_utils/peq_with_cb_and_phase.h"
#include "systemc/ext/tlm_utils/simple_target_socket.h"
#include "systemc/tlm_bridge/sc_ext.hh"
#include "systemc/tlm_port_wrapper.hh"

namespace sc_gem5
{

class TlmToGem5BridgeBase : public sc_core::sc_module
{
  protected:
    using sc_core::sc_module::sc_module;
};

template <unsigned int BITWIDTH>
class TlmToGem5Bridge : public TlmToGem5BridgeBase
{
  private:
    struct TlmSenderState : public Packet::SenderState
    {
        tlm::tlm_generic_payload &trans;
        TlmSenderState(tlm::tlm_generic_payload &trans) : trans(trans) {}
    };

    class BridgeMasterPort : public MasterPort
    {
      protected:
        TlmToGem5Bridge<BITWIDTH> &bridge;

        bool
        recvTimingResp(PacketPtr pkt) override
        {
            return bridge.recvTimingResp(pkt);
        }
        void recvReqRetry() override { bridge.recvReqRetry(); }
        void recvRangeChange() override { bridge.recvRangeChange(); }

      public:
        BridgeMasterPort(const std::string &name_,
                         TlmToGem5Bridge<BITWIDTH> &bridge_) :
            MasterPort(name_, nullptr), bridge(bridge_)
        {}
    };

    tlm_utils::peq_with_cb_and_phase<TlmToGem5Bridge<BITWIDTH>> peq;

    bool waitForRetry;
    tlm::tlm_generic_payload *pendingRequest;
    PacketPtr pendingPacket;

    bool needToSendRetry;

    bool responseInProgress;

    BridgeMasterPort bmp;
    tlm_utils::simple_target_socket<
        TlmToGem5Bridge<BITWIDTH>, BITWIDTH> socket;
    sc_gem5::TlmTargetWrapper<BITWIDTH> wrapper;

    System *system;

    void sendEndReq(tlm::tlm_generic_payload &trans);
    void sendBeginResp(tlm::tlm_generic_payload &trans,
                       sc_core::sc_time &delay);

    void handleBeginReq(tlm::tlm_generic_payload &trans);
    void handleEndResp(tlm::tlm_generic_payload &trans);

    PacketPtr generatePacket(tlm::tlm_generic_payload &trans);
    void destroyPacket(PacketPtr pkt);

    void checkTransaction(tlm::tlm_generic_payload &trans);

    void invalidateDmi(const ::MemBackdoor &backdoor);

  protected:
    // payload event call back
    void peq_cb(tlm::tlm_generic_payload &trans, const tlm::tlm_phase &phase);

    // The TLM target interface
    tlm::tlm_sync_enum nb_transport_fw(tlm::tlm_generic_payload &trans,
                                       tlm::tlm_phase &phase,
                                       sc_core::sc_time &t);
    void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &t);
    unsigned int transport_dbg(tlm::tlm_generic_payload &trans);
    bool get_direct_mem_ptr(tlm::tlm_generic_payload &trans,
                            tlm::tlm_dmi &dmi_data);

    // Gem5 port interface.
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();
    void recvRangeChange();

  public:
    ::Port &gem5_getPort(const std::string &if_name, int idx=-1) override;

    typedef TlmToGem5BridgeBaseParams Params;
    TlmToGem5Bridge(Params *p, const sc_core::sc_module_name &mn);

    tlm_utils::simple_target_socket<TlmToGem5Bridge<BITWIDTH>, BITWIDTH> &
    getSocket()
    {
        return socket;
    }

    void before_end_of_elaboration() override;

    const MasterID masterId;
};

} // namespace sc_gem5

#endif // __SYSTEMC_TLM_BRIDGE_TLM_TO_GEM5_HH__
