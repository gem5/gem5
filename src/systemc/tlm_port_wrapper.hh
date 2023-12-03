/*
 * Copyright 2018 Google, Inc.
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

#ifndef __SYSTEMC_TLM_PORT_WRAPPER_HH__
#define __SYSTEMC_TLM_PORT_WRAPPER_HH__

#include "base/logging.hh"
#include "sim/port.hh"
#include "systemc/ext/tlm_core/2/sockets/sockets.hh"

namespace sc_gem5
{

template <unsigned int BUSWIDTH, typename FW_IF, typename BW_IF, int N,
          sc_core::sc_port_policy POL>
class TlmInitiatorBaseWrapper;

template <unsigned int BUSWIDTH, typename FW_IF, typename BW_IF, int N,
          sc_core::sc_port_policy POL>
class TlmTargetBaseWrapper;

template <unsigned int BUSWIDTH, typename FW_IF, typename BW_IF, int N,
          sc_core::sc_port_policy POL>
class TlmInitiatorBaseWrapper : public gem5::Port
{
  public:
    typedef tlm::tlm_base_initiator_socket<BUSWIDTH, FW_IF, BW_IF, N, POL>
        InitiatorSocket;
    typedef typename InitiatorSocket::base_target_socket_type TargetSocket;
    typedef TlmTargetBaseWrapper<BUSWIDTH, FW_IF, BW_IF, N, POL> TargetWrapper;

    InitiatorSocket &
    initiator()
    {
        return _initiator;
    }

    TlmInitiatorBaseWrapper(InitiatorSocket &i, const std::string &_name,
                            gem5::PortID _id)
        : gem5::Port(_name, _id), _initiator(i)
    {}

    void
    bind(gem5::Port &peer) override
    {
        using namespace gem5;

        auto *target = dynamic_cast<TargetWrapper *>(&peer);
        fatal_if(!target,
                 "Attempt to bind TLM initiator socket %s to "
                 "incompatible port %s.",
                 name(), peer.name());

        initiator().bind(target->target());
        gem5::Port::bind(peer);
    }

    void
    unbind() override
    {
        using namespace gem5;

        panic("TLM sockets can't be unbound.");
    }

  private:
    InitiatorSocket &_initiator;
};

template <unsigned int BUSWIDTH, typename FW_IF, typename BW_IF, int N,
          sc_core::sc_port_policy POL>
class TlmTargetBaseWrapper : public gem5::Port
{
  public:
    typedef tlm::tlm_base_target_socket<BUSWIDTH, FW_IF, BW_IF, N, POL>
        TargetSocket;

    TargetSocket &
    target()
    {
        return _target;
    }

    TlmTargetBaseWrapper(TargetSocket &t, const std::string &_name,
                         gem5::PortID _id)
        : gem5::Port(_name, _id), _target(t)
    {}

    void
    bind(gem5::Port &peer) override
    {
        // Ignore attempts to bind a target socket. The initiator will
        // handle it.
        gem5::Port::bind(peer);
    }

    void
    unbind() override
    {
        using namespace gem5;

        panic("TLM sockets can't be unbound.");
    }

  private:
    TargetSocket &_target;
};

template <unsigned int BUSWIDTH = 32,
          typename TYPES = tlm::tlm_base_protocol_types, int N = 1,
          sc_core::sc_port_policy POL = sc_core::SC_ONE_OR_MORE_BOUND>
using TlmInitiatorWrapper =
    TlmInitiatorBaseWrapper<BUSWIDTH, tlm::tlm_fw_transport_if<TYPES>,
                            tlm::tlm_bw_transport_if<TYPES>, N, POL>;

template <unsigned int BUSWIDTH = 32,
          typename TYPES = tlm::tlm_base_protocol_types, int N = 1,
          sc_core::sc_port_policy POL = sc_core::SC_ONE_OR_MORE_BOUND>
using TlmTargetWrapper =
    TlmTargetBaseWrapper<BUSWIDTH, tlm::tlm_fw_transport_if<TYPES>,
                         tlm::tlm_bw_transport_if<TYPES>, N, POL>;

} // namespace sc_gem5

#endif //__SYSTEMC_TLM_PORT_WRAPPER_HH__
