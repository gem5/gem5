/*
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

#ifndef __SC_MASTER_PORT_HH__
#define __SC_MASTER_PORT_HH__

#include <tlm_utils/peq_with_cb_and_phase.h>

#include <systemc>
#include <tlm>

#include "mem/external_master.hh"
#include "sc_peq.hh"
#include "sim_control.hh"

namespace Gem5SystemC
{

// forward declaration
class Gem5MasterTransactor;

/**
 * This is a gem5 master port that translates TLM transactions to gem5 packets.
 *
 * Upon receiving a TLM transaction (b_transport, nb_transport_fw,
 * dbg_transport) the port generates a gem5 packet and initializes the packet
 * with information from the transaction payload. The original TLM payload is
 * added as a sender state to the gem5 packet. This way the payload can be
 * restored when the response packet arrives at the port.
 *
 * Special care is required, when the TLM transaction originates from a
 * SCSlavePort (i.e. it is a gem5 packet that enters back into the gem5 world).
 * This is a common scenario, when multiple gem5 CPUs communicate via a SystemC
 * interconnect. In this case, the master port restores the original packet
 * from the payload extension (added by the SCSlavePort) and forwards it to the
 * gem5 world. Throughout the code, this mechanism is called 'pipe through'.
 *
 * If gem5 operates in atomic mode, the master port registers the TLM blocking
 * interface and automatically translates non-blocking requests to blocking.
 * If gem5 operates in timing mode, the transactor registers the non-blocking
 * interface. Then, the transactor automatically translated blocking requests.
 * It is assumed that the mode (atomic/timing) does not change during
 * execution.
 */
class SCMasterPort : public ExternalMaster::ExternalPort
{
  private:
    struct TlmSenderState : public Packet::SenderState
    {
        tlm::tlm_generic_payload& trans;
        TlmSenderState(tlm::tlm_generic_payload& trans)
          : trans(trans)
        {
        }
    };

    tlm_utils::peq_with_cb_and_phase<SCMasterPort> peq;

    bool waitForRetry;
    tlm::tlm_generic_payload* pendingRequest;
    PacketPtr pendingPacket;

    bool needToSendRetry;

    bool responseInProgress;

    Gem5MasterTransactor* transactor;

    System* system;

    Gem5SimControl& simControl;

  protected:
    // payload event call back
    void peq_cb(tlm::tlm_generic_payload& trans, const tlm::tlm_phase& phase);

    // The TLM target interface
    tlm::tlm_sync_enum nb_transport_fw(tlm::tlm_generic_payload& trans,
                                       tlm::tlm_phase& phase,
                                       sc_core::sc_time& t);
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& t);
    unsigned int transport_dbg(tlm::tlm_generic_payload& trans);
    bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans,
                            tlm::tlm_dmi& dmi_data);

    // Gem5 SCMasterPort interface
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();
    void recvRangeChange();

  public:
    SCMasterPort(const std::string& name_,
                 const std::string& systemc_name,
                 ExternalMaster& owner_,
                 Gem5SimControl& simControl);

    void bindToTransactor(Gem5MasterTransactor* transactor);

    friend PayloadEvent<SCMasterPort>;

  private:
    void sendEndReq(tlm::tlm_generic_payload& trans);
    void sendBeginResp(tlm::tlm_generic_payload& trans,
                       sc_core::sc_time& delay);

    void handleBeginReq(tlm::tlm_generic_payload& trans);
    void handleEndResp(tlm::tlm_generic_payload& trans);

    PacketPtr generatePacket(tlm::tlm_generic_payload& trans);
    void destroyPacket(PacketPtr pkt);

    void checkTransaction(tlm::tlm_generic_payload& trans);
};

class SCMasterPortHandler : public ExternalMaster::Handler
{
  private:
    Gem5SimControl& control;

  public:
    SCMasterPortHandler(Gem5SimControl& control) : control(control) {}

    ExternalMaster::ExternalPort *
        getExternalPort(const std::string &name, ExternalMaster &owner,
                        const std::string &port_data);
};

}

#endif
