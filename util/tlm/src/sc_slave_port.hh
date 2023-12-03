/*
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

#ifndef __SC_SLAVE_PORT_HH__
#define __SC_SLAVE_PORT_HH__

#include <systemc>
#include <tlm>

#include "mem/external_slave.hh"
#include "sc_mm.hh"
#include "sc_peq.hh"
#include "sim_control.hh"

namespace Gem5SystemC
{

// forward declaration
class Gem5SlaveTransactor;

/**
 * Test that gem5 is at the same time as SystemC
 */
#define CAUGHT_UP                                                             \
    do {                                                                      \
        assert(gem5::curTick() == sc_core::sc_time_stamp().value());          \
    } while (0)

/**
 * This is a gem5 slave port that translates gem5 packets to TLM transactions.
 *
 * Upon receiving a packet (recvAtomic, recvTiningReq, recvFunctional) the port
 * creates a new TLM payload and initializes it with information from the gem5
 * packet. The original packet is added as an extension to the TLM payload.
 * Then the port issues a TLM transaction in the SystemC world. By storing the
 * original packet as a payload extension, the packet can be restored and send
 * back to the gem5 world upon receiving a response from the SystemC world.
 */
class SCSlavePort : public gem5::ExternalSlave::ExternalPort
{
  public:
    /** One instance of pe and the related callback needed */
    // payloadEvent<SCSlavePort> pe;
    void pec(PayloadEvent<SCSlavePort> *pe, tlm::tlm_generic_payload &trans,
             const tlm::tlm_phase &phase);

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

  protected:
    /** The gem5 Port slave interface */
    gem5::Tick recvAtomic(gem5::PacketPtr packet);
    void recvFunctional(gem5::PacketPtr packet);
    bool recvTimingReq(gem5::PacketPtr packet);
    bool recvTimingSnoopResp(gem5::PacketPtr packet);
    void recvRespRetry();
    void recvFunctionalSnoop(gem5::PacketPtr packet);

    Gem5SlaveTransactor *transactor;

  public:
    /** The TLM initiator interface */
    tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload &trans,
                                       tlm::tlm_phase &phase,
                                       sc_core::sc_time &t);

    SCSlavePort(const std::string &name_, const std::string &systemc_name,
                gem5::ExternalSlave &owner_);

    void bindToTransactor(Gem5SlaveTransactor *transactor);

    friend PayloadEvent<SCSlavePort>;
};

class SCSlavePortHandler : public gem5::ExternalSlave::Handler
{
  private:
    Gem5SimControl &control;

  public:
    SCSlavePortHandler(Gem5SimControl &control) : control(control) {}

    gem5::ExternalSlave::ExternalPort *
    getExternalPort(const std::string &name, gem5::ExternalSlave &owner,
                    const std::string &port_data);
};

} // namespace Gem5SystemC

#endif // __SC_SLAVE_PORT_H__
