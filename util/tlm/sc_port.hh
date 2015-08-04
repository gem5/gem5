/*
 * Copyright (c) 2015, University of Kaiserslautern
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
 * Authors: Matthias Jung
 */

#ifndef __SIM_SC_TRANSACTOR_HH__
#define __SIM_SC_TRANSACTOR_HH__

#include <tlm_utils/simple_initiator_socket.h>

#include <map>
#include <systemc>
#include <tlm>

#include "mem/external_slave.hh"
#include "sc_mm.hh"
#include "sc_module.hh"

namespace Gem5SystemC
{
/**
 * Test that gem5 is at the same time as SystemC
 */
#define CAUGHT_UP do { \
    assert(curTick() == sc_core::sc_time_stamp().value()); \
} while (0)


class sc_transactor : public tlm::tlm_initiator_socket<>,
        public tlm::tlm_bw_transport_if<>,
        public ExternalSlave::Port
{
  public:
    sc_transactor &iSocket;

    /**
     * A 'Fake Payload Event Queue', similar to the TLM PEQs. This will help
     * that gem5 behaves like a normal TLM Initiator
     */
    template<typename OWNER>
    class payloadEvent : public Event
    {
        public:
        OWNER &port;
        const std::string eventName;
        void (OWNER::* handler)(payloadEvent<OWNER> * pe,
            tlm::tlm_generic_payload& trans,
            const tlm::tlm_phase &phase);

        protected:
        tlm::tlm_generic_payload *t;
        tlm::tlm_phase p;

        void process() { (port.*handler)(this,*t, p); }

        public:
        const std::string name() const { return eventName; }

        payloadEvent(
            OWNER &port_,
            void (OWNER::* handler_)(payloadEvent<OWNER> * pe,
                tlm::tlm_generic_payload& trans,
                const tlm::tlm_phase &phase),
            const std::string &event_name) :
            port(port_),
            eventName(event_name),
            handler(handler_)
        { }

        /// Schedule an event into gem5
        void
        notify(tlm::tlm_generic_payload& trans,
           const tlm::tlm_phase &phase,
           const sc_core::sc_time& delay)
        {
            assert(!scheduled());

            t = &trans;
            p = phase;

            /**
             * Get time from SystemC as this will alway be more up to date
             * than gem5's
             */
            Tick nextEventTick = sc_core::sc_time_stamp().value()
                + delay.value();

            port.owner.wakeupEventQueue(nextEventTick);
            port.owner.schedule(this, nextEventTick);
       }
    };

    /** One instance of pe and the related callback needed */
    //payloadEvent<sc_transactor> pe;
    void pec(payloadEvent<sc_transactor> * pe,
        tlm::tlm_generic_payload& trans, const tlm::tlm_phase& phase);

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
    Tick recvAtomic(PacketPtr packet);
    void recvFunctional(PacketPtr packet);
    bool recvTimingReq(PacketPtr packet);
    bool recvTimingSnoopResp(PacketPtr packet);
    void recvRespRetry();
    void recvFunctionalSnoop(PacketPtr packet);

    /** The TLM initiator interface */
    tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload& trans,
                                       tlm::tlm_phase& phase,
                                       sc_core::sc_time& t);

    void invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                   sc_dt::uint64 end_range);

  public:
    sc_transactor(const std::string &name_,
           const std::string &systemc_name,
           ExternalSlave &owner_);
};

void registerPort(const std::string &name, Port &port);

void registerSCPorts();

}

#endif // __SIM_SC_PORT_HH__
